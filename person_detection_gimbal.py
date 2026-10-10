#####################################################################################################
# @file         person_detection_gimbal.py
# @brief        人体检测 + 坐标回传（配合 STM32 云台跟踪）
#
# 在正点原子官方例程 person_detection.py 基础上加了：
#   1. 挑画面里最大的那个人体框，取它中心的像素坐标 (x, y)
#   2. 通过 UART 按 13 字节定长帧发给 STM32，STM32 用它驱动二自由度云台
#   3. 在屏幕左上角显示帧率
#
# 实验平台: 正点原子 K230D BOX + 二自由度舵机云台 + STM32F103
#
# 接线（K230D BOX -> STM32）：
#   K230 UART2_TX (GPIO44) --> STM32 PA3 (USART2_RX)
#   K230 UART2_RX (GPIO45) <-- STM32 PA2 (USART2_TX)  （只发不收回传时可以不接）
#   GND                    --- STM32 GND               （必须共地！）
#
#   正点原子 K230D BOX 的 UART2 引在 PH2.0 接口2 上，对应 GPIO44/GPIO45，
#   和官方 UART 例程一致。若你的接线不同，只改下面的 UART_TX_GPIO / UART_RX_GPIO。
#
# 通信协议（13 字节定长帧，大端序，和 STM32 端 modules/k230_to_stm32.c 严格对应）：
#   字节 0-3    帧头       0xAA 0x55 0x01 0x08
#   字节 4-5    x          int16，人体框中心像素 X，-1 表示这一帧没检测到人
#   字节 6-7    y          int16，人体框中心像素 Y，-1 表示这一帧没检测到人
#   字节 8-9    reserved   uint16，保留，当前固定 0
#   字节 10-11  reserved   uint16，保留，当前固定 0
#   字节 12     checksum   前 12 字节逐字节异或
#
#   坐标用的是检测画面（rgb888p_size，默认 1280x960）的像素坐标，原点在左上角。
#   STM32 侧拿 x 减掉画面中心就是云台要消掉的水平偏差，y 同理对应俯仰。
#####################################################################################################

from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
import os
import ujson
from media.media import *
from media.sensor import *
from time import *
import nncase_runtime as nn
import ulab.numpy as np
import time
import utime
import image
import random
import gc
import sys
import struct
import aicube
import machine
from machine import Pin
from machine import FPIOA
from machine import UART


# =====================================================================================
#  串口发送部分
# =====================================================================================

class GimbalLink:
    """把人体中心坐标打包成 13 字节定长帧发给 STM32"""

    HEADER = (0xAA, 0x55, 0x01, 0x08)
    LOST_PX = -1

    def __init__(self, uart_id=2, tx_pin=44, rx_pin=45, baudrate=115200):
        # 正点原子 K230D BOX 的 UART2 在 PH2.0 接口2 上，对应 GPIO44/GPIO45；
        # 如果提示引脚不可用，用 help(fpioa.set_function) 查一下，
        # 部分固件上的常量名是 fpioa.UART2_TXD / fpioa.UART2_RXD。
        fpioa = FPIOA()
        fpioa.set_function(tx_pin, FPIOA.UART2_TXD)
        fpioa.set_function(rx_pin, FPIOA.UART2_RXD)

        self.uart = UART(UART.UART2, baudrate,
                         bits=UART.EIGHTBITS,
                         parity=UART.PARITY_NONE,
                         stop=UART.STOPBITS_ONE)
        self.sent = 0

    @staticmethod
    def _clamp_int16(value):
        if value > 32767:
            return 32767
        if value < -32768:
            return -32768
        return int(value)

    def send(self, x, y, detected):
        """x, y: 人体框中心的像素坐标（画面坐标，整数）。
        detected 为 0 时按协议补发 -1，STM32 收到就知道目标丢了。"""
        if detected:
            x = self._clamp_int16(x)
            y = self._clamp_int16(y)
        else:
            x = self.LOST_PX
            y = self.LOST_PX

        frame = bytearray(self.HEADER)
        # 大端序打包 x / y 和两个保留字段，和 STM32 侧解析严格对应
        frame += struct.pack(">hhHH", x, y, 0, 0)

        checksum = 0
        for value in frame:
            checksum ^= value
        frame.append(checksum & 0xFF)

        self.uart.write(frame)
        self.sent += 1


# =====================================================================================
#  按钮（沿用官方例程）
# =====================================================================================

class Button():
    def __init__(self, fpioa, pinx, valid=0):
        fpioa.set_function(pinx, fpioa.GPIO0 + pinx)
        pull = Pin.PULL_UP if valid == 0 else Pin.PULL_DOWN
        self.pin = Pin(pinx, Pin.IN, pull=pull, drive=7)
        self.valid = valid

    def is_pressing(self):
        return True if self.pin.value() == self.valid else False


# =====================================================================================
#  人体检测（沿用官方例程，只在 draw_result 里返回最大的那个人）
# =====================================================================================

class PersonDetectionApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, labels, anchors,
                 confidence_threshold=0.2, nms_threshold=0.5, nms_option=False,
                 strides=[8, 16, 32], rgb888p_size=[224, 224],
                 display_size=[1920, 1080], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.labels = labels
        self.anchors = anchors
        self.strides = strides
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.nms_option = nms_option
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT,
                                 np.uint8, np.uint8)

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right = self.get_padding_param()
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [0, 0, 0])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                            [1, 3, self.model_input_size[1], self.model_input_size[0]])

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            dets = aicube.anchorbasedet_post_process(
                results[0], results[1], results[2],
                self.model_input_size, self.rgb888p_size, self.strides,
                len(self.labels), self.confidence_threshold, self.nms_threshold,
                self.anchors, self.nms_option)
            return dets

    def pick_target(self, dets):
        """从检测结果里挑一个人当跟踪目标：选面积最大的那个。
        返回 (x1, y1, x2, y2, score)，没有合适的目标返回 None。
        坐标是 rgb888p 原始像素坐标。"""
        best = None
        best_area = 0

        if not dets:
            return None

        W = self.rgb888p_size[0]
        H = self.rgb888p_size[1]

        for det in dets:
            x1, y1, x2, y2 = det[2], det[3], det[4], det[5]
            score = det[1]
            w = x2 - x1
            h = y2 - y1
            if w <= 0 or h <= 0:
                continue

            # 太小的框（远处/误检）不要
            if h < 0.12 * H and w < 0.12 * W:
                continue

            area = w * h
            if area > best_area:
                best_area = area
                best = (x1, y1, x2, y2, score)

        return best

    def draw_result(self, pl, dets, target=None):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            pl.osd_img.clear()
            if target is None:
                return

            x1, y1, x2, y2, score = target
            W = self.rgb888p_size[0]
            H = self.rgb888p_size[1]
            dw = self.display_size[0]
            dh = self.display_size[1]

            dx1 = int(x1 * dw // W)
            dy1 = int(y1 * dh // H)
            dx2 = int(x2 * dw // W)
            dy2 = int(y2 * dh // H)

            pl.osd_img.draw_rectangle(dx1, dy1, dx2 - dx1, dy2 - dy1,
                                      color=(255, 0, 255, 0), thickness=2)
            pl.osd_img.draw_string_advanced(dx1, max(0, dy1 - 40), 32,
                                            "person %.2f" % score,
                                            color=(255, 0, 255, 0))

            # 画面中心画个十字，方便肉眼对照
            cx = dw // 2
            cy = dh // 2
            pl.osd_img.draw_line(cx - 30, cy, cx + 30, cy, color=(255, 255, 0, 0), thickness=2)
            pl.osd_img.draw_line(cx, cy - 30, cx, cy + 30, color=(255, 255, 0, 0), thickness=2)

    def get_padding_param(self):
        dst_w = self.model_input_size[0]
        dst_h = self.model_input_size[1]
        input_width = self.rgb888p_size[0]
        input_high = self.rgb888p_size[1]
        ratio_w = dst_w / input_width
        ratio_h = dst_h / input_high
        ratio = ratio_w if ratio_w < ratio_h else ratio_h
        new_w = (int)(ratio * input_width)
        new_h = (int)(ratio * input_high)
        dw = (dst_w - new_w) / 2
        dh = (dst_h - new_h) / 2
        top = int(round(dh - 0.1))
        bottom = int(round(dh + 0.1))
        left = int(round(dw - 0.1))
        right = int(round(dw - 0.1))
        return top, bottom, left, right


# =====================================================================================
#  主程序
# =====================================================================================

if __name__ == "__main__":
    fpioa = FPIOA()
    button0 = Button(fpioa, 34, 0)

    display_mode = "lcd"
    display_size = [640, 480]
    kmodel_path = "/sdcard/examples/kmodel/person_detect_yolov5n.kmodel"

    confidence_threshold = 0.2
    nms_threshold = 0.6
    rgb888p_size = [1280, 960]
    labels = ["person"]
    anchors = [10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326]

    # ---- 串口：发给 STM32（正点原子 K230D 的 UART2 在 GPIO44/GPIO45）----
    link = GimbalLink(uart_id=2, tx_pin=44, rx_pin=45, baudrate=115200)

    # ---- 坐标平滑参数 ----
    EMA_ALPHA     = 0.35    # 越大跟得越快、越小越平滑
    SEND_INTERVAL = 30      # 最短发送间隔(ms)，约 33fps
    LOST_INTERVAL = 200     # 没检测到人时，多久补发一次"丢失"帧(ms)

    sensor = Sensor(width=1280, height=960)
    pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size,
                  display_mode=display_mode)
    pl.create(sensor=sensor)

    person_det = PersonDetectionApp(
        kmodel_path, model_input_size=[640, 640], labels=labels, anchors=anchors,
        confidence_threshold=confidence_threshold, nms_threshold=nms_threshold,
        nms_option=False, strides=[8, 16, 32], rgb888p_size=rgb888p_size,
        display_size=display_size, debug_mode=0)
    person_det.config_preprocess()

    ema_x = 0.0
    ema_y = 0.0
    last_send = 0
    last_lost_send = 0

    # ---- 帧率统计 ----
    fps_count = 0
    fps_start = time.ticks_ms()
    display_fps = 0.0

    try:
        while True:
            os.exitpoint()
            with ScopedTiming("total", 1):
                img = pl.get_frame()
                res = person_det.run(img)

                target = person_det.pick_target(res)
                now = time.ticks_ms()

                if target is not None:
                    x1, y1, x2, y2, score = target
                    # 人体框中心 -> 像素坐标（和 STM32 侧约定的一致）
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0

                    # 一阶低通：检测框抖动很厉害，不平滑的话云台会一直抖
                    ema_x = ema_x + (cx - ema_x) * EMA_ALPHA
                    ema_y = ema_y + (cy - ema_y) * EMA_ALPHA

                    if time.ticks_diff(now, last_send) >= SEND_INTERVAL:
                        last_send = now
                        link.send(int(ema_x), int(ema_y), 1)
                else:
                    # 没检测到人：定期补发"丢失"帧，让 STM32 及时停下
                    if time.ticks_diff(now, last_lost_send) >= LOST_INTERVAL:
                        last_lost_send = now
                        link.send(0, 0, 0)

                # ---- 帧率统计 ----
                fps_count += 1
                elapsed = time.ticks_diff(now, fps_start)
                if elapsed >= 1000:
                    display_fps = fps_count * 1000.0 / elapsed
                    fps_count = 0
                    fps_start = now

                person_det.draw_result(pl, res, target)
                # 左上角帧率。放在 draw_result 之后画，才不会被它 clear 掉；
                # 没检测到人时 draw_result 会提前返回，这里依然会显示帧率
                pl.osd_img.draw_string_advanced(
                    8, 8, 32, "FPS: %.1f" % display_fps,
                    color=(255, 255, 255, 0))
                pl.show_image()
                gc.collect()

            if button0.is_pressing():
                try:
                    with open("/sdcard/main.py", "rb") as f:
                        os.remove("/sdcard/main.py")
                except Exception as e:
                    pass
                with open("/sdcard/CanMV Sample/main.py", "rb") as f:
                    code_src = f.read()
                with open("/sdcard/main.py", "wb") as f:
                    f.write(code_src)
                machine.reset()

    except Exception as e:
        sys.print_exception(e)
    finally:
        person_det.deinit()
        pl.destroy()
