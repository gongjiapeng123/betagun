#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
连接小车61614端口，提供图像数据给tcp服务器，并发布图像主题
'''

from __future__ import print_function
from __future__ import unicode_literals
from __future__ import division
from __future__ import absolute_import

import cv2
import numpy as np
import logging
import logging.config
from log_config import LOGGING
import argparse
import textwrap
import socket

HEAD = b'\x66\xAA'
END = b'\xFC'

class Base_:
    '''
    一个用opencv库获取双目摄像头图像并显示的类
    size: 设置摄像头图像的宽高
    '''
    def __init__(self, size=(640, 480)):
        self.capture1 = None  # 要显示时才初始化为cv2.VideoCapture
        self.capture0 = None

        self.width = size[0]
        self.height = size[1]

        self.fps = 30

    def open_cam(self, dev_id):
        '''
        打开摄像头
        :param dev_id: 摄像头id，大小设为self.size的大小
        :return:
        '''
        if dev_id == 1:
            self.capture1 = cv2.VideoCapture(1)
            self.capture1.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, self.width)
            self.capture1.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, self.height)
            return self.capture1
        elif dev_id == 0:
            self.capture0 = cv2.VideoCapture(0)
            self.capture0.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, self.width)
            self.capture0.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, self.height)
            return self.capture0

    def close_cam(self, dev_id):
        '''
        关闭摄像头
        :param dev_id: 摄像头id，大小设为self.size的大小
        :return:
        '''
        if dev_id == 1:
            self.capture1 and self.capture1.release()
        elif dev_id == 0:
            self.capture0 and self.capture0.release()

    def next_frame(self):
        '''
        获取下一帧
        :return: (rs1, rs0, frame1, frame0)
        '''
        rs1, frame1 = self.capture1.read()
        rs0, frame0 = self.capture0.read()

        return rs1, rs0, frame1, frame0

    def easy_show(self):
        '''
        开始简单地获取并显示摄像头图片，esc退出循环
        '''
        wait_time = 1000 // self.fps
        self.open_cam(1)
        self.open_cam(0)
        if self.capture1.isOpened() & self.capture0.isOpened():
            rs1, rs0, frame1, frame0 = self.next_frame()
            while rs1 & rs0:

                cv2.imshow('show1', frame1)
                cv2.imshow('show0', frame0)

                rs1, frame1 = self.capture1.read()
                rs0, frame0 = self.capture0.read()

                if cv2.waitKey(wait_time) == 27:
                    self.clean()
                    break
            else:
                self.clean()

    def clean(self):
        '''
        退出前的清理工作
        '''
        self.capture1 is not None and self.capture1.release()
        self.capture0 is not None and self.capture0.release()
        self.capture1 = self.capture0 = None
        cv2.destroyAllWindows()


class Capture(Base_):
    def __init__(self, size=(640, 480), send_size=(320, 120), need_to_show=False, need_to_connect_server=True):
        Base_.__init__(self, size)

        self.send_size = send_size  # 发送给服务器时的图片大小（两张拼接）
        self.need_to_show = need_to_show  # 是否在一个窗口显示图像：0不显示，1在一个窗口显示双目图像，2在两个窗口分别显示图像
        self.need_to_connect_server = need_to_connect_server  # 是否需要与服务器通信

        # logger
        logging.config.dictConfig(LOGGING)
        self.logger = logging.getLogger('betagun')

        self.exit = False  # 是否停止获取图像

        self.sock_image = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 61614图像端口

        self.has_sock_error = False  # 指明当前是否IPC通信有问题，有问题就不要重复发送数据了
        self.send_image_count = 0

        # 连接服务器端口
        if self.need_to_connect_server:
            self.connect_server()

    def connect_server(self):
        '''
        连接服务器，使得可以将图片传递给服务器
        :return:
        '''
        try:
            self.sock_image.connect(('127.0.0.1', 61614))

            # 告诉服务器本进程是python进程
            self.sock_image.send(b'#python:gxnu#')

            self.logger.info("I connect successfully")
        except socket.error as e:
            self.has_sock_error = True
            self.logger.error(e)

    def montage(self, frame1, frame0):
        '''
        双目摄像头时，拼接两帧图像
        '''
        # 若图像数据全为0，说明获取的图像不正确，这时大小可能与另一张的不同，修改其大小与正常的相同，以便拼接
        # 判断不为0的元素是否少于一定数量，少于则说明图的数据像基本都是0
        f1 = np.zeros((self.height, self.width, 3)) if np.count_nonzero(frame1) < 666 else frame1
        f0 = np.zeros((self.height, self.width, 3)) if np.count_nonzero(frame0) < 666 else frame0
        return np.hstack((f1, f0))

    def make_img_packet(self, frame, size):
        '''
        头（2字节）    栈长度（16字节，字符串：'4545171         '，表示该图片的大小）  数据字   数据（图片数据）  校验和  结束字节
        0x66  0xaa     0x08                                                        0x80    "5fdfasdgag"     0x00   0xfc
        :param frame:
        :param size: 要变换的大小
        :return: 发送给61613数据端口的图像数据
        '''
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]  # 视频编码参数
        frame = cv2.resize(frame, size, interpolation=cv2.INTER_LINEAR)
        result, img_encode = cv2.imencode('.jpg', frame, encode_param)  # 编码图像
        data_bytes = np.array(img_encode).tobytes()  # 转换为二进制字符串发送
        data_len_byte = str(len(data_bytes)).ljust(16).encode()
        # 图像数据太大，不计算校验和
        return HEAD + data_len_byte + b'\x80' + data_bytes + b'\x00' + END
        # return HEAD + data_len_byte + b'\x80'

    def send_image(self, frame):
        '''
        发送图像
        :param frame:
        :return:
        '''
        try:
            if not self.has_sock_error:
                self.sock_image.send(self.make_img_packet(frame, self.send_size))
                self.send_image_count += 1
                print(self.send_image_count)
                # self.has_sock_error = True  # 调试，发送一帧则立即停止
        except Exception as e:
            self.has_sock_error = True
            self.logger.error(e)

    def run(self):
        '''
        运行
        :return:
        '''
        self.open_cam(1)
        self.open_cam(0)

        # 读取第一帧
        rs1, rs0, frame1, frame0 = self.next_frame()

        if rs1 & rs0:
            self.logger.info("Now I'm fetching images")
        else:
            self.logger.error(
                "Sorry,I can't fetch image")

        # 该进程实际工作
        while rs1 & rs0:

            # 合并两个图像准备发送出去
            frame = self.montage(frame1, frame0)

            # 发送信息给小车服务器
            if self.need_to_connect_server:
                self.send_image(frame)

            # 是否弹出窗口显示图像
            if self.need_to_show:
                cv2.imshow('CAM', frame)

            # 读取下一帧
            rs1, rs0, frame1, frame0 = self.next_frame()

            if cv2.waitKey(1) == 27:
                self.clean()
                break

        else:
            self.logger.error('Capture error')  # 获取图像失败

        self.clean()  # 循环结束后清理工作


def run():
    # fromfile_prefix_chars='@'表明命令行可以使用@file直接读取file文件中的命令参数，但是file
    # 中的参数需要一个一行（ -t 5 -s foo 则文件中的字符串应为 '-t\n5\n-s\nfoo'）
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=textwrap.dedent(u'''\
                betagun 计算程序
                --------------------------------
                '''),
        fromfile_prefix_chars='@'
    )

    # 添加nargs='?'参数让这个位置参数成为可选地
    parser.add_argument(
        'width', action='store', type=int, default=640, nargs='?',
        help=u'摄像头获取图像的宽',
    )
    parser.add_argument(
        'height', action='store', type=int, default=480, nargs='?',
        help=u'摄像头获取图像的高',
    )
    parser.add_argument(
        '-s', '--show', action='store_true', dest='need_to_show',
        help=u'显示图像',
    )
    parser.add_argument(
        '-c', '--connect', action='store_true', dest='need_to_connect_server',
        help=u'连接服务器',
    )

    ns = parser.parse_args()
    Capture(
        size=(ns.width, ns.height),
        send_size=(320, 120),
        need_to_show=ns.need_to_show,
        need_to_connect_server=ns.need_to_connect_server
    ).run()


if __name__ == '__main__':
    run()

