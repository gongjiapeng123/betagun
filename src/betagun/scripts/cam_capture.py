#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
连接小车61614端口，订阅图像主题，并发送图像数据给tcp服务器
'''

from __future__ import print_function
from __future__ import unicode_literals
from __future__ import division
from __future__ import absolute_import

import cv2
import numpy as np
import argparse
import textwrap
import socket

import rospy
from sensor_msgs.msg import Image
import ros_numpy

HEAD = b'\x66\xAA'
END = b'\xFC'

class Capture():
    def __init__(self, need_to_show=False, need_to_connect_server=True):
        self.width = 640
        self.height = 480
        self.send_size = (320, 120)  # 发送给服务器时的图片大小（两张拼接）
        self.need_to_show = need_to_show  # 是否在一个窗口显示图像：0不显示，1在一个窗口显示双目图像，2在两个窗口分别显示图像
        self.need_to_connect_server = need_to_connect_server  # 是否需要与服务器通信

        self.exit = False  # 是否停止获取图像

        self.sock_image = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 61614图像端口

        self.has_sock_error = False  # 指明当前是否IPC通信有问题，有问题就不要重复发送数据了
        self.send_image_count = 0
        self.flag = 0b00  # 指明当前是否在获取图像数据 00：表示未获取 01：已获取右边 10：已获取左边 11：获取完毕
        self.frame_left = None
        self.frame_right = None
        
        rospy.init_node('capture', anonymous=True)

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

            print("I connect successfully")
        except socket.error as e:
            self.has_sock_error = True
            print(e)

    def montage(self, frame_left, frame_right):
        '''
        双目摄像头时，拼接两帧图像
        '''
        # 若图像数据全为0，说明获取的图像不正确，这时大小可能与另一张的不同，修改其大小与正常的相同，以便拼接
        # 判断不为0的元素是否少于一定数量，少于则说明图的数据像基本都是0
        f1 = np.zeros((self.height, self.width, 3)) if np.count_nonzero(frame_left) < 666 else frame_left
        f0 = np.zeros((self.height, self.width, 3)) if np.count_nonzero(frame_right) < 666 else frame_right
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
                # self.has_sock_error = True  # 调试，发送一帧立即停止
        except Exception as e:
            self.has_sock_error = True
            print(e)

    def subscriber_left(self):
        '''
        监听左侧
        '''
        def callback(data):
            self.flag |= 0b10         
            self.frame_left = ros_numpy.numpify(data)
            self.handle()

        rospy.Subscriber("/stereo/left/image_raw", Image, callback)

    def subscriber_right(self):
        '''
        监听右侧
        '''
        def callback(data):
            self.flag |= 0b01
            self.frame_right = ros_numpy.numpify(data)
            self.handle()

        rospy.Subscriber("/stereo/right/image_raw", Image, callback)

    def handle(self):
        if self.flag == 0b11:
            frame = self.montage(self.frame_left, self.frame_right)
            self.frame_left = None
            self.frame_right = None
            self.flag = 0b00
            if self.need_to_show:
                cv2.imshow('CAM', frame)
                cv2.waitKey(1)
            if self.need_to_connect_server:
                self.send_image(frame)

    def run(self):
        '''
        运行
        :return:
        '''
        self.subscriber_left()
        self.subscriber_right()
        print("Now I'm fetching images")
        rospy.spin()


def run():
    # fromfile_prefix_chars='@'表明命令行可以使用@file直接读取file文件中的命令参数，但是file
    # 中的参数需要一个一行（ -t 5 -s foo 则文件中的字符串应为 '-t\n5\n-s\nfoo'）
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=textwrap.dedent(u'''\
                betagun 传输图片给tcpserver
                --------------------------------
                '''),
        fromfile_prefix_chars='@'
    )

    parser.add_argument(
        '-s', '--show', action='store_true', dest='need_to_show',
        help=u'显示图像',
    )
    parser.add_argument(
        '-c', '--connect', action='store_true', dest='need_to_connect_server',
        help=u'连接服务器',
    )
    parser.add_argument(
        '__name', action='store', default='', nargs='?',
        help=u'ros',
    )
    parser.add_argument(
        '__log', action='store', default='', nargs='?',
        help=u'ros',
    )

    ns = parser.parse_args()
    Capture(
        need_to_show=ns.need_to_show,
        need_to_connect_server=ns.need_to_connect_server
    ).run()


if __name__ == '__main__':
    run()

