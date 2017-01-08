#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''
监听ros发布的主题，连接61615端口并发送计算结果
'''

from __future__ import print_function
from __future__ import unicode_literals
from __future__ import division
from __future__ import absolute_import

import argparse
import textwrap
import socket
import math
from crcmod.predefined import mkPredefinedCrcFun

crc8 = mkPredefinedCrcFun('crc-8')

# ROS
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

degrees2rad = math.pi / 180.0

# 命令的一些固定字节
HEAD = b'\x66\xAA'
END = b'\xFC'

class InfoCapture:
    
    def __init__(self, verbose=False):
        self.odom_topic_name = 'wheel_odom'
        self.odom_msg_type = Odometry
        self.verbose = verbose

        self.info_odom = {
            'ax': 0,  # 加速度
            'ay': 0,
            'az': 0,
            'wx': 0,  # 角速度
            'wy': 0,
            'wz': 0,
            'pitch': 0,  # 俯仰角，x轴的旋转角度(正的度数为向上俯仰，负的度数为向下俯冲)
            'roll': 0,  # 侧滚角，y轴的旋转角度(正的度数为向右侧滚，负的度数为向左侧滚)
            'yaw': 0,  # 航向角，z轴的旋转角度(指向北时为0度，向西为90度，向东为-90度)
            'x': 0,  # 位置
            'y': 0,
            'z': 0,
        }

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        '''
        ROS
        '''
        rospy.init_node(b'odometry', anonymous=True)

        # 连接tcp服务器
        self.connect_server()

    def subscriber_odom(self):
        '''
        监听里程计数据
        '''
        def parse_odom(data):
            position = data.pose.pose.position
            orientation = data.pose.pose.orientation
            linear = data.twist.twist.linear
            angular = data.twist.twist.angular

            self.info_odom['x'] = position.x
            self.info_odom['y'] = position.y
            self.info_odom['z'] = position.z
            self.info_odom['ax'] = linear.x
            self.info_odom['ay'] = linear.y
            self.info_odom['az'] = linear.z
            # 转换degree
            self.info_odom['wx'] = angular.x / degrees2rad
            self.info_odom['wy'] = angular.y / degrees2rad
            self.info_odom['wz'] = angular.z / degrees2rad
            
            direction = euler_from_quaternion([
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ])
            
            # 转换degree
            self.info_odom['pitch'] = direction[0] / degrees2rad
            self.info_odom['roll'] = direction[1] / degrees2rad
            self.info_odom['yaw'] = direction[2] / degrees2rad

            if self.verbose:
                print('*' * 20)
                print('A: {:0.2f} {:0.2f} {:0.2f}'.format(
                    self.info_odom['ax'],
                    self.info_odom['ay'],
                    self.info_odom['az']
                ))
                print('W: {:0.2f} {:0.2f} {:0.2f}'.format(
                    self.info_odom['wx'],
                    self.info_odom['wy'],
                    self.info_odom['wz']
                ))
                print('pitch, roll, yaw: {:0.2f} {:0.2f} {:0.2f}'.format(
                    self.info_odom['pitch'],
                    self.info_odom['roll'],
                    self.info_odom['yaw']
                ))
                print('x, y, z: {:0.2f} {:0.2f} {:0.2f}'.format(
                    self.info_odom['x'],
                    self.info_odom['y'],
                    self.info_odom['z']
                ))

        def make_packet(data):
            '''
            构造数据包
            头（2字节）    栈长度（）  数据字   数据           校验和    结束字节
            0x66  0xaa     0x##      0xa0     12 个字符串    0x##       0xfc
            :param frame:
            :param size: 要变换的大小
            :return: 发送给61615数据端口的数据
            '''
            parse_odom(data)
            data_bytes = (b'{} ' * 12)[: -1].format(
                self.info_odom['ax'],
                self.info_odom['ay'],
                self.info_odom['az'],
                self.info_odom['wx'],
                self.info_odom['wy'],
                self.info_odom['wz'],
                self.info_odom['pitch'],
                self.info_odom['roll'],
                self.info_odom['yaw'],
                self.info_odom['x'],
                self.info_odom['y'],
                self.info_odom['z'],
            )  # 转换为二进制字符串发送
            data_bytes_len = chr(len(data_bytes))
            data_check = b'{0}\xA1{1}'.format(data_bytes_len, data_bytes)  # 栈长度 + 命令字 + 数据

            return HEAD + data_check + chr(crc8(data_check)) + END

        def callback(data):
            packet = make_packet(data)
            self.sock.send(packet)

        rospy.Subscriber(self.odom_topic_name, self.odom_msg_type, callback)

    def connect_server(self):
        '''
        连接服务器，使得可以将数据传递给服务器
        :return:
        '''
        try:
            self.sock.connect(('127.0.0.1', 61616))

            # 告诉服务器本进程是python进程
            self.sock.send(b'#python:gxnu#')

            print('I connect successfully')
        except socket.error as e:
            print(e)

    def run(self):
        '''
        运行
        :return:
        '''
        self.subscriber_odom()

        print("Now I'm fetching infos")
        rospy.spin()


def run():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=textwrap.dedent(u'''\
                betagun 传输info给tcpserver
                --------------------------------
                '''),
        fromfile_prefix_chars='@'
    )

    parser.add_argument(
        '-v', '--verbose', action='store_true', dest='verbose',
        help=u'print info',
    )
    parser.add_argument(
        '__name', action='store',
        help=u'ros',
    )
    parser.add_argument(
        '__log', action='store',
        help=u'ros',
    )
    
    ns = parser.parse_args()
    InfoCapture(
        verbose=ns.verbose
    ).run()

    
if __name__ == '__main__':
    run()
