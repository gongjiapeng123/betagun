#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''
监听ros发布的主题，连接61615端口并发送计算结果
'''

from __future__ import print_function
from __future__ import unicode_literals
from __future__ import division
from __future__ import absolute_import

import socket
import math
from crcmod.predefined import mkCrcFun

# ROS
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

crc = mkCrcFun('crc-8')
degrees2rad = math.pi / 180.0

# 命令的一些固定字节
HEAD = b'\x66\xAA'
END = b'\xFC'


def byte_value(uint8):
    '''
    bytearray在python2中使用find时需要将uint8转换为byte str
    '''
    from platform import python_version
    return uint8 if python_version()[0] == '3' else chr(uint8)

class InfoCapture:
    
    def __init__(self, verbose=False):
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

        # 连接tcp服务器
        self.connect_server()

    def subscriber_odom(self):
        '''
        监听滤波后的里程计数据
        '''
        def make_packet():
            '''
            头（2字节）    栈长度（）  数据字   数据           校验和    结束字节
            0x66  0xaa     0x##      0xa0     12 个字符串    0x##       0xfc
            :param frame:
            :param size: 要变换的大小
            :return: 发送给61615数据端口的数据
            '''
            data_bytes = b' '.join(list(self.info_odom.keys())  # 转换为二进制字符串发送
            data_len_byte = chr(len(data_bytes)).encode() if python_version()[0] == '3' else chr(len(data_bytes))
            # 本地传输，不计算校验和
            return HEAD + data_len_byte + b'\xa0' + data_bytes + b'\x00' + END
        
        def callback(data):
            self.flag |= 0b10         
            self.frame_left = ros_numpy.numpify(data)
            self.handle()

        rospy.Subscriber('/odometry/filtered', Odometry, callback)

    def connect_server(self):
        '''
        连接服务器，使得可以将数据传递给服务器
        :return:
        '''
        try:
            self.sock.connect(('127.0.0.1', 61615))

            # 告诉服务器本进程是python进程
            self.sock.send(b'#python:gxnu#')

            print('I connect successfully')
        except socket.error as e:
            print(e)

    def _parse_and_publish(self, data):
        data_to_list = list(map(lambda s: float(s), str(data).split(b' ')))
		
        (self.ax, self.ay, self.az, 
        self.wx, self.wy, self.wz,
        self.pitch, self.roll, self.yaw,
        self.temperature) = data_to_list

        if self.verbose == 'raw':
            print('*' * 20)
            print('TEMP:', self.temperature)
            print('A:', self.ax, self.ay, self.az)
            print('W:', self.wx, self.wy, self.wz)
            print('pitch, roll, yaw:', self.pitch, self.roll, self.yaw)

        # 转换弧度，JY901的方向是(East, North, Up)，符合REP 103
        pitch_rad = roll_rad = yaw_rad = 0
        pitch_rad = self.pitch * degrees2rad
        roll_rad = self.roll * degrees2rad
        yaw_rad = self.yaw * degrees2rad

        # 此处微调数值
        self.imu_msg.linear_acceleration.x = self.ay * 9.8 + 0.05
        self.imu_msg.linear_acceleration.y = self.ax * 9.8 + 0.2
        self.imu_msg.linear_acceleration.z = self.az * 9.8 + 0.05

        self.imu_msg.angular_velocity.x = self.wy * degrees2rad
        self.imu_msg.angular_velocity.y = self.wx * degrees2rad
        self.imu_msg.angular_velocity.z = self.wz * degrees2rad

        q = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        self.imu_msg.orientation.x = q[0]
        self.imu_msg.orientation.y = q[1]
        self.imu_msg.orientation.z = q[2]
        self.imu_msg.orientation.w = q[3]
        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.header.frame_id = 'jy901_imu'
        self.imu_msg.header.seq = self.cnt
        self.cnt += 1

        if self.verbose == 'ros':
            print('*' * 20)
            print('TEMP:', self.temperature)
            print('A: {:0.2f} {:0.2f} {:0.2f}'.format(
                self.imu_msg.linear_acceleration.x, 
                self.imu_msg.linear_acceleration.y, 
                self.imu_msg.linear_acceleration.z
            ))
            print('W: {:0.2f} {:0.2f} {:0.2f}'.format(
                self.imu_msg.angular_velocity.x, 
                self.imu_msg.angular_velocity.y, 
                self.imu_msg.angular_velocity.z
            ))
            print('pitch, roll, yaw: {:0.2f} {:0.2f} {:0.2f}'.format(
                pitch_rad, 
                roll_rad, 
                yaw_rad
            ))

        self.pub.publish(self.imu_msg)

    def run(self):
        '''
        运行
        :return:
        '''
        self.subscriber_odom()

        print("Now I'm fetching infos")
        rospy.spin()

    
if __name__ == '__main__':
    info_capture = InfoCapture(verbose=True)
    info_capture.run()

