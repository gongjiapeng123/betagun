#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
连接小车616
'''

from __future__ import print_function
from __future__ import unicode_literals
from __future__ import division
from __future__ import absolute_import

import socket
import re
import struct

# ROS
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

degrees2rad = math.pi / 180.0

def byte_value(uint8):
    '''
    bytearray在python2中使用find时需要将uint8转换为byte str
    '''
    from platform import python_version
    return uint8 if python_version()[0] == '3' else chr(uint8)

class JY901:
    def __init__(self):
        self.temperature = 0  # 温度

        self.ax = 0  # 加速度
        self.ay = 0
        self.az = 0

        self.wx = 0  # 角速度
        self.wy = 0
        self.wz = 0

        self.pitch = 0  # 俯仰角，对于JY901，是其x轴的旋转角度（正的度数为向上俯仰，负的度数为向下俯冲）
        self.roll = 0  # 侧滚角，对于JY901，是其y轴的旋转角度（正的度数为向右侧滚，负的度数为向左侧滚）
        self.yaw = 0  # 航向角，对于JY901，是其z轴的旋转角度(指向北时为0度，向西为90度，向东为-90度)

        # 以上三个数值由根据模块所标注的x、y、z轴，以右手法则来看，拇指指向自己，顺时针为负值，逆时针为正值

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.pattern = re.compile(r'\x66\xaa.{3,}\xfc')

        '''
        ROS
        '''
        # node
        rospy.init_node(b'jy901')

        # We only care about the most recent measurement, i.e. queue_size=1
        self.pub = rospy.Publisher(b'imu', Imu, queue_size=1)

        # msg
        self.imu_msg = Imu()
        self.imu_msg.orientation_covariance = [
            0.0025, 0, 0,
            0, 0.0025, 0,
            0, 0, 0.0025
        ]
        self.imu_msg.angular_velocity_covariance = [
            0.02, 0, 0,
            0, 0.02, 0,
            0, 0, 0.02
        ]
        self.imu_msg.linear_acceleration_covariance = [
            0.04, 0, 0,
            0, 0.04, 0,
            0, 0, 0.04
        ]

    def _check(self):
        '''
        校验数据包，并获取其中数据
        '''
        pass

    def _parse_data(self, packet):
        pass

    def start(self):
        dataBuf = bytearray()
        try:
            self.sock.connect(('127.0.0.1', 61612))
            print("I connect successfully")
            self.sock.send(b'#python:liubiggun#')
            data = b''

            # 数据格式
            # 头             栈长度     数据字   数据             校验和    结束字节
            # 0x66  0xaa     0x##      0x81    10 个 字符串       0x##     0xfc
            # JY901数据（3个加速度，3个角速度，3个角度[pitch、roll、yaw]，温度）
            while not rospy.is_shutdown():
                data = self.sock.recv(512)
                dataBuf.extend(data)
                lastIndex = 0
                for match in self.pattern.finditer(dataBuf):
                    packet = self._check(match.group())
                    lastIndex = match.end()
                # 将已经进行正则匹配过的字符串剪掉
                if lastIndex > 0:
                    dataBuf = dataBuf[lastIndex:-1]

            self.sock.close() 
        except socket.error as e:
            print("connect error")

    
if __name__ == '__main__':
    jy901 = JY901()
    jy901.start()

