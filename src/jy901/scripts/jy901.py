#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''
连接小车61612，获取jy901数据并发送主题
'''

from __future__ import print_function
from __future__ import unicode_literals
from __future__ import division
from __future__ import absolute_import

import socket
import re
import math
from crcmod.predefined import mkPredefinedCrcFun

crc8 = mkPredefinedCrcFun('crc-8')

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
    
    def __init__(self, verbose=''):
        self.verbose = verbose
        
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
		
        self.cnt = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.pattern = re.compile(r'\x66\xaa.{3,}\xfc')

        '''
        ROS
        '''
        # node
        rospy.init_node(b'jy901', anonymous=True)

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

    def _check(self, packet):
        '''
        校验数据包，并获取其中数据
        '''
        data_len = packet[2]
        data = packet[4: 4 + data_len]
        check_sum = packet[-2]
        data_check = packet[2: -2]  # 栈长度 + 命令字 + 数据

        if crc8(str(data_check)) == check_sum:
            return data

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
        self.imu_msg.linear_acceleration.y = self.ax * 9.8
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

    def start(self):
        dataBuf = bytearray()
        try:
            self.sock.connect(('127.0.0.1', 61612))
            print("I connect successfully")
            self.sock.send(b'#python:gxnu#')
            data = b''

            # 数据格式
            # 头             栈长度     数据字   数据             校验和    结束字节
            # 0x66  0xaa     0x##      0x81    10 个 字符串       0x##     0xfc
            # JY901数据（3个加速度，3个角速度，3个角度[pitch、roll、yaw]，温度）
            while not rospy.is_shutdown():
                data = self.sock.recv(1)
                dataBuf.extend(data)
                lastIndex = 0
                for match in self.pattern.finditer(dataBuf):
                    data = self._check(match.group())
                    if data is not None:
                        self._parse_and_publish(data)
                    lastIndex = match.end()
                # 将已经进行正则匹配过的字符串剪掉
                if lastIndex > 0:
                    dataBuf = dataBuf[lastIndex:-1]

            self.sock.close() 
        except socket.error as e:
            print("connect error")

    
if __name__ == '__main__':
    jy901 = JY901(verbose='ros')
    jy901.start()

