#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''
连接小车61614，获取arduino数据并发送主题
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
import roslib
import tf
from wheel_odom.msg import CarSpeed
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
roslib.load_manifest('wheel_odom')

WHEEL_DIAMETER = 0.04
CODED_DISC_GRID_NUM = 50.0
VHZ = 20.0
DELTA_T = 1 / VHZ
WHEEL_L = 0.185

def byte_value(uint8):
    '''
    bytearray在python2中使用find时需要将uint8转换为byte str
    '''
    from platform import python_version
    return uint8 if python_version()[0] == '3' else chr(uint8)

class WheelOdom:

    def __init__(self, verbose=''):
        self.verbose = verbose
        
        self.left_count = 0
        self.right_count = 0
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
		
        self.cnt = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.pattern = re.compile(r'\x66\xaa.{3,}\xfc')

        '''
        ROS
        '''
        # node
        rospy.init_node(b'wheel_odom', anonymous=True)

        self.pub_car_speed = rospy.Publisher(b'car_speed', CarSpeed, queue_size=3)
        self.pub_odom = rospy.Publisher(b'odom', Odometry, queue_size=3)
        self.odom_broadcaster = tf.TransformBroadcaster()

        # msg
        self.odom_msg = Odometry()
        self.car_speed_msg = CarSpeed()

    def _check(self, packet):
        '''
        校验数据包，并获取其中数据
        '''
        if packet[3] != 0x83:
            return
        data_len = packet[2]
        data = packet[4: 4 + data_len]
        check_sum = packet[-2]
        data_check = packet[2: -2]  # 栈长度 + 命令字 + 数据

        if crc8(str(data_check)) == check_sum:
            return data

    def _get_velocity(self):
        '''
        获取速度，计数器信息带有正负
        '''
        self.left_speed = (self.left_count / CODED_DISC_GRID_NUM) * (WHEEL_DIAMETER * math.pi) * VHZ
        self.right_speed = (self.right_count / CODED_DISC_GRID_NUM) * (WHEEL_DIAMETER * math.pi) * VHZ

    def _speed_to_odom(self):  
        '''
        将两轮的速度转化为x轴的速度(即前进方向的速度)和绕z轴旋转的速度。
        程序中VHZ为速度采样频率。此处需将y轴速度设为0，即假定1/VHZ(s)内，
        机器人没有在垂直于轮子的方向上发生位移。
        左右轮速度的平均就是前进速度（即x轴速度），左右轮速度的差转化为旋转速度。
        '''
        delta_speed = self.right_speed - self.left_speed
        '''
        if delta_speed < 0:
            theta_to_speed = 0.0077  # 右转系数
        else:
            theta_to_speed = 0.0076  # 左转系数
        self.vth = delta_speed  * theta_to_speed * VHZ
        '''
        
        self.vth = (delta_speed) * DELTA_T / WHEEL_L
        self.vx = (self.left_speed + self.right_speed) / 2.0
        self.vy = 0.0

    def _parse_and_publish(self, data):
        '''
        解析并发布里程数据
        '''
        # 获取左右码盘计数器的值
        data_to_list = list(map(lambda s: int(s), str(data).split(b' ')))
        (self.left_count, self.right_count) = data_to_list

        self._get_velocity()
        self._speed_to_odom()
        
        current_time = rospy.Time.now()
        
        self.car_speed_msg.left_speed = self.left_speed
        self.car_speed_msg.right_speed = self.right_speed
        self.car_speed_msg.vx = self.vx
        self.car_speed_msg.vy = self.vy
        self.car_speed_msg.vth = self.vth
        self.odom_msg.header.stamp = current_time
        self.odom_msg.header.seq = self.cnt

        current_time = rospy.Time.now()
        self.odom_msg.header.stamp = current_time
        self.odom_msg.header.frame_id = 'wheel_odom'
        self.odom_msg.child_frame_id = 'base_footprint'
        self.odom_msg.header.seq = self.cnt
        self.cnt += 1

        if self.verbose:
            print('*' * 20)
            print('count: {} {}'.format(
                self.left_count, 
                self.right_count
            ))
            print('velocity(m/s): {} {}'.format(
                self.left_speed, 
                self.right_speed
            ))
            print('car_speed: vx: {} vth: {}'.format(
                self.vx, 
                self.vth
            ))
            #print('W: {:0.2f} {:0.2f} {:0.2f}'.format(
                #self.imu_msg.angular_velocity.x, 
                #self.imu_msg.angular_velocity.y, 
                #self.imu_msg.angular_velocity.z
            #))
            
        self.pub_car_speed.publish(self.car_speed_msg)
        self.pub_odom.publish(self.odom_msg)

    def start(self):
        dataBuf = bytearray()
        try:
            self.sock.connect(('127.0.0.1', 61613))
            print("I connect successfully")
            self.sock.send(b'#python:gxnu#')
            data = b''

            # 数据格式
            # 头             栈长度     数据字   数据             校验和    结束字节
            # 0x66  0xaa     0x##      0x83    2 个 字符串       0x##     0xfc
            # arduino传感器数据（两个计数器的值）
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
    wheelOdom = WheelOdom(verbose = True)
    wheelOdom.start()

