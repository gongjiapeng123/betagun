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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

degrees2rad = math.pi / 180.0

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

        self.pub = rospy.Publisher(b'odom', Odometry, queue_size=3)
        self.odom_broadcaster = tf.TransformBroadcaster()

        # msg
        self.odom_msg = Odometry()

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

    def _parse_and_publish(self, data):
        data_to_list = list(map(lambda s: float(s), str(data).split(b' ')))
		
        (self.left_count, self.right_count) = data_to_list

        self.odom_msg.header.stamp = rospy.Time.now()
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
            #print('W: {:0.2f} {:0.2f} {:0.2f}'.format(
                #self.imu_msg.angular_velocity.x, 
                #self.imu_msg.angular_velocity.y, 
                #self.imu_msg.angular_velocity.z
            #))
            
        self.pub.publish(self.imu_msg)

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
    wheelOdom = WheelOdom(verbose = True)
    wheelOdom.start()

