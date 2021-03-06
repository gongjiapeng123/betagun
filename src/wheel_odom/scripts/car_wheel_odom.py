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

WHEEL_DIAMETER = 0.04  # 轮子直径
CODED_DISC_GRID_NUM = 50.0  # 栅格数
WHEEL_L = 0.185  # 轮间距

def byte_value(uint8):
    '''
    bytearray在python2中使用find时需要将uint8转换为byte str
    '''
    from platform import python_version
    return uint8 if python_version()[0] == '3' else chr(uint8)

class WheelOdom:

    def __init__(self, verbose=''):
        self.verbose = verbose
        self.dt = 0.0  # 采样周期

        self.left_count = 0  # 采样时间内的码盘计数值
        self.right_count = 0
        self.left_cmd_speed = 0  # 控制命令的速度值(0~100)
        self.right_cmd_speed = 0


        self.left_s = 0.0  # 左轮行驶距离
        self.right_s = 0.0  # 右轮行驶距离
        self.car_delta_x = 0.0  # 前进行驶距离
        self.car_delta_y = 0.0  # 侧移行驶距离
        self.car_delta_th = 0.0  # 航向角变化角度
        self.left_speed = 0.0
        self.right_speed = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        # 全局位姿
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0  # yaw

        self.cnt = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.pattern = re.compile(r'\x66\xaa.{3,}\xfc')

        '''
        ROS
        '''
        # node
        rospy.init_node(b'wheel_odom', anonymous=True)

        self.pub_car_speed = rospy.Publisher(b'car_speed', CarSpeed, queue_size=3)
        self.pub_odom = rospy.Publisher(b'wheel_odom', Odometry, queue_size=3)
        self.odom_broadcaster = tf.TransformBroadcaster()

        # msg
        self.odom_msg = Odometry()
        self.car_speed_msg = CarSpeed()

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

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

    def _pretreat(self):
        '''
        获取速度前预处理，由于光电检测器的灵敏度有问题，低速直线运动时，右轮检测一直比左边小
        但是实际是接近直线行走的，这里简单处理一下，低速时若左右电机命令相差不大，近似直线命令，
        那么令右轮的速度检测值等于左轮，这是workaround，还需要重新选用光电编码器
        '''
        if ((self.left_cmd_speed > 0 and self.right_cmd_speed > 0) \
            or (self.left_cmd_speed < 0 and self.right_cmd_speed < 0)) \
            and (abs(self.left_cmd_speed) < 66 and abs(self.left_cmd_speed) < 66) \
            and abs(self.left_cmd_speed - self.right_cmd_speed) < 5:
            self.right_count = self.left_count

    def _velocity_to_speed(self):
        '''
        将两轮的速度转化为x轴的速度(即前进方向的速度)和绕z轴旋转的速度。
        程序中self.dt为速度采样事时间。此处需将y轴速度设为0，即假定
        self.dt(s)内，机器人没有在垂直于轮子的方向上发生位移。
        左右轮速度的平均就是前进速度（即x轴速度），左右轮速度的差转化为旋转速度。
        '''
        # 获取位移和速度，计数器信息带有正负
        self.left_s = (self.left_count / CODED_DISC_GRID_NUM) * (WHEEL_DIAMETER * math.pi)
        self.left_speed = self.left_s / self.dt
        self.right_s = (self.right_count / CODED_DISC_GRID_NUM) * (WHEEL_DIAMETER * math.pi)
        self.right_speed = self.right_s / self.dt

        self.car_delta_x = (self.left_s + self.right_s) / 2.0
        self.vx = self.car_delta_x / self.dt
        self.car_delta_y = 0.0
        self.vy = 0.0
        self.car_delta_th = (self.right_s - self.left_s) / WHEEL_L
        self.vth = self.car_delta_th / self.dt

        self.car_speed_msg.total_left_count = self.car_speed_msg.total_left_count + self.left_count
        self.car_speed_msg.total_right_count = self.car_speed_msg.total_right_count + self.right_count
        self.car_speed_msg.left_count = self.left_count
        self.car_speed_msg.right_count = self.right_count
        self.car_speed_msg.left_cmd_speed = self.left_cmd_speed
        self.car_speed_msg.right_cmd_speed = self.right_cmd_speed
        self.car_speed_msg.car_delta_x = self.car_delta_x
        self.car_speed_msg.car_delta_y = self.car_delta_y
        self.car_speed_msg.car_delta_th = self.car_delta_th
        self.car_speed_msg.left_speed = self.left_speed
        self.car_speed_msg.right_speed = self.right_speed
        self.car_speed_msg.vx = self.vx
        self.car_speed_msg.vy = self.vy
        self.car_speed_msg.vth = self.vth
        self.car_speed_msg.header.stamp = self.current_time
        self.car_speed_msg.header.frame_id = 'base_footprint'
        self.car_speed_msg.header.seq = self.cnt

    def _speed_to_odom(self):
        '''
        将速度信息转换为里程计信息
        '''
        delta_x = self.car_delta_x * math.cos(self.th) - self.car_delta_y * math.sin(self.th)
        delta_y = self.car_delta_x * math.sin(self.th) + self.car_delta_y * math.cos(self.th)
        delta_th = self.car_delta_th

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # 根据yaw获得四元数
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # 发布坐标变换
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            self.current_time,
            "base_footprint",
            "odom"
        )
        # set the position
        self.odom_msg.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(*odom_quat))
        self.odom_msg.pose.covariance = [
            0.1, 0, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0, 0,
            0, 0, 10000, 0, 0, 0,
            0, 0, 0, 10000, 0, 0,
            0, 0, 0, 0, 10000, 0,
            0, 0, 0, 0, 0, 0.01,
        ]

        # set the velocity
        self.odom_msg.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
        self.odom_msg.twist.covariance = [
            1, 0, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0, 0,
            0, 0, 10000, 0, 0, 0,
            0, 0, 0, 10000, 0, 0,
            0, 0, 0, 0, 10000, 0,
            0, 0, 0, 0, 0, 0.01,
        ]

        self.odom_msg.header.stamp = self.current_time
        self.odom_msg.header.frame_id = 'wheel_odom'
        self.odom_msg.child_frame_id = 'wheel'
        self.odom_msg.header.seq = self.cnt

    def _parse_and_publish(self, data):
        '''
        解析并发布里程数据
        '''
        self.current_time = rospy.Time.now()
        self.dt = (self.current_time - self.last_time).to_sec()
        # 获取左右码盘计数器的值
        data_to_list = list(map(lambda s: int(s), str(data).split(b' ')))
        (
            self.left_count,
            self.right_count,
            self.left_cmd_speed,
            self.right_cmd_speed,
        ) = data_to_list

        self._pretreat()
        self._velocity_to_speed()
        self._speed_to_odom()

        if self.verbose:
            print('*' * 20)
            print('cmd speed: {} {}'.format(
                self.left_cmd_speed,
                self.right_cmd_speed
            ))
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

        self.pub_car_speed.publish(self.car_speed_msg)
        self.pub_odom.publish(self.odom_msg)
        self.cnt += 1
        self.last_time = rospy.Time.now()

    def start(self):
        data_buf = bytearray()
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
                data_buf.extend(data)
                last_index = 0
                for match in self.pattern.finditer(data_buf):
                    data = self._check(match.group())
                    if data is not None:
                        self._parse_and_publish(data)
                    last_index = match.end()
                # 将已经进行正则匹配过的字符串剪掉
                if last_index > 0:
                    data_buf = data_buf[last_index:-1]

            self.sock.close()
        except socket.error as e:
            print("connect error")


if __name__ == '__main__':
    wheelOdom = WheelOdom(verbose = True)
    wheelOdom.start()

