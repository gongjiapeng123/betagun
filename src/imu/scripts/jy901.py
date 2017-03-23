#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''
连接小车61612，获取jy901数据并发送imu主题和odom主题，odom主题中航向角是相对的

jy901的坐标系：
右手，相对于传感器
X指向右侧，Y指向前方，Z指向上方
绕X旋转为pitch，Y为roll，Z为yaw

ROS base_link
的X指向前，Y指向左，Z指向上方

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
from sensor_msgs.msg import Imu
from imu.msg import CarPose
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

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

        self.is_first = True
        self.first_car_pose_msg = CarPose()

        # 以上三个数值由根据模块所标注的x、y、z轴，以右手法则来看，拇指指向自己，顺时针为负值，逆时针为正值
		
        self.vx_o = 0.0
        self.vy_o = 0.0
        self.vz_o = 0.0

        # 初始位置
        self.x_o = 0.0
        self.y_o = 0.0
        self.z_o = 0.0

        self.cnt = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.pattern = re.compile(r'\x66\xaa.{3,}\xfc')

        '''
        ROS
        '''
        # node
        rospy.init_node(b'jy901', anonymous=True)

        # We only care about the most recent measurement, i.e. queue_size=1
        self.pub_imu = rospy.Publisher(b'imu', Imu, queue_size=1)
        self.pub_car_pose = rospy.Publisher(b'car_pose', CarPose, queue_size=1)
        self.pub_odom = rospy.Publisher(b'imu_odom', Odometry, queue_size=1)
        self.odom_broadcaster = tf.TransformBroadcaster()

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
            0.00001, 0, 0,
            0, 0.00001, 0,
            0, 0, 0.00001
        ]
        self.car_pose_msg = CarPose()
        self.odom_msg = Odometry()

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

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
        self.current_time = rospy.Time.now()
        data_to_list = list(map(lambda s: float(s), str(data).split(b' ')))
		
        (self.ax, self.ay, self.az, 
        self.wx, self.wy, self.wz,
        self.pitch, self.roll, self.yaw,
        self.temperature) = data_to_list

        '''imu'''
        if self.verbose == 'raw':
            print('*' * 20)
            print('TEMP:', self.temperature)
            print('A:', self.ax, self.ay, self.az)
            print('W:', self.wx, self.wy, self.wz)
            print('pitch, roll, yaw:', self.pitch, self.roll, self.yaw)

        # 转换弧度，JY901的方向是(East, North, Up)，转换到车体PEP103
        pitch_rad = roll_rad = yaw_rad = 0
        pitch_rad = -self.pitch * degrees2rad
        roll_rad = self.roll * degrees2rad
        yaw_rad = self.yaw * degrees2rad

        # 此处微调数值
        self.imu_msg.linear_acceleration.x = self.ay - 0.05
        self.imu_msg.linear_acceleration.y = -self.ax - 0.45
        self.imu_msg.linear_acceleration.z = self.az + 0.10

        self.imu_msg.angular_velocity.x = self.wy * degrees2rad
        self.imu_msg.angular_velocity.y = -self.wx * degrees2rad
        self.imu_msg.angular_velocity.z = self.wz * degrees2rad

        q = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        self.imu_msg.orientation.x = q[0]
        self.imu_msg.orientation.y = q[1]
        self.imu_msg.orientation.z = q[2]
        self.imu_msg.orientation.w = q[3]
        self.imu_msg.header.stamp = self.current_time
        self.imu_msg.header.frame_id = 'jy901_imu'
        self.imu_msg.header.seq = self.cnt

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

        '''car pose'''
        if self.is_first:  # 第一次获取imu数据
            self.car_pose_msg.roll = self.first_car_pose_msg.roll = roll_rad
            self.car_pose_msg.pitch = self.first_car_pose_msg.pitch = pitch_rad
            self.car_pose_msg.yaw = self.first_car_pose_msg.yaw = yaw_rad
            self.car_pose_msg.roll_relative = self.first_car_pose_msg.roll_relative = 0
            self.car_pose_msg.pitch_relative = self.first_car_pose_msg.pitch_relative = 0
            self.car_pose_msg.yaw_relative = self.first_car_pose_msg.yaw_relative = 0

            self.is_first = False
        else:
            self.car_pose_msg.roll = roll_rad
            self.car_pose_msg.pitch = pitch_rad
            self.car_pose_msg.yaw = yaw_rad
            self.car_pose_msg.roll_relative = roll_rad - self.first_car_pose_msg.roll
            self.car_pose_msg.pitch_relative = pitch_rad - self.first_car_pose_msg.pitch
            self.car_pose_msg.yaw_relative = yaw_rad - self.first_car_pose_msg.yaw

        self.car_pose_msg.header.stamp = self.current_time
        self.car_pose_msg.header.frame_id = 'base_footprint'
        self.car_pose_msg.header.seq = self.cnt

        '''2d odom'''
        
        # 根据yaw获得四元数
        yaw_rad_o = self.car_pose_msg.yaw_relative
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, yaw_rad_o)

        # 根据yaw计算imu相对于全局坐标（odom）的x，y分量的加速度分量、速度分量、位移分量，注意坐标系的方向
        dt = (self.current_time - self.last_time).to_sec()

        ax_o = self.imu_msg.linear_acceleration.x * math.cos(yaw_rad_o) \
            + self.imu_msg.linear_acceleration.y * math.cos(yaw_rad_o + math.pi / 2)
        ay_o = self.imu_msg.linear_acceleration.x * math.sin(yaw_rad_o) \
            + self.imu_msg.linear_acceleration.y * math.sin(yaw_rad_o + math.pi / 2)
        self.vx_o = self.vx_o + ax_o * dt
        self.vy_o = self.vy_o + ay_o * dt

        self.x_o = self.x_o + self.vx_o * dt + ax_o * dt * dt / 2
        self.y_o = self.y_o + self.vy_o * dt + ay_o * dt * dt / 2

        # 发布坐标变换
        self.odom_broadcaster.sendTransform(
            (self.x_o, self.y_o, self.z_o),
            odom_quat,
            self.current_time,
            "base_footprint",
            "odom"
        )
        # set the position
        self.odom_msg.pose.pose = Pose(
            Point(self.x_o, self.y_o, self.z_o), 
            Quaternion(*odom_quat)
        )
        self.odom_msg.pose.covariance = [
            0.1, 0, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0, 0,
            0, 0, 10000, 0, 0, 0,
            0, 0, 0, 10000, 0, 0,
            0, 0, 0, 0, 10000, 0,
            0, 0, 0, 0, 0, 0.01,
        ]

        # set the velocity
        self.odom_msg.twist.twist = Twist(
            Vector3(self.vx_o, self.vy_o, self.vz_o), 
            Vector3(
                self.imu_msg.angular_velocity.x, 
                self.imu_msg.angular_velocity.y, 
                self.imu_msg.angular_velocity.z
            )
        )
        self.odom_msg.twist.covariance = [
            1, 0, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0, 0,
            0, 0, 10000, 0, 0, 0,
            0, 0, 0, 10000, 0, 0,
            0, 0, 0, 0, 10000, 0,
            0, 0, 0, 0, 0, 0.01,
        ]

        self.odom_msg.header.stamp = self.current_time
        self.odom_msg.header.frame_id = 'imu_odom'
        self.odom_msg.child_frame_id = 'imu'
        self.odom_msg.header.seq = self.cnt

        self.pub_imu.publish(self.imu_msg)
        self.pub_car_pose.publish(self.car_pose_msg)
        self.pub_odom.publish(self.odom_msg)
        self.cnt += 1
        self.last_time = rospy.Time.now()

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

