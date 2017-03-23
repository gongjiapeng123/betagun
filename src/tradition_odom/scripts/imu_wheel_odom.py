#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''
订阅car_speed和imu主题，并发送里程计主题，此处航向角是相对的
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
from jy901.msg import CarPose
from wheel_odom.msg import CarSpeed
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
roslib.load_manifest('tradition_odom')

class ImuWheelOdom:

    def __init__(self, verbose=''):
        self.verbose = verbose

        self.car_pose_msg = CarPose()
        self.car_speed_msg = CarSpeed()

        self.last_car_yaw = 0  # 上一时刻航向角

        # 初始位置
        self.x = 0.0
        self.y = 0.0

        self.cnt = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.pattern = re.compile(r'\x66\xaa.{3,}\xfc')

        '''
        ROS
        '''
        # node
        rospy.init_node(b'tradition_odom', anonymous=True)

        self.pub_odom = rospy.Publisher(b'tradition_odom', Odometry, queue_size=3)
        self.odom_broadcaster = tf.TransformBroadcaster()

        # msg
        self.odom_msg = Odometry()

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

    def _parse_and_publish(self):
        '''
        解析并发布里程数据
        '''
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()
        delta = self.car_speed_msg.vx * dt
        delta_x = delta * math.cos(self.car_pose_msg.yaw_relative)
        delta_y = delta * math.sin(self.car_pose_msg.yaw_relative)
        vth = (self.car_pose_msg.yaw_relative - self.last_car_yaw) / dt

        self.x += delta_x
        self.y += delta_y

        # 根据yaw获得四元数
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.car_pose_msg.yaw_relative)

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
        self.odom_msg.twist.twist = Twist(Vector3(self.car_speed_msg.vx, 0, 0), Vector3(0, 0, vth))
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

        if self.verbose:
            print('*' * 20)
            print('th: {} v: {}'.format(
                self.car_pose_msg.yaw_relative,
                self.car_speed_msg.vx
            ))

        self.pub_odom.publish(self.odom_msg)
        self.cnt += 1
        self.last_car_yaw = self.car_pose_msg.yaw_relative
        self.last_time = rospy.Time.now()

    def subscriber_car_pose(self):
        '''
        监听car_pose主题
        '''
        def callback(data):
            self.car_pose_msg = data

        rospy.Subscriber("imu", CarPose, callback)

    def subscriber_car_speed(self):
        '''
        监听car_speed主题
        '''
        def callback(data):
            self.car_speed_msg = data
            self._parse_and_publish()

        rospy.Subscriber("car_speed", CarSpeed, callback)

    def start(self):
        self.subscriber_car_pose()
        self.subscriber_car_speed()
        print("Now I'm fetching car pose and speed")
        rospy.spin()


if __name__ == '__main__':
    imuWheelOdom = ImuWheelOdom(verbose = True)
    imuWheelOdom.start()

