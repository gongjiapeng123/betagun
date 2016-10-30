#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from __future__ import unicode_literals
from __future__ import division
from __future__ import absolute_import

import serial
import serial.threaded
import struct
import ctypes
import datetime
import traceback
import math
import os

# ROS
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler


degrees2rad = math.pi / 180.0

def byte_value(uint8):
    from platform import python_version
    return uint8 if python_version()[0] == '3' else chr(uint8)


class JY901Protocol(serial.threaded.Packetizer):
    """
    使用串口多线程
    """

    def connection_made(self, transport):
        super(JY901Protocol, self).connection_made(transport)
        print("start")

    def data_received(self, data):
        """
        重写data_received方法。该方法每当有二进制数据从串口传入都会触发一次。知道没有0x55在buffer中
        JY901回传的数据包以0x55开头，但是并没有结束字节。这里由于其回传数据包中在起始字节0x55后面表示数据包类型的
        字节是0x5*，有0x50 & 0x5* = 0x50，所以，可以根据这个特点来提取数据包
        :param data:
        :return:
        """
        self.buffer.extend(data)
        while 0x55 in self.buffer:  # 如果当前已经收到buffer中第一个数据包的起始字节0x55
            index = self.buffer.find(byte_value(0x55))  # 0x55的索引
            if index == len(self.buffer) - 1:  # 如果0x55后面没有数据
                break
            if 0x50 & self.buffer[index + 1] != 0x50:  # 不满足数据包格式
                self.buffer = self.buffer[index + 1:]  # 丢弃第一个0x55个其之前无用的数据
            else:  # 满足数据包格式
                self.buffer = self.buffer[index:]  # 丢弃当前buffer第一个数据包之前无用的数据
                if len(self.buffer) > 11:  # 如果长度超过11，说明buffer中第一个数据包可以获取
                    packet, self.buffer = self.buffer[0:11], self.buffer[11:]  # 获取数据包，并保留剩下的buffer

                    # 校验和
                    checksum = ctypes.c_ubyte(0)
                    for b in packet[:-1]:
                        checksum.value += b
                    if checksum.value == packet[10]:  # 满足校验
                        """
                        处理数据包，因为是更新JY901实例中的数据，故该实现应该位于JY901类中，并且在运行子线程前应该将其实现
                        """
                        self.handle_packet(packet)
                else:  # 否则继续获取数据
                    break

    def connection_lost(self, exc):
        super(JY901Protocol, self).connection_lost(exc)
        if exc:
            traceback.print_exc(exc)
        print('port closed\n')


class JY901Base:
    def __init__(self, port=os.environ.get('BETAGUN_JY901_COMPORT'), baudrate=115200, timeout=1, verbose=False, callback=None, argv=()):
        """
        JY901九轴陀螺仪模块
        :param port: 串口设备字符串
        :param baudrate: 波特率
        :param timeout: 超时秒数
        :param verbose: 是否在在控制台输出各字段的值
        :param callback: 获取得到数据后需要进行的回调函数
        :param argv: 回调函数的参数元组
        :return:
        """
        self.ser = serial.Serial(
            port=port,  # number of device, numbering starts at
            # zero. if everything fails, the user
            # can specify a device string, note
            # that this isn't portable anymore
            # if no port is specified an unconfigured
            # an closed serial port object is created
            baudrate=baudrate,  # baud rate
            bytesize=serial.EIGHTBITS,  # number of databits
            parity=serial.PARITY_NONE,  # enable parity checking
            stopbits=serial.STOPBITS_ONE,  # number of stopbits
            # timeout=None,                  # set a timeout value, None for waiting forever
            # timeout=0,                     # non-block read
            # timeout = 2,                   # timeout block read
            timeout=timeout,
            xonxoff=0,  # enable software flow control
            rtscts=0,  # enable RTS/CTS flow control
            inter_byte_timeout=None
        )
        self.chipTime = 0  # 片上时间
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

        self.mx = 0  # 磁场
        self.my = 0
        self.mz = 0

        self.data_thread = None  # 获取串口数据包更新数据的子线程
        self.verbose = verbose
        self.update_data_callback = lambda: callback(*argv) if callback else None

        self.cnt = 0  # 实例已经处理的数据包数

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

    def _decode_packet(self, packet):
        """
        解析JY901模块回传内容，其回传内容为以下格式：
        数据头  数据包类型           数据                    校验和
        0x55     0x5*       short x  4 （8 bytes）    前10个字节相加(byte)      总共11个字节
        数据包为0x50的为片上时间回传，有不同的格式：
        0x55     0x50      YY MM DD hh mm ss msl msh        校验和             总共11个字节
        :param packet:JY901传过来的一个数据包
        :return:
        """

        def Chiptime():
            """
            片上时间：0x55     0x50      YY MM DD hh mm ss msl msh        校验和
            YY:20YY年  MM:月  DD:日   hh:时  mm:分  ss:秒  msl msh:毫秒(short)
            :return:
            """
            self.chipTime = datetime.datetime(2000 + packet[2], packet[3], packet[4],
                                              packet[5], packet[6], packet[7], data_values[5])

        def Acceleration():
            """
            加速度：0x55 0x51 AxL AxH AyL AyH AzL AzH TL TH SUM  ( TL TH 是温度)
            :return:
            """
            self.temperature = data_values[5] / 100
            self.ax = data_values[2] / 32768.0 * 16
            self.ay = data_values[3] / 32768.0 * 16
            self.az = data_values[4] / 32768.0 * 16

        def AngularV():
            """
            角速度：0x55 0x52 wxL wxH wyL wyH wzL wzH TL TH SUM ( TL TH 是温度)
            :return:
            """
            self.temperature = data_values[5] / 100
            self.wx = data_values[2] / 32768.0 * 2000
            self.wy = data_values[3] / 32768.0 * 2000
            self.wz = data_values[4] / 32768.0 * 2000

        def Angle():
            """
            角度：0x55 0x53 RollL RollH PitchL PitchH YawL YawH TL TH SUM ( TL TH 是温度)
            :return:
            """
            self.temperature = data_values[5] / 100
            self.pitch = data_values[2] / 32768.0 * 180
            self.roll = data_values[3] / 32768.0 * 180
            self.yaw = data_values[4] / 32768.0 * 180

        def Magnet():
            """
            磁场：0x55 0x54 HxL HxH HyL HyH HzL HzH TL TH SUM ( TL TH 是温度)
            :return:
            """
            self.temperature = data_values[5] / 100
            self.mx = data_values[2]
            self.my = data_values[3]
            self.mz = data_values[4]

        def nothing(): pass

        data_values = struct.unpack('<2B4hB', packet)
        do_decode = {0x50: Chiptime, 0x51: Acceleration, 0x52: AngularV, 0x53: Angle, 0x54: Magnet,
                     0x55: nothing, 0x56: nothing, 0x57: nothing, 0x58: nothing, 0x59: nothing}

        return do_decode[data_values[1]]()

    def _update_data(self, packet):
        """
        处理一条数据包
        :param packet:
        :return:
        """
        # 解析数据
        self._decode_packet(packet)
        if self.verbose == 'raw':
            print('Now is ', self.cnt, 'th packet')
            print('片上时间：', self.chipTime)
            print('温度：', self.temperature)
            print('加速度：', self.ax, self.ay, self.az)
            print('角速度：', self.wx, self.wy, self.wz)
            print('位姿（俯仰、侧滚、航向）：', self.pitch, self.roll, self.yaw)
            print('磁场：', self.mx, self.my, self.mz)

        # 转换弧度
        yaw_rad = pitch_rad = roll_rad = 0
        yaw_rad = self.yaw * degrees2rad
        # JY901的俯仰相关的轴指向右， 但是 ROS 指向左 (see REP 103)
        pitch_rad = -self.pitch * degrees2rad
        roll_rad = self.roll * degrees2rad

        # ROS中标准位x轴指向前，y轴指向左边，z轴指向上方，此处微调数值
        self.imu_msg.linear_acceleration.x = self.ay * 9.79 - 0.5
        self.imu_msg.linear_acceleration.y = -self.ax * 9.79 + 1.3
        self.imu_msg.linear_acceleration.z = -self.az * 9.79 + 0.5

        self.imu_msg.angular_velocity.x = self.wy * degrees2rad - 0.005
        self.imu_msg.angular_velocity.y = self.wx * degrees2rad + 0.035
        self.imu_msg.angular_velocity.z = self.wz * degrees2rad

        q = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        self.imu_msg.orientation.x = q[0]
        self.imu_msg.orientation.y = q[1]
        self.imu_msg.orientation.z = q[2]
        self.imu_msg.orientation.w = q[3]
        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.header.frame_id = 'base_imu_link'
        self.imu_msg.header.seq = self.cnt
        self.cnt += 1

        if self.verbose == 'ros':
            print('Now is {}th packet'.format(self.cnt))
            print('温度：', self.temperature)
            print('加速度： {:0.2f} {:0.2f} {:0.2f}'.format(self.imu_msg.linear_acceleration.x , self.imu_msg.linear_acceleration.y, self.imu_msg.linear_acceleration.z))
            print('角速度： {:0.2f} {:0.2f} {:0.2f}'.format(self.imu_msg.angular_velocity.x, self.imu_msg.angular_velocity.y, self.imu_msg.angular_velocity.z))
            print('位姿（俯仰、侧滚、航向）： {:0.2f} {:0.2f} {:0.2f}'.format(pitch_rad, roll_rad, yaw_rad))

        self.pub.publish(self.imu_msg)

        if self.update_data_callback:
            self.update_data_callback()

    def update_data_threaded(self, daemon=True):
        """
        子线程更新数据
        :param daemon: 是否为后台线程，True:主线程关闭则子线程也关闭;False:需要等待子线程结束，主线程才结束（设置为False是用来测试）
        :return:
        """
        if not self.data_thread:  # 子线程没有初始化
            self.data_thread = serial.threaded.ReaderThread(self.ser, JY901Protocol)  # 多线程串口协议
            self.data_thread.setDaemon(daemon)
            self.data_thread.start()
            self.data_thread.connect()
            self.data_thread.protocol.handle_packet = self._update_data

        elif self.data_thread and self.data_thread.protocol is None:  # 子线程已经初始化并运行过，但之前停止run
            self.data_thread.protocol = self.data_thread.protocol_factory()
            self.data_thread.setDaemon(daemon)
            self.data_thread.start()
            self.data_thread.connect()
            self.data_thread.protocol.handle_packet = self._update_data

    def update_data(self):
        """
        本线程循环更新数据
        :return:
        """
        buffer = bytearray()
        error = None
        while not rospy.is_shutdown():
            try:
                # read all that is there or wait for one byte (blocking)
                data = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException as e:
                error = e
                break
            else:
                if data:
                    # make a separated try-except for called used code
                    if 1:
                        buffer.extend(data)
                        while 0x55 in buffer:  # 如果当前已经收到buffer中第一个数据包的起始字节0x55
                            index = buffer.find(byte_value(0x55))  # 0x55的索引
                            if index == len(buffer) - 1:  # 如果0x55后面没有数据
                                break
                            if 0x50 & buffer[index + 1] != 0x50:  # 不满足数据包格式
                                buffer = buffer[index + 1:]  # 丢弃第一个0x55个其之前无用的数据
                            else:  # 满足数据包格式
                                buffer = buffer[index:]  # 丢弃当前buffer第一个数据包之前无用的数据
                                if len(buffer) > 11:  # 如果长度超过11，说明buffer中第一个数据包可以获取
                                    packet, buffer = buffer[0:11], buffer[11:]  # 获取数据包，

                                    # 校验和
                                    checksum = ctypes.c_ubyte(0)
                                    for b in packet[:-1]:
                                        checksum.value += b
                                    if checksum.value == packet[10]:  # 满足校验
                                        self._update_data(packet)
                                else:  # 否则继续获取数据
                                    break
                    else:
                        try:
                            buffer.extend(data)
                            while 0x55 in buffer:  # 如果当前已经收到buffer中第一个数据包的起始字节0x55
                                index = buffer.find(byte_value(0x55))  # 0x55的索引
                                if index == len(buffer) - 1:  # 如果0x55后面没有数据
                                    break
                                if 0x50 & buffer[index + 1] != 0x50:  # 不满足数据包格式
                                    buffer = buffer[index + 1:]  # 丢弃第一个0x55个其之前无用的数据
                                else:  # 满足数据包格式
                                    buffer = buffer[index:]  # 丢弃当前buffer第一个数据包之前无用的数据
                                    if len(buffer) > 11:  # 如果长度超过11，说明buffer中第一个数据包可以获取
                                        packet, buffer = buffer[0:11], buffer[11:]  # 获取数据包，
                                        # 校验和
                                        checksum = ctypes.c_ubyte(0)
                                        for b in packet[:-1]:
                                            checksum.value += b
                                        if checksum.value == packet[10]:  # 满足校验
                                            self._update_data(packet)

                                    else:  # 否则继续获取数据
                                        break
                        except Exception as e:
                            error = e
                            break
        if error:
            traceback.print_exc(error)


if __name__ == '__main__':
    def haha(ok, yes):
        print('Now it is callback following some para:', ok, yes)

    # myJY901 = JY901Base(verbose=True, callback=haha, argv=('ok', 'fasdf'))
    # myJY901.update_data_threaded(daemon=False)

    myJY901 = JY901Base(verbose='ros')
    myJY901.update_data()


