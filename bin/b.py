#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''
betagun 启动脚本
'''

from __future__ import print_function
from __future__ import unicode_literals
from __future__ import division
from __future__ import absolute_import

import argparse
import textwrap
import os
import sys
import time
import subprocess
import signal
import pickle

DIR = os.path.dirname(os.path.abspath(__file__))  # 脚本所在目录
BETAGUN_DIR = os.path.abspath(os.path.join(DIR, '..'))
TCP_SERVER_DIR = os.path.abspath(os.path.join(BETAGUN_DIR, 'tcp-server'))
WEB_SERVER_DIR = os.path.abspath(os.path.join(BETAGUN_DIR, 'web-server'))
ROS_BAGS_DIR = os.path.abspath(os.path.join(BETAGUN_DIR, 'bags'))
ROS_BAGS_PROCESS_PICKLE = os.path.abspath(os.path.join(ROS_BAGS_DIR, 'p'))
ROS_SRC_DIR = os.path.abspath(os.path.join(BETAGUN_DIR, 'src'))

def terminate_process_and_children(p):
    def _terminate_process_and_children(pid):
        ps_command = subprocess.Popen('ps -o pid --ppid {} --noheaders'.format(pid), shell=True, stdout=subprocess.PIPE)
        ps_output = ps_command.stdout.read()
        retcode = ps_command.wait()
        for pid_str in ps_output.split('\n')[:-1]:
            _terminate_process_and_children(int(pid_str))
        os.kill(pid, signal.SIGINT)
    _terminate_process_and_children(p.pid)
    p.terminate()

def start_tcp_server():
    print('start tcp server')
    subprocess.Popen('node {} > /dev/null 2>&1'.format(os.path.abspath(os.path.join(TCP_SERVER_DIR, 'start.js'))), shell=True)
    time.sleep(3)

def stop_tcp_server():
    print('stop tcp server')
    os.system("ps aux| grep 'tcp-server/start.js' | awk '{print $2}' | xargs kill > /dev/null 2>&1")
    time.sleep(3)

def start_web_server():
    print('start web server')
    subprocess.Popen('node {} > /dev/null 2>&1'.format(os.path.abspath(os.path.join(WEB_SERVER_DIR, 'dev.js'))), shell=True)
    time.sleep(3)

def stop_web_server():
    print('stop web server')
    os.system("ps aux| grep 'web-server/dev.js' | awk '{print $2}' | xargs kill > /dev/null 2>&1")
    time.sleep(3)

def start_ros(imu0_relative, vo, cam, cam2, bag):
    print('start ros')
    subprocess.Popen('roslaunch betagun odom_ekf.launch imu0_relative:={} vo:={} cam:={} cam2:={} > /dev/null 2>&1'.format(
        'true' if imu0_relative else 'false',
        'true' if vo else 'false',
        'true' if cam else 'false',
        'true' if cam2 else 'false'
    ), shell=True)
    p = {}
    if bag:
        p = subprocess.Popen('rosbag record -a > /dev/null 2>&1', shell=True, cwd=ROS_BAGS_DIR)
    with open(ROS_BAGS_PROCESS_PICKLE, b'wb') as f:
        pickle.dump(p, f, protocol=2)
    time.sleep(3)

def stop_ros():
    print('stop ros')
    p = {}
    if os.path.exists(ROS_BAGS_PROCESS_PICKLE):
        with open(ROS_BAGS_PROCESS_PICKLE, b'rb') as f:
            p = pickle.load(f)
    if isinstance(p, subprocess.Popen):
        terminate_process_and_children(p)
    os.system("ps aux| grep 'odom_ekf.launch' | awk '{print $2}' | xargs kill > /dev/null 2>&1")
    time.sleep(3)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=textwrap.dedent(u'''\
                betagun 启动命令
                --------------------------------
                '''),
        fromfile_prefix_chars='@'
    )

    parser.add_argument(
        'action',
        action='store',
        type=str,
        choices=['start', 'stop', 'restart'],
        help=u'操作',
    )
    parser.add_argument(
        '-t',
        '--tcp',
        action='store_true',
        help=u'仅对tcp action',
    )
    parser.add_argument(
        '-w',
        '--web',
        action='store_true',
        help=u'仅对web action',
    )
    parser.add_argument(
        '-r',
        '--ros',
        action='store_true',
        help=u'仅对ros action',
    )
    parser.add_argument(
        '-c',
        '--cam',
        action='store_true',
        help=u'是否打开双目摄像头',
    )
    parser.add_argument(
        '-c2',
        '--cam2',
        action='store_true',
        help=u'是否打开设备号只有一个的双目摄像头',
    )
    parser.add_argument(
        '-v',
        '--vo',
        action='store_true',
        help=u'是否打开vo',
    )
    parser.add_argument(
        '-i',
        '--imu0_relative',
        action='store_true',
        help=u'是否imu0_relative',
    )
    parser.add_argument(
        '-b',
        '--bag',
        action='store_true',
        help=u'是否rosbag recore',
    )

    ns = parser.parse_args()
    print(ns)

    if ns.tcp or ns.web or ns.ros:
        if ns.tcp:
            if ns.action == 'start':
                start_tcp_server()
            elif ns.action == 'restart':
                stop_tcp_server()
                start_tcp_server()
            else:
                stop_tcp_server()

        if ns.web:
            if ns.action == 'start':
                start_web_server()
            elif ns.action == 'restart':
                stop_web_server()
                start_web_server()
            else:
                stop_web_server()

        if ns.ros:
            if ns.action == 'start':
                start_ros(ns.imu0_relative, ns.vo, ns.cam, ns.cam2, ns.bag)
            elif ns.action == 'restart':
                stop_ros()
                start_ros(ns.imu0_relative, ns.vo, ns.cam, ns.cam2, ns.bag)
            else:
                stop_ros()

    else:
        if ns.action == 'start':
            start_tcp_server()
            start_web_server()
            start_ros(ns.imu0_relative, ns.vo, ns.cam, ns.cam2, ns.bag)
        elif ns.action == 'restart':
            stop_ros()
            stop_web_server()
            stop_tcp_server()

            start_tcp_server()
            start_web_server()
            start_ros(ns.imu0_relative, ns.vo, ns.cam, ns.cam2, ns.bag)
        else:
            stop_ros()
            stop_web_server()
            stop_tcp_server()


