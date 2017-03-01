#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''
监听ros发布的主题，连接61615端口并发送计算结果
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

DIR = os.path.dirname(os.path.abspath(__file__))  # 脚本所在目录
BETAGUN_DIR = os.path.abspath(os.path.join(DIR, '..'))
TCP_SERVER_DIR = os.path.abspath(os.path.join(BETAGUN_DIR, 'tcp-server'))
WEB_SERVER_DIR = os.path.abspath(os.path.join(BETAGUN_DIR, 'web-server'))
ROS_SRC_DIR = os.path.abspath(os.path.join(BETAGUN_DIR, 'src'))

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

def start_ros(vo):
    print('start ros')
    subprocess.Popen('roslaunch betagun odom_ekf.launch vo:={} > /dev/null 2>&1'.format('true' if vo else 'false'), shell=True)
    time.sleep(3)

def stop_ros():
    print('stop ros')
    os.system("ps aux| grep 'odom_ekf.launch' | awk '{print $2}' | xargs kill > /dev/null 2>&1")
    time.sleep(3)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=textwrap.dedent(u'''\
                betagun 传输info给tcpserver
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
        '-v', 
        '--vo', 
        action='store_true',
        help=u'是否打开vo',
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
                start_ros(ns.vo)
            elif ns.action == 'restart':
                stop_ros()
                start_ros(ns.vo)
            else:
                stop_ros()

    else:
        if ns.action == 'start':
            start_tcp_server()
            start_web_server()
            start_ros(ns.vo)
        elif ns.action == 'restart':
            stop_ros()
            stop_web_server()
            stop_tcp_server()

            start_tcp_server()
            start_web_server()
            start_ros(ns.vo)
        else:
            stop_ros()
            stop_web_server()
            stop_tcp_server()


