#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''
betagun 分析误差 场景单位cm
'''

from __future__ import print_function
from __future__ import unicode_literals
from __future__ import division
from __future__ import absolute_import

import argparse
import textwrap
import numpy as np
import matplotlib.pyplot as plt
import seaborn

UNIT = 97  # 实验场景一块砖边长97cm

# 实验地图类型
maps = [
    {
        'id': 'a',
        'comment': '直线 2*0'
    },
    {
        'id': 'b',
        'comment': '直线向左pi/6 2*1'
    },
    {
        'id': 'c',
        'comment': '直线向右pi/6 2*1'
    },
    {
        'id': 'd',
        'comment': '直线向左pi/4 2*2'
    },
    {
        'id': 'e',
        'comment': '直线向右pi/4 2*2'
    },
    {
        'id': 'f',
        'comment': '六边形 3*3'
    }
]

def handle_points(points):
    '''
    将读取的数据进行处理
    '''
    def reshape(ps):
        return ps.reshape(5000, 3)

    def cut_2d(ps):  # 获取three.js的x, z轴，即此处的x, y
        return ps[:, np.array([True, False, True])]

    def cut_zero(ps):  # 去除未绘制的点（去除向量为0的点）
        return ps[~np.all(ps==0, axis=1)]  # or ps[~(ps==0).all(1)]

    ps = reshape(points)
    ps = cut_2d(ps)
    ps = cut_zero(ps)
    ps[:, 0] = ps[:, 0] * (-1)
    return ps

def load_file(file):
    '''
    读取csv文件获取三种类型轨迹的点集，结构为二维数组[[x, y, z]]，需注意，由于输出的是定义好的three场景，
    z向北，x向西，y向上，且位置信息被初始的航向角影响，故此处需要进行坐标变换
    '''
    eo_points, wo_points, vo_points = (handle_points(points) for points in np.loadtxt(file, dtype=np.float, delimiter=','))
    return eo_points, wo_points, vo_points

def show_figure(points):
    '''
    显示轨迹
    '''
    x = points[:, 0]
    y = points[:, 1]
    # print(x, y)
    plt.plot(x, y)

    plt.grid(True, which='both')
    plt.axhline(0, color='white')
    plt.axvline(0, color='white')
    plt.xlim(-300, 300)
    plt.ylim(-300, 300)
    plt.xlabel('x')
    plt.ylabel('y')
    seaborn.set(style='ticks')
    seaborn.despine(ax=plt, offset=0)
    plt.show()

def run(file):
    eo_points, wo_points, vo_points = load_file(file)
    show_figure(eo_points)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=textwrap.dedent(u'''\
                betagun 分析误差
                --------------------------------
                '''),
        fromfile_prefix_chars='@'
    )

    parser.add_argument(
        'file',
        action='store',
        type=str,
        help=u'轨迹数据文件',
    )
    parser.add_argument(
        '-v', 
        '--vo', 
        action='store_true',
        help=u'是否打开vo',
    )

    ns = parser.parse_args()
    print(ns)
    
    run(ns.file)
