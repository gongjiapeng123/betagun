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
# import seaborn

MAX_POINTS = 2000
# MAX_POINTS = 5000
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

test_line_real = np.array([
    [-1.00, 20.00],
    [-1.00, 40.00],
    [-1.00, 60.00],
    [-0.50, 80.00],
    [0.50, 100.00],
    [1.00, 120.00],
    [1.00, 140.00],
    [1.50, 160.00],
    [1.00, 180.00],
    [-1.00, 200.00],
])

test_line_measure = np.array([
    [-1.28, 20.37],
    [-1.30, 40.70],
    [-1.00, 61.58],
    [-0.50, 79.84],
    [0.31, 101.75],
    [0.84, 120.63],
    [0.81, 141.42],
    [1.19, 161.53],
    [0.14, 181.04],
    [-0.38, 201.97],
])

test_curve_real = np.array([
    [-0.50, 21.00],
    [1.20, 61.00],
    [7.00, 93.50],
    [17.50, 136.00],
    [30.80, 181.00],
    [10.50, 252.00],
    [-7.00, 356.00],
    [-13.00, 401.00],
    [-5.00, 452.00],
    [-3.00, 501.00],
])

test_curve_measure = np.array([
    [-0.45, 21.16],
    [1.94, 62.08],
    [7.85, 94.41],
    [18.00, 137.63],
    [32.02, 183.25],
    [11.74, 255.26],
    [-7.64, 360.81],
    [-14.13, 405.78],
    [-5.06, 457.83],
    [-3.51, 509.92],
])

def handle_points(points):
    '''
    将读取的数据进行处理
    '''
    def reshape(ps):
        return ps.reshape(MAX_POINTS, 3)

    def cut_2d(ps):  # 获取three.js的x, z轴，即此处的x, y
        return ps[:, np.array([True, False, True])]

    def cut_zero(ps):  # 去除未绘制的点（去除向量为0的点）
        return ps[~np.all(ps==0, axis=1)]  # or ps[~(ps==0).all(1)]

    ps = reshape(points)
    ps = cut_2d(ps)
    ps = cut_zero(ps)
    res = np.vstack(([[0, 0]], ps))  # 添加原点
    return res

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
    plt.grid(True, which='both')
    x = points[:, 0]
    y = points[:, 1]
    # print(x, y)
    plt.plot(x, y)

    # real = plt.scatter(test_line_real[:, 0], test_line_real[:, 1], s=20, c='red')
    # measure = plt.scatter(test_line_measure[:, 0], test_line_measure[:, 1], s=20, c='green')
    # plt.xlim(50, -50)
    # plt.ylim(-50, 250)

    # real = plt.scatter(test_curve_real[:, 0], test_curve_real[:, 1], s=20, c='red')
    # measure = plt.scatter(test_curve_measure[:, 0], test_curve_measure[:, 1], s=20, c='green')
    # plt.xlim(50, -50)
    # plt.ylim(-100, 600)

    real = plt.scatter([-1], [-2], s=15, c='r')
    measure = plt.scatter([3.25], [-4.61], s=15, c='b')
    plt.xlim(100, -200)
    plt.ylim(-100, 300)

    plt.xlabel('x(cm)')
    plt.ylabel('y(cm)')
    
    plt.legend((real, measure), (u'实际测量点', u'算法定位点'), loc=4)
    # plt.legend((real, measure), (u'实际测量终点', u'算法定位终点'), loc=4)
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
