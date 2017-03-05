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

UNIT = 97  # 实验场景一块砖边长97cm

# 实验地图类型
maps = [
    {
        'comment': '直线 2*0'
    },
    {
        'comment': '直线向左pi/6 2*1'
    },
    {
        'comment': '直线向右pi/6 2*1'
    },
    {
        'comment': '直线向左pi/4 2*2'
    },
    {
        'comment': '直线向右pi/4 2*2'
    },
    {
        'comment': '六边形 3*3'
    }
]

def load_file(file):
    '''
    读取csv文件获取三种类型轨迹的点集，结构为二维数组[[x, y, z]]，需注意，由于输出的是定义好的three场景，
    z向北，x向西，y向上，且位置信息被初始的航向角影响，故此处需要进行坐标变换
    '''
    eo_points, wo_points, vo_points = (points.reshape(5000, 3) for points in np.loadtxt(file, dtype=np.float, delimiter=','))
    return eo_points, wo_points, vo_points

def run(file):
    eo_points, wo_points, vo_points = load_file(file)
    print(eo_points)

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
