#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from __future__ import unicode_literals
from __future__ import division
from __future__ import absolute_import

import time
import cv2
import numpy as np


def cul_exe_time(label='', trace=True):
    '''
    一个计时函数装饰器
    函数装饰器，此处为了在封闭作用域内保存参数，并返回嵌套的实际的函数装饰器类
    '''
    class CulExeTimer(object):
        '''
        包装器类（函数装饰器类）
        '''

        def __init__(self, func):
            '''
            构造函数在 @decorator 时调用，使得func=CulExeTime(label,trace)(func)
            => 形成了封闭作用域并保持了label和trace的值 且 func=CulExeTimer(func)
            构造时存储原func函数，并定义所需要的额外属性
            '''
            self.func = func
            self.thistime = 0
            self.totaltime = 0

        def __call__(self, *args, **kargs):
            '''
            重载__call__操作符函数使得CulExeTimer的实例（func）在外部执行
            func(*args,**kargs)时调用此方法，从而可以使用实例中的属性进行比
            原func函数更丰富的操作
            '''
            start = time.time()
            rs = self.func(*args, **kargs)
            self.thistime = time.time() - start
            self.totaltime += self.thistime
            if trace:
                print('{0} {1}:Elapsed: {2:.5f}ms, Total: {3:.5f}ms'.format(
                    label, self.func.__name__, self.thistime * 1000, self.totaltime * 1000))
            return rs

        def __get__(self, instance, owner):
            '''
            利用描述符，它可以保存装饰器的状态（self）及最初的类实例
            （instance），即调用instance.f(...)时，执行f._get__(self,instance,owner),
            self为f（已经变成CulExeTimer类的实例的f），instance即为最初的类实例
            owner为主体类
            '''
            #保存self和instance，并触发self.__call__，将instance有效传递
            return lambda *args, **kargs: self(instance, *args, **kargs)
    return CulExeTimer

def separate_image():
    image = cv2.imread('test.jpg')
    left, right = np.hsplit(image, 2)
    cv2.imwrite('left.png', left)
    cv2.imwrite('rigth.png', right)

def resize_image():
    img = cv2.imread('right.png')
    height, width = img.shape[:2]
    size = (int(width / 2), int(height / 2))  
    shrink = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
    cv2.imwrite('right2.png', shrink)
