#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from matplotlib import pyplot as plt
from _utils import cul_exe_time


@cul_exe_time()
def read_image():
    img = cv2.imread('left.png')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img, gray

@cul_exe_time()
def detect():
    sift = cv2.xfeatures2d.SURF_create(400)
    kp = sift.detect(gray, None)
    print(len(kp))
    return kp

img, gray = read_image()
kp = detect()

cv2.drawKeypoints(gray, kp, img, flags=0)

plt.imshow(img)
plt.show()
# cv2.imwrite('sift_keypoints.png', img)