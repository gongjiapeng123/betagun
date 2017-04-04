#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from matplotlib import pyplot as plt
import skimage.feature
from _utils import cul_exe_time

@cul_exe_time()
def read_img(filename):
    img = cv2.imread(filename)

    height, width = img.shape[:2]
    size = (int(width / 2), int(height / 2))  
    img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img2 = np.zeros_like(img)
    img2[:,:,0] = gray
    img2[:,:,1] = gray
    img2[:,:,2] = gray
    return img2, gray

@cul_exe_time()
def harris(img, gray):
    # find Harris corners
    gray = np.float32(gray)
    dst = cv2.cornerHarris(gray, 2, 3, 0.04)
    dst = cv2.dilate(dst, None)
    ret, dst = cv2.threshold(dst, 0.01 * dst.max(), 255, 0)
    dst = np.uint8(dst)

    # find centroids
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)

    # define the criteria to stop and refine the corners
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    corners = cv2.cornerSubPix(gray, np.float32(centroids), (5, 5), (-1, -1), criteria)

    # Now draw them
    res = np.hstack((centroids, corners))
    res = np.int0(res)
    # img[res[:, 1], res[:, 0]] = [0, 0, 255] 
    # img[res[:, 3], res[:, 2]] = [0, 255, 0]  
    
    # 绘制亚像素点精确后的角点
    for p in corners:
        cv2.circle(img, tuple(p), 3, (0, 0, 255), 1, cv2.LINE_AA)

    return img, gray, corners

@cul_exe_time()
def match(img1, gray1, corners1, img2, gray2, corners2):
    print(len(corners1), len(corners2))
    height, width = gray1.shape

    matches = []
    img = np.hstack((img1, img2))
    corners2_in_img = np.copy(corners2)
    corners2_in_img[:, 0] = corners2_in_img[:, 0] + width

    N = 2
    for p1 in corners1: 
        w1 = gray1[p1[1] - N: p1[1] + N + 1, p1[0] - N: p1[0] + N + 1]
        h, w = w1.shape
        w1 = np.pad(w1, ((0, 2 * N + 1 - h), (0, 2 * N + 1 - w)), mode='constant')
        correlations = []
        for p2 in corners2:
            w2 = gray2[p2[1] - N: p2[1] + N, p2[0] - N: p2[0] + N]
            h, w = w2.shape
            w2 = np.pad(w2, ((0, 2 * N + 1 - h), (0, 2 * N + 1 - w)), mode='constant')
            # correlation = skimage.feature.match_template(w2, w1, pad_input=False, mode='constant', constant_values=0)
            # correlations.append(correlation[0, 0])

            correlation = cv2.matchTemplate(w2, w1, method=cv2.TM_CCOEFF_NORMED)
            correlations.append(correlation[0, 0])
        
        max_correlation = max(correlations)
        if max_correlation > 0.5:
            correlations = np.array(correlations)
            match_p = corners2_in_img[correlations.argmax()]
            
            matches.append((p1, match_p))
            # print(max_correlation, p1, match_p)

    print(len(matches))
    for (p1, p2) in matches:
        cv2.circle(img, tuple(p1), 3, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.circle(img, tuple(p2), 3, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.line(img, tuple(p1), tuple(p2), (0, 255, 0), 1)
    
    return img

def translation_test():
    img1, gray1 = read_img('left.png')
    img1, gray1, corners1 = harris(img1, gray1)
    img2, gray2 = read_img('right.png')
    img2, gray2, corners2 = harris(img2, gray2)
    img = match(img1, gray1, corners1, img2, gray2, corners2)
    cv2.imwrite('result/harris_match_translation.png', img)
    print('*' * 20)

def rotate_test():
    img1, gray1 = read_img('s2.png')
    img1, gray1, corners1 = harris(img1, gray1)
    img2, gray2 = read_img('s3.png')
    img2, gray2, corners2 = harris(img2, gray2)
    img = match(img1, gray1, corners1, img2, gray2, corners2)
    cv2.imwrite('result/harris_match_rotate.png', img)
    print('*' * 20)

def scale_test():
    img1, gray1 = read_img('s1.png')
    img1, gray1, corners1 = harris(img1, gray1)
    img2, gray2 = read_img('s2.png')
    img2, gray2, corners2 = harris(img2, gray2)
    img = match(img1, gray1, corners1, img2, gray2, corners2)
    cv2.imwrite('result/harris_match_scale.png', img)
    print('*' * 20)

translation_test()
rotate_test()
scale_test()

# plt.imshow(img)
# plt.show()

