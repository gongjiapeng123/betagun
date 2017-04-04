#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from _utils import cul_exe_time

surf = cv2.xfeatures2d.SURF_create(500)
# FLANN parameters
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
search_params = dict(checks=50)   # or pass empty dictionary
flann = cv2.FlannBasedMatcher(index_params, search_params)

ransac = False

@cul_exe_time()
def read_img(filename):
    img = cv2.imread(filename)

    height, width = img.shape[:2]
    size = (int(width / 2), int(height / 2))
    img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return gray

@cul_exe_time()
def detect(gray):
    kp, des = surf.detectAndCompute(gray, None)
    return kp, des

@cul_exe_time()
def match(img1, kp1, des1, img2, kp2, des2):
    matches = flann.knnMatch(des1, des2, k=2)

    # Need to draw only good matches, so create a mask
    matchesMask = [[0, 0] for i in list(range(len(matches)))]
    good = []
    good_mathes = []
    # ratio test as per Lowe's paper
    for i, (m, n) in enumerate(matches):
        if m.distance < 0.7 * n.distance:
            matchesMask[i] = [1, 0]
            good.append(m)
            good_mathes.append((m, n))
    print('matches: {} good: {}'.format(len(matches), len(good)))

    result_image = None
    if ransac:
        left_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        right_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        M, mask = cv2.findHomography(left_pts, right_pts, cv2.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()

        h, w = img1.shape
        pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1],
                          [w - 1, 0]]).reshape(-1, 1, 2)
        dst = cv2.perspectiveTransform(pts, M)

        draw_params = dict(
            matchColor=(0, 255, 0),
            singlePointColor=None,
            matchesMask=matchesMask,
            flags=2
        )

        result_image = cv2.drawMatches(
            img1,
            kp1,
            cv2.polylines(img2, [np.int32(dst)], True, 255, 3, cv2.LINE_AA),
            kp2,
            good,
            None,
            **draw_params
        )
    else:

        draw_params = dict(
            matchColor=(0, 255, 0),
            singlePointColor=(0, 0, 255),
            matchesMask=matchesMask,
            flags=0
        )
        result_image = cv2.drawMatchesKnn(
            img1,
            kp1,
            img2,
            kp2,
            matches,
            None,
            **draw_params
        )
    # cv2.imshow('res', result_image)
    return result_image

def translation_test():
    img1 = read_img('left.png')
    kp1, des1 = detect(img1)
    img2 = read_img('right.png')
    kp2, des2 = detect(img2)
    img = match(img1, kp1, des1, img2, kp2, des2)
    cv2.imwrite('result/surf_match_translation.png', img)
    print('*' * 20)

def rotate_test():
    img1 = read_img('s2.png')
    kp1, des1 = detect(img1)
    img2 = read_img('s3.png')
    kp2, des2 = detect(img2)
    img = match(img1, kp1, des1, img2, kp2, des2)
    cv2.imwrite('result/surf_match_rotate.png', img)
    print('*' * 20)

def scale_test():
    img1 = read_img('s1.png')
    kp1, des1 = detect(img1)
    img2 = read_img('s2.png')
    kp2, des2 = detect(img2)
    img = match(img1, kp1, des1, img2, kp2, des2)
    cv2.imwrite('result/surf_match_scale.png', img)
    print('*' * 20)

translation_test()
rotate_test()
scale_test()

