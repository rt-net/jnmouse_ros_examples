#!/usr/bin/env python
# coding: utf-8

# Copyright 2020 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import cv2
import math
import numpy as np
import copy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Mouse:
    x = 0
    y = 0
    point = [0, 0]

    def __init__(self):
        self.point = [self.x, self.y]

def move_cursor(img, key, mouse):
    if key == ord('h'):
        mouse.x -= 1
    elif key == ord('j'):
        mouse.y += 1
    elif key == ord('k'):
        mouse.y -= 1
    elif key == ord('l'):
        mouse.x += 1

    if key == ord('H'):
        mouse.x -= 3
    elif key == ord('J'):
        mouse.y += 3
    elif key == ord('K'):
        mouse.y -= 3
    elif key == ord('L'):
        mouse.x += 3

    if mouse.x < 0:
        mouse.x = 0
    elif mouse.y < 0:
        mouse.y = 0

    cv2.line(img, (0, mouse.y), (img.shape[1],mouse.y), (255, 0, 0))
    cv2.line(img, (mouse.x, 0), (mouse.x,img.shape[0]), (0, 255, 0))


def append_image_points(userdata):
    mouse = userdata['mouse']
    image_points = userdata['image_points']
    image_point = (mouse.x, mouse.y)
    image_points.append([image_point])
    print('image point is: {}'.format(image_points))


def mouse_callback(event, x, y, flags, userdata):
    mouse = userdata['mouse']
    image_points = userdata['image_points']

    if event == cv2.EVENT_LBUTTONDOWN:
        mouse.x = x
        mouse.y = y
        mouse.point = (x, y)

    if event == cv2.EVENT_MBUTTONDOWN:
        append_image_points(image_points, mouse)
    if event == cv2.EVENT_RBUTTONDOWN:
        try:
            image_points.pop()
        except:
            print('image_points is None')
        print('image point is: {}'.format(image_points))


class SetImagePoints():
    def __init__(self):
        self._cv_bridge = CvBridge()
        self._captured_img = None
        self._point_of_line_center = None
        self._roi = None
        # ROI更新時の拡張サイズ指定（単位: pixel）
        # 値が大きいほど広範囲から検出を行う
        self._expand_range = 50

        self._pub_result_img = rospy.Publisher("/line_forrower_img", Image, queue_size=1)

        self._sub_img = rospy.Subscriber("/stereo/left/image_raw", Image, self._img_callback)


    def _img_callback(self, img):
        try:
            self._captured_img = self._cv_bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)


    def _monitor(self, img, pub):
        if img.ndim == 2:
            pub.publish(self._cv_bridge.cv2_to_imgmsg(img, "mono8"))
        elif img.ndim == 3:
            pub.publish(self._cv_bridge.cv2_to_imgmsg(img, "bgr8"))
        else:
            pass


    def img_processing(self, userdata):
        input_img = copy.deepcopy(self._captured_img)
        result_img = input_img

        scale = 1080/input_img.shape[0]
        scale = 2

        if input_img is not None:
            input_img = cv2.resize(input_img, None, fx=scale, fy=scale)
            while True:
                pointed_img = copy.deepcopy(input_img)
                for image_point in userdata['image_points']:
                    x = image_point[0][0]
                    y = image_point[0][1]
                    cv2.circle(pointed_img, (x, y), 3, (0, 0, 255), 1)
                k = cv2.waitKey(30)
                move_cursor(pointed_img, k, userdata['mouse'])
                cv2.imshow('img', pointed_img)

                if k == ord('p'):
                    append_image_points(userdata)
                elif k == ord('q') or k == 27:
                    rospy.signal_shutdown("finished")
                    break

            # 画像処理結果をパブリッシュ
            self._monitor(result_img, self._pub_result_img)

#        # 画像を拡大する
#        scale = 1080/input_img.shape[0]
#        scale = 1
#        input_img = cv2.resize(input_img, None, fx=scale, fy=scale)
#        userdata = {'mouse': mouse, 'image_points': image_points)
#        cv2.namedWindow('img', cv2.WINDOW_NOEMAL)
#        cv2.setMouseCallback('img', mouse_callback, userdata)

if __name__ == '__main__':
    rospy.init_node('set_image_points')
    si = SetImagePoints()

    mouse=Mouse()
    image_points=[]
    userdata = {'mouse': mouse, 'image_points': image_points}
    cv2.namedWindow('img', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('img', mouse_callback, userdata)

    rate = rospy.Rate(60)
    rate.sleep()
    while not rospy.is_shutdown():
        si.img_processing(userdata)
        rate.sleep()
