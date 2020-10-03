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
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


class LineTracker():

    def __init__(self):
        self._cv_bridge = CvBridge()
        self._captured_img = None
        self._point_of_line_center = None
        self._roi = None
        self._expand_range = 50

        self._pub_binary_img = rospy.Publisher("binary", Image, queue_size=1)
        self._pub_pbject_img = rospy.Publisher("tracking_line", Image, queue_size=1)
        self._pub_test_img = rospy.Publisher("test", Image, queue_size=1)
        self._pub_cmdvel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self._sub_img = rospy.Subscriber("/csi_cam_0/image_raw", Image, self._img_callback)

        rospy.wait_for_service("/motor_on")
        rospy.wait_for_service("/motor_off")
        rospy.on_shutdown(rospy.ServiceProxy("/motor_off", Trigger).call)
        rospy.ServiceProxy("/motor_on", Trigger).call()

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

    def _rotation_velocity(self):
        VELOCITY = 0.25 * math.pi
        if self._point_of_line_center is None:
            return 0.0
        half_width = self._captured_img.shape[1] / 2.0
        pos_x_rate = (half_width - self._point_of_line_center[0]) / half_width
        rot_vel = pos_x_rate * VELOCITY
        return rot_vel

    def _extract_line_in_binary(self, cv_img):
        if cv_img is None:
            print("none")
            return None
        min_value = 0;
        max_value = 100;

        mono_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        # 画面の下半分のみを使用するため，上半分を白くする
        mono_img[:mono_img.shape[0]/2, :] = 255
        binary = cv2.inRange(mono_img, min_value, max_value)

        return binary

    def _draw_tracking_point(self, input_img, tracking_point):
        return cv2.circle(input_img, tracking_point, 15, (255, 0, 0), thickness=-1)

    def _extract_line_center(self, line_img, contour):
        void_img = np.zeros(line_img.shape, dtype=np.uint8)
        contour_line_img = cv2.drawContours(void_img, [contour], 0, (0, 255, 0), 1)
        binary_img = cv2.cvtColor(contour_line_img, cv2.COLOR_BGR2GRAY)
        scan_height = binary_img.shape[0] - 10
        scan_line = binary_img[scan_height]
        found_edge = np.flatnonzero(scan_line)
        try:
            start_point = found_edge[0]
            end_point = found_edge[-1]
            mean = (start_point + end_point) / 2

            line_center_point = (mean, scan_height)
            return line_center_point
        except IndexError:
            return None

    def _draw_tracking_point(self, tracking_img):
        return cv2.circle(tracking_img, self._point_of_line_center, 5, (0, 255, 0), -1)

    
    def _preprocessing(self, binary_img):
        kernel = np.ones((7,7), np.uint8)
        binary_img = cv2.morphologyEx(binary_img, cv2.MORPH_CLOSE, kernel)
        return binary_img

    def _update_roi(self, points):
        try:
            x, y, w, h = cv2.boundingRect(points)
        except:
            x, y, w, h = (0, 0, self._captured_img.shape[1], self._captured_img.shape[0])
        ex = self._expand_range
        roi_x = x-ex
        roi_y = y-ex
        roi_w = w+ex*2
        roi_h = h+ex*2

        if roi_x < 0:
            roi_x = 0
        if roi_y < 0:
            roi_y = 0
        if roi_x + roi_w > self._captured_img.shape[1]:
            roi_w = self._captured_img.shape[1] - roi_x
        if roi_y + roi_h > self._captured_img.shape[0]:
            roi_h = self._captured_img.shape[0] - roi_y

        self._roi = (roi_x, roi_y, roi_w, roi_h)

    def _draw_roi(self, img):
        x, y, w, h = self._roi
        roi_img = copy.deepcopy(img)
        roi_img = cv2.rectangle(roi_img, (x, y), (x+w, y+h), (255,0,0), 4)
        return roi_img

    def _extract_biggest_contour(self, binary_img):
        biggest_contour_index = False
        biggest_contour_area = 0
        if self._roi is None:
            x, y, w, h = (0, 0, binary_img.shape[1], binary_img.shape[1])
        else:
            x, y, w, h = self._roi
        trimed_img = binary_img[y:y+h, x:x+w]
        contours, hierarchy = cv2.findContours(
            trimed_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE, offset=(x,y))

        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if biggest_contour_area < area:
                biggest_contour_area = area
                biggest_contour_index = i

        if biggest_contour_index is False:
            return False
        else:
            return contours[biggest_contour_index]

    def img_processing(self):
        line_img = copy.deepcopy(self._captured_img)
        tracking_img = copy.deepcopy(self._captured_img)
        binary_line_img = self._extract_line_in_binary(self._captured_img)

        if binary_line_img is not None:
            # モルフォロジー
            binary_line_img = self._preprocessing(binary_line_img)
            # ROI中から輪郭を検出
            # 画面中に最も大きく映るラインとしてトレースする
            biggest_contour = self._extract_biggest_contour(binary_line_img)
            # 大きく映ったラインの輪郭点の位置からROIを更新
            self._update_roi(biggest_contour)
            roi_img = self._draw_roi(binary_line_img)

            if biggest_contour is not False:
                line_img = cv2.drawContours(line_img, [biggest_contour], 0, (0, 255, 0), 5)
                self._point_of_line_center = self._extract_line_center(line_img, biggest_contour)
                if self._point_of_line_center is not None:
                    tracking_img = self._draw_tracking_point(tracking_img)
                    print(self._point_of_line_center)
                    print(self._roi)

            self._monitor(line_img, self._pub_pbject_img)
            self._monitor(binary_line_img, self._pub_binary_img)
            self._monitor(roi_img, self._pub_test_img)
            

    def control(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.05

        cmd_vel.angular.z = self._rotation_velocity()

        self._pub_cmdvel.publish(cmd_vel)


if __name__ == '__main__':
    rospy.init_node('line_tracing')
    rospy.sleep(3.)
    lt = LineTracker()

    rate = rospy.Rate(60)
    rate.sleep()
    while not rospy.is_shutdown():
        lt.img_processing()
        lt.control()
        rate.sleep()
