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


class LineFollower():
    def __init__(self):
        self._cv_bridge = CvBridge()
        self._captured_img = None
        self._point_of_line_center = None
        self._roi = None
        # ROI更新時の拡張サイズ指定（単位: pixel）
        # 値が大きいほど広範囲から検出を行う
        self._expand_range = 50

        self._pub_result_img = rospy.Publisher("/line_forrower_img", Image, queue_size=1)
        self._pub_cmdvel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self._sub_img = rospy.Subscriber("/panorama_img", Image, self._img_callback)

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
        # Z軸回転速度の係数
        VELOCITY = 0.75 * math.pi
        if self._point_of_line_center is None:
            return 0.0
        
        # 画面の中央にライン中央が来るようにする
        half_width = self._captured_img.shape[1] / 2.0
        pos_x_rate = (half_width - self._point_of_line_center[0]) / half_width
        rot_vel = pos_x_rate * VELOCITY
        return rot_vel


    def _extract_line_in_binary(self, cv_img):
        if cv_img is None:
            print("none")
            return None
        # しきい値処理用の輝度値
        min_value = 0;
        max_value = 100;
        mono_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        # 画面上部にはライン以外の情報が多いことから
        # 画面下部のみを使用するため，上半分を白くする
        mono_img[:mono_img.shape[0]/2, :] = 255

        # しきい値処理
        binary = cv2.inRange(mono_img, min_value, max_value)
        return binary


    def _extract_line_center(self, line_img, contour):
        void_img = np.zeros(line_img.shape, dtype=np.uint8)
        contour_img = cv2.drawContours(void_img, [contour], 0, (0, 255, 0), 1)
        binary_img = cv2.cvtColor(contour_img, cv2.COLOR_BGR2GRAY)
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


    def _preprocessing(self, binary_img):
        # ノイズ除去のためモルフォロジーを行う
        kernel = np.ones((7,7), np.uint8)
        binary_img = cv2.morphologyEx(binary_img, cv2.MORPH_CLOSE, kernel)
        return binary_img


    def _update_roi(self, points):
        # 輪郭点をバウンディングボックスで囲む
        try:
            x, y, w, h = cv2.boundingRect(points)
        except:
            x, y, w, h = (0, 0, self._captured_img.shape[1], self._captured_img.shape[0])

        # バウンディングボックスの点+self._expand_rangeの範囲を新たなROIとする
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


    def _draw_result_img(self, img, contour):
        # ROIの描画（青線）
        x, y, w, h = self._roi
        roi_img = copy.deepcopy(img)
        roi_img = cv2.rectangle(roi_img, (x, y), (x+w, y+h), (255,0,0), 4)

        # ラインの輪郭の描画（緑線）
        line_img = cv2.drawContours(roi_img, [contour], 0, (0, 255, 0), 5)

        # 追跡点の描画（赤円）
        result_img = cv2.circle(line_img, self._point_of_line_center, 15, (0, 0, 255), thickness=-1)
        print(self._point_of_line_center)

        return result_img


    def _extract_biggest_contour(self, binary_img):
        biggest_contour_index = False
        biggest_contour_area = 0

        # 最初は画面全体から検出する
        if self._roi is None:
            x, y, w, h = (0, 0, binary_img.shape[1], binary_img.shape[1])
        else:
            x, y, w, h = self._roi
        trimed_img = binary_img[y:y+h, x:x+w]

        # 輪郭検出
        contours, hierarchy = cv2.findContours(
            trimed_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE, offset=(x,y))

        # 最大面積の輪郭のインデクスを保存
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
        org_img = copy.deepcopy(self._captured_img)
        input_img = copy.deepcopy(org_img)
        result_img = None
        if input_img is not None:
            binary_line_img = self._extract_line_in_binary(input_img)
            # 画像の前処理
            binary_line_img = self._preprocessing(binary_line_img)

            # ROI（初期値は画面全体）中から輪郭を検出
            # 画面中に最も大きく映るラインを追跡する
            biggest_contour = self._extract_biggest_contour(binary_line_img)

            # 最大ラインの位置からROIを更新
            self._update_roi(biggest_contour)

            if biggest_contour is not False:
                # ラインの中央を検出
                self._point_of_line_center = self._extract_line_center(org_img, biggest_contour)
                if self._point_of_line_center is not None:
                    result_img = self._draw_result_img(org_img, biggest_contour)
                    # 画像処理結果をパブリッシュ
                    self._monitor(result_img, self._pub_result_img)
                

    def control(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2
        cmd_vel.angular.z = self._rotation_velocity()

        self._pub_cmdvel.publish(cmd_vel)


if __name__ == '__main__':
    rospy.init_node('line_tracing')
    lf = LineFollower()

    rate = rospy.Rate(60)
    rate.sleep()
    while not rospy.is_shutdown():
        lf.img_processing()
        lf.control()
        rate.sleep()
