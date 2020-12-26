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
import time 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


class CourceLine():
    def __init__(self, left=0, right=0):
        self.left = left
        self.right = right
        self.center = (left + right)/2
        self._width = right - left
    
    # CourceLineオブジェクト同士を比較演算子で評価可能に
    def __lt__(self, other):
        return self._width < other._width


class LineFollower():
    def __init__(self):
        self._cv_bridge = CvBridge()
        self._captured_img = None
        self._point_of_line_center = None
        self._widest_line = CourceLine()
        self._roi = None

        # ROI更新時の拡張サイズ指定（単位: pixel）
        # 値が大きいほど広範囲から検出を行う
        self._expand_range = 50

        # パブリッシャ
        self._pub_result_img = rospy.Publisher("/line_forrower_img", Image, queue_size=1)
        self._pub_binary_img = rospy.Publisher("/binary_line_img", Image, queue_size=1)
        self._pub_cmdvel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # サブスクライバ
        self._sub_img = rospy.Subscriber("/panorama_img", Image, self._img_callback)

        rospy.wait_for_service("/motor_on")
        rospy.wait_for_service("/motor_off")
        rospy.on_shutdown(rospy.ServiceProxy("/motor_off", Trigger).call)
        rospy.ServiceProxy("/motor_on", Trigger).call()

        # CUDA処理用
        self._gpu_img_src = cv2.cuda_GpuMat()
        self._gpu_img_dst = cv2.cuda_GpuMat()
        self._gpu_img_tmp = cv2.cuda_GpuMat()


    # サブスクライブ用のコールバック関数
    # _cv_bridge.imgmsg_to_cv2()の返り値はnumpy.array型（OpenCVにおける画像形式）
    def _img_callback(self, img):
        try:
            self._captured_img = self._cv_bridge.imgmsg_to_cv2(img, "bgr8")
            if self._captured_img.any() == None:
                self._sub_img = rospy.subscriber("/panorama_img", image, self._img_callback)
        except CvBridgeError as e:
            rospy.logerr(e)


    # パブリッシュ用
    def _monitor(self, img, pub):
        if img.ndim == 2:
            pub.publish(self._cv_bridge.cv2_to_imgmsg(img, "mono8"))
        elif img.ndim == 3:
            pub.publish(self._cv_bridge.cv2_to_imgmsg(img, "bgr8"))
        else:
            pass


    # 機体の回転速度設定
    def _rotation_velocity(self):
        # Z軸回転速度の係数
        VELOCITY = 0.75 * math.pi * 1.5
        if self._point_of_line_center is None:
            return 0.0
        
        # 画面の中央にライン中央が来るようにする
        half_width = self._captured_img.shape[1] / 2.0
        pos_x_rate = (half_width - self._point_of_line_center[0]) / half_width
        rot_vel = pos_x_rate * VELOCITY
        return rot_vel


    # バイナリイメージ化，モルフォロジー等の前処理を行う
    def _preprocessing(self, cv_img):
        if cv_img is None:
            return None
        # しきい値処理用の輝度値
        max_value = 100;
        mono_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        # 画面上部にはライン以外の情報が多いことから
        # 画面下部のみを使用するため，上半分を白くする
        mono_img[:mono_img.shape[0]/2, :] = 255

        # しきい値処理(CUDA)
        # しきい値以下の輝度の箇所を255，それ以外を0とする画像を作成
        self._gpu_img_src.upload(mono_img)
        _, self._gpu_img_dst = cv2.cuda.threshold(self._gpu_img_src, max_value, 255, cv2.THRESH_BINARY_INV)
        binary_img = self._gpu_img_dst.download()

        # ノイズ除去のためモルフォロジーを行う
        kernel = np.ones((7,7), np.uint8)
        binary_img = cv2.morphologyEx(binary_img, cv2.MORPH_CLOSE, kernel)

        # バイナリイメージのパブリッシュ（確認用）
        self._monitor(binary_img, self._pub_binary_img)

        return binary_img

    
    # 追跡するラインの中央を検出
    def _extract_line_center(self, binary_img):
        # 2値画像内の一行から探索を行う
        # ここでは画像の下20％の高さの位置（上から80％の位置）となっている
        h, w = binary_img.shape
        scanning_h = h*4/5
        # 最初は適当にROIを決定する
        if self._roi == None:
            centor = binary_img.shape[1]//2
            self._roi = (centor-200, centor+200)
        roi_left, roi_right = self._roi
        scanning_line = binary_img[scanning_h, roi_left:roi_right]
        i = 0
        pre = False
        now = False
        left_edge_list = []
        cl_list = []

        # ライン検出
        for pixel in scanning_line:
            if pixel != 0:
                now = True
            else:
                now = False

            if pre == False and now == True:
                left_edge_list.append(i)
            if (left_edge_list != []) and (pre == True and now == False):
                left = left_edge_list[-1]
                right = i
                cl = CourceLine(left, right)
                cl_list.append(cl)

            pre = now
            i += 1

        # 最も太いラインのインデクスを取得
        widest_line_index = self.calc_widest_line(cl_list)
        if cl_list != []:
            widest_line = cl_list[widest_line_index]
            # roi_leftの分だけ座標変換
            left = widest_line.left + roi_left
            right = widest_line.right + roi_left
            widest_line = CourceLine(left, right)

        else:
            widest_line = None

        return widest_line, scanning_h

    
    # ラインのリストを受け取って，その内最も幅が太いラインのインデクスを返す
    def calc_widest_line(self, cl_list):
        pre_line = CourceLine()
        now_line = CourceLine()
        widest_line_index = 0
        i = 0
        for line in cl_list:
            now_line = line
            if pre_line < now_line:
                widest_line_index = i
            pre_line = now_line
            i += 1
        return widest_line_index


    # 現在追跡しているライン情報を元にROIを更新
    def _update_roi(self):
        cl = self._widest_line
        ex = self._expand_range
        roi_left = cl.left-ex
        roi_right = cl.right+ex
        if roi_left < 0:
            roi_left = 0

        if roi_right > self._captured_img.shape[1]:
            roi_right = self._captured_img.shape[1]

        self._roi = (roi_left, roi_right)


    # 追跡点の描画
    def _draw_result_img(self, img):
        result_img = cv2.circle(img, self._point_of_line_center, 15, (0, 0, 255), thickness=-1)

        return result_img


    # 画像処理のメイン部分
    def img_processing(self):
        org_img = copy.deepcopy(self._captured_img)
        input_img = copy.deepcopy(org_img)
        result_img = None
        if input_img is not None:
            # 前処理
            binary_line_img = self._preprocessing(input_img)

            # 最も幅の広いラインを検出
            self._widest_line, scanning_h = self._extract_line_center(binary_line_img) 

            if self._widest_line != None:
                # ラインの中央を検出
                self._point_of_line_center = (self._widest_line.center, scanning_h)
                result_img = self._draw_result_img(org_img)
                print("widest_line_center: {}".format(self._widest_line.center))
                # 画像処理結果をパブリッシュ
                self._monitor(result_img, self._pub_result_img)
                self._update_roi()
            else:
                time.sleep(1)
                self.img_processing()
                

    # 機体制御のメイン部分
    def control(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1
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
