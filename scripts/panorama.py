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
import rosparam
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Panorama():
    def __init__(self):
        self._cv_bridge = CvBridge()
        self._captured_img_l = np.empty(0)
        self._captured_img_r = np.empty(0)
        self._image_points_l = None
        self._image_points_r = None

        self._w = 0
        self._h = 0
        self._top = 0
        self._bottom = 0
        self._left = 0
        self._right = 0

        # 元画像より大きな白一色の画像を作成し，その上に元画像を重ねることでパノラマ化
        # 拡張画像の倍率
        self._magnification = 1.5

        # パノラマとして使用する左画像の範囲[pixel]
        self._expand_range = 50

        # image_pointsをロード
        self._image_points_l = rosparam.load_file('image_points_left.yaml')
        self._image_points_r = rosparam.load_file('image_points_right.yaml')

        self._image_points_l = self._image_points_l[0][0]
        self._image_points_r = self._image_points_r[0][0]

        self._image_points_l = np.array(self._image_points_l)
        self._image_points_r = np.array(self._image_points_r)

        self._image_points_l = self._image_points_l.reshape(30, 1, 2)
        self._image_points_r = self._image_points_r.reshape(30, 1, 2)
        

        # 歪み補正後の画像をサブスクライブ
        self._sub_img_l = rospy.Subscriber("/left/image_rect", Image, self._img_callback, callback_args="left")
        self._sub_img_r = rospy.Subscriber("/right/image_rect", Image, self._img_callback, callback_args="right")
        
        # 画像を取得するまで再帰
        while self._captured_img_l.ndim != 3 or self._captured_img_r.ndim != 3:
            if rospy.is_shutdown():
                break
            self.__init__()
        
        self._h, self._w = self._captured_img_l.shape[:2]

        # 拡張画像用にl, r, t, bを計算
        mag = self._magnification
        h = self._h
        w = self._w

        margin_h = int((mag -1)*0.5*h)
        margin_w = int((mag -1)*0.5*w)

        self._left = margin_w # 元画像の左端
        self._right = margin_w + w # 元画像の右端
        self._top = margin_h # 元画像の上端
        self._bottom = margin_h + h # 元画像の下端

        # ホモグラフィを計算
        ex_range = self._expand_range
        pano_img_points_l = self._trans_image_points(self._image_points_l)
        pano_img_points_r = self._trans_image_points(self._image_points_r)

        H, mask = cv2.findHomography(pano_img_points_l, pano_img_points_r, 0)

        self._H = H

        # CUDA用
        
        self._img_gpu_src = cv2.cuda_GpuMat()
        self._img_gpu_dst = cv2.cuda_GpuMat()

        # パブリッシュ
        self._pub_panorama_img = rospy.Publisher("/panorama_img", Image, queue_size=1)
        self._pub_right_img = rospy.Publisher("/right_img", Image, queue_size=1)
        self._pub_warped_img = rospy.Publisher("/warped_img", Image, queue_size=1)



    def _img_callback(self, img, position):
        try:
            if position == "left":
                self._captured_img_l = self._cv_bridge.imgmsg_to_cv2(img, "bgr8")
            elif position == "right":
                self._captured_img_r = self._cv_bridge.imgmsg_to_cv2(img, "bgr8")

        except CvBridgeError as e:
            rospy.logerr(e)


    def _monitor(self, img, pub):
        if img.ndim == 2:
            pub.publish(self._cv_bridge.cv2_to_imgmsg(img, "mono8"))
        elif img.ndim == 3:
            pub.publish(self._cv_bridge.cv2_to_imgmsg(img, "bgr8"))
        else:
            pass


    # パノラマ用に拡張した画像を作成
    def _create_ex_img(self, img_l, img_r):
        h = self._h
        w = self._w
        mag = self._magnification
        l = self._left
        r = self._right
        t = self._top
        b = self._bottom

        expanded_h = int(h*mag)
        expanded_w = int(w*mag)

        # 下地となる白い画像の作成
        expanded_img_l = np.full((expanded_h, expanded_w, 3), 255, dtype='uint8')
        expanded_img_r = np.full((expanded_h, expanded_w, 3), 255, dtype='uint8')

        # 元の画像で一部上書き
        expanded_img_l[t:b, l:r] = img_l
        expanded_img_r[t:b, l:r] = img_r

        return expanded_img_l, expanded_img_r


    # image_pointsをパノラマ画像の座標に移す
    def _trans_image_points(self, image_points):
        image_points = copy.deepcopy(image_points)
        l = self._left
        r = self._right
        t = self._top
        b = self._bottom

        image_points[:, :, 0] = image_points[:, :, 0] + l
        image_points[:, :, 1] = image_points[:, :, 1] + t

        return image_points


    # パノラマ画像の作成
    def _create_panorama_img(self, ex_img_l, ex_img_r):
        l = self._left
        r = self._right
        t = self._top
        b = self._bottom
        H = self._H
        ex_range = self._expand_range

        ex_h, ex_w = ex_img_l.shape[:2]

        # 左画像を射影変換
        self._img_gpu_src.upload(ex_img_l)
        self._img_gpu_dst = cv2.cuda.warpPerspective(self._img_gpu_src , H, (ex_w, ex_h), borderValue=(255,255,255))
        warped_img_l = self._img_gpu_dst.download()

        panorama_img = ex_img_r
        panorama_img[:, :l+ex_range] = warped_img_l[:, :l+ex_range]
        panorama_img = panorama_img[t:b, :r]

        return panorama_img


    # メイン部分
    def img_processing(self):
        org_img_l = self._captured_img_l
        org_img_r = self._captured_img_r
        
        ex_img_l, ex_img_r = self._create_ex_img(org_img_l, org_img_r)
        pano_img = self._create_panorama_img(ex_img_l, ex_img_r)

        self._monitor(pano_img, self._pub_panorama_img)


if __name__ == '__main__':
    rospy.init_node('stereo_line_tracing')
    p = Panorama()

    rate = rospy.Rate(20)
    rate.sleep()
    while not rospy.is_shutdown():
        p.img_processing()
        rate.sleep()
