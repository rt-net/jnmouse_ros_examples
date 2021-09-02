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
import rospkg
import cv2
import numpy as np
import copy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class StereoDepthEstimator:
    def __init__(self):
        self._cv_bridge = CvBridge()
        self._captured_img_l = None
        self._captured_img_r = None
        self._captured_img_width = 1
        self._captured_img_height = 1
        self._left_camera_image_topic = "/camera_l/image_rect_color"
        self._right_camera_image_topic = "/camera_r/image_rect_color"
        sub_img_l = message_filters.Subscriber(self._left_camera_image_topic, Image)
        sub_img_r = message_filters.Subscriber(self._right_camera_image_topic, Image)
        self.mf = message_filters.ApproximateTimeSynchronizer([sub_img_l, sub_img_r], 100, 10.0)
        self.mf.registerCallback(self._img_callback)
        self._pub_depth_img = rospy.Publisher("/depth/image_rect", Image, queue_size=1)
        self._is_debug = rospy.get_param("~debug")
        self.img_scale = 0.5

        rospy.loginfo("waiting for left camera image")
        rospy.wait_for_message(self._left_camera_image_topic, Image)
        rospy.loginfo("waiting for right camera image")
        rospy.wait_for_message(self._right_camera_image_topic, Image)
        rospy.loginfo("camera image topic received")

        rospy.loginfo("loading camera parameter")
        camera_param = np.load('{}/config/camera_param_fisheye.npz'.format(rospkg.RosPack().get_path('jnmouse_ros_examples')))
        mtx_l, dist_l, mtx_r, dist_r, R, T = [camera_param[i] for i in ["mtx_l", "dist_l", "mtx_r", "dist_r", "R", "T"]]

        newmtx_r = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(mtx_r, dist_r, (self._captured_img_width, self._captured_img_height), None, balance=1.0)
        newmtx_l = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(mtx_l, dist_l, (self._captured_img_width, self._captured_img_height), None, balance=1.0)
        Rp_l, Rp_r, Pp_l, Pp_r, self.Q = cv2.fisheye.stereoRectify(newmtx_l, dist_l, newmtx_r, dist_r, (self._captured_img_width, self._captured_img_height), R, T, 0)

        min_disp = 16
        window_size = 9
        self.stereo = cv2.StereoSGBM_create(
            minDisparity = min_disp,
            numDisparities = 16*3,
            blockSize = window_size,
            P1 = 8*1*window_size**2,
            P2 = 32*1*window_size**2,
            disp12MaxDiff = 1,
            uniquenessRatio = 5,
            speckleWindowSize = 50,
            speckleRange = 2,
            mode = cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

    def _img_callback(self, img_l, img_r):
        try:
            self._captured_img_height = img_l.height
            self._captured_img_width = img_l.width
            self._captured_img_l = self._cv_bridge.imgmsg_to_cv2(img_l, "bgr8")
            self._captured_img_r = self._cv_bridge.imgmsg_to_cv2(img_r, "bgr8")

        except CvBridgeError as e:
            rospy.logerr(e)

    def _monitor(self, img, pub):
        if img.ndim == 2:
            if self._is_debug:
                # 距離測定位置に円を描画
                center_x = int(self._captured_img_width*self.img_scale/2)
                center_y = int(self._captured_img_height*self.img_scale/2)
                rospy.loginfo("distance[mm]: "+str(img[center_y, center_x]))
                cv2.circle(img, (center_x, center_y), 5, 0, thickness=1)

            pub.publish(self._cv_bridge.cv2_to_imgmsg(img, "16UC1"))
        elif img.ndim == 3:
            pub.publish(self._cv_bridge.cv2_to_imgmsg(img, "bgr8"))
        else:
            pass

    def depth_estimation(self):
        org_img_l = copy.deepcopy(self._captured_img_l)
        org_img_r = copy.deepcopy(self._captured_img_r)

        if org_img_l is not None and org_img_r is not None:
            # Block Matchingによるステレオマッチング
            rectified_grayimg_l = cv2.cvtColor(org_img_l, cv2.COLOR_BGR2GRAY)
            rectified_grayimg_r = cv2.cvtColor(org_img_r, cv2.COLOR_BGR2GRAY)
            rectified_grayimg_l_half = cv2.resize(rectified_grayimg_l, dsize=None, fx=self.img_scale, fy=self.img_scale)
            rectified_grayimg_r_half = cv2.resize(rectified_grayimg_r, dsize=None, fx=self.img_scale, fy=self.img_scale)
            disparity = self.stereo.compute(rectified_grayimg_l_half, rectified_grayimg_r_half) / 16 / self.img_scale

            # 視差[px]を距離[mm]に変換
            depth = self.Q[2, 3] / (self.Q[3, 2] * disparity + self.Q[3, 3])

            depth[np.where(depth < 0)] = 0
            depth[np.where(depth > 400)] = 0
            depth = depth.astype(np.uint16)

            self._monitor(depth, self._pub_depth_img)

        else:
            rospy.loginfo('There is no valid captured image')


if __name__ == '__main__':
    rospy.init_node('jnm_depth_estimator')
    sde = StereoDepthEstimator()

    rate = rospy.Rate(10)
    rate.sleep()
    while not rospy.is_shutdown():
        sde.depth_estimation()
        rate.sleep()
