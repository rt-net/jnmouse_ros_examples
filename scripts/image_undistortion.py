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

class ImageUndistortion:
    def __init__(self):
        cv_version = int(cv2.__version__.split('.')[0])
        if cv_version <= 4:
            rospy.logwarn("image_undistortion requires OpenCV version >= 4.0.0")

        self._cv_bridge = CvBridge()
        self._captured_img_l = None
        self._captured_img_r = None
        self._captured_img_height = 1
        self._captured_img_width = 1
        self._left_camera_image_topic = "/csi_cam_0/image_raw"
        self._right_camera_image_topic = "/csi_cam_1/image_raw"
        sub_img_l = message_filters.Subscriber(self._left_camera_image_topic, Image)
        sub_img_r = message_filters.Subscriber(self._right_camera_image_topic, Image)
        self.mf = message_filters.ApproximateTimeSynchronizer([sub_img_l, sub_img_r], 100, 10.0)
        self.mf.registerCallback(self._img_callback)
        self._pub_rect_img_l = rospy.Publisher("/camera_l/image_rect_color", Image, queue_size=1)
        self._pub_rect_img_r = rospy.Publisher("/camera_r/image_rect_color", Image, queue_size=1)

        rospy.loginfo("waiting for left camera image")
        rospy.wait_for_message(self._left_camera_image_topic, Image)
        rospy.loginfo("waiting for right camera image")
        rospy.wait_for_message(self._right_camera_image_topic, Image)
        rospy.loginfo("camera image topic received")

        camera_param = np.load('{}/config/camera_param_fisheye.npz'.format(rospkg.RosPack().get_path('jnmouse_ros_examples')))
        mtx_l, dist_l, mtx_r, dist_r, R, T = [camera_param[i] for i in ["mtx_l", "dist_l", "mtx_r", "dist_r", "R", "T"]]

        newmtx_r = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(mtx_r, dist_r, (self._captured_img_width, self._captured_img_height), None, balance=1.0)
        newmtx_l = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(mtx_l, dist_l, (self._captured_img_width, self._captured_img_height), None, balance=1.0)

        Rp_l, Rp_r, Pp_l, Pp_r, Q = cv2.fisheye.stereoRectify(newmtx_l, dist_l, newmtx_r, dist_r, (self._captured_img_width, self._captured_img_height), R, T, 0)

        self.map4p_l = cv2.fisheye.initUndistortRectifyMap(mtx_l, dist_l, Rp_l, Pp_l, (self._captured_img_width, self._captured_img_height), cv2.CV_16SC2)
        self.map4p_r = cv2.fisheye.initUndistortRectifyMap(mtx_r, dist_r, Rp_r, Pp_r, (self._captured_img_width, self._captured_img_height), cv2.CV_16SC2)

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
            pub.publish(self._cv_bridge.cv2_to_imgmsg(img, "mono8"))
        elif img.ndim == 3:
            pub.publish(self._cv_bridge.cv2_to_imgmsg(img, "bgr8"))
        else:
            pass

    def undistortion(self):
        org_img_l = copy.deepcopy(self._captured_img_l)
        org_img_r = copy.deepcopy(self._captured_img_r)
        if org_img_l is not None and org_img_r is not None:
            # 歪み補正とステレオ平行化
            rectified_img_l = cv2.remap(org_img_l, self.map4p_l[0], self.map4p_l[1], cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT)
            rectified_img_r = cv2.remap(org_img_r, self.map4p_r[0], self.map4p_r[1], cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT)

            self._monitor(rectified_img_l, self._pub_rect_img_l)
            self._monitor(rectified_img_r, self._pub_rect_img_r)

        else:
            rospy.loginfo('There is no valid captured image')


if __name__ == '__main__':
    rospy.init_node('image_undistortion')
    sde = ImageUndistortion()

    rate = rospy.Rate(10)
    rate.sleep()
    while not rospy.is_shutdown():
        sde.undistortion()
        rate.sleep()
