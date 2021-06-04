#!/usr/bin/env python
# coding: utf-8

# https://github.com/rt-net/jetson_nano_mouse_stereo_camera_develop/blob/master/undistort/undistort_fisheye_stereo.ipynb
# https://github.com/rt-net/jetson_nano_mouse_stereo_camera_develop/blob/master/camera_streaming/stereo_streaming_fisheye.py
# 上記の歪み補正をROS化したもの

import os
import rospy
import cv2
import numpy as np
import copy
import glob
import message_filters
from sensor_msgs.msg import Image
import rosparam
from cv_bridge import CvBridge, CvBridgeError

class ImageUndistortion:
    def __init__(self):
        self._cv_bridge = CvBridge()
        self._captured_img_l = None
        self._captured_img_r = None
        sub_img_l = message_filters.Subscriber("/csi_cam_0/image_raw", Image)
        sub_img_r = message_filters.Subscriber("/csi_cam_1/image_raw", Image)
        self.mf = message_filters.ApproximateTimeSynchronizer([sub_img_l, sub_img_r], 100, 10.0)
        self.mf.registerCallback(self._img_callback)
        self._pub_rect_img_l = rospy.Publisher("/camera_l/image_rect_color", Image, queue_size=1)
        self._pub_rect_img_r = rospy.Publisher("/camera_r/image_rect_color", Image, queue_size=1)
        self._captured_img_width = rosparam.get_param("/csi_cam_0/image_width")
        self._captured_img_height = rosparam.get_param("/csi_cam_0/image_height")

        camera_param = np.load('{}/catkin_ws/src/jnmouse_ros_examples/config/camera_param_fisheye.npz'.format(os.environ['HOME']))
        mtx_l, dist_l, mtx_r, dist_r, R, T = [camera_param[i] for i in ["mtx_l", "dist_l", "mtx_r", "dist_r", "R", "T"]]

        newmtx_r = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(mtx_r, dist_r, (self._captured_img_width, self._captured_img_height), None, balance=1.0)
        newmtx_l = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(mtx_l, dist_l, (self._captured_img_width, self._captured_img_height), None, balance=1.0)

        Rp_l, Rp_r, Pp_l, Pp_r, Q = cv2.fisheye.stereoRectify(newmtx_l, dist_l, newmtx_r, dist_r, (self._captured_img_width, self._captured_img_height), R, T, 0)

        self.map4p_l = cv2.fisheye.initUndistortRectifyMap(mtx_l, dist_l, Rp_l, Pp_l, (self._captured_img_width, self._captured_img_height), cv2.CV_16SC2)
        self.map4p_r = cv2.fisheye.initUndistortRectifyMap(mtx_r, dist_r, Rp_r, Pp_r, (self._captured_img_width, self._captured_img_height), cv2.CV_16SC2)

    def _img_callback(self, img_l, img_r):
        try:
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
