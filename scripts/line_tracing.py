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


class ObjectTracker():

    def __init__(self):
        self._cv_bridge = CvBridge()
        self._captured_image = None
        self._object_pixels = 0  # Maximum area detected in the current image[pixel]
        self._object_pixels_default = 0  # Maximum area detected from the first image[pixel]
        self._point_of_centroid = None

        self._pub_binary_image = rospy.Publisher("binary", Image, queue_size=1)
        self._pub_pbject_image = rospy.Publisher("object", Image, queue_size=1)
        self._pub_cmdvel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self._sub_image = rospy.Subscriber("/csi_cam_0/image_raw", Image, self._image_callback)

        rospy.wait_for_service("/motor_on")
        rospy.wait_for_service("/motor_off")
        rospy.on_shutdown(rospy.ServiceProxy("/motor_off", Trigger).call)
        rospy.ServiceProxy("/motor_on", Trigger).call()

    def _image_callback(self, img):
        try:
            self._captured_image = self._cv_bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def _pixels(self, cv_image):
        return cv_image.shape[0] * cv_image.shape[1]

    def _object_is_detected(self):
        # Lower limit of the ratio of the detected area to the screen.
        # Object tracking is not performed below this ratio.
        LOWER_LIMIT = 0.01

        if self._captured_image is not None:
            object_per_image = self._object_pixels / self._pixels(self._captured_image)
            return object_per_image > LOWER_LIMIT
        else:
            return False

    def _object_pixels_ratio(self):
        if self._captured_image is not None:
            diff_pixels = self._object_pixels - self._object_pixels_default
            return diff_pixels / self._pixels(self._captured_image)
        else:
            return 0

    def _object_is_bigger_than_default(self):
        return self._object_pixels_ratio() > 0.01

    def _object_is_smaller_than_default(self):
        return self._object_pixels_ratio() < -0.01


    def _calibrate_object_pixels_default(self):
        if self._object_pixels_default == 0 and self._object_pixels != 0:
            self._object_pixels_default = self._object_pixels

    def _extract_biggest_contour(self, binary_img):
        biggest_contour_index = False
        biggest_contour_area = 0
        contours, hierarchy = cv2.findContours(
            binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if biggest_contour_area < area:
                biggest_contour_area = area
                biggest_contour_index = i

        if biggest_contour_index is False:
            return False
        else:
            return contours[biggest_contour_index]

    def _calculate_centroid_point(self, contour):
        point = False
        if self._object_is_detected():
            M = cv2.moments(contour)
            centroid_x = int(M['m10'] / M['m00'])
            centroid_y = int(M['m01'] / M['m00'])
            point = (centroid_x, centroid_y)

        return point

    def _draw_contour(self, input_image, contour):
        return cv2.drawContours(input_image, [contour], 0, (0, 255, 0), 5)

    def _draw_centroid(self, input_image, point_centroid):
        return cv2.circle(input_image, point_centroid, 15, (255, 0, 0), thickness=-1)

    def _monitor(self, img, pub):
        if img.ndim == 2:
            pub.publish(self._cv_bridge.cv2_to_imgmsg(img, "mono8"))
        elif img.ndim == 3:
            pub.publish(self._cv_bridge.cv2_to_imgmsg(img, "bgr8"))
        else:
            pass

    def _rotation_velocity(self):
        VELOCITY = 0.25 * math.pi
        if not self._object_is_detected() or self._point_of_centroid is None:
            return 0.0

        half_width = self._captured_image.shape[1] / 2.0
        pos_x_rate = (half_width - self._point_of_centroid[0]) / half_width
        rot_vel = pos_x_rate * VELOCITY
        return rot_vel

    def _extract_line_in_binary(self, cv_image):
        if cv_image is None:
            return None
        min_value = 0;
        max_value = 100;

        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # 画面の下半分のみを使用するため，上半分を白くする
        gray_img[:gray_img.shape[0]/2, :] = 255
        binary = cv2.inRange(gray_img, min_value, max_value)

        return binary

    def _extract_tracking_point(self, binary_img):
        bottom_line = binary_img[-2:-1, :]
        line_position = np.where(bottom_line > 0)
        mean = np.sum(line_position[1])/line_position[1].shape
        tracking_point = (mean, binary_img.shape[0] - 1)
        return tracking_point
 
    def _draw_tracking_point(self, input_img, tracking_point):
        return cv2.circle(input_img, tracking_point, 15, (255, 0, 0), thickness=-1)


    def _extract_tracking_line(self, binary_img):
        tracking_contour_index = False
        center = binary_img.shape[1]/2
        line_center = 0
        kernel = np.ones((5,5), np.uint8)
        binary_img = cv2.morphologyEx(binary_img, cv2.MORPH_OPEN, kernel)
        contours, hierarchy = cv2.findContours(
            binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for i, cnt in enumerate(contours):
            cnt = np.array(cnt)
            print(cnt.shape)
            #bottom_contours = contours > binary_img.shape[0] - 10
            #print(contours)
            # これは２列目を抽出して条件分岐している
            # おそらく１列目を全て0にしてマスクにする方が分かりやすい
            # 次回実装する
            bottom_point_y = cnt[:, :, 1]
            print(bottom_point_y.shape)
            bottom_point = bottom_point_y[np.any(bottom_point_y > binary_img.shape[0] - 10, axis=1)]
            print(bottom_point)
            #print(contours.shape)
            #print(bottom_point)
#        return contours[biggest_contour_index]


    def image_processing(self):
        object_image = copy.deepcopy(self._captured_image)
        # modifying
        # object tracking proc to line tracing proc
        object_binary_img = self._extract_line_in_binary(self._captured_image)

        if object_binary_img is not None:
            print(object_binary_img.shape)
            tracking_point = self._extract_tracking_point(object_binary_img)
            self._extract_tracking_line(object_binary_img)
            tracking_point_img = self._draw_tracking_point(object_binary_img, tracking_point)
#            biggest_contour = self._extract_biggest_contour(object_binary_img)
#            if biggest_contour is not False:
#                self._object_pixels = cv2.contourArea(biggest_contour)
#                self._calibrate_object_pixels_default()
#
#                object_image = self._draw_contour(
#                    object_image, biggest_contour)
#
#                point = self._calculate_centroid_point(biggest_contour)
#                if point is not False:
#                    self._point_of_centroid = point
#                    object_image = self._draw_centroid(object_image, point)

            self._monitor(object_binary_img, self._pub_binary_image)
            self._monitor(tracking_point_img, self._pub_pbject_image)

    def control(self):
        cmd_vel = Twist()
        if self._object_is_detected():
            # Move backward and forward by difference from default area
            if self._object_is_smaller_than_default():
                cmd_vel.linear.x = 0.1
                print("forward")
            elif self._object_is_bigger_than_default():
                cmd_vel.linear.x = -0.1
                print("backward")
            else:
                cmd_vel.linear.x = 0
                print("stay")
            cmd_vel.angular.z = self._rotation_velocity()
        self._pub_cmdvel.publish(cmd_vel)


if __name__ == '__main__':
    rospy.init_node('object_tracking')
    rospy.sleep(3.)
    ot = ObjectTracker()

    rate = rospy.Rate(60)
    rate.sleep()
    while not rospy.is_shutdown():
        ot.image_processing()
        ot.control()
        rate.sleep()
