#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Landing pad locator node - detects black circle landing pad.

Responsibilities:
  - Subscribe to /camera/image_raw and /localization/uav_state
  - Detect black circle landing pad, return offset relative to UAV
  - Publish /localization/landing_pad pose

Constraints:
  - Only publish detection results and offsets, no landing control."""

import math
import sys
import os

import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_COMMON_DIR = os.path.join(_SCRIPT_DIR, "..", "common")
if _COMMON_DIR not in sys.path:
    sys.path.insert(0, _COMMON_DIR)

from uav_interfaces import TopicNames, ServiceNames, pixel_to_meter
from uav_mavlink_pkg.msg import UavState
from uav_mavlink_pkg.srv import DetectLandingPad, DetectLandingPadResponse

try:
    import cv2
except ImportError:
    cv2 = None


class LandingPadLocatorNode(object):
    """Black circle landing pad locator node."""

    def __init__(self):
        """Initialize node: subscribers, service, publishers."""
        rospy.init_node("landing_pad_locator_node")
        self._focal_length_px = rospy.get_param("~focal_length_px", 346.0)
        self._gray_threshold = rospy.get_param("~gray_threshold", 60)
        self._max_brightness = rospy.get_param("~max_brightness", 80)
        self._hough_dp = rospy.get_param("~hough_dp", 1.2)
        self._hough_min_dist = rospy.get_param("~hough_min_dist", 100)
        self._hough_param1 = rospy.get_param("~hough_param1", 50)
        self._hough_param2 = rospy.get_param("~hough_param2", 30)
        self._min_radius = rospy.get_param("~min_radius", 20)
        self._max_radius = rospy.get_param("~max_radius", 120)
        self._publish_debug = rospy.get_param("~publish_debug", True)

        # CvBridge
        self._bridge = CvBridge()

        # State cache
        self._current_height = 1.5

        # Publishers
        self._landing_pad_pub = rospy.Publisher(
            TopicNames.LOCALIZATION_LANDING_PAD, PoseStamped, queue_size=1
        )

        # Subscribers
        self._image_sub = rospy.Subscriber(
            TopicNames.CAMERA_IMAGE_RAW, Image, self._image_callback, queue_size=1
        )
        self._state_sub = rospy.Subscriber(
            TopicNames.LOCALIZATION_UAV_STATE, UavState, self._state_callback, queue_size=10
        )

        # 服务
        self._detect_srv = rospy.Service(
            "~detect_landing_pad", DetectLandingPad, self._handle_detect
        )

        if cv2 is None:
            rospy.logerr("[landing_pad_locator_node] opencv-python not installed")

        rospy.loginfo("[landing_pad_locator_node] Init complete")

    def _state_callback(self, msg):
        """
        UAV state callback, cache current height.
        Args:
            msg (UavState): Fused state.
        """
        self._current_height = msg.pose.position.z

    def _detect_black_circle(self, cv_image):
        """
        Detect black circle landing pad in image.
        Args:
            cv_image (numpy.ndarray): BGR image.
        Returns:
            tuple: (offset_x_m, offset_y_m, confidence) or (0.0, 0.0, 0.0).
        """
        if cv2 is None:
            return 0.0, 0.0, 0.0

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)

        # 提取暗区
        _, dark_mask = cv2.threshold(
            blurred, self._gray_threshold, 255, cv2.THRESH_BINARY_INV
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_OPEN, kernel)

        # Hough circle detection
        circles = cv2.HoughCircles(
            dark_mask,
            cv2.HOUGH_GRADIENT,
            dp=self._hough_dp,
            minDist=self._hough_min_dist,
            param1=self._hough_param1,
            param2=self._hough_param2,
            minRadius=self._min_radius,
            maxRadius=self._max_radius,
        )

        if circles is None:
            return 0.0, 0.0, 0.0

        img_h, img_w = gray.shape[:2]
        cx_img = img_w / 2.0
        cy_img = img_h / 2.0

        best = None
        best_score = float("inf")
        for c in circles[0]:
            cx, cy, r = int(c[0]), int(c[1]), int(c[2])
            if r <= 0:
                continue
            mask = np.zeros_like(gray)
            cv2.circle(mask, (cx, cy), r, 255, -1)
            mean_val = cv2.mean(gray, mask=mask)[0]
            dist_to_center = math.hypot(cx - cx_img, cy - cy_img)
            score = mean_val + dist_to_center * 0.1
            if score < best_score:
                best_score = score
                best = (cx, cy, r, mean_val)

        if best is None or best[3] >= self._max_brightness:
            return 0.0, 0.0, 0.0

        cx, cy = best[0], best[1]
        offset_x_px = float(cx) - cx_img
        offset_y_px = float(cy) - cy_img

        # Pixel offset to meters (reuse similar-triangle coefficients from legacy code)
        offset_x_m = pixel_to_meter(offset_x_px, self._current_height, self._focal_length_px)
        offset_y_m = pixel_to_meter(offset_y_px, self._current_height, self._focal_length_px)

        confidence = 1.0 - min(best[3] / self._max_brightness, 1.0)
        return offset_x_m, offset_y_m, confidence

    def _image_callback(self, msg):
        """
        Image callback, publish landing pad pose when circle detected.
        Args:
            msg (sensor_msgs.msg.Image): Input image.
        """
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("[landing_pad_locator_node] 图像转换失败: %s", e)
            return

        ox, oy, conf = self._detect_black_circle(cv_image)
        if conf <= 0.0:
            return

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "camera_link"
        pose.pose.position.x = ox
        pose.pose.position.y = oy
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        self._landing_pad_pub.publish(pose)

    def _handle_detect(self, req):
        """
        Detect landing pad service.
        Args:
            req (DetectLandingPadRequest): Contains image and height.
        Returns:
            DetectLandingPadResponse: Relative offset and detection flag.
        """
        resp = DetectLandingPadResponse()
        try:
            cv_image = self._bridge.imgmsg_to_cv2(req.image, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("[landing_pad_locator_node] 服务图像转换失败: %s", e)
            resp.detected = False
            return resp

        height = req.height if req.height > 0.1 else self._current_height
        ox, oy, conf = self._detect_black_circle(cv_image)
        resp.offset_x = ox
        resp.offset_y = oy
        resp.detected = conf > 0.0
        resp.confidence = conf
        return resp

    def run(self):
        """Enter ROS main loop."""
        rospy.spin()


def main():
    """Node entry point."""
    node = LandingPadLocatorNode()
    node.run()


if __name__ == "__main__":
    main()
