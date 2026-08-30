#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
独立安全监控节点（G 题）。

与 decision/safety_monitor_node 解耦，形成物理隔离的安全旁路。
直接订阅飞控/定位层原始数据，不依赖 decision 层状态。
"""

import sys
import os

import rospy
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_COMMON_DIR = os.path.join(_SCRIPT_DIR, "..", "common")
if _COMMON_DIR not in sys.path:
    sys.path.insert(0, _COMMON_DIR)

from uav_interfaces import TopicNames, SafetyLevel
from uav_mavlink_pkg.msg import UavState, SafetyEvent


class SafetySupervisorNode(object):
    """独立安全监控节点。"""

    def __init__(self):
        rospy.init_node("safety_supervisor_node")

        self._field_min_x = float(rospy.get_param("~field_min_x", 0.0))
        self._field_max_x = float(rospy.get_param("~field_max_x", 5.0))  # D题 50dm = 5.0m
        self._field_min_y = float(rospy.get_param("~field_min_y", 0.0))
        self._field_max_y = float(rospy.get_param("~field_max_y", 4.0))  # D题 40dm = 4.0m
        self._max_height = float(rospy.get_param("~max_height", 2.5))
        self._min_voltage = float(rospy.get_param("~min_voltage", 10.5))
        self._rc_timeout = float(rospy.get_param("~rc_timeout", 3.0))
        self._pose_timeout = float(rospy.get_param("~pose_timeout", 1.0))

        self._current_pose = None
        self._current_voltage = 12.0
        self._rc_connected = False
        self._last_pose_time = rospy.Time.now()
        self._last_rc_time = rospy.Time.now()

        self._safety_pub = rospy.Publisher(
            TopicNames.SAFETY_EVENT, SafetyEvent, queue_size=10
        )

        self._state_sub = rospy.Subscriber(
            TopicNames.LOCALIZATION_UAV_STATE, UavState,
            self._state_callback, queue_size=10
        )

        self._clear_srv = rospy.Service("~clear_emergency", Trigger, self._handle_clear)

        self._timer = rospy.Timer(rospy.Duration(0.1), self._tick)
        rospy.loginfo("[safety_supervisor_node] G 题独立安全监控初始化完成")

    def _state_callback(self, msg):
        self._current_pose = msg.pose
        self._current_voltage = msg.battery_voltage
        self._rc_connected = msg.rc_connected
        self._last_pose_time = rospy.Time.now()
        if self._rc_connected:
            self._last_rc_time = rospy.Time.now()

    def _handle_clear(self, req):
        return TriggerResponse(success=True, message="cleared")

    def _tick(self, event):
        if self._current_pose is None:
            return

        z = self._current_pose.position.z

        if z > self._max_height:
            self._publish_event(SafetyLevel.CRITICAL, "超出最大安全高度 %.1fm" % self._max_height, "紧急降落")

        # if self._current_voltage < self._min_voltage:  # disabled per user request
        #     self._publish_event(SafetyLevel.CRITICAL, "电池电压过低: %.2fV" % self._current_voltage, "立即降落")

        pose_elapsed = (rospy.Time.now() - self._last_pose_time).to_sec()
        if pose_elapsed > self._pose_timeout:
            self._publish_event(SafetyLevel.CRITICAL, "定位数据超时丢失", "切换 LAND 模式")

    def _publish_event(self, level, event, suggestion):
        msg = SafetyEvent()
        msg.header.stamp = rospy.Time.now()
        msg.level = level
        msg.event = event
        msg.source_node = "safety_supervisor_node"
        msg.suggestion = suggestion
        self._safety_pub.publish(msg)

    def run(self):
        rospy.spin()


def main():
    node = SafetySupervisorNode()
    node.run()


if __name__ == "__main__":
    main()
