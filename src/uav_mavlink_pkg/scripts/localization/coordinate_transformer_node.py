#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
坐标变换节点（G 题）。

职责：
- 发布 world → base_link → camera_link TF。
- 提供 UAV 位姿缓存供其他服务查询。

约束：
- 所有坐标转换集中在此节点，其他节点只处理世界坐标或机体坐标。
"""

import math
import sys
import os

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_COMMON_DIR = os.path.join(_SCRIPT_DIR, "..", "common")
if _COMMON_DIR not in sys.path:
    sys.path.insert(0, _COMMON_DIR)

from uav_interfaces import TopicNames
from uav_mavlink_pkg.msg import UavState


class CoordinateTransformerNode(object):
    """G 题坐标变换服务节点。"""

    def __init__(self):
        rospy.init_node("coordinate_transformer_node")

        self._camera_x = rospy.get_param("~camera_x", 0.0)
        self._camera_y = rospy.get_param("~camera_y", 0.0)
        self._camera_z = rospy.get_param("~camera_z", 0.0)

        self._current_pose = None
        self._current_yaw_deg = 0.0

        self._tf_broadcaster = tf2_ros.TransformBroadcaster()

        self._state_sub = rospy.Subscriber(
            TopicNames.LOCALIZATION_UAV_STATE, UavState,
            self._state_callback, queue_size=10
        )

        rospy.loginfo("[coordinate_transformer_node] G 题 TF 发布节点初始化完成")

    def _state_callback(self, msg):
        self._current_pose = msg.pose
        self._current_yaw_deg = msg.yaw_deg
        self._publish_tf(msg.pose, msg.yaw_deg)

    def _publish_tf(self, pose, yaw_deg):
        stamp = rospy.Time.now()

        # world → base_link
        t1 = TransformStamped()
        t1.header.stamp = stamp
        t1.header.frame_id = "world"
        t1.child_frame_id = "base_link"
        t1.transform.translation.x = pose.position.x
        t1.transform.translation.y = pose.position.y
        t1.transform.translation.z = pose.position.z
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, math.radians(yaw_deg))
        t1.transform.rotation.x = q[0]
        t1.transform.rotation.y = q[1]
        t1.transform.rotation.z = q[2]
        t1.transform.rotation.w = q[3]

        # base_link → camera_link
        t2 = TransformStamped()
        t2.header.stamp = stamp
        t2.header.frame_id = "base_link"
        t2.child_frame_id = "camera_link"
        t2.transform.translation.x = self._camera_x
        t2.transform.translation.y = self._camera_y
        t2.transform.translation.z = self._camera_z
        t2.transform.rotation.w = 1.0

        self._tf_broadcaster.sendTransform([t1, t2])

    def run(self):
        rospy.spin()


def main():
    node = CoordinateTransformerNode()
    node.run()


if __name__ == "__main__":
    main()
