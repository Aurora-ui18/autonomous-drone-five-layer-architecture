#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
飞行数据记录节点。

职责：
- 统一订阅关键话题（图像、状态、航点、安全事件）。
- 按任务启动时间创建 rosbag 与文本日志目录。
- 记录结构化 JSONL 飞行日志，便于赛后复盘。

约束：
- 只读不写控制指令，不影响飞行安全。
- 默认不录制原始图像话题以节省磁盘；可通过参数开启。
"""

import os
import json
import time
import datetime

import rospy
import rosbag
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Bool, Int8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance

import sys
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_COMMON_DIR = os.path.join(_SCRIPT_DIR, "..", "common")
if _COMMON_DIR not in sys.path:
    sys.path.insert(0, _COMMON_DIR)

from uav_interfaces import TopicNames
from uav_mavlink_pkg.msg import (
    UavState, MissionStatus, SafetyEvent, WaypointList
)


class FlightRecorderNode(object):
    """飞行记录节点。"""

    def __init__(self):
        """初始化记录器。"""
        rospy.init_node("flight_recorder_node")

        # 参数
        self._record_bag = rospy.get_param("~record_bag", True)
        self._record_jsonl = rospy.get_param("~record_jsonl", True)
        self._record_images = rospy.get_param("~record_images", False)
        self._output_dir = rospy.get_param("~output_dir", "~/uav_logs")
        self._max_bag_size_mb = float(rospy.get_param("~max_bag_size_mb", 2048.0))

        # 创建输出目录
        self._output_dir = os.path.expanduser(self._output_dir)
        self._mission_dir = os.path.join(
            self._output_dir,
            datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        )
        os.makedirs(self._mission_dir, exist_ok=True)

        # 文件句柄
        self._bag = None
        self._jsonl_file = None
        if self._record_bag:
            bag_path = os.path.join(self._mission_dir, "flight.bag")
            self._bag = rosbag.Bag(bag_path, "w")
            rospy.loginfo("[flight_recorder_node] bag 路径: %s", bag_path)
        if self._record_jsonl:
            jsonl_path = os.path.join(self._mission_dir, "flight.jsonl")
            self._jsonl_file = open(jsonl_path, "w", encoding="utf-8")
            rospy.loginfo("[flight_recorder_node] jsonl 路径: %s", jsonl_path)

        # 订阅 — 注意 EXECUTION_CMD_POSE 是 geometry_msgs/PoseStamped，不是 Odometry
        self._subs = []
        self._subscribe(TopicNames.LOCALIZATION_UAV_STATE, UavState)
        self._subscribe(TopicNames.DECISION_MISSION_STATUS, MissionStatus)
        self._subscribe(TopicNames.SAFETY_EVENT, SafetyEvent)
        self._subscribe("/Odometry", Odometry)

        if self._record_images:
            self._subscribe("/camera/image_compressed", CompressedImage)

        rospy.on_shutdown(self._shutdown)
        rospy.loginfo("[flight_recorder_node] 初始化完成")

    def _subscribe(self, topic, msg_type):
        """订阅话题并注册回调。"""
        self._subs.append(
            rospy.Subscriber(topic, msg_type, self._make_callback(topic, msg_type))
        )

    def _make_callback(self, topic, msg_type):
        """生成带话题信息的回调。"""
        def _callback(msg):
            now = rospy.Time.now()
            if self._bag is not None:
                try:
                    self._bag.write(topic, msg, now)
                except Exception as e:
                    rospy.logwarn_throttle(5.0, "写入 bag 失败: %s", e)
            if self._jsonl_file is not None:
                self._write_jsonl(topic, msg, now)
        return _callback

    def _write_jsonl(self, topic, msg, stamp):
        """写入 JSONL 摘要。兼容多种消息类型。"""
        entry = {
            "timestamp": stamp.to_sec(),
            "topic": topic,
            "type": msg.__class__.__name__,
        }
        # 提取位置
        if hasattr(msg, "pose"):
            try:
                # nav_msgs/Odometry: msg.pose 是 PoseWithCovariance
                if hasattr(msg.pose, "pose") and hasattr(msg.pose.pose, "position"):
                    p = msg.pose.pose.position
                    entry["pose"] = {"x": p.x, "y": p.y, "z": p.z}
                else:
                    # geometry_msgs/PoseStamped / UavState: msg.pose 是 Pose
                    p = msg.pose.position
                    entry["pose"] = {"x": p.x, "y": p.y, "z": p.z}
            except AttributeError:
                pass
        # 提取状态
        if hasattr(msg, "state"):
            try:
                entry["state"] = int(msg.state)
            except (TypeError, ValueError):
                pass
        if hasattr(msg, "level"):
            try:
                entry["level"] = int(msg.level)
                entry["event"] = getattr(msg, "event", "")
            except (TypeError, ValueError):
                pass
        if hasattr(msg, "cell_code"):
            entry["cell_code"] = msg.cell_code

        try:
            self._jsonl_file.write(json.dumps(entry, ensure_ascii=False) + "\n")
        except Exception as e:
            rospy.logwarn_throttle(5.0, "写入 jsonl 失败: %s", e)

    def _shutdown(self):
        """关闭文件。"""
        if self._bag is not None:
            self._bag.close()
            self._bag = None
        if self._jsonl_file is not None:
            self._jsonl_file.close()
            self._jsonl_file = None
        rospy.loginfo("[flight_recorder_node] 已关闭")

    def spin(self):
        """主循环。"""
        rospy.spin()


if __name__ == "__main__":
    node = FlightRecorderNode()
    node.spin()
