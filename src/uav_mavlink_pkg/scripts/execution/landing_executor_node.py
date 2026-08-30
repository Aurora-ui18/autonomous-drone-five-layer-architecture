#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
降落执行节点。

职责：
- 提供 /execution/land 服务。
- 实现分段下降 + 视觉修正 + LAND 模式的降落流程。

约束：
- 不直接调用 MAVLink，通过发布 /execution/cmd_pose 和 /FlightMode 控制。
"""

import sys
import os

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Int8

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_COMMON_DIR = os.path.join(_SCRIPT_DIR, "..", "common")
if _COMMON_DIR not in sys.path:
    sys.path.insert(0, _COMMON_DIR)

from uav_interfaces import TopicNames, ServiceNames, is_position_arrived
from uav_mavlink_pkg.msg import UavState
from uav_mavlink_pkg.srv import ExecuteLanding, ExecuteLandingResponse, DetectLandingPad


class LandingExecutorNode(object):
    """降落执行器。"""

    def __init__(self):
        """初始化降落执行节点。"""
        rospy.init_node("landing_executor_node")

        # 参数
        self._descent_stages = rospy.get_param("~land_descent_stages", [0.8, 0.3, 0.1])
        self._final_land_height = float(rospy.get_param("~final_land_height", 0.15))
        self._xy_correction_gain = float(rospy.get_param("~xy_correction_gain", 0.8))
        self._xy_tolerance = 0.08
        self._z_tolerance = 0.08
        self._land_mode_num = 9  # ArduPilot LAND 模式

        # 状态
        self._current_pose = None
        self._current_image = None

        # 发布者
        self._cmd_pose_pub = rospy.Publisher(
            TopicNames.EXECUTION_CMD_POSE, PoseStamped, queue_size=1
        )
        self._flight_mode_pub = rospy.Publisher(
            TopicNames.FLIGHT_MODE, Int8, queue_size=1
        )

        # 订阅者
        self._state_sub = rospy.Subscriber(
            TopicNames.LOCALIZATION_UAV_STATE, UavState,
            self._state_callback, queue_size=10
        )
        self._image_sub = rospy.Subscriber(
            TopicNames.CAMERA_IMAGE_RAW, Image,
            self._image_callback, queue_size=1
        )

        # 服务
        self._land_srv = rospy.Service(
            "/execution/land", ExecuteLanding, self._handle_land
        )

        # 服务客户端
        self._detect_pad_client = rospy.ServiceProxy(
            ServiceNames.DETECT_LANDING_PAD, DetectLandingPad
        )

        rospy.loginfo("[landing_executor_node] 初始化完成")

    def _state_callback(self, msg):
        """
        UAV 状态回调。

        Args:
            msg (UavState): 融合状态。
        """
        self._current_pose = msg.pose

    def _image_callback(self, msg):
        """
        图像回调。

        Args:
            msg (Image): 输入图像。
        """
        self._current_image = msg

    def _handle_land(self, req):
        """
        降落服务回调。

        Args:
            req (ExecuteLandingRequest): 降落请求。

        Returns:
            ExecuteLandingResponse: 降落结果。
        """
        resp = ExecuteLandingResponse()
        resp.success = False
        resp.message = ""

        target_x = float(req.target_x)
        target_y = float(req.target_y)
        start_z = float(req.start_z)

        rospy.loginfo(
            "[landing_executor_node] 开始降落流程，目标=(%.2f, %.2f)，起始高度=%.2f",
            target_x, target_y, start_z
        )

        rate = rospy.Rate(10)
        mode = int(req.mode)

        if mode == 1:
            # 45°±5° 俯角下降（移植自旧 uav_library.descend_45）
            if not self._land_descend_45(target_x, target_y, start_z):
                resp.message = "45°下降失败"
                return resp
        else:
            # 垂直下降：阶段 1 飞到降落点上方 start_z 高度
            if not self._fly_to(target_x, target_y, start_z):
                resp.message = "飞到起始高度失败"
                return resp

            # 阶段 2：分段下降并视觉修正
            for stage_z in self._descent_stages:
                offset_x, offset_y = self._detect_pad_offset()
                corr_x = target_x - offset_x * self._xy_correction_gain
                corr_y = target_y - offset_y * self._xy_correction_gain

                if not self._fly_to(corr_x, corr_y, stage_z):
                    resp.message = "下降到 %.2fm 失败" % stage_z
                    return resp

        # 阶段 3：切换到 LAND 模式
        rospy.loginfo("[landing_executor_node] 切换 LAND 模式")
        self._flight_mode_pub.publish(Int8(data=self._land_mode_num))

        # 等待降落完成
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if self._current_pose is not None and self._current_pose.position.z < self._final_land_height:
                rospy.loginfo("[landing_executor_node] 降落完成")
                resp.success = True
                resp.message = "降落成功"
                return resp
            if rospy.get_time() - start_time > 30.0:
                resp.message = "降落等待超时"
                return resp
            rate.sleep()

        resp.message = "ROS 关闭"
        return resp

    def _fly_to(self, x, y, z, xy_tol=None, z_tol=None, timeout=20.0):
        """
        飞往目标位置。

        Args:
            x (float): 目标 X。
            y (float): 目标 Y。
            z (float): 目标 Z。
            xy_tol (float, optional): 水平容差，默认使用节点参数。
            z_tol (float, optional): 高度容差，默认使用节点参数。
            timeout (float, optional): 超时时间（秒）。

        Returns:
            bool: 是否到达。
        """
        if self._current_pose is None:
            rospy.logwarn("[landing_executor_node] 无当前位姿，无法飞行")
            return False

        xy_tol = xy_tol if xy_tol is not None else self._xy_tolerance
        z_tol = z_tol if z_tol is not None else self._z_tolerance

        rate = rospy.Rate(10)
        start_time = rospy.get_time()

        while not rospy.is_shutdown():
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "world"
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            msg.pose.orientation.w = 1.0
            self._cmd_pose_pub.publish(msg)

            current = (
                self._current_pose.position.x,
                self._current_pose.position.y,
                self._current_pose.position.z
            )
            target = (x, y, z)
            if is_position_arrived(current, target, xy_tol, z_tol):
                rospy.loginfo("[landing_executor_node] 到达 (%.2f, %.2f, %.2f)", x, y, z)
                return True

            if rospy.get_time() - start_time > timeout:
                rospy.logwarn("[landing_executor_node] 飞往目标超时")
                return False

            rate.sleep()

        return False

    def _land_descend_45(self, target_x, target_y, start_z, steps=6):
        """
        45°±5° 俯角下降。

        移植自旧代码 uav_library.descend_45：
        从当前位置沿 45° 对角线下降到目标落点，水平位移:垂直下降 = 1:1。

        Args:
            target_x (float): 落点 X。
            target_y (float): 落点 Y。
            start_z (float): 起始高度。
            steps (int): 分段数。

        Returns:
            bool: 是否成功。
        """
        if self._current_pose is None:
            rospy.logwarn("[landing_executor_node] 无当前位姿，无法 45° 下降")
            return False

        start_x = self._current_pose.position.x
        start_y = self._current_pose.position.y
        rospy.loginfo(
            "[landing_executor_node] 45°下降: (%.2f,%.2f,%.2f) -> (%.2f,%.2f)",
            start_x, start_y, start_z, target_x, target_y
        )

        for i in range(1, steps + 1):
            ratio = i / float(steps)
            wx = start_x + (target_x - start_x) * ratio
            wy = start_y + (target_y - start_y) * ratio
            wz = start_z * (1.0 - ratio)

            # 末段贴近地面，容差放宽
            xy_tol = 0.25 if i < steps else 0.40
            z_tol = 0.25 if i < steps else 0.40

            if not self._fly_to(wx, wy, wz, xy_tol=xy_tol, z_tol=z_tol, timeout=20.0):
                rospy.logwarn("[landing_executor_node] 45°下降段 %d/%d 不到位", i, steps)
                return False
            rospy.sleep(0.3)

        rospy.loginfo("[landing_executor_node] 45°下降轨迹完成")
        return True

    def _detect_pad_offset(self):
        """
        检测降落点偏移。

        Returns:
            tuple: (offset_x, offset_y)，未检测到返回 (0.0, 0.0)。
        """
        if self._current_image is None:
            return 0.0, 0.0

        try:
            height = 1.5
            if self._current_pose is not None:
                height = self._current_pose.position.z

            resp = self._detect_pad_client(self._current_image, height)
            if resp.detected:
                return float(resp.offset_x), float(resp.offset_y)
        except Exception as e:
            rospy.logwarn("[landing_executor_node] 降落点检测失败: %s", e)

        return 0.0, 0.0

    def run(self):
        """进入 ROS 主循环。"""
        rospy.spin()


def main():
    """节点入口函数。"""
    node = LandingExecutorNode()
    node.run()


if __name__ == "__main__":
    main()
