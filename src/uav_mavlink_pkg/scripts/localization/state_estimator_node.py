#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
状态估计节点。

职责：
- 订阅 FAST-LIO /Odometry、飞控 /zmofly/imu_data、RC /rc_data。
- 融合并发布 /localization/uav_state（10Hz）。
- 提供 get_uav_state、get_local_position 查询服务。

约束：
- 定位层只发布状态，不发布控制指令，不订阅决策层状态。
"""

import math
import sys
import os

import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt64MultiArray
from geometry_msgs.msg import Twist

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_COMMON_DIR = os.path.join(_SCRIPT_DIR, "..", "common")
if _COMMON_DIR not in sys.path:
    sys.path.insert(0, _COMMON_DIR)

from uav_interfaces import TopicNames, ServiceNames
from uav_mavlink_pkg.msg import UavState
from uav_mavlink_pkg.srv import GetUavState, GetUavStateResponse
from uav_mavlink_pkg.srv import GetLocalPosition, GetLocalPositionResponse


class StateEstimatorNode(object):
    """无人机状态估计节点。"""

    def __init__(self):
        """
        初始化状态估计节点。

        创建订阅者、发布者与服务，缓存最新位姿、IMU、RC 数据。
        """
        rospy.init_node("state_estimator_node")

        # 参数
        self._publish_rate = rospy.get_param("~publish_rate", 10.0)
        self._rc_timeout = rospy.get_param("~rc_timeout", 3.0)
        self._battery_warning = rospy.get_param("~battery_warning", 10.5)

        # 状态缓存
        self._current_pose = None
        self._current_twist = None
        self._current_yaw_deg = 0.0
        self._fc_z = None           # 飞控 z (高度更准, 实测比雷达准)
        self._last_rc_time = rospy.Time(0)
        self._rc_connected = False
        self._battery_voltage = 0.0
        self._flight_mode = "UNKNOWN"
        self._is_localized = False

        # 发布者
        self._state_pub = rospy.Publisher(
            TopicNames.LOCALIZATION_UAV_STATE, UavState, queue_size=1
        )

        # 订阅者: 雷达 /Odometry (XY+yaw 准) + 飞控 /fc_position (Z 准)
        self._odom_sub = rospy.Subscriber(
            TopicNames.ODOMETRY, Odometry, self._odometry_callback, queue_size=10
        )
        self._fc_pos_sub = rospy.Subscriber(
            TopicNames.FC_POSITION, Odometry, self._fc_position_callback, queue_size=10
        )
        self._imu_sub = rospy.Subscriber(
            TopicNames.IMU_DATA, Imu, self._imu_callback, queue_size=10
        )
        self._rc_sub = rospy.Subscriber(
            TopicNames.RC_DATA, UInt64MultiArray, self._rc_callback, queue_size=10
        )

        # 服务
        self._get_state_srv = rospy.Service(
            "~get_uav_state", GetUavState, self._handle_get_uav_state
        )
        self._get_local_srv = rospy.Service(
            "~get_local_position", GetLocalPosition, self._handle_get_local_position
        )

        rospy.loginfo("[state_estimator_node] 初始化完成 (XY:雷达 /Odometry, Z:飞控 /fc_position)")

    def _odometry_callback(self, msg):
        """雷达里程计回调 (FAST-LIO): 取 XY + yaw + twist, Z 用飞控覆盖。

        Args:
            msg (nav_msgs.msg.Odometry): FAST-LIO 位姿 (XY/yaw 准)。
        """
        self._current_pose = msg.pose.pose
        self._current_twist = msg.twist.twist
        self._is_localized = True

        q = msg.pose.pose.orientation
        try:
            euler = tf.transformations.euler_from_quaternion(
                [q.x, q.y, q.z, q.w]
            )
            self._current_yaw_deg = math.degrees(euler[2])
        except Exception as e:
            rospy.logwarn("[state_estimator_node] 四元数转欧拉角失败: %s", e)

    def _fc_position_callback(self, msg):
        """飞控位置回调: 取 Z 轴 (高度比雷达准, 实测验证)。

        Args:
            msg (nav_msgs.msg.Odometry): 飞控 LOCAL_POSITION_NED 转发。
        """
        self._fc_z = msg.pose.pose.position.z

    def _imu_callback(self, msg):
        """
        IMU 回调（可选用于电池/模式扩展）。

        Args:
            msg (sensor_msgs.msg.Imu): 飞控 IMU 数据。
        """
        # IMU 目前仅用于健康检查；电池电压由飞控其他消息提供时可扩展
        pass

    def _rc_callback(self, msg):
        """
        RC 通道回调。

        Args:
            msg (std_msgs.msg.UInt64MultiArray): 8 通道 RC 数据。
        """
        self._last_rc_time = rospy.Time.now()
        self._rc_connected = True

    def _build_uav_state(self):
        """
        构建 UavState 消息。

        Returns:
            UavState: 当前融合状态。
        """
        state = UavState()
        state.header.stamp = rospy.Time.now()
        state.header.frame_id = "world"

        if self._current_pose is not None:
            state.pose = self._current_pose
            # Z 轴用飞控数据覆盖 (实测飞控高度更准)
            if self._fc_z is not None:
                state.pose.position.z = self._fc_z
        else:
            state.pose.orientation.w = 1.0

        if self._current_twist is not None:
            state.twist = self._current_twist
        else:
            state.twist = Twist()

        state.yaw_deg = self._current_yaw_deg
        state.is_localized = self._is_localized
        state.flight_mode = self._flight_mode
        state.battery_voltage = self._battery_voltage

        # RC 超时检测
        if self._rc_connected:
            dt = (rospy.Time.now() - self._last_rc_time).to_sec()
            state.rc_connected = dt < self._rc_timeout
        else:
            state.rc_connected = False

        return state

    def _handle_get_uav_state(self, _req):
        """
        查询当前 UAV 状态服务。

        Args:
            _req (GetUavStateRequest): 空请求。

        Returns:
            GetUavStateResponse: 当前状态。
        """
        resp = GetUavStateResponse()
        resp.state = self._build_uav_state()
        return resp

    def _handle_get_local_position(self, _req):
        """
        查询局部位置服务。

        复用旧代码 uav_library.get_local 的 wait_for_message + 重试逻辑，
        获取失败时回退到缓存的最新位姿。

        Args:
            _req (GetLocalPositionRequest): 空请求。

        Returns:
            GetLocalPositionResponse: 当前 (x, y, z, valid)。
        """
        resp = GetLocalPositionResponse()

        for attempt in range(3):
            try:
                msg = rospy.wait_for_message(
                    TopicNames.ODOMETRY, Odometry, timeout=3.0
                )
                resp.x = msg.pose.pose.position.x
                resp.y = msg.pose.pose.position.y
                # Z 用飞控数据 (实测更准)
                resp.z = self._fc_z if self._fc_z is not None else msg.pose.pose.position.z
                resp.valid = True
                # 同时刷新缓存
                self._current_pose = msg.pose.pose
                self._is_localized = True
                return resp
            except rospy.ROSException:
                rospy.logwarn(
                    "[state_estimator_node] 等待 /Odometry 超时(第%d次)",
                    attempt + 1
                )

        # 3 次超时，回退缓存
        rospy.logerr("[state_estimator_node] 3 次超时，返回缓存位置")
        if self._current_pose is not None:
            resp.x = self._current_pose.position.x
            resp.y = self._current_pose.position.y
            # Z 用飞控数据
            resp.z = self._fc_z if self._fc_z is not None else self._current_pose.position.z
            resp.valid = self._is_localized
        else:
            resp.x = resp.y = resp.z = 0.0
            resp.valid = False
        return resp

    def run(self):
        """进入 ROS 主循环，以固定频率发布状态。"""
        rate = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():
            self._state_pub.publish(self._build_uav_state())
            rate.sleep()


def main():
    """节点入口函数。"""
    node = StateEstimatorNode()
    node.run()


if __name__ == "__main__":
    main()
