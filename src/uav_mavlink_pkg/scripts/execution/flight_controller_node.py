#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
飞控接口节点。

职责：
- 建立并维护与 ZMO-M4 飞控的 MAVLink 串口连接。
- 订阅 /execution/cmd_pose 发送位置目标。
- 订阅 /FlightMode 切换飞行模式并解锁（GUIDED 模式时自动解锁）。
- 订阅 /YawCtrl 发送偏航速度指令。
- 发布 /fc_position（D题: 原 /Odometry 改到 /fc_position, 避免与 FAST-LIO 冲突）、/zmofly/uav_pose_data、/zmofly/imu_data、/rc_data、/uav_state。
- 发布 /fc_flight_mode（HEARTBEAT.custom_mode 解析, 供状态机判断遥控器接管）。
- 初始化完成后蜂鸣三声（auto_start 就绪提示）。
- 提供 /execution/takeoff Action Server。

约束：
- 执行层只做指令下发与状态反馈，不做规划/决策。
- 所有 MAVLink 发送操作必须加锁，防止多回调同时写串口。
"""

import math
import sys
import os
import threading
import time

import rospy
import tf
import actionlib
from pymavlink import mavutil
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8, UInt64MultiArray, Float32MultiArray

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_COMMON_DIR = os.path.join(_SCRIPT_DIR, "..", "common")
if _COMMON_DIR not in sys.path:
    sys.path.insert(0, _COMMON_DIR)

from uav_interfaces import TopicNames, ActionNames
from uav_mavlink_pkg.msg import TakeoffAction, TakeoffResult, TakeoffFeedback


try:
    import numpy as np
except ImportError:
    np = None


class FlightControllerNode(object):
    """MAVLink 飞控接口节点。"""

    def __init__(self):
        """
        初始化飞控接口节点。

        读取串口参数，创建 ROS 订阅者/发布者/Action Server，并启动 MAVLink 接收线程。
        """
        rospy.init_node("flight_controller_node")

        # 参数
        self._serial_port = rospy.get_param("~serial_port", "/dev/ttyXXX")  # 按实际串口设备修改
        self._baud = int(rospy.get_param("~baud", 230400))
        self._target_system = int(rospy.get_param("~target_system", 1))
        self._target_component = int(rospy.get_param("~target_component", 1))
        self._coordinate_frame = int(rospy.get_param("~coordinate_frame", 9))
        self._type_mask = int(rospy.get_param("~type_mask", 1479))
        self._home_z = float(rospy.get_param("~home_z", 1.0))
        self._takeoff_timeout = float(rospy.get_param("~takeoff_timeout", 30.0))
        self._arm_timeout = float(rospy.get_param("~arm_timeout", 10.0))

        # 状态
        self._init_yaw_deg = 0.0
        self._current_yaw_deg = 0.0
        self._home_x = 0.0
        self._home_y = 0.0
        self._connection = None
        self._mavlink_lock = threading.Lock()
        self._last_msg_time = time.time()
        self._reconnect_count = 0
        self._running = True

        # ROS 发布者
        self._rc_pub = rospy.Publisher(
            TopicNames.RC_DATA, UInt64MultiArray, queue_size=10
        )
        self._imu_pub = rospy.Publisher(
            TopicNames.IMU_DATA, Imu, queue_size=1, latch=True
        )
        self._uwb_pub = rospy.Publisher(
            "/zmofly/uwb_data", Odometry, queue_size=1, latch=True
        )
        self._uav_pose_pub = rospy.Publisher(
            TopicNames.FC_POSITION, Odometry, queue_size=1, latch=True
        )
        self._fc_mode_pub = rospy.Publisher(
            TopicNames.FC_FLIGHT_MODE, Int8, queue_size=1, latch=True
        )
        self._state_pub = rospy.Publisher(
            TopicNames.UAV_STATE, Int8, queue_size=1, latch=True
        )

        # ROS 订阅者
        self._cmd_pose_sub = rospy.Subscriber(
            TopicNames.EXECUTION_CMD_POSE, PoseStamped,
            self._cmd_pose_callback, queue_size=1
        )
        self._flight_mode_sub = rospy.Subscriber(
            TopicNames.FLIGHT_MODE, Int8,
            self._flight_mode_callback, queue_size=100
        )
        self._yaw_ctrl_sub = rospy.Subscriber(
            TopicNames.YAW_CTRL, Float32MultiArray,
            self._yaw_ctrl_callback, queue_size=100
        )
        self._reboot_sub = rospy.Subscriber(
            "/FlightReboot", Int8, self._reboot_callback, queue_size=1
        )
        self._pwm_ctrl_sub = rospy.Subscriber(
            TopicNames.PWM_CTRL, UInt64MultiArray,
            self._pwm_ctrl_callback, queue_size=100
        )
        self._beep_ctrl_sub = rospy.Subscriber(
            TopicNames.BEEP_CTRL, UInt64MultiArray,
            self._beep_ctrl_callback, queue_size=100
        )
        self._odom_sub = rospy.Subscriber(
            TopicNames.ODOMETRY, Odometry,
            self._odometry_callback, queue_size=5
        )

        # Action Server
        self._takeoff_server = actionlib.SimpleActionServer(
            ActionNames.TAKEOFF, TakeoffAction,
            self._execute_takeoff, auto_start=False
        )
        self._takeoff_server.start()

        # 建立 MAVLink 连接
        self._connection = self._create_mavlink_connection()
        if self._connection is None:
            rospy.logerr("[flight_controller_node] 无法连接到飞控，退出")
            sys.exit(1)

        # 等待心跳
        self._wait_heartbeat()

        # 请求数据流
        self._request_data_streams()

        # 获取初始偏航
        self._init_yaw()

        # 启动接收线程
        self._rx_thread = threading.Thread(target=self._uav_topic_pub)
        self._rx_thread.daemon = True
        self._rx_thread.start()

        rospy.loginfo("[flight_controller_node] 初始化完成")

        # 就绪提示: 蜂鸣三声 (比赛 auto_start 时提示"飞控已就绪, 即将起飞")
        self._beep_ready_sequence()

    def _create_mavlink_connection(self):
        """
        创建 MAVLink 串口连接，带重试机制。

        Returns:
            mavutil.mavlink_connection or None: 成功返回连接对象，否则返回 None。
        """
        while not rospy.is_shutdown():
            try:
                conn = mavutil.mavlink_connection(
                    self._serial_port, self._baud, timeout=1
                )
                rospy.loginfo(
                    "[flight_controller_node] 串口 %s@%d 连接成功",
                    self._serial_port, self._baud
                )
                return conn
            except Exception as e:
                rospy.logwarn(
                    "[flight_controller_node] 串口连接失败: %s，3秒后重试...", e
                )
                rospy.sleep(3.0)
        return None

    def _wait_heartbeat(self):
        """等待飞控心跳，超时则报错退出。"""
        rospy.loginfo("[flight_controller_node] 等待飞控心跳...")
        try:
            self._connection.wait_heartbeat(timeout=10.0)
            rospy.loginfo("[flight_controller_node] 收到飞控心跳")
        except Exception as e:
            rospy.logerr("[flight_controller_node] 等待心跳超时或异常: %s", e)
            rospy.logerr("请检查: 1. 飞控是否上电 2. 串口线 3. 波特率")
            sys.exit(1)

    def _request_data_streams(self):
        """向飞控请求 RC、IMU、本地位置数据流。"""
        sensor_stream_id = 1
        rc_stream_id = 3
        sensor_rate = 100
        rc_rate = 10
        for _ in range(3):
            with self._mavlink_lock:
                self._connection.mav.request_data_stream_send(
                    self._target_system, 200, sensor_stream_id, sensor_rate, 1
                )
        for _ in range(4):
            with self._mavlink_lock:
                self._connection.mav.request_data_stream_send(
                    self._target_system, 200, rc_stream_id, rc_rate, 1
                )
        rospy.loginfo("[flight_controller_node] 已请求数据流")

    def _init_yaw(self):
        """从 /Odometry 获取初始偏航角，用于后续坐标旋转。"""
        try:
            msg = rospy.wait_for_message(TopicNames.ODOMETRY, Odometry, timeout=10.0)
            q = msg.pose.pose.orientation
            euler = tf.transformations.euler_from_quaternion(
                [q.x, q.y, q.z, q.w]
            )
            self._init_yaw_deg = math.degrees(euler[2])
            self._home_x = msg.pose.pose.position.x
            self._home_y = msg.pose.pose.position.y
            rospy.loginfo(
                "[flight_controller_node] 初始偏航角=%.2f°", self._init_yaw_deg
            )
        except Exception as e:
            rospy.logwarn("[flight_controller_node] 获取初始偏航失败: %s", e)
            self._init_yaw_deg = 0.0

    def _world_to_local(self, x, y):
        """
        将世界坐标绕初始偏航角旋转，得到飞控本地坐标。

        复用旧代码 uav_library.set_point 中的旋转逻辑。

        Args:
            x (float): 世界坐标 X。
            y (float): 世界坐标 Y。

        Returns:
            tuple: (X, Y) 飞控本地坐标。
        """
        rad = math.radians(self._init_yaw_deg * (-1.0))
        X = x * math.cos(rad) + y * math.sin(rad)
        Y = -1.0 * x * math.sin(rad) + y * math.cos(rad)
        return X, Y

    def _cmd_pose_callback(self, msg):
        """目标位置回调，直接透传世界坐标，与参考代码 data_to_uav.pose_callback 一致。"""
        p_x = msg.pose.position.x
        p_y = msg.pose.position.y
        p_z = msg.pose.position.z

        with self._mavlink_lock:
            now_time = 0
            self._connection.mav.set_position_target_local_ned_send(
                now_time, self._target_system, self._target_component,
                self._coordinate_frame, self._type_mask,
                p_x, p_y, p_z, 0, 0, 0, 0, 0, 0, 0, 0
            )

    def _odometry_callback(self, msg):
        """转发 FAST-LIO odometry 为 vision_position_estimate 给飞控 EKF 融合。"""
        if self._connection is None:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        roll, pitch, yaw = euler[0], euler[1], euler[2]

        with self._mavlink_lock:
            self._connection.mav.vision_position_estimate_send(
                0, x, y, z, roll, pitch, yaw,
                [0.0] * 21
            )

    def _flight_mode_callback(self, msg):
        """
        飞行模式切换回调。

        当目标模式为 GUIDED(4) 时，自动记录 home 点并解锁。

        Args:
            msg (std_msgs.msg.Int8): 模式编号。
        """
        mode_num = int(msg.data)
        rospy.loginfo("[flight_controller_node] 切换模式: %d", mode_num)

        with self._mavlink_lock:
            self._connection.mav.command_long_send(
                self._target_system, self._target_component, 176, 0,
                1, mode_num, 0, 0, 0, 0, 0
            )

        rospy.sleep(rospy.Duration(1, 10000000))

        try:
            ack = self._connection.messages.get("COMMAND_ACK")
            rospy.loginfo("[flight_controller_node] 模式切换 ACK: %s", ack)
        except Exception as e:
            rospy.logwarn("[flight_controller_node] 读取 ACK 异常: %s", e)

        if mode_num == 4:
            # GUIDED 模式：自动解锁并设置 home 位置
            self._arm_and_set_home()

    def _arm_and_set_home(self):
        """解锁飞控并发送 home 位置保持悬停。"""
        try:
            home_msg = rospy.wait_for_message(
                TopicNames.ODOMETRY, Odometry, timeout=5.0
            )
            self._home_x = home_msg.pose.pose.position.x
            self._home_y = home_msg.pose.pose.position.y
        except Exception as e:
            rospy.logwarn("[flight_controller_node] 获取 home 点失败: %s", e)

        with self._mavlink_lock:
            self._connection.mav.command_long_send(
                self._target_system, self._target_component, 400, 0,
                1, 0, 0, 0, 0, 0, 0
            )
            self._connection.mav.set_position_target_local_ned_send(
                0, self._target_system, self._target_component,
                self._coordinate_frame, self._type_mask,
                self._home_x, self._home_y, self._home_z, 0, 0, 0, 0, 0, 0, 0, 0
            )
        rospy.loginfo(
            "[flight_controller_node] 已解锁并设置 home 位置: (%.2f, %.2f, %.2f)",
            self._home_x, self._home_y, self._home_z
        )

    def _yaw_ctrl_callback(self, msg):
        """
        偏航速度控制回调。

        Args:
            msg (std_msgs.msg.Float32MultiArray): [x, y, z, target_yaw_speed]。
        """
        if len(msg.data) < 4:
            return
        yaw_x = msg.data[0]
        yaw_y = msg.data[1]
        yaw_z = msg.data[2]
        target_yaw = msg.data[3]
        if abs(target_yaw) < 0.5:
            return
        with self._mavlink_lock:
            self._connection.mav.set_position_target_local_ned_send(
                0, 0, 1, 9, 1479,
                yaw_x, yaw_y, yaw_z, 0, 0, 0, 0, 0, 0, 0, target_yaw
            )

    def _reboot_callback(self, msg):
        """
        飞控重启回调。

        Args:
            msg (std_msgs.msg.Int8): msg.data == 1 时重启。
        """
        if int(msg.data) != 1:
            rospy.logwarn("[flight_controller_node] 无效重启参数")
            return
        rospy.logwarn("[flight_controller_node] 2秒后重启飞控")
        rospy.sleep(2.0)
        with self._mavlink_lock:
            self._connection.mav.command_long_send(
                self._target_system, self._target_component, 246, 0,
                1, 0, 0, 0, 0, 0, 0
            )

    def _pwm_ctrl_callback(self, msg):
        """
        PWM 舵机控制回调，转发到飞控 MAVLink 命令 183。

        复用旧代码 data_to_uav.py 的 pwm_callback 逻辑。

        Args:
            msg (std_msgs.msg.UInt64MultiArray): [channel, us]。
        """
        if len(msg.data) < 2:
            return
        if self._connection is None:
            return
        pwm_channel = 7 + int(msg.data[0])
        pwm_up_time = int(msg.data[1])
        with self._mavlink_lock:
            self._connection.mav.command_long_send(
                self._target_system, 0, 183, 0,
                pwm_channel, pwm_up_time, 0, 0, 0, 0, 0
            )
        rospy.loginfo(
            "[flight_controller_node] PWM ch=%d us=%d", pwm_channel, pwm_up_time
        )

    def _beep_ctrl_callback(self, msg):
        """
        蜂鸣器控制回调，转发到飞控 MAVLink 命令 31011。

        复用旧代码 data_to_uav.py 的 beep_callback 逻辑。

        Args:
            msg (std_msgs.msg.UInt64MultiArray): [enable, open]。
        """
        if len(msg.data) < 2:
            return
        beep_enable = int(msg.data[0])
        beep_open = int(msg.data[1])
        with self._mavlink_lock:
            self._connection.mav.command_long_send(
                self._target_system, self._target_component, 31011, 0,
                beep_enable, beep_open, 0, 0, 0, 0, 0
            )
        rospy.loginfo(
            "[flight_controller_node] beep enable=%d open=%d",
            beep_enable, beep_open
        )

    def _beep_ready_sequence(self):
        """初始化完成后蜂鸣三声 (节奏对齐 TAKING_OFF 起飞蜂鸣: 每 0.4s 一声)。"""
        for i in range(3):
            on_delay = 0.1 + i * 0.4
            off_delay = 0.3 + i * 0.4
            rospy.Timer(
                rospy.Duration(on_delay),
                lambda _e: self._beep_ctrl_callback(UInt64MultiArray(data=[1, 1])),
                oneshot=True,
            )
            rospy.Timer(
                rospy.Duration(off_delay),
                lambda _e: self._beep_ctrl_callback(UInt64MultiArray(data=[1, 0])),
                oneshot=True,
            )

    def _execute_takeoff(self, goal):
        """
        Takeoff Action 执行函数。

        Args:
            goal (uav_mavlink_pkg.msg.TakeoffGoal): 目标高度与容差。
        """
        target_height = float(goal.target_height)
        tolerance = float(goal.tolerance)
        result = TakeoffResult()
        feedback = TakeoffFeedback()

        # 切 GUIDED 模式（会自动解锁并设置 home）
        self._flight_mode_callback(Int8(data=4))

        start_time = rospy.get_time()
        rate = rospy.Rate(10)
        success = False

        while not rospy.is_shutdown():
            elapsed = rospy.get_time() - start_time
            if elapsed > self._takeoff_timeout:
                rospy.logwarn(
                    "[flight_controller_node] 起飞超时(%.0fs)", self._takeoff_timeout
                )
                break

            # 发送目标高度
            # (world_to_local removed - reference uses direct coordinates)
            with self._mavlink_lock:
                self._connection.mav.set_position_target_local_ned_send(
                    0, self._target_system, self._target_component,
                    self._coordinate_frame, self._type_mask,
                    self._home_x, self._home_y, target_height, 0, 0, 0, 0, 0, 0, 0, 0
                )

            # 读取当前高度
            current_z = self._get_current_z()
            feedback.current_height = current_z
            feedback.status = "climbing"
            self._takeoff_server.publish_feedback(feedback)

            if abs(current_z - target_height) <= tolerance:
                rospy.loginfo(
                    "[flight_controller_node] 起飞完成，当前高度=%.2fm", current_z
                )
                success = True
                break

            rate.sleep()

        result.success = success
        result.status = "done" if success else "timeout"
        if success:
            self._takeoff_server.set_succeeded(result)
        else:
            self._takeoff_server.set_aborted(result)

    def _get_current_z(self):
        """
        获取当前高度。

        Returns:
            float: 当前 Z 坐标，获取失败返回 0.0。
        """
        try:
            msg = rospy.wait_for_message(
                TopicNames.ODOMETRY, Odometry, timeout=0.5
            )
            return msg.pose.pose.position.z
        except Exception:
            return 0.0

    def _uav_topic_pub(self):
        """MAVLink 消息接收线程，解析并发布 RC/IMU/位置等话题。"""
        while not rospy.is_shutdown() and self._running:
            try:
                uav_msg = self._connection.recv_msg()
            except Exception as e:
                rospy.logwarn(
                    "[flight_controller_node] 串口读取异常: %s", e
                )
                uav_msg = None

            if uav_msg is None:
                if time.time() - self._last_msg_time > 3.0:
                    rospy.logerr(
                        "[flight_controller_node] 3秒未收到飞控消息，尝试重连..."
                    )
                    self._reconnect()
                continue

            self._reconnect_count = 0
            self._last_msg_time = time.time()

            msg_type = uav_msg.get_type()

            if msg_type == "RC_CHANNELS":
                self._publish_rc(uav_msg)
            elif msg_type == "LOCAL_POSITION_NED":
                self._publish_uav_pose(uav_msg)
            elif msg_type == "HIGHRES_IMU":
                self._publish_imu(uav_msg)
            elif msg_type == "HEARTBEAT":
                self._publish_fc_mode(uav_msg)

    def _publish_fc_mode(self, uav_msg):
        """发布飞控实际模式 (HEARTBEAT.custom_mode), 供状态机判断遥控器接管。

        ArduPilot custom_mode 约定: 0=Stabilize, 4=Guided, 9=Land, ...
        状态机据此判断: 如果飞控不在 GUIDED(4), 说明遥控器切走了, 停发 cmd_pose。

        Args:
            uav_msg: HEARTBEAT 消息。
        """
        try:
            mode = int(uav_msg.custom_mode)
            self._fc_mode_pub.publish(Int8(data=mode))
        except Exception:
            pass

    def _publish_rc(self, uav_msg):
        """
        发布 RC 通道数据。

        Args:
            uav_msg (pymavlink.mavutil.mavlink.MAVLink_rc_channels_message): RC 消息。
        """
        rc_channel_data = UInt64MultiArray()
        rc_channel_data.data = [
            uav_msg.chan1_raw, uav_msg.chan2_raw, uav_msg.chan3_raw,
            uav_msg.chan4_raw, uav_msg.chan5_raw, uav_msg.chan6_raw,
            uav_msg.chan7_raw, uav_msg.chan8_raw
        ]
        self._rc_pub.publish(rc_channel_data)

    def _publish_uav_pose(self, uav_msg):
        """
        发布飞控本地位置到 /fc_position 和 /zmofly/uav_pose_data。

        D题: 飞控位置改发 /fc_position, /Odometry 只留 FAST-LIO (避免两个源互相覆盖)。

        Args:
            uav_msg: LOCAL_POSITION_NED 消息。
        """
        uav_pose_data = Odometry()
        uav_pose_data.pose.pose.position.x = uav_msg.x / 100.0
        uav_pose_data.pose.pose.position.y = uav_msg.y / 100.0
        uav_pose_data.pose.pose.position.z = uav_msg.z / 100.0
        uav_pose_data.header.stamp = rospy.Time.now()
        uav_pose_data.header.frame_id = "world"

        self._uav_pose_pub.publish(uav_pose_data)

        uwb_data = Odometry()
        uwb_data.pose.pose.position.x = uav_msg.x / 100.0
        uwb_data.pose.pose.position.y = uav_msg.y / 100.0
        uwb_data.pose.pose.position.z = uav_msg.z / 100.0
        self._uwb_pub.publish(uwb_data)

    def _publish_imu(self, uav_msg):
        """
        发布 IMU 数据。

        Args:
            uav_msg: HIGHRES_IMU 消息。
        """
        imu_data = Imu()
        imu_data.header.frame_id = "zmofly_imu"
        imu_data.header.stamp = rospy.Time.now()
        imu_data.orientation.w = 1.0
        imu_data.orientation.x = 0.0
        imu_data.orientation.y = 0.0
        imu_data.orientation.z = 0.0
        imu_data.angular_velocity.x = uav_msg.xgyro
        imu_data.angular_velocity.y = -uav_msg.ygyro
        imu_data.angular_velocity.z = -uav_msg.zgyro
        imu_data.linear_acceleration.x = uav_msg.xacc
        imu_data.linear_acceleration.y = -uav_msg.yacc
        imu_data.linear_acceleration.z = uav_msg.zacc
        self._imu_pub.publish(imu_data)

    def _reconnect(self):
        """尝试重新建立 MAVLink 连接。"""
        try:
            self._connection.close()
        except Exception:
            pass
        self._connection = self._create_mavlink_connection()
        if self._connection:
            self._last_msg_time = time.time()
            rospy.loginfo("[flight_controller_node] 串口重连成功")
        else:
            rospy.logerr("[flight_controller_node] 串口重连失败")

    def run(self):
        """进入 ROS 主循环。"""
        rospy.spin()
        self._running = False


def main():
    """节点入口函数。"""
    node = FlightControllerNode()
    node.run()


if __name__ == "__main__":
    main()
