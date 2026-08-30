#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D 题执行层: 激光笔 + 3 通道按色投放 (PWM 集中节点)。

框架规范 (框架设计文档 §5.6):
- 激光、投放都是 PWM 舵机信号, 共用同一 MAVLink 下发通道。
- 集中在本节点 (执行层) 控制 PWM, 避免多节点同时写 PWM 冲突。
- 决策层 (状态机/behavior_selector) 只发 /decision/*_cmd 逻辑指令, 不直接碰硬件。

通道映射 (D 题最终方案):
- ch1: 红色物块投放舵机
- ch2: 绿色物块投放舵机
- ch3: 蓝色物块投放舵机
- ch4: 激光笔开关 (2500=开, 500=关, 飞控侧真关断)

订阅:
- /decision/laser_cmd (Int8): 1=ON, 0=OFF
- /decision/drop_cmd  (Int8): 0=红, 1=绿, 2=蓝 (释放对应色); -1=收起全部
"""

import sys
import os
import rospy
from std_msgs.msg import Int8, UInt64MultiArray

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_COMMON_DIR = os.path.join(_SCRIPT_DIR, "..", "common")
if _COMMON_DIR not in sys.path:
    sys.path.insert(0, _COMMON_DIR)
from uav_interfaces import TopicNames, TargetColor


class SimpleLaser(object):
    """激光笔开关控制。"""

    def __init__(self, ch_laser, on_us=2500, off_us=500, pwm_pub=None):
        self.ch_laser = int(ch_laser)
        self.on_us = int(on_us)
        self.off_us = int(off_us)
        self._pwm_pub = pwm_pub

    def _set_pwm(self, channel, us):
        if self._pwm_pub is None:
            return
        msg = UInt64MultiArray()
        msg.data = [int(channel), int(us)]
        self._pwm_pub.publish(msg)

    def on(self):
        self._set_pwm(self.ch_laser, self.on_us)

    def off(self):
        self._set_pwm(self.ch_laser, self.off_us)


class DropServoTriChannel(object):
    """3 通道按色投放舵机控制 (ch1红/ch2绿/ch3蓝)。"""

    def __init__(self, color_channels, retract_us, release_us, release_hold_s, pwm_pub=None):
        self.color_channels = {int(k): int(v) for k, v in color_channels.items()}
        self.retract_us = int(retract_us)
        self.release_us = int(release_us)
        self.release_hold_s = float(release_hold_s)
        self._pwm_pub = pwm_pub

    def _set_pwm(self, channel, us):
        if self._pwm_pub is None:
            return
        msg = UInt64MultiArray()
        msg.data = [int(channel), int(us)]
        self._pwm_pub.publish(msg)

    def retract_all(self):
        for ch in self.color_channels.values():
            self._set_pwm(ch, self.retract_us)

    def release_color(self, color):
        """释放指定颜色物块, 延时后自动收起。"""
        if color not in self.color_channels:
            rospy.logwarn("[gimbal_executor_node] 未知颜色码: %d", color)
            return
        ch = self.color_channels[color]
        self._set_pwm(ch, self.release_us)
        rospy.loginfo(
            "[gimbal_executor_node] 投放 color=%s -> ch%d release",
            TargetColor.to_string(color), ch,
        )
        # 延时收起 (避免舵机持续受力)
        rospy.Timer(
            rospy.Duration(self.release_hold_s),
            self._retract_callback(ch),
            oneshot=True,
        )

    def _retract_callback(self, ch):
        def _cb(_event):
            self._set_pwm(ch, self.retract_us)
            rospy.loginfo("[gimbal_executor_node] ch%d 收起", ch)
        return _cb


class GimbalExecutorNode(object):
    """D 题执行层: 激光笔 + 3 通道投放 (PWM 集中节点)。"""

    def __init__(self):
        rospy.init_node("gimbal_executor_node")

        # ---- 激光笔参数 ----
        self._ch_laser = int(rospy.get_param("~laser_ch_laser", 4))
        self._on_us = int(rospy.get_param("~laser_on_us", 2500))
        self._off_us = int(rospy.get_param("~laser_off_us", 500))

        # ---- 投放舵机参数 ----
        self._color_channels = {
            TargetColor.RED: int(rospy.get_param("~ch_red", 1)),
            TargetColor.GREEN: int(rospy.get_param("~ch_green", 2)),
            TargetColor.BLUE: int(rospy.get_param("~ch_blue", 3)),
        }
        self._retract_us = int(rospy.get_param("~retract_us", 1500))
        self._release_us = int(rospy.get_param("~release_us", 2500))
        self._release_hold_s = float(rospy.get_param("~release_hold_s", 1.0))

        # 发布者 (唯一 PWM 发布点, 框架 §5.6)
        self._pwm_pub = rospy.Publisher(
            TopicNames.PWM_CTRL, UInt64MultiArray, queue_size=10
        )

        # 设备
        self._laser = SimpleLaser(
            ch_laser=self._ch_laser,
            on_us=self._on_us, off_us=self._off_us,
            pwm_pub=self._pwm_pub,
        )
        self._drop = DropServoTriChannel(
            color_channels=self._color_channels,
            retract_us=self._retract_us,
            release_us=self._release_us,
            release_hold_s=self._release_hold_s,
            pwm_pub=self._pwm_pub,
        )

        # 订阅 (逻辑指令, 来自决策层)
        self._laser_sub = rospy.Subscriber(
            TopicNames.DECISION_LASER_CMD, Int8,
            self._laser_cmd_callback, queue_size=10
        )
        self._drop_sub = rospy.Subscriber(
            TopicNames.DECISION_DROP_CMD, Int8,
            self._drop_cmd_callback, queue_size=10
        )

        # 启动时: 关激光 + 收起全部舵机
        self._laser.off()
        self._drop.retract_all()

        rospy.loginfo(
            "[gimbal_executor_node] D 题执行器初始化完成 laser_ch=%d drop_channels=%s",
            self._ch_laser, self._color_channels,
        )

    def _laser_cmd_callback(self, msg):
        cmd = int(msg.data)
        if cmd:
            self._laser.on()
            rospy.loginfo("[gimbal_executor_node] 激光笔 ON")
        else:
            self._laser.off()
            rospy.loginfo("[gimbal_executor_node] 激光笔 OFF")

    def _drop_cmd_callback(self, msg):
        """drop_cmd: 0/1/2=释放对应色, -1=收起全部。"""
        color = int(msg.data)
        if color < 0:
            self._drop.retract_all()
            rospy.loginfo("[gimbal_executor_node] 收起全部舵机")
        else:
            self._drop.release_color(color)

    def run(self):
        rospy.spin()


def main():
    node = GimbalExecutorNode()
    node.run()


if __name__ == "__main__":
    main()
