#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LED/Beeper actuator for G-task firefighting.
- PWM mode: reuses /pwm_ctrl topic via flight controller MAVLink cmd 183.
- A2(ch2): fire-warning LED (2500us=ON, 500us=OFF, tested on ZMO-M4).
- Beeper: reuses /beep_ctrl topic.
"""

import os
import sys
import threading
import time

import rospy
from std_msgs.msg import Int8, UInt64MultiArray, Bool

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_COMMON_DIR = os.path.join(_SCRIPT_DIR, "..", "common")
if _COMMON_DIR not in sys.path:
    sys.path.insert(0, _COMMON_DIR)

from uav_interfaces import TopicNames, DMissionState
from uav_mavlink_pkg.msg import MissionStatus


class LEDBase(object):
    def set_on(self): pass
    def set_off(self): pass


class LEDGPIO(LEDBase):
    def __init__(self, gpio):
        self.gpio = int(gpio)
        self._ready = False
        self._val_path = "/sys/class/gpio/gpio{}/value".format(self.gpio)
        try:
            if not os.path.exists("/sys/class/gpio/gpio{}".format(self.gpio)):
                with open("/sys/class/gpio/export", "w") as f:
                    f.write(str(self.gpio))
            with open("/sys/class/gpio/gpio{}/direction".format(self.gpio), "w") as f:
                f.write("out")
            self._ready = True
            rospy.loginfo("[led_beeper_node] GPIO%d ready", self.gpio)
        except Exception as e:
            rospy.logwarn("[led_beeper_node] GPIO%d init failed: %s", self.gpio, e)

    def _write(self, v):
        if not self._ready:
            return
        try:
            with open(self._val_path, "w") as f:
                f.write(str(v))
        except Exception as e:
            rospy.logwarn("[led_beeper_node] GPIO write failed: %s", e)

    def set_on(self):
        self._write(1)

    def set_off(self):
        self._write(0)


class LEDPWM(LEDBase):
    def __init__(self, channel, on_us=2500, off_us=500):
        self.channel = int(channel)
        self.on_us = int(on_us)
        self.off_us = int(off_us)
        self.pub = rospy.Publisher(
            TopicNames.PWM_CTRL, UInt64MultiArray, queue_size=1, latch=True
        )
        rospy.loginfo(
            "[led_beeper_node] PWM ch%d ready (via /pwm_ctrl)", self.channel
        )

    def _send(self, us):
        msg = UInt64MultiArray()
        msg.data = [self.channel, int(us)]
        self.pub.publish(msg)

    def set_on(self):
        self._send(self.on_us)

    def set_off(self):
        self._send(self.off_us)


class LEDNone(LEDBase):
    def set_on(self):
        rospy.loginfo("[led_beeper_node] (none) ON")

    def set_off(self):
        rospy.loginfo("[led_beeper_node] (none) OFF")


class LedBeeperNode(object):

    def __init__(self):
        rospy.init_node("led_beeper_node")

        self._led_mode = str(rospy.get_param("~led_mode", "pwm")).lower()
        self._led_gpio = int(rospy.get_param("~led_gpio", 7))
        self._led_pwm_channel = int(rospy.get_param("~led_pwm_channel", 2))
        self._led_on_us = int(rospy.get_param("~led_on_us", 2500))
        self._led_off_us = int(rospy.get_param("~led_off_us", 500))
        self._beep_interval = float(rospy.get_param("~beep_interval", 0.2))
        self._beep_count = int(rospy.get_param("~beep_count", 3))
        self._led_blink_rate = float(rospy.get_param("~led_blink_rate", 5.0))

        if self._led_mode == "gpio":
            self._led = LEDGPIO(self._led_gpio)
        elif self._led_mode == "pwm":
            self._led = LEDPWM(
                self._led_pwm_channel, self._led_on_us, self._led_off_us
            )
        else:
            self._led = LEDNone()

        self._beep_pub = rospy.Publisher(
            TopicNames.BEEP_CTRL, UInt64MultiArray, queue_size=10
        )

        self._last_state = DMissionState.INIT
        self._led_state = "off"
        self._blink_thread = None
        self._blink_stop = threading.Event()

        self._status_sub = rospy.Subscriber(
            TopicNames.DECISION_MISSION_STATUS, MissionStatus,
            self._status_callback, queue_size=10
        )
        self._led_sub = rospy.Subscriber(
            TopicNames.DECISION_LED_CMD, Int8,
            self._led_callback, queue_size=10
        )

        self._led.set_off()
        rospy.loginfo(
            "[led_beeper_node] init complete led_mode=%s", self._led_mode
        )

    def _status_callback(self, msg):
        state = int(msg.state)
        if state == DMissionState.FINISHED and self._last_state != DMissionState.FINISHED:
            self._beep_open(self._beep_interval, self._beep_count)
        self._last_state = state

    def _led_callback(self, msg):
        if msg.data:
            self._apply_led("blink")
        else:
            self._apply_led("off")

    def _apply_led(self, new_state):
        if new_state == self._led_state:
            return
        if self._led_state == "blink":
            self._blink_stop.set()
            self._blink_thread = None
        self._led_state = new_state

        if new_state == "on":
            self._led.set_on()
        elif new_state == "off":
            self._led.set_off()
        elif new_state == "blink":
            self._blink_stop.clear()
            self._blink_thread = threading.Thread(target=self._blink_loop)
            self._blink_thread.daemon = True
            self._blink_thread.start()

    def _blink_loop(self):
        period = 1.0 / max(self._led_blink_rate, 0.1)
        toggle = False
        while not rospy.is_shutdown() and not self._blink_stop.is_set():
            if toggle:
                self._led.set_on()
            else:
                self._led.set_off()
            toggle = not toggle
            time.sleep(period)
        self._led.set_off()

    def _beep_open(self, time_interval, frequency):
        if time_interval <= 0 or frequency <= 0:
            rospy.logwarn("[led_beeper_node] invalid beeper params")
            return

        rospy.loginfo("[led_beeper_node] beeper %d times interval=%.2fs", frequency, time_interval)

        def send(enable, open_val):
            msg = UInt64MultiArray()
            msg.data = [int(enable), int(open_val)]
            self._beep_pub.publish(msg)

        for _ in range(frequency):
            send(1, 1)
            rospy.sleep(time_interval)
            send(1, 0)
            rospy.sleep(time_interval)
        send(0, 0)

    def run(self):
        rospy.spin()


def main():
    node = LedBeeperNode()
    node.run()


if __name__ == "__main__":
    main()
