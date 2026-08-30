#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UAV public interface constants and utility functions.

This module centralizes all ROS nodes need:
- G-topic field geometry parameters
- ROS topic/service name constants
- Mission state/command/safety level/waypoint type enums
- Coordinate and yaw utility functions

Design principle:
- All constants provided as immutable data structures, avoiding runtime mutation.
- No ROS runtime dependency, can be safely imported by any node.
"""

from __future__ import print_function

import math


# ------------------------------------------------------------------------------
# G-topic field geometry parameters (unit: dm)
# ------------------------------------------------------------------------------
G_FIELD_WIDTH_DM = 48
G_FIELD_DEPTH_DM = 40
G_CELL_SIZE_DM = 8
G_PATROL_HEIGHT_DM = 14.0
G_FIRE_DESCEND_HEIGHT_DM = 10.0
G_FIRE_APPROACH_DIST_DM = 5.0


# ------------------------------------------------------------------------------
# ROS topic name constants
# ------------------------------------------------------------------------------
class TopicNames(object):
    """ROS topic name constants."""

    # Perception layer
    CAMERA_IMAGE_RAW = "/camera/image_raw"
    CAMERA_IMAGE_COMPRESSED = "/camera/image_compressed"
    CAMERA_CAMERA_INFO = "/camera/camera_info"
    PERCEPTION_FIRE_SOURCE = "/perception/fire_source"
    PERCEPTION_FIRE_DEBUG = "/perception/fire_source/debug"

    # D-topic perception
    PERCEPTION_QR_RESULT = "/perception/qr_result"
    PERCEPTION_TARGET_BUCKET = "/perception/target_bucket"
    PERCEPTION_TARGET_DEBUG = "/perception/target_bucket/debug"

    # D-topic planning
    PLANNING_GUIDE_PATH = "/planning/guide_path"

    # Localization layer
    LOCALIZATION_UAV_STATE = "/localization/uav_state"
    LOCALIZATION_LANDING_PAD = "/localization/landing_pad"

    # Planning layer
    PLANNING_GLOBAL_WAYPOINTS = "/planning/global_waypoints"
    PLANNING_LOCAL_WAYPOINTS = "/planning/local_waypoints"
    PLANNING_MISSION_WAYPOINTS = "/planning/mission_waypoints"
    PLANNING_MISSION_PROGRESS = "/planning/mission_progress"

    # Decision layer
    DECISION_MISSION_CMD = "/decision/mission_cmd"
    DECISION_MISSION_STATUS = "/decision/mission_status"
    DECISION_LASER_CMD = "/decision/laser_cmd"
    DECISION_LED_CMD = "/decision/led_cmd"
    DECISION_DROP_CMD = "/decision/drop_cmd"  # G-topic drop PWM command
    DECISION_TARGET_CONFIRMED = "/decision/target_confirmed"  # 决策层确认目标 (带格坐标/序号, 供地面站上报)

    # Execution layer
    EXECUTION_CMD_POSE = "/execution/cmd_pose"
    FLIGHT_MODE = "/FlightMode"
    PWM_CTRL = "/pwm_ctrl"
    BEEP_CTRL = "/beep_ctrl"
    YAW_CTRL = "/YawCtrl"
    FC_POSITION = "/fc_position"  # 飞控 LOCAL_POSITION_NED 转发 (独立话题, 不占用 /Odometry)
    FC_FLIGHT_MODE = "/fc_flight_mode"  # 飞控实际模式 (HEARTBEAT 解析, 供状态机判断遥控器接管)
    UAV_STATE = "/uav_state"
    RC_DATA = "/rc_data"
    IMU_DATA = "/zmofly/imu_data"
    ODOMETRY = "/Odometry"

    # Fire truck communication
    FIRETRUCK_CMD = "/firetruck/cmd"

    # Safety
    SAFETY_EVENT = "/safety/event"


class ServiceNames(object):
    """ROS service name constants."""

    GET_UAV_STATE = "/state_estimator/get_uav_state"
    GET_LOCAL_POSITION = "/state_estimator/get_local_position"
    DETECT_LANDING_PAD = "/landing_pad_locator/detect_landing_pad"


class ActionNames(object):
    """ROS action name constants."""

    TAKEOFF = "/execution/takeoff"


# ------------------------------------------------------------------------------
# Mission state enum
# ------------------------------------------------------------------------------
class MissionState(object):
    """Mission state machine state enum."""

    INIT = 0
    READY = 1
    TAKING_OFF = 2
    PATROLING = 3
    # G-topic firefighting states
    FIRE_FOUND = 20
    FIRE_APPROACHING = 21
    FIRE_DESCENDING = 22
    FIRE_DROPPING = 23
    FIRE_RESUMING = 24
    FIRE_DROP_RELEASE = 25
    LANDING = 7
    FINISHED = 8
    ERROR = 9
    EMERGENCY = 10

    @classmethod
    def to_string(cls, state):
        mapping = {
            cls.INIT: "INIT",
            cls.READY: "READY",
            cls.TAKING_OFF: "TAKING_OFF",
            cls.PATROLING: "PATROLING",
            cls.FIRE_FOUND: "FIRE_FOUND",
            cls.FIRE_APPROACHING: "FIRE_APPROACHING",
            cls.FIRE_DESCENDING: "FIRE_DESCENDING",
            cls.FIRE_DROPPING: "FIRE_DROPPING",
            cls.FIRE_RESUMING: "FIRE_RESUMING",
            cls.FIRE_DROP_RELEASE: "FIRE_DROP_RELEASE",
            cls.LANDING: "LANDING",
            cls.FINISHED: "FINISHED",
            cls.ERROR: "ERROR",
            cls.EMERGENCY: "EMERGENCY",
        }
        return mapping.get(state, "UNKNOWN")


class MissionCommand(object):
    """MissionCommand command enum."""

    NOP = 0
    ARM = 1
    TAKEOFF = 2
    PATROL = 3
    LAND = 7
    ABORT = 8
    RETURN = 9
    # G-topic firefighting commands
    FIRE_FOUND = 20
    FIRE_APPROACH = 21
    FIRE_DESCEND = 22
    FIRE_DROP = 23
    RESUME_PATROL = 24


class SafetyLevel(object):
    """SafetyEvent safety level enum."""

    INFO = 0
    WARN = 1
    ERROR = 2
    CRITICAL = 3


class WaypointType(object):
    """Waypoint type enum."""

    HOVER_DETECT = 0
    PATROL = 1
    LAND = 4
    # G-topic firefighting waypoint types
    FIRE_APPROACH = 6
    FIRE_DESCEND = 7
    FIRE_DROP = 8


class FireTruckCmd(object):
    """Fire truck communication command enum."""

    POS_REPORT = 0
    FIRE_ALERT = 1
    STATUS_QUERY = 2


# ------------------------------------------------------------------------------
# Macro cell coordinate utility functions
# ------------------------------------------------------------------------------
def macro_cell_to_xy_dm(col, row):
    """
    Macro cell (col, row) -> center coordinate (dm).

    Args:
        col (int): column 0..5.
        row (int): row 0..4.

    Returns:
        tuple: (x_dm, y_dm).
    """
    x = col * G_CELL_SIZE_DM
    y = row * G_CELL_SIZE_DM
    return x, y


def xy_to_macro_cell(x_dm, y_dm):
    """
    World coordinate (dm) -> macro cell (col, row).

    Args:
        x_dm (float): X coordinate (dm).
        y_dm (float): Y coordinate (dm).

    Returns:
        tuple: (col, row).
    """
    col = int(x_dm // G_CELL_SIZE_DM)
    row = int(y_dm // G_CELL_SIZE_DM)
    col = max(0, min(5, col))
    row = max(0, min(4, row))
    return col, row


# ------------------------------------------------------------------------------
# Utility functions
# ------------------------------------------------------------------------------
def normalize_yaw(yaw_deg):
    """
    Normalize yaw to [-180, 180).

    Args:
        yaw_deg (float): yaw angle (degrees).

    Returns:
        float: normalized yaw (degrees).
    """
    while yaw_deg >= 180.0:
        yaw_deg -= 360.0
    while yaw_deg < -180.0:
        yaw_deg += 360.0
    return yaw_deg


def yaw_diff(target_deg, current_deg):
    """
    Compute minimum difference between two yaw angles.

    Args:
        target_deg (float): target yaw (degrees).
        current_deg (float): current yaw (degrees).

    Returns:
        float: minimum difference (degrees), range [-180, 180].
    """
    diff = normalize_yaw(target_deg - current_deg)
    return diff


def point_distance_2d(p1, p2):
    """
    Compute 2D Euclidean distance.

    Args:
        p1 (tuple): (x1, y1).
        p2 (tuple): (x2, y2).

    Returns:
        float: 2D distance.
    """
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def is_position_arrived(current, target, xy_tol=0.10, z_tol=0.15):
    """
    Check if target waypoint has been reached.

    Args:
        current (tuple): current position (x, y, z), meters.
        target (tuple): target position (x, y, z), meters.
        xy_tol (float, optional): horizontal tolerance (m), default 0.10.
        z_tol (float, optional): vertical tolerance (m), default 0.15.

    Returns:
        bool: whether arrived.
    """
    dx = abs(current[0] - target[0])
    dy = abs(current[1] - target[1])
    dz = abs(current[2] - target[2])
    return dx <= xy_tol and dy <= xy_tol and dz <= z_tol


def pixel_to_meter(offset_px, height_m, focal_length_px=500.0):
    """
    Convert pixel offset to real-world meter offset using similar triangles.

    At height_m above ground, each pixel represents height_m / focal_length_px meters.

    Args:
        offset_px (float): pixel offset from image center.
        height_m (float): current UAV height above ground (meters).
        focal_length_px (float, optional): camera focal length in pixels, default 500.

    Returns:
        float: real-world offset in meters.
    """
    if height_m <= 0.0:
        return 0.0
    return float(offset_px) * height_m / float(focal_length_px)

# ==============================================================================
# D-topic (Search & Rescue) field geometry parameters
# ==============================================================================
D_FIELD_WIDTH_DM = 50        # X 方向 50dm (10 格 x 5dm)
D_FIELD_DEPTH_DM = 40        # Y 方向 40dm (8 格 x 5dm)
D_CELL_SIZE_DM = 5           # 每格 5dm
D_GRID_COLS = 10             # X0..X9
D_GRID_ROWS = 8              # Y0..Y7
D_FLIGHT_HEIGHT_M = 1.0      # 起飞/巡逻/引导统一高度
D_DROP_HEIGHT_M = 0.5        # 投放高度 <1m (0.5m 低空投放, 落点更准)
D_DROP_DESCEND_TOLERANCE_M = 0.2  # 到达投放高度容差


# 禁飞区 20 格 (搜索可飞越, 引导禁止飞越) — 双模式规则
NO_FLY_ZONES = [
    (5, 0),
    (2, 1), (3, 1), (8, 1), (9, 1),
    (0, 2), (5, 2), (6, 2), (8, 2),
    (2, 3),
    (0, 4), (4, 4), (8, 4),
    (1, 5), (3, 5), (6, 5),
    (6, 6),
    (1, 7), (4, 7), (8, 7),
]
NO_FLY_ZONE_SET = frozenset(NO_FLY_ZONES)


class DMissionState(object):
    """D 题搜救任务状态枚举。"""

    INIT = 0
    READY = 1
    TAKING_OFF = 2          # 爬升 + QR 解码
    PATROLING = 3           # 蛇形搜索
    TARGET_FOUND = 30       # 发现目标
    DROP_APPROACHING = 31   # 飞到目标上方
    DROP_DESCENDING = 32    # 下降到投放高度
    DROP_COLOR_CHECK = 33   # 颜色二次确认
    DROP_RELEASING = 34     # 释放物块
    DROP_RESUMING = 35      # 升回 1.0m 恢复搜索
    ALL_DROPS_DONE = 36     # 3 个全部投完
    PATH_CALCULATING = 37   # A* 计算最远目标路径
    GUIDE_STARTING = 38     # 飞到最远目标上方, 激光 ON
    GUIDE_FLYING = 39       # 沿 A* 路径逐格引导
    GUIDE_FINISHING = 40    # 到达出口格, 激光 OFF
    LANDING = 7
    FINISHED = 8
    ERROR = 9
    EMERGENCY = 10

    @classmethod
    def to_string(cls, state):
        mapping = {
            cls.INIT: "INIT", cls.READY: "READY", cls.TAKING_OFF: "TAKING_OFF",
            cls.PATROLING: "PATROLING", cls.TARGET_FOUND: "TARGET_FOUND",
            cls.DROP_APPROACHING: "DROP_APPROACHING",
            cls.DROP_DESCENDING: "DROP_DESCENDING",
            cls.DROP_COLOR_CHECK: "DROP_COLOR_CHECK",
            cls.DROP_RELEASING: "DROP_RELEASING",
            cls.DROP_RESUMING: "DROP_RESUMING",
            cls.ALL_DROPS_DONE: "ALL_DROPS_DONE",
            cls.PATH_CALCULATING: "PATH_CALCULATING",
            cls.GUIDE_STARTING: "GUIDE_STARTING",
            cls.GUIDE_FLYING: "GUIDE_FLYING",
            cls.GUIDE_FINISHING: "GUIDE_FINISHING",
            cls.LANDING: "LANDING", cls.FINISHED: "FINISHED",
            cls.ERROR: "ERROR", cls.EMERGENCY: "EMERGENCY",
        }
        return mapping.get(state, "UNKNOWN")


class TargetColor(object):
    """目标桶颜色枚举。"""
    RED = 0
    GREEN = 1
    BLUE = 2

    @classmethod
    def to_string(cls, c):
        return {0: "RED", 1: "GREEN", 2: "BLUE"}.get(c, "UNKNOWN")


class DWaypointType(object):
    """D 题航点类型枚举。"""
    SEARCH = 1              # 蛇形搜索点
    DROP_APPROACH = 10      # 投放接近点
    DROP_DESCEND = 11       # 投放下降点
    GUIDE = 20              # 引导航点
    LAND = 4                # 降落点


def grid_to_xy_dm(gx, gy):
    """D 题格子坐标 (gx, gy) -> 格中心世界坐标 (dm)。

    原点 (X0,Y0) 格中心 = (0,0) dm。
    """
    x = gx * D_CELL_SIZE_DM
    y = gy * D_CELL_SIZE_DM
    return x, y


def xy_to_grid_d(x_dm, y_dm):
    """D 题世界坐标 (dm) -> 格子坐标 (gx, gy)。"""
    gx = int(round(x_dm / D_CELL_SIZE_DM))
    gy = int(round(y_dm / D_CELL_SIZE_DM))
    gx = max(0, min(D_GRID_COLS - 1, gx))
    gy = max(0, min(D_GRID_ROWS - 1, gy))
    return gx, gy


def is_no_fly_zone(gx, gy):
    """判断格子是否为禁飞区。"""
    return (gx, gy) in NO_FLY_ZONE_SET


if __name__ == "__main__":
    print("G topic field: {}dm x {}dm".format(G_FIELD_WIDTH_DM, G_FIELD_DEPTH_DM))
    print("macro cell col=2, row=3 center: {}".format(macro_cell_to_xy_dm(2, 3)))
    print("coord (22.0, 36.0) -> macro cell: {}".format(xy_to_macro_cell(22.0, 36.0)))
    print("pixel_to_meter(100px, h=1.5m): {:.3f}m".format(pixel_to_meter(100, 1.5)))
