# 架构图集（Architecture Diagrams）

> 本文档用 mermaid 描述五层解耦框架的架构、消息流与任务状态机。
> 状态机为 2026 搜救场景参考实现的流转设计，作为编写新任务状态机的模板。
> 完整设计说明见 [框架设计文档.md](框架设计文档.md)。

---

## 1. 五层解耦架构

```mermaid
flowchart TB
    subgraph Perception["感知层 Perception · 按场景扩展"]
        CAM["camera_node<br>相机采集 / 预处理"]
        DET["xxx_detector_node<br>目标 / QR / 特征检测"]
    end
    subgraph Localization["定位层 Localization · 通用复用"]
        SE["state_estimator_node<br>状态估计融合：雷达 XY + 飞控 Z<br>发布 /localization/uav_state 10Hz"]
        TF["coordinate_transformer_node<br>TF: world → base_link → camera_link"]
        PAD["landing_pad_locator_node<br>停机坪视觉定位（Hough 黑圆）"]
    end
    subgraph Planning["规划层 Planning · 按场景扩展"]
        GP["global_planner_node<br>全局路径策略（蛇形 / 全覆盖 / A*）"]
        MP["mission_planner_node / local_planner_node<br>航点透传（latch）"]
    end
    subgraph Decision["决策层 Decision · 按场景扩展"]
        SM["mission_state_machine_node<br>任务状态机（10Hz Timer 驱动）<br>★ 全系统唯一飞行指令来源"]
        BS["behavior_selector_node<br>状态 → 行为映射"]
        MON["safety_monitor_node<br>任务级边界告警"]
    end
    subgraph Execution["执行层 Execution · 通用复用"]
        FC["flight_controller_node<br>MAVLink 桥接 · 10Hz 流式下行"]
        GE["gimbal_executor_node<br>PWM 舵机 / 激光集中控制"]
        LE["landing_executor_node<br>多段降落执行"]
        LB["led_beeper_node<br>蜂鸣器"]
    end
    subgraph Cross["横切层 · 通用复用"]
        COM["common/<br>uav_interfaces 接口常量枚举<br>path_planner 纯逻辑规划库"]
        SS["safety_supervisor_node<br>独立进程安全旁路"]
        REC["flight_recorder_node<br>rosbag + JSONL"]
    end

    Perception -- "检测结果（像素偏移 / 特征）" --> Decision
    Localization -- "UavState 10Hz（is_localized）" --> Decision
    Planning -- "WaypointList（latch=True）" --> Decision
    Decision -- "/execution/cmd_pose 10Hz 流式" --> Execution
    Execution -- "MAVLink" --> HW["飞控 / 舵机 / LED（硬件）"]

    Localization -. "直接订阅（旁路）" .-> SS
    SS -. "SafetyEvent(CRITICAL) → 状态机转紧急降落" .-> SM
    REC -. "订阅全链路话题" .-> Cross
```

**关键约束**：层间只经 ROS Topic/Service 通信；只有决策层能发飞行指令；感知层永不直接控制硬件。

---

## 2. 核心消息流（数据链路）

```mermaid
flowchart LR
    subgraph EXT["外部传感器"]
        FCLI["FAST-LIO 激光里程计"]
        CAM["下视相机"]
    end
    subgraph LOC["定位层"]
        SE["state_estimator_node"]
    end
    subgraph EXEC["执行层"]
        FC["flight_controller_node"]
        GE["gimbal_executor_node"]
    end
    subgraph DEC["决策层"]
        SM["mission_state_machine_node"]
        BS["behavior_selector_node"]
    end
    subgraph PERC["感知层"]
        PD["检测器节点组"]
    end
    subgraph SAFE["安全"]
        SS["safety_supervisor_node"]
    end

    FCLI -- "/Odometry（XY / yaw）" --> SE
    CAM -- "/camera/image_raw" --> PD
    PD -- "QRResult / TargetBucket（像素偏移）" --> SM

    FC -- "vision_position_estimate" --> FCU["飞控 FCU"]
    FCU -- "MAVLink 位置反馈" --> FC
    FC -- "/fc_position（Z / IMU / RC）" --> SE
    SE -- "/localization/uav_state 10Hz" --> SM

    GP["global_planner_node"] -- "WaypointList" --> SM
    SM -- "/execution/cmd_pose 10Hz 流式" --> FC
    FC -- "SET_POSITION_TARGET 10Hz" --> FCU

    SM -- "/decision/laser_cmd" --> BS
    SM -- "/decision/drop_cmd（颜色码）" --> GE
    BS -- "/pwm_ctrl" --> GE
    GE -- "MAV_CMD_DO_SET_SERVO" --> FCU

    SE -. 直接订阅 .-> SS
    SS -. "SafetyEvent(CRITICAL)" .-> SM
```

**10Hz 流式抗丢包**：`SET_POSITION_TARGET` 以 10Hz 持续发送，单帧丢失由 100ms 后的下一帧补偿——这是对旧框架「发一次等到达」断连问题的根治方案。

---

## 3. 安全链路（双层监控）

```mermaid
flowchart TD
    subgraph L4["决策层内部"]
        MON["safety_monitor_node<br>订阅 /mission_status + UavState<br>任务级告警（WARN）"]
        SM["mission_state_machine_node"]
    end
    SE["state_estimator_node<br>/localization/uav_state"]
    subgraph BYPASS["独立进程旁路"]
        SS["safety_supervisor_node<br>越界 / 低电压 / 遥控丢失 / 定位丢失<br>发布 SafetyEvent(CRITICAL)"]
    end
    SE -- "直接订阅（不经决策层）" --> SS
    SS -- "SafetyEvent(CRITICAL)" --> SM
    MON -- "任务级 SafetyEvent" --> SM
    SM -- "响应 CRITICAL → 转 LANDING 紧急降落" --> LAND["landing_executor"]
```

设计要点：`safety_supervisor` 与被监控的状态机**互不依赖**——状态机卡死时旁路监控依然存活；旁路本身不直接控制飞控，紧急动作仍由状态机统一执行，控制权单一。若状态机彻底卡死，由遥控器拨杆接管（非 GUIDED 模式下状态机自动停发指令让出控制权）。

---

## 4. 任务状态机参考实现（2026 搜救场景）

> 业务层示例，展示状态机的粒度与转移设计。编写新场景状态机时以此为模板。

```mermaid
stateDiagram-v2
    [*] --> INIT
    INIT --> READY: 航点 + 定位就绪
    READY --> TAKING_OFF: $START 指令 / auto_start
    TAKING_OFF --> PATROLING: 爬升→悬停→QR 解码出口<br>（15s 超时兜底默认出口）
    PATROLING --> TARGET_FOUND: 视觉目标确认（多帧 + 去重）
    TARGET_FOUND --> DROP_APPROACHING: 记录颜色/格号，蜂鸣
    DROP_APPROACHING --> DROP_DESCENDING: 到达目标上方（视觉伺服）
    DROP_DESCENDING --> DROP_COLOR_CHECK: 降到投放高度
    DROP_COLOR_CHECK --> DROP_RELEASING: 颜色确认
    DROP_RELEASING --> DROP_RESUMING: 按颜色选舵机通道投放
    DROP_RESUMING --> PATROLING: 未投完 → 断点续巡
    DROP_RESUMING --> ALL_DROPS_DONE: 全部投放完成
    ALL_DROPS_DONE --> PATH_CALCULATING
    PATH_CALCULATING --> GUIDE_STARTING: A* 选最远目标规划路径
    GUIDE_STARTING --> GUIDE_FLYING: 到达引导起点（激光关）
    GUIDE_FLYING --> GUIDE_FINISHING: 激光开，沿 A* 逐格飞行
    GUIDE_FINISHING --> LANDING: 到达出口格，激光关
    LANDING --> FINISHED: 4 阶段分段下降 + 黑圆视觉伺服
    FINISHED --> [*]
```

**防重复触发机制**：确认目标仅在 `PATROLING` 阶段进入投放流程；已投目标按格号 ±1 去重；恢复巡逻时重置航点计时，避免当前格被"超时跳过"。

---

## 5. 坐标系与场地规范（参考实现）

```
巡逻区域: 50dm × 40dm，网格 10列 × 8行（X0~X9 × Y0~Y7），每格 5dm × 5dm
原点: (X0, Y0) 格中心 = (0, 0) dm；起飞点 = 场地左下角
机头朝向: X+；机体左侧: Y+
飞行高度: 巡逻/引导统一 1.0m；投放 0.5m
内部单位: 路径规划用 dm，ROS 消息与地面站协议用 m（×0.1 换算）
```

---

*图集基于框架设计文档 v2.1 绘制，2026-08-30。*
