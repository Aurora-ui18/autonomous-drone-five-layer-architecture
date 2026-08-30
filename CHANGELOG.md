# 更新日志（CHANGELOG）

本项目遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/) 格式。

---

## [1.0.0] - 2026-08-30 · 首次开源发布

本仓库以独立开源项目形式发布「自主系统五层解耦架构」的**通用骨架**：

### 新增
- 五层解耦框架通用代码：公共层（`common`）、执行层（`execution`）、定位层（`localization`）、安全旁路（`safety`）、记录层（`logging`），共 12 个 Python 模块约 2900 行
- 全部自定义 ROS 接口：12 个消息（msg）、5 个服务（srv）、1 个动作（`Takeoff.action`）
- 最小骨架启动文件 `launch/skeleton_minimal.launch`（执行 + 定位 + 安全旁路 + 记录 5 节点）与通用参数示例 `config/skeleton_params.yaml`
- `CMakeLists.txt` / `package.xml` 面向开源仓库重整（安装清单与骨架节点对齐）
- 文档三件套：`README.md`（架构总览 / 快速开始 / 演进史）、`docs/architecture.md`（mermaid 图集）、`docs/框架设计文档.md` v2.1 与 `docs/使用说明.md` v2.2

---

## 框架演进史（跨五届赛事实飞迭代）

框架并非一次性设计，以下为历次实战迭代记录。每届任务均完成全流程实飞验证；各届代码规模与接口演进均为对仓库实际代码的统计。

### 2026 · 搜救飞行器（本科）—— 五层框架完善版（本仓库参考实现）
- 规模：29 个 .py / 约 6800 行 / 12 msg + 5 srv + 1 action
- 10Hz Timer 状态机 + 10Hz 流式 MAVLink 位姿命令，根治历史断连问题
- 新增 QR 出口解码、HSV + 白平衡三色目标检测（业务层）、A* 引导路径、三色投放、黑圆停机坪视觉伺服降落
- 双层安全监控成型：独立旁路 `safety_supervisor` 发布 CRITICAL 事件，状态机响应紧急降落

### 2023 · 空地协同智能消防系统 —— 五层框架演进
- 规模：26 个 .py / 约 4900 行 / 9 msg + 5 srv
- 新增车机协同通信节点、火源检测（业务层）、6×5 全覆盖巡逻
- 五层结构在空地协同场景下进一步验证与扩展

### 2024 · 立体货架盘点无人机系统 —— ⭐ 五层解耦成型
- 规模：23 个 .py / 约 5450 行 / 11 msg + 8 srv（首次出现自定义接口契约）
- **第一次整体重构**：平铺脚本（`demo.py` + 功能库 + 模型文件混放）拆分为 common / decision / execution / localization / perception / planning / safety / logging 分层结构
- 公共层接口集中管理（`uav_interfaces.py`）、安全旁路与记录层建立
- 28 届"全能库"的六种职责自此归位到各层专职节点（详见 README「一堂重构课」）

### 2025 · 野生动物巡查系统 —— 脚本式框架延续
- 规模：15 个 .py / 约 3300 行 / 无自定义接口
- 首次拆出地面站通信（`gs_comm`）与网格全覆盖路径规划（`path_planner`）独立模块，为五层化打下基础
- 仓库 `legacy/` 目录完整保留 28 届代码备份，见证逐届继承而非推倒重写

### 第 28 届 CRAIC 国赛 —— 框架雏形
- 规模：10 个 .py / 约 3800 行 / 无自定义接口
- 脚本式实现：主流程全部集中在 `demo.py`，864 行巨石库 `uav_library.py` 同时承担飞控控制、PWM 舵机、蜂鸣器、图像获取、定位等待、遥控数据、日志六种职责
- 首次完成完整实飞，暴露脚本堆叠在维护与复用上的问题，催生分层设计思想

---

[1.0.0]: https://github.com/<用户名>/autonomous-drone-five-layer-architecture/releases/tag/v1.0.0
