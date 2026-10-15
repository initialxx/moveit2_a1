# Onero A1 × MoveIt 2 工作区

把 Onero A1 七自由度机械臂接入 MoveIt 2 的 ROS 2 包集合。

**当前阶段**：第一阶段 —— 话题桥（`joint_state_topic_hardware_interface`）
**后续计划**：第二阶段 —— 自写 action server 方案

---

## 包结构

| 包 | 作用 |
|---|---|
| `onero_a1_description` | URDF / mesh / ros2_control 硬件配置 |
| `onero_a1_bringup` | launch + 控制器配置 + **话题桥 Python 节点** |
| `onero_a1_moveit_config` | SRDF / 运动学 / 限位 / OMPL / MoveIt 控制器映射 / RViz |

### 依赖方向

```
onero_a1_moveit_config ──▶ onero_a1_bringup ──▶ onero_a1_description
```

---

## 验证状态

| 模式 | 验证内容 | 结果 |
|---|---|---|
| **1. 纯 MoveIt 模拟** | 硬件激活 / robot model / OMPL / 控制器 | ✅ |
| | 参数命名空间（4 项 + 2 项反例） | ✅ |
| | 关节限位实际生效（2.0 / 3.0，非 URDF 的 30） | ✅ |
| | `arm` 组规划（20 点） | ✅ |
| **2. 话题桥全链路** | 桥接节点独立测试（乱序映射 / 跟随 / NaN 防护） | ✅ |
| | 直接给 JTC 发轨迹，7 关节零误差到位 | ✅ |
| | **经 move_group 完整「规划 → 执行」** | ✅ |
| | **看门狗：失联停用 + 恢复激活** | ✅ |
| **3. 真机** | SDK `Arm.servoj()` + `humanoid_right` + 原装夹爪重力补偿，100Hz，速度/加速度缩放 0.05 | ✅ 单次轨迹执行成功，用户确认实物基本满足要求 |

> **真机记录（2026-10-15 14:57）**：JTC 返回 `Goal successfully reached`，MoveIt 返回
> `SUCCEEDED`。停稳后 joint1 误差约 0.03145 rad（1.80°），joint4 约 0.01044 rad（0.60°），
> 全部关节满足当前 0.05 rad 到位容差。记录期间无新增 CAN 丢帧和控制异常。
> 这代表本次姿态/轨迹验证通过，不代表所有工作空间姿态的精度已完成标定。

> **工作区迁移后已复验（2026-10-14）**：迁移会导致 `build/` `install/` 中的绝对路径失效，
> 已清理重编（3 包）并重跑端到端 —— `error_code = 1`、轨迹 21 点、到位误差 0.0193 rad ✓。
> 一键复验：`bash tools/verify_workspace.sh joint1-a1_r 0.4`

> 验收命令：
> ```bash
> source setup_env.sh
> ros2 launch onero_a1_moveit_config demo.launch.py rviz:=false use_fake_hardware:=false simulate:=true &
> python3 tools/test_moveit_e2e.py joint1-a1_r 0.4
> ```
> 期望输出：`error_code = 1`、`规划轨迹点数 ≈ 20`、关节实际到位（误差 < 0.05 rad）。

> **目录调整（2026-10-15）**：项目已从 `.onero_ws/` 移到仓库根目录。迁移前的 `build/` 和 `install/` 记录了旧绝对路径，已清理并在根目录重建。重建命令：
> ```bash
> rm -rf build install log
> source setup_env.sh
> colcon build --symlink-install
> source install/setup.bash
> ```
> 构建结果：3 个包完成；本次没有重跑端到端测试。

---

## 环境准备

```bash
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-ros2controlcli \
  ros-jazzy-joint-state-topic-hardware-interface   # ← 话题桥插件（必需）
```

> ⚠️ **本容器环境的两个坑**（普通环境可忽略）：
> 1. `$HOME` 与 `/opt/ros` 为只读，`ros2 launch` 默认要写 `~/.ros/log` 会直接失败。
>    用 `source setup_env.sh` 可规避（把 `ROS_HOME` 指到工作区内）。
> 2. 无 X 显示，RViz 起不来。加 `rviz:=false`，或换有图形界面的机器。

---

## 编译

```bash
source setup_env.sh
colcon build --symlink-install
source install/setup.bash
```

---

## 运行：三种模式

| 模式 | 命令 | 说明 |
|---|---|---|
| **1. 纯 MoveIt 模拟** | `ros2 launch onero_a1_moveit_config demo.launch.py rviz:=false` | 不涉及 armsdk，验证 MoveIt 配置 |
| **2. 全链路联调** ★ | `... demo.launch.py rviz:=false use_fake_hardware:=false simulate:=true` | 走完整话题桥，但不连真机 |
| **3. 真机** | `... demo.launch.py use_fake_hardware:=false simulate:=false can_channel:=can0` | 需先使能电机 |

**建议顺序**：先 1 确认 MoveIt 配置对，再 2 确认桥接链路通，最后 3 上真机。

---

## 真机运动不到位：记录一次完整执行

真机桥接现通过 SDK `Arm.servoj()` 执行；安装方向、夹爪标定和重力计算由 SDK 负责。
本机已在 `.venv-sdk` 安装 SDK 动力学运行环境；迁移机器时运行
`bash tools/install_sdk_runtime.sh`，再 `source setup_env.sh`。
启动文件只为桥接节点选择这个 Python，不改变其他 ROS 节点的解释器。
请求重力补偿时，会在打开 CAN/使能之前通过 SDK 公共接口检查依赖和模型，失败则退出。

当前实物配置为 `humanoid_right`、原装 A1 夹爪、无额外负载。
该安装在基座坐标中的重力为 `[-9.81, 0, 0]`，`world -> base_link`
也使用同一 SDK 安装定义。`with_gripper` 选择 SDK 已有的原装夹爪标定系数，
不虚构质量或质心。其他安装方式须传入对应的 `mounting`。

JTC 在轨迹跟踪超差后会中止动作，并把当时实测位置设为保持目标。
此后 `controller_state.reference` 和 `error` 都相对保持位置，不能据此判断
机械臂是否接近原始规划终点。必须同时保留动作结果和原始规划轨迹。

启动时开启可选的控制诊断（默认关闭）：

```bash
source setup_env.sh
ros2 launch onero_a1_moveit_config demo.launch.py \
  rviz:=true use_fake_hardware:=false simulate:=false \
  can_channel:=slcan0 control_rate:=100 trace_control:=true \
  mounting:=humanoid_right with_gripper:=true gravity_compensation:=true
```

另一终端在点击 RViz 的 **Plan & Execute** 前开始记录：

```bash
source setup_env.sh
python3 tools/record_tracking.py
```

先确认运动全程没有桌面、支架接触或线缆拖拽。在 RViz 中把 Velocity Scaling
和 Acceleration Scaling 都设为 `0.05`，只做一次小范围 Plan & Execute；
动作结束（成功或失败）后再等 5 秒，在记录终端按 Ctrl-C。
这个缩放值只用于降低诊断动作的速度和加速度，不是经过标定的安全限值。
一次记录只执行一个目标，不要随后再次规划，以便原始目标与执行对应。

记录保存在 `.logs/tracking/`（已被 Git 忽略）：

- `*.jsonl`：全程控制器参考/反馈/输出、桥接输入、SDK 调用参数及返回反馈、
  动作状态和结果、相关参数、WARN/ERROR 日志。
- `*.summary.json`：原始 MoveGroup 规划终点与最新反馈的逐关节差值。
  原始轨迹从只读 GetResult 服务取得；规划预览仅作辅助，不当成已执行的证明。

记录工具只订阅话题和读取结果/参数，不发送目标，不使能电机，也不占用 CAN。
`control_trace.sdk_input_position` 是 ROS 目标，`command_position` 是 SDK 限制后的
目标（URDF 坐标），`raw_command_position` 是零位换算后交给 CAN 后端的位置。
`command_torque_ff` 是 SDK 计算、限幅后的实际前馈。记录不是总线抓包或驱动器执行确认。
SDK 无响应时会复用缓存反馈，需结合 `missed_joint_indices`、`exception` 判断有效性；
`period_s` 为真实控制周期间隔，`io_s` 包含 SDK 补偿/限幅/CAN 调用耗时，
`command_age_s` 为调用时命令年龄。
`feedback_torque` 是 SDK 解码后的电机反馈，不应未经标定直接视为关节输出轴力矩。
诊断样本在独立 ROS 回调中发布，不在 CAN 线程写文件；开启诊断仍会增加少量 CPU/DDS 开销。

| 记录现象 | 下一步处理 |
|---|---|
| 命令年龄或周期间隔明显增大、频繁丢帧 | 排查控制线程调度、DDS 和 SLCAN 吞吐/排队；频率参数不等于实际频率 |
| SDK 收到正确位置且反馈有效，低速后跟踪明显改善 | 检查规划速度/加速度与电机跟踪能力、期望速度传递和 PD 参数 |
| 低速仍有稳定关节偏差 | 检查驱动器限矩、增益、负载与重力补偿；补偿前确认安装姿态和夹爪质量/质心 |
| 动作成功、相对原始终点的关节误差小，但物理末端偏差大 | 检查 URDF 零位、基座坐标和夹爪 TCP 标定 |

保持现有轨迹容差和到位容差，先定位造成超差的环节，再做单项调整。
位置模式本身并不等于故障；期望速度固定为零是需评估的因素，不能凭此断言根因。

---

## 数据流（模式 2/3）

```
move_group
   │  FollowJointTrajectory action
   ▼
arm_controller (joint_trajectory_controller)
   │  命令接口：position
   ▼
joint_state_topic_hardware_interface
   │  /robot_joint_commands  (sensor_msgs/JointState)
   ▼
onero_topic_bridge  ← 本项目的适配层，可配置控制频率
   │  SDK Arm.servoj() → SDK 补偿/限幅/零位换算 → CanBackend → CAN → 电机
   │
   │  /robot_joint_states    (sensor_msgs/JointState)
   ▲
joint_state_topic_hardware_interface  →  arm_controller  →  move_group

         ┌──────────────────────────┐
         │ onero_bridge_watchdog    │  ← 旁路监控
         │ 订阅 /robot_joint_states  │     失联则停用 arm_controller
         └──────────────────────────┘
```

---

## ⚠️ 重要：URDF / SRDF 是自动生成的，不要手改

`onero_a1_description/urdf/onero_a1_body.urdf` 与
`onero_a1_moveit_config/config/onero_a1.srdf` 都由脚本从 **armsdk 的权威文件**生成。

**原因**：曾经出现过手抄 URDF 导致关节坐标系漂移（末端恒定偏差 19.76 mm、
Link3–Link6 坐标系整体差 180°）的事故，且**不会报错**，极难排查。

### 重新生成

```bash
# 机器人本体（只改 mesh 路径 / robot name，几何身份带自校验）
python3 tools/sync_description_from_sdk.py \
    --sdk-urdf /path/to/armsdk/onero_sdk/urdf/A1/a1_r.urdf \
    --out      src/onero_a1_description/urdf/onero_a1_body.urdf

# SRDF（碰撞矩阵照搬 SDK，追加 planning group / 命名位姿 / virtual joint）
python3 tools/sync_srdf_from_sdk.py \
    --sdk-srdf /path/to/armsdk/onero_sdk/urdf/A1/a1_r.srdf \
    --out      src/onero_a1_moveit_config/config/onero_a1.srdf
```

生成脚本会**自校验**：若 origin / axis / limit / parent / child 有任何改动，
立即中止且不写文件。

---

## ⚠️ 参数命名空间速查

MoveIt 2 的参数**不都是平铺的**。放错位置往往**不报错，只是静默失效**。
本项目用 `MoveItConfigsBuilder` 自动拼装，避免手工出错。最终参数路径为：

| 内容 | 参数路径 |
|---|---|
| IK 配置 | `robot_description_kinematics.arm.*` |
| 关节限位 | `robot_description_planning.joint_limits.<关节>.*` |
| 规划管线 | `planning_pipelines` + `ompl.*` |
| 控制器映射 | `moveit_simple_controller_manager.*` |

### 验收命令

```bash
ros2 param get /move_group robot_description_kinematics.arm.kinematics_solver
ros2 param get /move_group robot_description_planning.joint_limits.joint1-a1_r.max_velocity
ros2 param get /move_group ompl.planning_plugins
```

期望：分别返回 `kdl_kinematics_plugin/KDLKinematicsPlugin`、`2.0`、`['ompl_interface/OMPLPlanner']`。

> **特别提醒**：`joint_limits.yaml` 若未包在 `robot_description_planning` 下，
> 限位会失效，MoveIt 将使用 URDF 里的 `velocity="30.0"`
> —— 那是 SolidWorks 导出的占位符，真机上限只有 3–6 rad/s。**这是安全隐患。**

---

## 话题桥节点参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `simulate` | `true` | true 不连硬件（一阶跟随模拟）；false 连真机 CAN |
| `joint_commands_topic` | `/robot_joint_commands` | 命令订阅 |
| `joint_states_topic` | `/robot_joint_states` | 状态发布 |
| `control_rate` | `200.0` | 控制循环频率 Hz |
| `state_publish_rate` | `100.0` | 状态发布频率 Hz |
| `can_channel` | `can0` | SocketCAN 接口（simulate=false 时） |
| `model` | `A1R` | armsdk 模型名 |
| `kp` / `kd` | `50.0` / `1.0` | MIT 阻抗增益 |
| `on_fault_action` | `log` | 电机故障处置：`log` / `hold` / `disable` |
| `max_consecutive_misses` | `10` | 连续丢帧多少次后报 ERROR |
| `allow_disable_service` | `false` | 是否允许 `~/disable` 真的断使能（**默认拒绝**，见下） |
| `gravity_compensation` | `false` | 重力矩前馈（需要 `pinocchio`） |
| `max_gravity_torque` | `15.0` | 重力矩前馈幅值钳制（安全网） |

### 使能 / 失能服务（仅真机模式）

| 服务 | 行为 |
|---|---|
| `~/enable` | 使能电机，并把目标对齐当前位置（避免使能瞬间跳变） |
| `~/disable` | 断使能电机。**默认拒绝执行**，需 `-p allow_disable_service:=true` |

> ⚠️ **为什么 `~/disable` 默认拒绝**：断使能=卸力，带载且无支撑的机械臂会**掉落**。
> 必须由人确认机械臂已被物理支撑托住后再开启该服务。
> 另外，失能后 `arm_controller` 仍是 active，MoveIt 仍会下发轨迹（本节点会忽略）；
> 如需彻底阻断，请另行停用控制器。

### 重力补偿前馈（默认关）

开启 `gravity_compensation:=true` 会用 pinocchio 计算重力矩并作为前馈下发。
**它不是拿来就能用的**，因为错误的负载参数会算出错误力矩：

- 需要 `pip install 'onero-sdk[dynamics]'`（pinocchio）
- 未安装、找不到 URDF、维度不符时**一律显式报错**并退回零前馈，**绝不静默降级**
- 力矩带 ±`max_gravity_torque` 钳制，异常值直接退化为 0
- ⚠️ **本特性尚未在真机上验证过**（开发环境无 pinocchio 也无硬件），
  首次启用请务必空载、低速、有人看护

> **移植来源与踩坑记录**：这个特性是从 `armsdk/ros2` 移植的。原实现调用
> `_find_urdf("A1R")`，而签名是 `_find_urdf(model_name, variant="r")`，
> 于是**永远返回 None、重力补偿从不生效且一行日志都不打**。
> 这里改用正确调用 `_find_urdf("A1", "r")`，并把每种失败都讲清楚。
> 详见 `.notes/两套实现对比与合并建议.md` 的 S10。

> **kp/kd 说明**：MoveIt 的 `FollowJointTrajectory` 协议里**没有增益字段**，
> 所以 kp/kd 只能通过 ROS 参数传入（非逐周期可变）。
> 若将来需要逐周期动态调阻抗，需改为自定义接口（见选型文档）。

---

## 🔒 安全设计

### 问题：硬件接口不校验状态时间戳

`joint_state_topic_hardware_interface` 的 `read()` 只是把**最近收到的**状态抄进
状态接口，**没有任何时间戳校验**：

```cpp
joint_states_[POSITION_INTERFACE_INDEX][j] = latest_joint_state_.position[i];
```

**后果**：若话题桥节点崩溃或卡死，硬件接口会永远返回最后一帧状态，
MoveIt 会认为「机械臂静止、一切正常」，继续规划并下发轨迹。

### 三层防护

| 层 | 组件 | 职责 |
|---|---|---|
| **1. 参数校验** | `topic_bridge` | 启动时校验 `on_fault_action` 等参数，配置写错立即失败 |
| **2. CAN 健康** | `topic_bridge` | 接入 SDK 的 `on_miss` / `on_fault` 回调，感知丢帧与电机故障 |
| **3. 外部看门狗** | `health_watchdog` | 监控状态话题新鲜度，失联时**停用 `arm_controller`** |

### 看门狗行为

```
状态话题新鲜        -> 控制器 active
状态超时(默认 1.0s) -> 记录 ERROR + 停用 arm_controller  -> MoveIt 无法再下发轨迹
状态恢复            -> 自动重新激活 arm_controller
```

由 launch 参数控制：`watchdog_timeout`（默认 1.0）、`watchdog_auto_deactivate`（默认 true）。

### 电机故障处置（`on_fault_action`）

| 取值 | 行为 | 适用 |
|---|---|---|
| `log` | 仅记录（**默认**） | 首次调试，观察为主 |
| `hold` | 冻结目标，保持当前位置，忽略后续轨迹 | 大多数生产场景 |
| `disable` | 断使能电机 | ⚠️ **带载且无支撑时会掉落**，谨慎使用 |

> **本方案不能替代硬件安全链。** 完整的机械臂安全设计应包含：
> 驱动器过流/过温保护、物理急停按钮、机械限位、以及安全-rated 的控制架构。
> 软件看门狗只能覆盖「通信中断」这一类故障。

---

## 测试工具

```bash
# 1) 话题桥节点独立测试（含 NaN 防护回归测试，不需要 ros2_control）
ros2 run onero_a1_bringup topic_bridge --ros-args -p simulate:=true &
python3 tools/test_bridge.py

# 2) 执行层测试：直接给 arm_controller 发轨迹
python3 tools/test_execution.py

# 3) 最终验收：经 move_group 走完整「规划 → 执行」
python3 tools/test_moveit_e2e.py joint1-a1_r 0.4

# 4) 链路诊断：手动注入命令，逐段判断在哪一环断掉
python3 tools/diag_chain.py

# 5) MoveIt 规划链路测试
python3 tools/test_plan.py arm joint1-a1_r 0.5

# 6) 看门狗端到端验证：杀掉桥接 → 确认控制器停用 → 重启 → 确认自动恢复
bash tools/test_watchdog.sh

# 7) 一键复验（启动全链路 + 端到端测试 + 自动收尾）
bash tools/verify_workspace.sh joint1-a1_r 0.4
```

> ⚠️ **工具脚本的两条硬性约束**（都踩过，见 `.notes/session-handover.md` 第六节）：
> 1. **不要用 `set -u`** —— ROS 的 `setup.bash` 会引用未绑定变量并直接中止，
>    表现为"脚本无任何输出、退出码 127"。
> 2. **`pkill -f` 别在调用方命令行里留下字面量** —— 例如 `bash -c '... topic_bridge ...'`
>    会被 `pkill -f "[t]opic_bridge"` 连坐杀死（退出码 143）。
>    把流程写进脚本文件即可规避。

---

## ⚠️ 调试记录：两个已修复的隐蔽 bug

这两个问题都**不报错**，只是行为不对，值得记录以免重犯。

### bug 1：状态话题 QoS 不兼容

**现象**：`ros2 topic echo /robot_joint_states` 收不到消息，
但订阅它的硬件插件却工作正常。

**原因**：桥接节点用 `BEST_EFFORT` 发布状态（为了匹配硬件插件的
`SensorDataQoS`）。但 BEST_EFFORT 发布者**无法被默认的 RELIABLE 订阅者接收**。

**修复**：改为 `RELIABLE` 发布。RELIABLE 发布者可兼容 RELIABLE 和
BEST_EFFORT 两种订阅者，是严格更优的选择。

### bug 2：NaN 沿链路传播（安全问题）

**现象**：JTC 报告执行成功（`error_code=0`、有 feedback），
但关节位置始终不动，`/robot_joint_states` 全是 NaN。

**根因**：`hardware_interface/handle.hpp` 中：

```cpp
value_ = initial_value.empty() ? std::numeric_limits<double>::quiet_NaN() : ...
```

**没有 `initial_value` 的接口会被初始化为 NaN。** 控制器认领命令接口之前，
`/robot_joint_commands` 里会短暂出现 NaN；桥接节点把它写进状态后，
**NaN 与任何数运算仍是 NaN，从此永久传播、无法自愈**。

更严重的是：真机上这会把 NaN 位置下发给 `CanBackend.command()`。

**修复（两层防护）**：

1. `topic_bridge.py`：拒绝非有限值（`math.isfinite` 检查）、
   状态被污染时自动恢复、发布前再做一次兜底过滤
2. `onero_a1.ros2_control.xacro`：所有接口（含 `command_interface`）
   显式声明 `initial_value`，从源头避免 NaN

`tools/test_bridge.py` 第 5 步是该问题的回归测试。

---

## 已知限制

- **RViz 可用** ✅ —— 已验证，见 `docs/screenshots/demo_rviz.png`。
  （此前记为"不可用"是误判：AI 工具沙箱的 `/tmp` 是独立 tmpfs，
  遮住了宿主机的 `/tmp/.X11-unix/X1`。你在自己的终端里直接运行即可。）
- `update_rate` 固定 100Hz；话题桥每周期有 2 次跨进程通信，实测抖动后方可上调
- 状态看门狗已实现（`health_watchdog`），但**不能替代硬件急停**
- 仅暴露 `position` 命令接口；力矩/阻抗需另行设计
- `kp` / `kd` 目前只能通过启动参数设置，运行中不可调

## 相关文档

| 文档 | 内容 |
|---|---|
| `docs/真机接入检查清单.md` | 上真机前的逐项确认（环境、硬件、分步验证、安全） |
| `docs/screenshots/demo_rviz.png` | RViz 完整 demo 界面截图 |
