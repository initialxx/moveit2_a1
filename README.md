# Onero A1 × MoveIt 2 工作区

把 Onero A1 七自由度机械臂接入 MoveIt 2 的 ROS 2 包集合。

**当前阶段**：单臂基础执行已验证，继续通过 A1 专用示例验收 MoveIt 功能。
执行链路采用标准 `FollowJointTrajectory` / ros2_control / SDK，无需另写轨迹 Action Server。

---

## 包结构

| 包 | 作用 |
|---|---|
| `onero_a1_description` | URDF / mesh / ros2_control 硬件配置 |
| `onero_a1_bringup` | launch + 控制器配置 + **话题桥 Python 节点** |
| `onero_a1_moveit_config` | SRDF / 运动学 / 限位 / OMPL / MoveIt 控制器映射 / RViz |
| `onero_a1_demos` | A1 本体的规划、笛卡尔路径、约束和碰撞示例；默认仅规划 |

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
| **4. 本体功能示例** | 自碰撞查询、关节/命名/位姿规划、笛卡尔直线/矩形、姿态约束、场景绕障 | ✅ 假硬件下通过（2026-10-15） |
| | 新示例的关节/位姿轨迹执行与原始终点比对 | ✅ 假硬件零误差；这些新场景尚未在真机验收 |

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
> ros2 launch onero_a1_moveit_config moveit.launch.py rviz:=false use_fake_hardware:=false simulate:=true &
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

`onero_a1_moveit_config/moveit.launch.py` 是 A1 的完整启动入口：加载模型和规划配置，
启动执行层，收到有效关节状态后启动 MoveIt 和可选的 RViz。
真机模式会连接 CAN、使能电机并保持实测位置；具体运动由 RViz 或用户程序发起。
`onero_a1_bringup/bringup.launch.py` 只启动执行层，功能示例则位于 `onero_a1_demos`。

| 模式 | 命令 | 说明 |
|---|---|---|
| **1. 纯 MoveIt 模拟** | `ros2 launch onero_a1_moveit_config moveit.launch.py` | 默认 `hardware_mode:=fake`，使用假硬件 |
| **2. 全链路联调** ★ | `... moveit.launch.py hardware_mode:=sim` | 走完整话题桥，但不连真机 |
| **3. 真机** | `... moveit.launch.py hardware_mode:=real` | 连接 CAN 并通过 SDK 使能、保持实测姿态 |

`hardware_mode` 统一选择运行方式；`fake` 对应假硬件，`sim` 对应话题桥模拟，
`real` 对应话题桥真机。旧 `use_fake_hardware` / `simulate` 命令仍兼容；
如果与显式 `hardware_mode` 冲突，会在启动节点前报错。无界面时加 `rviz:=false`。

两个启动入口按模式自动选择控制默认值，显式命令行参数优先：

| 默认参数 | `fake` | `sim` | `real` |
|---|---|---|---|
| `gravity_compensation` | false | false | true |
| `kp` | 50（不使用） | 50（不使用） | 50 |
| `control_rate` | 200（无话题桥） | 200 Hz | 100 Hz |
| `velocity_feedforward` | false | false | true |

真机默认增益为 50、频率为 100 Hz。`mounting`、`with_gripper` 表示安装与负载，
由用户按实物指定；`gripper_model:=woan_picker`、RViz 和执行开关仍为三个模式共用的默认值。
仿真分支不创建 SDK 真机后端；即使显式开启重力补偿，也不会连接 CAN 或计算动力学力矩。

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
ros2 launch onero_a1_moveit_config moveit.launch.py \
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

## A1 本体 MoveIt 示例

这些示例移植了官方 [MoveGroup 示例](https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html)
和 [Planning Scene 示例](https://moveit.picknik.ai/main/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html)
的验证场景，通过标准 MoveGroup、GetCartesianPath、Planning Scene、ExecuteTrajectory 接口工作。
它们针对 **A1R 本体、`arm` 组、`Link7` 法兰**；尚不代表夹爪 TCP、夹爪碰撞或抓取控制已完成。

先用假硬件在独立 ROS 域打开 RViz（无需连接 CAN；两终端使用相同的测试域）：

```bash
source setup_env.sh
export ROS_DOMAIN_ID=91
ros2 launch onero_a1_moveit_config moveit.launch.py \
  use_fake_hardware:=true mounting:=humanoid_right rviz:=true
```

另一终端选择一个示例。**以下命令全部仅规划/查询，不执行机械臂运动。**

```bash
source setup_env.sh
export ROS_DOMAIN_ID=91
ros2 run onero_a1_demos run collision
ros2 run onero_a1_demos run joint --joint joint1-a1_r --delta 0.1
ros2 run onero_a1_demos run named --target ready
ros2 run onero_a1_demos run pose --dx 0.01 --dy 0 --dz -0.01
ros2 run onero_a1_demos run cartesian
ros2 run onero_a1_demos run cartesian --cartesian-shape square --square-size 0.01
ros2 run onero_a1_demos run constraints
ros2 run onero_a1_demos run scene
```

| 示例 | 验证内容 |
|---|---|
| `collision` | 正常姿态有效；只读查询折叠几何，确认确实返回非相邻连杆的自碰撞接触 |
| `joint` / `named` | 相对当前状态的关节目标 / SRDF 命名姿态，验证完整关节轨迹 |
| `pose` | 相对当前法兰位姿的目标，通过 IK/OMPL 规划 |
| `cartesian` | 法兰直线或小矩形，必须 `fraction=1`，并具有有效时间参数化 |
| `constraints` | 法兰移动过程中保持朝向，逐个轨迹点检查姿态约束 |
| `scene` | 布置阻挡原路径中间点、但不阻挡起终点的球体；重新规划并逐点检查无碰撞，最后移除该对象 |

碰撞反例的 joint4=3.14 rad **故意超过硬件限位**，只用于检查几何碰撞引擎；
`collision --execute` 被拒绝，不会把它发送给任何运动接口。
所有运动示例默认 `--velocity 0.05 --acceleration 0.05`，位移以 `base_link` 为参考系，
可用 `--frame world` 切换。`home`/`ready` 是模型预设值，尚未针对真实桌面/夹爪空间验收。
场景示例默认 joint4 变化 0.4 rad、障碍物半径 0.01m；部分起点可能无法布置有效障碍物，
程序会报告失败，可通过 `--scene-delta` / `--obstacle-radius` 调整。

A1 的 OMPL 配置已启用 `enforce_joint_model_state_space`。
本轮发现原来的位姿空间采样会使小范围姿态约束请求超时；改为关节空间采样后通过。
这不保证任意约束、任意目标都有解；碰撞、限位和不可达目标仍应明确返回失败。

在假硬件上也可以检查执行链路：

```bash
ros2 run onero_a1_demos run joint --delta 0.1 --execute
```

`--execute` 会将已规划并逐点检查的轨迹交给当前运行的 MoveIt 执行层，
因此连接真机时它也会产生真实运动。真机验证由用户在预览后执行；当前本体模型尚未覆盖实物夹爪，
桌面/支架需另行加载并校准环境配置（见下一节）。
程序会保存原始规划终点，并在执行成功后与新鲜关节反馈比较，避免把中止保持点误当原始目标。
报告写入 `.logs/demos/*.json`，包含完整轨迹、执行结果、关节误差以及相关场景/约束检查。
`pose`、`cartesian`、`constraints` 还会检查规划终点和执行后法兰位姿：
默认 `--position-tolerance 0.005`（5 mm）、`--orientation-tolerance 0.05`（约 2.86°）。
这是示例的验收阈值，不是机械臂的精度标称值，也不会调整 SDK 增益或控制器容差。
即使控制器 action 成功，只要末端误差超限，示例仍报告 `failed`；
`execution_action_result` 单独保留控制器结果，避免把“动作已经执行”与“末端验收通过”混为一谈。
该检查发生在执行结束后，不是运动中的实时偏差保护。

`start_pose`、`pose_target`、`planned_pose`、`actual_pose` 以及
`requested_translation_xyz_m`、`actual_translation_xyz_m` 用于对比请求与实际位移；
`fk_position_error_xyz_m` 是“反馈计算位置 - 请求目标”的三轴误差，`fk_orientation_error_rad` 是朝向误差。
这些值均由关节反馈和 URDF 计算，参考系为报告中的 `frame`，末端为 `Link7`；
`fk_position_error_m` 不能替代实物夹爪 TCP 的外部测量。

2026-10-15 18:40 的水平直线诊断发现：请求 X 增加 10 mm、Z 不变，规划轨迹的最大 Z 偏差
不足 0.005 mm，但前三次关节反馈 FK 的 Z 位移分别约为 -19.24、-14.87、-17.10 mm。
当时旧版只按每关节 0.05 rad 判定执行通过，末端误差虽已记录却未参与验收；
因此旧报告中的 `executed` 不能作为厘米级笛卡尔运动准确完成的证明。
后续只读保持状态采样中，SDK 下发目标与 ROS 输入一致，仍存在 joint1/joint4 稳态位置偏差。
位置控制精度仍需进一步处理，新增验收检查本身不会消除实际下沉。

### 位置跟踪增益对比

`moveit.launch.py` 提供并转发 `kp`、`kd`；真机模式默认 `50.0`、`1.0`，可显式覆盖。
它们是 SDK `Arm.servoj()` 的全关节固定增益，仅在服务启动时读取；
节点参数已设为只读，避免 `ros2 param set` 显示成功却未改变实际控制值。
启动日志会打印生效值，`trace_control:=true` 时还记录实际下发的
`command_kp` / `command_kd` 数组。

已有记录中，相同/相近目标在不同运动历史下存在不同方向的稳态偏差，
尚不能据此计算唯一的重力修正系数。下一项对比保持 `kd=1`、100 Hz、
速度/加速度缩放 0.05 不变，比较 `kp=50` 与候选 `kp=80`。
80 只是诊断候选值，不代表经过真机验收的默认值；本轮没有更改 SDK 的重力系数或增益默认值。
SDK 的逐关节 `150/40` 增益属于 SpeedJ/SpeedL 路径，当前 `servoj` 不使用该组参数。

两轮应从同一个已确认小臂、夹爪均无接触的姿态开始，使用相同环境和目标。
每轮只执行一次水平直线，再保持 5 秒收集误差；若出现振动，应停止该轮测试。
若起始关节位置差异明显，不应把两轮误差变化全部归因于增益。

候选轮的启动命令（基线轮仅把 `kp:=80.0` 改为 `kp:=50.0`）：

```bash
source setup_env.sh
ros2 launch onero_a1_moveit_config moveit.launch.py \
  use_fake_hardware:=false simulate:=false can_channel:=slcan0 \
  mounting:=humanoid_right with_gripper:=true gravity_compensation:=true \
  control_rate:=100.0 kp:=80.0 kd:=1.0 trace_control:=true \
  allow_execution:=true rviz:=true \
  environment_file:="$PWD/src/onero_a1_moveit_config/config/environment.desktop.yaml"
```

第二终端先运行 `python3 tools/record_tracking.py`，第三终端执行一次：

```bash
ros2 run onero_a1_demos run cartesian \
  --cartesian-shape line --frame world --dx 0.01 --dy 0 --dz 0 \
  --velocity 0.05 --acceleration 0.05 --execute
```

验收保持末端位置 5 mm、朝向 0.05 rad；没有为增益对比放宽容差。
重点比较 `original_goal_error_rad`、`fk_position_error_xyz_m`、
`actual_translation_xyz_m`，并结合控制记录确认实际增益和是否有丢帧/限幅。
只想采集静止保持状态时，可用 `python3 tools/record_tracking.py --observe-only --duration 10`；
该选项仅改变提示文本，记录器始终只读，不会代替操作者暂停正在执行的轨迹。

### 轨迹速度前馈

ros2_control 现使用 `position + velocity` 命令接口；JTC 将同一采样点的位置和速度
放进 `/robot_joint_commands`，桥接层按关节名映射后调用 SDK `Arm.servoj()`。
不通过位置差分估算速度，也不使用反馈速度代替目标速度。

- `velocity_feedforward:=false`：显式关闭时，全部七个关节的 SDK 期望速度归零。
- `velocity_feedforward:=true`：真机默认值，全部七个关节转发同帧目标速度；fake/sim 默认 false。
- `velocity_feedforward_scale`：仅为旧工具和诊断记录保留，固定为七个 `1.0`，无需填写。
  旧的分关节开关或缩放配置会在连接硬件前被拒绝；启停统一使用 `velocity_feedforward`。
- `velocity_feedforward_timeout:=0.1`：超过 100 ms 未收到新命令，或源时间戳过期，
  速度前馈归零，位置目标仍保持。缺失/非有限速度、非完整关节帧也归零。
- JTC 拒绝末点非零速度的轨迹；取消/保持使用零速度，重新使能会清除旧速度。

以上参数仅在启动时设置。`trace_control:=true` 记录 `input_velocity`（同帧原始速度）、
`sdk_input_velocity`（开关及过期检查后的速度）、`command_velocity`（SDK 限幅后下发值）
和 `velocity_feedforward_status`。示例 JSON 同时保存前馈配置。

用于当前真机对比的启动参数如下；先结束原服务，按现场流程安置好机械臂，再重新启动：

```bash
source setup_env.sh
ros2 launch onero_a1_moveit_config moveit.launch.py \
  use_fake_hardware:=false simulate:=false can_channel:=slcan0 \
  mounting:=humanoid_right with_gripper:=true gravity_compensation:=true \
  control_rate:=100.0 kp:=120.0 kd:=1.0 \
  velocity_feedforward:=true trace_control:=true \
  allow_execution:=true rviz:=true \
  environment_file:="$PWD/src/onero_a1_moveit_config/config/environment.desktop.yaml"
```

首先固定 kp/kd、频率、起点与轨迹，只切换前馈开关；不要同时改成 LeRobot 的高速度/加速度。
速度前馈主要改善运动中的跟踪，停稳后速度为零，不代表持位误差已经解决。
`prepare_trial` 只固定返回目标，返回起点和方向不同的两次结果不能直接比较增益效果。

新增命令接口会改变模型指纹，旧轨迹报告仍按原规则拒绝跨模型回放。
本次已校验接口更新前后的几何、关节限位、SRDF、规划限速不变，为现有 5 cm 轨迹生成
`.logs/trials/horizontal_5cm_velocity_interfaces.json`（位置点和时间保持原样）；原文件保留。
使用新文件预览 `prepare_trial`，记录器从执行前启动，动作完成后继续记录至少 10 秒。
只有返回起点检查通过，才用 `replay_trial` 回放水平轨迹；5 mm 验收和 0.01 rad 起点检查不变。

### SDK 直连与 MoveIt 对照

`sdk_compare` 使用 A 轮控制记录中实际送入 SDK 的位置、速度和时间，不重新插值或规划，
并固定 kp/kd、频率、安装方式、夹爪重力配置和零位。B 轮直接调用 SDK 公共 `Arm.servoj()`，
重力补偿由 SDK 根据 B 轮反馈计算；CAN 传输观察器只记录原始命令、不修改数据。
它不修改 SDK，也不同时运行 LeRobot 或其他机械臂控制程序。

当前问题是停稳后的偏差，因此首先使用 `--phase hold`：从 A 轮成功结束后第 1 秒起，
选取 10 秒的固定目标保持命令。B 轮起点与这段 **A 轮实测姿态** 比较，不要求它事先
达到原始目标的 5 mm 容差；B 轮最终仍按原始目标和 5 mm 验收。

在原 MoveIt 服务仍运行时导出（只查询模型、FK、碰撞状态，不发送运动）：

```bash
source setup_env.sh
ros2 run onero_a1_demos sdk_compare export \
  --from-report .logs/trials/kp120_ff_prepare_02.json \
  --trace .logs/tracking/kp120_ff_prepare_02.jsonl \
  --phase hold --output .logs/trials/sdk_ab_hold.json
```

也可以使用 `use_fake_hardware:=true allow_execution:=false mounting:=humanoid_right`
并加载同一 `environment_file` 的规划服务导出，工具会核对机械几何、SRDF、限位和安装变换。
它只使用该服务检查指定姿态，假硬件的当前显示姿态不作为真机反馈。
导出后环境和实物障碍物须保持不变；直连执行没有 MoveIt 的运行期监控。

默认命令只预览，不连接 CAN：

```bash
"$ONERO_SDK_PYTHON" -m onero_a1_demos.sdk_compare run \
  --bundle .logs/trials/sdk_ab_hold.json
```

按现场安全流程结束真机 launch，保持机械臂在 A 轮末段姿态。B 轮只接管已经使能且无故障的
机械臂，起点要求最大关节差不超过 0.01 rad、法兰差不超过 5 mm。若已失能或移动，先用
原流程恢复并重新记录/导出 A 轮；工具不会自动归位、修改零位或追赶到不匹配的起点。

```bash
"$ONERO_SDK_PYTHON" -m onero_a1_demos.sdk_compare run \
  --bundle .logs/trials/sdk_ab_hold.json \
  --output .logs/trials/sdk_ab_hold_result.json --execute
```

记录完成后报告立即写出，程序继续刷新目标和重力补偿。先按现场流程支撑好机械臂，再
Ctrl+C 结束保持；程序关闭时不发送失能指令，但不能依赖断开后的通信看门狗维持承重。
运动中 Ctrl+C 会结束记录并转入保持，再次 Ctrl+C 才退出。
对照采集期间仍严格要求七关节新反馈；采集完成后的保持允许短暂缺帧并继续刷新原目标，
连续丢帧和电机故障仍由 SDK 保护。保持阶段的缺帧、恢复及退出原因在结束时另存为
`<结果文件名>.hold_events.json`，不覆盖已经完成的对照报告。
新桥接和直连工具共用 CAN 进程锁，直连也会检查原 ROS 域中的话题桥；其他未使用这把锁的
程序仍须由操作者停止。实物场景或模型/SDK/配置文件变化时，需要重新导出。

离线对比（可在 B 轮继续保持时运行）：

```bash
ros2 run onero_a1_demos sdk_compare compare \
  --bundle .logs/trials/sdk_ab_hold.json \
  --result .logs/trials/sdk_ab_hold_result.json
```

工具比较同一参考序列、起点、最终关节误差和最后 3 秒平均法兰误差，并记录 B 轮真实 CAN
编码后的目标位置、速度、kp/kd、前馈力矩。保持段对照不用于评价动态跟踪。
以后确需整段轨迹对照时，可导出 `--phase motion`，仍使用 A 轮原始逐周期命令和起点检查。

整段运动对照需要先返回对照包内记录的 **A 轮实测起点**。该起点可能与服务
启动快照不同，因此使用以下命令而非 `return_startup`：

```bash
# 原真机配置的 MoveIt 服务运行时，先仅规划并检查碰撞
ros2 run onero_a1_demos sdk_compare prepare \
  --bundle .logs/trials/ab_motion_01.json

# 确认轨迹后执行；返回后按 B 轮同一组起点阈值检查
ros2 run onero_a1_demos sdk_compare prepare \
  --bundle .logs/trials/ab_motion_01.json --execute
```

此命令固定速度、加速度缩放为 0.05，核对原模型、控制配置和安装变换，不修改对照包。
只有显示 `A 轮起点检查通过` 后，才结束真机 launch 并用 SDK 执行 B 轮；B 轮连接后仍会
独立重新检查起点，不自动追赶位置。原对照包和 A 轮记录可继续使用，无需重新录制 A 轮。

### 固定离桌起点和轨迹，避免对比条件变化

2026-10-16 确认本阶段末端到位要求为 **5 mm 以内**。
`kp=80` 的实物观感已有改善，但此前 50/80 两轮的起点、位移不同，尚不能定量归因。
后续先以 80 作为测试配置，重复同一条轨迹；如需与 50 对比，使用同一份轨迹文件。
默认启动增益仍为 50；此处的 80 需显式通过启动参数设置。

先把机械臂移到经现场确认的小臂、夹爪均离开桌面的姿态，再仅规划并保存一次水平直线：

```bash
ros2 run onero_a1_demos run cartesian \
  --cartesian-shape line --frame world --dx 0.10 --dy 0 --dz 0 \
  --velocity 0.05 --acceleration 0.05 \
  --output .logs/trials/horizontal_10cm.json
```

文件同时保存起始姿态、完整轨迹、模型/限位标识、参考系和桥接配置。
文件已存在时请使用新名字；旧版本报告缺少这些校验信息，需重新规划保存。
确认 RViz 路径后，用以下命令执行**刚才保存的同一条轨迹**：

```bash
ros2 run onero_a1_demos run replay_trial \
  --from-report .logs/trials/horizontal_10cm.json \
  --output .logs/trials/kp80_first.json --execute
```

`replay_trial` 不重新求 IK，也不重新计算时间；它保留关节采样点、速度、加速度和
`time_from_start`，仅清零绝对启动时间戳。每次仍按当前场景逐点检查碰撞和限位。
起点要求每关节差值不超过 0.01 rad，同时法兰起点位置/朝向满足验收容差；
不满足时会拒绝执行。不能在此命令中更改 `--dx` 或 `--velocity` 等目标/时间参数，
需要改变动作时重新生成轨迹文件。

完成一轮后，要回到这份轨迹保存的**离桌测试起点**，先预览返回路径：

```bash
ros2 run onero_a1_demos run prepare_trial \
  --from-report .logs/trials/horizontal_10cm.json
```

确认后执行返回：

```bash
ros2 run onero_a1_demos run prepare_trial \
  --from-report .logs/trials/horizontal_10cm.json --execute
```

`prepare_trial` 会从当前实测姿态重新规划到保存的起点，不保证沿原路径返回。
返回后若未满足起点精度，报告会失败，应先解决持位精度再重复试验，不能放宽起点检查硬接轨迹。
这与 `return_startup` 不同，后者仍然返回本次服务启动时的姿态。

第二轮使用同一份基准文件和新的输出名，例如 `--output .logs/trials/kp80_second.json`。
如要比较增益，结束当前服务并以另一增益启动后，仍先用 `prepare_trial` 回到同一测试起点，
再用 `replay_trial` 执行同一份基准；不要重新生成相对运动替换基准。
每轮保持记录器开启，执行结束后再记录至少 5 秒。

离线比较两份报告（此命令不初始化 ROS，也不发送任何机械臂指令）：

```bash
ros2 run onero_a1_demos compare_trials \
  .logs/trials/kp80_first.json .logs/trials/kp80_second.json
```

工具核对轨迹、模型、时间参数、安装配置和实际起点，再列出法兰位置误差及三轴位移；
条件不一致时会明确提示不能直接比较。增益记录来自桥接配置，实际下发值仍可查控制记录中的
`command_kp`、`command_kd`。
新增 `requested_torque_ff`（请求补偿）、`sdk_torque_ff`（SDK 限幅后）、
`command_torque_ff`（最终下发）及 `sdk_ff_limited`，用于区分补偿不足和限幅。
这些诊断记录不改变增益、前馈或任何限幅策略。

目前软件验收点是 `Link7` 法兰，误差由关节反馈和 URDF 计算。
最终验证实物夹爪末端的 5 mm 精度，还需夹爪 TCP 标定和外部测量。

Pilz、MoveIt Servo、夹爪/TCP 和完整抓取放置属于后续功能，不在本轮本体示例范围内。

### 回到服务启动时的姿态

SDK 话题桥在启动时，固定保存 SDK 初始化返回的完整实测关节位置，随后运动或重新使能不会改写它。
这是本次服务的启动姿态，与 SRDF 中的 `home`（全零位）、每条轨迹的起点是不同的目标。
快照保存在 `/onero_topic_bridge` 的只读参数 `startup_pose`，并写入
`$ROS_HOME/startup_poses/*.json`（使用 `setup_env.sh` 时为工作区 `.ros/startup_poses/`）。

保留当前 MoveIt 服务，在另一终端先预览返回路径：

```bash
source setup_env.sh
ros2 run onero_a1_demos run return_startup --velocity 0.05 --acceleration 0.05
```

确认路径后执行（服务需已允许轨迹执行）：

```bash
ros2 run onero_a1_demos run return_startup --velocity 0.05 --acceleration 0.05 --execute
```

命令从当前实测状态规划到启动时的七关节位置，仍检查限位、起点、目标和路径碰撞；
它不保证沿此前路径原路返回。执行后同时报告相对规划终点和启动快照的关节误差，
默认到位容差为每关节 `0.05 rad`。仅预览和实际执行会分别重新规划。
命令适用于话题桥模式；`simulate:=true` 保存的是模拟初值，纯假硬件模式没有 SDK 话题桥快照。

**旧版本已启动的服务没有这份快照，命令会明确报错，不会用其他位置替代。**
重启服务会记录重启时的位置，不能靠重启追回上一次未保存的初始位。

---

## 桌面与支架碰撞模型

启动参数 `environment_file` 可加载由长方体组成的固定环境，使用 MoveIt 标准
[Planning Scene 接口](https://moveit.picknik.ai/main/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html)。
不传此参数时不会自动加入桌面或支架；读取真实关节位置也不会自动识别环境。

2026-10-16 补测布局保存在 `config/environment.desktop_measured.yaml`：根部安装中心高
330 mm（相对深色底板顶），立柱 60×60×370 mm，金属底座前后×横向×厚度为
100×150×5 mm，连接板前后×厚度×高度为 160×5×80 mm。根部中心距连接板上边
40 mm、前边 120 mm；立柱中心在根部前方 80 mm、内侧 35 mm。
底板按指定的 1000×1000×10 mm 近似；立柱中心距底板前边 970 mm，横向居中。
立柱后表面、金属底座后缘和底板后缘对齐。沿用根部原点后，底板顶面为
`z=-0.330 m`，木桌顶面为 `z=-0.340 m`。

新配置暂标为 `example_only: true`，用于 `allow_execution:=false` 预览核对；木桌足迹暂与
深色底板对齐，安装方向仍沿用 `humanoid_right`，未独立测量细小倾角。预览时将下方
启动命令的 `environment_file` 改为此新文件。该环境配置不改变 URDF、关节零位、增益或
SDK 重力配置，也不能用于证明真实末端已经达到 5 mm。

以下为早期草案及其原始估计值，和补测配置分别保留：

`config/environment.desktop_draft.yaml` 是按实装照片组成的**预览草案**，包含桌面、深色底板、
支架固定底座、立柱和根部连接板。已录入用户提供的近似值：

| 项目 | 数值 |
|---|---|
| 桌面长 × 宽 × 厚 | 1 m × 1 m × 0.03 m |
| 深色底板厚度 | 0.01 m |
| 支架总高 | 约 0.40 m；草案暂按含固定底座理解 |
| 机械臂根部安装中心距深色底板上表面 | 约 0.35 m |

“根部安装面/法兰”指机械臂与中央支架连接的圆形安装面，不是末端装夹爪的位置。
当前 `world` 原点与机械臂 `base_link` 原点重合，Z 向上；底板顶面为 `z=-0.35 m`，
木桌顶面为 `z=-0.36 m`。YAML 的 `xyz` 表示长方体**中心**，`size` 为三个方向的尺寸，
单位都是米；可选 `rpy` 为弧度，`color` 为 RGBA。

草案中的支架截面、固定底座尺寸、连接板包围盒和水平偏移仍为估计；深色底板暂覆盖整个桌面。
它标记为 `example_only: true`；如果真机模式还允许轨迹执行，会在启动驱动前拒绝加载。
假硬件预览命令：

```bash
source setup_env.sh
ros2 launch onero_a1_moveit_config moveit.launch.py \
  use_fake_hardware:=true mounting:=humanoid_right rviz:=true \
  environment_file:="$(ros2 pkg prefix --share onero_a1_moveit_config)/config/environment.desktop_draft.yaml"
```

假硬件预览使用虚拟初始关节值，**不是当前真机姿态**。仿真与真机串行运行、先退出上一套时，
无需专门修改 `ROS_DOMAIN_ID`；多个终端应处于同一域。

要读取真机当前姿态、叠加草案做规划预览，可退出假硬件启动后运行：

```bash
source setup_env.sh
ros2 launch onero_a1_moveit_config moveit.launch.py \
  use_fake_hardware:=false simulate:=false can_channel:=slcan0 \
  mounting:=humanoid_right with_gripper:=true gravity_compensation:=true \
  control_rate:=100.0 rviz:=true allow_execution:=false \
  environment_file:="$(ros2 pkg prefix --share onero_a1_moveit_config)/config/environment.desktop_draft.yaml"
```

`allow_execution:=false` 同时关闭 MoveIt 的轨迹执行、禁止本启动流程启动 `arm_controller`。
RViz 中使用 **Plan** 查看路径即可。**这不是电机失能的只读模式**：现有 SDK 驱动仍会使能电机、
按当前姿态保持并进行重力补偿；没有提交 MoveIt 轨迹也不代表电机没有输出。
应先退出上一套启动，避免已有控制器或 CAN 驱动干扰。场景草案可用于核对摆放和规划接口，
但其无碰撞结果不能作为真实运动的放行依据。

环境启用后的启动顺序：

1. 检查 YAML 是否完整、安装方式是否匹配、是否把预览草案用于真机。
2. 启动驱动并等待关节反馈；此时轨迹控制器尚未启动。真机 SDK 仍会使能并保持当前位置，
   这不是只读采集模式，碰撞模型不保护这段保持控制。
3. 启动 `move_group`，同步加载碰撞体并回读确认，再检查当前姿态。
4. 场景检查通过且 `allow_execution:=true` 时启动 `arm_controller` 和 RViz；
   检查失败或仅预览时只打开 RViz，控制器保持未启动。

`move_group` 的规划服务在加载场景时已经存在；应等待环境加载完成的日志后再发起规划。
加载器不修改允许碰撞矩阵，不删除已有其他障碍物，退出后本次加入的碰撞体仍留在场景中。
如果小臂原本贴在底板上，可能报告起点碰撞；应核对尺寸、坐标及起始姿态，不能靠忽略桌面碰撞强行放行。
根部连接处如报碰撞，也需核对安装几何，当前不会自动放行该接触。

允许真机执行前，需校准草案中标注的估计项以及根部高度、安装方向，另存为实物配置，再将
`example_only` 改为 `false`，把其绝对路径传给原有真机启动命令。仅修改标记不能完成校准。
照片里的另一只机械臂、实物夹爪、线缆和其他桌面物品尚未建模，因此这份环境模型不代表完整工作台避碰。
后续如手动删除/修改场景对象，规划使用的是修改后的场景；当前加载器只负责启动初始化。

---

## Woan Picker 模型与实体开合

MoveIt、bringup 和独立显示启动默认 `gripper_model:=woan_picker`，启用 SDK 同源的夹爪外形、
碰撞体、八关节机构联动和 `gripper` 规划组。只用机械臂本体时显式传 `gripper_model:=none`。
模型来源、安装变换与网格校验值记录在
`onero_a1_description/urdf/woan_picker_source.json`，可用
`python3 tools/sync_woan_picker_from_sdk.py` 重新导入；不会修改 SDK。

真机使用 SDK `OneroGripper` 的 A1R 标定与机械臂共用同一个 CAN transport/I/O 锁。
启动只查询当前开度，收到动作目标才使能并开合；反馈发布到 `/joint_states`，其余机构关节
由 URDF mimic 联动。假硬件与 `simulate:=true` 使用模拟反馈，不连接夹爪 CAN。

接口为 [MoveIt 支持的 GripperCommand](https://moveit.picknik.ai/main/doc/examples/controller_configuration/controller_configuration_tutorial.html)：

| 项目 | 值 |
|---|---|
| Action | `/gripper_controller/gripper_cmd` |
| 类型 | `control_msgs/action/GripperCommand` |
| 控制关节 | `woan_picker_left_finger_joint` |
| position | 0 rad 闭合；0.55 rad 中间开度；1.1 rad 全开 |
| max_effort | 当前只接受 0，沿用 SDK 的位置控制参数；尚未提供校准后的夹持力控制 |
| RViz 规划组 | `gripper`，命名姿态 `open` / `closed` |

这是 SDK 适配节点提供的 Action，不额外占用 ros2_control 的机械臂七关节接口。
目标开度变化速率限制为 0.5 归一化开度/秒，全目标行程约 2 秒，实际到位以反馈为准；
该 Action 没有速度字段，机械臂的速度缩放参数不改变夹爪开合速率。
取消会停止推进并保持反馈开度；到位依据新鲜真实反馈，通信异常、失能反馈和硬件故障中止动作。
默认堵转返回失败，结果中的 `stalled` 会置位；不把夹持力或是否夹住物体伪装成已测量结果。

先退出其他控制实例，假硬件预览与开合：

```bash
source setup_env.sh
ros2 launch onero_a1_moveit_config moveit.launch.py \
  use_fake_hardware:=true simulate:=true mounting:=humanoid_right \
  gripper_model:=woan_picker allow_execution:=true rviz:=true
```

另一终端同样 `source setup_env.sh` 后，打开或闭合（先执行其中一条，完成后再执行另一条）：

```bash
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
  '{command: {position: 1.1, max_effort: 0.0}}' --feedback

ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
  '{command: {position: 0.0, max_effort: 0.0}}' --feedback
```

真机先读取当前姿态、显示夹爪与补测环境：

```bash
source setup_env.sh
ros2 launch onero_a1_moveit_config moveit.launch.py \
  use_fake_hardware:=false simulate:=false can_channel:=slcan0 model:=A1R \
  mounting:=humanoid_right with_gripper:=true gravity_compensation:=true \
  control_rate:=100.0 kp:=120.0 kd:=1.0 velocity_feedforward:=true \
  gripper_model:=woan_picker allow_execution:=false rviz:=true \
  environment_file:="$PWD/src/onero_a1_moveit_config/config/environment.desktop_measured.yaml"
```

`allow_execution:=false` 也禁止夹爪动作目标。使用允许真机执行的环境配置并启用
`allow_execution:=true` 后，开合 Action 与仿真相同；有环境文件时，环境和轨迹控制器就绪后
才激活夹爪动作入口。夹爪查询失败时等待真实反馈，不用闭合/零位猜测实物姿态。

`with_gripper:=true` 仍单独选择 SDK 重力档位；模型内保留来源 CAD 的惯性，但不会自动
叠加实测 payload。现有 `arm` 笛卡尔目标仍指 Link7 法兰，SDK 未提供权威夹爪 TCP，
因此暂不把指尖当作新的控制点。添加夹爪会改变模型指纹，旧轨迹报告需要重新规划保存。
SDK 直连 A/B 工具目前仅支持 `gripper_model:=none` 的原单臂对照。

当前 A1R / can0、humanoid_right 安装、Woan Picker 的开合及控制诊断，可用以下精简命令：

```bash
source setup_env.sh
ros2 launch onero_a1_moveit_config moveit.launch.py \
  hardware_mode:=real \
  mounting:=humanoid_right \
  with_gripper:=true \
  trace_control:=true
```

已省略默认值 `can_channel:=can0`、`model:=A1R`、`kd:=1.0`、`rviz:=true`、
`allow_execution:=true`、`gripper_model:=woan_picker`、`gravity_compensation:=true`、`velocity_feedforward:=true`。
真机模式还默认使用 `kp:=50.0` 和 `control_rate:=100.0`，无需重复填写。
`trace_control` 默认 false，仅需日常控制时可省略，采集跟踪诊断时保留 true。
此命令没有加载桌面/支架碰撞体；使用环境模型时另加相应 `environment_file`。

空载自动标定由命令显式触发：

```bash
ros2 service call /gripper_controller/calibrate std_srvs/srv/Trigger '{}'
```

它自动全开、全闭，采集新鲜且稳定的 SDK 端点反馈，检查顺序和有效行程，再更新当前
开合坐标并原子保存。每个端点先发送到 SDK 原生指令 1 / 0，再采集稳定反馈；
保存的指令端点与反馈端点分别记录，正常全开/全闭重用标定时的指令，反馈则按实测行程换算。
中间开度按实测反馈行程计算目标（例如半开对应两个反馈端点的中点），
再限制指令变化速度；端点驱动量不线性套用到整个行程。动作到位还要求指令斜坡完成。
这样避免把运动中的短暂停顿当成端点，或把带负载时的反馈角度直接当作驱动目标。
旧版（`schema_version: 1`）文件仍能读取，但应重新空载标定，生成新版文件。
标定须空载、无外部阻挡；稳定停止本身不能区分机械端点与夹住物体。
普通开合动作在标定期间拒绝接收。故障、丢帧、超时或停止会保留原标定并停止推进目标。
停止标定或夹爪动作使用：

```bash
ros2 service call /gripper_controller/stop std_srvs/srv/Trigger '{}'
```

退出服务调用客户端不会撤销服务端标定，需使用上述停止服务。
真机默认保存到 `$ROS_HOME/gripper_calibration/A1R_slcan0_woan_picker.json`（按型号、通道命名），
本工作区即 `.ros/gripper_calibration/A1R_slcan0_woan_picker.json`。后续启动自动加载，
日常启动只查询开度。可以用 `gripper_calibration_file:=/绝对路径/profile.json` 指定路径；
保存的型号、通道、仿真模式必须匹配。仿真默认不写真实设备文件。
更换夹爪、通道或 SDK 端点定义后重新标定。

自动标定文件优先于手工启动端点，完成标定后无需再填下面的数值。
`/gripper_controller` 的 `closed_sdk_opening` / `open_sdk_opening` 参数反映活动标定。

如果需要手工标定，适配层也提供下面的启动参数：
适配层提供 `gripper_closed_sdk_opening` / `gripper_open_sdk_opening`（默认 0 / 1），
在 SDK 公共归一化坐标与模型 0..1.1 rad 之间做双向映射。参数仅启动时设置，
不修改 SDK、固件零位或其他设备的默认端点，也不把任意堵转当作到位成功。

2026-10-16 这台 A1R/slcan0 的 `woan_close_01.log` 在空载贴合后停于
0.0341134793 rad；用户确认两指贴合且无外部阻挡，对应原 SDK 开度 0.0310122539。
这台设备的重测启动命令增加：

```bash
gripper_closed_sdk_opening:=0.0310123
```

该值来自这只夹爪的本次贴合记录，换设备应独立核对。
模型 0 rad 将发送上述 SDK 闭合端点，真实 SDK 反馈也按相同标定还原为模型角度。
这类旧式手工坐标映射不区分驱动目标和反馈值；如仍有闭合跟踪偏差，请使用上述自动标定。
`/gripper_controller/state` 同时发布标定后的 `position_rad`、原 SDK `sdk_opening`、
`sdk_raw_position` 和所用端点，便于复核。首次重测只需先半开再闭合，并检查
`reached_goal=true`、`stalled=false`，以及实物贴合状态。

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

# SRDF（基于 SDK 模型，仅排除 URDF 相邻连杆；追加 MoveIt 语义）
python3 tools/sync_srdf_from_sdk.py \
    --sdk-srdf /path/to/armsdk/onero_sdk/urdf/A1/a1_r.srdf \
    --out      src/onero_a1_moveit_config/config/onero_a1.srdf
```

生成脚本会**自校验**：若 origin / axis / limit / parent / child 有任何改动，
立即中止且不写文件。

SRDF 不再照搬 SDK 中全部 `Never` 排除项：本体 8 个连杆共 28 对，
仅排除 7 对直接相邻连杆，其余 21 对保留自碰撞检查。
正常姿态的无碰撞结果不等于可以永久禁用这些连杆对；更新模型后应重新检查。

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

下表为直接运行 `topic_bridge` 节点时的默认值；使用 launch 启动时，
`kp`、`control_rate`、`gravity_compensation` 和 `velocity_feedforward` 采用前述 `hardware_mode` 对应默认值。

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
| `gravity_compensation` | `true` | 真机默认启用重力矩前馈（需要 SDK dynamics / `pinocchio`），模拟模式不计算 |
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

### 重力补偿前馈（真机默认开启）

`gravity_compensation:=true` 让 SDK 根据最新实测关节姿态、安装方向和负载档位，
计算抵消自重所需的关节力矩，并通过 `Arm.servoj(tau_ff=...)` 下发。
它在静止保持时也起作用；速度前馈则传递运动轨迹的目标速度，两者独立。
启动入口按模式选择该参数：real 默认 true，fake/sim 默认 false；
真机需要关闭时显式传 `gravity_compensation:=false`。
模拟模式不计算重力补偿，也不加载这项动力学依赖。

动力学由 SDK 内部的 Pinocchio 提供，适配层调用公共 `gravity_torque()` 接口。
依赖和模型在打开 CAN 前检查；缺失或返回无效力矩时显式报错。
输出还受 SDK 限幅和适配层 `max_gravity_torque` 限幅约束。
当前 `humanoid_right`、原装夹爪配置已由用户进行真机测试；使用该配置时保留
`mounting:=humanoid_right with_gripper:=true gravity_compensation:=true`。

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
