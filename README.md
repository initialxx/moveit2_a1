# Onero A1 × MoveIt 2

将 Onero A1 七自由度机械臂和 Woan Picker 夹爪接入 MoveIt 2 的 ROS 2 工作区。
支持 A1R 单臂规划、轨迹执行、碰撞场景及夹爪开合。
机械臂通过标准 FollowJointTrajectory / ros2_control 接口执行，适配层调用 Onero SDK 控制 CAN。

## 项目结构

| 位置 | 职责 |
|---|---|
| `src/onero_a1_description` | 机械臂、夹爪的 URDF、网格与 ros2_control 硬件描述 |
| `src/onero_a1_bringup` | 话题桥、SDK 会话、控制器、状态看门狗与环境加载 |
| `src/onero_a1_moveit_config` | SRDF、运动学、规划限位、OMPL、控制器映射和启动入口 |
| `src/onero_a1_common` | 可安装的公共 MoveIt Python 客户端 |
| `src/onero_a1_examples` | 功能示例，一项功能一个源码文件 |
| `tools/diagnostics` | 控制链路记录、精度验收、姿态存档、固定轨迹回放与 SDK 对照 |

```text
onero_a1_examples ──▶ onero_a1_common
                 └─▶ onero_a1_moveit_config ──▶ onero_a1_bringup ──▶ onero_a1_description
tools/diagnostics ──▶ onero_a1_common
```

## 环境准备与构建

使用 Ubuntu 24.04、ROS 2 Jazzy。安装 ROS 和 MoveIt 控制依赖：

```bash
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-ros2controlcli \
  ros-jazzy-joint-state-topic-hardware-interface
```

在工作区根目录构建：

```bash
source setup_env.sh
colcon build --symlink-install
source setup_env.sh
```

`setup_env.sh` 加载 ROS 和工作区环境，将 ROS 日志放在工作区 `.ros/log/`。
`overlay/` 提供话题桥依赖的本地覆盖；安装对应系统软件包后可直接使用系统依赖。
无图形界面时，启动命令添加 `rviz:=false`。

真机需要在所用 Python 环境中安装 [onero-sdk](https://pypi.org/project/onero-sdk/)，或配置 SDK 源码路径。
ROS 启动进程需能读取 SDK 安装方向模块，话题桥解释器需能调用 SDK Arm。
重力补偿运行环境使用隔离虚拟环境：

```bash
bash tools/install_sdk_runtime.sh
source setup_env.sh
```

启动文件通过 `ONERO_SDK_PYTHON` 选择 SDK 话题桥解释器；存在 `.venv-sdk/bin/python` 时，
环境脚本默认使用它。SDK 源码开发可通过 `ONERO_SDK_DIR` 指定源码路径。

## 启动模式与默认值

完整启动入口：`ros2 launch onero_a1_moveit_config moveit.launch.py`。
执行层入口：`ros2 launch onero_a1_bringup bringup.launch.py`。

| `hardware_mode` | 行为 |
|---|---|
| `sim`（默认） | 启动话题桥、模拟关节反馈和夹爪开合，不连接 CAN |
| `real` | 通过 SDK 连接 CAN，使能机械臂并保持实测姿态 |

```bash
# 模拟
ros2 launch onero_a1_moveit_config moveit.launch.py \
  hardware_mode:=sim mounting:=humanoid_right

# 真机：读取实测姿态并加载现场环境，先只允许规划预览
ros2 launch onero_a1_moveit_config moveit.launch.py \
  hardware_mode:=real mounting:=humanoid_right with_gripper:=true \
  allow_execution:=false \
  environment_file:="$PWD/src/onero_a1_moveit_config/config/environment.desktop.yaml"
```

两个启动入口按模式选择控制默认值，显式参数优先：

| 参数 | `sim` | `real` |
|---|---|---|
| `gravity_compensation` | false | true |
| `velocity_feedforward` | false | true |
| `kp` | 50（不使用） | 50 |
| `control_rate` | 200 Hz | 100 Hz |

通用默认值：`model:=A1R`、`can_channel:=can0`、`kd:=1.0`、`mounting:=upright`、
`gripper_model:=woan_picker`、`with_gripper:=false`、`rviz:=true`、`allow_execution:=true`、
`trace_control:=false`。

`mounting` 和 `with_gripper` 应按实际安装与负载指定。
`gripper_model` 选择几何模型和开合接口；`with_gripper` 选择 SDK 的原装夹爪重力补偿档位。
切换运行模式前结束上一套服务，各客户端与服务端使用同一个 ROS 域。

运行模式统一由 `hardware_mode` 选择。`sim` 与 `real` 共用话题硬件接口、控制器和夹爪接口。
`sim` 使用一阶跟随模拟关节运动，不计算重力、摩擦或接触动力学，不能用于验证真机到位精度。

## 功能示例与公共接口

使用方法及源码入口见 [Examples 指南](src/onero_a1_examples/README.md)。

```bash
ros2 run onero_a1_examples joint_goal --joint joint1-a1_r --delta 0.1
ros2 run onero_a1_examples named_pose --target ready
ros2 run onero_a1_examples pose_goal --dx 0 --dy 0 --dz 0.01
ros2 run onero_a1_examples cartesian_path --shape line --dx 0 --dy 0 --dz 0.01
ros2 run onero_a1_examples orientation_constraint --dx 0 --dy 0 --dz 0.01
ros2 run onero_a1_examples planning_scene
ros2 run onero_a1_examples collision_check
ros2 run onero_a1_examples gripper --opening 0.5
ros2 run onero_a1_examples return_startup
```

默认只规划/查询，添加 `--execute` 才执行；碰撞查询只读。
服务端 `allow_execution:=true` 和客户端 `--execute` 同时满足才执行示例轨迹。
`allow_execution:=false` 会禁止轨迹和夹爪动作，但真机 SDK 会使能并保持当前位置。

examples 默认使用 `world`：X 向前、Y 向左、Z 向上；位移为米、角度为弧度。
手臂位姿目标对应 `Link7` 法兰。需要基座坐标时使用 `--frame base_link`。
启动文件按 `mounting` 发布 world 与 base_link 的 TF，MoveIt 使用 TF 和 URDF 求解关节轨迹。
`home` / `ready` 为模型命名姿态，使用真机前需核对实际空间。

公共客户端的职责见 [Common 指南](src/onero_a1_common/README.md)。
精度报告、姿态存档、回放和 SDK 对照见 [诊断工具指南](tools/diagnostics/README.md)。

## 控制链路

```text
MoveIt ExecuteTrajectory
  → arm_controller /follow_joint_trajectory
  → ros2_control position + velocity 命令接口
  → /robot_joint_commands
  → onero_topic_bridge → SDK Arm.servoj() → CAN → 电机

电机反馈 → SDK → /robot_joint_states → joint_state_broadcaster → /joint_states → MoveIt
```

JTC 与话题桥传递同一采样点的位置、速度，按关节名匹配。
`velocity_feedforward:=true` 启用七个关节的目标速度前馈，false 将七关节期望速度归零。
`velocity_feedforward_scale` 固定为七个 1，无需填写。
命令缺失、非有限、时间戳无效或超过 `velocity_feedforward_timeout`（默认 0.1 秒）时，
速度前馈归零。取消、保持、重新使能时清除缓存速度。

`kp` / `kd` 为 SDK servoj 的全关节固定增益，仅在启动时设置。
`gravity_compensation:=true` 调用 SDK `gravity_torque()`，按实测姿态、安装与负载计算自重补偿，
经 `tau_ff` 下发；SDK 限幅与 `max_gravity_torque`（默认 15）共同约束力矩前馈。
动力学依赖和模型在打开 CAN 前检查。模拟模式不计算动力学。

## 桌面与支架环境

`environment_file` 接收碰撞体 YAML 绝对路径，通过 MoveIt Planning Scene 加载环境。
未指定时不自动加入桌面或支架，关节反馈不包含环境识别信息。

桌面与支架配置使用 `environment.desktop.yaml`。使用前核对文件内的尺寸、位置和安装方式。
真机执行拒绝使用 `example_only: true` 的配置；预览可使用 `allow_execution:=false`。

YAML 中 `frame_id` 为 world，`xyz` 表示长方体中心，`size` 为三个方向尺寸，单位为米；
`rpy` 为弧度，`color` 为 RGBA。`mounting` 必须与启动参数一致。
`environment.desktop.yaml` 描述木桌、深色底板、金属底座、立柱和连接板：

| 项目 | 几何约定 |
|---|---|
| 根部安装中心至深色底板顶面 | 0.330 m |
| 立柱 | 0.060 × 0.060 × 0.370 m |
| 金属底座（前后 × 横向 × 厚度） | 0.100 × 0.150 × 0.005 m |
| 连接板（前后 × 厚度 × 高度） | 0.160 × 0.005 × 0.080 m |
| 深色底板 | 1.000 × 1.000 × 0.010 m，顶面 z=-0.330 m |
| 木桌 | 1.000 × 1.000 × 0.030 m，顶面 z=-0.340 m |

环境配置不校准机械臂运动学、关节零位或负载模型，标记设为 false 也不能代替尺寸核对。
其他机械臂、线缆和桌面物品需另外建模；夹爪几何由 `gripper_model` 加入机器人模型。

启动流程为：检查配置 → 等待实测关节反馈 → 加载并回读场景 → 检查起始姿态 → 启动轨迹控制器。
应等待环境就绪后再发送规划请求。起点碰撞时核对模型与实际姿态，加载器不会自动放行接触。
环境对象保存在 MoveIt 场景中；加载器退出不移除对象，手动修改对象后以场景中的数据为准。

## Woan Picker

`gripper_model:=woan_picker` 启用夹爪外形、碰撞体、mimic 机构、gripper 规划组和开合接口。
`gripper_model:=none` 用于无夹爪模型场景。

| 接口 | 说明 |
|---|---|
| `/gripper_controller/gripper_cmd` | control_msgs/action/GripperCommand |
| `command.position` | 模型主关节角度：0 全闭，1.1 rad 全开 |
| `command.max_effort` | 仅接受 0，使用 SDK 位置控制参数 |
| `/gripper_controller/state` | 开度、SDK 反馈、活动标定及指令状态 |
| `/gripper_controller/calibrate` | 空载自动全开、全闭并保存端点 |
| `/gripper_controller/stop` | 停止当前动作/标定，保持反馈开度 |

夹爪与机械臂共用 SDK CAN transport 和 I/O 锁。启动只查询开度，收到动作目标才使能并开合。
夹爪主关节发布到 `/joint_states`，其他机构由 URDF mimic 联动。

空载且完整开合范围无遮挡时标定：

```bash
ros2 service call /gripper_controller/calibrate std_srvs/srv/Trigger '{}'
```

标定分别保存原生指令端点与稳定反馈端点；全开/全闭使用驱动端点，中间开度按实测反馈行程换算。
普通目标在标定期间拒绝接收。故障、反馈异常、超时或停止会中断标定并保留有效配置。
取消服务调用客户端不撤销服务端标定；需要中止时调用：

```bash
ros2 service call /gripper_controller/stop std_srvs/srv/Trigger '{}'
```

标定文件按型号和 CAN 通道命名，例如 `.ros/gripper_calibration/A1R_can0_woan_picker.json`。
启动自动加载匹配的标定；`gripper_calibration_file` 可指定路径，`none` 禁用文件。
型号、通道、仿真模式必须与文件一致，更换设备或端点定义后重新标定。
`gripper_closed_sdk_opening` / `gripper_open_sdk_opening` 提供启动时的手工坐标端点，
自动标定文件优先。

模型保留 SDK 的安装变换和 CAD 惯性；选择视觉模型不会自动修改 SDK payload 或 TCP。
SDK 原装夹爪补偿档位由 `with_gripper` 单独控制。夹爪开合不包含带物夹取成功判定。

## 模型生成与参数配置

机械臂本体 URDF、SRDF 和夹爪资源由 SDK 权威模型生成。生成工具保留连杆/关节几何与限位：

```bash
python3 tools/sync_description_from_sdk.py \
  --sdk-urdf /path/to/armsdk/onero_sdk/urdf/A1/a1_r.urdf \
  --out src/onero_a1_description/urdf/onero_a1_body.urdf

python3 tools/sync_srdf_from_sdk.py \
  --sdk-srdf /path/to/armsdk/onero_sdk/urdf/A1/a1_r.srdf \
  --out src/onero_a1_moveit_config/config/onero_a1.srdf

python3 tools/sync_woan_picker_from_sdk.py
```

本体碰撞配置排除直接相邻连杆，夹爪配置包含机构连接和闭合接触对。
模型来源和网格校验值见 `src/onero_a1_description/urdf/woan_picker_source.json`。

MoveItConfigsBuilder 组织参数命名空间：

| 配置 | 参数路径 |
|---|---|
| 逆运动学 | `robot_description_kinematics.arm.*` |
| 规划限位 | `robot_description_planning.joint_limits.<关节>.*` |
| OMPL 管线 | `planning_pipelines`、`ompl.*` |
| 控制器映射 | `moveit_simple_controller_manager.*` |

```bash
ros2 param get /move_group robot_description_kinematics.arm.kinematics_solver
ros2 param get /move_group robot_description_planning.joint_limits.joint1-a1_r.max_velocity
ros2 param get /move_group ompl.planning_plugins
```

arm 组使用 KDL，OMPL 默认 RRTConnect，并配置 RRTstar、PRM 等规划器。
规划速度、加速度以 `joint_limits.yaml` 为准。

## 状态监控与服务

`health_watchdog` 监控状态话题新鲜度，超过 `watchdog_timeout`（默认 1 秒）时停用 arm_controller，
反馈恢复后按看门狗流程重新激活控制器。`watchdog_auto_deactivate` 默认为 true。
话题桥通过 SDK 回调统计 CAN 丢帧和电机故障。

| `on_fault_action` | 行为 |
|---|---|
| `log`（默认） | 记录故障 |
| `hold` | 冻结目标并保持反馈姿态 |
| `disable` | 电机失能 |

真机提供 `/onero_topic_bridge/enable` 和 `/onero_topic_bridge/disable` 服务。
使能对齐实测位置；失能服务默认拒绝执行，需显式设置 `allow_disable_service:=true`。
断使能会卸除支撑力矩，现场支撑和硬件急停按设备操作流程处理。
软件状态监控不替代驱动器保护和硬件急停。

## 功能范围与使用限制

- 单臂示例面向 A1R，双臂协调、Servo、MTC 和相机感知管线未配置。
- 手臂支持 position + velocity 轨迹接口，夹爪使用 GripperCommand。
- 控制器更新率为 100 Hz；话题桥频率按运行模式配置。
- kp/kd 只在启动时设置，运行中不可调。
- 夹爪未提供经过标定的夹持力接口，也未定义权威指尖 TCP。
- 精度诊断使用关节反馈和 URDF 计算 Link7 位姿；实物末端精度需要独立测量。
- 规划场景的避碰效果取决于模型、安装变换与实际环境的一致性。

## 使用文档

- [功能示例与源码](src/onero_a1_examples/README.md)
- [公共 MoveIt 客户端](src/onero_a1_common/README.md)
- [诊断工具](tools/diagnostics/README.md)
- [真机接入检查清单](docs/真机接入检查清单.md)
