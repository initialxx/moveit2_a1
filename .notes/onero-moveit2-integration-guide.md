# 从零把机械臂接入 MoveIt 2 —— 概念与步骤完全指南

> 面向：第一次接触 MoveIt 2 的机械臂厂商工程师
> 假设：机械臂已有一套自己的控制 SDK（本文以 Python + SocketCAN 为例），
>       现在希望用 MoveIt 2 做运动规划并驱动真机。
> 本文所有 MoveIt 2 参数名、命名空间、源码位置均在本机 moveit2 (main, 2.15.2) 源码中核实过。

---

## 第 0 章 先建立正确的心智模型

**这是全文最重要的一节。** 90% 的初次接入失败，都源于对 MoveIt 2 职责的误解。

### MoveIt 2 是「大脑」，不是「手脚」

```
        ┌─────────────────────────────────────┐
        │            MoveIt 2                 │
        │  运动学 · 碰撞检测 · 路径规划 · 场景   │
        │                                     │
        │  它回答："应该走一条什么样的轨迹？"     │
        │  它不知道："怎么跟你的电机说话"         │
        └─────────────────────────────────────┘
                        │
                 一条带时间戳的轨迹
                        │
                        ▼
        ┌─────────────────────────────────────┐
        │        你要写的东西                   │
        │  把轨迹变成 CAN 帧，把编码器变成关节角   │
        └─────────────────────────────────────┘
                        │
                        ▼
                     真机电机
```

一个类比：

- **MoveIt 2 = 导航软件**。它知道地图、会算路线、能避开障碍。
- **你的驱动 = 司机**。导航说"前方 200 米右转"，司机得真的去踩方向盘。
- 导航软件永远不会替你踩方向盘。**这就是为什么"把 armsdk 接入 MoveIt 2"本质上是写一个驱动，而不是改 MoveIt。**

### 由此推出的三条铁律

1. **MoveIt 2 官方仓库不会收厂商驱动。** 它的 33 个包全是通用算法（规划器、运动学、场景管理）。
   全仓库 2599 个文件里没有一个机器人驱动。UR、Franka、Kinova、xArm、Trossen
   —— 126 个官方列出的机器人，**驱动 100% 在厂商自己的仓库里**。详见第 11 章。

2. **MoveIt 2 不强制要求 ros2_control。** 官方教程原话：
   > "it is not required to use `ros2_control` for your robot. You could write a proprietary
   > action interface. In practice, 99% of users choose `ros2_control`."

   我已在本机源码验证：`moveit_plugins/moveit_simple_controller_manager/package.xml` 的依赖是
   `moveit_common, moveit_core, rclcpp, pluginlib, control_msgs, rclcpp_action`
   —— **完全没有 ros2_control**。它只是一个 `rclcpp_action` 客户端。

3. **接入点是一个「缝」**：MoveIt 把轨迹交给一个 action server。谁来当这个 server 是你的自由。

---

## 第 1 章 名词表（先混个脸熟）

| 名词 | 一句话解释 |
|---|---|
| **URDF** | 机器人的"物理结构"：有哪些连杆、关节、尺寸、限位。ROS 通用格式。 |
| **SRDF** | MoveIt 专有。给 URDF 补上"语义"：哪些关节组成一个规划组、末端是哪个、哪些连杆永远不会碰、有哪些预设姿态。 |
| **Planning Group** | 一组一起规划的关节。你这台臂就是 7 个关节一个组，通常叫 `arm`。 |
| **Kinematics / IK** | 逆运动学：给定末端想要的位姿，反解出 7 个关节角。MoveIt 靠插件做（默认 KDL）。 |
| **Planning Scene** | 世界模型：机器人当前状态 + 环境障碍物 + 被抓着的物体。规划器在这个"场景"里搜索。 |
| **OMPL** | 最常用的采样型规划器库（RRTConnect 等）。MoveIt 的默认规划后端。 |
| **Planning Pipeline** | 规划流水线 = 规划器插件 + 前置请求适配器 + 后置响应适配器。 |
| **Trajectory** | 规划的输出：一连串 `(关节角, 速度, 时间点)`。注意是**带时间戳的**，不只是路径。 |
| **Time Parameterization** | 把"几何路径"变成"带时间戳的轨迹"的过程（决定走多快）。 |
| **Controller** | 执行轨迹的组件。在 MoveIt 眼里就是"某个能接收轨迹的 action server"。 |
| **ros2_control** | ROS 2 的标准硬件抽象框架。把「控制器」和「硬件」解耦，但它**不是 MoveIt 的一部分**。 |
| **Hardware Interface** | ros2_control 里的硬件插件（`SystemInterface`），负责真正读写硬件。C++ pluginlib 插件。 |
| **joint_state_broadcaster** | ros2_control 提供的一个小控制器，负责把硬件状态发布成 `/joint_states`。 |
| **joint_trajectory_controller** | ros2_control 提供的控制器，接收 `FollowJointTrajectory` action 并插值下发。 |
| **TF** | 坐标变换树。RViz 靠它把各个连杆画在正确位置。 |
| **MoveItPy / moveit_cpp** | MoveIt 的编程接口（Python / C++），比走 action 更方便。 |

---

## 第 2 章 MoveIt 2 的内部组成

你本机 `/home/initialxx/project/moveit2` 的顶层目录，就是它的全部：

| 包 | 作用 | 你会碰吗 |
|---|---|---|
| `moveit_core` | 核心算法：`RobotModel`（URDF→模型）、`PlanningScene`、`RobotState`、碰撞检测 | 只读源码理解概念 |
| `moveit_ros` | ROS 封装：`move_group` 节点、`planning_scene_monitor`、`moveit_cpp`、`moveit_servo` | 会用 |
| `moveit_planners` | 规划器插件：OMPL、CHOMP、STOMP、Pilz | 会用 |
| `moveit_kinematics` | 运动学插件：KDL、IKFast、cached_ik、srv_kinematics | 会用 |
| `moveit_plugins` | 控制器管理插件：`moveit_simple_controller_manager`、`moveit_ros_control_interface` | **重点：接入点在这** |
| `moveit_setup_assistant` | 图形化配置向导（简称 MSA），生成配置包 | **会用，省大量时间** |
| `moveit_configs_utils` | Python 工具，帮你正确拼装所有参数（**强烈推荐**） | **重点** |
| `moveit_py` | Python 绑定，`MoveItPy` | 会用 |
| `moveit_common` / `moveit_runtime` | 杂项 / 运行时依赖聚合 | 不用管 |

**给新手的建议**：不要试图读 `moveit_core` 的算法。你的工作 99% 集中在
`moveit_plugins`（知道有哪几种控制器管理器）+ 配置文件 + 你自己的驱动包。

---

## 第 3 章 一次「规划→执行」的完整数据流

把这张图看懂，MoveIt 就没有神秘感了。

```
① 用户发目标
   RViz 拖动 / MoveGroup action / MoveItPy
                    │
                    ▼
② move_group 读取"现在在哪"
   ↑ 来自 /joint_states (sensor_msgs/JointState)
   ↑ planning_scene_monitor 持续订阅
                    │
                    ▼
③ 规划器在 PlanningScene 里搜索无碰撞路径
   OMPL/RRTConnect，返回一串关节角（此时还没有时间信息）
                    │
                    ▼
④ 时间参数化
   AddTimeOptimalParameterization → 每个点加上时间戳/速度
                    │
                    ▼
⑤ TrajectoryExecutionManager 决定"交给哪个控制器"
   ↑ 查 moveit_simple_controller_manager.controller_names 参数
                    │
                    ▼
⑥ ★★★ 接缝：把轨迹打包成 FollowJointTrajectory action goal ★★★
   发给 action_ns 指定的 action server
                    │
                    ▼
⑦ 你的驱动节点收到轨迹
   插值 → 通过 CAN 发给 7 个电机
                    │
                    ▼
⑧ 你的驱动节点持续发布 /joint_states
                    │
                    ▼
⑨ robot_state_publisher 把 /joint_states + URDF 算成 TF
                    │
                    ▼
⑩ RViz 显示机械臂动起来了
```

**关键理解**：

- 第 ⑥ 步就是你要替换的位置。MoveIt 不关心对面是 C++ 的 ros2_control，还是 Python 的 rclpy 节点。
- 第 ⑧ 步是**闭环**。如果 `/joint_states` 不发或发得不对，MoveIt 会以为机器人没动，
  下一次规划会从错误的状态出发 —— 这是新手最常见的"机械臂乱动"原因。
- 第 ④ 步之后轨迹是**带时间的**。如果你的驱动忽略时间戳自己插值，
  就等于把 MoveIt 算好的速度规划扔掉了（现有 armsdk 代码正是这么做的）。

---

## 第 4 章 你必须提供的 8 样东西

分成三类。「描述类」让 MoveIt 知道机器人长什么样，「控制类」让它能动，「可视化类」让你能看见。

### 描述类

| # | 东西 | 内容 | 来源 |
|---|---|---|---|
| 1 | **URDF** | 连杆、关节、几何、惯性、限位 | 你们的 CAD 导出。**必须是唯一事实来源** |
| 2 | **SRDF** | planning group、末端 link、碰撞矩阵、命名位姿 | MSA 生成 / 手写 |
| 3 | **kinematics.yaml** | IK 插件名、搜索分辨率、超时 | 手写（很短） |
| 4 | **joint_limits.yaml** | 规划用的速度/加速度上限 | 手写，**必须比硬件限位保守** |

### 控制类

| # | 东西 | 内容 |
|---|---|---|
| 5 | **moveit_controllers.yaml** | 告诉 MoveIt：控制器叫什么、action 在哪个命名空间、管哪些关节 |
| 6 | **控制器实现** | 一个实现 `FollowJointTrajectory` 的 action server（三选一，见第 8 章） |
| 7 | **/joint_states 发布者** | 频率 ≥ 50Hz，关节名与 URDF **逐字符一致** |

### 可视化类

| # | 东西 | 内容 |
|---|---|---|
| 8 | **RViz 配置 (.rviz)** + robot_state_publisher | 否则 RViz 打开是空白，MSA 生成的包里自带一份 |

---

## 第 5 章 ★ 参数命名空间速查 —— 最容易踩的坑

**这一章请重点看。** MoveIt 2 的参数**不都是平铺的**，有一半必须放在特定命名空间下。
放错了不会报错，只会静默失效 —— 这是最阴险的一类 bug。

我在本机源码里逐个核实了读取位置：

| 内容 | **必须放在这个参数路径下** | 谁读它（源码位置） |
|---|---|---|
| URDF | `robot_description` | `robot_state_publisher`, `move_group` |
| SRDF | `robot_description_semantic` | `move_group` |
| IK 配置 | `robot_description_kinematics.<组名>.*` | `kinematics_plugin_loader.cpp:260` |
| 关节限位 | `robot_description_planning.joint_limits.<关节名>.*` | `parameter_name_list.hpp:250` |
| 规划管线列表 | `planning_pipelines`（**字符串数组**） | `move_group.cpp:244` |
| 默认管线名 | `default_planning_pipeline` | `move_group.cpp:263` |
| 各管线的参数 | `<管线名>.*`，例如 `ompl.planning_plugins` | `planning_pipeline.cpp:91` |
| 控制器配置 | `moveit_simple_controller_manager.*` | `moveit_simple_controller_manager.cpp:94,111` |

### 三个具体的「静默失效」陷阱

**陷阱 1：`kinematics.yaml` 平铺传入**

```yaml
# ✗ 错：文件里只有 arm: 一层，平铺传给节点后变成参数 arm.kinematics_solver
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
```
```yaml
# ✓ 对：文件内容还是 arm: 开头，但加载时必须包在 robot_description_kinematics 下
robot_description_kinematics:
  arm:
    kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
```
> 后果：MoveIt 报 "No kinematics plugins defined"，**完全无法做笛卡尔规划**。

**陷阱 2：`joint_limits.yaml` 少了外层**

```yaml
# ✗ 错
joint_limits:
  joint1: { max_velocity: 1.0 }
```
```yaml
# ✓ 对
robot_description_planning:
  joint_limits:
    joint1: { has_velocity_limits: true, max_velocity: 1.0 }
```
> 后果：**限位静默失效**，MoveIt 直接用 URDF 里的 `velocity=` 值。
> 如果 URDF 里随手写了 `velocity="30.0"` 而真机只有 3 rad/s —— 这是**安全问题**。

**陷阱 3：`ompl_planning.yaml` 少了外层**

```yaml
# ✗ 错：直接 planning_plugins: [...]
# ✓ 对：
ompl:
  planning_plugins:
    - ompl_interface/OMPLPlanner
  request_adapters: [...]
  response_adapters: [...]
  planner_configs: {...}
  arm:
    default_planner_config: RRTConnect
```
> 后果：`PlanningPipeline` 构造时发现 `planning_plugins` 为空，
> **抛 `std::runtime_error` 直接终止进程**（`planning_pipeline.cpp:136-141`）。

### 强烈建议：用 `MoveItConfigsBuilder` 自动拼装

手写这些命名空间太容易错。MoveIt 官方提供 `moveit_configs_utils`：

```python
from moveit_configs_utils import MoveItConfigsBuilder

moveit_config = (
    MoveItConfigsBuilder("onero_a1", package_name="onero_a1_moveit_config")
    .robot_description(file_path="config/a1.urdf.xacro")
    .robot_description_semantic(file_path="config/a1.srdf")
    .robot_description_kinematics(file_path="config/kinematics.yaml")
    .joint_limits(file_path="config/joint_limits.yaml")
    .trajectory_execution(file_path="config/moveit_controllers.yaml")
    .planning_pipelines(pipelines=["ompl"])
    .to_moveit_configs()
)

# moveit_config.to_dict() 就是可以直接塞给 move_group 的参数字典，
# 命名空间已经全部正确。这就是官方教程和 MSA 生成代码的做法。
```

而且 `moveit_configs_utils/launches.py` 里还现成提供了：
- `generate_demo_launch()` —— 一条命令起全套（含 ros2_control_node + spawner）
- `generate_spawn_controllers_launch()` —— 自动读 `moveit_controllers.yaml` 里的
  `controller_names`，逐个 spawn，并附带 `joint_state_broadcaster`

---

## 第 6 章 关键接口详解

### 6.1 `/joint_states`（sensor_msgs/JointState）

```python
msg = JointState()
msg.header.stamp = node.get_clock().now().to_msg()   # 必须用节点时钟！
msg.name     = ["joint1-a1_r", ...]                   # 与 URDF 逐字符一致
msg.position = [...]                                  # rad
msg.velocity = [...]                                  # rad/s
msg.effort   = [...]                                  # Nm
```

**三个必须做对的点**：

1. **关节名必须和 URDF 完全一致**。`joint1-a1_r` ≠ `joint_1` ≠ `joint1`。
   不一致时 MoveIt 会静默忽略这些关节，机械臂"不动"但没有任何报错。
2. **时间戳必须用节点时钟**（`node.get_clock().now()`），不要用 `time.monotonic()`。
   否则用仿真时间（`use_sim_time`）时整个 TF 树会崩。
3. **频率要够**。MoveIt 的 `planning_scene_monitor` 有超时机制，
   一般建议 ≥ 50Hz。你们 SDK 是 200Hz，很好。

### 6.2 `FollowJointTrajectory` action（control_msgs）

这是 MoveIt 和执行器之间**唯一的契约**。

```
Goal:
  trajectory:
    joint_names: ["joint1-a1_r", ...]        # 可能只是全部关节的子集！
    points:
      - positions:     [7 个关节角]           # rad
        velocities:    [7 个速度]             # rad/s  ← 别丢！
        accelerations: [7 个加速度]           # rad/s² ← 别丢！
        effort:        [...]                  # Nm
        time_from_start: {sec, nanosec}       # 从轨迹开始算起的时间
    header.stamp                              # 非 0 时表示"到这个时刻再开始"
  path_tolerance:   [...]                     # 每关节允许的路径偏差
  goal_tolerance:   [...]                     # 每关节允许的终点偏差
  goal_time_tolerance: ...                    # 允许的结束时间偏差

Feedback:
  actual:  {positions, velocities, ...}       # 实际状态
  desired: {positions, velocities, ...}       # 期望状态
  error:   {positions, velocities, ...}       # 误差

Result:
  error_code:  # SUCCESSFUL / INVALID_GOAL / INVALID_JOINTS /
               # OLD_HEADER_TIMESTAMP / PATH_TOLERANCE_VIOLATED /
               # GOAL_TOLERANCE_VIOLATED
  error_string:
```

**必须做对的四件事**：

1. **成功要如实报告**。`moveit_simple_controller_manager` 会检查 `error_code`。
   如果没到位却报 `SUCCESSFUL`，MoveIt 会认为一切正常，继续往下规划 —— 错误会累积。
2. **不要丢弃 velocities/accelerations**。MoveIt 已经算好了速度规划，
   你只要按时间执行即可。自己重新线性插值相当于废掉规划器的工作。
3. **支持取消**。用户会按 RViz 的 Stop。取消时要平滑停住并保持位置。
4. **`joint_names` 可能是子集**。虽然单臂一般是全部 7 个，但代码要做映射而不是假设顺序。

### 6.3 TF

由 `robot_state_publisher` 自动完成：订阅 `/joint_states` + 读 `robot_description`，
算出整棵 TF 树。你只需要保证 URDF 里有 `base_link`，并且 SRDF 里有一个
`virtual_joint` 把 `world` 连到 `base_link`（MSA 会生成）。

### 6.4 参数：`robot_description` / `robot_description_semantic`

分别装 URDF 和 SRDF 的字符串。注意 SRDF 也必须是 `<robot name="...">` 的 XML，
且 **name 属性应与 URDF 一致**（不一致时 MoveIt 会警告）。

---

## 第 7 章 ros2_control 是什么，为什么大家都在用

### 7.1 它解决的问题

没有 ros2_control 时，每个厂商各写各的驱动，MoveIt 得适配无数种接口。
ros2_control 定义了一套标准，把问题拆成三层：

```
┌──────────────────────────────────────────────┐
│  controller_manager  (一个 ROS 2 节点)         │
│    ├── joint_state_broadcaster                │  ← 控制器（插件）
│    │     硬件状态 → /joint_states              │
│    └── joint_trajectory_controller            │  ← 控制器（插件）
│          接收 FollowJointTrajectory，          │
│          按时间插值，写"命令接口"                │
├──────────────────────────────────────────────┤
│  Hardware Component  (插件)                   │
│    read()  : 硬件 → 状态接口                   │  ← 厂商在这里干活
│    write() : 命令接口 → 硬件                   │
└──────────────────────────────────────────────┘
```

**好处**：控制器是通用的，你只写硬件插件；反过来硬件插件也能被任何控制器复用。

### 7.2 生命周期

硬件组件有状态机：`unconfigured → inactive → active`。
`on_activate()` 是**使能电机**的正确位置（比手动调 service 规范得多）。

### 7.3 ★ 致命约束：不能用 Python 写硬件接口

ros2_control 的硬件组件是 **pluginlib 动态库**（C++，靠 `dlopen` 加载）。
ros2_control 维护者 christophfroehlich 在
[issue #1615](https://github.com/ros-controls/ros2_control/issues/1615) 里被问到
"能不能用 Python 写" 时明确回答：

> **"No, currently this is not possible."**

那位提问者的处境和你们几乎一样：*"我不熟 C++，我已经有 Python 控制程序了。"*

### 7.4 ★★ 但维护者给了官方出路

在同一条 issue 里，他补了一句：

> "If you don't have hard realtime requirements, you can use a **topic based system**
> with existing python hardware drivers."

也就是 **[PickNikRobotics/topic_based_ros2_control](https://github.com/PickNikRobotics/topic_based_ros2_control)**
—— 由 PickNik（MoveIt 的主导公司）维护的通用 C++ 硬件插件，
它把 ros2_control 桥接到两个普通话题：

- `/robot_joint_command`（订阅）
- `/robot_joint_states`（发布）

**你的 Python SDK 一行 C++ 都不用写**，只要发布关节状态、订阅命令即可。
URDF 里换一行 `<plugin>` 就接入。

> ⚠️ 注意它的局限：它走的是 position/velocity 命令接口。
> 如果你们要用 MIT 阻抗（同时给 position + velocity + kp/kd + torque），
> 这条路表达能力不够，需要自己写 C++ `SystemInterface`（见第 8 章路线 C）。

### 7.5 两个 YAML 千万别搞混

这是新手 100% 会混淆的地方：

| 文件 | 给谁读 | 关键内容 |
|---|---|---|
| **`ros2_controllers.yaml`** | `controller_manager` 节点 | `update_rate`；每个控制器的 `type: joint_trajectory_controller/JointTrajectoryController`、`joints`、`command_interfaces`、`state_interfaces`、`constraints` |
| **`moveit_controllers.yaml`** | `move_group` 节点 | `moveit_simple_controller_manager.controller_names`；每个控制器的 `type: FollowJointTrajectory`、`action_ns`、`joints` |

只有走 ros2_control（路线 B/C）时才需要**两个都有**。
走路线 A（自己写 action server）只需要 `moveit_controllers.yaml`。

`moveit_controllers.yaml` 的正确结构（已核对源码 `PARAM_BASE_NAME`）：

```yaml
moveit_simple_controller_manager:
  controller_names:
    - arm_controller
  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory      # ← 是相对命名空间，见下方说明
    default: true
    joints: [joint1-a1_r, joint2-a1_r, ...]
```

> ⚠️ `action_ns` 是**相对于控制器的命名空间**拼接的。源码里是
> `controller_name + "/" + action_ns`。所以上面的配置实际监听的是
> `/arm_controller/follow_joint_trajectory`。**不要**写成绝对路径。

### 7.6 read()/write() 的实时契约（走路线 C 必读）

- `read()`/`write()` 在 controller_manager 的更新循环里被调用，**绝不能阻塞**。
- 返回 `return_type::ERROR` 是**不可恢复**的 —— 硬件进入 FINALIZED 状态，
  官方文档说"唯一的选择是重新加载整个插件，而 controller_manager 目前没有这个服务"。
  **CAN 丢帧只能计数+打日志，返回 `OK`。**
- 你们的 `CanBackend.command()` 是阻塞的（最多 3–10ms 等收包），
  `get_state()` 更是逐电机串行查询（最坏 7×timeout）。
  **直接塞进 `read()`/`write()` 是错的。**

业界四种处理方式（按正确性递增）：

1. UR 的 `non_blocking_read` 参数 + 接收超时，`write()` 用"是否收到包"来门控；
2. UR 的 `asyncThread()`：所有非实时 I/O 丢到独立线程（20ms 循环）；
3. **Jazzy 原生异步硬件组件**：
   ```xml
   <ros2_control name="x" type="system" is_async="true">
     <properties>
       <async scheduling_policy="detached" thread_priority="98" affinity="[2,4]"/>
     </properties>
   ```
   Franka 用的就是这个。注意头文件自己的注释警告"目前与 controller_manager 的更新不同步"，
   所以必须用 `realtime_tools::RealtimeBuffer` 做交接。
4. Trossen 的做法：所有 CAN I/O 放在厂商 SDK 自己的线程里，
   `read()`/`write()` 只是纯内存拷贝 —— **和你们 SDK 的架构最契合**。

**反面教材（别抄）**：Kinova 的 `write()` 里直接阻塞调用 `RefreshFeedback()`，
故障恢复路径还在 `write()` 里同步调了两次急停。

---

## 第 8 章 三条可行的接入路线

### 路线 A：纯 Python 节点实现 FollowJointTrajectory

```
MoveIt ──FollowJointTrajectory──▶ 你的 rclpy 节点 ──CAN──▶ 电机
```

- **要写**：一个 Python 节点，200Hz 控制循环 + action server + `/joint_states` 发布。
- **优点**：最少的改动；Python 直接调 armsdk；官方明确允许。
- **缺点**：没有 ros2_control 生态（控制器切换、`joint_state_broadcaster`、
  任何假设 `controller_manager` 存在的 MoveIt 功能，例如 Servo 的某些集成）。
- **适合**：快速验证、内部使用、非产品化。

### 路线 B：ros2_control + topic_based_ros2_control

```
MoveIt ──▶ joint_trajectory_controller ──▶ TopicBasedSystem ──话题──▶ 你的 Python 节点 ──CAN──▶ 电机
```

- **要写**：Python 节点退化成"话题桥"（发布 `/robot_joint_states`、订阅 `/robot_joint_command`）+ 配置。
- **优点**：**零 C++**，同时拿到完整 ros2_control 生态。
- **缺点**：多一层话题跳转（延迟）；命令接口表达能力有限，
  **不适合 MIT 阻抗/力矩控制**。
- **适合**：位置控制为主、想要标准化、团队没有 C++ 能力。

### 路线 C：自己写 C++ SystemInterface（行业标准）

对标 UR / Franka / Kinova / xArm / Trossen。

- **要写**：一个 `ament_cmake` 包，`hardware_interface::SystemInterface` 子类，
  pluginlib 导出；**CAN/MIT 协议层用 C++ 重新实现**（你们的 `protocol.py` +
  `can_bus.py` 大约几百行，完全可以移植）。
- **优点**：质量最高、最标准、无 Python 依赖、可以暴露 position+velocity+effort
  三种命令接口以支持 MIT 阻抗；这才是一个"正规军"厂商驱动。
- **缺点**：工作量最大；协议层会出现 Python/C++ 两份实现，**必须加一致性测试防漂移**。
- **适合**：最终产品形态。

### 推荐：分阶段走

```
阶段 1  ──▶  路线 A 打通全链路
              （证明 URDF/SRDF/规划/执行/显示全对）
                     │
                     ▼
阶段 2  ──▶  路线 C 产品化
              （此时你已经完全理解整个系统，移植协议风险可控）
```

**不要一上来就写 C++**。因为如果 URDF 或配置有问题，你会在 C++ 里 debug 一个
其实跟 C++ 无关的问题。先用最快的路把"规划→真机"跑通，再替换执行层 ——
替换时 URDF/SRDF/配置**完全不用动**。

---

## 第 9 章 从零开始的完整步骤

核心原则：**先假后真，逐层验证**。每一层跑通了再往上加。

### 阶段 0：环境

```bash
sudo apt install ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers
pip install -e /path/to/armsdk          # 让 onero_sdk 可导入
```

建立一个 colcon 工作区（**不要**把包直接丢在 moveit2 源码目录里）：

```
~/onero_ws/
└── src/
    ├── onero_a1_description/       # URDF/xacro + mesh
    ├── onero_a1_driver/            # 驱动（先做路线 A）
    └── onero_a1_moveit_config/     # MoveIt 配置 + launch + rviz
```

### 阶段 1：只做「描述」，先让它显示出来

**目标：RViz 里能看到机械臂，能手动拖动滑块让它动。**

1. 建 `onero_a1_description` 包（`ament_cmake`，`install(DIRECTORY ...)`）。
2. 把权威 URDF 放进去。**用一份，不要抄第二份。**
   - 如果要用 xacro 加 `ros2_control` 块，就用 `<xacro:include>` 引入权威 URDF，
     **绝不复制关节坐标**。
   - mesh 路径用 `package://onero_a1_description/meshes/...`，
     把 STL 真的放进包里（不要用指向包外的软链接）。
3. 写一个最小 launch：`robot_state_publisher` + `joint_state_publisher_gui` + `rviz2`。

**验证**：
```bash
ros2 launch onero_a1_description display.launch.py
```
拖动滑块，RViz 里各连杆应该正确联动。
**此时还没有 MoveIt，也没有真机。** 这一步能筛掉 90% 的 URDF 错误。

> 检查 URDF 的一个好用技巧：对比不同来源的 URDF 时，用 FK 数值比对。
> 我前面就是这么发现某份 xacro 与权威 URDF 有 19.76mm 系统性偏差的。

### 阶段 2：接上 MoveIt，但用假硬件

**目标：RViz 里能拖着末端规划并"执行"，机械臂（假的）动起来。**

1. 用 **MoveIt Setup Assistant** 生成配置包：
   ```bash
   ros2 run moveit_setup_assistant moveit_setup_assistant
   ```
   依次过：URDF → 碰撞矩阵 → virtual joint → planning group(`arm`) →
   预设姿态 → 末端 → 控制器 → 作者信息 → 生成。

   MSA 会生成 `kinematics.yaml`、`joint_limits.yaml`、`moveit_controllers.yaml`、
   `ompl_planning.yaml`、`.rviz`、SRDF，**以及正确拼装参数的 launch**。

2. 用 MSA 生成的 launch 启动（它默认用假硬件 `mock_components/GenericSystem`）。

**验证**：RViz 里拖橙色球 → Plan → Execute，幽灵臂应该动起来。

> 这一步极其重要：它证明 URDF/SRDF/kinematics/joint_limits/planning pipeline
> **全部配置正确**。后面接真机时如果再出问题，就一定在驱动层，范围大大缩小。

### 阶段 3：接入真机

**目标：Execute 之后真机动。**

按第 8 章选路线。无论哪条，都要注意：

- 关节名逐字符对齐；
- `/joint_states` 用节点时钟、频率 ≥ 50Hz；
- 如实返回 `error_code`；
- 先让电机使能逻辑挂在正确的位置（路线 A 用 service，路线 B/C 用 `on_activate()`）；
- **第一次测试把速度压到极低**，并准备好物理急停。

### 阶段 4：调优与验收

- 规划用的 `joint_limits.yaml` 要比硬件限位保守（加速度尤其）；
- 调整 `goal_time_tolerance` / `goal_tolerance` 以匹配你们控制器的实际跟踪能力；
- 若发现"到位了但 MoveIt 说失败"或反之，回头检查第 6.2 节的 `error_code`。

---

## 第 10 章 坑清单（按踩中概率排序）

| # | 坑 | 症状 | 后果 |
|---|---|---|---|
| 1 | `kinematics.yaml` 没包在 `robot_description_kinematics` 下 | "No kinematics plugins defined" | 无法做位姿目标 |
| 2 | `joint_limits.yaml` 没包在 `robot_description_planning` 下 | 无任何报错 | **限位静默失效，安全风险** |
| 3 | 用了两份 URDF（一份给 SDK，一份给 MoveIt） | 规划位置与真机差几毫米到几厘米 | 碰撞检测失效、末端定位错 |
| 4 | `planning_pipelines` 参数没给 | `Failed to load any planning pipelines` | move_group 起不来 |
| 5 | `ompl_planning.yaml` 没包在 `ompl:` 下 | `Planning plugin name is empty` 异常 | 进程终止 |
| 6 | 关节名与 URDF 不一致 | 机械臂不动，无报错 | 极难排查 |
| 7 | `/joint_states` 用 `time.monotonic()` 打时间戳 | TF 报 extrapolation 错 | RViz 卡死 |
| 8 | 执行完成总是返回 `SUCCESSFUL` | MoveIt 以为成功 | 误差累积，行为诡异 |
| 9 | 丢弃轨迹的 velocity/acceleration 自己插值 | 运动一顿一顿 | 规划器白算 |
| 10 | `action_ns` 写成绝对路径 | MoveIt 找不到 action server | 执行超时 |
| 11 | 在 `write()` 里阻塞调用 SDK | controller_manager 掉周期 | 抖动、看门狗触发 |
| 12 | `write()` 里 CAN 丢帧返回 `ERROR` | 硬件进入 FINALIZED，无法恢复 | 必须重启 |
| 13 | mesh 用指向包外的软链接 | 单独 clone 后 RViz 空白 | 打包/发布即坏 |
| 14 | launch 里 `rviz2` 忘了 `-d xxx.rviz` | RViz 打开全空 | 以为没配好 |

---

## 第 11 章 关于「给 MoveIt 2 提 PR」

### 现实结论

**厂商驱动进不了 moveit2 仓库。** 这不是明文规定，而是 100% 一致的先例：

- moveit2 全仓库 2599 个路径里**没有一个厂商驱动**，33 个包全是通用算法；
- `ros2_control` 在整个仓库的 `package.xml` 里只出现过**一次**，还是个 `<test_depend>`；
- [moveit.ai/robots](https://moveit.ai/robots/) 列出的 126 个机器人，**驱动全部在外部仓库**
  （`frankaemika/franka_ros`、`PilzDE/pilz_robots`、`xArm-Developer/xarm_ros` …）；
- 连 MoveIt 自己的 PR2 参考机器人都放在独立仓库 `moveit/moveit_pr2`；
- 官方 [FAQ](https://moveit.ai/documentation/faqs/) 说：想把机器人加进**列表**，
  要 PR 的是 **moveit.ros.org 仓库**，不是 moveit2；
- 官方[贡献指南](https://moveit.ai/documentation/contributing/pullrequests/)给出的评审标准原话：
  > "Is it general enough to work with everyone's use-case and not only the requestor's setup?"

  厂商驱动天然不满足这一条。

**唯一看似反例的是 Pilz**：`pilz_industrial_motion_planner` 确实在 moveit2 里，且带公司名。
但它是一个**通用规划器插件**（任何机器人都能用 PTP/LIN/CIRC），且是由核心维护者
[PR #452](https://github.com/moveit/moveit2/pull/452) 移植的。而 Pilz 的**机器人驱动**
依然在外部仓库 `PilzDE/pilz_robots`。

> **规律：通用算法插件 → 欢迎上游；机器人专用驱动 → 放自己仓库。**

### 那 PR 该往哪提？

| 目标 | 内容 | 可行性 |
|---|---|---|
| **`moveit.ros.org`** | 把你们的机器人加进官方机器人列表 | ✅ 官方文档明确欢迎 |
| **自己的仓库** | `onero_a1_description` / `_driver` / `_moveit_config` | ✅ 这才是主战场 |
| **moveit2 上游** | 通用 bug 修复、`moveit_configs_utils` 改进、`moveit_py` 绑定、文档 | ⚠️ 可行，但要挑通用问题（见下） |
| **ros2_control 生态** | 如果走路线 C，硬件接口可以贡献给生态 | ⚠️ 视情况 |

上游**确实会收**的类别（近两年 400 个已合并 PR 中约 20% 来自非核心贡献者）：
core bug 修复、Windows/macOS 可移植性、RViz 可视化、MoveIt Servo、
Setup Assistant、`moveit_py` 绑定、`moveit_configs_utils`、文档、
以及**通用的**新参数。

**给你们的建议**：先在自己仓库把集成做扎实（带测试、带文档、能复现），
然后把过程中发现的**通用问题**单独提小 PR 给 moveit2。
"我为了让我的机械臂跑起来改了这里" 是不被接受的 ——
"我发现了一个所有人都可能踩的参数解析 bug" 才会被合并。

---

## 附录 A 推荐目录结构模板

```
onero_a1_ws/src/
├── onero_a1_description/                  # ament_cmake
│   ├── urdf/
│   │   ├── onero_a1.urdf.xacro            # 顶层，<xacro:include> 权威 URDF
│   │   └── onero_a1.ros2_control.xacro    # 可选：ros2_control 块
│   ├── meshes/                            # 真实文件，不用软链接
│   ├── launch/display.launch.py
│   └── package.xml
│
├── onero_a1_driver/                       # 路线 A: ament_python / 路线 C: ament_cmake
│   ├── onero_a1_driver/arm_node.py
│   ├── config/                            # can_channel / kp / kd 等
│   └── launch/bringup.launch.py
│
└── onero_a1_moveit_config/                # ament_cmake，MSA 生成后微调
    ├── config/
    │   ├── onero_a1.srdf
    │   ├── kinematics.yaml
    │   ├── joint_limits.yaml
    │   ├── ompl_planning.yaml
    │   ├── moveit_controllers.yaml        # 给 move_group
    │   └── ros2_controllers.yaml          # 仅路线 B/C，给 controller_manager
    ├── launch/
    │   ├── moveit.launch.py               # 驱动与 MoveIt 启动，默认假硬件
    │   └── bringup.launch.py              # 真机
    └── config/moveit.rviz
```

---

## 附录 B 逐层验收检查清单

```
□ 层级 1  URDF 正确
   □ check_urdf 通过
   □ RViz + joint_state_publisher_gui 能看到模型，各连杆联动正确
   □ 关节名与 SDK 侧完全一致
   □ mesh 路径为 package://，且文件在包内

□ 层级 2  假硬件下发 MoveIt 全链路
   □ move_group 启动无 ERROR
   □ ros2 param get /move_group robot_description_kinematics.arm.kinematics_solver 有值
   □ RViz 能 Plan 出轨迹
   □ Execute 后幽灵臂动起来
   □ ros2 topic hz /joint_states 有稳定频率

□ 层级 3  真机
   □ /joint_states 数值与真机关节角吻合（手动掰一下验证）
   □ 使能逻辑正确
   □ 低速单点运动 OK
   □ 轨迹执行到位
   □ error_code 如实反映结果
   □ 取消（RViz Stop）能平滑停住

□ 层级 4  产品化
   □ 限位比硬件保守
   □ 异常/CAN 丢帧不导致硬件进入 FINALIZED
   □ 有回归测试
   □ 文档与实际一致（这条最容易被忽略）
```

---

## 参考资源

- MoveIt 2 官方教程「接入新机器人」（是一个章节锚点，不是独立页面）：
  https://moveit.picknik.ai/main/doc/examples/examples.html#integration-with-a-new-robot
- 控制器配置教程（含 "not required to use ros2_control" 原话）：
  https://moveit.picknik.ai/main/doc/examples/controller_configuration/controller_configuration_tutorial.html
- Setup Assistant：
  https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html
- ros2_control 编写硬件组件：
  https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html
- ros2_control read()/write() 错误处理：
  https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/handling_errors_during_read_write.html
- 异步硬件组件：
  https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/asynchronous_components.html
- Python 硬件接口不可行的讨论：
  https://github.com/ros-controls/ros2_control/issues/1615
- 话题桥方案：https://github.com/PickNikRobotics/topic_based_ros2_control

### 值得抄的参考实现

| 厂商 | 仓库 | 备注 |
|---|---|---|
| Trossen | `TrossenRobotics/trossen_arm_ros` (jazzy 分支) | **最佳小模板**，46 个文件，Jazzy 对齐；`read()`/`write()` 是纯缓存拷贝 |
| UR | `UniversalRobots/Universal_Robots_ROS2_Driver` | `non_blocking_read` + `asyncThread()` 的教科书 |
| Franka | `frankarobotics/franka_ros2` | Jazzy 原生异步硬件组件 + `RealtimeBuffer` |
| Kinova | `Kinovarobotics/ros2_kortex` | 7-DOF 结构最相近；但 `write()` 阻塞是**反面教材** |
| xArm | `xArm-Developer/xarm_ros2` | 多命令接口的写法参考 |
