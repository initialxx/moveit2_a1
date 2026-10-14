# 机械臂接入 MoveIt 2：两种适配方案对比

> **文档目的**：为「Onero A1 机械臂接入 MoveIt 2」的技术选型提供决策依据
> **读者**：技术决策者、ROS 2 工程师、集成商
> **状态**：选型阶段，尚未实施
> **结论可信度标注**：本文对每条关键结论标注来源
> - `[源码]` = 在本机 MoveIt 2 (main, 2.15.2) / ros2_control (Jazzy 4.48.1) 源码或已安装头文件中直接核实
> - `[官方]` = 来自官方文档或上游仓库
> - `[估算]` = 基于架构的量级推算，**需实测确认**

---

## 0. 摘要（给没时间细读的人）

### 0.1 背景一句话

MoveIt 2 是运动规划框架，**不负责与硬件通信**。它与执行层之间有一个固定契约：`FollowJointTrajectory` action。
"接入 MoveIt 2"的本质，就是**提供一个能满足这个契约的执行层**。

### 0.2 两个方案的本质区别

| | 方案 A：自写 action server | 方案 B：话题桥 + ros2_control |
|---|---|---|
| **做什么** | 自己用 Python 实现 `FollowJointTrajectory` action server | 复用 ros2_control 现成的 `joint_trajectory_controller`，自己只写一个话题转发节点 |
| **本质** | **你拥有控制循环** | **框架拥有控制循环，你的节点是被动响应者** |
| **代码量** | 较大（要自己实现轨迹执行语义） | 很小（约 50–100 行转发） |
| **实时性** | **更好**（无每周期跨进程通信） | 稍差（每周期 2 次 DDS 往返） |
| **接口自由度** | **无限制** | 仅 position/velocity/effort |

### 0.3 一句话建议

> **如果你们的机械臂定位为「一个只有位置流的执行器」→ 选方案 B。**
> **如果定位为「有力控 / 拖动示教 / 感知的完整平台」→ 方案 A 的架构优势会明显得多。**
>
> 两者**不是不可逆的选择**，可以先跑通一条再切换（见第 6 节）。

### 0.4 最重要的一个澄清

**这两个方案需要你做的「描述层」工作完全相同**——URDF、SRDF、运动学配置、关节限位、规划器配置**一行都不用改**。
选择只影响**执行层**（约占整个工作量的三分之一）。

### 0.5 关于「合入 MoveIt 2 官方」

**厂商驱动不会、也不需要合入 moveit2 仓库**（这与选型无关，是生态的既有惯例）。
"接入官方"的真实形式是三条通道：

| 通道 | 形式 | 效果 | 优先级 |
|---|---|---|---|
| **rosdistro 发布** | bloom + PR 到 `ros/rosdistro` | 用户可 `sudo apt install` | ★★★ 最实质 |
| **MoveIt 机器人列表** | PR 到 `moveit.ros.org` | 进入官方机器人列表 | ★★★ 最直接 |
| moveit2 上游 | PR 到 `moveit/moveit2`，提**通用**改进 | 代码进入 MoveIt 2 本体 | ★★ 需挑通用问题 |

**两个方案在这点上完全相同**——都不需要修改 moveit2 任何代码。详见第 7 节。

---

## 1. 背景：MoveIt 2 的接缝在哪里

### 1.1 三层结构

```
┌──────────────────────────────────────────────────────────┐
│ 第 1 层：MoveIt 2 的契约                                  │
│   FollowJointTrajectory action                            │
│   字段：positions / velocities / accelerations /          │
│         effort / time_from_start                          │
│   ⚠️ 没有 kp、没有 kd —— 没有任何增益字段                  │
└──────────────────────────────────────────────────────────┘
                          │
┌──────────────────────────────────────────────────────────┐
│ 第 2 层：执行层（两种方案的差异所在）                       │
│   方案 A：你的 Python action server                       │
│   方案 B：joint_trajectory_controller + 话题桥            │
└──────────────────────────────────────────────────────────┘
                          │
┌──────────────────────────────────────────────────────────┐
│ 第 3 层：硬件（CAN / 电机）                                │
│   两个方案最终都调用同一个 SDK 接口                         │
└──────────────────────────────────────────────────────────┘
```

`[源码]` `control_msgs/action/FollowJointTrajectory` 的完整字段已核实，确实不含任何增益字段。

### 1.2 一个关键事实：ros2_control 不是必须的

MoveIt 官方教程原话：

> "it is not required to use `ros2_control` for your robot. You could write a proprietary
> action interface. In practice, 99% of users choose `ros2_control`."

`[官方]` MoveIt 2 Controller Configuration 教程

`[源码]` 已核实：`moveit_plugins/moveit_simple_controller_manager/package.xml` 的依赖是
`moveit_common, moveit_core, rclcpp, pluginlib, control_msgs, rclcpp_action`
——**完全不依赖 ros2_control**。它只是一个 `rclcpp_action` 客户端。

**所以方案 A 是官方明确允许的路径，不是"歪门邪道"。**

---

## 2. 两个方案的架构

### 2.1 方案 A：自写 action server

```
MoveIt 2
   │  FollowJointTrajectory（一条轨迹，一次性送达）
   ▼
┌────────────────────────────────────┐
│  你的 Python 节点（拥有控制循环）      │
│  ├─ action server                  │
│  ├─ 轨迹插值 / 容差 / 取消 / 反馈     │
│  ├─ 200Hz 控制循环                 │
│  └─ CAN I/O（调用 armsdk）          │
└────────────────────────────────────┘
   │
   ▼
电机
```

**特征**：控制周期内**没有任何跨进程通信**。轨迹作为一个 action goal 一次送达，之后全在你自己的进程里跑。

### 2.2 方案 B：话题桥 + ros2_control

```
MoveIt 2
   │  FollowJointTrajectory
   ▼
┌────────────────────────────────────┐
│  controller_manager（ros2_control） │
│  └─ joint_trajectory_controller    │  ← 现成，不用写
│       │ 命令接口（每周期）           │
│       ▼                            │
│     TopicBasedSystem               │  ← 现成，不用写
└────────────────────────────────────┘
   │  /robot_joint_commands  ↑ 每周期 2 次 DDS 跨进程往返
   │  /robot_joint_states    ↓
   ▼
┌────────────────────────────────────┐
│  你的 Python 节点（被动响应者）       │
│  ├─ 订阅命令话题 → 转发给 armsdk     │
│  └─ 发布状态话题 ← armsdk            │
└────────────────────────────────────┘
   │
   ▼
电机
```

**特征**：控制循环归 `controller_manager` 所有，你的节点降级为"话题响应器"。

`[源码]` `topic_based_ros2_control/TopicBasedSystem` 是一个通用的 C++ 硬件组件插件，由 PickNik
（MoveIt 的主导公司）维护，它把 ros2_control 桥接到两个普通话题。

---

## 3. 逐维度对比

### 3.1 总览表

| 维度 | 方案 A | 方案 B | 谁更优 |
|---|---|---|---|
| 接口表达能力 | 无限制 | 仅 3 个标准接口 | **A** |
| 实时性（延迟/抖动） | 无每周期 IPC | +0.3–2ms 抖动 `[估算]` | **A** |
| 你要写的代码量 | 300–500 行 + 测试 `[估算]` | 50–100 行 | **B** |
| 轨迹执行语义 | **要自己实现** | JTC 免费提供 | **B** |
| ros2_control 生态 | 无 | 完整 | **B** |
| 控制循环归属 | **你** | controller_manager | 取决于需求 |
| 模态能力（力控/示教） | 自然协调 | 需与框架抢控制权 | **A** |
| 部署依赖 | 仅 rclpy | + ros2_control + 话题桥 | **A** |
| 可观测性/调试工具 | 自己造 | `ros2 control` CLI 现成 | **B** |
| 行业惯例 | 少数派 | 主流（UR/Franka/Kinova 等） | **B** |
| 长期维护风险 | 自己维护轨迹执行器 | 依赖第三方插件 | 平手 |

### 3.2 接口表达能力

**方案 B 的限制来自 ros2_control 的接口白名单。**

`[源码]` `hardware_interface/system_interface.hpp:62-63` 定义了关节的标准接口：

> Those interfaces are: `HW_IF_POSITION`, `HW_IF_VELOCITY`, `HW_IF_ACCELERATION`, and `HW_IF_EFFORT`.

`[源码]` `topic_based_ros2_control` 只认这 4 个名字，其他一律抛异常：

```cpp
if (!getInterface(...))
  throw std::runtime_error("Interface is not found in the standard list.");
```

**补充说明**：ros2_control 本身**支持**任意命名的自定义接口，通过 `<gpio>` 标签实现
（`[源码]` `hardware_info.hpp:410` 有 `std::vector<ComponentInfo> gpios;`，且 `ComponentInfo::type`
可为 "sensor, joint, or GPIO"）。但**话题桥不支持 gpio**——所以自定义接口在方案 B 下不可用。

| | 关节命令接口 | 自定义接口（kp/kd 等） |
|---|---|---|
| 方案 A | **无限制** | **无限制** |
| 方案 B | 仅 4 个标准名字 | ❌ 抛异常 |
| （参考）自写 C++ SystemInterface | 4 个标准名字 | ✅ 走 `<gpio>` |

**实际影响**：本 SDK 中以下能力在方案 B 下**无法通过标准接口暴露**：
`force_mode`、`freedrive`、`set_payload`、`enable_collision_detection`、`speedj`/`speedl`、夹爪控制、`stop`。

> ⚠️ **注意**：这不等于"方案 B 做不到"。方案 B 下你的 Python 节点依然存在，
> 依然可以提供这些 service——只是它们会**挂在框架旁边**，而非框架之内。差别在架构整洁度，
> 不在可行性。详见 3.6 节。

### 3.3 实时性

**方案 A 在这一点上反而更优。**

| | 每周期跨进程通信次数 | 额外抖动 `[估算]` |
|---|---|---|
| 方案 A | **0 次** | 无 |
| 方案 B | **2 次**（命令下行 + 状态上行） | 0.3–2 ms |

`[源码]` 方案 B 使用的 `TopicBasedSystem` 存在 5 处实时性违规：

| # | 问题 | 源码位置 |
|---|---|---|
| ① | `read()` 里调用 `rclcpp::spin_some(node_)`，在实时读循环里做 executor 工作 | `read()` |
| ② | 订阅回调整条消息深拷贝（`latest_joint_state_ = *joint_state`），含 7 个字符串 + 3 个 vector 的堆分配，且该回调经 ① 在 `read()` 内执行 | 构造函数 |
| ③ | `write()` 每周期新建 `JointState` 并反复 `push_back`，堆分配 | `write()` |
| ④ | `write()` 内直接 `publish()`，涉及 DDS 序列化 | `write()` |
| ⑤ | 它是**同步**组件（非 Jazzy 的 `is_async`），上述开销全部计入控制周期抖动 | 类定义 |

**量级 `[估算]`**：

| `update_rate` | 周期 | 方案 B 额外开销占比 | 判断 |
|---|---|---|---|
| 100 Hz | 10 ms | 3–20% | ✅ 可接受 |
| 200 Hz | 5 ms | 6–40% | ⚠️ 需实测 |
| 500 Hz | 2 ms | 15–100% | ❌ 不可行 |

> **重要提示**：上述为架构量级推算，**必须实测确认**。实测方法见附录 B。

**但要放在整体链路里看**：本 SDK 的 CAN 往返本身就有 1–3ms（`recv_timeout` 默认 0.003），
Python 控制循环也不是硬实时。`[估算]` **方案 B 引入的开销与 CAN 相当，而非碾压性。**
真正的瓶颈在 CAN 和 Python，不在话题桥。

### 3.4 轨迹执行语义（方案 B 的最大优势）

方案 A 需要自己实现以下全部逻辑；方案 B 由 `joint_trajectory_controller` 免费提供：

| 需实现的语义 | 方案 A | 方案 B |
|---|---|---|
| 按时间插值（给了 velocity/acceleration 时需三次/五次插值） | 自己写 | ✅ |
| `header.stamp` 非零时等待到点再开始 | 自己写 | ✅ |
| 执行中 `path_tolerance` 检查 | 自己写 | ✅ |
| 终点 `goal_tolerance` + `goal_time_tolerance` | 自己写 | ✅ |
| 正确的 `error_code` / `error_string` | 自己写 | ✅ |
| 取消时平滑停住 | 自己写 | ✅ |
| feedback 按频率发布 | 自己写 | ✅ |
| 关节子集映射 | 自己写 | ✅ |
| `allow_nonzero_velocity_at_trajectory_end` | 自己写 | ✅ |

**这些边界情况不亲手写一遍是想不全的。** 本 SDK 现有的一个早期 ROS 2 节点实现，
正是在这些地方出错（执行结果恒返回成功、线性插值丢弃速度、忽略 `header.stamp`、无容差处理）。

`[估算]` 认真实现并测试这套语义，约 **300–500 行 Python** + 配套测试。

### 3.5 ros2_control 生态：方案 B 得到什么

`[源码]` 已核实以下控制器在 ROS 2 Jazzy 的 apt 源中可用：

| 控制器 | 对本项目的意义 |
|---|---|
| `joint_trajectory_controller` | **轨迹执行——最主要的一条** |
| `joint_state_broadcaster` | 状态发布 |
| **`admittance_controller`** | **导纳控制——与力控方向直接相关** |
| **`parallel_gripper_controller`** | **夹爪** |
| `forward_command_controller` | 直通命令 |
| `effort_controllers` / `velocity_controllers` / `position_controllers` | 各类直通 |

此外方案 B 还获得：

- **控制器生命周期管理**与**热切换**（`switch_controller` service）
- **硬件组件生命周期**——`on_activate()` 是使能电机的规范位置
- **统一的更新周期调度**
- **CLI 可观测性**——`ros2 control list_hardware_interfaces` / `list_controllers` / `view_controller_chains`
- **运行时 introspection**——可实时查看每个接口的值

### 3.6 架构归属：谁拥有控制循环（最本质的差异）

这是两个方案**最深层的区别**，也是很多摩擦的根源。

| | 方案 A | 方案 B |
|---|---|---|
| 控制循环归谁 | **你** | `controller_manager` |
| 你的节点角色 | **主人** | 被动响应者 |
| 位置流通道 | 你自己定义 | 被 JTC + 话题桥占用 |
| 非位置能力 | 同一节点内自然协调 | 挂在一旁，与框架割裂 |

**具体例子：进入 freedrive（拖动示教）时会发生什么？**

- **方案 A**：你的节点拥有循环。收到请求 → 停止跟踪 → 切到重力补偿 + 阻尼模式 → 完成。
  干净利落。
- **方案 B**：`joint_trajectory_controller` **仍在向外发位置命令**（或保持上一个目标），
  你的节点必须**主动忽略**这些命令。这不是"切换模式"，而是**与框架对抗**。

**每增加一个模态能力**（freedrive、力控、碰撞响应），方案 B 都要重新处理一次
"怎么让 JTC 让路"。方案 A 则天然协调。

### 3.7 共同点（避免误解）

以下几点**两个方案完全相同**，不构成选择依据：

| 共同点 | 说明 |
|---|---|
| **描述层工作** | URDF、SRDF、`kinematics.yaml`、`joint_limits.yaml`、`ompl_planning.yaml`、`.rviz` 配置——**完全相同，一行不改** |
| **硬件访问方式** | 最终都调用同一个 armsdk 接口（`servoj` / `CanBackend.command()`） |
| **kp/kd 需要侧信道** | `[源码]` `FollowJointTrajectory` 无增益字段，**两个方案都必须用侧信道传 kp/kd** |
| **需要 200Hz 内部循环** | 都需要一个持续向电机发命令的循环（保持位置/跟踪目标） |
| **`/joint_states` 发布** | 都要发布，且关节名必须与 URDF 逐字符一致 |
| **无法合入 moveit2 仓库** | 见第 7 节 |

> **关于 kp/kd 侧信道的重要澄清**：这不是 ros2_control 的限制，而是 **MoveIt 契约的限制**。
> 换成方案 A，MoveIt 依然不提供增益字段。所以"侧信道"不构成两个方案的差异。

---

## 4. 方案 A 下 SDK 能力的暴露策略

SDK 的能力按与 MoveIt 的关系分三类，**必须区别对待**：

### 类别 1：执行层——这是核心路径，不是"额外能力"

| API | 角色 |
|---|---|
| `servoj(joints, ...)` | 逐周期关节目标。**MoveIt 轨迹的每个控制周期就是调它** |

**这不是"顺便暴露"，这是发动机。** 方案 A 的 action server 内部真正要做的事，
就是把 `FollowJointTrajectory` 的轨迹按时间插值后，逐周期调用 `servoj`。

> **实现建议**：SDK 中还有一个 `execute_trajectory(frames, kp, kd)`，接收预计算的
> `TrajectoryFrame{joints, velocities, dt}` 逐帧执行。但它**是阻塞的，且不支持取消和 feedback**，
> 直接用做 action server 会很别扭（用户按 Stop 时无法优雅中断）。
> 另外 `dt` 是相对增量，而 `FollowJointTrajectory` 用绝对 `time_from_start`，需要重采样。
> **推荐用非阻塞的 `servoj` 在自建循环里执行。**

### 类别 2：与 MoveIt **竞争**——能暴露，但必须处理互斥 ⚠️

| API | 性质 |
|---|---|
| `movej` / `movel` / `movep` / `movec` | **SDK 自己规划**（Planned Motion，阻塞式） |

若暴露成 service，就会出现**两个规划器并存**：

```
MoveIt 规划的轨迹  ──▶ action server ──▶ servoj ──▶ 电机
                                        ▲
有人调 movej service ──▶ SDK 自己规划 ───┘
```

**后果**：MoveIt **完全不知道那次运动发生过**（它只看到 `/joint_states` 变化）。
该次运动**未经碰撞检查**；若与 MoveIt 正在执行的轨迹并发，两边会冲突。

**处置方式（二选一）**：
- **不暴露**——统一走 MoveIt（最干净）
- **暴露但加互斥锁**——定位为"快速点对点 / 维护用"，MoveIt 执行期间直接拒绝

### 类别 3：与 MoveIt **互补**——零冲突，可自由暴露 ✅

| API | 说明 |
|---|---|
| `freedrive_enable` / `freedrive_disable` | 拖动示教。MoveIt 不管这个 |
| `force_mode` / `force_mode_disable` | 力控。MoveIt 不做力控 |
| `set_payload(mass, cog)` | 负载辨识 |
| `enable_collision_detection` | SDK 自身的碰撞检测（**基于力矩残差，是"接触后检测"，不是几何预规划**，与 MoveIt 不重复） |
| `stop` / `emergency_stop` | 急停 |
| 夹爪控制、`get_external_wrench` | |

---

## 5. 方案 B 的两个额外风险

### 5.1 状态陈旧无检测（比延迟更现实的风险）

`[源码]` `TopicBasedSystem::read()` 直接使用最近收到的状态，**没有任何时间戳检查**：

```cpp
joint_states_[POSITION_INTERFACE_INDEX][j] = latest_joint_state_.position[i];
```

**后果**：若 Python 节点崩溃或卡住，`latest_joint_state_` 会永远停留在最后一帧，
`read()` 持续返回同一个值。**MoveIt 会认为"机械臂静止、一切正常"，继续规划并下发轨迹。**

且状态订阅使用 `rclcpp::SensorDataQoS()`（**best-effort，允许丢帧**），设计上不保证送达。

> **必须自行实现看门狗**：Python 侧检测 CAN 丢帧，controller_manager 侧检测状态超时。

### 5.2 命令发布阈值

`[源码]` 命令话题有个阈值参数（默认 `1e-5`）：**当命令与当前状态之差小于该值时，不发布命令**。

实际影响很小（默认值极小，近乎"一直发"），但若 Python 节点以"收到命令"作为唯一驱动信号，
静止时该信号会中断。**注意给 Python 节点自身一个独立的保持循环。**

---

## 6. 决策标准与可逆性

### 6.1 决策树

```
你们需要什么？
│
├─ 快速验证 / 内部使用 / 希望即插即用生态工具
│     └─▶ 方案 B
│
├─ 产品化，且需要暴露 SDK 全部能力（力控/示教/感知）
│     └─▶ 方案 A
│
├─ 对控制周期实时性有硬要求（Servo 遥操作 / 高频力控）
│     └─▶ 方案 A
│
└─ 团队没有精力实现并测试轨迹执行语义
      └─▶ 方案 B
```

### 6.2 一句话决策依据

> **你们的机械臂是「一个只有位置流的执行器」，还是「有力控/示教/感知的完整平台」？**
>
> - 前者 → 方案 B
> - 后者 → 方案 A 的架构优势会明显得多，因为那些模态能力与"谁拥有控制循环"强相关

### 6.3 ★ 选择是可逆的

**两个方案坐在同一个接缝后面。** 因为 MoveIt 只认 `FollowJointTrajectory`：

```
切换执行层时【不变】：
  URDF / SRDF / kinematics.yaml / joint_limits.yaml / ompl_planning.yaml
  moveit_controllers.yaml 里的 controller_names

切换执行层时【要改】：
  URDF 里的 <ros2_control> 块、launch 文件、可能的 action_ns
```

**描述层完全不用动。** 建议先用成本低的一条跑通，验证全链路，再根据实际需求决定是否切换。

### 6.4 关于第三种方案（不在本文对比范围）

行业主流（UR / Franka / Kinova / xArm / Trossen）走的是**自写 C++ `SystemInterface`**，
即"方案 B 的框架 + 自己的硬件插件"。

**本项目的特殊情况**：SDK 是 Python + SocketCAN，而 ros2_control 的硬件组件是
**C++ pluginlib 插件，无法用 Python 实现**。`[官方]` ros2_control 维护者在
[issue #1615](https://github.com/ros-controls/ros2_control/issues/1615) 中明确回答：

> "No, currently this is not possible."

因此走 C++ 路线意味着**把 CAN/MIT 协议层用 C++ 重写**，并维护 Python/C++ 两份实现。
这是本文不将其列为主要选项的原因，但它是长期产品化的标准形态。

---

## 7. 最终以何种形式「合入官方」

本节回答一个常见误解：**"接入 MoveIt 2"不需要、也不会以"把代码合入 moveit2 仓库"的形式完成。**

### 7.1 前提：厂商驱动不会合入 moveit2 仓库

这不是明文规定，而是 100% 一致的先例：

- `[源码]` moveit2 全仓库 2599 个路径中**没有一个厂商驱动**，33 个包全是通用算法
- `[源码]` `ros2_control` 在整个仓库的 `package.xml` 里只出现过**一次**，还是个 `<test_depend>`
- `[官方]` moveit.ai 官方机器人列表的 **126 个机器人，驱动全部在外部仓库**
  （`frankaemika/franka_ros`、`PilzDE/pilz_robots`、`xArm-Developer/xarm_ros` …）
- `[官方]` 官方 FAQ：想把机器人加进**列表**，要 PR 的是 **moveit.ros.org**，不是 moveit2
- `[官方]` 官方贡献指南的评审标准原话：
  > "Is it general enough to work with everyone's use-case and not only the requestor's setup?"

**对比佐证**：`[源码]` UR 的驱动在 apt 里是一个独立包（`ros-jazzy-ur-robot-driver` 3.9.0），
它的依赖列表是 `ros-jazzy-controller-manager`、`ros-jazzy-joint-trajectory-controller`、
`ros-jazzy-hardware-interface` 等——**它消费这个生态，而不是属于这个生态**。

> **正确的类比**：MoveIt 2 是**平台**，你们的驱动是**插件**。
> 没有人会问"我的 Chrome 插件怎么合入 Chromium"——MoveIt 2 的机器人驱动是同一回事。
>
> **MoveIt 2 的插件化架构本身就是它的"合入方式"**：通过实现它的接口来接入，
> 就是它设计的用法，不是被拒绝。

### 7.2 ★ 官方接纳的四条通道

| # | 通道 | 形式 | 最终效果 | 优先级 |
|---|---|---|---|---|
| 1 | **rosdistro 发布** | bloom + PR 到 `ros/rosdistro` | 用户可 `apt install` | ★★★ 最实质 |
| 2 | **MoveIt 机器人列表** | PR 到 `moveit.ros.org` | 出现在官方机器人列表 | ★★★ 最直接 |
| 3 | **moveit2 上游改进** | PR 到 `moveit/moveit2` | 代码进入 MoveIt 2 本体 | ★★ 需挑通用问题 |
| 4 | ros2_control 生态 | PR 到相关仓库 | 改进被生态共享 | ★ 视方案而定 |

---

#### 通道 1：rosdistro 发布 —— 进入 ROS 官方 apt 源 ★最重要

**这是"合入官方"最实质的含义**：你们的包成为 ROS 官方发行版的一部分，
用户一条命令即可安装，无需 clone 仓库、无需处理依赖。

`[源码]` **现实例证**——Kinova Gen3（同样是 7-DOF）在 `jazzy/distribution.yaml` 中的真实条目：

```yaml
  ros2_kortex:
    doc:
      type: git
      url: https://github.com/Kinovarobotics/ros2_kortex.git
      version: main
    release:
      packages:
      - kinova_gen3_7dof_robotiq_2f_85_moveit_config
      - kortex_bringup          # ← launch + 控制器配置
      - kortex_description      # ← URDF + mesh
      - kortex_driver           # ← 硬件接口 / 驱动
      tags:
        release: release/jazzy/{package}/{version}
      url: https://github.com/ros2-gbp/ros2_kortex-release.git   # ← bloom 自动生成的发布仓库
      version: 0.2.6-1
    source:
      type: git
      url: https://github.com/Kinovarobotics/ros2_kortex.git
      version: main
    status: developed
```

**这个结构就是本项目的模板**——包名一一对应：

```
kortex_description   →   onero_a1_description     （URDF + mesh）
kortex_driver        →   onero_a1_driver          （驱动/适配层）
kortex_bringup       →   onero_a1_bringup         （launch + 控制器配置）
*_moveit_config      →   onero_a1_moveit_config   （SRDF + MoveIt 配置）
```

**流程**：

1. 在自有仓库中建立上述 ROS 包
2. 满足 REP-144（包命名 / 版本规范）
3. 执行 `bloom-release --rosdistro jazzy --track jazzy <repo>` —— bloom 会自动创建
   release 仓库（位于 `ros2-gbp` 组织下）并向 `ros/rosdistro` 提交 PR
4. PR 合并后，ROS build farm 自动构建
5. **用户即可 `sudo apt install ros-jazzy-onero-a1`**

**最终效果**：

- 出现在 `index.ros.org`（ROS 官方包索引）
- 可通过 apt 安装，与 MoveIt 官方包地位平等
- 多发行版支持（humble / jazzy / rolling）可自动化
- 版本管理、CI、依赖解析全部由 ROS 基础设施承担

`[源码]` **规模参考**：`jazzy/distribution.yaml` 中共有 **839 个仓库条目**，
`status` 字段分布为 `maintained` 420 / `developed` 401 / `unmaintained` 1 / `end-of-life` 3。
**这是一个开放的、厂商可自由加入的名单**，没有"厂商驱动不能进"的限制。

---

#### 通道 2：MoveIt 官方机器人列表

- **形式**：向 `moveit.ros.org` 仓库提交 PR
- **依据**：`[官方]` MoveIt 官方 FAQ 明确指路
  > "I would like to add a new robot to the list of robots using MoveIt. What should I do?
  > → Create a pull request to moveit.ros.org's Github repo"
- **最终效果**：出现在官方机器人列表页（当前列有 126 个机器人）
- **前置条件**：有一个公开可用的驱动仓库（即通道 1 的前置条件）

**这是"被 MoveIt 官方承认"最直接的体现**，也是对外宣传最有说服力的材料。

---

#### 通道 3：moveit2 上游通用改进

- **形式**：PR 到 `moveit/moveit2`
- **能合并的内容**：`[官方]` 依近两年 400 个已合并 PR 的抽样，约 **20% 来自非核心贡献者**，
  集中在：核心 bug 修复、Windows/macOS 可移植性、RViz 可视化、MoveIt Servo、
  Setup Assistant、`moveit_py` 绑定、`moveit_configs_utils`、文档、**通用的**新参数
- **不能合并的内容**：任何机器人专用代码
- **评审标准**：见 7.1 引用的官方原话——必须"对所有人都适用"，而非"为了让我这套跑起来"

**这类 PR 的价值不在功能，在于技术声誉**，且必须作为集成的**副产品**自然产生，而非刻意追求。

---

#### 通道 4：ros2_control 生态贡献（视选型而定）

- 若选**方案 B**：对 `topic_based_ros2_control` 的改进可提 PR（例如纠正其文档中
  关于 `effort` 命令接口支持情况的说明）
- 若长期走 **C++ `SystemInterface`**：硬件接口本身仍建议放自有仓库，
  但过程中的通用改进可回报上游

---

### 7.3 目标形态总览

```
你们的 GitHub org
│
├── onero_a1_description/        ┐
├── onero_a1_driver/             │  4 个 ROS 包（与 UR / Kinova 同构）
├── onero_a1_bringup/            │
└── onero_a1_moveit_config/      ┘
         │
         ├──▶ bloom ──▶ PR 到 ros/rosdistro ──▶ ROS build farm
         │        └──▶ ros-jazzy-onero-a1 进入官方 apt 源     ★ 最实质的「合入」
         │
         ├──▶ PR ──▶ moveit.ros.org
         │        └──▶ 出现在 moveit.ai/robots 官方列表        ★ 最直接的「官方认可」
         │
         └──▶ PR ──▶ moveit/moveit2（通用改进）
                  └──▶ 代码进入 MoveIt 2 本体                  ★ 唯一的「代码合入」
```

**三个 ✅ 加起来，才是"机械臂被 MoveIt 官方生态接纳"的完整形态。**
论实际影响力：**通道 1 > 通道 2 > 通道 3**。

### 7.4 与选型的关系：两个方案在此完全相同

> **方案 A 和方案 B 在"合入官方"这件事上没有任何区别。**
>
> 两者都走"自有仓库 → rosdistro 发布 → 提列表 PR"，**都不需要修改 moveit2 一行代码**。

因此——**"能不能合入官方"不构成第 3 节的选型依据**。选型请回到第 6 节的决策标准。

### 7.5 推进顺序建议

| 阶段 | 动作 | 前置条件 |
|---|---|---|
| 1 | 4 个包在自有仓库跑通 | 集成完成（描述层 + 执行层） |
| 2 | 发布到 rosdistro | 包规范、CI、版本号策略 |
| 3 | PR 到 moveit.ros.org | 驱动公开可用 |
| 4 | 顺带提交 moveit2 通用 PR | 在集成过程中发现通用问题 |

### 7.6 一个高可行性的 moveit2 PR 选题（示例）

在集成过程中我们发现了 MoveIt 2 一个真实且**完全通用**的痛点：

> **参数放错命名空间时会「静默失效」，不报错。**

| 场景 | 现状 | 应有行为 |
|---|---|---|
| `joint_limits.yaml` 未包在 `robot_description_planning` 下 | **无任何输出**，静默使用 URDF 限位 | 启动时警告"检测到未使用的 `joint_limits.*` 参数" |
| `kinematics.yaml` 未包在 `robot_description_kinematics` 下 | 仅一句含糊的 "No kinematics plugins defined" | 明确指出期望的参数路径 |

**为什么这是个好的上游 PR：**

- ✅ **完全通用**——与本项目是哪个厂商的机械臂**毫无关系**，所有用户都可能踩
- ✅ **有安全动机**——限位静默失效是真实的安全隐患
- ✅ **符合官方评审标准**——"general enough to work with everyone's use-case" 完美满足
- ✅ **易于证明价值**——可写出"没有此 PR 即失败"的测试
- ✅ **小而聚焦**——符合官方"smallest digestible chunks"的要求

**这类由集成经验自然产出的通用改进，才是真正能进入 moveit2 的贡献。**

---

## 附录 A：术语表

| 术语 | 说明 |
|---|---|
| **URDF** | 机器人物理结构描述（连杆、关节、尺寸、限位） |
| **SRDF** | MoveIt 专有。补充语义信息：规划组、末端、碰撞矩阵、预设姿态 |
| **action server** | ROS 2 的 action 服务端。此处指实现 `FollowJointTrajectory` 的节点 |
| **ros2_control** | ROS 2 标准硬件抽象框架。**不是 MoveIt 的一部分，也不是必须的** |
| **controller_manager** | ros2_control 的核心节点，管理控制器与硬件的生命周期 |
| **`joint_trajectory_controller`** | ros2_control 现成控制器，接收轨迹并插值下发 |
| **硬件组件 / `SystemInterface`** | ros2_control 中真正读写硬件的插件，**必须是 C++** |
| **命令接口 / 状态接口** | ros2_control 中控制器与硬件之间的数据通道，靠**名字**对接 |
| **话题桥** | 指 `topic_based_ros2_control`，把 ros2_control 桥接到普通话题的通用插件 |
| **MIT 阻抗控制** | 电机内部的 PD 控制：`τ = kp·Δp + kd·Δv + τ_ff` |
| **`servoj`** | 逐周期关节目标命令（非阻塞），MoveIt 轨迹的执行方式 |
| **`movej` / `movel` / `movep`** | SDK 自行规划的阻塞式运动命令，与 MoveIt 形成竞争关系 |
| **freedrive** | 拖动示教模式（kp≈0），机械臂可被手动拖动 |

## 附录 B：建议的实测方案

选型确定前，建议实测以下数据（约需 30 分钟）：

| 测量项 | 方法 | 关注点 |
|---|---|---|
| 话题往返延迟 | `ros2 topic delay /robot_joint_states` | 方案 B 的状态上行延迟 |
| 话题频率 | `ros2 topic hz /robot_joint_commands` | 是否与 `update_rate` 一致 |
| 控制周期抖动 | 在硬件组件的 `read()`/`write()` 中记录耗时分布 | **最关键的一项** |
| CAN 往返 | SDK 侧打点 | 对比 IPC 开销与 CAN 开销的占比 |
| 端到端 | MoveIt 发目标 → 电机实际到位的时间差 | 最终用户体验 |

> **原则：不要凭架构估算做决策，实测数据才是依据。**

## 附录 C：信息来源

**本机源码核实**（`/home/initialxx/project/moveit2`，main @ 2.15.2；ros2_control Jazzy 4.48.1）
- `moveit_plugins/moveit_simple_controller_manager/`（依赖、参数名、action 客户端）
- `moveit_ros/planning/kinematics_plugin_loader/src/kinematics_plugin_loader.cpp:260`（运动学参数命名空间）
- `moveit_ros/planning/planning_pipeline/src/planning_pipeline.cpp:91,136-141`（管线参数命名空间、空插件抛异常）
- `moveit_ros/move_group/src/move_group.cpp:243-291`（`planning_pipelines` 参数与回退行为）
- `moveit_configs_utils/moveit_configs_utils/moveit_configs_builder.py`（参数拼装规则）
- `moveit_setup_assistant/moveit_setup_controllers/src/moveit_controllers_config.cpp:164-165`（MSA 默认控制器管理器）
- `/opt/ros/jazzy/include/hardware_interface/.../system_interface.hpp:62-63`（标准接口列表）
- `/opt/ros/jazzy/include/hardware_interface/.../hardware_info.hpp:410`（gpio 接口）
- `control_msgs/action/FollowJointTrajectory`（字段定义）
- 上游 `topic_based_ros2_control` 的 `src/topic_based_system.cpp` 与头文件

**官方文档与官方仓库**
- MoveIt 2 Controller Configuration 教程（"not required to use ros2_control"）
- MoveIt 2 Contributing / Pull Requests（评审标准："general enough to work with everyone's use-case"）
- MoveIt 2 FAQ（机器人列表 PR 流程指向 moveit.ros.org）
- ros2_control issue #1615（Python 硬件接口不可行；话题桥方案推荐）
- ros2_control 官方文档：Writing a Hardware Component / Handling Errors During read() and write() / Asynchronous Components
- **`ros/rosdistro` 的 `jazzy/distribution.yaml`**（839 个仓库条目；`ur_robot_driver`、
  `ros2_kortex` 等厂商驱动的真实条目，本文第 7.2 节所引）
- `PickNikRobotics/topic_based_ros2_control`（方案 B 的插件本体）

**厂商参考实现**
- `TrossenRobotics/trossen_arm_ros`（jazzy 分支）——最佳小模板
- `UniversalRobots/Universal_Robots_ROS2_Driver`——`non_blocking_read` + 异步线程
- `frankarobotics/franka_ros2`——Jazzy 原生异步硬件组件
- `Kinovarobotics/ros2_kortex`——7-DOF 结构最相近，**且 rosdistro 条目结构可直接套用**
- `xArm-Developer/xarm_ros2`——多命令接口写法

---

## 附录 D：待决策事项清单

| # | 事项 | 归属 | 影响 |
|---|---|---|---|
| 1 | **方案 A / B 选型** | 技术决策 | 本文核心 |
| 2 | kp/kd 侧信道的接口形式（参数 / service 档位 / 话题流） | 工程 | 两方案都需要 |
| 3 | `movej` 系列是否对外暴露 | 产品 + 工程 | 需定义互斥策略 |
| 4 | 是否需要 `admittance_controller` 等生态控制器 | 产品 | 影响方案 B 的吸引力 |
| 5 | 长期是否投入 C++ `SystemInterface` | 战略 | 影响产品化路径 |
| 6 | **ROS 包命名与拆分方案**（`onero_a1_description` / `_driver` / `_bringup` / `_moveit_config`） | 工程 | rosdistro 发布的前置条件 |
| 7 | **包版本号策略与 CI**（REP-144 规范） | 工程 | rosdistro 发布的前置条件 |
| 8 | **rosdistro 发布计划** | 产品 | 生态入口（通道 1） |
| 9 | **moveit.ros.org 列表 PR 计划** | 产品 | 官方可见性（通道 2） |
| 10 | 是否投入人力做 moveit2 通用改进 PR | 技术 + 市场 | 技术声誉（通道 3） |
| 11 | 状态看门狗的实现位置 | 工程 | **安全相关** |
| 12 | 开源许可证与仓库可见性 | 法务 + 产品 | 影响通道 1 / 2 的可行性 |

> **提示**：第 6–9、12 项是「合入官方」的前置条件，建议与第 1 项并行推进，
> 因为它们与执行层选型**互不依赖**（详见第 7.4 节）。
