# Onero A1 MoveIt 2 Examples

本包提供机械臂功能示例：每个功能一个 Python 文件，展示目标构造和标准 MoveIt 调用。
默认只规划，添加 `--execute` 才执行。目标适用于 A1R 七关节和 Woan Picker。

## 代码结构

| 入口 | 源码 | 展示功能 |
|---|---|---|
| `joint_goal` | [joint_goal.py](onero_a1_examples/joint_goal.py) | 相对关节目标与关节空间规划 |
| `named_pose` | [named_pose.py](onero_a1_examples/named_pose.py) | 从 SRDF 读取 `ready` / `home` |
| `pose_goal` | [pose_goal.py](onero_a1_examples/pose_goal.py) | 法兰位置及朝向目标 |
| `cartesian_path` | [cartesian_path.py](onero_a1_examples/cartesian_path.py) | 笛卡尔直线、XY/XZ/YZ 平面正方形路径 |
| `orientation_constraint` | [orientation_constraint.py](onero_a1_examples/orientation_constraint.py) | 运动过程保持朝向 |
| `planning_scene` | [planning_scene.py](onero_a1_examples/planning_scene.py) | 临时碰撞体、避让及清理 |
| `collision_check` | [collision_check.py](onero_a1_examples/collision_check.py) | 查询当前或命名姿态的有效性与接触对 |
| `gripper` | [gripper.py](onero_a1_examples/gripper.py) | 通过 MoveIt 规划并执行夹爪开合 |
| `return_startup` | [return_startup.py](onero_a1_examples/return_startup.py) | 读取适配层启动姿态并规划返回 |

公共接口位于 `onero_a1_common/moveit_client.py`，封装状态读取、FK、规划、显示、执行及取消。
[support.py](onero_a1_examples/support.py) 仅处理示例命令行和节点生命周期。
示例通过公共接口调用 MoveIt，再由控制器、适配层和 SDK 控制机械臂。

较大幅度的动作、观察重点和逐项回位指令见 [功能演示指令](SHOWCASE.md)，
其中手臂运动统一使用 `--velocity 0.5 --acceleration 0.5`。

## 构建和仿真启动

依赖安装见仓库根 README。在工作区根目录运行：

```bash
source setup_env.sh
colcon build --packages-up-to onero_a1_examples --symlink-install
source setup_env.sh
ros2 launch onero_a1_moveit_config moveit.launch.py \
  hardware_mode:=sim mounting:=humanoid_right
```

另一终端同样进入工作区并 `source setup_env.sh`。
下面这一步仅用于模拟模式中进入方便尝试末端运动的姿态：

```bash
ros2 run onero_a1_examples named_pose --target ready --execute
```

## 运行示例

以下命令默认只规划/查询。查看 RViz 路径后，可添加 `--execute` 执行。
`world` 是默认参考系：X 向前、Y 向左、Z 向上。
位移单位为米，角度为弧度；末端目标是 `Link7` 法兰，不是指尖 TCP。

```bash
ros2 run onero_a1_examples joint_goal --joint joint1-a1_r --delta 0.1
ros2 run onero_a1_examples named_pose --target home
ros2 run onero_a1_examples pose_goal --dx 0 --dy 0 --dz 0.02
ros2 run onero_a1_examples pose_goal --dx 0 --dy 0 --dz 0 --yaw 0.1
ros2 run onero_a1_examples cartesian_path --shape line --dx 0 --dy 0 --dz 0.02
ros2 run onero_a1_examples cartesian_path --shape square --size 0.02 --dx 0 --dy 0 --dz 0
ros2 run onero_a1_examples cartesian_path --shape square --plane xz --size 0.02 --dx 0 --dy 0 --dz 0
ros2 run onero_a1_examples orientation_constraint --dx 0 --dy 0 --dz 0.02 --planning-time 20
ros2 run onero_a1_examples planning_scene --scene-delta 0.4 --obstacle-radius 0.01
ros2 run onero_a1_examples collision_check
ros2 run onero_a1_examples collision_check --target home
```

在模拟模式中执行夹爪开合：

```bash
ros2 run onero_a1_examples gripper --opening 1 --execute
ros2 run onero_a1_examples gripper --opening 0.5 --execute
ros2 run onero_a1_examples gripper --opening 0 --execute
```

从启动位置移动后，再运行 `ros2 run onero_a1_examples return_startup --execute`。
`return_startup` 在 sim / real 中均使用服务启动时记录的关节位置。

## 真机启动

先结束模拟服务。桌面支架配置使用 `environment.desktop.yaml`：

```bash
ros2 launch onero_a1_moveit_config moveit.launch.py \
  hardware_mode:=real mounting:=humanoid_right with_gripper:=true \
  allow_execution:=false \
  environment_file:="$PWD/src/onero_a1_moveit_config/config/environment.desktop.yaml"
```

该命令使能并保持实测姿态，同时关闭轨迹执行以预览路径。
核对实际空间和规划路径后，以 `allow_execution:=true` 重新启动，
再对合适的示例添加 `--execute`。执行时会重新规划。

真机的安装方向、模型和环境需与现场一致。真机默认 kp=50、kd=1、100 Hz，
速度前馈与重力补偿开启。若小臂接触桌面，先确认离桌路径，建立悬空测试位；
模型的 `ready/home` 需按实际空间核对，不能直接代替现场测试位。
`return_startup` 会返回服务启动时捕获的位置，包括启动时在桌面上的情况。

## 规划器选择

支持 OMPL、CHOMP、STOMP，默认使用 OMPL / RRTConnect。

| 示例 | OMPL | CHOMP | STOMP |
|---|---|---|---|
| `joint_goal`、`named_pose`、`return_startup`、`planning_scene` | 支持 | 支持 | 支持 |
| `pose_goal` | 支持 | 不支持 | 支持 |
| `orientation_constraint` | 支持 | 不支持 | 不支持 |
| `gripper` | 支持 | 不支持 | 不支持 |

夹爪固定使用 OMPL / RRTConnect。`cartesian_path` 使用独立的笛卡尔路径接口，
`collision_check` 只查询碰撞，两者不选择规划器。

用法：

```bash
ros2 run onero_a1_examples joint_goal --pipeline ompl --planner RRTstar
ros2 run onero_a1_examples joint_goal --pipeline chomp
ros2 run onero_a1_examples joint_goal --pipeline stomp
```

## 参数

### 公共参数

| 参数 | 默认值 | 含义 / 可选值 | 适用示例 |
|---|---|---|---|
| `--frame` | `world` | 参考系：`world`、`base_link` | 全部 |
| `--execute` | 关闭 | 执行规划轨迹；省略时只规划 | 除 `collision_check` 外 |
| `--velocity` | `0.05` | 速度缩放，范围 `(0, 1]` | 除 `collision_check` 外 |
| `--acceleration` | `0.05` | 加速度缩放，范围 `(0, 1]` | 除 `collision_check` 外 |
| `--planning-time` | `10` | 规划请求时间预算，单位秒 | 除 `collision_check` 外 |
| `--pipeline` | `ompl` | 规划管线：`ompl`、`chomp`、`stomp`；适用范围见上表 | 手臂规划示例，`cartesian_path` 除外 |
| `--planner` | `RRTConnect` | OMPL 算法：`RRTConnect`、`RRTstar`、`PRM`、`EST`、`LBKPIECE`、`BiTRRT`、`AnytimePathShortening` | 使用 OMPL 的手臂规划示例 |
| `--help` | — | 显示当前示例帮助 | 全部 |

CHOMP/STOMP 省略 `--planner`。夹爪的速度、加速度参数仅作用于规划轨迹，实际开合由适配层开度斜坡控制。

### 功能参数

| 参数 | 默认值 | 含义 / 可选值 | 适用示例 |
|---|---|---|---|
| `--joint` | `joint1-a1_r` | 目标关节，可选 `joint1-a1_r` 至 `joint7-a1_r` | `joint_goal` |
| `--delta` | `0.1` | 关节相对转角，单位 rad | `joint_goal` |
| `--target` | `ready` / 当前姿态 | SRDF 姿态名：`home`、`ready`；命名姿态默认 `ready`，碰撞查询默认当前姿态 | `named_pose`、`collision_check` |
| `--dx`、`--dy`、`--dz` | `0`、`0`、`0.01` | 沿参考系 X、Y、Z 的相对位移，单位米 | `pose_goal`、`cartesian_path`、`orientation_constraint` |
| `--roll`、`--pitch`、`--yaw` | 均为 `0` | 绕参考系 X、Y、Z 轴的增量旋转，单位 rad | `pose_goal` |
| `--shape` | `line` | 路径形状：`line` 直线、`square` 正方形 | `cartesian_path` |
| `--size` | `0.02` | 正方形边长，单位米，必须大于 0 | `cartesian_path` |
| `--plane` | `yz` | 正方形平面：`xy`、`xz`、`yz` | `cartesian_path` |
| `--path-angle-tolerance` | `0.1` | 路径中各轴允许的朝向偏差，单位 rad，必须大于 0 | `orientation_constraint` |
| `--scene-delta` | `0.4` | joint4 目标角度的相对变化，单位 rad | `planning_scene` |
| `--obstacle-radius` | `0.01` | 球形障碍物半径，单位米，必须大于 0 | `planning_scene` |
| `--opening` | `1` | 夹爪归一化开度，范围 `[0, 1]`；0 全闭、1 全开 | `gripper` |

### 规划器配置

| 配置文件 | 参数 | 默认值 | 含义 |
|---|---|---|---|
| [chomp_planning.yaml](../onero_a1_moveit_config/config/chomp_planning.yaml) | `planning_time_limit` | `10.0` | CHOMP 优化时间上限，单位秒 |
| [chomp_planning.yaml](../onero_a1_moveit_config/config/chomp_planning.yaml) | `max_iterations` | `200` | CHOMP 最大优化轮数 |
| [stomp_planning.yaml](../onero_a1_moveit_config/config/stomp_planning.yaml) | `stomp_moveit.num_iterations` | `40` | STOMP 最大优化轮数 |

规划时间预算和优化轮数影响路径搜索，不代表机械臂的运动时长。修改配置文件后需重新启动服务。
服务端启动参数见 [启动模式与默认值](../../README.md#启动模式与默认值)及
[机械臂执行容差](../../README.md#机械臂执行容差)。

## 运行结果

规划成功时输出轨迹点数和时长，执行成功时输出 MoveIt 执行结果；失败时返回非零状态码。
笛卡尔路径不完整时拒绝执行。碰撞查询输出状态有效性和接触对象。
Ctrl+C 请求取消活动 Action，并清理当前示例创建的临时场景对象。

精度报告、姿态存档、轨迹回放和 SDK 对照见 [诊断工具](../../tools/diagnostics/README.md)。

## world 目标如何变成运动

启动文件按 `mounting` 发布 `world → base_link` 的 TF。
示例请求中标明参考系，MoveIt 结合 TF 和 URDF 求解关节目标并规划轨迹，
通过 `ExecuteTrajectory → 控制器 → 适配层 → SDK → CAN` 执行。
按当前 humanoid_right 配置，world 的向上 1 cm 对应 base_link 的 +X 方向 1 cm；
各关节如何配合由逆运动学求解。参考系选项不会自动校准安装方向、模型或关节零位。
