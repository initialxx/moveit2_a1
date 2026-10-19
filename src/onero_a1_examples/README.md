# Onero A1 MoveIt 2 Examples

本包面向应用开发人员：每个功能一个 Python 文件，展示目标构造和标准 MoveIt 调用。
默认只规划，添加 `--execute` 才执行。目标适用于 A1R 七关节和 Woan Picker。

## 代码结构

| 入口 | 源码 | 展示功能 |
|---|---|---|
| `joint_goal` | [joint_goal.py](onero_a1_examples/joint_goal.py) | 相对关节目标与关节空间规划 |
| `named_pose` | [named_pose.py](onero_a1_examples/named_pose.py) | 从 SRDF 读取 `ready` / `home` |
| `pose_goal` | [pose_goal.py](onero_a1_examples/pose_goal.py) | 法兰位置及朝向目标 |
| `cartesian_path` | [cartesian_path.py](onero_a1_examples/cartesian_path.py) | 笛卡尔直线、多段正方形路径 |
| `orientation_constraint` | [orientation_constraint.py](onero_a1_examples/orientation_constraint.py) | 运动过程保持朝向 |
| `planning_scene` | [planning_scene.py](onero_a1_examples/planning_scene.py) | 临时碰撞体、避让及清理 |
| `collision_check` | [collision_check.py](onero_a1_examples/collision_check.py) | 查询当前或命名姿态的有效性与接触对 |
| `gripper` | [gripper.py](onero_a1_examples/gripper.py) | 通过 MoveIt 规划并执行夹爪开合 |
| `return_startup` | [return_startup.py](onero_a1_examples/return_startup.py) | 读取适配层启动姿态并规划返回 |

公共接口位于 `onero_a1_common/moveit_client.py`，封装状态读取、FK、规划、显示、执行及取消。
[support.py](onero_a1_examples/support.py) 仅处理示例命令行和节点生命周期。
示例通过公共接口调用 MoveIt，再由控制器、适配层和 SDK 控制机械臂。

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

## 公共参数与结果

- `--frame`：默认 `world`，需要基座坐标时传 `--frame base_link`。
- `--velocity` / `--acceleration`：默认 0.05，表示限速缩放比例。
- `--planning-time`：默认 10 秒；目标可达性与起点、朝向、碰撞及限位有关。
- `--execute`：显式执行，默认只规划；碰撞查询只做只读请求。
- `--help`：显示当前示例参数。

规划成功时打印轨迹点数和时长，执行成功时打印 MoveIt 的执行结果。
失败时打印对应接口的错误并返回非零。笛卡尔路径不完整时拒绝执行。
Ctrl+C 会请求取消活动 Action，并清理本示例创建的临时场景对象。
碰撞查询返回的是 `valid` 和接触对；命令成功返回表示查询完成，不等于该姿态一定无碰撞。

这些示例展示功能调用。精度验收、报告、姿态存档、轨迹回放和 SDK 对照由
[内部诊断工具](../../tools/diagnostics/README.md) 提供，不参与示例执行流程。

## world 目标如何变成运动

启动文件按 `mounting` 发布 `world → base_link` 的 TF。
示例请求中标明参考系，MoveIt 结合 TF 和 URDF 求解关节目标并规划轨迹，
通过 `ExecuteTrajectory → 控制器 → 适配层 → SDK → CAN` 执行。
按当前 humanoid_right 配置，world 的向上 1 cm 对应 base_link 的 +X 方向 1 cm；
各关节如何配合由逆运动学求解。参考系选项不会自动校准安装方向、模型或关节零位。
