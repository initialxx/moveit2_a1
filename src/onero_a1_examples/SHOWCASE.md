# 功能演示指令

这组动作通过较明显的位移、转向和路径形状展示 MoveIt 功能。
适用场景：A1R、Woan Picker、`humanoid_right` 安装方向、
`environment.desktop.yaml` 桌面支架环境，以及已确认安全的悬空启动姿态。

所有手臂运动指令使用 `--velocity 0.5 --acceleration 0.5`，表示规划限值的 50%。
位移单位为米，角度为弧度；默认参考系为 `world`，X 向前、Y 向左、Z 向上。
位置和路径描述的是 `Link7` 法兰。

## 启动与演示顺序

在已验证的悬空姿态启动服务，以该姿态作为每项演示的回位基准：

```bash
cd /home/initialxx/project/moveit2_ws
source setup_env.sh

ros2 launch onero_a1_moveit_config moveit.launch.py \
  hardware_mode:=real \
  mounting:=humanoid_right \
  with_gripper:=true \
  environment_file:="$PWD/src/onero_a1_moveit_config/config/environment.desktop.yaml"
```

另一终端进入工作区并加载环境：

```bash
cd /home/initialxx/project/moveit2_ws
source setup_env.sh
```

先空载闭合夹爪，使演示起点的工具状态一致：

```bash
ros2 run onero_a1_examples gripper --opening 0 \
  --velocity 0.5 --acceleration 0.5 --execute
```

每项先去掉 `--execute` 规划预览，确认目标和整个路径适合现场后再执行。
每个动作结束后停留约 2 秒，再执行该项下方的回位命令。
相对运动从当前反馈计算目标，失败时先查看原因，避免连续重复。
启动姿态发生改变后，应重新核对各动作的可达性和空间。

| 演示 | 观众看到的动作 | 功能重点 |
|---|---|---|
| 单关节运动 | 根部关节抬臂约 34° | 指定一个关节的角度目标 |
| 末端位置目标 | 向后 5 cm、向右 10 cm、向上 10 cm 到达新位置 | 多关节协同到达末端目标，中途路径可弯曲 |
| 末端朝向目标 | 法兰位置基本保持，末端向左转约 29° | 指定末端朝向 |
| 竖直直线 | 向上 12 cm | 法兰沿直线移动并保持朝向 |
| 水平直线 | 向右 12 cm | 法兰沿水平直线移动 |
| 竖直正方形 | 在 YZ 平面画边长 10 cm 的方形 | 多段笛卡尔路径，从正面观察 |
| 姿态保持 | 夹爪保持原朝向，斜向右上方搬运 | 整条规划路径的朝向约束 |
| 障碍物避让 | RViz 出现直径 4 cm 的球体，手臂改变路径绕行 | 场景更新、碰撞检查与重规划 |
| 命名姿态 | 手臂整体切换到 `ready` | 按预设的七关节角度规划 |
| 启动位返回 | 回到本轮服务启动时的位置 | 使用启动快照作为关节目标 |

## 1. 单关节抬臂

joint1 增加 `0.6 rad`，约 34°。观察根部关节带动手臂抬起。
其他关节的终点目标保持原值，规划中仍可配合避障。

```bash
ros2 run onero_a1_examples joint_goal \
  --joint joint1-a1_r --delta 0.6 \
  --velocity 0.5 --acceleration 0.5 --execute

ros2 run onero_a1_examples return_startup \
  --velocity 0.5 --acceleration 0.5 --execute
```

## 2. 末端到达指定位置

法兰目标为向后 5 cm、向右 10 cm、向上 10 cm。
重点观察多关节如何共同把末端送到新位置；该示例只约束终点位姿。

```bash
ros2 run onero_a1_examples pose_goal \
  --dx -0.05 --dy -0.10 --dz 0.10 \
  --velocity 0.5 --acceleration 0.5 --execute

ros2 run onero_a1_examples return_startup \
  --velocity 0.5 --acceleration 0.5 --execute
```

## 3. 末端转向

法兰目标位置保持，绕 world 的 Z 轴向左转 `0.5 rad`，约 29°。
各关节共同配合实现该末端姿态。

```bash
ros2 run onero_a1_examples pose_goal \
  --dx 0 --dy 0 --dz 0 --yaw 0.5 \
  --velocity 0.5 --acceleration 0.5 --execute

ros2 run onero_a1_examples return_startup \
  --velocity 0.5 --acceleration 0.5 --execute
```

## 4. 竖直直线

观察法兰向上走 12 cm，规划过程中保持朝向。
与单关节抬臂相比，该项要求末端沿竖直直线运动。

```bash
ros2 run onero_a1_examples cartesian_path \
  --shape line --dx 0 --dy 0 --dz 0.12 \
  --velocity 0.5 --acceleration 0.5 --execute

ros2 run onero_a1_examples return_startup \
  --velocity 0.5 --acceleration 0.5 --execute
```

## 5. 水平直线

观察法兰向右走 12 cm，规划过程中保持高度和朝向。

```bash
ros2 run onero_a1_examples cartesian_path \
  --shape line --dx 0 --dy -0.12 --dz 0 \
  --velocity 0.5 --acceleration 0.5 --execute

ros2 run onero_a1_examples return_startup \
  --velocity 0.5 --acceleration 0.5 --execute
```

## 6. 竖直画正方形

从正面观察 YZ 平面：先到达后方 5 cm、上方 2 cm 的绘制起点，
再依次向左、向上、向右、向下各走 10 cm。
方形结束时停在绘制起点，随后用 `return_startup` 返回演示基准。

```bash
ros2 run onero_a1_examples cartesian_path \
  --shape square --plane yz --size 0.10 \
  --dx -0.05 --dy 0 --dz 0.02 \
  --velocity 0.5 --acceleration 0.5 --execute

ros2 run onero_a1_examples return_startup \
  --velocity 0.5 --acceleration 0.5 --execute
```

`--plane` 支持 `xy`、`xz`、`yz`，平面坐标按 `--frame` 解释，默认 `yz`，可省略。
规划的方形路径与实物跟踪精度分别评价；查看完整四边及拐角，而非只看最后是否回位。

## 7. 保持朝向搬运

法兰向后 4 cm、向右 10 cm、向上 10 cm 移动。
规划要求全过程保持原朝向，各轴允许的路径朝向偏差为 `0.05 rad`，约 2.9°。
可以把它讲解为“端平物体移动”，本例只演示姿态约束，不包含物体抓取或附着。

```bash
ros2 run onero_a1_examples orientation_constraint \
  --dx -0.04 --dy -0.10 --dz 0.10 \
  --path-angle-tolerance 0.05 --planning-time 20 \
  --velocity 0.5 --acceleration 0.5 --execute

ros2 run onero_a1_examples return_startup \
  --velocity 0.5 --acceleration 0.5 --execute
```

## 8. 障碍物避让

同时展示 RViz：先规划 joint4 增加 `0.4 rad` 的动作，再在原路线中间放置直径 4 cm 的虚拟球体，
重新规划绕行。球体位置通过 FK 和碰撞查询选择，保证起点、终点可用，原路径中间被阻挡。
动作完成后删除球体。该项的看点是绕行路径和场景变化。

```bash
ros2 run onero_a1_examples planning_scene \
  --scene-delta 0.4 --obstacle-radius 0.03 \
  --planning-time 20 \
  --velocity 0.5 --acceleration 0.5 --execute

ros2 run onero_a1_examples return_startup \
  --velocity 0.5 --acceleration 0.5 --execute
```

## 9. 命名姿态与回位

`ready` 是模型中的固定七关节姿态，动作范围较大。
先预览整个姿态切换和桌后空间，确认现场净空后执行；然后返回本轮启动快照。
命名姿态与启动位返回的区别在于目标来源，需要结合指令和 RViz 说明。

```bash
ros2 run onero_a1_examples named_pose --target ready \
  --planning-time 20 \
  --velocity 0.5 --acceleration 0.5 --execute

ros2 run onero_a1_examples return_startup \
  --velocity 0.5 --acceleration 0.5 --execute
```

## 10. 夹爪开合

空载依次全开、半开、全闭，每次完成后停留约 2 秒。

```bash
ros2 run onero_a1_examples gripper --opening 1 \
  --velocity 0.5 --acceleration 0.5 --execute

ros2 run onero_a1_examples gripper --opening 0.5 \
  --velocity 0.5 --acceleration 0.5 --execute

ros2 run onero_a1_examples gripper --opening 0 \
  --velocity 0.5 --acceleration 0.5 --execute
```

夹爪使用 `GripperCommand` 发送最终开度。上述速度、加速度参数只影响夹爪规划轨迹，
实际开合速度由适配层的开度斜坡控制，未提供独立的实际开合加速度设置。

## 11. 碰撞查询

该功能通过 RViz 场景和终端结果展示，不产生实物运动，也没有速度、加速度参数。
查询当前姿态或命名姿态的有效性，查看返回值和碰撞接触对象：

```bash
ros2 run onero_a1_examples collision_check
ros2 run onero_a1_examples collision_check --target ready
ros2 run onero_a1_examples collision_check --target home
```

命令正常返回表示查询完成，`状态有效：True/False` 才表示所查询姿态的有效性。
