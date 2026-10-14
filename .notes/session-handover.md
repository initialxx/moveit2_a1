# 承接说明：旧会话结论与当前状态

> **为什么有这份文档**：项目前期（技术选型、上游政策调研、方案论证）是在**另一个工作区**
> `/home/initialxx/project/moveit2` 的一次会话里完成的。工作区迁移后新会话默认看不到那段
> 上下文，故把**结论性内容**固化到这里，避免重复考古。
>
> | 项 | 值 |
> |---|---|
> | 旧会话 ID | `0f223455-aef1-4d9a-a55f-abd585096520` |
> | 旧会话存档 | `~/.dsh/sessions/--home-initialxx-project-moveit2--/0f223455-.../session.v4.jsonl.zstd` |
> | 会话格式 | DSH session v4（zstd 压缩的 JSONL），可用 `zstdcat` 解开 |
> | 旧工作区 cwd | `/home/initialxx/project/moveit2`（上游 moveit2 克隆，仍在） |
> | 现工作区 | `/home/initialxx/project/moveit2_ws`（`.notes/` `.onero_ws/` `.research/` 已迁入） |
>
> 需要原始细节（完整问答、工具调用记录）时，解开上面的存档再查。

---

## 一、目标

**原始诉求**：把公司自研的 `armsdk` 接入 MoveIt 2，让 MoveIt 2 能控制 Onero A1 七自由度机械臂，
**并希望能给 MoveIt 2 官方提 PR 合入**。

**修订后的目标**（因下述调研结论而调整）：

> 用**话题桥**方案接入：`joint_state_topic_hardware_interface` + 自写 Python 适配节点
> （`onero_topic_bridge`），跑通 `onero_a1_moveit_config` 的 demo（先模拟验证全链路，再接真机）。
> **第二阶段的自写 action server 暂不进行。**
> 最终交付可直接用于 rosdistro 发布的 ROS 包。

---

## 二、最重要的结论：不存在"合入 moveit2"这一步

前期派了两个子代理做上游政策调研（报告见 `.research/REPORT_vendor_ros2_moveit2_integration.md`），
结论是**确定的**：

### 2.1 上游不会接受厂商驱动包

不是成文规定，而是**100% 一致的先例**：

- moveit2 仓库 2599 个路径中，**厂商/驱动包数量为 0**（33 个包全是通用的）
- `ros2_control` 在整个仓库的 `package.xml` 里只出现**一次**，且是 `<test_depend>`
- [moveit.ai/robots](https://moveit.ai/robots/) 列出的 **126/126** 个机器人驱动**全部住在 moveit2 仓库之外**

**UR 就是活证据**：`ros-jazzy-ur-robot-driver` 在 apt 里，依赖 MoveIt 2 生态，但不属于它。

### 2.2 正确的心理模型

MoveIt 2 是**平台**，厂商驱动是**插件/消费者**，两者是**依赖关系**而非**合并关系**：

| 你写的东西 | 接入的平台 | 要合并进去吗 |
|---|---|---|
| VS Code 扩展 | VS Code | ❌ |
| Chrome 插件 | Chrome | ❌ |
| **机械臂驱动** | **MoveIt 2** | ❌ |

没人会问"我的 Chrome 插件怎么合入 Chromium"。

### 2.3 "被官方生态接纳"的三条真实路径

| 路径 | 动作 | 效果 | 价值 |
|---|---|---|---|
| **A** | PR 到 `moveit.ros.org` 仓库 | 进入官方机器人列表（126 个之一） | 可见性 |
| **B** | 发布到 rosdistro（bloom → build farm） | 用户 `sudo apt install ros-jazzy-onero-a1` | **最高** |
| **C** | PR 到 moveit2 提**通用改进** | 真正能合并的贡献 | 生态回馈 |

> **路径 B > 路径 A > 路径 C。** 本项目的交付物形态是"你们自己仓库里的 ROS 包 + rosdistro 发布"，
> 与 UR / Franka / Kinova 完全同构。

### 2.4 路径 C 的一个具体选题（由本项目经验自然产出）

调研中发现 MoveIt 2 一个真实的普遍痛点：**参数放错命名空间会静默失效，不报错**。

| 场景 | 现状 | 应有行为 |
|---|---|---|
| `joint_limits.yaml` 未包在 `robot_description_planning` 下 | 无任何输出，静默用 URDF 限位 | 启动时警告"检测到未被使用的 `joint_limits.*` 参数" |
| `kinematics.yaml` 未包在 `robot_description_kinematics` 下 | 只有含糊的 "No kinematics plugins defined" | 明确指出期望的参数路径 |

**为什么是好 PR**：完全通用（与哪家机械臂无关）、有安全动机（限位静默失效是真隐患）、
符合上游 "general enough to work with everyone's use-case" 的评审标准、小而聚焦、有第一手证据。

> 本项目 `README.md` 的"参数命名空间速查"一节就是踩坑记录，可直接作为 PR 的佐证材料。

---

## 三、技术选型决策链

讨论过程中的关键问答（已浓缩为结论）：

1. **适配层可以是 ros2_control，也可以是自写 action server** —— 两者都成立。
2. **`FollowJointTrajectory` 是事实上落地的接入口**，无论 Python 还是 C++：
   - **C++ 路线**：`ros2_control` + `joint_trajectory_controller`（JTC 实现 action server）
   - **Python 路线**：自己实现 `FollowJointTrajectory` action server，但要**自己补**
     轨迹拆解、容差判定、取消处理等一堆边界逻辑
3. **走 ros2_control 且基于 C++ 是最方便、功能最完整的**；不走则要自行承担边界逻辑。
4. **`kp`/`kd` 只能通过 ROS 参数传入**（启动时设置，运行中不可调）——
   因为 `FollowJointTrajectory` 协议里**没有增益字段**。若要逐周期动态调阻抗，
   必须改用自定义接口。
5. **话题桥的代价**：接口表达受限于话题、实时性有损失（每周期 2 次跨进程通信）。
   不走 ros2_control 可避免这些限制，但要自己写 action server。
6. **"侧信道"（旁路通道）不满足 MoveIt 2 的标准框架**，只能作为补充手段。

**最终决策**：两个方案都要做，但**先走最快的话题桥**把整条链路跑起来，
第二阶段再做自写 action server（**当前已明确暂缓**）。

---

## 四、当前状态（截至 2026-10-14）

### 4.1 工作区布局

```
/home/initialxx/project/moveit2_ws/
├── .gitignore              # 41 行，忽略 8 类产物
├── .notes/                 # 选型对比 + 技术附录 + 接入指南 + 本文档
├── .onero_ws/              # ★ 实际开发工作区
│   ├── src/
│   │   ├── onero_a1_description/     URDF / mesh / ros2_control 硬件配置
│   │   ├── onero_a1_bringup/         launch + 控制器配置 + 话题桥节点 + 看门狗
│   │   └── onero_a1_moveit_config/   SRDF / 运动学 / 限位 / OMPL / 控制器映射 / RViz
│   ├── tools/              # 生成脚本 + 测试脚本 + 诊断脚本
│   ├── docs/               # 真机接入检查清单 + 截图
│   ├── overlay/            # ⚠️ 临时替代 apt 包，正式安装后删除
│   └── setup_env.sh        # 环境入口（必须 source）
└── .research/              # 上游调研报告 + 缓存
```

### 4.2 Git

```
91af54c  初始提交：Onero A1 接入 MoveIt 2（话题桥方案）   52 files, 6241 insertions
```

### 4.3 验证状态

| 模式 | 内容 | 结果 |
|---|---|---|
| 纯 MoveIt 模拟 | 硬件激活 / robot model / OMPL / 控制器 / 参数命名空间 / 关节限位 / 规划 | ✅ |
| 话题桥全链路 | 桥接节点独立测试 / JTC 直发轨迹 / 经 move_group 规划→执行 / 看门狗 | ✅ |
| 真机 | — | ⏳ 待硬件 |

### 4.4 工作区迁移后的复验（2026-10-14）

迁移导致 `build/` `install/` 里的绝对路径失效，已**清理并重新编译**（3 包，2.15s），
随后重跑端到端验收：

```
经 move_group「规划 → 执行」joint1-a1_r → 0.4 rad
  error_code = 1（SUCCESSFUL）
  规划轨迹点数 = 21
  实际到位 +0.4193，误差 0.0193 rad（阈值 0.05）
  → 通过 ✓
```

复验脚本：`bash tools/verify_workspace.sh [关节名] [目标位置]`

---

## 五、待办清单

| # | 事项 | 来源 | 阻塞 |
|---|---|---|---|
| 1 | **接真机验证**（模式 3） | 目标本身 | 需要硬件 + CAN |
| 2 | KDL 根连杆惯量警告的处理方式（需先定修法） | 检查清单 5.1 | 需决策 |
| 3 | `kp`/`kd` 运行中可调（需自定义接口） | 检查清单 5.4 | 需决策 |
| 4 | 正式 `apt install` 话题桥插件并删除 `overlay/` | 环境 | 无（换正式环境时做） |
| 5 | rosdistro 发布布局调整（包目前在隐藏目录 `.onero_ws/src/`） | 路径 B | 需决策 |
| 6 | 路径 A：PR 到 moveit.ros.org 机器人列表 | 上游接纳 | 依赖 1 完成 |
| 7 | 路径 C：参数命名空间静默失效的上游 PR | 调研所得 | 无 |
| 8 | 第二阶段：自写 action server | 目标（暂缓） | 用户明确暂缓 |

---

## 六、环境坑（会重复踩，务必记住）

### 6.1 DSH 沙箱：后台进程随调用结束被杀

沙箱用 `bwrap --die-with-parent` 包裹**每一次** bash 调用。`cmd &` 起的进程在该次调用
返回后立刻被杀。**因此「启动 ROS 栈 + 跑测试 + 收尾」必须写在同一次 bash 调用里**
（`tools/verify_workspace.sh`、`tools/test_watchdog.sh` 都是这么写的）。

### 6.2 不要用 `set -u`

`/opt/ros/jazzy/setup.bash` 第 8 行引用未绑定的 `AMENT_TRACE_SETUP_FILES`，
在 `set -u` 下 source 会**直接中止**，表现为"脚本一行输出都没有、退出码 127"。

### 6.3 `$HOME` 与 `/opt/ros` 只读

`ros2 launch` 默认要写 `~/.ros/log` 会失败。必须 `source setup_env.sh`（把 `ROS_HOME` 指到工作区内）。

### 6.4 sandbox 的 `/tmp` 是独立 tmpfs

会遮住宿主机的 `/tmp/.X11-unix/X1`。**RViz 在 AI 工具的 shell 里起不来是误判**，
用户在自己的终端里直接跑是可以的（见 `docs/screenshots/demo_rviz.png`）。

### 6.5 URDF / SRDF 必须由脚本生成，禁止手改

`onero_a1_body.urdf` 与 `onero_a1.srdf` 由 `tools/sync_*_from_sdk.py` 从 armsdk 权威文件生成，
**带自校验**。历史上手抄 URDF 导致过末端恒定偏差 19.76 mm、Link3–Link6 坐标系差 180° 的
事故，且**不报错**，极难排查。

---

## 七、相关文档索引

| 文档 | 内容 |
|---|---|
| `.onero_ws/README.md` | 主文档：包结构、三种运行模式、数据流、安全设计、两个隐蔽 bug 的调试记录 |
| `.onero_ws/docs/真机接入检查清单.md` | 上真机前的逐项确认 |
| `.notes/onero-moveit2-adapter-comparison.md` | 自写 action server vs 话题桥 的对比 |
| `.notes/onero-moveit2-adapter-comparison-techappendix.md` | 上文的详细技术附录 |
| `.notes/onero-moveit2-integration-guide.md` | 接入指南 |
| `.research/REPORT_vendor_ros2_moveit2_integration.md` | 上游政策调研完整报告（含原始片段与 verified/inferred 标注） |
