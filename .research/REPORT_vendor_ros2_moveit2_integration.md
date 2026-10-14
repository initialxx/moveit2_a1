# Vendor ROS 2 + MoveIt 2 + ros2_control integration — reference survey

Legend: **[V]** = verified by fetching the raw file / GitHub API tree / local compiled header.
**[I]** = inference or recollection, not directly fetched.

Local ground truth used throughout: `/opt/ros/jazzy` with `ros-jazzy-hardware-interface 4.48.1-1noble`
(`HARDWARE_INTERFACE_VERSION_STR "4.48.1"`), `ros-jazzy-controller-interface 4.48.1`. **[V]**

---

## 1. Vendor repo structures

### 1.1 Universal Robots — `UniversalRobots/Universal_Robots_ROS2_Driver` @ `humble` **[V]**

Packages (dirs containing `package.xml`):

| package | role |
|---|---|
| `ur_robot_driver` | **ros2_control SystemInterface + driver node** |
| `ur_bringup` | launch files, `config/ur_controllers.yaml` |
| `ur_moveit_config` | **MoveIt config package** |
| `ur_controllers` | vendor controllers (e.g. scaled JTC, passthrough) |
| `ur_calibration` | kinematics calibration |
| `ur_dashboard_msgs` | msgs |
| `ur` | metapackage |

Hardware interface **[V]**:
- `ur_robot_driver/include/ur_robot_driver/hardware_interface.hpp` → `class URPositionHardwareInterface : public hardware_interface::SystemInterface`
- `ur_robot_driver/src/hardware_interface.cpp` (1722 lines)
- `ur_robot_driver/hardware_interface_plugin.xml`:
  ```xml
  <library path="ur_robot_driver_plugin">
    <class name="ur_robot_driver/URPositionHardwareInterface"
           type="ur_robot_driver::URPositionHardwareInterface"
           base_class_type="hardware_interface::SystemInterface">
  ```
- Extra: `ur_robot_driver/src/ur_ros2_control_node.cpp` — a **custom `ros2_control_node`** built from `controller_manager/controller_manager.hpp` (not just the stock binary). Relevant if you need custom executor setup.

API style **[V]**: **classic** — `on_init(const HardwareInfo&)`, `export_state_interfaces()` returning `std::vector<StateInterface>`, `export_command_interfaces()`, `on_configure/on_activate/on_cleanup/on_shutdown(const rclcpp_lifecycle::State&)`, `read/write(const rclcpp::Time&, const rclcpp::Duration&)`, `prepare_command_mode_switch` / `perform_command_mode_switch`.

MoveIt config package **[V]**: `ur_moveit_config/`
```
ur_moveit_config/config/{controllers.yaml,joint_limits.yaml,kinematics.yaml,ompl_planning.yaml,ur_servo.yaml}
ur_moveit_config/launch/ur_moveit.launch.py
ur_moveit_config/srdf/{ur.srdf.xacro,ur_macro.srdf.xacro}
ur_moveit_config/ur_moveit_config/launch_common.py
```
`ur_moveit_config/package.xml` exec_depends: `moveit_kinematics, moveit_planners_ompl, moveit_ros_move_group, moveit_ros_visualization, moveit_servo, moveit_simple_controller_manager, warehouse_ros_sqlite, rviz2, ur_description, urdf, xacro` **[V]**

**Separate repo from moveit2? YES.** MoveIt is an `exec_depend`, never vendored. The URDF/xacro lives in a *third* repo, `UniversalRobots/Universal_Robots_ROS2_Description` (that repo's root IS the `ur_description` package) **[V]**.

`<ros2_control>` tag **[V]** — `Universal_Robots_ROS2_Description/urdf/ur.ros2_control.xacro`:
```xml
<ros2_control name="${name}" type="system">
  <hardware>
    <xacro:unless value="${use_fake_hardware or sim_gazebo or sim_ignition}">
      <plugin>ur_robot_driver/URPositionHardwareInterface</plugin>
      <param name="robot_ip">${robot_ip}</param>
      <param name="script_filename">${script_filename}</param>
      <param name="non_blocking_read">${non_blocking_read}</param>   <!-- default true -->
      <param name="servoj_gain">2000</param>
      <param name="servoj_lookahead_time">0.03</param>
      ...
```
Note the **fake-hardware switch**: `<plugin>mock_components/GenericSystem</plugin>` when `use_fake_hardware` is true. Copy this pattern — it's universal across vendors.

### 1.2 Franka — `frankarobotics/franka_ros2` @ `jazzy` **[V]**

Packages: `franka_hardware`, `franka_bringup`, `franka_fr3_moveit_config`, `franka_gripper`, `franka_msgs`, `franka_semantic_components`, `franka_robot_state_broadcaster`, `franka_example_controllers`, `franka_gazebo/{franka_gazebo_bringup,franka_gazebo_hardware}`, `franka_mobile*`, `franka_spine/*`, `franka_desk_api`, `franka_selfcollision`, `mobile_fr3_duo_trajectory_controller`, `franka_ros2` (metapackage), plus a **vendored copy of `realtime_tools`** at `realtime_tools/realtime_tools` **[V]**.

Hardware interface **[V]**:
- `franka_hardware/include/franka_hardware/franka_hardware_interface.hpp` → `class FrankaHardwareInterface : public hardware_interface::SystemInterface`
- `franka_hardware/src/franka_hardware_interface.cpp`
- `static const size_t kNumberOfJoints = 7;`
- Uses `realtime_tools/mutex.hpp` + `realtime_tools/realtime_thread_safe_box.hpp` for RT-safe exchange with a separate control thread (`FrankaExecutor`, `Robot`, `ActionServer`).
- Classic API: `on_init(const HardwareInfo&)`, `export_state_interfaces()`, `export_command_interfaces()`, `prepare/perform_command_mode_switch`. Mode enum `ControlInterface { None, Effort, JointVelocity, JointPosition, CartesianVelocity, ..., CartesianPose }` ↔ multiple `command_interface`s on the same joints.

`<ros2_control>` tag **[V]** — `franka_hardware/ros2_control/franka_arm.ros2_control.xacro`:
```xml
<ros2_control name="${arm_prefix}FrankaHardwareInterface" type="system" is_async="${is_async}">
  <xacro:if value="${is_async}">
    <properties>
      <async scheduling_policy="detached" print_warnings="true"
             thread_priority="${thread_priority}" manage_overruns="false"/>
    </properties>
  </xacro:if>
  <hardware>
    <param name="version">1.1.0</param>
    <param name="robot_type">${robot_type}</param>
    <param name="prefix">${arm_prefix}</param>
    <xacro:insert_block name="hardware_plugin"/>
  </hardware>
  ...
```
**This is the single most useful pattern for you** (see §4): Franka runs the hardware component in its own thread via the built-in async API rather than blocking the CM loop.

MoveIt config `franka_fr3_moveit_config/` with `config/{fr3_controllers.yaml, fr3_ros_controllers.yaml, fr3_joint_limits.yaml, kinematics.yaml, ompl_planning.yaml}`, `launch/{moveit.launch.py,move_group.launch.py}` **[V]**. Separate repo from moveit2: **YES**.

### 1.3 Kinova — `Kinovarobotics/ros2_kortex` @ `humble` **[V]** (the `kinova-ros2` name is the older/legacy one; `ros2_kortex` is current **[I]**)

Packages **[V]**: `kortex_api` (wrapper around the downloaded Kinova C++ API — contains only `CMakeLists.txt`/`package.xml`/`CHANGELOG`, i.e. it *finds* the vendor SDK, does not vendor it), `kortex_driver`, `kortex_description`, `kortex_bringup`, and `kortex_moveit_config/{kinova_gen3_6dof_robotiq_2f_85_moveit_config, kinova_gen3_7dof_robotiq_2f_85_moveit_config, kinova_gen3_lite_moveit_config}`.

Hardware interface **[V]**:
- `kortex_driver/include/kortex_driver/hardware_interface.hpp` → `class KortexMultiInterfaceHardware : public hardware_interface::SystemInterface`
- `kortex_driver/src/hardware_interface.cpp` (1047 lines)
- `kortex_driver/hardware_interface_plugin.xml`:
  ```xml
  <library path="kortex_driver">
    <class name="kortex_driver/KortexMultiInterfaceHardware"
           type="kortex_driver::KortexMultiInterfaceHardware"
           base_class_type="hardware_interface::SystemInterface">
  ```

`<ros2_control>` tag **[V]** — `kortex_description/arms/gen3/7dof/urdf/kortex.ros2_control.xacro` (this is a **7-DOF** arm, closest structural analogue to yours):
```xml
<ros2_control name="${name}" type="system">
  <hardware>
    <xacro:unless value="${use_fake_hardware or sim_gazebo or sim_ignition or sim_isaac}">
      <plugin>kortex_driver/KortexMultiInterfaceHardware</plugin>
      <param name="robot_ip">${robot_ip}</param>
      <param name="username">${username}</param>
      <param name="password">${password}</param>
      <param name="port">${port}</param>
      <param name="port_realtime">${port_realtime}</param>
      <param name="session_inactivity_timeout_ms">${session_inactivity_timeout_ms}</param>
      <param name="connection_inactivity_timeout_ms">${connection_inactivity_timeout_ms}</param>
      ...
  <joint name="${prefix}joint_1"> ... </joint>   <!-- ×7, command_interface name="position" only -->
```
Note the **URDF-level joint limits**: `<command_interface name="position"><param name="min">-2.41</param><param name="max">2.41</param>` per joint, plus `<state_interface name="position"><param name="initial_value">`.

`ros2_controllers.yaml` **[V]** — `kortex_description/arms/gen3/7dof/config/ros2_controllers.yaml`:
`controller_manager.ros__parameters.update_rate: 1000`; controllers `joint_state_broadcaster/JointStateBroadcaster`, `joint_trajectory_controller/JointTrajectoryController`, `picknik_twist_controller/PicknikTwistController`, `position_controllers/GripperActionController`, `picknik_reset_fault_controller/PicknikResetFaultController`.

Launch **[V]**: `kortex_bringup/launch/{gen3.launch.py,kortex_control.launch.py,gen3_dual.launch.py,gen3_lite.launch.py}`.

**Blocking-SDK anti-pattern (important, see §4)** — `kortex_driver/src/hardware_interface.cpp` **[V]**:
- `read()` uses a cached `feedback_` from `base_cyclic_.RefreshFeedback()`, refreshed **only on the first pass**.
- `write()` has a `block_write` flag that does `feedback_ = base_cyclic_.RefreshFeedback();` — i.e. a **synchronous network round-trip inside `write()`**.
- The fault-reset path inside `write()` calls the blocking `base_.SetServoingMode()`, `base_.ApplyEmergencyStop()` (twice), `base_.ClearFaults()`.
This is a real, shipped example of exactly what you should *not* copy.

### 1.4 UFactory — `xArm-Developer/xarm_ros2` @ `humble` **[V]**

Packages **[V]**: `xarm_api`, `xarm_controller`, `xarm_description`, `xarm_gazebo`, `xarm_moveit_config`, `xarm_moveit_servo`, `xarm_msgs`, `xarm_planner`, `xarm_sdk` (vendored vendor SDK), `uf_ros_lib` (Python launch helpers / `MoveItConfigsBuilder` subclass), `demo/mbot_demo`, `xarm_vision/d435i_xarm_setup`, `thirdparty/realsense_gazebo_plugin`.

Hardware interface **[V]**:
- `xarm_controller/include/xarm_controller/hardware/uf_robot_system_hardware.h` → `class UFRobotSystemHardware : public hardware_interface::SystemInterface` (namespace `uf_robot_hardware`), plus `uf_robot_fake_system_hardware.{h,cpp}` and a `bak_` variant.
- `xarm_controller/src/hardware/uf_robot_system_hardware.cpp`
- `xarm_controller/uf_hardware_interface_plugins.xml`:
  ```xml
  <library path="uf_robot_hardware">
    <class name="uf_robot_hardware/UFRobotSystemHardware"
           type="uf_robot_hardware::UFRobotSystemHardware"
           base_class_type="hardware_interface::SystemInterface">
    <class name="uf_robot_hardware/UFRobotFakeSystemHardware" ...>
  ```
- Header pulls in `std::thread`, `std::queue`, `controller_manager_msgs/srv/{list_controllers,switch_controller}.hpp`, and holds an `xarm_api::XArmDriver xarm_driver_` member. It also tracks `read_max_time_`, `read_total_time_`, `read_failed_cnts_` — i.e. the vendor **instrumented the blocking read** and can auto-reactivate controllers. Classic API again (`on_init(const HardwareInfo&)`, `export_state_interfaces()`).

Two distinct YAMLs, do not confuse them **[V]**:
- `xarm_controller/config/xarm7_controllers.yaml` → for `controller_manager`:
  `update_rate: 150`; `joint_state_broadcaster/JointStateBroadcaster`; `xarm7_traj_controller/JointTrajectoryController` with `command_interfaces: [position, velocity]`, `state_interfaces: [position, velocity]`, `open_loop_control: true`.
- `xarm_moveit_config/config/xarm7/controllers.yaml` → for **MoveIt**, different schema:
  ```yaml
  controller_names: [xarm7_traj_controller]
  xarm7_traj_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints: [joint1, ..., joint7]
  ```
- Launch: `xarm_controller/launch/{_ros2_control.launch.py, _robot_ros2_control.launch.py, _dual_ros2_control.launch.py, xarm7_control_rviz_display.launch.py}` + `xarm_controller/launch/lib/robot_controller_lib.py`.

### 1.5 Trossen — `TrossenRobotics/trossen_arm_ros` @ `jazzy` **[V]** ← *best small-scale Jazzy template*

This is a **46-file**, clean, Jazzy-targeted repo. Structure **[V]**:
```
trossen_arm_hardware/     include/trossen_arm_hardware/interface.hpp
                          src/interface.cpp
                          trossen_arm_hardware.xml
trossen_arm_bringup/      launch/trossen_arm.launch.py
                          config/{controllers.yaml,dual_arm_controllers.yaml,gravity_compensation.yaml}
trossen_arm_controllers/  gravity_compensation.{hpp,cpp}, trossen_arm_controllers.xml
trossen_arm_moveit/       config/{ros2_controllers.yaml, moveit_controllers.yaml, wxai.srdf.xacro, wxai.urdf.xacro,
                                  kinematics.yaml, joint_limits.yaml, ompl_planning.yaml, pilz_cartesian_limits.yaml,
                                  sensors3d.yaml, initial_positions.yaml, moveit.rviz}
                          launch/moveit.launch.py
                          .setup_assistant          <-- MSA-generated
trossen_arm_ros/          metapackage
dependencies.repos
```

**Three-repo split — the pattern to copy** **[V]** `dependencies.repos`:
```yaml
repositories:
  trossen_arm_description:
    type: git
    url: https://github.com/TrossenRobotics/trossen_arm_description.git
    version: jazzy
  trossen_arm:
    type: git
    url: https://github.com/TrossenRobotics/trossen_arm.git
    version: v1.9.0
```
i.e. `trossen_arm_ros` (ROS glue) + `trossen_arm` (C++ SDK, `libtrossen_arm`) + `trossen_arm_description` (URDF/meshes). MoveIt 2 is **not** vendored — it's a system dependency.

Hardware interface **[V]**:
- `trossen_arm_hardware/include/trossen_arm_hardware/interface.hpp` → `class TrossenArmHardwareInterface : public hardware_interface::SystemInterface`, `using trossen_arm::TrossenArmDriver;`
- `trossen_arm_hardware/trossen_arm_hardware.xml`:
  ```xml
  <library path="trossen_arm_hardware">
    <class name="trossen_arm_hardware/TrossenArmHardwareInterface"
           type="trossen_arm_hardware::TrossenArmHardwareInterface"
           base_class_type="hardware_interface::SystemInterface">
  ```
- `trossen_arm_hardware/package.xml` deps **[V]**:
  `<depend>hardware_interface</depend> <depend>libtrossen_arm</depend> <depend>pluginlib</depend> <depend>rclcpp_lifecycle</depend> <depend>rclcpp</depend>`
- `trossen_arm_hardware/CMakeLists.txt` **[V]**:
  ```cmake
  find_package(ament_cmake REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(libtrossen_arm REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  add_library(${PROJECT_NAME} SHARED src/interface.cpp)
  target_link_libraries(${PROJECT_NAME} ...)
  pluginlib_export_plugin_description_file(hardware_interface ${PROJECT_NAME}.xml)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(${ROS_DEPENDENCIES})
  ```
  **Note: no `ros2_control_cmake` here** — Trossen uses plain `cmake_minimum_required(VERSION 3.10.0)`.
- API style **[V]**: **classic** — `on_init(const HardwareInfo&)`, `export_state_interfaces() -> std::vector<StateInterface>`, `export_command_interfaces() -> std::vector<CommandInterface>`, `on_configure/on_activate/on_deactivate/on_cleanup(const rclcpp_lifecycle::State&)`, `read/write`, `prepare/perform_command_mode_switch`.
- `read()` = pure cache read: `robot_output_ = arm_driver_->get_robot_output();` then copies positions/velocities/efforts. **[V]**
- `write()` = `arm_driver_->set_all_positions(joint_position_commands_, 0.0, false)` (or `set_all_external_efforts`), with a `first_update_` latch that seeds commands from current positions. **[V]**
- No mutex/thread in the hardware interface itself **[V]** — all RT-safety is delegated to the vendor SDK.

### 1.6 AgileX Piper — `agilexrobotics/piper_ros` @ `humble` **[V]** ⚠️

Branches: `noetic` (default), `foxy`, `humble`, `humble_beta1` **[V]**. 367★, MIT, pushed 2026-03-04 **[V]**.

Packages **[V]**: `src/piper`, `src/piper_description`, `src/piper_humble`, `src/piper_msgs`, `src/piper_moveit/{piper_no_gripper_moveit,piper_with_gripper_moveit}`, `src/piper_sim/{piper_gazebo,piper_mujoco}`.

⚠️ **`piper_ros` has NO ros2_control hardware interface.** **[V]**
- `src/piper_humble/` contains **only** `CMakeLists.txt` + `package.xml` — no source.
- The real driver is a plain Python ROS 2 node: `src/piper/piper/piper_ctrl_single_node.py` (+ `piper_ctrl_single_node_new.py`, `piper_read_slave_joint.py`).
- The MoveIt config's `config/piper.ros2_control.xacro` uses `<plugin>mock_components/GenericSystem</plugin>` only, and `demo.launch.py` is a bare `generate_demo_launch(moveit_config)`.
- So on Piper, MoveIt → `arm_controller` works in sim, but real hardware is driven by a *separate* node outside `controller_manager`.

**The actual Python CAN SDK** is a different repo: `agilexrobotics/piper_sdk` (409★, Python, pushed 2026-09-07) **[V]**. Since your robot is SocketCAN + Python SDK, this is the closest *language/runtime* match, but it is **not** a ros2_control integration — treat it as "what the vendor gives you", and you'd be writing the `SystemInterface` yourself. This is the gap you're filling.

### 1.7 AR4 / Annin Robotics **[V]**

**There is no official Annin Robotics ROS 2 driver.** Verified via GitHub search **[V]**:
- `Annin-Robotics/ar4-hmi` (45★, pushed 2025-11-20) — "Annin Robotics AR4 Software Library", not a ROS package.
- `ycheng517/ar4_ros_driver_examples` (10★) — community driver/examples.
- `adob/ar4` (3★, 2024) — firmware/software fork.
- Host-side AR4 ROS support is delivered via the AR4 HMI/`ar4_ros_driver` on Annin's own site, not GitHub **[I]**.

Takeaway: AR4 is **not** a usable structural reference for a vendor-quality ROS 2 stack.

### 1.8 ARX **[V]**

- Official SDK: `ARX` arm SDK repos exist (`arx5-sdk`-style) **[I]**.
- ROS 2: `SimonKennethRobo/arx_ros2` (3★, pushed 2026-01-30, C++) — **third-party, not vendor** **[V]**.
- Not a strong reference.

### 1.9 ros2_control's own reference implementations — `ros-controls/ros2_control_demos` **[V]**

Packages `example_1` … `example_17`. Most relevant:
- **`example_7`** — 6-DOF industrial robot, `RobotSystem` SystemInterface + a custom controller; uses the **new Jazzy API**. Files: `example_7/hardware/include/ros2_control_demo_example_7/r6bot_hardware.hpp`, `example_7/hardware/r6bot_hardware.cpp`, `example_7/ros2_control_demo_example_7.xml`, `example_7/description/ros2_control/r6bot.ros2_control.xacro`, `example_7/CMakeLists.txt`, `example_7/package.xml`.
- `example_3` — `RRBotSystemMultiInterface` (multiple command interfaces per joint — **this is your MIT impedance case**: position + velocity + effort on the same joint).
- `example_6`/`example_14` — Actuator + Sensor decomposition (per-joint plugins).
- `example_17` — publishing diagnostics from a hardware component via `get_node()`.
- `example_1` — minimal SystemInterface, canonical "hello world".

---

## 2. Minimal `SystemInterface` plugin skeleton for ROS 2 Jazzy

### 2.1 Two API generations coexist in Jazzy 4.48.1 — compile-verified **[V]**

I compiled both variants against `/opt/ros/jazzy` headers. Both succeed; the classic one **emits a deprecation warning**:

```
old_api.cpp:8:53: warning: 'virtual CallbackReturn
hardware_interface::HardwareComponentInterface::on_init(const hardware_interface::HardwareInfo&)'
is deprecated: Use on_init(const HardwareComponentInterfaceParams & params) instead. [-Wdeprecated-declarations]
```
New-API file: `exit=0`, **no warnings**. Old-API file: `exit=0`, one deprecation warning. **[V]**

Deprecated → replacement map (from `hardware_component_interface.hpp`) **[V]**:

| deprecated | replacement |
|---|---|
| `on_init(const HardwareInfo &)` | `on_init(const HardwareComponentInterfaceParams &)` |
| `export_state_interfaces()` → `std::vector<StateInterface>` | `on_export_state_interfaces()` → `std::vector<StateInterface::ConstSharedPtr>` |
| `export_command_interfaces()` → `std::vector<CommandInterface>` | `on_export_command_interfaces()` → `std::vector<CommandInterface::SharedPtr>` |
| `init(HardwareInfo, Logger, NodeClockInterface::SharedPtr)` | framework calls `init(HardwareComponentParams)` |

In the new model **the framework creates the interfaces for you** from the `<ros2_control>` XML; you access them via the maps `joint_state_interfaces_`, `joint_command_interfaces_`, `sensor_state_interfaces_`, `gpio_*`, `unlisted_*` (`std::unordered_map<std::string, InterfaceDescription>`) and the templated accessors **[V]**:
```cpp
void set_state(const std::string & interface_name, const T & value);
template <typename T> T get_state(const std::string & interface_name) const;
void set_command(const std::string & interface_name, const T & value);
template <typename T> T get_command(const std::string & interface_name) const;
```
Interface keys are `"<joint_name>/<interface_type>"`, e.g. `"joint_1/position"`. **[V]**

### 2.2 Exact signatures (both generations) **[V]**

From `rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp`:
```cpp
enum class CallbackReturn : uint8_t { SUCCESS, FAILURE, ERROR };
virtual CallbackReturn on_configure(const State & previous_state);
virtual CallbackReturn on_cleanup  (const State & previous_state);
virtual CallbackReturn on_shutdown (const State & previous_state);
virtual CallbackReturn on_activate (const State & previous_state);
virtual CallbackReturn on_deactivate(const State & previous_state);
virtual CallbackReturn on_error    (const State & previous_state);
```
These are **inherited from `LifecycleNodeInterface`**, not declared by `SystemInterface` — which is why they don't appear in `system_interface.hpp`.

From `hardware_interface/system_interface.hpp` (the whole file is 73 lines) **[V]**:
```cpp
namespace hardware_interface {
class SystemInterface : public HardwareComponentInterface
{
public:
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override = 0;
};
}
```
From `hardware_component_interface.hpp` **[V]**:
```cpp
virtual return_type read (const rclcpp::Time & time, const rclcpp::Duration & period) = 0;
virtual return_type write(const rclcpp::Time & time, const rclcpp::Duration & period);
virtual return_type prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces);
virtual return_type perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces);
virtual CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params);
virtual std::vector<StateInterface::ConstSharedPtr>   on_export_state_interfaces();
virtual std::vector<CommandInterface::SharedPtr>      on_export_command_interfaces();
virtual std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_state_interface_descriptions();
virtual std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_command_interface_descriptions();
virtual rclcpp::Clock::SharedPtr get_clock() const;
const HardwareInfo & get_hardware_info() const;
```
`HardwareComponentInterfaceParams` (in `types/hardware_component_interface_params.hpp`) **[V]**:
```cpp
struct HardwareComponentInterfaceParams {
  hardware_interface::HardwareInfo hardware_info;
  rclcpp::Executor::WeakPtr executor;
};
```
`return_type` **[V]**:
```cpp
enum class return_type : std::uint8_t { OK = 0, ERROR = 1, DEACTIVATE = 2 };
```
(`DEACTIVATE` is new — return it from `read()`/`write()` to ask CM to deactivate the component without a hard error.) **[V]**

HW interface name constants come from `hardware_interface/types/hardware_interface_type_values.hpp`: `HW_IF_POSITION`, `HW_IF_VELOCITY`, `HW_IF_ACCELERATION`, `HW_IF_EFFORT`. **[V]**

### 2.3 `package.xml` — exact Jazzy package names **[V]**

Minimum (Trossen, real vendor) **[V]**:
```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<depend>hardware_interface</depend>
<depend>pluginlib</depend>
<depend>rclcpp</depend>
<depend>rclcpp_lifecycle</depend>
<depend>libyour_vendor_sdk</depend>   <!-- your vendor SDK's ament/CMake package -->
```
The docs also mandate exactly these four: "Add at least the following packages into `<depend>` tag: `hardware_interface`, `pluginlib`, `rclcpp`, and `rclcpp_lifecycle`." **[V]** — <https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html>

Optional adds seen in `ros2_control_demos/example_7` **[V]**: `<build_depend>ros2_control_cmake</build_depend>`, `<depend>controller_manager</depend>`, `<depend>realtime_tools</depend>`, `<depend>backward_ros</depend>`, `<depend>control_msgs</depend>`, `<depend>kdl_parser</depend>`, `<depend>trajectory_msgs</depend>`.
Exec deps needed to make MoveIt actually run **[V]**: `joint_state_broadcaster`, `joint_trajectory_controller`, `robot_state_publisher`, `xacro`, `rviz2`, `ros2controlcli`, `ros2launch`, `launch`, `launch_ros`.

### 2.4 `CMakeLists.txt` essentials **[V]**

`example_7` (the "official" style, uses `ros2_control_cmake`) **[V]**:
```cmake
cmake_minimum_required(VERSION 3.16)
project(my_robot_hardware LANGUAGES CXX)
find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()
find_package(controller_manager 4.0.0)   # enforces ros2_control 4.x == Jazzy
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(my_robot_hardware SHARED src/my_robot_hardware.cpp)
target_compile_features(my_robot_hardware PUBLIC cxx_std_17)
target_include_directories(my_robot_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/my_robot_hardware>)
target_link_libraries(my_robot_hardware PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib)

pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware.xml)
```
Minimal style (Trossen) **[V]**: same but no `ros2_control_cmake`, plus
`ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)` and
`ament_export_dependencies(${ROS_DEPENDENCIES})`.

`pluginlib_export_plugin_description_file(hardware_interface <file>.xml)` is the load-bearing line — it's what makes `pluginlib` find your plugin. **[V]**

### 2.5 pluginlib XML export **[V]**

File `my_robot_hardware.xml` at package root (naming convention: same as project):
```xml
<library path="my_robot_hardware">
  <class name="my_robot_hardware/MyRobotSystemHardware"
         type="my_robot_hardware::MyRobotSystemHardware"
         base_class_type="hardware_interface::SystemInterface">
    <description>ros2_control SystemInterface for the MyRobot 7-DOF arm.</description>
  </class>
</library>
```
- `name` = the plugin name you put in `<plugin>` in the URDF. Convention `<package>/<Class>`.
- `type` = fully-qualified C++ class.
- Matching macro at the bottom of the `.cpp` (outside the namespace):
  ```cpp
  #include <pluginlib/class_list_macros.hpp>
  PLUGINLIB_EXPORT_CLASS(my_robot_hardware::MyRobotSystemHardware, hardware_interface::SystemInterface)
  ```
  **[V]**

### 2.6 `<ros2_control>` URDF tag **[V]**

```xml
<ros2_control name="MyRobotSystem" type="system">
  <hardware>
    <plugin>my_robot_hardware/MyRobotSystemHardware</plugin>
    <param name="can_interface">can0</param>
    <param name="joint_limit_tolerance">0.05</param>
    <param name="kp">80.0</param>
    <param name="kd">2.0</param>
  </hardware>
  <joint name="joint_1">
    <command_interface name="position">
      <param name="min">-2.9</param>
      <param name="max">2.9</param>
    </command_interface>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <state_interface name="position">
      <param name="initial_value">0.0</param>
    </state_interface>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  <!-- ... joint_2 .. joint_7 ... -->
  <sensor name="tcp_fts_sensor">
    <state_interface name="force.x"/> ...
  </sensor>
</ros2_control>
```
Key facts **[V]**:
- `type` ∈ `system` | `actuator` | `sensor`.
- `<param>` values are **strings** in `info_.hardware_parameters` — you `std::stod`/`std::stoi` them yourself (see UR's `stoi(info_.hardware_parameters["servoj_gain"])`).
- Multiple `<command_interface>` per joint is exactly `example_3` (`RRBotSystemMultiInterface`) — that's your MIT impedance case.
- `min`/`max` are optional per interface; Kinova uses them to clamp joint limits.
- Optional `is_async="true"` + `<properties><async .../></properties>` on the `<ros2_control>` tag **[V]** — see §4.

---

## 3. Launch wiring: `controller_manager` + broadcasters + spawners + `robot_state_publisher`

### 3.1 Canonical MoveIt 2 helper (this is what MSA-generated packages call) **[V]**

`moveit_configs_utils/moveit_configs_utils/launches.py` (in the local moveit2 checkout):

- `generate_spawn_controllers_launch(moveit_config, controller_manager_timeout=None)` **[V]**:
  ```python
  controller_names = moveit_config.trajectory_execution.get(
      "moveit_simple_controller_manager", {}).get("controller_names", [])
  ros2_controllers_path = moveit_config.package_path / "config/ros2_controllers.yaml"
  spawner_args = (["--param-file", str(ros2_controllers_path)]
                  if ros2_controllers_path.exists() else [])
  if controller_manager_timeout is not None:
      spawner_args += ["--controller-manager-timeout", str(controller_manager_timeout)]
  for controller in controller_names + ["joint_state_broadcaster"]:
      ld.add_action(Node(package="controller_manager", executable="spawner",
                         arguments=[controller] + spawner_args, output="screen"))
  ```
  → **`joint_state_broadcaster` is spawned last and automatically**; every name in `moveit_controllers.yaml → controller_names` is spawned first.

- `generate_demo_launch(moveit_config, launch_package_path=None)` **[V]** starts, in order:
  1. `launch/static_virtual_joint_tfs.launch.py` (virtual joints from SRDF)
  2. `launch/rsp.launch.py` → `robot_state_publisher` with `moveit_config.robot_description` + `publish_frequency` (default `15.0`)
  3. `launch/move_group.launch.py` → `moveit_ros_move_group/move_group`
  4. `launch/moveit_rviz.launch.py` (if `use_rviz`)
  5. `launch/warehouse_db.launch.py` (if `db`)
  6. the **controller manager**:
     ```python
     Node(package="controller_manager", executable="ros2_control_node",
          parameters=[str(moveit_config.package_path / "config/ros2_controllers.yaml")],
          remappings=[("/controller_manager/robot_description", "/robot_description")])
     ```
  7. `launch/spawn_controllers.launch.py`

  Note the remap: on Jazzy `ros2_control_node` reads the URDF from its **private** `~/robot_description`; Trossen instead remaps `('~/robot_description', '/robot_description')`. Both spellings appear in the wild **[V]**.

### 3.2 Hand-rolled variant (Trossen, `trossen_arm_bringup/launch/trossen_arm.launch.py`) **[V]**

```python
controller_manager_node = Node(
    package='controller_manager', executable='ros2_control_node',
    parameters=[ParameterFile(param_file=<pkg>/config/controllers.yaml, allow_substs=True)],
    remappings=[('~/robot_description', '/robot_description')],
    output={'both': 'screen'})

controller_spawner_nodes = [
    Node(name=f'{c}_spawner', package='controller_manager', executable='spawner',
         arguments=[c], output={'both': 'screen'})
    for c in ['arm_controller', 'gripper_controller', 'joint_state_broadcaster']]

return [controller_manager_node,
        description_launch_include,           # robot_state_publisher + RViz
        RegisterEventHandler(OnProcessStart(target_action=controller_manager_node,
                                            on_start=controller_spawner_nodes))]
```
**Spawner ordering matters**: `RegisterEventHandler(OnProcessStart(...))` (or `TimerAction`) is the standard way to avoid the "controller manager not available" race. On Jazzy, `spawner` also accepts `--controller-manager-timeout <sec>` and `--param-file <yaml>`.

### 3.3 The two YAML schemas — keep them separate **[V]**

**(a) `config/ros2_controllers.yaml`** — consumed by `controller_manager` (`--param-file` or node parameters). Structure (exactly what MoveIt Setup Assistant emits — see `moveit_setup_assistant/moveit_setup_controllers/src/ros2_controllers_config.cpp`):
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100          # Hz   <-- MSA default
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

joint_trajectory_controller:
  ros__parameters:
    joints: [joint_1, ..., joint_7]
    command_interfaces: [position]          # MSA also emits velocity if you add it
    state_interfaces: [position, velocity]
    allow_nonzero_velocity_at_trajectory_end: true   # MSA emits true
```
Real-world overrides **[V]**: Kinova gen3 7dof uses `update_rate: 1000`; xArm7 uses `150`; Trossen uses `100`; Piper uses `200`. Trossen adds `state_publish_rate: 50.0`, `action_monitor_rate: 20.0`, `allow_partial_joints_goal: false`, `allow_nonzero_velocity_at_trajectory_end: false`, `constraints: {stopped_velocity_tolerance: 0.02, goal_time: 2.0}`.

> ⚠️ `update_rate` is the **hardware/CM loop rate**, not the trajectory sampling rate. For a 7-DOF MIT-impedance arm over SocketCAN, this is the number that must fit inside your CAN round-trip budget.

**(b) `config/moveit_controllers.yaml`** — consumed by MoveIt's `moveit_simple_controller_manager`. Different schema **[V]**:
```yaml
trajectory_execution:
  allowed_execution_duration_scaling: 1.2
  allowed_goal_duration_margin: 0.5
  allowed_start_tolerance: 0.01

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller
    - gripper_controller
  arm_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints: [joint_1, ..., joint_7]
  gripper_controller:
    action_ns: gripper_cmd
    type: ParallelGripperCommand
    default: true
    max_effort: 100.0
    joints: [left_carriage_joint]
```
`controller_names` is what `generate_spawn_controllers_launch` iterates over — so **this file drives the spawner list**.
> ⚠️ Alert: MoveIt's `type: FollowJointTrajectory` (an action type name) is unrelated to ros2_control's `type: joint_trajectory_controller/JointTrajectoryController` (a pluginlib class). They must both be present, in the two different files.

### 3.4 Minimal package skeleton to copy

`agilexrobotics/piper_ros@humble:src/piper_moveit/piper_no_gripper_moveit/` is a **literal, unmodified MSA output** (it even has `.setup_assistant`) **[V]**:
```
config/{initial_positions.yaml, joint_limits.yaml, kinematics.yaml, moveit.rviz,
        moveit_controllers.yaml, pilz_cartesian_limits.yaml, piper.ros2_control.xacro,
        piper.srdf, piper.urdf.xacro, ros2_controllers.yaml}
launch/{demo.launch.py, move_group.launch.py, moveit_rviz.launch.py, piper_moveit.launch.py,
        rsp.launch.py, setup_assistant.launch.py, spawn_controllers.launch.py,
        static_virtual_joint_tfs.launch.py, warehouse_db.launch.py}
CMakeLists.txt  package.xml  .setup_assistant
```
and each launch file is a 3-liner **[V]**:
```python
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch
def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_no_gripper_moveit").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
```
**Recommendation:** generate your MoveIt config package with MoveIt Setup Assistant, then only edit `config/<robot>.ros2_control.xacro` (swap `mock_components/GenericSystem` for your plugin) and `config/ros2_controllers.yaml`. That gets you the whole standard launch tree for free.

---

## 4. Pitfalls when the hardware interface wraps a blocking SDK call

### 4.1 The contract, as written

- **`read()`**: "The data readings from the physical hardware has to be updated and reflected accordingly in the exported state interfaces. That is, the data pointed by the interfaces shall be updated. The method is called in the resource_manager's read loop." **[V]** (`hardware_component_interface.hpp`)
- **`write()`**: "The physical hardware shall be updated with the latest value from the exported command interfaces." **[V]**
- **`perform_command_mode_switch`**: "**This is part of the realtime update loop, and should be fast.**" **[V]** (`hardware_component_interface.hpp`) — this is the only place the upstream headers say "realtime" explicitly, but it is the same loop as `read()`/`write()`.
- `trigger_read()` / `trigger_write()` wrap `read()`/`write()`: "Triggers the read/write method synchronously or **asynchronously depending on the HardwareInfo**". If async, `HardwareComponentCycleStatus { bool successful; return_type result; std::optional<std::chrono::nanoseconds> execution_time; }` is what CM gets back. **[V]**
- **Lifecycle states** — <https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/lifecycle_of_a_hardware_component.html> **[V]**:
  - UNCONFIGURED (`on_init`, `on_cleanup`): "Hardware is only initialized, but communication is not started"
  - INACTIVE (`on_configure`, `on_deactivate`): "Communication with the hardware is established... States can be read, but command interfaces (System and Actuator only) are not available." ⚠️ with the caveat: "As of now, it is left to the hardware component implementation to continue using the command received from the `CommandInterfaces` or to skip them completely." (issue [#931](https://github.com/ros-controls/ros2_control/issues/931))
  - ACTIVE (`on_activate`): "Power circuits of hardware are active... Command interfaces are available and the commands should be sent to the hardware"
  - FINALIZED (`on_shutdown`)
- **Errors** — <https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/handling_errors_during_read_write.html> **[V]**: returning `return_type::ERROR` from `read()`/`write()` triggers `on_error(previous_state)`. "If successful `CallbackReturn::SUCCESS` is returned and hardware is again in `UNCONFIGURED` state, if any `ERROR` or `FAILURE` happens the hardware ends in `FINALIZED` state and **can not be recovered. The only option is to reload the complete plugin, but there is currently no service for this in the Controller Manager.**" → **do not return ERROR for transient bus hiccups**; log, count, and return `OK` (this is exactly what UR does: `// TODO(anyone): could not read from the driver --> return ERROR --> on error will be called` / `return hardware_interface::return_type::OK;`) **[V]**.

### 4.2 Why blocking in `write()` is the specific failure mode

`controller_manager`'s update loop is `read() → (controllers update) → write()` at `update_rate` Hz. **`write()` blocking stalls the entire loop for every other component too** — all controllers stop being updated, `joint_state_broadcaster` stops publishing, and the JTC's action server stops servicing goals. If your SDK's `write` is a request/response round trip (UR's RTDE, Kinova's `RefreshFeedback`, a Python `socket`/CAN ack), you turn a 1 kHz loop into a whatever-your-latency-is loop. **[I]** (mechanism is inferred from the documented loop contract; the individual vendor snippets below are **[V]**.)

### 4.3 What real vendors actually do — three patterns, in increasing order of correctness

**Pattern A — "non-blocking read" flag + receive timeout (UR).** **[V]**
`ur_robot_driver/src/hardware_interface.cpp`:
```cpp
non_blocking_read_ = (info_.hardware_parameters["non_blocking_read"] == "true") || ...;
...
driver_config.non_blocking_read = non_blocking_read_;
ur_driver_->startRTDECommunication(non_blocking_read_);
...
if (!non_blocking_read_)
  RCLCPP_ERROR(..., "Unable to read from hardware...");
// TODO(anyone): could not read from the driver --> return ERROR --> on error will be called
return hardware_interface::return_type::OK;      // <-- never fails the loop
...
// write():
if (... && robot_program_running_ && (!non_blocking_read_ || packet_read_)) {
  ur_driver_->writeJointCommand(urcl_position_commands_, urcl::comm::ControlMode::MODE_SERVOJ, receive_timeout_);
```
i.e. `read()` is allowed to *miss* — it just doesn't update states that cycle, and `write()` is gated on `packet_read_` so commands are only sent when fresh state exists. `receive_timeout_` (`RobotReceiveTimeout`) bounds the socket wait.

**Pattern B — dedicated async thread for everything non-RT (UR).** **[V]**
`ur_robot_driver/include/ur_robot_driver/hardware_interface.hpp` declares `void asyncThread();`; `.cpp`:
```cpp
async_thread_ = std::make_shared<std::thread>(&URPositionHardwareInterface::asyncThread, this);
...
void URPositionHardwareInterface::asyncThread() {
  async_thread_shutdown_ = false;
  while (!async_thread_shutdown_) {
    if (initialized_) { checkAsyncIO(); }
    std::this_thread::sleep_for(std::chrono::nanoseconds(20000000));  // 20 ms
  }
}
```
All GPIO / tool / status writes go through `checkAsyncIO()`, **never** through `write()`. Thread is joined in `on_deactivate`/`on_cleanup`. This is the pattern to copy for "I have a blocking vendor call".

**Pattern C — the framework's own async hardware component (Franka + upstream Jazzy).** **[V]**
URDF:
```xml
<ros2_control name="..." type="system" is_async="true">
  <properties>
    <async affinity="[2,4]" scheduling_policy="detached" print_warnings="true" thread_priority="30"/>
  </properties>
```
Docs <https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/asynchronous_components.html> **[V]**:
- `is_async` (default false) — run the component in its own thread.
- `properties/async` attrs: `thread_priority` (0–99, default 50), `affinity` (list of CPU cores), `scheduling_policy` = `synchronized` (default; CM triggers read/write) | `detached` (component manages its own timing), `print_warnings` (default true).
- "The thread priority is only used when the hardware component is run asynchronously. When the hardware component is run asynchronously, it uses the FIFO scheduling policy."
- Rationale given: "a sensor takes longer to read data that **affects the periodicity of the controller_manager control loop**."
- Implemented in `/opt/ros/jazzy/include/hardware_interface/hardware_interface/async_components.hpp`, class `AsyncComponentThread`, with the caveat in its own doc-comment: **"Not synchronized with the controller manager's update currently"** **[V]**.
- Franka franka_ros2 is a real, shipped user of this: `franka_hardware/ros2_control/franka_arm.ros2_control.xacro` with `is_async="${is_async}"` / `thread_priority:=98` / `scheduling_policy="detached"` **[V]**.
- ⚠️ Consequence: with `detached`, `read()`/`write()` are **not called from the CM loop**, so your command/state memory is concurrently accessed by two threads — you must use `realtime_tools::RealtimeBuffer` / `realtime_thread_safe_box` / `realtime_tools::mutex` (exactly what Franka does: `#include <realtime_tools/mutex.hpp>`, `#include <realtime_tools/realtime_thread_safe_box.hpp>`) **[V]**.

**Anti-pattern — blocking network I/O inside `write()` (Kinova).** **[V]**
`kortex_driver/src/hardware_interface.cpp`:
```cpp
return_type KortexMultiInterfaceHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (block_write)
  {
    feedback_ = base_cyclic_.RefreshFeedback();   // blocking round trip in the RT loop
    return return_type::OK;
  }
  if (!std::isnan(reset_fault_cmd_) && fault_controller_running_) {
      base_.SetServoingMode(servoing_mode_hw_);
      base_.ApplyEmergencyStop(0, {false, 0, 100});
      base_.ApplyEmergencyStop(0, {false, 0, 100});   // "calling it once appeared to be unreliable"
      base_.ClearFaults();
  ...
```
It ships and it works, but it is the thing to avoid.

**Pattern D — vendor SDK does its own RT thread; `read()`/`write()` are pure cache copies (Trossen).** **[V]**
```cpp
return_type TrossenArmHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &) {
  robot_output_ = arm_driver_->get_robot_output();     // read from SDK's own buffer
  joint_positions_  = robot_output_.joint.all.positions;
  joint_velocities_ = robot_output_.joint.all.velocities;
  joint_efforts_    = robot_output_.joint.all.efforts;
  return return_type::OK;
}
return_type TrossenArmHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {
  if (first_update_) { joint_position_commands_ = joint_positions_; first_update_ = false; }
  if (arm_position_mode_running_) arm_driver_->set_all_positions(joint_position_commands_, 0.0, false);
  ...
}
```
No mutex, no thread **in the hardware interface** — the C++ SDK (`libtrossen_arm`, separate repo, separate thread) owns the real-time exchange. **If you can push the CAN I/O into a separate process/library that maintains its own state cache at its own rate, do that — it's the cleanest possible `SystemInterface`.**

### 4.4 Practical checklist for a SocketCAN + MIT-impedance + Python-SDK robot

1. **Do not call Python from `read()`/`write()` in the CM loop.** Either (a) run the Python SDK in a separate non-RT process and bridge over a lock-free IPC/topic (the `topic_based_ros2_control/TopicBasedSystem` plugin does exactly the topic variant — Kinova uses it for Isaac: `<plugin>topic_based_ros2_control/TopicBasedSystem</plugin>` with `joint_commands_topic`/`joint_states_topic` **[V]**), or (b) reimplement the CAN framing in C++ inside the `SystemInterface`. `pybind11` + GIL contention inside a 1 kHz loop is a trap. **[I]**
2. **Own the CAN socket in a dedicated non-RT thread** (Pattern B) pinned with `properties/async` (Pattern C), publishing into a `realtime_tools::RealtimeBuffer` that `read()` copies out of and `write()` copies into. **[I]**, patterned on **[V]** UR/Franka.
3. **MIT impedance = 5 command interfaces per joint** (`position`, `velocity`, `effort` + `kp`/`kd` as `<gpio>` or unlisted interfaces). Declare them all in the `<ros2_control>` tag. `ros2_control_demos/example_3` (`RRBotSystemMultiInterface`) is the reference for multi-interface joints, and `franka_hardware_interface.hpp` shows the mode-switch enum + `prepare/perform_command_mode_switch` that gates which set is live. **[V]** for the mechanisms; mapping kp/kd onto `<gpio>`/unlisted interfaces is **[I]**.
4. **Never return `ERROR` from `read()` for a dropped CAN frame** — it is unrecoverable without reloading the plugin. Count failures, extrapolate/hold state, return `OK`. Use `return_type::DEACTIVATE` if you must stop. **[V]** (doc quote + `return_type` enum)
5. **`update_rate` must be ≥ your JTC's expectation but ≤ 1/CAN-round-trip.** Kinova ships 1000 Hz with a proprietary UDP protocol; xArm 150 Hz; Trossen 100 Hz. For SocketCAN at 1 Mbit/s with 7 joints × MIT frame (~8 bytes payload + overhead) expect a realistic few hundred Hz ceiling per bus. **[I]** — the vendor numbers are **[V]**.
6. **Don't forget the fake-hardware switch** (`<plugin>mock_components/GenericSystem</plugin>` behind `use_fake_hardware`) — every vendor above has it, and it lets you develop the MoveIt config with zero hardware. **[V]**
7. **Remap the robot description**: Jazzy's `ros2_control_node` reads `~/robot_description`; use `remappings=[('~/robot_description', '/robot_description')]` (Trossen) or `('/controller_manager/robot_description', '/robot_description')` (MoveIt's `generate_demo_launch`). **[V]**

---

## Appendix: URLs used

ros2_control docs (Jazzy):
- https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html
- https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/lifecycle_of_a_hardware_component.html
- https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/handling_errors_during_read_write.html
- https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/asynchronous_components.html

Repos:
- https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver (branch `humble`)
- https://github.com/UniversalRobots/Universal_Robots_ROS2_Description (branch `humble`)
- https://github.com/frankarobotics/franka_ros2 (branch `jazzy`)
- https://github.com/Kinovarobotics/ros2_kortex (branch `humble`)
- https://github.com/xArm-Developer/xarm_ros2 (branch `humble`)
- https://github.com/TrossenRobotics/trossen_arm_ros (branch `jazzy`)
- https://github.com/TrossenRobotics/trossen_arm (branch `main`)
- https://github.com/agilexrobotics/piper_ros (branch `humble`)
- https://github.com/agilexrobotics/piper_sdk
- https://github.com/ros-controls/ros2_control_demos (branch `jazzy`)
- https://github.com/Annin-Robotics/ar4-hmi
- https://github.com/ycheng517/ar4_ros_driver_examples
- https://github.com/SimonKennethRobo/arx_ros2

Local (verified on this machine): `/opt/ros/jazzy/include/hardware_interface/**`,
`/home/initialxx/project/moveit2/moveit_configs_utils/moveit_configs_utils/launches.py`,
`/home/initialxx/project/moveit2/moveit_setup_assistant/moveit_setup_controllers/{src/ros2_controllers_config.cpp,templates/config/ros2_control.xacro}`.
