# Onero A1 Common

可由 colcon 安装的公共 Python 模块，供 examples、应用程序及诊断工具复用。

- `constants.JOINTS`：A1R 七关节名称。
- `moveit_client.MoveItClient`：状态读取、FK、碰撞查询、参数读取、规划、显示、执行及取消/场景清理。
- `moveit_client.MoveItError`：服务和 Action 的基本错误。

客户端通过标准 ROS 接口访问 MoveIt。
`tools/diagnostics/client.py` 通过继承客户端实现报告记录和严格校验。

`MoveItClient.plan()` 从参数对象的 `pipeline`、`planner` 字段选择规划器；
支持情况见 [规划器选择](../onero_a1_examples/README.md#规划器选择)，
可选值和默认值见 [示例参数](../onero_a1_examples/README.md#参数)。

```python
from onero_a1_common.moveit_client import MoveItClient
```

可运行的完整用法见 `onero_a1_examples`。
