# 车载 ROS1 与无人机 ROS2 夹爪接口

车载工控机与无人机通过电台网络运行 `ros1_bridge`，夹爪控制不再使用 UDP 字节协议。

## 话题接口

| 话题 | 方向 | 类型 | 定义 |
|---|---|---|---|
| `/clamp_command` | 无人机 ROS2 -> 车载 ROS1 | `std_msgs/UInt8` -> `std_msgs/msg/UInt8` | `0` 松开，`1` 夹紧 |
| `/clamp_state` | 车载 ROS1 -> 无人机 ROS2 | `std_msgs/UInt8` -> `std_msgs/msg/UInt8` | `0` 已解锁，`1` 锁定中，`2` 已锁定，`3` 解锁中，`128` 未知/故障 |

消息只使用 `data` 字段。ROS1 侧节点仍通过 TCP 连接 STM32（`192.168.8.103:5000`），该 TCP 链路和 STM32 闭环逻辑不变。

## 通信要求

1. 两端电台网络必须互通，桥接节点同时连接 ROS1 Master 和 ROS2 DDS 网络。
2. `ros1_bridge` 使用标准消息自动桥接：`std_msgs/UInt8` 对应 `std_msgs/msg/UInt8`。
3. 无人机发送命令后，以 `/clamp_state` 的完成态确认动作：夹紧等待 `2`，松开等待 `0`。
4. 收到 `128` 或超过约 1 秒没有状态更新时，按通信/夹爪状态不可确认处理，不得认为动作完成。
5. 无人机启动或重连后应重新发布当前任务命令；不要依赖旧的锁存命令触发动作。

## ROS2 示例

```bash
ros2 topic pub --once /clamp_command std_msgs/msg/UInt8 "{data: 1}"  # 夹紧
ros2 topic pub --once /clamp_command std_msgs/msg/UInt8 "{data: 0}"  # 松开
ros2 topic echo /clamp_state
```

ROS1 侧等效测试：

```bash
rostopic pub --once /clamp_command std_msgs/UInt8 "data: 1"
rostopic echo /clamp_state
```
