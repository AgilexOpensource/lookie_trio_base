# lookie_trio_base

[English](README.md)

Lookie Trio 全向底盘的 ROS 2 驱动，基于 [ugv_sdk](https://github.com/westonrobot/ugv_sdk)（`ScoutMiniOmniRobot`，AGX_V2 协议，三个驱动电机共用一个驱动器）。

接口沿用 ROS 底盘驱动的常见约定，只使用标准消息：输入 `cmd_vel`；输出 `odom`、`odom` → `base_link` TF、`battery_state`、以 `sensor_msgs/Joy` 表示的遥控器，以及 `/diagnostics`。

> **安全提示：** 遥控器处于指令（CAN）模式时，底盘会执行收到的每一条 `cmd_vel`。首次运行请架空车轮或留出足够空间，并把遥控器或急停放在手边。

## 环境要求

已在 Ubuntu 22.04 + ROS 2 Humble + ugv_sdk 0.10.0 上测试。

## 安装

```bash
sudo apt install libasio-dev python3-colcon-common-extensions python3-rosdep
mkdir -p ~/lookie_ws/src && cd ~/lookie_ws
git clone https://github.com/AgilexOpensource/lookie_trio_base.git src/lookie_trio_base
git clone https://github.com/westonrobot/ugv_sdk.git src/ugv_sdk
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to lookie_trio_base
source install/setup.bash
```

## 快速开始

以 500 kbit/s 启动底盘 CAN 接口，接口名须与 `port_name` 一致（默认 `can_chassis`），然后启动驱动：

```bash
sudo ip link set can_chassis up type can bitrate 500000
ros2 launch lookie_trio_base lookie_trio_base.launch.py
```

节点启动时打开 `port_name` 并使能指令模式。若 `connect_retries` 次尝试仍无法打开总线，则报错并以退出码 1 退出；底盘无应答的情况通过反馈诊断上报。

在另一个终端查看反馈并发送一段短指令。**指令会让底盘运动。**

```bash
ros2 topic echo --once /battery_state
ros2 topic pub --times 20 --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
ros2 topic echo --once /odom
```

任何 `cmd_vel` 来源都可使用，例如 `ros2 run teleop_twist_keyboard teleop_twist_keyboard`。

## 接口

节点名 `lookie_trio_base`。话题名相对节点命名空间，默认命名空间为空，即 `/cmd_vel`、`/odom` 等。`/diagnostics` 按诊断的惯例使用绝对名。

| 名称 | 类型 | 消息 | 说明 |
|---|---|---|---|
| `cmd_vel` | 订阅话题 | `geometry_msgs/msg/Twist` | 速度指令，使用 `linear.x`、`linear.y` 和 `angular.z` |
| `odom` | 发布话题 | `nav_msgs/msg/Odometry` | 积分位姿和本体速度，`odom_frame` → `base_frame`，50 Hz |
| `odom_frame` → `base_frame` | TF | | 与 `odom` 位姿相同；仅在 `publish_odom_tf` 为 true 时发布 |
| `battery_state` | 发布话题 | `sensor_msgs/msg/BatteryState` | 电池电压，1 Hz；其他字段未知（NaN） |
| `rc/joy` | 发布话题 | `sensor_msgs/msg/Joy` | 遥控器，50 Hz，见[遥控器](#遥控器) |
| `/diagnostics` | 发布话题 | `diagnostic_msgs/msg/DiagnosticArray` | 底盘和电机状态，1 Hz，见[诊断](#诊断) |

`odom`、`battery_state` 和 `rc/joy` 仅在 CAN 反馈新鲜时发布。

### 遥控器

| 字段 | 内容 |
|---|---|
| `axes` | 共 9 项，-1.0 到 1.0：左摇杆水平、左摇杆垂直、右摇杆水平、右摇杆垂直、旋钮 VRA，然后是开关 SWA、SWB、SWC、SWD，分别为上 1.0、中 0.0、下 -1.0 |
| `buttons` | 空 |

### 诊断

状态名以节点全名开头，例如 `/lookie_trio_base: base`，同一图中的两个节点在 `/diagnostics` 上不会冲突。

| 状态 | 级别 | 键值 |
|---|---|---|
| `base` | OK；底盘不在 CAN 指令模式、会忽略 `cmd_vel` 时为 WARN；无 CAN 反馈时为 STALE | `control_mode`、`vehicle_state`、`error_code`、`battery_voltage`（V） |
| `motor 1` … `motor 3` | OK；无 CAN 反馈时为 STALE | `rpm`、`current`（A）、`pulse_count`、`driver_voltage`（V）、`driver_temperature`（°C）、`motor_temperature`（°C）、`driver_state` |

`vehicle_state`、`error_code` 和 `driver_state` 是底盘固件上报的原始代码。

## 参数

默认值见 [config/lookie_trio_base.yaml](config/lookie_trio_base.yaml)。所有参数只读，启动时设定。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `port_name` | `can_chassis` | 底盘 CAN 接口 |
| `connect_retries` | `5` | 连接底盘的尝试次数，每秒一次，用完仍失败则退出 |
| `odom_frame` | `odom` | 里程计坐标系 |
| `base_frame` | `base_link` | 底盘坐标系 |
| `publish_odom_tf` | `true` | 广播 `odom_frame` → `base_frame` TF；已有其他节点发布时设为 `false` |
| `cmd_velocity_signs` | `[1, -1, 1]` | `[x, y, wz]` 各轴符号（+1 或 -1），`cmd_vel` 下发前相乘 |
| `odom_velocity_signs` | `[-1, 1, 1]` | `[x, y, wz]` 各轴符号（+1 或 -1），与底盘反馈相乘，作用于 `odom` 和 TF |
| `feedback_timeout` | `0.5` | 超过该秒数未收到 CAN 运动状态则暂停里程计，须大于 0 |

### launch 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `namespace` | 空 | 节点命名空间 |
| `params_file` | `config/lookie_trio_base.yaml` | 参数文件 |
| `port_name`、`odom_frame`、`base_frame`、`publish_odom_tf` | 空 | 覆盖同名参数；为空时使用 `params_file` 中的值 |

### 速度符号

两组符号把底盘映射到 ROS 约定：前进 `x`、向左 `y`、逆时针 `wz` 为正。默认值是 Lookie Trio 上的实测值。Trio 上正的 `linear.y` 让底盘向右走，前进反馈方向也相反。

校验符号时，逐个发送前进、向左平移和左转指令：

1. 某个轴运动方向反了，翻转 `cmd_velocity_signs` 对应项。
2. 方向正确后，`odom` 中对应速度应为正；否则翻转 `odom_velocity_signs` 对应项。

## 里程计 TF

驱动默认广播 `odom_frame` → `base_frame`。如果已有其他节点（例如激光或视觉里程计）向同一坐标系广播变换，关闭本驱动的 TF，保证 TF 树中每个坐标系只有一个父节点；`odom` 话题照常发布：

```bash
ros2 launch lookie_trio_base lookie_trio_base.launch.py publish_odom_tf:=false
```

## 命名空间和坐标系

话题名跟随 launch 参数 `namespace`，坐标系名则不会，因为 TF 是一棵全局的树。同一网络中有多个驱动时，两者都要设置：

```bash
ros2 launch lookie_trio_base lookie_trio_base.launch.py namespace:=base1 odom_frame:=base1/odom base_frame:=base1/base_link
```

此时话题为 `/base1/cmd_vel`、`/base1/odom` 等。若要彼此完全不可见，改用不同的 `ROS_DOMAIN_ID`。

## 行为

- 里程计以 50 Hz 积分前进、横向速度（按航向旋转）和角速度，启动时从零开始，并带固定的对角协方差（平面位姿和速度为 0.01，未知轴为 1e6），便于融合工具使用。
- `rc/joy` 按 `joy` 的惯例排布：摇杆和旋钮在前，三档开关放在 `axes` 而不是 `buttons`，因为 `buttons` 约定只表示 0/1 按键。
- 连接底盘失败时按 `connect_retries` 每秒重试一次，开机时 CAN 起得慢不必手动重启节点。
- 每次积分步长上限 0.2 s，航向归一到 [-pi, pi]，定时器卡顿不会造成位姿跳变，长时间运行也不会损失精度。
- 退出时下发零速度，底盘不会保持最后一条指令。
- 超过 `feedback_timeout` 未收到 CAN 运动状态时，里程计和 TF 暂停，不用缓存的旧速度积分，并打印警告；反馈恢复后继续并打印日志。期间诊断状态为 STALE。
- 电池电压单位为 V（SDK 的值在本底盘上需乘 0.1）。
- 三个电机共用一个驱动器，只发一帧低速状态：各 `motor` 诊断中的驱动器数值都相同。

## 故障排查

- `cannot open CAN interface`：按[快速开始](#快速开始)启动接口，并用 `ip -brief link show` 检查。
- `no ... base found`：底盘未上电、不在该接口或波特率不对；用 `candump can_chassis` 确认有数据帧。
- 底盘不动：底盘不在 CAN 指令模式时 `base` 诊断会告警。把遥控器切到指令模式并松开急停。可用 `ros2 topic echo /diagnostics` 或 `rqt_robot_monitor` 查看。
- 运动或转向方向反了，或 `odom` 朝反方向变化：见[速度符号](#速度符号)。
- RViz 中机器人在两个位姿间跳动，或 TF 报某坐标系有两个父节点：另有节点广播了同一里程计 TF，见[里程计 TF](#里程计-tf)。
- `base feedback lost ... odometry paused`：底盘 CAN 帧中断，检查线缆和供电。

## 测试

```bash
colcon test --packages-select lookie_trio_base && colcon test-result --verbose
```

## 许可证

Apache-2.0，见 [LICENSE](LICENSE)。
