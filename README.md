# lookie_trio_base

[中文](README.zh-CN.md)

ROS 2 driver for the Lookie Trio omnidirectional base, built on [ugv_sdk](https://github.com/westonrobot/ugv_sdk) (`ScoutMiniOmniRobot`, AGX_V2 protocol, three drive motors on one driver).

It follows the usual ROS base driver layout and uses standard messages only: `cmd_vel` in; `odom`, the `odom` → `base_link` TF, `battery_state`, the remote controller as `sensor_msgs/Joy` and `/diagnostics` out.

> **Safety:** The base moves on every `cmd_vel` it receives while the remote controller is in command (CAN) mode. Lift the wheels off the ground or keep clear space for the first run, and keep the remote controller or an emergency stop at hand.

## Requirements

Tested on Ubuntu 22.04 with ROS 2 Humble and ugv_sdk 0.10.0.

## Install

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

## Quick start

Bring up the CAN interface of the base at 500 kbit/s; it must be the interface named by `port_name`, `can_chassis` by default. Then start the driver:

```bash
sudo ip link set can_chassis up type can bitrate 500000
ros2 launch lookie_trio_base lookie_trio_base.launch.py
```

At startup the node opens `port_name` and enables commanded mode. If the bus cannot be opened in `connect_retries` attempts it logs an error and exits with code 1; a base that stays silent is reported through the feedback diagnostics instead.

In another terminal, check the feedback and send a short command. **Commands move the base.**

```bash
ros2 topic echo --once /battery_state
ros2 topic pub --times 20 --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
ros2 topic echo --once /odom
```

Any `cmd_vel` source works, for example `ros2 run teleop_twist_keyboard teleop_twist_keyboard`.

## Interfaces

The node is `lookie_trio_base`. Topic names are relative to its namespace; with the default empty namespace they are `/cmd_vel`, `/odom` and so on. `/diagnostics` is absolute, as is usual for diagnostics.

| Name | Kind | Type | Description |
|---|---|---|---|
| `cmd_vel` | Subscribed topic | `geometry_msgs/msg/Twist` | Velocity command; uses `linear.x`, `linear.y` and `angular.z` |
| `odom` | Published topic | `nav_msgs/msg/Odometry` | Integrated pose and body velocity, `odom_frame` → `base_frame`, 50 Hz |
| `odom_frame` → `base_frame` | TF | | Same pose as `odom`; only with `publish_odom_tf` |
| `battery_state` | Published topic | `sensor_msgs/msg/BatteryState` | Battery voltage, 1 Hz; the other fields are unknown (NaN) |
| `rc/joy` | Published topic | `sensor_msgs/msg/Joy` | Remote controller, 50 Hz; see [Remote controller](#remote-controller) |
| `/diagnostics` | Published topic | `diagnostic_msgs/msg/DiagnosticArray` | Base and motor state, 1 Hz; see [Diagnostics](#diagnostics) |

`odom`, `battery_state` and `rc/joy` are published only while CAN feedback is fresh.

### Remote controller

| Field | Content |
|---|---|
| `axes` | 9 entries, -1.0 to 1.0: left stick horizontal, left stick vertical, right stick horizontal, right stick vertical, knob VRA, then switches SWA, SWB, SWC, SWD as 1.0 up, 0.0 middle, -1.0 down |
| `buttons` | empty |

### Diagnostics

Status names start with the node's full name, for example `/lookie_trio_base: base`, so two nodes in the same graph never collide on `/diagnostics`.

| Status | Level | Values |
|---|---|---|
| `base` | OK; WARN when the base is not in CAN command mode and ignores `cmd_vel`; STALE without CAN feedback | `control_mode`, `vehicle_state`, `error_code`, `battery_voltage` (V) |
| `motor 1` … `motor 3` | OK; STALE without CAN feedback | `rpm`, `current` (A), `pulse_count`, `driver_voltage` (V), `driver_temperature` (°C), `motor_temperature` (°C), `driver_state` |

`vehicle_state`, `error_code` and `driver_state` are the raw codes reported by the base firmware.

## Parameters

[config/lookie_trio_base.yaml](config/lookie_trio_base.yaml) holds the defaults below. All parameters are read-only and set at startup.

| Parameter | Default | Description |
|---|---|---|
| `port_name` | `can_chassis` | CAN interface of the base |
| `connect_retries` | `5` | Attempts to open the CAN bus and connect to the base, one per second, before the node gives up and exits |
| `odom_frame` | `odom` | Odometry frame |
| `base_frame` | `base_link` | Base frame |
| `publish_odom_tf` | `true` | Broadcast the `odom_frame` → `base_frame` TF; set `false` when another node owns it |
| `cmd_velocity_signs` | `[1, -1, 1]` | Signs `[x, y, wz]` (+1 or -1) applied to `cmd_vel` before it is sent |
| `odom_velocity_signs` | `[-1, 1, 1]` | Signs `[x, y, wz]` (+1 or -1) applied to the base feedback; used by `odom` and TF |
| `feedback_timeout` | `0.5` | Seconds without a CAN motion state before odometry pauses; must be greater than 0 |

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `namespace` | empty | Node namespace |
| `params_file` | `config/lookie_trio_base.yaml` | Parameter file |
| `port_name`, `odom_frame`, `base_frame`, `publish_odom_tf` | empty | Override the parameter of the same name; empty keeps the value from `params_file` |

### Velocity signs

Both sign parameters map the base to the ROS convention: forward `x`, left `y` and counter-clockwise `wz` are positive. The defaults are the values measured on Lookie Trio. On Trio positive `linear.y` drove the base to its right, and the forward feedback was reversed.

To verify the signs, send forward, left and left turn commands one at a time:

1. If the base moves the wrong way on an axis, flip that entry of `cmd_velocity_signs`.
2. Then, while it moves the right way, the matching velocity in `odom` must be positive; if not, flip that entry of `odom_velocity_signs`.

## Odometry TF

The driver broadcasts `odom_frame` → `base_frame` by default. If another node already broadcasts a transform into the same frame, for example a lidar or visual odometry, turn it off so every frame keeps one parent in the TF tree; `odom` is still published:

```bash
ros2 launch lookie_trio_base lookie_trio_base.launch.py publish_odom_tf:=false
```

## Namespace and frames

Topic names follow the `namespace` launch argument, but frame names do not, because TF is one global tree. When several drivers share a network, set both:

```bash
ros2 launch lookie_trio_base lookie_trio_base.launch.py namespace:=base1 odom_frame:=base1/odom base_frame:=base1/base_link
```

The topics are then `/base1/cmd_vel`, `/base1/odom` and so on. Use different `ROS_DOMAIN_ID` values to keep the drivers from seeing each other at all.

## Behavior

- Odometry integrates the forward and lateral speed (rotated by the heading) and yaw rate at 50 Hz, starting from zero at startup, with a fixed diagonal covariance (0.01 for the planar pose and velocity, 1e6 for the unknown axes) so that fusion tools can use it.
- `rc/joy` follows the `joy` layout: sticks and knob on the first entries, switches on `axes` rather than `buttons`, because `buttons` is meant for 0/1 push buttons.
- The node retries the CAN connection `connect_retries` times, one second apart, so a slow CAN start-up at boot does not need a manual restart.
- Each integration step is capped at 0.2 s and the heading is wrapped to [-pi, pi], so a stalled timer cannot produce a pose jump and long runs keep their precision.
- On shutdown the node sends a zero velocity, so the base does not keep the last command.
- When no CAN motion state has arrived for `feedback_timeout`, odometry and TF pause instead of integrating the last cached velocity, and the node logs a warning; they resume with a log line once feedback returns. Diagnostics turn STALE meanwhile.
- Battery voltage is in volts (the SDK value is scaled by 0.1 for this base).
- The three motors share one driver, which sends a single low-speed state frame: the driver values are the same in every `motor` diagnostic.

## Troubleshooting

- `cannot open CAN interface`: bring the interface up as in [Quick start](#quick-start) and check `ip -brief link show`.
- `no ... base found`: the base is off, on another interface or at another bitrate; check `candump can_chassis` shows frames.
- The base does not move: the `base` diagnostic warns when the base is not in CAN command mode. Switch the remote controller to command mode and release the emergency stop. `ros2 topic echo /diagnostics` or `rqt_robot_monitor` shows it.
- The base moves or turns the wrong way, or `odom` drifts the opposite way: see [Velocity signs](#velocity-signs).
- RViz shows the robot jumping between two poses, or TF warns about a frame with two parents: another node broadcasts the same odometry TF; see [Odometry TF](#odometry-tf).
- `base feedback lost ... odometry paused`: CAN frames from the base stopped; check the cable and power.

## Test

```bash
colcon test --packages-select lookie_trio_base && colcon test-result --verbose
```

## License

Apache-2.0, see [LICENSE](LICENSE).
