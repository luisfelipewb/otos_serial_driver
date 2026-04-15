# otos_serial_driver

ROS 2 Jazzy serial bridge driver for SparkFun Qwiic OTOS (PAA5160E1 + LSM6DSO) connected through an Arduino.

Hardware topology:

OTOS <-> I2C/Qwiic <-> Arduino UNO R4 WiFi <-> USB Serial <-> PC (ROS 2)

## Features

- Arduino firmware streams OTOS position, velocity, acceleration, and standard deviations at ~50 Hz.
- ROS 2 Python node publishes `nav_msgs/msg/Odometry` on `/odom`.
- Optional `odom -> base_link` TF broadcast.
- Covariance diagonals updated from streamed OTOS standard deviations.
- Services:
  - `/otos/reset` (`std_srvs/srv/Empty`)
  - `/otos/calibrate` (`std_srvs/srv/Empty`)
- ROS-side scalar calibration through parameters:
  - `linear_scalar`
  - `angular_scalar`

## Serial Protocol

Baud: `115200`

Data frame (CSV, one line per sample):

x,y,h,vx,vy,wz,ax,ay,az,sx,sy,sh,svx,svy,swz,sax,say,saz

- Values:
  - `x,y,h`: pose in meters, meters, radians
  - `vx,vy,wz`: velocity in m/s, m/s, rad/s
  - `ax,ay,az`: acceleration in m/s^2, m/s^2, rad/s^2 (`az` maps from OTOS `acc.h`)
  - `sx,sy,sh,...`: standard deviations from OTOS firmware
- Command frames (PC -> Arduino):
  - `R\n` reset tracking
  - `C\n` calibrate IMU + reset tracking
  - `S,<linear_scalar>,<angular_scalar>\n` set scalars + reset tracking
- Status lines are prefixed with `#` and are ignored by CSV parser.

## Firmware Upload

1. Open `firmware/otos_serial_bridge/otos_serial_bridge.ino` in Arduino IDE.
2. Install library `SparkFun Qwiic OTOS Arduino Library`.
3. Select board `Arduino UNO R4 WiFi` and upload.
4. Verify serial output at `115200` has 18 comma-separated numeric fields.

## Build

```bash
cd ~/your_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select otos_serial_driver
source install/setup.bash
```

## Run

```bash
ros2 launch otos_serial_driver otos.launch.py serial_port:=/dev/ttyACM0
```

Tip: prefer stable `/dev/serial/by-id/...` path instead of `/dev/ttyUSB0` or `/dev/ttyACM0`.

## Use Services

```bash
ros2 service call /otos/reset std_srvs/srv/Empty
ros2 service call /otos/calibrate std_srvs/srv/Empty
```

## Set Scalars From ROS

```bash
ros2 param set /otos_serial_node linear_scalar 1.004
ros2 param set /otos_serial_node angular_scalar 0.998
```

Valid range for both scalars is `[0.872, 1.127]`.

## Covariance Behavior

If `use_sensor_stddev_covariance` is true:

- Pose covariance diagonals use: `sx^2`, `sy^2`, `sh^2`
- Twist covariance diagonals use: `svx^2`, `svy^2`, `swz^2`
- Remaining diagonals come from fallback params in `config/otos_params.yaml`

Important: OTOS standard deviations are model-based statistical outputs from firmware and do not guarantee physical tracking error bounds.

## Parameters

See `config/otos_params.yaml` for defaults.
