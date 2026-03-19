# `eskf_cpp_reduced`

Separate ROS2/C++ package for the reduced visual-guidance filter.

This package implements the reduced filter based on:

- `compute_eskf_jacobians_reduced.m`
- `ErrorStateKalmanFilter_reduced.m`

It estimates only:

- target position `p_t`
- target velocity `v_t`
- normalized image feature `pbar`

The interceptor state is not estimated here. Instead, the node consumes interceptor truth-like inputs from:

- `/mavros/global_position/local` as `nav_msgs/msg/Odometry`
- `/mavros/imu/data_raw` as `sensor_msgs/msg/Imu`

Inside the node, the interceptor odometry is converted from:

- ENU to NED for position
- ENU to NED for local linear velocity
- ENU/FLU to NED/FRD for attitude

Radar input is `nav_msgs/msg/Odometry` as well:

- pose position is treated as target position in NED by default
- twist linear is treated as target velocity in NED when `radar.use_vr: true`
- if `radar.measurement_is_relative: true`, the node converts `[p_r; v_r]` with
  `p_r = p_i - p_t` and `v_r = v_i - v_t`

## Behavior

- Waits for the first radar measurement before starting the filter
- Freezes `pbar` propagation until the first image measurement arrives
- Uses delayed image updates with state-history replay
- Does not use magnetometer updates
- Does not use ZUPT updates

## Topics

- Input IMU: `topics.imu`
- Input image feature: `topics.image`
- Input radar: `topics.radar`
- Input interceptor odom: `topics.interceptor_odom`
- Output target odom: `topics.output_state`
- Output `pbar`: `topics.output_pose`

Default values are in `config/eskf_reduced_params.yaml`.

## Build

Standalone CMake:

```bash
cmake -S eskf_cpp_reduced -B eskf_cpp_reduced/build
cmake --build eskf_cpp_reduced/build
```

ROS2 workspace:

```bash
cd ~/ros2_ws/src
ln -s /path/to/Delayed-KalmanFilter-VisualGuidance/eskf_cpp_reduced .
cd ~/ros2_ws
colcon build --packages-select eskf_cpp_reduced
source install/setup.bash
```

## Launch

```bash
ros2 launch eskf_cpp_reduced eskf_reduced.launch.py
```

You can override the YAML file with the `config_file` launch argument.
