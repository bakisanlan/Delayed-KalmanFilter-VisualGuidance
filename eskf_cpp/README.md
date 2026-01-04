# ESKF C++ - Error-State Kalman Filter

Production-ready C++ implementation of the Error-State Kalman Filter for visual guidance and drone interception, based on "High-Speed Interception Multicopter Control by Image-Based Visual Servoing" by Kun Yang et al.

## Features

- **18-state nominal / 17-state error** ESKF implementation
- **ROS2 integration** with sensor subscribers and pose publishers
- **Frequency decoupling** - ESKF can run slower than IMU rate
- **Delayed measurement handling** for image processing latency
- **YAML configuration** for all parameters
- **Standalone test harness** for validation against MATLAB

## Quick Start

### Building (Standalone, no ROS2)

```bash
cd eskf_cpp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

### Running Test Harness

```bash
./eskf_test_harness ../config/eskf_params.yaml
```

### Building with ROS2

```bash
cd ~/ros2_ws/src
ln -s /path/to/eskf_cpp .
cd ~/ros2_ws
colcon build --packages-select eskf_cpp
source install/setup.bash
```

### Launching ROS2 Node

```bash
ros2 launch eskf_cpp eskf.launch.py
```

## Configuration

Edit `config/eskf_params.yaml` to configure:

- **Timing**: IMU rate, ESKF rate, image delay
- **IMU Noise**: σ_ωn, σ_an, σ_ωw, σ_aw
- **Measurement Noise**: Image and radar sigma
- **Initial Covariance**: State uncertainty
- **ROS2 Topics**: Input/output topic names

## File Structure

```
eskf_cpp/
├── include/eskf_cpp/    # Headers
│   ├── eskf_types.hpp   # Type definitions
│   ├── eskf_math.hpp    # Math utilities
│   ├── eskf_jacobian.hpp
│   ├── eskf_core.hpp    # Main ESKF class
│   └── eskf_config.hpp  # YAML loader
├── src/                 # Source files
│   ├── eskf_jacobian.cpp
│   ├── eskf_core.cpp
│   ├── eskf_config.cpp
│   ├── eskf_node.cpp    # ROS2 node
│   └── eskf_test_harness.cpp
├── config/
│   └── eskf_params.yaml
├── launch/
│   └── eskf.launch.py
├── msg/
│   └── ESKFState.msg
├── CMakeLists.txt
└── package.xml
```

## State Definitions

**Nominal State (18):** `[q(4), pr(3), vr(3), pbar(2), bgyr(3), bacc(3)]`

**Error State (17):** `[δθ(3), δpr(3), δvr(3), δpbar(2), δbgyr(3), δbacc(3)]`

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mavros/imu/data_raw` | sensor_msgs/Imu | IMU input |
| `/yolo/target` | geometry_msgs/Point | Image features |
| `/radar/pr` | geometry_msgs/Vector3 | Radar position |
| `/eskf/pose` | geometry_msgs/PoseWithCovarianceStamped | State output |

## Dependencies

- **Eigen3** - Matrix operations
- **yaml-cpp** - Configuration parsing
- **ROS2 Humble** (optional) - ROS2 integration

## License

MIT License
