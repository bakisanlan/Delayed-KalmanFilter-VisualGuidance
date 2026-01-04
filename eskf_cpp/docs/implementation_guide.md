# ESKF C++ Implementation Guide

This guide explains the C++ implementation of the Error-State Kalman Filter for users who are not C++ experts. It provides a walkthrough of the code architecture, key algorithms, and how to modify the filter for your needs.

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Building the Code](#building-the-code)
3. [Code Walkthrough](#code-walkthrough)
4. [ESKF Frequency Decoupling](#eskf-frequency-decoupling)
5. [Running the Test Harness](#running-the-test-harness)
6. [ROS2 Integration](#ros2-integration)
7. [Tuning Parameters](#tuning-parameters)
8. [Debugging Tips](#debugging-tips)

---

## Architecture Overview

### Class Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                         eskf_types.hpp                          │
│  - NominalState (18-dim Eigen vector)                           │
│  - ErrorState (17-dim Eigen vector)                             │
│  - ESKFParams (configuration structure)                         │
│  - State indices (nominal_idx::, error_idx::)                   │
└─────────────────────────────────────────────────────────────────┘
                              ↓ uses
┌─────────────────────────────────────────────────────────────────┐
│                         eskf_math.hpp                           │
│  - skew(v) → 3x3 skew-symmetric matrix                          │
│  - expQuaternion(δθ) → quaternion from axis-angle               │
│  - expRotation(φ) → rotation matrix (Rodrigues formula)         │
│  - computeLv(), computeLw() → IBVS Jacobians                    │
└─────────────────────────────────────────────────────────────────┘
                              ↓ uses
┌─────────────────────────────────────────────────────────────────┐
│                       eskf_jacobian.hpp/cpp                     │
│  - computeESKFJacobians() → returns Fc, Gc, Fd, Gd              │
│  - Called every prediction step                                 │
└─────────────────────────────────────────────────────────────────┘
                              ↓ uses
┌─────────────────────────────────────────────────────────────────┐
│                         eskf_core.hpp/cpp                       │
│  ErrorStateKalmanFilter class:                                  │
│  - predict(ω, a, t) → propagate state and covariance            │
│  - correctImage(z, delay) → delayed image correction            │
│  - correctRadar(z) → radar correction                           │
│  - accumulateIMU() → for frequency decoupling                   │
│  - History buffer for delayed measurements                      │
└─────────────────────────────────────────────────────────────────┘
```

### Data Flow

```
IMU (200 Hz)  ─────┐
                   ↓ accumulate
              ┌─────────┐
              │  ESKF   │  ← dt_eskf (e.g., 100 Hz)
              │ predict │
              └────┬────┘
                   ↓
Image (30 Hz) ────→ correctImage() ─── with delay compensation ───┐
                                                                   ↓
Radar (0.5 Hz) ───→ correctRadar() ───────────────────────────────→ State Estimate
```

---

## Building the Code

### Prerequisites (Ubuntu 22.04)

```bash
# Install Eigen3
sudo apt install libeigen3-dev

# Install yaml-cpp
sudo apt install libyaml-cpp-dev

# (Optional) Install ROS2 Humble
# Follow: https://docs.ros.org/en/humble/Installation.html
```

### Standalone Build (No ROS2)

```bash
cd eskf_cpp
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

This creates:
- `libeskf_core.a` - Static library with ESKF implementation
- `eskf_test_harness` - Standalone test program

### ROS2 Build

```bash
# In your ROS2 workspace
cd ~/ros2_ws/src
ln -s /path/to/eskf_cpp .  # symlink or copy
cd ~/ros2_ws
colcon build --packages-select eskf_cpp
source install/setup.bash
```

---

## Code Walkthrough

### 1. State Representation

The ESKF uses two state vectors:

```cpp
// Nominal state (18 elements)
// Indices: q(0-3), pr(4-6), vr(7-9), pbar(10-11), bgyr(12-14), bacc(15-17)
using NominalState = Eigen::Matrix<double, 18, 1>;

// Error state (17 elements) - attitude uses 3-element axis-angle instead of 4-element quaternion
// Indices: δθ(0-2), δpr(3-5), δvr(6-8), δpbar(9-10), δbgyr(11-13), δbacc(14-16)
using ErrorState = Eigen::Matrix<double, 17, 1>;
```

### 2. Prediction Step

The prediction step (from `eskf_core.cpp`):

```cpp
void ErrorStateKalmanFilter::predict(const Vector3d& omega_meas, 
                                      const Vector3d& a_meas, 
                                      double timestamp) {
    // 1. Propagate nominal state
    NominalState x_new = predictNominalState(x_, omega_meas, a_meas, params_.dt_eskf);
    
    // 2. Compute Jacobians (Fd: state transition, Gc: noise coupling)
    ESKFJacobians jac = computeESKFJacobians(x_, omega_meas, a_meas, 
                                              params_.dt_eskf, params_.R_b2c);
    
    // 3. Propagate covariance: P = Fd * P * Fd' + Gc * Qd * Gc'
    ErrorCovariance P_new;
    P_new.noalias() = jac.Fd * P_ * jac.Fd.transpose() 
                    + jac.Gc * Qd_ * jac.Gc.transpose();
    
    // 4. Update state and history
    x_ = x_new;
    P_ = P_new;
    updateHistory(imu);
}
```

### 3. Correction Step

The image correction with delay handling:

```cpp
Vector2d ErrorStateKalmanFilter::correctImage(const Vector2d& z_pbar, int delay_steps) {
    // 1. Get delayed state from history
    const NominalState& x_prior = history_[idx_delayed].x;
    const ErrorCovariance& P_prior = history_[idx_delayed].P;
    
    // 2. Compute innovation: y = z - h(x)
    Vector2d innovation = z_pbar - getPbar(x_prior);
    
    // 3. Kalman gain: K = P * H' * (H * P * H' + R)^(-1)
    const Eigen::Matrix<double, 17, 2> K = P_prior * H_img_.transpose() * S.inverse();
    
    // 4. Error state update: δx = K * y
    ErrorState delta_x = K * innovation;
    
    // 5. Inject error into nominal state
    NominalState x_corrected = injectErrorState(x_prior, delta_x);
    
    // 6. Covariance update (Joseph form for numerical stability)
    ErrorCovariance P_corrected = (I - K*H) * P_prior * (I - K*H)' + K * R * K';
    
    // 7. ESKF reset: P = G * P * G'
    P_corrected = resetCovariance(P_corrected, delta_x);
    
    // 8. Re-propagate from delayed time to current time
    repropagate(x_corrected, P_corrected, idx_delayed);
    
    return innovation;
}
```

### 4. Quaternion Exponential Map

Converting axis-angle to quaternion:

```cpp
Quaterniond expQuaternion(const Vector3d& delta_theta) {
    double theta = delta_theta.norm();  // Rotation angle
    
    if (theta < 1e-10) {
        // Small angle: q ≈ [1, δθ/2]
        return Quaterniond(1.0, delta_theta(0)*0.5, delta_theta(1)*0.5, delta_theta(2)*0.5);
    }
    
    // Full formula: q = [cos(θ/2), sin(θ/2) * axis]
    double half_theta = theta * 0.5;
    Vector3d axis = delta_theta / theta;
    return Quaterniond(cos(half_theta), 
                       sin(half_theta) * axis(0),
                       sin(half_theta) * axis(1),
                       sin(half_theta) * axis(2));
}
```

---

## ESKF Frequency Decoupling

A key feature for real-world applications: running ESKF slower than IMU rate.

### Why?

- **Computational efficiency**: Running at 100 Hz instead of 200 Hz halves CPU load
- **Resource management**: Frees up processing for control, vision, etc.
- **Minimal accuracy loss**: Averaging IMU samples preserves information

### How It Works

```cpp
// In main loop (runs at IMU rate, e.g., 200 Hz)
for each IMU sample:
    eskf.accumulateIMU(omega, accel);  // Simple accumulation
    
    if (eskf.isReadyForPrediction()) {  // Check if ESKF should run
        IMUMeasurement avg = eskf.getAveragedIMU();  // Compute average
        eskf.predict(avg.omega, avg.accel, timestamp);
    }
```

### Inside the ESKF

```cpp
void accumulateIMU(const Vector3d& omega, const Vector3d& accel) {
    omega_accum_ += omega;   // Sum angular velocities
    accel_accum_ += accel;   // Sum accelerations
    imu_count_++;
}

IMUMeasurement getAveragedIMU() {
    IMUMeasurement avg;
    avg.omega = omega_accum_ / imu_count_;  // Average
    avg.accel = accel_accum_ / imu_count_;
    
    // Reset accumulators
    omega_accum_.setZero();
    accel_accum_.setZero();
    imu_count_ = 0;
    
    return avg;
}
```

### Configuration

In `eskf_params.yaml`:

```yaml
timing:
  imu_rate_hz: 200     # Sensor rate
  eskf_rate_hz: 100    # Filter rate (can be lower)
```

---

## Running the Test Harness

The test harness validates the C++ implementation against the MATLAB simulation.

```bash
cd build
./eskf_test_harness ../config/eskf_params.yaml
```

### Expected Output

```
=== ESKF C++ Test Harness ===

============ ESKF Configuration ============
Timing:
  IMU rate:    200 Hz
  ESKF rate:   200 Hz
  Image delay: 80 ms
...

Starting simulation...
Progress: 10%
Progress: 20%
...
Progress: 100%

Simulation Complete!
Execution time: 1234 ms
Processing rate: 40485 Hz

=== ESKF Performance Statistics ===
Position RMSE:   0.2345 m
Velocity RMSE:   0.1234 m/s
Attitude RMSE:   0.5678 deg
Gyro Bias RMSE:  0.0012 rad/s
Accel Bias RMSE: 0.0234 m/s²
===================================

Results saved to: cpp_results.csv
```

---

## ROS2 Integration

### Launch the Node

```bash
ros2 launch eskf_cpp eskf.launch.py
```

### With Custom Topics

```bash
ros2 launch eskf_cpp eskf.launch.py \
    imu_topic:=/my_imu/data \
    radar_topic:=/my_radar/position
```

### Check Output

```bash
ros2 topic echo /eskf/pose
```

---

## Tuning Parameters

### IMU Noise

Match these to your sensor specifications:

| Parameter | Typical MEMS | Tactical | Navigation |
|-----------|--------------|----------|------------|
| σ_ωn (gyro noise) | 0.01 rad/s | 0.001 | 0.0001 |
| σ_an (accel noise) | 0.1 m/s² | 0.01 | 0.001 |
| σ_ωw (gyro bias RW) | 1e-5 rad/s√s | 1e-6 | 1e-7 |
| σ_aw (accel bias RW) | 1e-4 m/s²√s | 1e-5 | 1e-6 |

### Initial Covariance

Set based on your initial uncertainty:

```yaml
initial_covariance:
  attitude: 0.05      # 3° uncertainty → 0.05 rad
  position: 10.0      # 10 m uncertainty
  velocity: 1.0       # 1 m/s uncertainty
```

### Measurement Noise

- **Image sigma**: Depends on camera resolution and detection accuracy
- **Radar sigma**: Depends on radar range resolution

---

## Debugging Tips

### 1. Covariance Growing Unbounded

**Symptom**: P trace goes to infinity  
**Cause**: Measurements not being incorporated  
**Fix**: Check measurement topic connections, verify innovation computation

### 2. Filter Diverging

**Symptom**: Large position/attitude errors  
**Cause**: Incorrect noise tuning or initialization  
**Fix**: Increase process noise, check initial state

### 3. Performance Too Slow

**Symptom**: Can't achieve real-time rate  
**Fix**: 
- Reduce ESKF rate (use frequency decoupling)
- Enable Release build (`-DCMAKE_BUILD_TYPE=Release`)
- Use `.noalias()` for Eigen matrix products

### 4. Adding Logging

```cpp
#include <iostream>

// In predict():
std::cout << "State: " << x_.transpose() << std::endl;
std::cout << "P trace: " << P_.trace() << std::endl;
```

---

## Adding New Measurements

To add a new sensor (e.g., GPS):

1. **Define measurement Jacobian** in `eskf_core.hpp`:
   ```cpp
   using GPSJacobian = Eigen::Matrix<double, 3, 17>;
   GPSJacobian getGPSMeasurementMatrix() const;
   ```

2. **Implement correction** in `eskf_core.cpp`:
   ```cpp
   Vector3d correctGPS(const Vector3d& z_gps) {
       // Similar to correctRadar()
   }
   ```

3. **Add subscriber** in `eskf_node.cpp`:
   ```cpp
   gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(...);
   ```

---

## Performance Considerations

### Eigen Optimizations

The code uses several Eigen optimizations:

```cpp
// noalias() prevents temporary allocation
P_new.noalias() = jac.Fd * P_ * jac.Fd.transpose();

// Block access for submatrices
x_.segment<3>(4) = position;
P_.block<3,3>(0, 0) = attitude_cov;
```

### Memory Management

- State history uses `std::deque` for efficient front/back operations
- All matrices are fixed-size (no dynamic allocation in hot path)
- Pre-computed measurement matrices (H_img_, H_radar_)

---

## Conclusion

This implementation provides a production-ready ESKF that matches the MATLAB reference. The key features for real-world use are:

1. **Frequency decoupling** - Run filter slower than sensors
2. **Delayed measurements** - Handle image processing latency
3. **ROS2 integration** - Easy to connect to robot systems
4. **Configurable** - All parameters in YAML

For questions or issues, refer to the original MATLAB implementation and ESKF_Report.tex for mathematical details.
