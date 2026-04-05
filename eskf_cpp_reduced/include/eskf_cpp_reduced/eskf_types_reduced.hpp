/**
 * @file eskf_types_reduced.hpp
 * @brief Reduced-state ESKF types for target position/velocity/pbar estimation.
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>

namespace eskf {

using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Quaterniond = Eigen::Quaterniond;
using RotationMatrix = Eigen::Matrix3d;

namespace constants {
    /// Unit vector in z-direction (gravity direction in NED)
    inline const Vector3d E3{0.0, 0.0, 1.0};

    /// Minimum camera depth to prevent division by zero [m]
    constexpr double MIN_DEPTH = 0.1;
    
    /// Small angle threshold for numerical stability
    constexpr double SMALL_ANGLE_THRESHOLD = 1e-10;
}

/**
 * @brief Configuration parameters for the Reduced ESKF
 */
struct ESKFParams {
    std::string topic_imu = "/interceptor/imu/data_raw";
    std::string topic_image = "/interceptor/camera/target_pbar";
    std::string topic_radar = "/radar/target_odom";
    std::string topic_interceptor_odom = "/mavros/global_position/local";
    std::string topic_interceptor_state = "/interceptor/state";
    std::string topic_odom = "/eskf_reduced/odom";
    std::string topic_pbar = "/eskf_reduced/pbar";
    
    double dt_imu = 1.0 / 200.0;
    double dt_eskf = 1.0 / 200.0;
    double image_delay = 0.080;
    double image_timeout_sec = 2.0;
    
    double sigma_img = 0.005;
    double sigma_radar_pos = 1.0;
    double sigma_radar_vel = 0.5;
    double radar_noise_inflation = 1.0;
    
    RotationMatrix R_b2c = (RotationMatrix() << 
        0, 1, 0,
        0, 0, 1,
        1, 0, 0).finished();
    
    int history_length = 25;
    
    double init_sigma_position = 3.0;
    double init_sigma_velocity = 0.5;
    double init_sigma_pbar = 0.1;
    
    bool enable_false_detection_image = true;
    bool enable_false_detection_radar = true;
    double chi2_threshold_image = 18.42;
    double chi2_threshold_radar_3dof = 16.27;
    double chi2_threshold_radar_6dof = 27.86;
    bool use_vr = false;
    bool radar_measurement_is_relative = false;

    // Delay compensation
    bool enable_image_extrapolation = false; ///< true = Method 2 (extrapolate), false = Method 1 (repropagate)

    double sigma_target_rw = 1.5;
    
    bool log_enabled = false;
    double log_rate_hz = 20.0;
    std::string log_dir = "log/";
};

namespace reduced {

using NominalState = Eigen::Matrix<double, 8, 1>;
using ErrorState = Eigen::Matrix<double, 8, 1>;

using ErrorCovariance = Eigen::Matrix<double, 8, 8>;
using StateJacobian = Eigen::Matrix<double, 8, 8>;
using NoiseJacobian = Eigen::Matrix<double, 8, 3>;
using ProcessNoise = Eigen::Matrix3d;

using ImageJacobian = Eigen::Matrix<double, 2, 8>;
using RadarMeasurement = Vector6d;
using RadarJacobian = Eigen::Matrix<double, 6, 8>;
using ImageNoise = Eigen::Matrix2d;
using RadarNoise = Eigen::Matrix<double, 6, 6>;

namespace nominal_idx {
constexpr int PT_START = 0;
constexpr int PT_SIZE = 3;
constexpr int VT_START = 3;
constexpr int VT_SIZE = 3;
constexpr int PBAR_START = 6;
constexpr int PBAR_SIZE = 2;
constexpr int STATE_SIZE = 8;
}  // namespace nominal_idx

namespace error_idx {
constexpr int DPT_START = 0;
constexpr int DPT_SIZE = 3;
constexpr int DVT_START = 3;
constexpr int DVT_SIZE = 3;
constexpr int DPBAR_START = 6;
constexpr int DPBAR_SIZE = 2;
constexpr int STATE_SIZE = 8;
}  // namespace error_idx

namespace noise_idx {
constexpr int VT_RW_START = 0;
constexpr int VT_RW_SIZE = 3;
constexpr int NOISE_SIZE = 3;
}  // namespace noise_idx

struct InterceptorState {
    Vector3d position_ned = Vector3d::Zero();
    Vector3d velocity_ned = Vector3d::Zero();
    RotationMatrix R_b2e = RotationMatrix::Identity();
};

struct IMUMeasurement {
    Vector3d omega = Vector3d::Zero();
    double timestamp = 0.0;

    IMUMeasurement() = default;
    IMUMeasurement(const Vector3d& omega_in, double timestamp_in)
        : omega(omega_in), timestamp(timestamp_in) {}
};

struct StateHistoryEntry {
    NominalState x = NominalState::Zero();
    ErrorCovariance P = ErrorCovariance::Zero();
    IMUMeasurement imu;
    InterceptorState interceptor;
    double timestamp = 0.0;
};

namespace state_access {

inline Vector3d getPosition(const NominalState& x) {
    return x.segment<3>(nominal_idx::PT_START);
}

inline Vector3d getVelocity(const NominalState& x) {
    return x.segment<3>(nominal_idx::VT_START);
}

inline Vector2d getPbar(const NominalState& x) {
    return x.segment<2>(nominal_idx::PBAR_START);
}

}  // namespace state_access

}  // namespace reduced
}  // namespace eskf

