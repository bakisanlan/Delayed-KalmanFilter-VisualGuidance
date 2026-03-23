/**
 * @file eskf_types.hpp
 * @brief Type definitions, state indices, and constants for ESKF
 * 
 * This file defines the core data structures for the Error-State Kalman Filter:
 * - Nominal state (21 dimensions): [q(4), pr(3), vr(3), pbar(2), bgyr(3), bacc(3), bmag(3)]
 * - Error state (20 dimensions): [δθ(3), δpr(3), δvr(3), δpbar(2), δbgyr(3), δbacc(3), δbmag(3)]
 * 
 * Based on ESKF_Report.tex mathematical formulation.
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>

namespace eskf {

// ============================================================================
// Type Aliases for Eigen matrices
// ============================================================================

// Fixed-size vectors
using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using Vector6d = Eigen::Matrix<double, 6, 1>;  // For radar [pr; vr]
using Quaterniond = Eigen::Quaterniond;

// State vectors
using NominalState = Eigen::Matrix<double, 21, 1>;    ///< Nominal state x
using ErrorState = Eigen::Matrix<double, 20, 1>;      ///< Error state δx

// Covariance matrices
using ErrorCovariance = Eigen::Matrix<double, 20, 20>;  ///< P matrix (20x20)

// Jacobian matrices
using StateJacobian = Eigen::Matrix<double, 20, 20>;    ///< Fc, Fd
using NoiseJacobian = Eigen::Matrix<double, 20, 15>;    ///< Gc, Gd
using ProcessNoise = Eigen::Matrix<double, 15, 15>;     ///< Qc, Qd

// Measurement matrices
using ImageMeasurement = Vector2d;                       ///< pbar measurement
using RadarMeasurement = Vector6d;                       ///< [pr; vr] measurement
using ImageJacobian = Eigen::Matrix<double, 2, 20>;      ///< H_img
using RadarJacobian = Eigen::Matrix<double, 6, 20>;      ///< H_radar (6x20)
using ImageNoise = Eigen::Matrix2d;                      ///< R_img
using RadarNoise = Eigen::Matrix<double, 6, 6>;          ///< R_radar (6x6)

// ZUPT (Zero Velocity Update) matrices
using ZUPTMeasurement = Vector6d;                        ///< [accel; gyro] residual
using ZUPTJacobian = Eigen::Matrix<double, 6, 20>;       ///< H_zupt

// Magnetometer measurement matrices (NORMALIZED - dimensionless)
using MagMeasurement = Vector3d;                         ///< Magnetometer measurement (unit vector)
using MagJacobian = Eigen::Matrix<double, 3, 20>;        ///< H_mag (3x20)
using MagNoise = Eigen::Matrix3d;                        ///< R_mag (3x3)
using ZUPTNoise = Eigen::Matrix<double, 6, 6>;           ///< R_zupt

// Rotation matrix
using RotationMatrix = Eigen::Matrix3d;

// ============================================================================
// State Indices - Nominal State (21 dimensions)
// ============================================================================

namespace nominal_idx {
    // Quaternion [q0, q1, q2, q3] - scalar first convention
    constexpr int Q_START = 0;
    constexpr int Q_SIZE = 4;
    
    // Relative position in earth frame
    constexpr int PR_START = 4;
    constexpr int PR_SIZE = 3;
    
    // Relative velocity in earth frame
    constexpr int VR_START = 7;
    constexpr int VR_SIZE = 3;
    
    // Normalized image coordinates
    constexpr int PBAR_START = 10;
    constexpr int PBAR_SIZE = 2;
    
    // Gyroscope bias
    constexpr int BGYR_START = 12;
    constexpr int BGYR_SIZE = 3;
    
    // Accelerometer bias
    constexpr int BACC_START = 15;
    constexpr int BACC_SIZE = 3;
    
    // Magnetometer bias
    constexpr int BMAG_START = 18;
    constexpr int BMAG_SIZE = 3;
    
    // Total nominal state size
    constexpr int STATE_SIZE = 21;
}

// ============================================================================
// State Indices - Error State (20 dimensions)
// ============================================================================

namespace error_idx {
    // Attitude error (axis-angle representation)
    constexpr int DTHETA_START = 0;
    constexpr int DTHETA_SIZE = 3;
    
    // Position error
    constexpr int DPR_START = 3;
    constexpr int DPR_SIZE = 3;
    
    // Velocity error
    constexpr int DVR_START = 6;
    constexpr int DVR_SIZE = 3;
    
    // Image feature error
    constexpr int DPBAR_START = 9;
    constexpr int DPBAR_SIZE = 2;
    
    // Gyroscope bias error
    constexpr int DBGYR_START = 11;
    constexpr int DBGYR_SIZE = 3;
    
    // Accelerometer bias error
    constexpr int DBACC_START = 14;
    constexpr int DBACC_SIZE = 3;
    
    // Magnetometer bias error
    constexpr int DBMAG_START = 17;
    constexpr int DBMAG_SIZE = 3;
    
    // Total error state size
    constexpr int STATE_SIZE = 20;
}

// ============================================================================
// Noise Vector Indices (15 dimensions)
// ============================================================================

namespace noise_idx {
    // Gyroscope measurement noise
    constexpr int OMEGA_N_START = 0;
    constexpr int OMEGA_N_SIZE = 3;
    
    // Accelerometer measurement noise
    constexpr int A_N_START = 3;
    constexpr int A_N_SIZE = 3;
    
    // Gyroscope bias random walk
    constexpr int OMEGA_W_START = 6;
    constexpr int OMEGA_W_SIZE = 3;
    
    // Accelerometer bias random walk
    constexpr int A_W_START = 9;
    constexpr int A_W_SIZE = 3;
    
    // Magnetometer bias random walk
    constexpr int MAG_W_START = 12;
    constexpr int MAG_W_SIZE = 3;
    
    // Total noise vector size
    constexpr int NOISE_SIZE = 15;
}

// ============================================================================
// Physical Constants
// ============================================================================

namespace constants {
    /// DEFAULT Gravity magnitude [m/s²] - can be overridden via ESKFParams.gravity
    constexpr double DEFAULT_GRAVITY = 9.81;
    
    /// Unit vector in z-direction (gravity direction in NED)
    inline const Vector3d E3{0.0, 0.0, 1.0};
    

    
    /// Minimum camera depth to prevent division by zero [m]
    constexpr double MIN_DEPTH = 0.1;
    
    /// Small angle threshold for numerical stability
    constexpr double SMALL_ANGLE_THRESHOLD = 1e-10;
}

// ============================================================================
// IMU Measurement Structure
// ============================================================================

/**
 * @brief IMU measurement containing gyroscope and accelerometer data
 */
struct IMUMeasurement {
    Vector3d omega;     ///< Angular velocity [rad/s]
    Vector3d accel;     ///< Linear acceleration [m/s²]
    double timestamp;   ///< Measurement timestamp [s]
    
    IMUMeasurement() : omega(Vector3d::Zero()), accel(Vector3d::Zero()), timestamp(0.0) {}
    IMUMeasurement(const Vector3d& w, const Vector3d& a, double t = 0.0) 
        : omega(w), accel(a), timestamp(t) {}
};

// ============================================================================
// State History Entry for Delayed Measurements
// ============================================================================

/**
 * @brief History entry storing state and covariance at a specific time
 */
struct StateHistoryEntry {
    NominalState x;             ///< Nominal state
    ErrorCovariance P;          ///< Error covariance
    IMUMeasurement imu;         ///< IMU measurement used for this step
    double timestamp;           ///< Time of this entry
    
    StateHistoryEntry() 
        : x(NominalState::Zero())
        , P(ErrorCovariance::Zero())
        , timestamp(0.0) {}
};

// ============================================================================
// ESKF Configuration Structure
// ============================================================================

/**
 * @brief Configuration parameters for the ESKF
 */
struct ESKFParams {
    // Topic names (ROS2)
    std::string topic_imu = "/mavros/imu/data_raw";    ///< IMU topic
    std::string topic_image = "/yolo/target";           ///< Image feature topic
    std::string topic_radar = "/radar/target_odom";    ///< Radar target-state odometry topic
    std::string topic_interceptor_odom = "/mavros/global_position/local";  ///< Interceptor odometry: pose in ENU/FLU, linear velocity in local ENU
    std::string topic_odom = "/eskf_reduced/odom";      ///< Output odometry topic
    std::string topic_pbar = "/eskf_reduced/pbar";      ///< Output pbar topic
    
    // Timing parameters
    double dt_imu = 1.0 / 200.0;     ///< IMU sampling period [s]
    double dt_eskf = 1.0 / 200.0;    ///< ESKF update period [s]
    double image_delay = 0.080;       ///< Image processing delay [s]
    double image_timeout_sec = 2.0;   ///< Freeze pbar after this many seconds without image [s]
    
    // IMU noise parameters (continuous-time)
    double sigma_omega_n = 0.01;     ///< Gyro noise std [rad/s]
    double sigma_a_n = 0.1;          ///< Accel noise std [m/s²]
    double sigma_omega_w = 1e-5;     ///< Gyro bias walk [rad/s√s]
    double sigma_a_w = 1e-4;         ///< Accel bias walk [m/s²√s]
    
    // Measurement noise
    double sigma_img = 0.005;        ///< Image coordinate noise
    double sigma_radar_pos = 1.0;    ///< Radar position noise [m]
    double sigma_radar_vel = 0.5;    ///< Radar velocity noise [m/s]
    
    // Camera-to-body rotation (transforms vector from body to camera frame)
    RotationMatrix R_b2c = (RotationMatrix() << 
        0, 1, 0,
        0, 0, 1,
        1, 0, 0).finished();
    
    // History buffer length (for delayed measurements)
    int history_length = 25;
    
    // Initial covariance standard deviations
    double init_sigma_attitude = 0.05;   ///< ~3° [rad]
    double init_sigma_position = 3.0;    ///< [m]
    double init_sigma_velocity = 0.5;    ///< [m/s]
    double init_sigma_pbar = 0.1;        ///< Normalized coordinates
    double init_sigma_bgyr = 0.005;      ///< [rad/s]
    double init_sigma_bacc = 0.05;       ///< [m/s²]
    
    // Chi-square gating thresholds (for outlier rejection)
    // chi2inv(0.9999, DoF) - 99.99% confidence level
    bool enable_false_detection_image = true;  ///< Enable chi-square gating for image
    bool enable_false_detection_radar = true;  ///< Enable chi-square gating for radar
    double chi2_threshold_image = 18.42; ///< 2 DoF image measurement
    double chi2_threshold_radar = 27.86; ///< 6 DoF radar measurement
    bool use_vr = false;                 ///< Use radar target velocity in measurement update
    bool radar_measurement_is_relative = false;  ///< Radar topic provides interceptor-target relative [position; velocity] in NED

    // Reduced-filter process noise
    double sigma_target_rw = 1.5;        ///< Target velocity random-walk driving noise [m/s²]
    
    // Physical constants
    double gravity = 9.81;               ///< Gravity magnitude [m/s²]
    
    // Data Logging parameters
    bool log_enabled = false;             ///< Enable CSV logging of state and covariance
    double log_rate_hz = 20.0;            ///< Logging rate [Hz]
    std::string log_file_path = "log/eskf_log.csv";  ///< Output file path
};

// ============================================================================
// Helper Functions for State Access
// ============================================================================

namespace state_access {

/**
 * @brief Extract quaternion from nominal state (returns Eigen quaternion)
 */
inline Quaterniond getQuaternion(const NominalState& x) {
    // Eigen Quaterniond constructor: (w, x, y, z)
    return Quaterniond(x(0), x(1), x(2), x(3));
}

/**
 * @brief Set quaternion in nominal state
 */
inline void setQuaternion(NominalState& x, const Quaterniond& q) {
    x(0) = q.w();
    x(1) = q.x();
    x(2) = q.y();
    x(3) = q.z();
}

/**
 * @brief Get relative position from nominal state
 */
inline Vector3d getPosition(const NominalState& x) {
    return x.segment<3>(nominal_idx::PR_START);
}

/**
 * @brief Get relative velocity from nominal state
 */
inline Vector3d getVelocity(const NominalState& x) {
    return x.segment<3>(nominal_idx::VR_START);
}

/**
 * @brief Get image features (pbar) from nominal state
 */
inline Vector2d getPbar(const NominalState& x) {
    return x.segment<2>(nominal_idx::PBAR_START);
}

/**
 * @brief Get gyroscope bias from nominal state
 */
inline Vector3d getGyroBias(const NominalState& x) {
    return x.segment<3>(nominal_idx::BGYR_START);
}

/**
 * @brief Get accelerometer bias from nominal state
 */
inline Vector3d getAccelBias(const NominalState& x) {
    return x.segment<3>(nominal_idx::BACC_START);
}

/**
 * @brief Get magnetometer bias from nominal state
 */
inline Vector3d getMagBias(const NominalState& x) {
    return x.segment<3>(nominal_idx::BMAG_START);
}

} // namespace state_access

} // namespace eskf
