/**
 * @file eskf_types_reduced.hpp
 * @brief Reduced-state ESKF types for target position/velocity/pbar estimation.
 */

#pragma once

#include "eskf_cpp_reduced/eskf_types.hpp"

namespace eskf::reduced {

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

}  // namespace eskf::reduced
