/**
 * @file eskf_math.hpp
 * @brief Mathematical utilities for ESKF - quaternions, rotations, and matrix operations
 * 
 * This file provides core mathematical functions used throughout the ESKF:
 * - Skew-symmetric matrix construction
 * - Quaternion exponential map (axis-angle to quaternion)
 * - Rotation matrix exponential map (Rodrigues formula)
 * - Quaternion utilities
 * 
 * Based on errorstate.m and compute_eskf_jacobians.m from MATLAB implementation.
 */

#pragma once

#include "eskf_cpp_reduced/eskf_types_reduced.hpp"
#include <cmath>

namespace eskf {
namespace math {

/**
 * @brief Safely bound depth bounded away from zero preserving its sign.
 * 
 * @param z Depth value
 * @return Safely bounded depth parameter
 */
inline double boundDepth(double z) {
    return z >= 0.0 ? std::max(z, constants::MIN_DEPTH) : std::min(z, -constants::MIN_DEPTH);
}

// ============================================================================
// Skew-Symmetric Matrix
// ============================================================================

/**
 * @brief Compute skew-symmetric (cross-product) matrix from 3-vector
 * 
 * For vector v = [vx, vy, vz]^T, returns matrix S such that S*x = v × x
 * 
 *     [  0   -vz   vy ]
 * S = [  vz   0   -vx ]
 *     [ -vy  vx    0  ]
 * 
 * @param v Input 3-vector
 * @return 3x3 skew-symmetric matrix
 */
inline Eigen::Matrix3d skew(const Vector3d& v) {
    Eigen::Matrix3d S;
    S <<     0.0, -v(2),  v(1),
           v(2),    0.0, -v(0),
          -v(1),  v(0),    0.0;
    return S;
}

// ============================================================================
// Quaternion Operations
// ============================================================================

/**
 * @brief Quaternion exponential map: convert rotation vector to quaternion
 * 
 * Given rotation vector δθ (axis-angle representation):
 *   q = exp_q(δθ) = [cos(|δθ|/2); sin(|δθ|/2) * δθ/|δθ|]
 * 
 * For small angles, approximation is used for numerical stability.
 * 
 * @param delta_theta Rotation vector (3x1) in radians
 * @return Unit quaternion representing the rotation
 */
inline Quaterniond expQuaternion(const Vector3d& delta_theta) {
    const double theta = delta_theta.norm();
    
    if (theta < constants::SMALL_ANGLE_THRESHOLD) {
        // Small angle approximation: q ≈ [1, δθ/2]
        return Quaterniond(1.0, 
                          delta_theta(0) * 0.5, 
                          delta_theta(1) * 0.5, 
                          delta_theta(2) * 0.5).normalized();
    }
    
    const double half_theta = theta * 0.5;
    const double sin_half = std::sin(half_theta);
    const double cos_half = std::cos(half_theta);
    const Vector3d axis = delta_theta / theta;
    
    return Quaterniond(cos_half, 
                       sin_half * axis(0), 
                       sin_half * axis(1), 
                       sin_half * axis(2));
}

/**
 * @brief Rotation matrix exponential map using Rodrigues' formula
 * 
 * Given rotation vector φ = θ * u (where u is unit axis):
 *   R = exp([φ]×) = I*cos(θ) + sin(θ)*[u]× + (1-cos(θ))*u*u^T
 * 
 * This is eq. 78 from "Quaternion kinematics for the error-state Kalman Filter"
 * 
 * @param phi Rotation vector (axis-angle, 3x1)
 * @return 3x3 rotation matrix
 */
inline RotationMatrix expRotation(const Vector3d& phi) {
    const double theta = phi.norm();
    
    if (theta < constants::SMALL_ANGLE_THRESHOLD) {
        // Small angle: R ≈ I + [φ]×
        return RotationMatrix::Identity() + skew(phi);
    }
    
    const Vector3d u = phi / theta;
    const Eigen::Matrix3d u_skew = skew(u);
    
    return RotationMatrix::Identity() * std::cos(theta) 
           + u_skew * std::sin(theta) 
           + (1.0 - std::cos(theta)) * (u * u.transpose());
}

/**
 * @brief Multiply two quaternions: q_result = q1 ⊗ q2
 * 
 * Note: Eigen's quaternion multiplication is q1 * q2, which corresponds to 
 * applying q2 rotation first, then q1. This matches our MATLAB convention.
 * 
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @return Resulting quaternion (normalized)
 */
inline Quaterniond quaternionMultiply(const Quaterniond& q1, const Quaterniond& q2) {
    return (q1 * q2).normalized();
}

/**
 * @brief Convert quaternion to rotation matrix (body to earth)
 * 
 * @param q Unit quaternion
 * @return 3x3 rotation matrix R_b2e
 */
inline RotationMatrix quaternionToRotation(const Quaterniond& q) {
    return q.normalized().toRotationMatrix();
}

/**
 * @brief Normalize quaternion and ensure positive scalar part
 * 
 * ESKF convention: quaternion scalar (w) should be positive to avoid 
 * sign ambiguity (q and -q represent the same rotation)
 * 
 * @param q Input quaternion
 * @return Normalized quaternion with positive w
 */
inline Quaterniond normalizeQuaternion(const Quaterniond& q) {
    Quaterniond q_normalized = q.normalized();
    if (q_normalized.w() < 0) {
        q_normalized.coeffs() *= -1.0;
    }
    return q_normalized;
}

// ============================================================================
// Reference Frame Conversions
// ============================================================================

inline const RotationMatrix kEnuToNed = (RotationMatrix() <<
    0.0, 1.0,  0.0,
    1.0, 0.0,  0.0,
    0.0, 0.0, -1.0).finished();

inline const RotationMatrix kFrdToFlu = (RotationMatrix() <<
    1.0,  0.0,  0.0,
    0.0, -1.0,  0.0,
    0.0,  0.0, -1.0).finished();

inline Vector3d enuToNed(const Vector3d& value_enu) {
    return kEnuToNed * value_enu;
}

inline Vector3d fluToFrd(const Vector3d& value_flu) {
    return kFrdToFlu * value_flu;
}

inline Quaterniond convertBodyOrientationEnuFluToNedFrd(const Quaterniond& q_enu_flu) {
    RotationMatrix R_flu2enu = q_enu_flu.normalized().toRotationMatrix();
    RotationMatrix R_frd2ned = kEnuToNed * R_flu2enu * kFrdToFlu;
    return normalizeQuaternion(Quaterniond(R_frd2ned));
}

// ============================================================================
// Covariance Matrix Utilities
// ============================================================================

inline Eigen::Matrix3d buildCovarianceBlock3x3(const std::array<double, 36>& covariance_msg,
                                               const Eigen::Matrix3d& fallback_covariance) {
    if (covariance_msg[0] < 0.0) {
        return fallback_covariance;
    }

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            const double value = covariance_msg[row * 6 + col];
            if (!std::isfinite(value)) {
                return fallback_covariance;
            }
            covariance(row, col) = value;
        }
    }

    covariance = 0.5 * (covariance + covariance.transpose());
    constexpr double kMinVariance = 1e-9;
    for (int i = 0; i < 3; ++i) {
        if (covariance(i, i) < 0.0) {
            return fallback_covariance;
        }
        covariance(i, i) = std::max(covariance(i, i), kMinVariance);
    }

    return covariance;
}

// ============================================================================
// IBVS (Image-Based Visual Servoing) Jacobians
// ============================================================================

/**
 * @brief Compute target position in camera frame (p_c)
 * 
 * @param p_t Target position in Earth (NED) frame
 * @param interceptor Interceptor state containing position and R_b2e
 * @param R_b2c Rotation from body to camera frame
 * @return 3D vector p_c
 */
inline Vector3d computeTargetInCameraFrame(const Vector3d& p_t,
                                           const reduced::InterceptorState& interceptor,
                                           const RotationMatrix& R_b2c) {
    const RotationMatrix R_e2b = interceptor.R_b2e.transpose();
    return R_b2c * R_e2b * -(interceptor.position_ned - p_t);
}

/**
 * @brief Compute normalized image features (pbar) from target and interceptor states
 * 
 * @param p_t Target position in Earth (NED) frame
 * @param interceptor Interceptor state containing position and R_b2e
 * @param R_b2c Rotation from body to camera frame
 * @return 2D vector [pbar_x, pbar_y]
 */
inline Vector2d computeImageFeatures(const Vector3d& p_t,
                                     const reduced::InterceptorState& interceptor,
                                     const RotationMatrix& R_b2c) {
    const Vector3d p_c = computeTargetInCameraFrame(p_t, interceptor, R_b2c);
    const double p_c_z = boundDepth(p_c(2));
    return Vector2d(p_c(0) / p_c_z, p_c(1) / p_c_z);
}

/**
 * @brief Project target position covariance into pbar covariance
 * 
 * @param P_pos 3x3 position covariance matrix
 * @param p_c Target position in camera frame
 * @param interceptor Interceptor state containing R_b2e
 * @param R_b2c Rotation from body to camera frame
 * @return 2x2 projected pbar covariance matrix
 */
inline Eigen::Matrix2d projectPositionCovarianceToPbar(const Eigen::Matrix3d& P_pos,
                                                       const Vector3d& p_c,
                                                       const reduced::InterceptorState& interceptor,
                                                       const RotationMatrix& R_b2c) {
    const double p_c_z = boundDepth(p_c(2));
    Eigen::Matrix<double, 2, 3> J_c;
    J_c << 1.0 / p_c_z, 0.0, -p_c(0) / (p_c_z * p_c_z),
           0.0, 1.0 / p_c_z, -p_c(1) / (p_c_z * p_c_z);
           
    const RotationMatrix R_e2b = interceptor.R_b2e.transpose();
    const Eigen::Matrix<double, 2, 3> H_pbar_pt = J_c * R_b2c * R_e2b;
    
    return H_pbar_pt * P_pos * H_pbar_pt.transpose();
}

/**
 * @brief Compute IBVS velocity Jacobian Lv
 * 
 * Lv relates camera velocity to image feature rate of change:
 *   pbar_dot (due to velocity) = Lv * v_c
 * 
 *      [ -1/pz    0    px/pz ]
 * Lv = [   0   -1/pz   py/pz ]
 * 
 * @param pbar_x Normalized x image coordinate
 * @param pbar_y Normalized y image coordinate  
 * @param p_c_z  Depth (z-component in camera frame)
 * @return 2x3 velocity Jacobian matrix
 */
inline Eigen::Matrix<double, 2, 3> computeLv(double pbar_x, double pbar_y, double p_c_z) {
    const double inv_z = 1.0 / p_c_z;
    
    Eigen::Matrix<double, 2, 3> Lv;
    Lv << -inv_z,    0.0,    pbar_x * inv_z,
            0.0,  -inv_z,    pbar_y * inv_z;
    return Lv;
}

/**
 * @brief Compute IBVS angular velocity Jacobian Lw
 * 
 * Lw relates camera angular velocity to image feature rate of change:
 *   pbar_dot (due to rotation) = Lw * omega_c
 * 
 *      [  px*py    -(1+px²)    py ]
 * Lw = [ (1+py²)   -px*py    -px ]
 * 
 * @param pbar_x Normalized x image coordinate
 * @param pbar_y Normalized y image coordinate
 * @return 2x3 angular velocity Jacobian matrix
 */
inline Eigen::Matrix<double, 2, 3> computeLw(double pbar_x, double pbar_y) {
    const double px = pbar_x;
    const double py = pbar_y;
    
    Eigen::Matrix<double, 2, 3> Lw;
    Lw << px * py,       -(1.0 + px * px),    py,
          (1.0 + py * py),  -px * py,          -px;
    return Lw;
}

/**
 * @brief Compute A_pbar Jacobian for δpbar_dot/δpbar
 * 
 * This is the partial derivative of pbar dynamics w.r.t. pbar itself.
 * 
 * @param pbar_x Normalized x image coordinate
 * @param pbar_y Normalized y image coordinate
 * @param v_c Camera frame velocity
 * @param omega_c Camera frame angular velocity
 * @param p_c_z Depth in camera frame
 * @return 2x2 Jacobian matrix
 */
inline Eigen::Matrix2d computeApbar(double pbar_x, double pbar_y,
                                     const Vector3d& v_c, 
                                     const Vector3d& omega_c,
                                     double p_c_z) {
    const double vz = v_c(2);
    const double wx = omega_c(0);
    const double wy = omega_c(1);
    const double wz = omega_c(2);
    const double z_inv = 1.0 / p_c_z;
    
    Eigen::Matrix2d A_pbar;
    A_pbar << vz * z_inv + pbar_y * wx - 2.0 * pbar_x * wy,    pbar_x * wx + wz,
              -pbar_y * wy - wz,    vz * z_inv + 2.0 * pbar_y * wx - pbar_x * wy;
    return A_pbar;
}

/**
 * @brief Compute A_pc_z Jacobian for δpbar_dot/δp_c_z
 * 
 * This is the partial derivative of pbar dynamics w.r.t. camera depth.
 * 
 * @param pbar_x Normalized x image coordinate
 * @param pbar_y Normalized y image coordinate
 * @param v_c Camera frame velocity
 * @param p_c_z Depth in camera frame
 * @return 2x1 Jacobian vector
 */
inline Vector2d computeApcz(double pbar_x, double pbar_y,
                            const Vector3d& v_c, double p_c_z) {
    const double vx = v_c(0);
    const double vy = v_c(1);
    const double vz = v_c(2);
    const double z_sq = p_c_z * p_c_z;
    
    Vector2d A_pcz;
    A_pcz << (vx - pbar_x * vz) / z_sq,
             (vy - pbar_y * vz) / z_sq;
    return A_pcz;
}



/**
 * @brief Compute reduced discrete process noise covariance Qd (3x3)
 *
 * The reduced filter injects process noise only through the target-velocity
 * driving noise. Position and pbar process noise remain zero directly.
 *
 * @param params ESKF parameters containing reduced target RW noise
 * @return 3x3 discrete process noise covariance for target-velocity RW
 */
inline reduced::ProcessNoise computeReducedDiscreteProcessNoise(const ESKFParams& params) {
    const double q_rw = params.sigma_target_rw * params.sigma_target_rw * params.dt_eskf;
    return q_rw * reduced::ProcessNoise::Identity();
}

} // namespace math
} // namespace eskf
