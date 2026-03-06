/**
 * @file eskf_core.cpp
 * @brief Implementation of Error-State Kalman Filter core class
 * 
 * Direct translation of ErrorStateKalmanFilter.m to C++.
 */

#include "eskf_cpp/eskf_core.hpp"
#include "eskf_cpp/utils/print.hpp"
#include <algorithm>
#include <cmath>

namespace eskf {

// ============================================================================
// Constructor
// ============================================================================

ErrorStateKalmanFilter::ErrorStateKalmanFilter(const ESKFParams& params)
    : params_(params)
    , x_(NominalState::Zero())
    , P_(ErrorCovariance::Zero())
    , current_time_(0.0)
    , omega_accum_(Vector3d::Zero())
    , accel_accum_(Vector3d::Zero())
    , imu_count_(0)
{
    // Compute IMU samples per ESKF update
    samples_per_eskf_ = std::max(1, static_cast<int>(
        std::round(params_.dt_eskf / params_.dt_imu)));
    
    // Compute discrete process noise
    Qd_ = math::computeDiscreteProcessNoise(params_);
    
    // Initialize gravity vector from config parameter
    gravity_ned_ = Vector3d(0.0, 0.0, params_.gravity);
    
    // Setup measurement noise covariances
    R_img_ = params_.sigma_img * params_.sigma_img * Eigen::Matrix2d::Identity();
    
    // Radar: 6x6 block diagonal for [position; velocity]
    R_radar_.setZero();
    R_radar_.block<3,3>(0, 0) = params_.sigma_radar_pos * params_.sigma_radar_pos * Eigen::Matrix3d::Identity();
    R_radar_.block<3,3>(3, 3) = params_.sigma_radar_vel * params_.sigma_radar_vel * Eigen::Matrix3d::Identity();
    
    // Setup fixed measurement matrices
    H_img_ = getImageMeasurementMatrix();
    H_radar_ = getRadarMeasurementMatrix();
    
    // ZUPT noise: use IMU noise parameters for stationary detection
    R_zupt_.setZero();
    R_zupt_.block<3,3>(0, 0) = params_.sigma_a_n * params_.sigma_a_n * Eigen::Matrix3d::Identity();
    R_zupt_.block<3,3>(3, 3) = params_.sigma_omega_n * params_.sigma_omega_n * Eigen::Matrix3d::Identity();
    
    // Magnetometer measurement noise
    R_mag_ = params_.sigma_mag_n * params_.sigma_mag_n * MagNoise::Identity();
    
    // Initialize error covariance
    P_ = createInitialCovariance();
    
    // Pre-allocate history buffer
    history_.resize(params_.history_length);
}

// ============================================================================
// Reset
// ============================================================================

void ErrorStateKalmanFilter::reset(const NominalState& x_init, 
                                    const ErrorCovariance& P_init) {
    x_ = x_init;
    P_ = P_init;
    current_time_ = 0.0;
    
    // Reset accumulators
    omega_accum_.setZero();
    accel_accum_.setZero();
    imu_count_ = 0;
    
    // Reset history
    for (auto& entry : history_) {
        entry.x = x_;
        entry.P = P_;
        entry.timestamp = current_time_;
    }
}

// ============================================================================
// IMU Accumulation
// ============================================================================

void ErrorStateKalmanFilter::accumulateIMU(const Vector3d& omega, 
                                            const Vector3d& accel) {
    omega_accum_ += omega;
    accel_accum_ += accel;
    imu_count_++;
}

bool ErrorStateKalmanFilter::isReadyForPrediction() const {
    return imu_count_ >= samples_per_eskf_;
}

IMUMeasurement ErrorStateKalmanFilter::getAveragedIMU() {
    IMUMeasurement avg;
    
    if (imu_count_ > 0) {
        avg.omega = omega_accum_ / static_cast<double>(imu_count_);
        avg.accel = accel_accum_ / static_cast<double>(imu_count_);
    }
    
    // Reset accumulators
    omega_accum_.setZero();
    accel_accum_.setZero();
    imu_count_ = 0;
    
    return avg;
}

// ============================================================================
// Prediction Step
// ============================================================================

void ErrorStateKalmanFilter::predict(const Vector3d& omega_meas, 
                                      const Vector3d& a_meas, 
                                      double timestamp) {
    current_time_ = timestamp;
    
    // 1. Propagate nominal state
    NominalState x_new = predictNominalState(x_, omega_meas, a_meas, params_.dt_eskf);
    
    // 2. Compute ESKF Jacobians
    ESKFJacobians jac = computeESKFJacobians(x_, omega_meas, a_meas, 
                                              params_.dt_eskf, params_.R_b2c);
    
    // 3. Propagate error covariance
    // P_new = Fd * P * Fd' + Gc * Qd * Gc'
    ErrorCovariance P_new;
    P_new.noalias() = jac.Fd * P_ * jac.Fd.transpose() 
                    + jac.Gc * Qd_ * jac.Gc.transpose();
    
    // 4. If no image received yet OR pbar is frozen (timeout), skip pbar propagation
    // This prevents pbar covariance from exploding when unobserved
    if (!first_image_received_ || pbar_frozen_) {
        // Zero off-diagonal correlations between pbar and other states
        // Error state is 20D: [δθ(3), δp(3), δv(3), δpbar(2), δbgyr(3), δbacc(3), δbmag(3)]
        // pbar is at indices 9-10
        
        // Rows 9-10 (pbar): zero columns 0-8 (before pbar) and 11-19 (after pbar)
        P_new.block<2,9>(error_idx::DPBAR_START, 0).setZero();  // pbar row, before pbar
        P_new.block<2,9>(error_idx::DPBAR_START, error_idx::DPBAR_START + 2).setZero();  // pbar row, after pbar (bgyr+bacc+bmag=9)
        
        // Columns 9-10 (pbar): zero rows 0-8 and 11-19
        P_new.block<9,2>(0, error_idx::DPBAR_START).setZero();  // before pbar, pbar col
        P_new.block<9,2>(error_idx::DPBAR_START + 2, error_idx::DPBAR_START).setZero();  // after pbar (bgyr+bacc+bmag=9), pbar col
        
        // Restore pbar diagonal covariance to its initial/previous value
        P_new.block<2,2>(error_idx::DPBAR_START, error_idx::DPBAR_START) = 
            P_.block<2,2>(error_idx::DPBAR_START, error_idx::DPBAR_START);
        
        // Also keep pbar state frozen in nominal state
        x_new.segment<2>(nominal_idx::PBAR_START) = x_.segment<2>(nominal_idx::PBAR_START);
    }
    
    // Update state and covariance
    x_ = x_new;
    P_ = P_new;
    
    // Clamp pbar covariance to prevent numerical issues
    // clampPbarCovariance();
    
    // Update history buffer
    IMUMeasurement imu(omega_meas, a_meas, timestamp);
    updateHistory(imu);
}

NominalState ErrorStateKalmanFilter::predictNominalState(
    const NominalState& x,
    const Vector3d& omega_meas,
    const Vector3d& a_meas,
    double dt) const 
{
    using namespace math;
    
    // Extract current state
    const Quaterniond q = state_access::getQuaternion(x);
    const Vector3d p_r = state_access::getPosition(x);
    const Vector3d v_r = state_access::getVelocity(x);
    const Vector2d pbar = state_access::getPbar(x);
    const Vector3d b_gyr = state_access::getGyroBias(x);
    const Vector3d b_acc = state_access::getAccelBias(x);
    const Vector3d b_mag = state_access::getMagBias(x);
    
    // Corrected IMU measurements
    const Vector3d omega = omega_meas - b_gyr;
    const Vector3d a_body = a_meas - b_acc;
    
    // Rotation matrix (body to earth)
    const RotationMatrix R_b2e = quaternionToRotation(q);
    
    // === Quaternion update using exponential map ===
    const Vector3d omega_dt = omega * dt;
    const Quaterniond dq = expQuaternion(omega_dt);
    Quaterniond q_new = quaternionMultiply(q, dq);
    q_new = normalizeQuaternion(q_new);
    
    // === Velocity update ===
    const Vector3d a_world = R_b2e * a_body + gravity_ned_;
    const Vector3d v_r_new = v_r + a_world * dt;
    
    // === Position update (trapezoidal) ===
    const Vector3d p_r_new = p_r + 0.5 * (v_r + v_r_new) * dt;
    
    // === Image feature update ===
    const RotationMatrix R_e2b = R_b2e.transpose();
    const Vector3d p_c = params_.R_b2c * R_e2b * (-p_r);
    const double p_c_z = std::max(p_c(2), constants::MIN_DEPTH);
    
    const Vector3d v_c = params_.R_b2c * R_e2b * v_r;
    const Vector3d w_c = params_.R_b2c * omega;
    
    const double pbar_x = pbar(0);
    const double pbar_y = pbar(1);
    
    const auto Lv = computeLv(pbar_x, pbar_y, p_c_z);
    const auto Lw = computeLw(pbar_x, pbar_y);
    
    const Vector2d pbar_dot = Lv * v_c + Lw * w_c;
    const Vector2d pbar_new = pbar + pbar_dot * dt;
    
    // === Biases don't change in nominal propagation ===
    const Vector3d b_gyr_new = b_gyr;
    const Vector3d b_acc_new = b_acc;
    const Vector3d b_mag_new = b_mag;
    
    // Assemble new state
    NominalState x_new;
    state_access::setQuaternion(x_new, q_new);
    x_new.segment<3>(nominal_idx::PR_START) = p_r_new;
    x_new.segment<3>(nominal_idx::VR_START) = v_r_new;
    x_new.segment<2>(nominal_idx::PBAR_START) = pbar_new;
    x_new.segment<3>(nominal_idx::BGYR_START) = b_gyr_new;
    x_new.segment<3>(nominal_idx::BACC_START) = b_acc_new;
    x_new.segment<3>(nominal_idx::BMAG_START) = b_mag_new;  // Propagate mag bias unchanged
    
    return x_new;
}

// ============================================================================
// Image Correction (with delay)
// ============================================================================

Vector2d ErrorStateKalmanFilter::correctImage(const Vector2d& z_pbar, 
                                               int delay_steps) {
    Vector2d innovation = Vector2d::Zero();
    
    // Check valid delay index
    const int idx_delayed = static_cast<int>(history_.size()) - delay_steps - 1;
    if (idx_delayed < 0 || idx_delayed >= static_cast<int>(history_.size())) {
        return innovation;  // Invalid delay, return empty
    }
    
    // Get prior estimates from history
    const NominalState& x_prior = history_[idx_delayed].x;
    const ErrorCovariance& P_prior = history_[idx_delayed].P;
    
    // Predicted measurement (pbar from state)
    const Vector2d z_pred = state_access::getPbar(x_prior);
    
    // Innovation
    innovation = z_pbar - z_pred;
    
    // Innovation covariance
    const Eigen::Matrix<double, 20, 2> PH_T = P_prior * H_img_.transpose();
    const Eigen::Matrix2d S = H_img_ * PH_T + R_img_;
    
    // === Mahalanobis Distance Chi-Square Gating ===
    // d² = y' * S^(-1) * y follows chi-square with 2 DoF
    const double d_mahal_sq = innovation.transpose() * S.inverse() * innovation;
    
    if (params_.enable_false_detection_image && d_mahal_sq > params_.chi2_threshold_image) {
        PRINT_WARNING(YELLOW "[IMAGE]: False detection rejected (Mahalanobis d²=%.2f > %.2f)" RESET "\n",
                    d_mahal_sq, params_.chi2_threshold_image)
        return Vector2d::Zero();
    }
    
    // Kalman gain
    const Eigen::Matrix<double, 20, 2> K = PH_T * S.inverse();
    
    // Error state correction
    const ErrorState delta_x = K * innovation;
    
    // Inject error into nominal state
    const NominalState x_corrected = injectErrorState(x_prior, delta_x);
    
    // Covariance update (Joseph form for numerical stability)
    const StateJacobian I_KH = StateJacobian::Identity() - K * H_img_;
    ErrorCovariance P_corrected = I_KH * P_prior * I_KH.transpose() 
                                 + K * R_img_ * K.transpose();
    
    // ESKF reset
    P_corrected = resetCovariance(P_corrected, delta_x);
    
    // Re-propagate to current time
    repropagate(x_corrected, P_corrected, idx_delayed);
    
    // Mark that first image has been received (pbar now observable)
    if (!first_image_received_) {
        first_image_received_ = true;
        PRINT_INFO(BOLDGREEN "[IMAGE]: First image received - pbar propagation enabled" RESET "\n")
    }
    
    return innovation;
}

// ============================================================================
// Radar Correction (no delay)
// ============================================================================

Vector6d ErrorStateKalmanFilter::correctRadar(const Vector6d& z_radar) {
    // Check if we should use velocity measurements
    if (params_.use_vr) {
        // === Full 6D update (position + velocity) ===
        // Predicted measurement [p_r; v_r] from state
        Vector6d z_pred;
        z_pred.head<3>() = state_access::getPosition(x_);
        z_pred.tail<3>() = state_access::getVelocity(x_);
        
        // Innovation
        const Vector6d innovation = z_radar - z_pred;
        
        // Innovation covariance
        const Eigen::Matrix<double, 20, 6> PH_T = P_ * H_radar_.transpose();
        const Eigen::Matrix<double, 6, 6> S = H_radar_ * PH_T + R_radar_;
        
        // === Mahalanobis Distance Chi-Square Gating ===
        const double d_mahal_sq = innovation.transpose() * S.inverse() * innovation;
        
        if (params_.enable_false_detection_radar && d_mahal_sq > params_.chi2_threshold_radar) {
            PRINT_WARNING(YELLOW "[RADAR]: False detection rejected (Mahalanobis d²=%.2f > %.2f)" RESET "\n",
                        d_mahal_sq, params_.chi2_threshold_radar)
            return Vector6d::Zero();
        }
        
        // Kalman gain
        const Eigen::Matrix<double, 20, 6> K = PH_T * S.inverse();
        
        // Error state correction
        const ErrorState delta_x = K * innovation;
        
        // Inject error into nominal state
        x_ = injectErrorState(x_, delta_x);
        
        // Covariance update (Joseph form for 6x6 R)
        const StateJacobian I_KH = StateJacobian::Identity() - K * H_radar_;
        ErrorCovariance P_updated = I_KH * P_ * I_KH.transpose() 
                                   + K * R_radar_ * K.transpose();
        
        // ESKF reset
        P_ = resetCovariance(P_updated, delta_x);
        
        return innovation;
    } else {
        // === Position-only 3D update (use_vr == false) ===
        // Predicted measurement [p_r] from state
        const Vector3d z_pred = state_access::getPosition(x_);
        
        // Innovation (position only)
        const Vector3d innovation_pos = z_radar.head<3>() - z_pred;
        
        // Use only top 3 rows of H_radar_ (position part)
        const Eigen::Matrix<double, 3, 20> H_pos = H_radar_.topRows<3>();
        
        // Use only top-left 3x3 block of R_radar_ (position noise)
        const Eigen::Matrix3d R_pos = R_radar_.topLeftCorner<3, 3>();
        
        // Innovation covariance
        const Eigen::Matrix<double, 20, 3> PH_T = P_ * H_pos.transpose();
        const Eigen::Matrix3d S = H_pos * PH_T + R_pos;
        
        // === Mahalanobis Distance Chi-Square Gating ===
        // Note: 3 DoF for position-only measurement
        const double d_mahal_sq = innovation_pos.transpose() * S.inverse() * innovation_pos;
        
        // Use chi2inv(0.9999, 3) = 16.27 for 3 DoF
        constexpr double chi2_threshold_3dof = 16.27;
        if (params_.enable_false_detection_radar && d_mahal_sq > chi2_threshold_3dof) {
            PRINT_WARNING(YELLOW "[RADAR]: False detection rejected (Mahalanobis d²=%.2f > %.2f)" RESET "\n",
                        d_mahal_sq, chi2_threshold_3dof)
            return Vector6d::Zero();
        }
        
        // Kalman gain
        const Eigen::Matrix<double, 20, 3> K = PH_T * S.inverse();
        
        // Error state correction
        const ErrorState delta_x = K * innovation_pos;
        
        // Inject error into nominal state
        x_ = injectErrorState(x_, delta_x);
        
        // Covariance update (Joseph form for 3x3 R)
        const StateJacobian I_KH = StateJacobian::Identity() - K * H_pos;
        ErrorCovariance P_updated = I_KH * P_ * I_KH.transpose() 
                                   + K * R_pos * K.transpose();
        
        // ESKF reset
        P_ = resetCovariance(P_updated, delta_x);
        
        // Return 6D innovation with zeros for velocity part
        Vector6d innovation_full = Vector6d::Zero();
        innovation_full.head<3>() = innovation_pos;
        return innovation_full;
    }
}

// ============================================================================
// Magnetometer Correction (NORMALIZED - unit vectors, dimensionless)
// ============================================================================

Vector3d ErrorStateKalmanFilter::correctMag(const Vector3d& z_mag) {
    // z_mag should be a unit vector (normalized by caller)
    // B_ned should also be a unit vector (from config)
    
    // Compute attitude-dependent Jacobian
    const MagJacobian H_mag = computeMagJacobian(x_, params_.B_ned);
    
    // Predicted measurement: R_e2b * B_ned - b_mag (all dimensionless)
    const RotationMatrix R_b2e = math::quaternionToRotation(getQuaternion());
    const RotationMatrix R_e2b = R_b2e.transpose();
    const Vector3d z_pred = R_e2b * params_.B_ned - getMagBias();
    
    // Innovation
    const Vector3d innovation = z_mag - z_pred;
    
    // Innovation covariance
    const Eigen::Matrix<double, 20, 3> PH_T = P_ * H_mag.transpose();
    const MagNoise S = H_mag * PH_T + R_mag_;
    
    // === Mahalanobis Distance Chi-Square Gating ===
    // d² = y' * S^(-1) * y follows chi-square with 3 DoF
    const double d_mahal_sq = innovation.transpose() * S.inverse() * innovation;
    
    if (params_.enable_false_detection_mag && d_mahal_sq > params_.chi2_threshold_mag) {
        PRINT_WARNING(YELLOW "[MAG]: False detection rejected (Mahalanobis d²=%.2f > %.2f)" RESET "\n",
                    d_mahal_sq, params_.chi2_threshold_mag)
        return Vector3d::Zero();
    }
    
    // Kalman gain
    const Eigen::Matrix<double, 20, 3> K = PH_T * S.inverse();
    
    // Error state correction
    const ErrorState delta_x = K * innovation;
    
    // Inject error into nominal state
    x_ = injectErrorState(x_, delta_x);
    
    // Covariance update (Joseph form)
    const StateJacobian I_KH = StateJacobian::Identity() - K * H_mag;
    ErrorCovariance P_updated = I_KH * P_ * I_KH.transpose() 
                               + K * R_mag_ * K.transpose();
    
    // ESKF reset
    P_ = resetCovariance(P_updated, delta_x);
    
    return innovation;
}

// ============================================================================
// ZUPT (Zero Velocity Update)
// ============================================================================

bool ErrorStateKalmanFilter::detectZUPT(const Vector3d& omega_meas, 
                                         const Vector3d& a_meas) {
    if (!zupt_enabled_) {
        return false;
    }
    
    // Get current state estimates
    const RotationMatrix R_b2e = math::quaternionToRotation(getQuaternion());
    const Vector3d b_gyr = getGyroBias();
    const Vector3d b_acc = getAccelBias();
    
    // Innovation: y = 0 - predicted_meas = -(meas - bias + R'g)
    Vector6d z_tilde;
    z_tilde.head<3>() = -(a_meas - b_acc + R_b2e.transpose() * gravity_ned_);
    z_tilde.tail<3>() = -(omega_meas - b_gyr);
    
    // Compute ZUPT Jacobian
    const ZUPTJacobian H = computeZUPTJacobian(x_, gravity_ned_);
    
    // Chi-square test: z' * (H*P*H' + α*R)^-1 * z < χ²
    const ZUPTNoise S = H * P_ * H.transpose() + params_.zupt_alpha * R_zupt_;
    const double chi2 = z_tilde.transpose() * S.inverse() * z_tilde;
    
    // Diagnostic logging (throttled by caller)
    static int debug_counter = 0;
    if (++debug_counter % 200 == 0) {  // Log every ~1 second at 200Hz
        PRINT_DEBUG(CYAN "[ZUPT]: chi2=%.2f (thresh=%.2f), accel=[%.3f,%.3f,%.3f], gyro=[%.4f,%.4f,%.4f]" RESET "\n",
                    chi2, params_.chi2_threshold_zupt,
                    z_tilde(0), z_tilde(1), z_tilde(2),
                    z_tilde(3), z_tilde(4), z_tilde(5))
    }
    
    return chi2 < params_.chi2_threshold_zupt;
}

Vector6d ErrorStateKalmanFilter::correctZUPT(const Vector3d& omega_meas, 
                                              const Vector3d& a_meas) {
    if (!zupt_enabled_) {
        return Vector6d::Zero();
    }
    
    // Get current state estimates
    const RotationMatrix R_b2e = math::quaternionToRotation(getQuaternion());
    const Vector3d b_gyr = getGyroBias();
    const Vector3d b_acc = getAccelBias();
    
    // Innovation: y = 0 - predicted_meas = -(meas - bias + R'g)
    Vector6d innovation;
    innovation.head<3>() = -(a_meas - b_acc + R_b2e.transpose() * gravity_ned_);
    innovation.tail<3>() = -(omega_meas - b_gyr);
    
    // Compute ZUPT Jacobian (depends on current attitude)
    const ZUPTJacobian H = computeZUPTJacobian(x_, gravity_ned_);
    
    // Innovation covariance (use non-inflated noise for update)
    const Eigen::Matrix<double, 20, 6> PH_T = P_ * H.transpose();
    const ZUPTNoise S = H * PH_T + R_zupt_;
    
    // Kalman gain
    const Eigen::Matrix<double, 20, 6> K = PH_T * S.inverse();
    
    // Error state correction
    const ErrorState delta_x = K * innovation;
    
    // Inject error into nominal state
    x_ = injectErrorState(x_, delta_x);
    
    // Covariance update (Joseph form)
    const StateJacobian I_KH = StateJacobian::Identity() - K * H;
    ErrorCovariance P_updated = I_KH * P_ * I_KH.transpose() 
                               + K * R_zupt_ * K.transpose();
    
    // ESKF reset
    P_ = resetCovariance(P_updated, delta_x);
    
    // Mark that ZUPT was triggered at least once
    zupt_triggered_once_ = true;
    
    return innovation;
}

void ErrorStateKalmanFilter::disableZUPT() {
    zupt_enabled_ = false;
}

void ErrorStateKalmanFilter::notifyRadarReceived() {
    radar_received_ = true;
}

// ============================================================================
// State Injection
// ============================================================================

NominalState ErrorStateKalmanFilter::injectErrorState(
    const NominalState& x_nominal,
    const ErrorState& delta_x) const 
{
    using namespace math;
    
    NominalState x_corrected = x_nominal;
    
    // === Attitude: q_corrected = q_nominal ⊗ δq(δθ) ===
    const Vector3d delta_theta = delta_x.segment<3>(error_idx::DTHETA_START);
    const Quaterniond dq = expQuaternion(delta_theta);
    const Quaterniond q_nom = state_access::getQuaternion(x_nominal);
    Quaterniond q_corrected = quaternionMultiply(q_nom, dq);
    q_corrected = normalizeQuaternion(q_corrected);
    state_access::setQuaternion(x_corrected, q_corrected);
    
    // === Position: additive ===
    x_corrected.segment<3>(nominal_idx::PR_START) += 
        delta_x.segment<3>(error_idx::DPR_START);
    
    // === Velocity: additive ===
    x_corrected.segment<3>(nominal_idx::VR_START) += 
        delta_x.segment<3>(error_idx::DVR_START);
    
    // === Image features: additive ===
    x_corrected.segment<2>(nominal_idx::PBAR_START) += 
        delta_x.segment<2>(error_idx::DPBAR_START);
    
    // === Biases: additive ===
    x_corrected.segment<3>(nominal_idx::BGYR_START) += 
        delta_x.segment<3>(error_idx::DBGYR_START);
    x_corrected.segment<3>(nominal_idx::BACC_START) += 
        delta_x.segment<3>(error_idx::DBACC_START);
    x_corrected.segment<3>(nominal_idx::BMAG_START) += 
        delta_x.segment<3>(error_idx::DBMAG_START);
    
    return x_corrected;
}

// ============================================================================
// Covariance Reset
// ============================================================================

ErrorCovariance ErrorStateKalmanFilter::resetCovariance(
    const ErrorCovariance& P,
    const ErrorState& delta_x) const 
{
    // G = blkdiag(I₃ - ½[δθ]×, I₃, I₃, I₂, I₃, I₃)
    // For small δθ, G ≈ I₁₇
    
    const Vector3d delta_theta = delta_x.segment<3>(error_idx::DTHETA_START);
    
    StateJacobian G = StateJacobian::Identity();
    G.block<3,3>(0, 0) = Eigen::Matrix3d::Identity() - 0.5 * math::skew(delta_theta);
    
    return G * P * G.transpose();
}

// ============================================================================
// Re-propagation
// ============================================================================

void ErrorStateKalmanFilter::repropagate(const NominalState& x_start,
                                          const ErrorCovariance& P_start,
                                          size_t idx_start) {
    NominalState x_reprop = x_start;
    ErrorCovariance P_reprop = P_start;
    
    for (size_t j = idx_start; j < history_.size() - 1; ++j) {
        const Vector3d& omega_meas = history_[j].imu.omega;
        const Vector3d& a_meas = history_[j].imu.accel;
        
        // Propagate nominal state
        x_reprop = predictNominalState(x_reprop, omega_meas, a_meas, params_.dt_eskf);
        
        // Propagate covariance
        ESKFJacobians jac = computeESKFJacobians(x_reprop, omega_meas, a_meas,
                                                  params_.dt_eskf, params_.R_b2c);
        P_reprop.noalias() = jac.Fd * P_reprop * jac.Fd.transpose() 
                           + jac.Gc * Qd_ * jac.Gc.transpose();
    }
    
    // Update current state
    x_ = x_reprop;
    P_ = P_reprop;
}

// ============================================================================
// History Management
// ============================================================================

void ErrorStateKalmanFilter::updateHistory(const IMUMeasurement& imu) {
    // Shift history (circular buffer using deque)
    if (history_.size() >= static_cast<size_t>(params_.history_length)) {
        history_.pop_front();
    }
    
    // Add current state
    StateHistoryEntry entry;
    entry.x = x_;
    entry.P = P_;
    entry.imu = imu;
    entry.timestamp = current_time_;
    history_.push_back(entry);
}

// ============================================================================
// Measurement Matrices
// ============================================================================

ImageJacobian ErrorStateKalmanFilter::getImageMeasurementMatrix() const {
    // H_img: z = pbar, so H maps δpbar (indices 9-10 in error state)
    ImageJacobian H = ImageJacobian::Zero();
    H.block<2,2>(0, error_idx::DPBAR_START) = Eigen::Matrix2d::Identity();
    return H;
}

RadarJacobian ErrorStateKalmanFilter::getRadarMeasurementMatrix() const {
    // H_radar: z = [p_r; v_r], so H maps [δpr; δvr]
    // 6x17 Jacobian: first 3 rows for position, next 3 for velocity
    RadarJacobian H = RadarJacobian::Zero();
    H.block<3,3>(0, error_idx::DPR_START) = Eigen::Matrix3d::Identity();  // Position
    H.block<3,3>(3, error_idx::DVR_START) = Eigen::Matrix3d::Identity();  // Velocity
    return H;
}

// ============================================================================
// Initial Covariance
// ============================================================================

ErrorCovariance ErrorStateKalmanFilter::createInitialCovariance() const {
    ErrorCovariance P = ErrorCovariance::Zero();
    
    // δθ - attitude
    P.block<3,3>(error_idx::DTHETA_START, error_idx::DTHETA_START) = 
        params_.init_sigma_attitude * params_.init_sigma_attitude * Eigen::Matrix3d::Identity();
    
    // δpr - position
    P.block<3,3>(error_idx::DPR_START, error_idx::DPR_START) = 
        params_.init_sigma_position * params_.init_sigma_position * Eigen::Matrix3d::Identity();
    
    // δvr - velocity
    P.block<3,3>(error_idx::DVR_START, error_idx::DVR_START) = 
        params_.init_sigma_velocity * params_.init_sigma_velocity * Eigen::Matrix3d::Identity();
    
    // δpbar - image features
    P.block<2,2>(error_idx::DPBAR_START, error_idx::DPBAR_START) = 
        params_.init_sigma_pbar * params_.init_sigma_pbar * Eigen::Matrix2d::Identity();
    
    // δbgyr - gyro bias
    P.block<3,3>(error_idx::DBGYR_START, error_idx::DBGYR_START) = 
        params_.init_sigma_bgyr * params_.init_sigma_bgyr * Eigen::Matrix3d::Identity();
    
    // δbacc - accel bias
    P.block<3,3>(error_idx::DBACC_START, error_idx::DBACC_START) = 
        params_.init_sigma_bacc * params_.init_sigma_bacc * Eigen::Matrix3d::Identity();
    
    // δbmag - mag bias
    P.block<3,3>(error_idx::DBMAG_START, error_idx::DBMAG_START) = 
        params_.init_sigma_bmag * params_.init_sigma_bmag * Eigen::Matrix3d::Identity();
    
    return P;
}

// ============================================================================
// Pbar Covariance Clamping
// ============================================================================

void ErrorStateKalmanFilter::clampPbarCovariance() {
    constexpr double MAX_PBAR_COVARIANCE = 10.0;  // Maximum allowed pbar variance
    constexpr double MIN_PBAR_COVARIANCE = 1e-6;  // Minimum to reset NaN/negative
    
    // Clamp pbar covariance diagonal elements (indices 9 and 10 in error state)
    for (int i = error_idx::DPBAR_START; i < error_idx::DPBAR_START + error_idx::DPBAR_SIZE; ++i) {
        if (std::isnan(P_(i, i))) {
            PRINT_WARNING(BOLDRED "[ESKF]: NaN in pbar covariance P(%d,%d)! Reset to %.2e" RESET "\n",
                        i, i, MIN_PBAR_COVARIANCE)
            P_(i, i) = MIN_PBAR_COVARIANCE;
        }
        if (P_(i, i) < 0) {
            PRINT_WARNING(BOLDRED "[ESKF]: Negative variance P(%d,%d)=%.2e! Taking abs" RESET "\n",
                        i, i, P_(i, i))
            P_(i, i) = std::abs(P_(i, i));
        }
        // Prevent unbounded growth
        if (P_(i, i) > MAX_PBAR_COVARIANCE) {
            PRINT_WARNING(YELLOW "[ESKF]: Pbar P(%d,%d)=%.2f clamped to %.2f" RESET "\n",
                        i, i, P_(i, i), MAX_PBAR_COVARIANCE)
            P_(i, i) = MAX_PBAR_COVARIANCE;
            
            // Zero out off-diagonal elements in row i and column i to decorrelate pbar
            for (int j = 0; j < P_.cols(); ++j) {
                if (j != i) {
                    P_(i, j) = 0.0;
                    P_(j, i) = 0.0;
                }
            }
        }
    }
}

// ============================================================================
// Static Helper Functions
// ============================================================================

NominalState createInitialState(
    const Quaterniond& q_true,
    const Vector3d& p_r_true,
    const Vector3d& v_r_true,
    const Vector2d& pbar_true,
    const Vector3d& euler_error_deg,
    const Vector3d& pos_error,
    const Vector3d& vel_error,
    const Vector2d& pbar_error) 
{
    using namespace math;
    
    NominalState x;
    
    // Apply attitude error
    if (euler_error_deg.norm() > 0) {
        const Vector3d euler_rad = euler_error_deg * M_PI / 180.0;
        // Create rotation from Euler angles (ZYX convention)
        Eigen::AngleAxisd roll(euler_rad(2), Vector3d::UnitX());
        Eigen::AngleAxisd pitch(euler_rad(1), Vector3d::UnitY());
        Eigen::AngleAxisd yaw(euler_rad(0), Vector3d::UnitZ());
        Quaterniond dq = yaw * pitch * roll;
        Quaterniond q_est = quaternionMultiply(q_true, dq);
        q_est = normalizeQuaternion(q_est);
        state_access::setQuaternion(x, q_est);
    } else {
        state_access::setQuaternion(x, q_true);
    }
    
    // Apply other errors (additive)
    x.segment<3>(nominal_idx::PR_START) = p_r_true + pos_error;
    x.segment<3>(nominal_idx::VR_START) = v_r_true + vel_error;
    x.segment<2>(nominal_idx::PBAR_START) = pbar_true + pbar_error;
    
    // Initialize biases to zero
    x.segment<3>(nominal_idx::BGYR_START).setZero();
    x.segment<3>(nominal_idx::BACC_START).setZero();
    
    return x;
}

Vector2d computeImageFeatures(const Vector3d& p_r,
                               const Quaterniond& q,
                               const RotationMatrix& R_b2c) {
    const RotationMatrix R_b2e = math::quaternionToRotation(q);
    const RotationMatrix R_e2b = R_b2e.transpose();
    
    // p_c = R_b2c * R_e2b * (-p_r)
    const Vector3d p_c = R_b2c * R_e2b * (-p_r);
    const double p_c_z = std::max(p_c(2), constants::MIN_DEPTH);
    
    // Normalized image coordinates
    return Vector2d(p_c(0) / p_c_z, p_c(1) / p_c_z);
}

} // namespace eskf
