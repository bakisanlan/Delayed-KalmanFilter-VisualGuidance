/**
 * @file eskf_core.cpp
 * @brief Implementation of Error-State Kalman Filter core class
 * 
 * Direct translation of ErrorStateKalmanFilter.m to C++.
 */

#include "eskf_cpp/eskf_core.hpp"
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
    
    // Setup measurement noise covariances
    R_img_ = params_.sigma_img * params_.sigma_img * Eigen::Matrix2d::Identity();
    R_radar_ = params_.sigma_radar * params_.sigma_radar * Eigen::Matrix3d::Identity();
    
    // Setup fixed measurement matrices
    H_img_ = getImageMeasurementMatrix();
    H_radar_ = getRadarMeasurementMatrix();
    
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
    
    // Update state and covariance
    x_ = x_new;
    P_ = P_new;
    
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
    const Vector3d a_world = R_b2e * a_body + constants::GRAVITY_NED;
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
    
    // Assemble new state
    NominalState x_new;
    state_access::setQuaternion(x_new, q_new);
    x_new.segment<3>(nominal_idx::PR_START) = p_r_new;
    x_new.segment<3>(nominal_idx::VR_START) = v_r_new;
    x_new.segment<2>(nominal_idx::PBAR_START) = pbar_new;
    x_new.segment<3>(nominal_idx::BGYR_START) = b_gyr_new;
    x_new.segment<3>(nominal_idx::BACC_START) = b_acc_new;
    
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
    
    // Kalman gain
    const Eigen::Matrix<double, 17, 2> PH_T = P_prior * H_img_.transpose();
    const Eigen::Matrix2d S = H_img_ * PH_T + R_img_;
    const Eigen::Matrix<double, 17, 2> K = PH_T * S.inverse();
    
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
    
    return innovation;
}

// ============================================================================
// Radar Correction (no delay)
// ============================================================================

Vector3d ErrorStateKalmanFilter::correctRadar(const Vector3d& z_radar) {
    // Predicted measurement (p_r from state)
    const Vector3d z_pred = state_access::getPosition(x_);
    
    // Innovation
    const Vector3d innovation = z_radar - z_pred;
    
    // Kalman gain
    const Eigen::Matrix<double, 17, 3> PH_T = P_ * H_radar_.transpose();
    const Eigen::Matrix3d S = H_radar_ * PH_T + R_radar_;
    const Eigen::Matrix<double, 17, 3> K = PH_T * S.inverse();
    
    // Error state correction
    const ErrorState delta_x = K * innovation;
    
    // Inject error into nominal state
    x_ = injectErrorState(x_, delta_x);
    
    // Covariance update (Joseph form)
    const StateJacobian I_KH = StateJacobian::Identity() - K * H_radar_;
    ErrorCovariance P_updated = I_KH * P_ * I_KH.transpose() 
                               + K * R_radar_ * K.transpose();
    
    // ESKF reset
    P_ = resetCovariance(P_updated, delta_x);
    
    return innovation;
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
    // H_radar: z = p_r, so H maps δpr (indices 3-5 in error state)  
    RadarJacobian H = RadarJacobian::Zero();
    H.block<3,3>(0, error_idx::DPR_START) = Eigen::Matrix3d::Identity();
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
    
    return P;
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
