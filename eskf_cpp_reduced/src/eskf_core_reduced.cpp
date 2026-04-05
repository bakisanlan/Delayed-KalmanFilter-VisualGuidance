/**
 * @file eskf_core_reduced.cpp
 * @brief Reduced-state ESKF core implementation.
 */

#include "eskf_cpp_reduced/eskf_core_reduced.hpp"
#include "eskf_cpp_reduced/utils/print.hpp"

#include <algorithm>
#include <cmath>

namespace eskf::reduced {

ReducedErrorStateKalmanFilter::ReducedErrorStateKalmanFilter(const ESKFParams& params)
    : params_(params) {
    samples_per_eskf_ = std::max(1, static_cast<int>(std::round(params_.dt_eskf / params_.dt_imu)));

    Qd_ = math::computeReducedDiscreteProcessNoise(params_);

    R_img_ = params_.sigma_img * params_.sigma_img * ImageNoise::Identity();
    R_radar_.setZero();
    R_radar_.block<3, 3>(0, 0) =
        params_.sigma_radar_pos * params_.sigma_radar_pos * Eigen::Matrix3d::Identity();
    R_radar_.block<3, 3>(3, 3) =
        params_.sigma_radar_vel * params_.sigma_radar_vel * Eigen::Matrix3d::Identity();

    H_img_ = getImageMeasurementMatrix();
    H_radar_ = getRadarMeasurementMatrix();
    P_ = createInitialCovariance();

    history_.assign(std::max(1, params_.history_length), StateHistoryEntry());
}

void ReducedErrorStateKalmanFilter::reset(const NominalState& x_init,
                                          const ErrorCovariance& P_init) {
    x_ = x_init;
    P_ = P_init;
    current_time_ = 0.0;
    omega_accum_.setZero();
    imu_count_ = 0;

    history_.assign(std::max(1, params_.history_length), StateHistoryEntry());
    for (auto& entry : history_) {
        entry.x = x_;
        entry.P = P_;
        entry.timestamp = current_time_;
    }
}

void ReducedErrorStateKalmanFilter::accumulateIMU(const Vector3d& omega) {
    omega_accum_ += omega;
    imu_count_++;
}

bool ReducedErrorStateKalmanFilter::isReadyForPrediction() const {
    return imu_count_ >= samples_per_eskf_;
}

IMUMeasurement ReducedErrorStateKalmanFilter::getAveragedIMU(double timestamp) {
    IMUMeasurement avg;
    if (imu_count_ > 0) {
        avg.omega = omega_accum_ / static_cast<double>(imu_count_);
    }
    avg.timestamp = timestamp;

    omega_accum_.setZero();
    imu_count_ = 0;

    return avg;
}

void ReducedErrorStateKalmanFilter::predict(const Vector3d& omega_meas,
                                            const InterceptorState& interceptor,
                                            double timestamp) {
    current_time_ = timestamp;

    NominalState x_new = predictNominalState(x_, omega_meas, interceptor, params_.dt_eskf);
    const ReducedESKFJacobians jac = computeReducedESKFJacobians(
        x_, omega_meas, params_.dt_eskf, params_.R_b2c,
        interceptor.R_b2e, interceptor.position_ned, interceptor.velocity_ned);

    ErrorCovariance P_new;
    P_new.noalias() = jac.Fd * P_ * jac.Fd.transpose() + jac.Gc * Qd_ * jac.Gc.transpose();

    if (!first_image_received_ || pbar_frozen_) {
        P_new.block<2, 6>(error_idx::DPBAR_START, 0).setZero();
        P_new.block<6, 2>(0, error_idx::DPBAR_START).setZero();
        P_new.block<2, 2>(error_idx::DPBAR_START, error_idx::DPBAR_START) =
            P_.block<2, 2>(error_idx::DPBAR_START, error_idx::DPBAR_START);
        x_new.segment<2>(nominal_idx::PBAR_START) = x_.segment<2>(nominal_idx::PBAR_START);
    }

    x_ = x_new;
    P_ = P_new;

    updateHistory(IMUMeasurement(omega_meas, timestamp), interceptor);
}

NominalState ReducedErrorStateKalmanFilter::predictNominalState(
    const NominalState& x,
    const Vector3d& omega_meas,
    const InterceptorState& interceptor,
    double dt) const {
    using namespace state_access;
    using namespace math;

    const Vector3d p_t = getPosition(x);
    const Vector3d v_t = getVelocity(x);
    const Vector2d pbar = state_access::getPbar(x);

    const Vector3d p_t_new = p_t + v_t * dt;
    const Vector3d v_t_new = v_t;

    const RotationMatrix R_e2b = interceptor.R_b2e.transpose();
    const Vector3d p_c = params_.R_b2c * R_e2b * (-(interceptor.position_ned - p_t));
    const double p_c_z = math::boundDepth(p_c(2));

    const Vector3d v_c = params_.R_b2c * R_e2b * (interceptor.velocity_ned - v_t);
    const Vector3d w_c = params_.R_b2c * omega_meas;

    const auto Lv = computeLv(pbar(0), pbar(1), p_c_z);
    const auto Lw = computeLw(pbar(0), pbar(1));
    const Vector2d pbar_dot = Lv * v_c + Lw * w_c;
    const Vector2d pbar_new = pbar + pbar_dot * dt;

    NominalState x_new = NominalState::Zero();
    x_new.segment<3>(nominal_idx::PT_START) = p_t_new;
    x_new.segment<3>(nominal_idx::VT_START) = v_t_new;
    x_new.segment<2>(nominal_idx::PBAR_START) = pbar_new;
    return x_new;
}

Vector2d ReducedErrorStateKalmanFilter::correctImage(const Vector2d& z_pbar,
                                                     int delay_steps) {
    const int idx_delayed = static_cast<int>(history_.size()) - delay_steps - 1;
    if (idx_delayed < 0 || idx_delayed >= static_cast<int>(history_.size())) {
        return Vector2d::Zero();
    }

    // Dispatch based on delay compensation 'enable_image_extrapolation'
    if (params_.enable_image_extrapolation) {
        return correctImageExtrapolate(z_pbar, idx_delayed);
    }

    // ---- Method 0: No compensation — apply delayed pixel directly -----------
    return correctImageNone(z_pbar);
}

// ---------------------------------------------------------------------------
Vector2d ReducedErrorStateKalmanFilter::correctImageNone(const Vector2d& z_pbar) {
    // Method 0: No Delay Compensation (Naive Baseline)
    //
    // The delayed pixel z_d = pbar(k-D) is applied directly to the CURRENT
    // state (x_, P_) as if it arrived without any delay.  No history lookup,
    // no re-propagation, no noise inflation.
    //

    const Vector2d z_pred = state_access::getPbar(x_);
    Vector2d innovation   = z_pbar - z_pred;

    const Eigen::Matrix<double, 8, 2> PH_T = P_ * H_img_.transpose();
    const ImageNoise S = H_img_ * PH_T + R_img_;
    const double d_mahal_sq = innovation.transpose() * S.inverse() * innovation;

    if (params_.enable_false_detection_image && d_mahal_sq > params_.chi2_threshold_image) {
        PRINT_WARNING(YELLOW "[IMAGE-NOCOMP]: False detection rejected (d^2=%.2f > %.2f)" RESET "\n",
                      d_mahal_sq, params_.chi2_threshold_image)
        return Vector2d::Zero();
    }

    const Eigen::Matrix<double, 8, 2> K = PH_T * S.inverse();
    x_ = injectErrorState(x_, K * innovation);

    const StateJacobian I_KH = StateJacobian::Identity() - K * H_img_;
    P_ = resetCovariance(I_KH * P_ * I_KH.transpose() + K * R_img_ * K.transpose());

    if (!first_image_received_) {
        first_image_received_ = true;
        pbar_frozen_ = false;
        PRINT_INFO(BOLDGREEN "[IMAGE-NOCOMP]: First image received - pbar propagation enabled" RESET "\n")
    }

    return innovation;
}

// ---------------------------------------------------------------------------
Vector2d ReducedErrorStateKalmanFilter::correctImageExtrapolate(const Vector2d& z_pbar,
                                                                int idx_delayed) {
    // Method 2: Measurement Extrapolation (Forward Prediction)
    //
    // Integrate the IBVS dynamics over the stored history from idx_delayed
    // to the end to forward-predict z_d = pbar(k-D) to current time k.
    //
    // Simultaneously accumulate the linearised pbar propagation matrix Phi
    // so the inflated noise R_ext = Phi * R_img * Phi^T captures the
    // growing uncertainty introduced by the extrapolation.
    using namespace math;

    Vector2d z_ext = z_pbar;          // forward-predicted measurement
    Eigen::Matrix2d Phi = Eigen::Matrix2d::Identity();   // noise propagation

    const int N = static_cast<int>(history_.size());
    for (int j = idx_delayed; j < N - 1; ++j) {
        const Vector3d& omega_j      = history_[j].imu.omega;
        const InterceptorState& ic_j = history_[j].interceptor;
        const NominalState& x_j      = history_[j].x;

        const double pbar_x = x_j(nominal_idx::PBAR_START);
        const double pbar_y = x_j(nominal_idx::PBAR_START + 1);
        const Vector3d p_t_j = x_j.segment<3>(nominal_idx::PT_START);
        const Vector3d v_t_j = x_j.segment<3>(nominal_idx::VT_START);

        // Camera-frame quantities at history step j
        const RotationMatrix R_e2b_j = ic_j.R_b2e.transpose();
        const Vector3d p_c_j  = params_.R_b2c * R_e2b_j * (-(ic_j.position_ned - p_t_j));
        const double   p_c_z  = boundDepth(p_c_j(2));
        const Vector3d v_c_j  = params_.R_b2c * R_e2b_j * (ic_j.velocity_ned - v_t_j);
        const Vector3d omega_c_j = params_.R_b2c * omega_j;

        // IBVS interaction matrices
        const auto Lv_j = computeLv(pbar_x, pbar_y, p_c_z);
        const auto Lw_j = computeLw(pbar_x, pbar_y);

        // Euler-integrate the measurement forward by one step
        z_ext += (Lv_j * v_c_j + Lw_j * omega_c_j) * params_.dt_eskf;

        // Linearised pbar propagation: F_j = I + A_pbar_j * dt
        const Eigen::Matrix2d A_pbar_j = computeApbar(pbar_x, pbar_y, v_c_j, omega_c_j, p_c_z);
        const Eigen::Matrix2d F_j = Eigen::Matrix2d::Identity() + A_pbar_j * params_.dt_eskf;
        Phi = F_j * Phi;    // accumulate: Phi_D = F_{D-1} * ... * F_0
    }

    // Inflated noise: R_ext = Phi * R_img * Phi^T
    const ImageNoise R_ext = Phi * R_img_ * Phi.transpose();

    // Standard present-time KF update on (x_, P_)
    const Vector2d z_pred = state_access::getPbar(x_);
    Vector2d innovation   = z_ext - z_pred;

    const Eigen::Matrix<double, 8, 2> PH_T = P_ * H_img_.transpose();
    const ImageNoise S = H_img_ * PH_T + R_ext;
    const double d_mahal_sq = innovation.transpose() * S.inverse() * innovation;

    if (params_.enable_false_detection_image && d_mahal_sq > params_.chi2_threshold_image) {
        PRINT_WARNING(YELLOW "[IMAGE-EXTRAP]: False detection rejected (d^2=%.2f > %.2f)" RESET "\n",
                      d_mahal_sq, params_.chi2_threshold_image)
        return Vector2d::Zero();
    }

    const Eigen::Matrix<double, 8, 2> K = PH_T * S.inverse();
    const ErrorState delta_x = K * innovation;
    x_ = injectErrorState(x_, delta_x);

    const StateJacobian I_KH = StateJacobian::Identity() - K * H_img_;
    P_ = resetCovariance(I_KH * P_ * I_KH.transpose() + K * R_ext * K.transpose());

    if (!first_image_received_) {
        first_image_received_ = true;
        pbar_frozen_ = false;
        PRINT_INFO(BOLDGREEN "[IMAGE-EXTRAP]: First image received - pbar propagation enabled" RESET "\n")
    }

    return innovation;
}


RadarMeasurement ReducedErrorStateKalmanFilter::correctRadar(const RadarMeasurement& z_target,
                                                             const RadarNoise& radar_noise) {
    R_radar_ = radar_noise;

    if (params_.use_vr) {
        RadarMeasurement z_pred = RadarMeasurement::Zero();
        z_pred.head<3>() = state_access::getPosition(x_);
        z_pred.tail<3>() = state_access::getVelocity(x_);

        const RadarMeasurement innovation = z_target - z_pred;

        const Eigen::Matrix<double, 8, 6> PH_T = P_ * H_radar_.transpose();
        const RadarNoise S = H_radar_ * PH_T + R_radar_;
        const double d_mahal_sq = innovation.transpose() * S.inverse() * innovation;

        if (params_.enable_false_detection_radar && d_mahal_sq > params_.chi2_threshold_radar_6dof) {
            PRINT_WARNING(YELLOW "[RADAR-RED]: False detection rejected (d^2=%.2f > %.2f)" RESET "\n",
                          d_mahal_sq, params_.chi2_threshold_radar_6dof)
            PRINT_WARNING(YELLOW "             Innovation pos=[%.2f, %.2f, %.2f] vel=[%.2f, %.2f, %.2f]" RESET "\n",
                          innovation(0), innovation(1), innovation(2),
                          innovation(3), innovation(4), innovation(5))
            return RadarMeasurement::Zero();
        }

        const Eigen::Matrix<double, 8, 6> K = PH_T * S.inverse();
        const ErrorState delta_x = K * innovation;
        x_ = injectErrorState(x_, delta_x);

        const StateJacobian I_KH = StateJacobian::Identity() - K * H_radar_;
        const ErrorCovariance P_updated =
            I_KH * P_ * I_KH.transpose() + K * R_radar_ * K.transpose();
        P_ = resetCovariance(P_updated);

        return innovation;
    }

    const Vector3d z_pred = state_access::getPosition(x_);
    const Vector3d innovation_pos = z_target.head<3>() - z_pred;

    const Eigen::Matrix<double, 3, 8> H_pos = H_radar_.topRows<3>();
    const Eigen::Matrix3d R_pos = R_radar_.topLeftCorner<3, 3>();
    const Eigen::Matrix<double, 8, 3> PH_T = P_ * H_pos.transpose();
    const Eigen::Matrix3d S = H_pos * PH_T + R_pos;
    const double d_mahal_sq = innovation_pos.transpose() * S.inverse() * innovation_pos;

    if (params_.enable_false_detection_radar && d_mahal_sq > params_.chi2_threshold_radar_3dof) {
        PRINT_WARNING(YELLOW "[RADAR-RED]: False detection rejected (Mahalanobis d^2=%.2f > %.2f)" RESET "\n",
                      d_mahal_sq, params_.chi2_threshold_radar_3dof)
        PRINT_WARNING(YELLOW "             Innovation pos=[%.2f, %.2f, %.2f]" RESET "\n",
                      innovation_pos(0), innovation_pos(1), innovation_pos(2))
        return RadarMeasurement::Zero();
    }

    const Eigen::Matrix<double, 8, 3> K = PH_T * S.inverse();
    const ErrorState delta_x = K * innovation_pos;
    x_ = injectErrorState(x_, delta_x);

    const StateJacobian I_KH = StateJacobian::Identity() - K * H_pos;
    const ErrorCovariance P_updated = I_KH * P_ * I_KH.transpose() + K * R_pos * K.transpose();
    P_ = resetCovariance(P_updated);

    RadarMeasurement innovation = RadarMeasurement::Zero();
    innovation.head<3>() = innovation_pos;
    return innovation;
}

void ReducedErrorStateKalmanFilter::notifyRadarReceived() {
    radar_received_ = true;
}

void ReducedErrorStateKalmanFilter::updatePbarFromGeometry(const InterceptorState& interceptor) {
    using namespace state_access;
    using namespace math;

    const Vector3d p_t = getPosition(x_);

    // 1. Update pbar state directly from geometry
    const Vector2d pbar_geom = computeImageFeatures(p_t, interceptor, params_.R_b2c);
    x_.segment<2>(nominal_idx::PBAR_START) = pbar_geom;

    // 2. Propagate target position covariance into pbar covariance
    const Eigen::Matrix3d P_pos = P_.block<3, 3>(error_idx::DPT_START, error_idx::DPT_START);
    const Vector3d p_c = computeTargetInCameraFrame(p_t, interceptor, params_.R_b2c);
    const Eigen::Matrix2d P_pbar_proj = projectPositionCovarianceToPbar(
        P_pos, p_c, interceptor, params_.R_b2c);

    P_.block<2, 6>(error_idx::DPBAR_START, 0).setZero();
    P_.block<6, 2>(0, error_idx::DPBAR_START).setZero();
    P_.block<2, 2>(error_idx::DPBAR_START, error_idx::DPBAR_START) = P_pbar_proj;
}

NominalState ReducedErrorStateKalmanFilter::injectErrorState(
    const NominalState& x_nominal,
    const ErrorState& delta_x) const {
    NominalState x_corrected = x_nominal;
    x_corrected.segment<3>(nominal_idx::PT_START) += delta_x.segment<3>(error_idx::DPT_START);
    x_corrected.segment<3>(nominal_idx::VT_START) += delta_x.segment<3>(error_idx::DVT_START);
    x_corrected.segment<2>(nominal_idx::PBAR_START) += delta_x.segment<2>(error_idx::DPBAR_START);
    return x_corrected;
}

ErrorCovariance ReducedErrorStateKalmanFilter::resetCovariance(const ErrorCovariance& P) const {
    return P;
}

void ReducedErrorStateKalmanFilter::repropagate(const NominalState& x_start,
                                                const ErrorCovariance& P_start,
                                                size_t idx_start) {
    NominalState x_reprop = x_start;
    ErrorCovariance P_reprop = P_start;

    for (size_t j = idx_start; j < history_.size() - 1; ++j) {
        const Vector3d& omega_meas = history_[j].imu.omega;
        const InterceptorState& interceptor = history_[j].interceptor;

        x_reprop = predictNominalState(x_reprop, omega_meas, interceptor, params_.dt_eskf);

        const ReducedESKFJacobians jac = computeReducedESKFJacobians(
            x_reprop, omega_meas, params_.dt_eskf, params_.R_b2c,
            interceptor.R_b2e, interceptor.position_ned, interceptor.velocity_ned);

        P_reprop.noalias() =
            jac.Fd * P_reprop * jac.Fd.transpose() + jac.Gc * Qd_ * jac.Gc.transpose();
    }

    x_ = x_reprop;
    P_ = P_reprop;
}

void ReducedErrorStateKalmanFilter::updateHistory(const IMUMeasurement& imu,
                                                  const InterceptorState& interceptor) {
    if (history_.size() >= static_cast<size_t>(params_.history_length)) {
        history_.pop_front();
    }

    StateHistoryEntry entry;
    entry.x = x_;
    entry.P = P_;
    entry.imu = imu;
    entry.interceptor = interceptor;
    entry.timestamp = current_time_;
    history_.push_back(entry);
}

ImageJacobian ReducedErrorStateKalmanFilter::getImageMeasurementMatrix() const {
    ImageJacobian H = ImageJacobian::Zero();
    H.block<2, 2>(0, error_idx::DPBAR_START) = Eigen::Matrix2d::Identity();
    return H;
}

RadarJacobian ReducedErrorStateKalmanFilter::getRadarMeasurementMatrix() const {
    RadarJacobian H = RadarJacobian::Zero();
    H.block<3, 3>(0, error_idx::DPT_START) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(3, error_idx::DVT_START) = Eigen::Matrix3d::Identity();
    return H;
}

ErrorCovariance ReducedErrorStateKalmanFilter::createInitialCovariance() const {
    ErrorCovariance P = ErrorCovariance::Zero();
    P.block<3, 3>(error_idx::DPT_START, error_idx::DPT_START) =
        params_.init_sigma_position * params_.init_sigma_position * Eigen::Matrix3d::Identity();
    P.block<3, 3>(error_idx::DVT_START, error_idx::DVT_START) =
        params_.init_sigma_velocity * params_.init_sigma_velocity * Eigen::Matrix3d::Identity();
    P.block<2, 2>(error_idx::DPBAR_START, error_idx::DPBAR_START) =
        params_.init_sigma_pbar * params_.init_sigma_pbar * Eigen::Matrix2d::Identity();
    return P;
}

NominalState createInitialState(const Vector3d& p_t_true,
                                const Vector3d& v_t_true,
                                const Vector2d& pbar_true,
                                const Vector3d& pos_error,
                                const Vector3d& vel_error,
                                const Vector2d& pbar_error) {
    NominalState x = NominalState::Zero();
    x.segment<3>(nominal_idx::PT_START) = p_t_true + pos_error;
    x.segment<3>(nominal_idx::VT_START) = v_t_true + vel_error;
    x.segment<2>(nominal_idx::PBAR_START) = pbar_true + pbar_error;
    return x;
}

}  // namespace eskf::reduced
