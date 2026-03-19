/**
 * @file eskf_core_reduced.hpp
 * @brief Reduced-state ESKF core for target position/velocity/pbar estimation.
 */

#pragma once

#include "eskf_cpp_reduced/eskf_config.hpp"
#include "eskf_cpp_reduced/eskf_jacobian_reduced.hpp"
#include "eskf_cpp_reduced/eskf_math.hpp"
#include "eskf_cpp_reduced/eskf_types_reduced.hpp"

#include <deque>

namespace eskf::reduced {

class ReducedErrorStateKalmanFilter {
public:
    explicit ReducedErrorStateKalmanFilter(const ESKFParams& params);

    void reset(const NominalState& x_init, const ErrorCovariance& P_init);

    void accumulateIMU(const Vector3d& omega);
    bool isReadyForPrediction() const;
    IMUMeasurement getAveragedIMU(double timestamp);

    void predict(const Vector3d& omega_meas,
                 const InterceptorState& interceptor,
                 double timestamp);

    Vector2d correctImage(const Vector2d& z_pbar, int delay_steps);
    RadarMeasurement correctRadar(const RadarMeasurement& z_target,
                                 const RadarNoise& radar_noise);

    void notifyRadarReceived();
    bool wasRadarReceived() const { return radar_received_; }
    bool wasImageReceived() const { return first_image_received_; }

    void setPbarFrozen(bool frozen) { pbar_frozen_ = frozen; }
    bool isPbarFrozen() const { return pbar_frozen_; }

    const NominalState& getState() const { return x_; }
    const ErrorCovariance& getCovariance() const { return P_; }
    const ESKFParams& getParams() const { return params_; }
    double getTimestamp() const { return current_time_; }

    Vector3d getTargetPosition() const { return state_access::getPosition(x_); }
    Vector3d getTargetVelocity() const { return state_access::getVelocity(x_); }
    Vector2d getPbar() const { return state_access::getPbar(x_); }

    Eigen::Matrix<double, 8, 1> getCovarianceDiagonal() const { return P_.diagonal(); }

private:
    NominalState predictNominalState(const NominalState& x,
                                     const Vector3d& omega_meas,
                                     const InterceptorState& interceptor,
                                     double dt) const;

    NominalState injectErrorState(const NominalState& x_nominal,
                                  const ErrorState& delta_x) const;

    ErrorCovariance resetCovariance(const ErrorCovariance& P) const;

    void repropagate(const NominalState& x_start,
                     const ErrorCovariance& P_start,
                     size_t idx_start);

    void updateHistory(const IMUMeasurement& imu,
                       const InterceptorState& interceptor);

    ImageJacobian getImageMeasurementMatrix() const;
    RadarJacobian getRadarMeasurementMatrix() const;
    ErrorCovariance createInitialCovariance() const;

    ESKFParams params_;

    NominalState x_ = NominalState::Zero();
    ErrorCovariance P_ = ErrorCovariance::Zero();
    ProcessNoise Qd_ = ProcessNoise::Zero();

    double current_time_ = 0.0;

    ImageNoise R_img_ = ImageNoise::Identity();
    RadarNoise R_radar_ = RadarNoise::Identity();

    ImageJacobian H_img_ = ImageJacobian::Zero();
    RadarJacobian H_radar_ = RadarJacobian::Zero();

    std::deque<StateHistoryEntry> history_;

    Vector3d omega_accum_ = Vector3d::Zero();
    int imu_count_ = 0;
    int samples_per_eskf_ = 1;

    bool radar_received_ = false;
    bool first_image_received_ = false;
    bool pbar_frozen_ = true;
};

NominalState createInitialState(const Vector3d& p_t_true,
                                const Vector3d& v_t_true,
                                const Vector2d& pbar_true = Vector2d::Zero(),
                                const Vector3d& pos_error = Vector3d::Zero(),
                                const Vector3d& vel_error = Vector3d::Zero(),
                                const Vector2d& pbar_error = Vector2d::Zero());

Vector2d computeImageFeatures(const Vector3d& p_t,
                              const InterceptorState& interceptor,
                              const RotationMatrix& R_b2c);

}  // namespace eskf::reduced
