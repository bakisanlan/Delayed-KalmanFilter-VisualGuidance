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

    void updatePbarFromGeometry(const InterceptorState& interceptor);

    void notifyRadarReceived();
    bool wasRadarReceived() const { return radar_received_; }
    bool wasImageReceived() const { return first_image_received_; }

    void setPbarFrozen(bool frozen) {
        pbar_frozen_ = frozen;
        if (!frozen) steps_since_unfreeze_ = 0;  // reset: history not yet trustworthy
    }
    bool isPbarFrozen() const { return pbar_frozen_; }

    const NominalState& getState() const { return x_; }
    const ErrorCovariance& getCovariance() const { return P_; }
    const ESKFParams& getParams() const { return params_; }
    double getTimestamp() const { return current_time_; }

    Vector3d getTargetPosition() const { return state_access::getPosition(x_); }
    Vector3d getTargetVelocity() const { return state_access::getVelocity(x_); }
    Vector2d getPbar() const { return state_access::getPbar(x_); }
    Vector3d getRelativePosition(const reduced::InterceptorState& interceptor) const;

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

    /**
     * @brief Method 2: Measurement Extrapolation.
     *
     * Forward-predicts the delayed pixel measurement z_d = pbar(k-D) to the
     * current time k by integrating the IBVS dynamics over the stored
     * interceptor/IMU history.  The measurement noise covariance is inflated
     * by propagating R_img through the accumulated A_pbar linearisation:
     *
     *   R_ext = Phi_D * R_img * Phi_D^T   where Phi_D = prod_{j} F_j
     *                                      and   F_j = I + A_pbar_j * dt
     *
     * A standard present-time KF update is then applied to (x_, P_) using the
     * extrapolated measurement z_ext and the inflated noise R_ext.
     *
     * @param z_pbar   Raw delayed pixel measurement (2x1)
     * @param idx_delayed Index in history_ of the state at capture time (k-D)
     * @return Innovation vector (2x1), or zero vector if gated/skipped.
     */
    Vector2d correctImageNone(const Vector2d& z_pbar);

    Vector2d correctImageExtrapolate(const Vector2d& z_pbar, int idx_delayed);

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
    int  steps_since_unfreeze_ = 0;
};

NominalState createInitialState(const Vector3d& p_t_true,
                                const Vector3d& v_t_true,
                                const Vector2d& pbar_true = Vector2d::Zero(),
                                const Vector3d& pos_error = Vector3d::Zero(),
                                const Vector3d& vel_error = Vector3d::Zero(),
                                const Vector2d& pbar_error = Vector2d::Zero());

}  // namespace eskf::reduced
