/**
 * @file eskf_core.hpp
 * @brief Error-State Kalman Filter core class
 * 
 * Main ESKF implementation with:
 * - Prediction step (nominal state + covariance propagation)
 * - Correction steps (image with delay, radar without delay)
 * - IMU accumulation for frequency decoupling
 * - State history for delayed measurement handling
 * 
 * Based on ErrorStateKalmanFilter.m from MATLAB implementation.
 */

#pragma once

#include "eskf_types.hpp"
#include "eskf_math.hpp"
#include "eskf_jacobian.hpp"

#include <vector>
#include <deque>

namespace eskf {

/**
 * @brief Error-State Kalman Filter for Visual Guidance
 * 
 * ESKF maintains separation between:
 *   - Nominal state x (18): [q(4), pr(3), vr(3), pbar(2), bgyr(3), bacc(3)]
 *   - Error state δx (17): [δθ(3), δpr(3), δvr(3), δpbar(2), δbgyr(3), δbacc(3)]
 * 
 * The covariance P is 17x17 for the error state (not the nominal state).
 * 
 * Key features:
 * - Frequency decoupling: ESKF can run at lower rate than IMU
 * - Delayed measurements: Image measurements handled with state history buffer
 * - Joseph form covariance update for numerical stability
 */
class ErrorStateKalmanFilter {
public:
    /**
     * @brief Construct ESKF with given parameters
     * @param params Configuration parameters
     */
    explicit ErrorStateKalmanFilter(const ESKFParams& params);
    
    /**
     * @brief Reset filter to initial state
     * @param x_init Initial nominal state (18x1)
     * @param P_init Initial error covariance (17x17)
     */
    void reset(const NominalState& x_init, const ErrorCovariance& P_init);
    
    // ========================================================================
    // IMU Accumulation (for frequency decoupling)
    // ========================================================================
    
    /**
     * @brief Accumulate IMU measurement for later prediction
     * 
     * When ESKF runs slower than IMU rate, measurements are accumulated
     * and averaged before the prediction step.
     * 
     * @param omega Angular velocity [rad/s]
     * @param accel Linear acceleration [m/s²]
     */
    void accumulateIMU(const Vector3d& omega, const Vector3d& accel);
    
    /**
     * @brief Check if enough IMU samples accumulated for prediction
     * @return true if ready for prediction step
     */
    bool isReadyForPrediction() const;
    
    /**
     * @brief Get averaged IMU measurement and reset accumulators
     * @return Averaged IMU measurement
     */
    IMUMeasurement getAveragedIMU();
    
    // ========================================================================
    // Prediction Step
    // ========================================================================
    
    /**
     * @brief ESKF prediction step
     * 
     * 1. Propagates nominal state using IMU measurements
     * 2. Propagates error covariance (error state stays at zero)
     * 3. Updates history buffer for delayed measurements
     * 
     * @param omega_meas Measured angular velocity [rad/s]
     * @param a_meas Measured linear acceleration [m/s²]
     * @param timestamp Current timestamp [s]
     */
    void predict(const Vector3d& omega_meas, const Vector3d& a_meas, double timestamp);
    
    // ========================================================================
    // Correction Steps
    // ========================================================================
    
    /**
     * @brief Correction with delayed image measurement
     * 
     * Handles measurement delay by:
     * 1. Looking up state from history at delayed time
     * 2. Computing Kalman gain and error state update
     * 3. Injecting error into nominal state
     * 4. Re-propagating to current time using stored IMU history
     * 
     * @param z_pbar Measured normalized image coordinates (2x1)
     * @param delay_steps Delay in ESKF cycles (not IMU cycles)
     * @return Innovation vector (for diagnostics), empty if measurement rejected
     */
    Vector2d correctImage(const Vector2d& z_pbar, int delay_steps);
    
    /**
     * @brief Correction with radar measurement (no delay)
     * 
     * Radar measures relative position directly:
     *   z_radar = p_r + noise
     * 
     * @param z_radar Measured relative position in earth frame (3x1)
     * @return Innovation vector (for diagnostics)
     */
    Vector3d correctRadar(const Vector3d& z_radar);
    
    // ========================================================================
    // State Access
    // ========================================================================
    
    /** @brief Get current nominal state */
    const NominalState& getState() const { return x_; }
    
    /** @brief Get current error covariance */
    const ErrorCovariance& getCovariance() const { return P_; }
    
    /** @brief Get parameters */
    const ESKFParams& getParams() const { return params_; }
    
    /** @brief Get current timestamp */
    double getTimestamp() const { return current_time_; }
    
    // Convenience accessors
    Quaterniond getQuaternion() const { return state_access::getQuaternion(x_); }
    Vector3d getPosition() const { return state_access::getPosition(x_); }
    Vector3d getVelocity() const { return state_access::getVelocity(x_); }
    Vector2d getPbar() const { return state_access::getPbar(x_); }
    Vector3d getGyroBias() const { return state_access::getGyroBias(x_); }
    Vector3d getAccelBias() const { return state_access::getAccelBias(x_); }
    
    /** @brief Get covariance diagonal for monitoring */
    Eigen::Matrix<double, 17, 1> getCovarianceDiagonal() const { return P_.diagonal(); }

private:
    // ========================================================================
    // Internal Methods
    // ========================================================================
    
    /**
     * @brief Propagate nominal state using IMU measurements
     * @param x Current nominal state
     * @param omega_meas Measured angular velocity
     * @param a_meas Measured acceleration
     * @param dt Time step (defaults to dt_eskf)
     * @return New nominal state
     */
    NominalState predictNominalState(const NominalState& x,
                                      const Vector3d& omega_meas,
                                      const Vector3d& a_meas,
                                      double dt) const;
    
    /**
     * @brief Inject error state into nominal state
     * 
     * x_corrected = x_nominal ⊕ δx
     * 
     * @param x_nominal Current nominal state
     * @param delta_x Error state correction
     * @return Corrected nominal state
     */
    NominalState injectErrorState(const NominalState& x_nominal,
                                   const ErrorState& delta_x) const;
    
    /**
     * @brief Reset covariance after error state injection
     * 
     * P_reset = G * P * G'
     * where G accounts for attitude parametrization reset
     * 
     * @param P Covariance before reset
     * @param delta_x Injected error state
     * @return Reset covariance
     */
    ErrorCovariance resetCovariance(const ErrorCovariance& P,
                                    const ErrorState& delta_x) const;
    
    /**
     * @brief Re-propagate from corrected delayed state to current time
     * @param x_start Corrected state at delayed time
     * @param P_start Corrected covariance at delayed time
     * @param idx_start Starting index in history buffer
     */
    void repropagate(const NominalState& x_start,
                     const ErrorCovariance& P_start,
                     size_t idx_start);
    
    /**
     * @brief Update history buffer with current state
     */
    void updateHistory(const IMUMeasurement& imu);
    
    /**
     * @brief Compute image measurement matrix (error state)
     * @return 2x17 measurement Jacobian
     */
    ImageJacobian getImageMeasurementMatrix() const;
    
    /**
     * @brief Compute radar measurement matrix (error state)
     * @return 3x17 measurement Jacobian
     */
    RadarJacobian getRadarMeasurementMatrix() const;
    
    /**
     * @brief Create initial covariance from parameter sigmas
     */
    ErrorCovariance createInitialCovariance() const;
    
    // ========================================================================
    // Member Variables
    // ========================================================================
    
    ESKFParams params_;              ///< Configuration parameters
    
    NominalState x_;                 ///< Current nominal state (18x1)
    ErrorCovariance P_;              ///< Current error covariance (17x17)
    ProcessNoise Qd_;                ///< Discrete process noise (12x12)
    
    double current_time_;            ///< Current timestamp
    
    // Measurement noise covariances
    ImageNoise R_img_;               ///< Image noise (2x2)
    RadarNoise R_radar_;             ///< Radar noise (3x3)
    
    // Fixed measurement matrices
    ImageJacobian H_img_;            ///< Image measurement Jacobian (2x17)
    RadarJacobian H_radar_;          ///< Radar measurement Jacobian (3x17)
    
    // History buffer for delayed measurements
    std::deque<StateHistoryEntry> history_;
    
    // IMU accumulation for frequency decoupling
    Vector3d omega_accum_;           ///< Accumulated angular velocity
    Vector3d accel_accum_;           ///< Accumulated acceleration
    int imu_count_;                  ///< Number of accumulated samples
    int samples_per_eskf_;           ///< IMU samples per ESKF update
};

// ============================================================================
// Static Helper Functions
// ============================================================================

/**
 * @brief Create initial nominal state with known errors (for testing)
 * 
 * @param q_true True quaternion
 * @param p_r_true True relative position
 * @param v_r_true True relative velocity
 * @param pbar_true True image features
 * @param euler_error_deg Attitude error in Euler angles [deg]
 * @param pos_error Position error [m]
 * @param vel_error Velocity error [m/s]
 * @param pbar_error Image feature error
 * @return Initial nominal state with errors applied
 */
NominalState createInitialState(
    const Quaterniond& q_true,
    const Vector3d& p_r_true,
    const Vector3d& v_r_true,
    const Vector2d& pbar_true,
    const Vector3d& euler_error_deg = Vector3d::Zero(),
    const Vector3d& pos_error = Vector3d::Zero(),
    const Vector3d& vel_error = Vector3d::Zero(),
    const Vector2d& pbar_error = Vector2d::Zero());

/**
 * @brief Compute normalized image features from relative position
 * 
 * @param p_r Relative position in earth frame
 * @param q Quaternion (body to earth rotation)
 * @param R_b2c Body to camera rotation
 * @return Normalized image coordinates [pbar_x, pbar_y]
 */
Vector2d computeImageFeatures(const Vector3d& p_r,
                               const Quaterniond& q,
                               const RotationMatrix& R_b2c);

} // namespace eskf
