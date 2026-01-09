/**
 * @file eskf_jacobian.hpp
 * @brief ESKF Jacobian computation for error-state propagation
 * 
 * Computes the continuous and discrete-time Jacobians for ESKF:
 * - Fc (17x17): Continuous-time error state Jacobian
 * - Gc (17x12): Continuous-time noise Jacobian
 * - Fd (17x17): Discrete-time state transition matrix
 * - Gd (17x12): Discrete-time noise Jacobian
 * 
 * Based on compute_eskf_jacobians.m and ESKF_Report.tex equations.
 */

#pragma once

#include "eskf_types.hpp"
#include "eskf_math.hpp"

namespace eskf {

/**
 * @brief Structure to hold all ESKF Jacobians
 */
struct ESKFJacobians {
    StateJacobian Fc;   ///< Continuous-time error state Jacobian (17x17)
    NoiseJacobian Gc;   ///< Continuous-time noise Jacobian (17x12)
    StateJacobian Fd;   ///< Discrete-time state transition (17x17)
    NoiseJacobian Gd;   ///< Discrete-time noise Jacobian (17x12)
};

/**
 * @brief Compute all ESKF Jacobians for error-state propagation
 * 
 * This function computes the continuous-time Jacobians Fc and Gc, then
 * discretizes them to Fd and Gd. The attitude block uses exact exponential
 * discretization via Rodrigues formula.
 * 
 * Error state: δx = [δθ(3), δpr(3), δvr(3), δpbar(2), δbgyr(3), δbacc(3)]
 * Noise vector: n = [n_ω(3), n_a(3), ω_w(3), a_w(3)]
 * 
 * The continuous-time dynamics are:
 *   δẋ = Fc * δx + Gc * n
 * 
 * @param x_nominal Current nominal state (18x1)
 * @param omega_meas Measured angular velocity [rad/s]
 * @param a_meas Measured linear acceleration [m/s²]
 * @param dt Time step for discretization [s]
 * @param R_b2c Body-to-camera rotation matrix
 * @return ESKFJacobians structure containing Fc, Gc, Fd, Gd
 */
ESKFJacobians computeESKFJacobians(
    const NominalState& x_nominal,
    const Vector3d& omega_meas,
    const Vector3d& a_meas,
    double dt,
    const RotationMatrix& R_b2c);

/**
 * @brief Compute ZUPT measurement Jacobian
 * 
 * When stationary, the measurement model is:
 *   a_meas ≈ bias_a + R_b2e' * g (plus noise)
 *   ω_meas ≈ bias_ω (plus noise)
 * 
 * The 6x17 Jacobian H relates error state to residuals:
 *   H1 = [skew(R_b2e'*g)  0₃  0₃  0₂  0₃  -I₃]  (accel rows)
 *   H2 = [0₃              0₃  0₃  0₂  -I₃  0₃]  (gyro rows)
 * 
 * @param x_nominal Current nominal state (for extracting quaternion/biases)
 * @return 6x17 ZUPT measurement Jacobian
 */
ZUPTJacobian computeZUPTJacobian(const NominalState& x_nominal);

} // namespace eskf
