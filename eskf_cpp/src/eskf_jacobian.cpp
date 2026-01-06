/**
 * @file eskf_jacobian.cpp
 * @brief Implementation of ESKF Jacobian computation
 * 
 * Direct translation of compute_eskf_jacobians.m to C++.
 */

#include "eskf_cpp/eskf_jacobian.hpp"

namespace eskf {

ESKFJacobians computeESKFJacobians(
    const NominalState& x_nominal,
    const Vector3d& omega_meas,
    const Vector3d& a_meas,
    double dt,
    const RotationMatrix& R_b2c)
{
    using namespace state_access;
    using namespace math;
    
    ESKFJacobians jac;
    
    // ========================================================================
    // Extract nominal states
    // ========================================================================
    const Quaterniond q = getQuaternion(x_nominal);
    const Vector3d p_r = getPosition(x_nominal);
    const Vector3d v_r = getVelocity(x_nominal);
    const Vector2d pbar = getPbar(x_nominal);
    const Vector3d b_gyr = getGyroBias(x_nominal);
    const Vector3d b_acc = getAccelBias(x_nominal);
    
    const double pbar_x = pbar(0);
    const double pbar_y = pbar(1);
    
    // ========================================================================
    // Rotation matrices
    // ========================================================================
    const RotationMatrix R_b2e = quaternionToRotation(q);  // Body to Earth
    const RotationMatrix R_e2b = R_b2e.transpose();         // Earth to Body
    
    // ========================================================================
    // Corrected IMU using nominal bias
    // ========================================================================
    const Vector3d omega = omega_meas - b_gyr;   // Corrected angular velocity
    const Vector3d a_body = a_meas - b_acc;       // Corrected body acceleration
    
    // ========================================================================
    // Camera frame quantities
    // ========================================================================
    // p_c = R_b2c * R_e2b * (-p_r)
    const Vector3d p_c = R_b2c * R_e2b * (-p_r);
    const double p_c_z = std::max(p_c(2), constants::MIN_DEPTH);
    
    // v_c = R_b2c * R_e2b * v_r
    const Vector3d v_c = R_b2c * R_e2b * v_r;
    
    // omega_c = R_b2c * omega
    const Vector3d omega_c = R_b2c * omega;
    
    // ========================================================================
    // IBVS Jacobian submatrices
    // ========================================================================
    const Eigen::Matrix2d A_pbar = computeApbar(pbar_x, pbar_y, v_c, omega_c, p_c_z);
    const auto Lv = computeLv(pbar_x, pbar_y, p_c_z);
    const auto Lw = computeLw(pbar_x, pbar_y);
    const Vector2d A_pc_z = computeApcz(pbar_x, pbar_y, v_c, p_c_z);
    
    // ========================================================================
    // Build Continuous-Time Fc Matrix (17x17)
    // ========================================================================
    jac.Fc.setZero();
    
    // --- Row 0-2: δθ_dot = -skew(omega)*δθ - δbgyro ---
    jac.Fc.block<3,3>(error_idx::DTHETA_START, error_idx::DTHETA_START) = -skew(omega);
    jac.Fc.block<3,3>(error_idx::DTHETA_START, error_idx::DBGYR_START) = -Eigen::Matrix3d::Identity();
    
    // --- Row 3-5: δpr_dot = δvr ---
    jac.Fc.block<3,3>(error_idx::DPR_START, error_idx::DVR_START) = Eigen::Matrix3d::Identity();
    
    // --- Row 6-8: δvr_dot = -R*skew(a_body)*δθ - R*δbacc ---
    jac.Fc.block<3,3>(error_idx::DVR_START, error_idx::DTHETA_START) = -R_b2e * skew(a_body);
    jac.Fc.block<3,3>(error_idx::DVR_START, error_idx::DBACC_START) = -R_b2e;
    
    // --- Row 9-10: δpbar_dot ---
    // δpbar_dot = A_pbar*δpbar + Lv*δvc + Lw*δwc + A_pc_z*δpc_z
    //
    // Where:
    //   δvc   = R_b2c*R_e2b*δvr + R_b2c*skew(R_e2b*v_r)*δθ
    //   δwc   = -R_b2c*δbgyro
    //   δpc_z = -e3*R_b2c*R_e2b*δpr - e3*R_b2c*skew(R_e2b*p_r)*δθ
    
    const Vector3d e3 = constants::E3;
    
    // ∂δvc/∂δθ = R_b2c * skew(R_e2b * v_r)
    const Eigen::Matrix3d dvc_ddtheta = R_b2c * skew(R_e2b * v_r);
    
    // ∂δvc/∂δvr = R_b2c * R_e2b
    const Eigen::Matrix3d dvc_ddvr = R_b2c * R_e2b;
    
    // ∂δwc/∂δbgyro = -R_b2c
    const Eigen::Matrix3d dwc_ddbgyr = -R_b2c;
    
    // ∂δpc_z/∂δθ = -e3' * R_b2c * skew(R_e2b * p_r)  (1x3)
    const Eigen::RowVector3d dpcz_ddtheta = -e3.transpose() * R_b2c * skew(R_e2b * p_r);
    
    // ∂δpc_z/∂δpr = -e3' * R_b2c * R_e2b  (1x3)
    const Eigen::RowVector3d dpcz_ddpr = -e3.transpose() * R_b2c * R_e2b;
    
    // ∂δpbar_dot/∂δθ = Lv * dvc_ddtheta + A_pc_z * dpcz_ddtheta
    jac.Fc.block<2,3>(error_idx::DPBAR_START, error_idx::DTHETA_START) = 
        Lv * dvc_ddtheta + A_pc_z * dpcz_ddtheta;
    
    // ∂δpbar_dot/∂δpr = A_pc_z * dpcz_ddpr
    jac.Fc.block<2,3>(error_idx::DPBAR_START, error_idx::DPR_START) = 
        A_pc_z * dpcz_ddpr;
    
    // ∂δpbar_dot/∂δvr = Lv * dvc_ddvr
    jac.Fc.block<2,3>(error_idx::DPBAR_START, error_idx::DVR_START) = 
        Lv * dvc_ddvr;
    
    // ∂δpbar_dot/∂δpbar = A_pbar
    jac.Fc.block<2,2>(error_idx::DPBAR_START, error_idx::DPBAR_START) = A_pbar;
    
    // ∂δpbar_dot/∂δbgyro = Lw * dwc_ddbgyr
    jac.Fc.block<2,3>(error_idx::DPBAR_START, error_idx::DBGYR_START) = 
        Lw * dwc_ddbgyr;
    
    // --- Row 11-13: δbgyro_dot = 0 (handled by noise) ---
    // --- Row 14-16: δbacc_dot = 0 (handled by noise) ---
    // (already zero from setZero)
    
    // ========================================================================
    // Build Continuous-Time Gc Matrix (17x12)
    // ========================================================================
    // Noise vector: n = [n_ω(3), n_a(3), ω_w(3), a_w(3)]
    
    jac.Gc.setZero();
    
    // δθ_dot affected by gyro noise: -I * n_ω
    jac.Gc.block<3,3>(error_idx::DTHETA_START, noise_idx::OMEGA_N_START) = 
        -Eigen::Matrix3d::Identity();
    
    // δvr_dot affected by accel noise: -R_b2e * n_a
    jac.Gc.block<3,3>(error_idx::DVR_START, noise_idx::A_N_START) = 
        -R_b2e;
    
    // δpbar_dot affected by gyro noise through δwc = -R_b2c * n_ω
    jac.Gc.block<2,3>(error_idx::DPBAR_START, noise_idx::OMEGA_N_START) = 
        Lw * (-R_b2c);
    
    // δbgyro_dot affected by bias random walk
    jac.Gc.block<3,3>(error_idx::DBGYR_START, noise_idx::OMEGA_W_START) = 
        Eigen::Matrix3d::Identity();
    
    // δbacc_dot affected by bias random walk
    jac.Gc.block<3,3>(error_idx::DBACC_START, noise_idx::A_W_START) = 
        Eigen::Matrix3d::Identity();
    
    // ========================================================================
    // Discretization
    // ========================================================================
    // First-order approximation: Fd ≈ I + Fc*dt, Gd ≈ Gc*dt
    // With exact exponential for attitude block
    
    // --- Fd (17x17) ---
    jac.Fd = StateJacobian::Identity() + jac.Fc * dt;
    
    // Exact discretization for attitude: exp(-skew(omega)*dt)
    const Vector3d omega_dt = omega * dt;
    const RotationMatrix Phi_theta = expRotation(-omega_dt);
    jac.Fd.block<3,3>(error_idx::DTHETA_START, error_idx::DTHETA_START) = Phi_theta;
    
    // Correct attitude-to-bias coupling (first-order approximation)
    jac.Fd.block<3,3>(error_idx::DTHETA_START, error_idx::DBGYR_START) = 
        -Eigen::Matrix3d::Identity() * dt;
    
    // --- Gd (17x12) ---
    jac.Gd = jac.Gc * dt;
    
    return jac;
}

} // namespace eskf
