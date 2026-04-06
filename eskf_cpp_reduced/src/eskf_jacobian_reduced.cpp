/**
 * @file eskf_jacobian_reduced.cpp
 * @brief Reduced-state ESKF Jacobian implementation.
 */

#include "eskf_cpp_reduced/eskf_jacobian_reduced.hpp"

namespace eskf::reduced {

ReducedESKFJacobians computeReducedESKFJacobians(
    const NominalState& x_nominal,
    const Vector3d& omega_meas,
    double dt,
    const RotationMatrix& R_b2c,
    const RotationMatrix& R_b2e,
    const Vector3d& p_i,
    const Vector3d& v_i,
    double min_depth) {
    using namespace state_access;
    using namespace math;

    ReducedESKFJacobians jac;

    const Vector3d p_t = getPosition(x_nominal);
    const Vector3d v_t = getVelocity(x_nominal);
    const Vector2d pbar = getPbar(x_nominal);

    const double pbar_x = pbar(0);
    const double pbar_y = pbar(1);

    const RotationMatrix R_e2b = R_b2e.transpose();

    const Vector3d p_c = R_b2c * R_e2b * (-(p_i - p_t));
    const double p_c_z = math::boundDepth(p_c(2), min_depth);

    const Vector3d v_c = R_b2c * R_e2b * (v_i - v_t);
    const Vector3d omega_c = R_b2c * omega_meas;

    const Eigen::Matrix2d A_pbar = computeApbar(pbar_x, pbar_y, v_c, omega_c, p_c_z);
    const auto Lv = computeLv(pbar_x, pbar_y, p_c_z);
    const Vector2d A_pc_z = computeApcz(pbar_x, pbar_y, v_c, p_c_z);

    jac.Fc.setZero();

    jac.Fc.block<3, 3>(error_idx::DPT_START, error_idx::DVT_START) =
        Eigen::Matrix3d::Identity();

    const Vector3d e3 = constants::E3;
    const Eigen::Matrix3d dvc_ddvt = -R_b2c * R_e2b;
    const Eigen::RowVector3d dpcz_ddpt = e3.transpose() * R_b2c * R_e2b;

    jac.Fc.block<2, 3>(error_idx::DPBAR_START, error_idx::DPT_START) =
        A_pc_z * dpcz_ddpt;
    jac.Fc.block<2, 3>(error_idx::DPBAR_START, error_idx::DVT_START) =
        Lv * dvc_ddvt;
    jac.Fc.block<2, 2>(error_idx::DPBAR_START, error_idx::DPBAR_START) = A_pbar;

    jac.Gc.setZero();
    jac.Gc.block<3, 3>(error_idx::DVT_START, noise_idx::VT_RW_START) =
        Eigen::Matrix3d::Identity();

    jac.Fd = StateJacobian::Identity() + jac.Fc * dt;
    jac.Gd = jac.Gc * dt;

    return jac;
}

}  // namespace eskf::reduced
