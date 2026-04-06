/**
 * @file eskf_jacobian_reduced.hpp
 * @brief Reduced-state ESKF Jacobian computation.
 */

#pragma once

#include "eskf_cpp_reduced/eskf_math.hpp"
#include "eskf_cpp_reduced/eskf_types_reduced.hpp"

namespace eskf::reduced {

struct ReducedESKFJacobians {
    StateJacobian Fc = StateJacobian::Zero();
    NoiseJacobian Gc = NoiseJacobian::Zero();
    StateJacobian Fd = StateJacobian::Identity();
    NoiseJacobian Gd = NoiseJacobian::Zero();
};

ReducedESKFJacobians computeReducedESKFJacobians(
    const NominalState& x_nominal,
    const Vector3d& omega_meas,
    double dt,
    const RotationMatrix& R_b2c,
    const RotationMatrix& R_b2e,
    const Vector3d& p_i,
    const Vector3d& v_i,
    double min_depth);

}  // namespace eskf::reduced
