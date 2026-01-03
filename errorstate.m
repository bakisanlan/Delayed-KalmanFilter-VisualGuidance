% Error states kinematics
%%reference frames, e=earth frame, b=body frame, c=camera plane
% x=nominal states, xt=true states, δx=error states
% x  = [ q  pr  vr  pbar  bgyro  bacc]  
% δx = [δθ δpr δvr δpbar δbgyro δbacc]  
% pr, vr, δpr, δvr are defined in earth frame 
% R(q)  = Rotation from body frame to earth frame
% R_c_b = Rotation from body frame to camera plane(constant)
% 
% wt  = wm - wbt - wn
% wbt = wb + δbgyro (wb=gyroscope bias)
% at  = Rt(am - abt - an) + g
% abt = ab + δbacc  (ab=accelerometer bias)
% wt=true body angular rate,   wm=measured body angular rate from IMU
% at=true linear acceleration, am=measured body linaer acceleration(exlude g)
% an, wn white noise of accelerometer and gyroscope

% pc = R_c_b*R(q)*(-pr)
% vc = R_c_b*R(q)*vr
% wc = R_c_b*w

% IBVS model
% Ls =  [-1/p_c_z 0         pbarx/pc_z  pbarx*pbary  -(1+pbarx^2)   pbary;
%         0       -1/p_c_z  pbary/pc_z  1+pbary^2    -pbarx*pbary  -pbarx]

%%continuous nominal state kinematics
% q_dot     = 0.5*q⊗(wm-wb)
% pr_dot    = vr
% vr_dot    = R*(am-ab) + g
% pbar_dot  = Ls*[vc ; wc] 
% bgyro_dot = 0
% bacc_dot  = 0

%%continuous error state kinematics
% δθ_dot     = -skew([wm - wb])*δθ -δbgyro -wn
% δpr_dot    = δvr
% δvr_dot    = -R*skew([am - ab])*δθ - Rδbacc - an
% δpbar_dot  = A_pbar*δpbar + A_vc*δvc + A_wc*δwc + A_pzc*δpc_z
% δbgyro_dot = ww
% δbacc_dot  = aw

%Jacobians terms of δpbar_dot
% A_pbar = [vc_z/pc_z + pbary*wc(1) - 2pbarx*wc(2), pbarx*wc(1) + wc_z                       ;
%          -pbary*wc(2) - wc_z                     , vc_z/pc_z +pbarx*wc(2) - 2pbary*wc(1)]

% A_vc   = [-1/pc_z 0        pbarx/pc_z; 
%           0      -1/pc_z  pbary/pc_z]

% A_wc   = [pbarx*pbary  -(1+pbarx^2)   pbary;
%          1+pbary^2    -pbarx*pbary  -pbarx]

% A_pc_z = [(vc_x - pbarx*vc_z)/pc_z^2 ;
%           (vc_y - pbary*vc_z)/pc_z^2]

% Equalities of δvc, δwc, δpc_z in terms of error states in continuous time
% e3 = [0 0 1]
% δvc   =  R_c_b*R(q)'*δvr - R_c_b*skew(R(q)'*vr)*δθ
% δwc   = -R_c_b*δbgyro - R_c_b*wn
% δpc_z = -e3*R_c_b*R(q)'*δpr + e3*R_c_b*skew(R(q)'pr)*δθ



function u_skew = skew(u_vec)
u_skew = [ 0        -u_vec(3)  u_vec(2);
           u_vec(3)  0        -u_vec(1);
          -u_vec(2)  u_vec(1)  0];
end

function quat = exp_quat(d_rot)
    % This function create a quaternion based on exponential map to
    % small angular deviation

    % d_rot is 3xM matrix

    norm_rot = vecnorm(d_rot,2,1);

    qw = cos(norm_rot/2);
    qr = (d_rot./norm_rot) .* sin(norm_rot/2);

    % dealt with NaN values which mean zero rotation
    idx_nan = ~isfinite(qr(1,:));
    qr(:,idx_nan) = zeros(3,sum(idx_nan));

    quat = [qw ; qr];

end



function rotM = exp_rot(d_rot)
    % This function create a rotation matrix based on exponential map to
    % small angular deviation,
    %eq 78 of Quaternion kinematics for the error-state Kalman Filter

    d_rot = reshape(d_rot,[],1); %force d_rot to be column vector
    norm_rot = norm(d_rot);

    if norm_rot == 0

        rotM = eye(3);
        return;
    end

    u_vec = d_rot / norm_rot;
    u_skew = skew(u_vec);

    rotM = eye(3)*cos(norm_rot) + u_skew * sin(norm_rot) + u_vec*u_vec'*(1-cos(norm_rot));
end