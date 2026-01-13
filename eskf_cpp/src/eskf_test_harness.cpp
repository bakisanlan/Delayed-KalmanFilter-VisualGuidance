/**
 * @file eskf_test_harness.cpp
 * @brief Standalone test program for ESKF - replicates MATLAB run_eskf_simulation.m
 * 
 * This program tests the ESKF C++ implementation without ROS2 dependencies.
 * It generates synthetic sensor data matching the MATLAB simulation and
 * compares results for validation.
 * 
 * Usage: ./eskf_test_harness [config_file]
 */

#include "eskf_cpp/eskf_core.hpp"
#include "eskf_cpp/eskf_config.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <random>
#include <cmath>
#include <chrono>

using namespace eskf;

// ============================================================================
// IMU Simulator (matches IMUModel.m)
// ============================================================================

class IMUSimulator {
public:
    IMUSimulator(const ESKFParams& params, unsigned int seed = 35)
        : params_(params)
        , rng_(seed)
        , normal_dist_(0.0, 1.0)
        , omega_b_(Vector3d(0.005, -0.003, 0.002))    // Initial gyro bias
        , a_b_(Vector3d(0.02, -0.01, 0.015))          // Initial accel bias
    {}
    
    /**
     * @brief Generate noisy IMU measurement with evolving biases
     */
    IMUMeasurement measure(const Vector3d& omega_true, const Vector3d& a_true) {
        IMUMeasurement meas;
        
        // Update biases via random walk
        const double dt = params_.dt_imu;
        const double sigma_omega_w = params_.sigma_omega_w * std::sqrt(dt);
        const double sigma_a_w = params_.sigma_a_w * std::sqrt(dt);
        
        omega_b_(0) += sigma_omega_w * normal_dist_(rng_);
        omega_b_(1) += sigma_omega_w * normal_dist_(rng_);
        omega_b_(2) += sigma_omega_w * normal_dist_(rng_);
        
        a_b_(0) += sigma_a_w * normal_dist_(rng_);
        a_b_(1) += sigma_a_w * normal_dist_(rng_);
        a_b_(2) += sigma_a_w * normal_dist_(rng_);
        
        // Measurement noise
        Vector3d n_omega, n_a;
        n_omega << params_.sigma_omega_n * normal_dist_(rng_),
                   params_.sigma_omega_n * normal_dist_(rng_),
                   params_.sigma_omega_n * normal_dist_(rng_);
        n_a << params_.sigma_a_n * normal_dist_(rng_),
               params_.sigma_a_n * normal_dist_(rng_),
               params_.sigma_a_n * normal_dist_(rng_);
        
        // Measured values = true + bias + noise
        meas.omega = omega_true + omega_b_ + n_omega;
        meas.accel = a_true + a_b_ + n_a;
        
        return meas;
    }
    
    Vector3d getGyroBias() const { return omega_b_; }
    Vector3d getAccelBias() const { return a_b_; }
    
    void reset() {
        omega_b_ = Vector3d(0.005, -0.003, 0.002);
        a_b_ = Vector3d(0.02, -0.01, 0.015);
    }

private:
    ESKFParams params_;
    std::mt19937 rng_;
    std::normal_distribution<double> normal_dist_;
    Vector3d omega_b_;
    Vector3d a_b_;
};

// ============================================================================
// True State Propagator (matches propagate_true_state.m)
// ============================================================================

struct TrueState {
    Quaterniond q;            // Attitude (body to earth)
    Vector3d p_int;           // Interceptor position
    Vector3d v_int;           // Interceptor velocity
    Vector3d p_tgt;           // Target position
    Vector3d v_tgt;           // Target velocity
    Vector3d omega_true;      // True angular velocity
    Vector3d a_body_true;     // True body acceleration
};

void propagateTrueState(TrueState& state, double t, double dt, 
                         const RotationMatrix& R_b2c) {
    // Simple trajectory for testing: gentle acceleration and rotation
    // This matches the MATLAB simulation behavior
    
    // Angular velocity (sinusoidal, matching MATLAB)
    state.omega_true = Vector3d(1.0, 2.0, 3.0) * (0.01 * std::sin(0.5 * t));
    
    // Rotation matrices
    RotationMatrix R_b2e = math::quaternionToRotation(state.q);
    RotationMatrix R_e2b = R_b2e.transpose();

    // Body acceleration (Specific Force)
    // Matches MATLAB: a_body_true = R_b2e' * (-g * e3) + sin(0.5 * t) *  0.1*[6; 0.4 ; -0.5]
    Vector3d gravity_vec(0.0, 0.0, 9.81); 
    Vector3d maneuver = Vector3d(6.0, 0.4, -0.5) * (0.1 * std::sin(0.5 * t));
    
    state.a_body_true = R_e2b * (-gravity_vec) + maneuver;
    
    // Update attitude using exponential map
    Vector3d omega_dt = state.omega_true * dt;
    Quaterniond dq = math::expQuaternion(omega_dt);
    state.q = math::quaternionMultiply(state.q, dq);
    state.q = math::normalizeQuaternion(state.q);
    
    // Update velocity (earth frame)
    Vector3d a_world = R_b2e * state.a_body_true + constants::GRAVITY_NED;
    Vector3d v_int_new = state.v_int + a_world * dt;
    
    // Update position (trapezoidal)
    state.p_int = state.p_int + 0.5 * (state.v_int + v_int_new) * dt;
    state.v_int = v_int_new;
    
    // Target moves slowly
    state.p_tgt += state.v_tgt * dt;
}

// ============================================================================
// Error Metrics
// ============================================================================

struct ErrorMetrics {
    std::vector<double> pos_error;
    std::vector<double> vel_error;
    std::vector<double> att_error_deg;
    std::vector<double> bgyr_error;
    std::vector<double> bacc_error;
    
    double computeRMSE(const std::vector<double>& errors) {
        if (errors.empty()) return 0.0;
        double sum_sq = 0.0;
        int valid_count = 0;
        for (double e : errors) {
            if (std::isfinite(e)) {  // Skip NaN and Inf
                sum_sq += e * e;
                valid_count++;
            }
        }
        if (valid_count == 0) return 0.0;
        return std::sqrt(sum_sq / valid_count);
    }
    
    void printSummary() {
        std::cout << "\n=== ESKF Performance Statistics ===\n";
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "Position RMSE:   " << computeRMSE(pos_error) << " m\n";
        std::cout << "Velocity RMSE:   " << computeRMSE(vel_error) << " m/s\n";
        std::cout << "Attitude RMSE:   " << computeRMSE(att_error_deg) << " deg\n";
        std::cout << "Gyro Bias RMSE:  " << computeRMSE(bgyr_error) << " rad/s\n";
        std::cout << "Accel Bias RMSE: " << computeRMSE(bacc_error) << " m/s²\n";
        std::cout << "===================================\n";
    }
};

// ============================================================================
// Main Test
// ============================================================================

int main(int argc, char** argv) {
    std::cout << "=== ESKF C++ Test Harness ===\n\n";
    
    // Load configuration
    ESKFParams params;
    if (argc > 1) {
        std::cout << "Loading config from: " << argv[1] << "\n";
        try {
            params = loadConfig(argv[1]);
        } catch (const std::exception& e) {
            std::cerr << "Error loading config: " << e.what() << "\n";
            std::cout << "Using default parameters.\n";
        }
    }
    printConfig(params);
    
    // Simulation parameters
    const double t_total = 25.0;
    const double dt_imu = params.dt_imu;
    const double dt_eskf = params.dt_eskf;
    const double dt_image = 1.0 / 30.0;
    const double dt_radar = 1.0 / 0.5;
    const double t_delay = params.image_delay;
    const int D = static_cast<int>(std::round(t_delay / dt_eskf));
    
    const int N_imu = static_cast<int>(t_total / dt_imu) + 1;
    const int eskf_sample_idx = static_cast<int>(std::round(dt_eskf / dt_imu));
    const int image_sample_idx = static_cast<int>(std::round(dt_image / dt_imu));
    const int radar_sample_idx = static_cast<int>(std::round(dt_radar / dt_imu));
    
    std::cout << "Simulation: " << t_total << "s, IMU: " << (1.0/dt_imu) 
              << "Hz, ESKF: " << (1.0/dt_eskf) << "Hz\n";
    std::cout << "Image delay: " << (t_delay*1000) << "ms (" << D << " ESKF cycles)\n\n";
    
    // Initialize true state
    TrueState truth;
    truth.q = Quaterniond(1, 0, 0, 0);  // Identity (level)
    truth.p_int = Vector3d(0, 0, -65);   // Interceptor position (NED)
    truth.v_int = Vector3d::Zero();
    truth.p_tgt = Vector3d(50, 10, -40); // Target position
    truth.v_tgt = Vector3d::Zero();
    truth.omega_true = Vector3d::Zero();
    truth.a_body_true = Vector3d::Zero();
    
    // Initial relative state
    Vector3d p_r_true = truth.p_int - truth.p_tgt;
    Vector3d v_r_true = truth.v_int - truth.v_tgt;
    
    // Compute initial image features
    Vector2d pbar_true = computeImageFeatures(p_r_true, truth.q, params.R_b2c);
    
    // Create ESKF
    ErrorStateKalmanFilter eskf(params);
    
    // Create initial state with errors (matching MATLAB)
    NominalState x_init = createInitialState(
        truth.q, p_r_true, v_r_true, pbar_true,
        Vector3d(1, 1, 1),          // Euler error [deg]
        Vector3d(0.5, 0.5, 0.5),    // Position error [m]
        Vector3d(0.1, -0.05, 0),    // Velocity error [m/s]
        Vector2d(0.01, -0.02)       // Pbar error
    );
    eskf.reset(x_init, eskf.getCovariance());
    
    // Create IMU simulator
    IMUSimulator imu_sim(params);
    
    // Error metrics
    ErrorMetrics metrics;
    
    // Random generator for measurement noise
    std::mt19937 rng(42);
    std::normal_distribution<double> normal(0.0, 1.0);
    
    // Counters
    int eskf_update_counter = 0;
    int image_update_counter = 0;
    int radar_update_counter = 0;
    
    // CSV output
    std::ofstream csv_file("cpp_results.csv");
    csv_file << "time,"
             // Estimated State (21)
             << "px_est,py_est,pz_est,vx_est,vy_est,vz_est,"
             << "qw_est,qx_est,qy_est,qz_est,"
             << "pbarx_est,pbary_est,"
             << "bgx_est,bgy_est,bgz_est,bax_est,bay_est,baz_est,"
             << "bmx_est,bmy_est,bmz_est,"
             // True State (21)
             << "px_true,py_true,pz_true,vx_true,vy_true,vz_true,"
             << "qw_true,qx_true,qy_true,qz_true,"
             << "pbarx_true,pbary_true,"
             << "bgx_true,bgy_true,bgz_true,bax_true,bay_true,baz_true,"
             << "bmx_true,bmy_true,bmz_true,"
             // Covariance Diagonal (20)
             << "P_dtheta_x,P_dtheta_y,P_dtheta_z,"
             << "P_dpr_x,P_dpr_y,P_dpr_z,"
             << "P_dvr_x,P_dvr_y,P_dvr_z,"
             << "P_dpbar_x,P_dpbar_y,"
             << "P_dbgyr_x,P_dbgyr_y,P_dbgyr_z,"
             << "P_dbacc_x,P_dbacc_y,P_dbacc_z,"
             << "P_dbmag_x,P_dbmag_y,P_dbmag_z\n";
    
    std::cout << "Starting simulation...\n";
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Main simulation loop
    for (int k = 0; k < N_imu; ++k) {
        double t = k * dt_imu;
        
        // === Propagate true state ===
        propagateTrueState(truth, t, dt_imu, params.R_b2c);
        
        // Update true relative state
        p_r_true = truth.p_int - truth.p_tgt;
        v_r_true = truth.v_int - truth.v_tgt;
        pbar_true = computeImageFeatures(p_r_true, truth.q, params.R_b2c);
        
        // === Generate IMU measurement ===
        IMUMeasurement imu_meas = imu_sim.measure(truth.omega_true, truth.a_body_true);
        
        // === Accumulate IMU for ESKF ===
        eskf.accumulateIMU(imu_meas.omega, imu_meas.accel);
        eskf_update_counter++;
        
        // === ESKF Prediction (at dt_eskf rate) ===
        if (eskf_update_counter >= eskf_sample_idx) {
            eskf_update_counter = 0;
            IMUMeasurement avg_imu = eskf.getAveragedIMU();
            eskf.predict(avg_imu.omega, avg_imu.accel, t);
        }
        
        // === Image Correction (with delay) ===
        image_update_counter++;
        if (image_update_counter >= image_sample_idx && k > D) {
            image_update_counter = 0;
            
            // Check if target is visible (positive camera z)
            RotationMatrix R_b2e = math::quaternionToRotation(truth.q);
            Vector3d p_cam = params.R_b2c * R_b2e.transpose() * (-p_r_true);
            
            if (p_cam(2) > 2.0) {
                // Generate noisy measurement
                Vector2d z_pbar;
                z_pbar(0) = p_cam(0) / p_cam(2) + params.sigma_img * normal(rng);
                z_pbar(1) = p_cam(1) / p_cam(2) + params.sigma_img * normal(rng);
                
                eskf.correctImage(z_pbar, D);
            }
        }
        
        // === Radar Correction (0.5 Hz, no delay) ===
        radar_update_counter++;
        if (radar_update_counter >= radar_sample_idx) {
            radar_update_counter = 0;
            
            // Generate noisy measurement (position + velocity = 6D)
            Vector6d z_radar;
            z_radar.head<3>() = p_r_true;
            z_radar.tail<3>() = v_r_true;
            z_radar(0) += params.sigma_radar_pos * normal(rng);
            z_radar(1) += params.sigma_radar_pos * normal(rng);
            z_radar(2) += params.sigma_radar_pos * normal(rng);
            z_radar(3) += params.sigma_radar_vel * normal(rng);
            z_radar(4) += params.sigma_radar_vel * normal(rng);
            z_radar(5) += params.sigma_radar_vel * normal(rng);
            
            eskf.correctRadar(z_radar);
        }
        
        // === Compute errors ===
        Vector3d pos_est = eskf.getPosition();
        Vector3d vel_est = eskf.getVelocity();
        Quaterniond q_est = eskf.getQuaternion();
        Vector3d bgyr_est = eskf.getGyroBias();
        Vector3d bacc_est = eskf.getAccelBias();
        
        double pos_err = (pos_est - p_r_true).norm();
        double vel_err = (vel_est - v_r_true).norm();
        
        // Attitude error from quaternion
        Quaterniond q_err = truth.q.conjugate() * q_est;
        double att_err_deg = 2.0 * std::acos(std::min(1.0, std::abs(q_err.w()))) * 180.0 / M_PI;
        
        double bgyr_err = (bgyr_est - imu_sim.getGyroBias()).norm();
        double bacc_err = (bacc_est - imu_sim.getAccelBias()).norm();
        
        // Debug: detect first NaN
        static bool nan_detected = false;
        if (!nan_detected && (!std::isfinite(pos_err) || !std::isfinite(vel_err))) {
            nan_detected = true;
            std::cout << "\n!!! NaN detected at k=" << k << ", t=" << t << "\n";
            std::cout << "pos_est: " << pos_est.transpose() << "\n";
            std::cout << "p_r_true: " << p_r_true.transpose() << "\n";
            std::cout << "vel_est: " << vel_est.transpose() << "\n";
            std::cout << "v_r_true: " << v_r_true.transpose() << "\n";
            
            // Calculate and print p_c_z (true)
            RotationMatrix R_b2e = math::quaternionToRotation(truth.q);
            Vector3d p_cam = params.R_b2c * R_b2e.transpose() * (-p_r_true);
            std::cout << "p_c_z (true): " << p_cam(2) << "\n";
        }


        metrics.pos_error.push_back(pos_err);
        metrics.vel_error.push_back(vel_err);
        metrics.att_error_deg.push_back(att_err_deg);
        metrics.bgyr_error.push_back(bgyr_err);
        metrics.bacc_error.push_back(bacc_err);
        
        // Write to CSV (every sample to match MATLAB plot resolution, or maybe every 10th if too large)
        if (k % 10 == 0) {
            Vector2d pbar_est = eskf.getPbar();
            Vector3d bmag_est = eskf.getMagBias();
            Vector3d bg_true = imu_sim.getGyroBias();
            Vector3d ba_true = imu_sim.getAccelBias();
            Vector3d bm_true = Vector3d::Zero();  // No true mag bias in sim
            Eigen::Matrix<double, 20, 1> P_diag = eskf.getCovarianceDiagonal();
            
            csv_file << std::fixed << std::setprecision(6)
                     << t << ","
                     // Estimated
                     << pos_est(0) << "," << pos_est(1) << "," << pos_est(2) << ","
                     << vel_est(0) << "," << vel_est(1) << "," << vel_est(2) << ","
                     << q_est.w() << "," << q_est.x() << "," << q_est.y() << "," << q_est.z() << ","
                     << pbar_est(0) << "," << pbar_est(1) << ","
                     << bgyr_est(0) << "," << bgyr_est(1) << "," << bgyr_est(2) << ","
                     << bacc_est(0) << "," << bacc_est(1) << "," << bacc_est(2) << ","
                     << bmag_est(0) << "," << bmag_est(1) << "," << bmag_est(2) << ","
                     // True
                     << p_r_true(0) << "," << p_r_true(1) << "," << p_r_true(2) << ","
                     << v_r_true(0) << "," << v_r_true(1) << "," << v_r_true(2) << ","
                     << truth.q.w() << "," << truth.q.x() << "," << truth.q.y() << "," << truth.q.z() << ","
                     << pbar_true(0) << "," << pbar_true(1) << ","
                     << bg_true(0) << "," << bg_true(1) << "," << bg_true(2) << ","
                     << ba_true(0) << "," << ba_true(1) << "," << ba_true(2) << ","
                     << bm_true(0) << "," << bm_true(1) << "," << bm_true(2);
            
            // Covariance (20 elements)
            for (int i = 0; i < 20; ++i) {
                csv_file << "," << P_diag(i);
            }
            csv_file << "\n";
        }
        
        // Progress
        if (k % (N_imu / 10) == 0) {
            std::cout << "Progress: " << (100 * k / N_imu) << "%\n";
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    csv_file.close();
    
    std::cout << "\nSimulation Complete!\n";
    std::cout << "Execution time: " << duration.count() << " ms\n";
    std::cout << "Processing rate: " << (N_imu * 1000.0 / duration.count()) << " Hz\n";
    
    // Print results
    metrics.printSummary();
    
    std::cout << "\nResults saved to: cpp_results.csv\n";
    
    return 0;
}
