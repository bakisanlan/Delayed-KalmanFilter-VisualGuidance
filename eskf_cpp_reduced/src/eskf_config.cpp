/**
 * @file eskf_config.cpp
 * @brief Implementation of YAML configuration loader
 */

#include "eskf_cpp_reduced/eskf_config.hpp"
#include "eskf_cpp_reduced/utils/print.hpp"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdexcept>

namespace eskf {

namespace {

// Helper to safely get value with default
template<typename T>
T getScalar(const YAML::Node& node, const std::string& key, T default_value) {
    if (node[key]) {
        return node[key].as<T>();
    }
    return default_value;
}

// Parse rotation matrix from flat array
RotationMatrix parseRotationMatrix(const YAML::Node& node) {
    if (!node || node.size() != 9) {
        // Default: camera looking forward through body x-axis
        RotationMatrix R;
        R << 0, 1, 0,
             0, 0, 1,
             1, 0, 0;
        return R;
    }
    
    RotationMatrix R;
    for (int i = 0; i < 9; ++i) {
        R(i / 3, i % 3) = node[i].as<double>();
    }
    return R;
}

} // anonymous namespace

ESKFParams loadConfig(const std::string& filename) {
    try {
        YAML::Node config = YAML::LoadFile(filename);
        return loadConfigFromString(YAML::Dump(config));
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Failed to load config file: " + std::string(e.what()));
    }
}

ESKFParams loadConfigFromString(const std::string& yaml_content) {
    YAML::Node config = YAML::Load(yaml_content);
    ESKFParams params;
    
    // === Topic Names ===
    if (config["topics"]) {
        const auto& topics = config["topics"];
        params.topic_imu = getScalar<std::string>(topics, "imu", "/mavros/imu/data_raw");
        params.topic_image = getScalar<std::string>(topics, "image", "/yolo/target");
        params.topic_radar = getScalar<std::string>(topics, "radar", "/radar/target_odom");
        params.topic_interceptor_odom = getScalar<std::string>(topics, "interceptor_odom", "/mavros/global_position/local");
        params.topic_odom = getScalar<std::string>(topics, "output_state", "/eskf_reduced/odom");
        params.topic_pbar = getScalar<std::string>(topics, "output_pose", "/eskf_reduced/pbar");
    }
    
    // === Timing Parameters ===
    if (config["timing"]) {
        const auto& timing = config["timing"];
        double imu_rate = getScalar<double>(timing, "imu_rate_hz", 200.0);
        double eskf_rate = getScalar<double>(timing, "eskf_rate_hz", 200.0);
        
        params.dt_imu = 1.0 / imu_rate;
        params.dt_eskf = 1.0 / eskf_rate;
        params.image_delay = getScalar<double>(timing, "image_delay_ms", 80.0) / 1000.0;
        params.image_timeout_sec = getScalar<double>(timing, "image_timeout_sec", 2.0);
    }
    
    // === IMU Noise Parameters ===
    if (config["imu_noise"]) {
        const auto& imu = config["imu_noise"];
        params.sigma_a_n = getScalar<double>(imu, "sigma_a_n", 0.1);
        params.sigma_omega_n = getScalar<double>(imu, "sigma_omega_n", 0.01);
        params.sigma_a_w = getScalar<double>(imu, "sigma_a_w", 1e-4);
        params.sigma_omega_w = getScalar<double>(imu, "sigma_omega_w", 1e-5);
    }
    
    // === Measurement Noise ===
    if (config["measurement_noise"]) {
        const auto& meas = config["measurement_noise"];
        params.sigma_img = getScalar<double>(meas, "image_sigma", 0.005);
        params.sigma_radar_pos = getScalar<double>(
            meas, "radar_pos_sigma",
            getScalar<double>(meas, "radar_sigma_pos", 1.0));
        params.sigma_radar_vel = getScalar<double>(
            meas, "radar_vel_sigma",
            getScalar<double>(meas, "radar_sigma_vel", 0.5));
    }
    
    // === Radar Configuration ===
    if (config["radar"]) {
        const auto& radar = config["radar"];
        params.use_vr = getScalar<bool>(radar, "use_vr", false);
        params.radar_measurement_is_relative = getScalar<bool>(radar, "measurement_is_relative", false);
    }

    // === Reduced Filter ===
    if (config["reduced_filter"]) {
        const auto& reduced = config["reduced_filter"];
        params.sigma_target_rw = getScalar<double>(reduced, "target_rw_sigma", 1.5);
    }
    
    // === Initial Covariance ===
    if (config["initial_covariance"]) {
        const auto& init = config["initial_covariance"];
        params.init_sigma_attitude = getScalar<double>(init, "attitude", 0.05);
        params.init_sigma_position = getScalar<double>(init, "position", 3.0);
        params.init_sigma_velocity = getScalar<double>(init, "velocity", 0.5);
        params.init_sigma_pbar = getScalar<double>(init, "pbar", 0.1);
        params.init_sigma_bgyr = getScalar<double>(init, "gyro_bias", 0.005);
        params.init_sigma_bacc = getScalar<double>(init, "accel_bias", 0.05);
    }
    
    // === Camera Transform ===
    if (config["camera"] && config["camera"]["R_b2c"]) {
        params.R_b2c = parseRotationMatrix(config["camera"]["R_b2c"]);
    }
    
    // === Chi-Square Gating ===
    if (config["chi2_gating"]) {
        const auto& chi2 = config["chi2_gating"];
        params.enable_false_detection_image = getScalar<bool>(chi2, "enable_false_detection_image", true);
        params.enable_false_detection_radar = getScalar<bool>(chi2, "enable_false_detection_radar", true);
        params.chi2_threshold_image = getScalar<double>(chi2, "image_threshold", 18.42);
        params.chi2_threshold_radar = getScalar<double>(chi2, "radar_threshold", 27.86);
    }
    
    // === History Buffer ===
    if (config["history"]) {
        params.history_length = getScalar<int>(config["history"], "buffer_length", 25);
    }
    
    // === Physical Constants ===
    if (config["constants"]) {
        params.gravity = getScalar<double>(config["constants"], "gravity", 9.81);
    }
    
    // === Data Logging ===
    if (config["logging"]) {
        const auto& log = config["logging"];
        params.log_enabled = getScalar<bool>(log, "enable", false);
        params.log_rate_hz = getScalar<double>(log, "rate_hz", 20.0);
        params.log_file_path = getScalar<std::string>(log, "file_path", "log/eskf_log.csv");
    }
    
    return params;
}

void printConfig(const ESKFParams& params) {
    PRINT_INFO(BOLDCYAN "\n============ ESKF Configuration ============" RESET "\n")
    
    PRINT_INFO(BOLDWHITE "Timing:" RESET "\n")
    PRINT_INFO(CYAN "  IMU rate:      %.1f Hz" RESET "\n", 1.0 / params.dt_imu)
    PRINT_INFO(CYAN "  ESKF rate:     %.1f Hz" RESET "\n", 1.0 / params.dt_eskf)
    PRINT_INFO(CYAN "  Image delay:   %.1f ms" RESET "\n", params.image_delay * 1000)
    PRINT_INFO(CYAN "  Image timeout: %.1f s" RESET "\n", params.image_timeout_sec)
    
    PRINT_INFO(BOLDWHITE "\nIMU Noise:" RESET "\n")
    PRINT_INFO(CYAN "  sigma_omega_n: %.6f rad/s" RESET "\n", params.sigma_omega_n)
    PRINT_INFO(CYAN "  sigma_a_n:     %.6f m/s\u00b2" RESET "\n", params.sigma_a_n)
    PRINT_INFO(CYAN "  sigma_omega_w: %.6e rad/s\u221as" RESET "\n", params.sigma_omega_w)
    PRINT_INFO(CYAN "  sigma_a_w:     %.6e m/s\u00b2\u221as" RESET "\n", params.sigma_a_w)
    
    PRINT_INFO(BOLDWHITE "\nMeasurement Noise:" RESET "\n")
    PRINT_INFO(CYAN "  Image:     %.4f" RESET "\n", params.sigma_img)
    PRINT_INFO(CYAN "  Radar pos: %.2f m" RESET "\n", params.sigma_radar_pos)
    PRINT_INFO(CYAN "  Radar vel: %.2f m/s" RESET "\n", params.sigma_radar_vel)
    PRINT_INFO(CYAN "  Target rw: %.3f m/s^2" RESET "\n", params.sigma_target_rw)
    
    PRINT_INFO(BOLDWHITE "\nRadar:" RESET "\n")
    PRINT_INFO(CYAN "  Use vr:    %s" RESET "\n", params.use_vr ? GREEN "yes" RESET : RED "no" RESET)
    PRINT_INFO(CYAN "  Relative:  %s" RESET "\n", params.radar_measurement_is_relative ? GREEN "yes" RESET : RED "no" RESET)

    PRINT_INFO(BOLDWHITE "\nTopics:" RESET "\n")
    PRINT_INFO(CYAN "  IMU:             %s" RESET "\n", params.topic_imu.c_str())
    PRINT_INFO(CYAN "  Radar:           %s" RESET "\n", params.topic_radar.c_str())
    PRINT_INFO(CYAN "  Image:           %s" RESET "\n", params.topic_image.c_str())
    PRINT_INFO(CYAN "  Interceptor odom:%s" RESET "\n", params.topic_interceptor_odom.c_str())
    PRINT_INFO(CYAN "  Output odom:     %s" RESET "\n", params.topic_odom.c_str())
    PRINT_INFO(CYAN "  Output pbar:     %s" RESET "\n", params.topic_pbar.c_str())
    
    PRINT_INFO(BOLDWHITE "\nConstants:" RESET "\n")
    PRINT_INFO(CYAN "  Gravity:   %.4f m/s²" RESET "\n", params.gravity)
    
    PRINT_INFO(BOLDWHITE "\nInitial Covariance (1-sigma):" RESET "\n")
    PRINT_INFO(CYAN "  Attitude:  %.4f rad" RESET "\n", params.init_sigma_attitude)
    PRINT_INFO(CYAN "  Position:  %.2f m" RESET "\n", params.init_sigma_position)
    PRINT_INFO(CYAN "  Velocity:  %.2f m/s" RESET "\n", params.init_sigma_velocity)
    PRINT_INFO(CYAN "  Pbar:      %.4f" RESET "\n", params.init_sigma_pbar)
    PRINT_INFO(CYAN "  Gyro bias: %.5f rad/s" RESET "\n", params.init_sigma_bgyr)
    PRINT_INFO(CYAN "  Acc bias:  %.4f m/s\u00b2" RESET "\n", params.init_sigma_bacc)
    
    PRINT_INFO(BOLDWHITE "\nHistory buffer: " RESET CYAN "%d entries" RESET "\n", params.history_length)
    
    PRINT_INFO(BOLDWHITE "\nChi-Square Gating:" RESET "\n")
    PRINT_INFO(CYAN "  Image gating: %s" RESET "\n", params.enable_false_detection_image ? "enabled" : "disabled")
    PRINT_INFO(CYAN "  Radar gating: %s" RESET "\n", params.enable_false_detection_radar ? "enabled" : "disabled")
    PRINT_INFO(CYAN "  Image thresh: %.2f (2 DoF)" RESET "\n", params.chi2_threshold_image)
    PRINT_INFO(CYAN "  Radar thresh: %.2f (%s)" RESET "\n",
               params.use_vr ? params.chi2_threshold_radar : 16.27,
               params.use_vr ? "6 DoF pos+vel" : "3 DoF pos-only")
    
    PRINT_INFO(BOLDWHITE "\nData Logging:" RESET "\n")
    PRINT_INFO(CYAN "  Enabled: %s" RESET "\n", params.log_enabled ? GREEN "yes" RESET : RED "no" RESET)
    if (params.log_enabled) {
        PRINT_INFO(CYAN "  Rate:    %.1f Hz" RESET "\n", params.log_rate_hz)
        PRINT_INFO(CYAN "  File:    %s" RESET "\n", params.log_file_path.c_str())
    }
    
    PRINT_INFO(BOLDCYAN "=============================================" RESET "\n\n")
}

} // namespace eskf
