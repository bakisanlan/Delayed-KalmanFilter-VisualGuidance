/**
 * @file eskf_config.cpp
 * @brief Implementation of YAML configuration loader
 */

#include "eskf_cpp/eskf_config.hpp"

#include <yaml-cpp/yaml.h>
#include <iostream>
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
    
    // === Timing Parameters ===
    if (config["timing"]) {
        const auto& timing = config["timing"];
        double imu_rate = getScalar<double>(timing, "imu_rate_hz", 200.0);
        double eskf_rate = getScalar<double>(timing, "eskf_rate_hz", 200.0);
        
        params.dt_imu = 1.0 / imu_rate;
        params.dt_eskf = 1.0 / eskf_rate;
        params.image_delay = getScalar<double>(timing, "image_delay_ms", 80.0) / 1000.0;
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
        params.sigma_radar = getScalar<double>(meas, "radar_sigma", 1.0);
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
    
    // === History Buffer ===
    if (config["history"]) {
        params.history_length = getScalar<int>(config["history"], "buffer_length", 25);
    }
    
    return params;
}

void printConfig(const ESKFParams& params) {
    std::cout << "\n============ ESKF Configuration ============\n";
    std::cout << "Timing:\n";
    std::cout << "  IMU rate:    " << (1.0 / params.dt_imu) << " Hz\n";
    std::cout << "  ESKF rate:   " << (1.0 / params.dt_eskf) << " Hz\n";
    std::cout << "  Image delay: " << (params.image_delay * 1000) << " ms\n";
    
    std::cout << "\nIMU Noise (Paper Notation):\n";
    std::cout << "  σ_ωn (gyro):   " << params.sigma_omega_n << " rad/s\n";
    std::cout << "  σ_an (accel):  " << params.sigma_a_n << " m/s²\n";
    std::cout << "  σ_ωw (gyro RW):" << params.sigma_omega_w << " rad/s√s\n";
    std::cout << "  σ_aw (acc RW): " << params.sigma_a_w << " m/s²√s\n";
    
    std::cout << "\nMeasurement Noise:\n";
    std::cout << "  Image sigma:  " << params.sigma_img << "\n";
    std::cout << "  Radar sigma:  " << params.sigma_radar << " m\n";
    
    std::cout << "\nInitial Covariance (std dev):\n";
    std::cout << "  Attitude: " << params.init_sigma_attitude << " rad\n";
    std::cout << "  Position: " << params.init_sigma_position << " m\n";
    std::cout << "  Velocity: " << params.init_sigma_velocity << " m/s\n";
    std::cout << "  Pbar:     " << params.init_sigma_pbar << "\n";
    std::cout << "  Gyro bias:" << params.init_sigma_bgyr << " rad/s\n";
    std::cout << "  Acc bias: " << params.init_sigma_bacc << " m/s²\n";
    
    std::cout << "\nHistory buffer: " << params.history_length << " entries\n";
    std::cout << "=============================================\n\n";
}

} // namespace eskf
