/**
 * @file eskf_logger.hpp
 * @brief CSV logger for the reduced ESKF.
 */

#pragma once

#include "eskf_cpp_reduced/eskf_types_reduced.hpp"

#include <fstream>
#include <string>

namespace eskf {

class EskfLogger {
public:
    EskfLogger();
    ~EskfLogger();

    /**
     * @brief Initialize the `.log` file for terminal output immediately on node startup.
     */
    void initTerminalLog(const std::string& log_dir);

    /**
     * @brief Start logging, scan dir for max counter, and open a new CSV.
     * @param log_dir The directory to save logs (e.g. "log/")
     * @param rate_hz The desired log rate in Hz
     */
    bool start(const std::string& log_dir, double rate_hz);

    /**
     * @brief Log a state and covariance at a given timestamp.
     * Rate limits internally based on rate_hz.
     */
    void logState(double timestamp, 
                  const reduced::NominalState& x, 
                  const reduced::ErrorCovariance& P);

    /**
     * @brief Log a radar measurement (target state) at a given timestamp.
     */
    void logRadarMeasurement(double timestamp, const reduced::RadarMeasurement& z_target);

    /**
     * @brief Log a camera/image measurement (pbar) at a given timestamp.
     */
    void logImageMeasurement(double timestamp, const Vector2d& z_pbar);

    /**
     * @brief Stop logging and close the CSV.
     */
    void stop();

    bool isLogging() const { return is_logging_; }

private:
    int findNextCounter(const std::string& log_dir);

    std::ofstream file_;
    std::ofstream radar_file_;
    std::ofstream camera_file_;
    bool is_logging_ = false;
    double log_period_ = 0.0;
    double last_log_time_ = -1.0;
    std::string current_filename_prefix_;
};

} // namespace eskf
