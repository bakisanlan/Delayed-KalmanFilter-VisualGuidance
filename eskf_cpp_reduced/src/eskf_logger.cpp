/**
 * @file eskf_logger.cpp
 * @brief Implementation of the CSV logger.
 */

#include "eskf_cpp_reduced/eskf_logger.hpp"
#include "eskf_cpp_reduced/utils/print.hpp"

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <regex>

namespace eskf {

EskfLogger::EskfLogger() = default;

EskfLogger::~EskfLogger() {
    stop();
}

void EskfLogger::initTerminalLog(const std::string& log_dir) {
    if (!current_filename_prefix_.empty()) return;

    std::filesystem::path dir_path(log_dir);
    if (!std::filesystem::exists(dir_path)) {
        try {
            std::filesystem::create_directories(dir_path);
        } catch (const std::filesystem::filesystem_error& e) {
            PRINT_ERROR(RED "[LOGGER]: Failed to create log directory '%s': %s" RESET "\n",
                        log_dir.c_str(), e.what())
            return;
        }
    }

    int next_counter = findNextCounter(log_dir);

    std::time_t t = std::time(nullptr);
    std::tm* tm = std::localtime(&t);
    std::ostringstream time_stream;
    time_stream << std::put_time(tm, "%Y%m%d_%H%M%S");

    std::ostringstream filename_base;
    filename_base << std::setw(4) << std::setfill('0') << next_counter
                  << "_" << time_stream.str();

    current_filename_prefix_ = filename_base.str();

    std::filesystem::path log_path = dir_path / (current_filename_prefix_ + ".log");
    eskf::Printer::setLogFile(log_path.string());
}

bool EskfLogger::start(const std::string& log_dir, double rate_hz) {
    if (is_logging_) {
        return true;
    }

    log_period_ = (rate_hz > 0.0) ? (1.0 / rate_hz) : 0.0;
    last_log_time_ = -1.0;

    std::filesystem::path dir_path(log_dir);

    if (current_filename_prefix_.empty()) {
        initTerminalLog(log_dir); 
    }

    std::string csv_filename = current_filename_prefix_ + ".csv";
    std::filesystem::path file_path = dir_path / csv_filename;

    file_.open(file_path.string(), std::ios::out | std::ios::trunc);
    if (!file_.is_open()) {
        PRINT_ERROR(RED "[LOGGER]: Failed to open CSV file '%s'" RESET "\n", file_path.string().c_str())
        return false;
    }

    // Write header
    file_ << "timestamp,"
          << "pt_x,pt_y,pt_z,"
          << "vt_x,vt_y,vt_z,"
          << "pbar_x,pbar_y,"
          << "P_pt_x,P_pt_y,P_pt_z,"
          << "P_vt_x,P_vt_y,P_vt_z,"
          << "P_pbar_x,P_pbar_y\n";

    // Open radar measurement CSV
    std::string radar_csv = current_filename_prefix_ + "_radar.csv";
    std::filesystem::path radar_path = dir_path / radar_csv;
    radar_file_.open(radar_path.string(), std::ios::out | std::ios::trunc);
    if (radar_file_.is_open()) {
        radar_file_ << "timestamp,"
                    << "meas_pos_n,meas_pos_e,meas_pos_d,"
                    << "meas_vel_n,meas_vel_e,meas_vel_d\n";
    }

    // Open camera measurement CSV
    std::string camera_csv = current_filename_prefix_ + "_camera.csv";
    std::filesystem::path camera_path = dir_path / camera_csv;
    camera_file_.open(camera_path.string(), std::ios::out | std::ios::trunc);
    if (camera_file_.is_open()) {
        camera_file_ << "timestamp,"
                     << "meas_pbar_x,meas_pbar_y\n";
    }

    is_logging_ = true;
    PRINT_INFO(GREEN "[LOGGER]: Started logging to '%s' (Rate: %.1f Hz)" RESET "\n",
               file_path.string().c_str(), rate_hz)
    return true;
}

void EskfLogger::logState(double timestamp,
                          const reduced::NominalState& x,
                          const reduced::ErrorCovariance& P) {
    if (!is_logging_ || !file_.is_open()) {
        return;
    }

    // Rate limiting
    if (log_period_ > 0.0 && last_log_time_ >= 0.0 && (timestamp - last_log_time_) < log_period_) {
        return;
    }

    last_log_time_ = timestamp;

    const auto p_t = reduced::state_access::getPosition(x);
    const auto v_t = reduced::state_access::getVelocity(x);
    const auto pbar = reduced::state_access::getPbar(x);
    const auto P_diag = P.diagonal();

    // 1 (time) + 3 (pt) + 3 (vt) + 2 (pbar) 
    // + 3 (P_pt) + 3 (P_vt) + 2 (P_pbar)
    file_ << std::fixed << std::setprecision(6) << timestamp << ","
          << p_t(0) << "," << p_t(1) << "," << p_t(2) << ","
          << v_t(0) << "," << v_t(1) << "," << v_t(2) << ","
          << pbar(0) << "," << pbar(1) << ","
          << P_diag(reduced::error_idx::DPT_START + 0) << ","
          << P_diag(reduced::error_idx::DPT_START + 1) << ","
          << P_diag(reduced::error_idx::DPT_START + 2) << ","
          << P_diag(reduced::error_idx::DVT_START + 0) << ","
          << P_diag(reduced::error_idx::DVT_START + 1) << ","
          << P_diag(reduced::error_idx::DVT_START + 2) << ","
          << P_diag(reduced::error_idx::DPBAR_START + 0) << ","
          << P_diag(reduced::error_idx::DPBAR_START + 1) << "\n";
}

void EskfLogger::logRadarMeasurement(double timestamp,
                                      const reduced::RadarMeasurement& z_target) {
    if (!is_logging_ || !radar_file_.is_open()) {
        return;
    }

    radar_file_ << std::fixed << std::setprecision(6) << timestamp << ","
                << z_target(0) << "," << z_target(1) << "," << z_target(2) << ","
                << z_target(3) << "," << z_target(4) << "," << z_target(5) << "\n";
}

void EskfLogger::logImageMeasurement(double timestamp,
                                      const Vector2d& z_pbar) {
    if (!is_logging_ || !camera_file_.is_open()) {
        return;
    }

    camera_file_ << std::fixed << std::setprecision(6) << timestamp << ","
                 << z_pbar(0) << "," << z_pbar(1) << "\n";
}

void EskfLogger::stop() {
    if (is_logging_ && file_.is_open()) {
        file_.flush();
        file_.close();
        PRINT_INFO(GREEN "[LOGGER]: Stopped logging and closed CSV" RESET "\n")
        eskf::Printer::closeLogFile();
    }
    if (radar_file_.is_open()) {
        radar_file_.flush();
        radar_file_.close();
    }
    if (camera_file_.is_open()) {
        camera_file_.flush();
        camera_file_.close();
    }
    is_logging_ = false;
    last_log_time_ = -1.0;
}

int EskfLogger::findNextCounter(const std::string& log_dir) {
    int counter = 1;
    std::filesystem::path counter_file = std::filesystem::path(log_dir) / "log_counter.txt";

    // Read the current counter
    if (std::filesystem::exists(counter_file)) {
        std::ifstream in(counter_file.string());
        if (in.is_open()) {
            in >> counter;
            in.close();
        } else {
            PRINT_WARNING(YELLOW "[LOGGER]: Error reading log_counter.txt" RESET "\n")
        }
    }

    // Increment and write the next counter back to the file
    std::ofstream out(counter_file.string(), std::ios::trunc);
    if (out.is_open()) {
        out << std::setw(4) << std::setfill('0') << (counter + 1) << "\n";
        out.close();
    } else {
        PRINT_WARNING(YELLOW "[LOGGER]: Error writing to log_counter.txt" RESET "\n")
    }

    return counter;
}

} // namespace eskf
