/**
 * @file eskf_node.cpp
 * @brief ROS2 Node for Error-State Kalman Filter
 * 
 * Subscribes to:
 *   - IMU data (sensor_msgs/Imu)
 *   - Image features (geometry_msgs/Point) 
 *   - Radar measurements (geometry_msgs/Vector3)
 * 
 * Publishes:
 *   - Odometry (nav_msgs/Odometry) - pose + velocity
 *   - Pbar (geometry_msgs/Point) - image feature state
 */

#include "eskf_cpp/eskf_core.hpp"
#include "eskf_cpp/eskf_config.hpp"
#include "eskf_cpp/utils/print.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <fstream>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>

using namespace eskf;
using namespace std::chrono_literals;

class ESKFNode : public rclcpp::Node {
public:
    ESKFNode() : Node("eskf_node") {
        // Declare parameters
        this->declare_parameter("config_file", "");
        this->declare_parameter("imu_topic", "/mavros/imu/data_raw");
        this->declare_parameter("mag_topic", "/mavros/imu/mag");
        this->declare_parameter("image_topic", "/yolo/target");
        this->declare_parameter("radar_topic", "/radar/pr");
        this->declare_parameter("odom_topic", "/eskf/odom");
        this->declare_parameter("pbar_topic", "/eskf/pbar");
        this->declare_parameter("print_level", "INFO");  // ALL, DEBUG, INFO, WARNING, ERROR, SILENT
        
        // Set print level from parameter
        std::string print_level = this->get_parameter("print_level").as_string();
        Printer::setPrintLevel(print_level);
        
        // Load configuration
        std::string config_file = this->get_parameter("config_file").as_string();
        if (!config_file.empty()) {
            try {
                params_ = loadConfig(config_file);
                PRINT_INFO(GREEN "[CONFIG]: Loaded from %s" RESET "\n", config_file.c_str())
            } catch (const std::exception& e) {
                PRINT_WARNING(YELLOW "[CONFIG]: Failed to load: %s. Using defaults." RESET "\n", e.what())
            }
        }
        printConfig(params_);
        
        // Create ESKF instance
        eskf_ = std::make_unique<ErrorStateKalmanFilter>(params_);
        
        // Setup subscribers
        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        std::string mag_topic = this->get_parameter("mag_topic").as_string();
        std::string image_topic = this->get_parameter("image_topic").as_string();
        std::string radar_topic = this->get_parameter("radar_topic").as_string();
        
        // IMU subscription with sensor_data QoS (best effort) to match MAVROS
        rclcpp::QoS imu_qos(10);
        imu_qos.best_effort();  // MAVROS uses best_effort reliability
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, imu_qos, 
            std::bind(&ESKFNode::imuCallback, this, std::placeholders::_1));
        
        // Magnetometer subscription (same QoS as IMU)
        mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
            mag_topic, imu_qos,
            std::bind(&ESKFNode::magCallback, this, std::placeholders::_1));
        
        image_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            image_topic, 10,
            std::bind(&ESKFNode::imageCallback, this, std::placeholders::_1));
        
        radar_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            radar_topic, 10,
            std::bind(&ESKFNode::radarCallback, this, std::placeholders::_1));
        
        // Setup publishers
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string pbar_topic = this->get_parameter("pbar_topic").as_string();
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
        pbar_pub_ = this->create_publisher<geometry_msgs::msg::Point>(pbar_topic, 10);
        
        // Timer for diagnostics (1 Hz)
        diag_timer_ = this->create_wall_timer(
            1s, std::bind(&ESKFNode::diagnosticsCallback, this));
        
        // Compute samples per ESKF update
        samples_per_eskf_ = std::max(1, static_cast<int>(
            std::round(params_.dt_eskf / params_.dt_imu)));
        delay_steps_ = static_cast<int>(std::round(params_.image_delay / params_.dt_eskf));
        
        PRINT_INFO(BOLDCYAN "\n[ESKF]: Node initialized" RESET "\n")
        PRINT_INFO(CYAN "  IMU topic:      %s" RESET "\n", imu_topic.c_str())
        PRINT_INFO(CYAN "  Image topic:    %s" RESET "\n", image_topic.c_str())
        PRINT_INFO(CYAN "  Radar topic:    %s" RESET "\n", radar_topic.c_str())
        PRINT_INFO(CYAN "  Samples/update: %d" RESET "\n", samples_per_eskf_)
        PRINT_INFO(CYAN "  Image delay:    %d steps" RESET "\n", delay_steps_)
        PRINT_INFO(CYAN "  Image timeout:  %.1f s" RESET "\n", params_.image_timeout_sec)
        
        // Initialize logging if enabled
        if (params_.log_enabled) {
            initializeLogging();
        }
        
        PRINT_WARNING(YELLOW "[ESKF]: Waiting for first radar measurement..." RESET "\n")
    }
    
    ~ESKFNode() {
        // Close log file if open
        if (log_file_.is_open()) {
            log_file_.close();
            PRINT_INFO(GREEN "[LOG]: File closed" RESET "\n")
        }
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Log first IMU message received (once)
        if (!first_imu_received_) {
            first_imu_received_ = true;
            PRINT_INFO(GREEN "[IMU]: First message received" RESET "\n")
        }
        
        // Extract measurements and convert from FLU (MAVROS) to FRD (ESKF)
        Vector3d omega(
             msg->angular_velocity.x,
            -msg->angular_velocity.y,
            -msg->angular_velocity.z
        );
        Vector3d accel(
             msg->linear_acceleration.x,
            -msg->linear_acceleration.y,
            -msg->linear_acceleration.z
        );
        
        // Before initialization: buffer IMU for static attitude init
        if (!is_initialized_) {
            if (init_imu_buffer_.size() < static_cast<size_t>(init_buffer_size_)) {
                init_imu_buffer_.push_back({omega, accel});
                if (init_imu_buffer_.size() % 100 == 0) {
                    PRINT_INFO(CYAN "[INIT]: Buffering IMU... %zu/%d samples" RESET "\n",
                                init_imu_buffer_.size(), init_buffer_size_)
                }
            }
            return;
        }
        
        // Get timestamp
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        
        // === ZUPT Logic ===
        static bool zupt_status_logged = false;
        if (!zupt_status_logged) {
            zupt_status_logged = true;
            PRINT_INFO(CYAN "[ZUPT]: Status = %s" RESET "\n", 
                        eskf_->isZUPTEnabled() ? GREEN "enabled" RESET : RED "disabled" RESET)
        }
        
        if (eskf_->isZUPTEnabled()) {
            if (eskf_->detectZUPT(omega, accel)) {
                Vector6d innovation = eskf_->correctZUPT(omega, accel);
                static int zupt_log_counter = 0;
                if (++zupt_log_counter % 200 == 0) {
                    PRINT_INFO(MAGENTA "[ZUPT]: Correction applied - a=[%.3f,%.3f,%.3f] w=[%.4f,%.4f,%.4f]" RESET "\n",
                        innovation(0), innovation(1), innovation(2),
                        innovation(3), innovation(4), innovation(5))
                }
            } else {
                if (eskf_->wasRadarReceived() && eskf_->wasZUPTTriggered()) {
                    eskf_->disableZUPT();
                    PRINT_INFO(GREEN "[ZUPT]: Disabled - platform moving" RESET "\n")
                }
            }
        }
        
        // Accumulate IMU measurements
        eskf_->accumulateIMU(omega, accel);
        imu_count_++;
        
        // Run prediction when enough samples accumulated
        if (imu_count_ >= samples_per_eskf_) {
            IMUMeasurement avg = eskf_->getAveragedIMU();
            eskf_->predict(avg.omega, avg.accel, timestamp);
            imu_count_ = 0;
            prediction_count_++;  // For Hz tracking
            
            // Check image timeout
            if (last_image_time_ > 0.0) {
                double time_since_image = timestamp - last_image_time_;
                if (time_since_image > params_.image_timeout_sec && !eskf_->isPbarFrozen()) {
                    eskf_->setPbarFrozen(true);
                    PRINT_WARNING(YELLOW "[IMAGE]: Timeout (%.1fs) - pbar frozen" RESET "\n", time_since_image)
                }
            }
            
            publishState(msg->header.stamp);
        }
    }
    
    void imageCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        if (!first_image_received_) {
            first_image_received_ = true;
            PRINT_INFO(GREEN "[IMAGE]: First message - pbar=[%.4f, %.4f]" RESET "\n", msg->x, msg->y)
        }
        
        if (!is_initialized_) {
            return;
        }
        
        double timestamp = this->now().seconds();
        if (eskf_->isPbarFrozen()) {
            eskf_->setPbarFrozen(false);
            PRINT_INFO(GREEN "[IMAGE]: Resumed after timeout" RESET "\n")
        }
        last_image_time_ = timestamp;
        
        Vector2d z_pbar(msg->x, msg->y);
        Vector2d innovation = eskf_->correctImage(z_pbar, delay_steps_);
        
        if (innovation.norm() > 0) {
            PRINT_DEBUG(CYAN "[IMAGE]: Innovation = [%.4f, %.4f]" RESET "\n", innovation(0), innovation(1))
        }
    }
    
    void radarCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        Vector6d z_radar;
        z_radar.head<3>() << msg->x, msg->y, msg->z;
        z_radar.tail<3>().setZero();
        
        if (!is_initialized_) {
            initializeFilter(z_radar);
            return;
        }
        
        eskf_->notifyRadarReceived();
        Vector6d innovation = eskf_->correctRadar(z_radar);
        
        PRINT_DEBUG(BLUE "[RADAR]: Innovation pos=[%.3f,%.3f,%.3f] vel=[%.3f,%.3f,%.3f]" RESET "\n",
                    innovation(0), innovation(1), innovation(2),
                    innovation(3), innovation(4), innovation(5))
    }
    
    void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
        // Skip if magnetometer is disabled
        if (!params_.enable_mag) {
            return;
        }
        
        if (!first_mag_received_) {
            first_mag_received_ = true;
            double mag_uT = std::sqrt(msg->magnetic_field.x * msg->magnetic_field.x +
                                      msg->magnetic_field.y * msg->magnetic_field.y +
                                      msg->magnetic_field.z * msg->magnetic_field.z) * 1e-3;
            PRINT_INFO(GREEN "[MAG]: First message - magnitude %.2f \u03bcT (normalized for ESKF)" RESET "\n", mag_uT)
        }
        
        if (!is_initialized_) {
            return;
        }
        
        // Convert FLU (MAVROS) to FRD (ESKF): x stays, negate y and z
        Vector3d z_mag_raw(
             msg->magnetic_field.x,
            -msg->magnetic_field.y,
            -msg->magnetic_field.z
        );
        
        // Normalize to unit vector (magnitude-independent)
        double mag_norm = z_mag_raw.norm();
        if (mag_norm < 1e-6) {
            return;  // Invalid measurement
        }
        Vector3d z_mag = z_mag_raw / mag_norm;  // Unit vector
        
        Vector3d innovation = eskf_->correctMag(z_mag);
        
        static int mag_log_counter = 0;
        if (++mag_log_counter % 100 == 0 && innovation.norm() > 0) {
            Vector3d bmag = eskf_->getMagBias();
            PRINT_DEBUG(MAGENTA "[MAG]: Innovation=[%.4f,%.4f,%.4f], bias=[%.4f,%.4f,%.4f]" RESET "\n",
                       innovation(0), innovation(1), innovation(2),
                       bmag(0), bmag(1), bmag(2))
        }
    }
    
    void initializeFilter(const Vector6d& z_radar) {
        PRINT_INFO(BOLDGREEN "\n========== ESKF INITIALIZATION ==========" RESET "\n")
        PRINT_INFO(CYAN "[RADAR]: First measurement received" RESET "\n")
        PRINT_INFO(CYAN "  Position: [%.2f, %.2f, %.2f]" RESET "\n", z_radar(0), z_radar(1), z_radar(2))
        PRINT_INFO(CYAN "  Velocity: [%.2f, %.2f, %.2f]" RESET "\n", z_radar(3), z_radar(4), z_radar(5))
        
        Quaterniond q_init = computeInitialAttitude();
        
        NominalState x_init = NominalState::Zero();
        x_init(0) = q_init.w();
        x_init(1) = q_init.x();
        x_init(2) = q_init.y();
        x_init(3) = q_init.z();
        x_init.segment<3>(nominal_idx::PR_START) = z_radar.head<3>();
        x_init.segment<3>(nominal_idx::VR_START) = z_radar.tail<3>();
        x_init.segment<2>(nominal_idx::PBAR_START).setZero();
        x_init.segment<3>(nominal_idx::BGYR_START).setZero();
        x_init.segment<3>(nominal_idx::BACC_START).setZero();
        x_init.segment<3>(nominal_idx::BMAG_START).setZero();  // Initialize mag bias to zero
        
        eskf_->reset(x_init, eskf_->getCovariance());
        is_initialized_ = true;
        eskf_->notifyRadarReceived();
        init_imu_buffer_.clear();
        
        PRINT_INFO(BOLDGREEN "[ESKF]: Filter RUNNING" RESET "\n")
        PRINT_INFO(BOLDGREEN "========================================" RESET "\n\n")
    }
    
    Quaterniond computeInitialAttitude() {
        if (init_imu_buffer_.empty()) {
            PRINT_WARNING(YELLOW "[INIT]: No IMU samples - using identity" RESET "\n")
            return Quaterniond::Identity();
        }
        
        Vector3d accel_avg = Vector3d::Zero();
        for (const auto& sample : init_imu_buffer_) {
            accel_avg += sample.accel;
        }
        accel_avg /= static_cast<double>(init_imu_buffer_.size());
        Vector3d gravity_measured = -accel_avg;
        
        double variance = 0.0;
        for (const auto& sample : init_imu_buffer_) {
            variance += (sample.accel - accel_avg).squaredNorm();
        }
        variance /= static_cast<double>(init_imu_buffer_.size());
        
        PRINT_INFO(CYAN "[INIT]: %zu samples, variance=%.4f" RESET "\n", init_imu_buffer_.size(), variance)
        PRINT_INFO(CYAN "  Gravity: [%.3f, %.3f, %.3f]" RESET "\n",
                    gravity_measured(0), gravity_measured(1), gravity_measured(2))
        
        if (variance > 1.0) {
            PRINT_WARNING(YELLOW "[INIT]: High variance - platform may not be stationary!" RESET "\n")
        }
        
        Vector3d g = gravity_measured.normalized();
        double roll = std::atan2(g(1), g(2));
        double pitch = std::atan2(-g(0), std::sqrt(g(1)*g(1) + g(2)*g(2)));
        double yaw = 0.0;
        
        PRINT_INFO(GREEN "[INIT]: Attitude = roll:" CYAN "%.1f" GREEN "° pitch:" CYAN "%.1f" GREEN "° yaw:" CYAN "%.1f" GREEN "°" RESET "\n",
                    roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI)
        
        Eigen::AngleAxisd rollAngle(roll, Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Vector3d::UnitZ());
        
        Quaterniond q = yawAngle * pitchAngle * rollAngle;
        q.normalize();
        
        return q;
    }
    
    void publishState(const rclcpp::Time& stamp) {
        // Get state
        Quaterniond q = eskf_->getQuaternion();
        Vector3d pos = eskf_->getPosition();
        Vector3d vel = eskf_->getVelocity();
        Vector2d pbar = eskf_->getPbar();
        Vector3d bgyr = eskf_->getGyroBias();
        Vector3d bacc = eskf_->getAccelBias();
        Vector3d bmag = eskf_->getMagBias();
        auto cov_diag = eskf_->getCovarianceDiagonal();
        
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = "world";
        odom_msg.child_frame_id = "base_link";
        
        // Pose
        odom_msg.pose.pose.position.x = pos(0);
        odom_msg.pose.pose.position.y = pos(1);
        odom_msg.pose.pose.position.z = pos(2);
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        
        // Pose covariance (6x6: position then orientation)
        for (int i = 0; i < 3; ++i) {
            odom_msg.pose.covariance[i * 7] = cov_diag(error_idx::DPR_START + i);
        }
        for (int i = 0; i < 3; ++i) {
            odom_msg.pose.covariance[(i + 3) * 7] = cov_diag(error_idx::DTHETA_START + i);
        }
        
        // Twist (velocity in body frame)
        odom_msg.twist.twist.linear.x = vel(0);
        odom_msg.twist.twist.linear.y = vel(1);
        odom_msg.twist.twist.linear.z = vel(2);
        // Angular velocity not estimated directly, set to zero
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;
        
        // Twist covariance (6x6: linear then angular)
        for (int i = 0; i < 3; ++i) {
            odom_msg.twist.covariance[i * 7] = cov_diag(error_idx::DVR_START + i);
        }
        
        odom_pub_->publish(odom_msg);
        
        // === Publish Pbar ===
        auto pbar_msg = geometry_msgs::msg::Point();
        pbar_msg.x = pbar(0);
        pbar_msg.y = pbar(1);
        pbar_msg.z = 0.0;  // 2D state
        
        pbar_pub_->publish(pbar_msg);
        
        // === CSV Logging (rate-limited) ===
        if (log_file_.is_open()) {
            log_skip_counter_++;
            if (log_skip_counter_ >= log_skip_samples_) {
                log_skip_counter_ = 0;
                
                double timestamp = stamp.seconds();
                Vector3d euler_rad = q.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX order
                Vector3d euler_deg = euler_rad * 180.0 / M_PI;
                for (int i = 0; i < 3; ++i) {
                    if (euler_deg(i) > 180.0) euler_deg(i) -= 360.0;
                    if (euler_deg(i) < -180.0) euler_deg(i) += 360.0;
                }
                
                // Write: timestamp, roll, pitch, yaw, x, y, z, vx, vy, vz, pbar_x, pbar_y,
                //        bgyr (3), bacc (3), bmag (3), P_dtheta (3), P_dpos (3), P_dvel (3), P_dpbar (2), P_dbgyr (3), P_dbacc (3), P_dbmag (3)
                                
                log_file_ << std::fixed << std::setprecision(6)
                          << timestamp << ","
                          << euler_deg(0) << "," << euler_deg(1) << "," << euler_deg(2) << ","
                          << pos(0) << "," << pos(1) << "," << pos(2) << ","
                          << vel(0) << "," << vel(1) << "," << vel(2) << ","
                          << pbar(0) << "," << pbar(1) << ","
                          << bgyr(0) << "," << bgyr(1) << "," << bgyr(2) << ","
                          << bacc(0) << "," << bacc(1) << "," << bacc(2) << ","
                          << bmag(0) << "," << bmag(1) << "," << bmag(2);
                
                // Covariance diagonal (all 20 error states)
                for (int i = 0; i < 20; ++i) {
                    log_file_ << "," << cov_diag(i);
                }
                log_file_ << "\n";  // No flush for performance
            }
        }
    }
    
    void diagnosticsCallback() {
        if (!is_initialized_) {
            static int diag_wait_counter = 0;
            if (++diag_wait_counter % 5 == 0) {
                PRINT_WARNING(YELLOW "[ESKF]: Waiting for radar..." RESET "\n")
            }
            return;
        }
        
        // Calculate actual ESKF Hz (predictions per second)
        double current_hz = static_cast<double>(prediction_count_);
        prediction_count_ = 0;  // Reset counter for next second
        
        auto P_diag = eskf_->getCovarianceDiagonal();
        double P_att = P_diag.segment<3>(error_idx::DTHETA_START).sum();
        double P_pos = P_diag.segment<3>(error_idx::DPR_START).sum();
        double P_vel = P_diag.segment<3>(error_idx::DVR_START).sum();
        double P_pbar = P_diag.segment<2>(error_idx::DPBAR_START).sum();
        
        Quaterniond q = eskf_->getQuaternion();
        Vector3d pos = eskf_->getPosition();
        Vector3d vel = eskf_->getVelocity();
        Vector2d pbar = eskf_->getPbar();
        
        Vector3d euler_rad = q.toRotationMatrix().eulerAngles(2, 1, 0);
        Vector3d euler_deg = euler_rad * 180.0 / M_PI;
        for (int i = 0; i < 3; ++i) {
            if (euler_deg(i) > 180.0) euler_deg(i) -= 360.0;
            if (euler_deg(i) < -180.0) euler_deg(i) += 360.0;
        }
        
        PRINT_INFO(BOLDCYAN "[STATE]:" RESET 
            " att=" CYAN "[%.1f,%.1f,%.1f]°" RESET 
            " pos=" GREEN "[%.2f,%.2f,%.2f]m" RESET 
            " vel=" BLUE "[%.2f,%.2f,%.2f]m/s" RESET 
            " pbar=" MAGENTA "[%.4f,%.4f]" RESET "\n",
            euler_deg(0), euler_deg(1), euler_deg(2),
            pos(0), pos(1), pos(2),
            vel(0), vel(1), vel(2),
            pbar(0), pbar(1))
        
        PRINT_INFO(BOLDCYAN "[COV]:" RESET 
            " att=" CYAN "%.4f" RESET 
            " pos=" GREEN "%.4f" RESET 
            " vel=" BLUE "%.4f" RESET 
            " pbar=" MAGENTA "%.6f" RESET 
            " | " BOLDREDPURPLE "%.0f Hz" RESET "\n",
            P_att, P_pos, P_vel, P_pbar, current_hz)
    }
    
    // Members
    ESKFParams params_;
    std::unique_ptr<ErrorStateKalmanFilter> eskf_;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr radar_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pbar_pub_;
    rclcpp::TimerBase::SharedPtr diag_timer_;
    
    bool is_initialized_ = false;
    int imu_count_ = 0;
    int samples_per_eskf_ = 1;
    int delay_steps_ = 0;
    bool first_imu_received_ = false;
    bool first_mag_received_ = false;
    bool first_image_received_ = false;
    double last_image_time_ = 0.0;
    int prediction_count_ = 0;  // Counter for Hz measurement
    
    // CSV Logging
    std::ofstream log_file_;
    int log_skip_counter_ = 0;
    int log_skip_samples_ = 1;  // Computed from rate
    
    void initializeLogging() {
        // Compute skip samples: eskf_rate / log_rate
        double eskf_rate = 1.0 / params_.dt_eskf;
        log_skip_samples_ = std::max(1, static_cast<int>(std::round(eskf_rate / params_.log_rate_hz)));
        
        // Generate timestamp-based filename
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        std::tm* tm_now = std::localtime(&time_t_now);
        
        std::ostringstream filename_ss;
        filename_ss << "/home/ituarc/ros2_ws/src/eskf_cpp/log/eskf_" 
                    << std::put_time(tm_now, "%Y%m%d_%H%M%S")
                    << ".csv";
        std::string log_filename = filename_ss.str();
        
        // Create directory if it doesn't exist (under package directory)
        std::filesystem::path log_path(log_filename);
        if (log_path.has_parent_path()) {
            std::filesystem::create_directories(log_path.parent_path());
        }
        
        // Open file
        log_file_.open(log_filename);
        if (!log_file_.is_open()) {
            PRINT_ERROR(RED "[LOG]: Failed to open %s" RESET "\n", log_filename.c_str())
            return;
        }
        
        // Write CSV header
        log_file_ << "timestamp,roll_deg,pitch_deg,yaw_deg,x,y,z,vx,vy,vz,pbar_x,pbar_y,"
                  << "bgyr_x,bgyr_y,bgyr_z,bacc_x,bacc_y,bacc_z,bmag_x,bmag_y,bmag_z,"
                  << "P_dtheta_x,P_dtheta_y,P_dtheta_z,"
                  << "P_dpos_x,P_dpos_y,P_dpos_z,"
                  << "P_dvel_x,P_dvel_y,P_dvel_z,"
                  << "P_dpbar_x,P_dpbar_y,"
                  << "P_dbgyr_x,P_dbgyr_y,P_dbgyr_z,"
                  << "P_dbacc_x,P_dbacc_y,P_dbacc_z,"
                  << "P_dbmag_x,P_dbmag_y,P_dbmag_z\n";
        
        double actual_log_rate = eskf_rate / log_skip_samples_;
        PRINT_INFO(GREEN "[LOG]: Logging to %s @ %.1f Hz" RESET "\n",
                   log_filename.c_str(), actual_log_rate)
    }
    
    struct IMUSample {
        Vector3d omega;
        Vector3d accel;
    };
    std::vector<IMUSample> init_imu_buffer_;
    int init_buffer_size_ = 400;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ESKFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
