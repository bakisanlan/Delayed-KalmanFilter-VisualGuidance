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
 *   - ESKF state (eskf_cpp/msg/ESKFState)
 *   - Pose with covariance (geometry_msgs/PoseWithCovarianceStamped)
 */

#include "eskf_cpp/eskf_core.hpp"
#include "eskf_cpp/eskf_config.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// Note: Custom message would be included if generated
// #include "eskf_cpp/msg/eskf_state.hpp"

using namespace eskf;
using namespace std::chrono_literals;

class ESKFNode : public rclcpp::Node {
public:
    ESKFNode() : Node("eskf_node") {
        // Declare parameters
        this->declare_parameter("config_file", "");
        this->declare_parameter("imu_topic", "/mavros/imu/data_raw");
        this->declare_parameter("image_topic", "/yolo/target");
        this->declare_parameter("radar_topic", "/radar/pr");
        this->declare_parameter("pose_topic", "/eskf/pose");
        
        // Load configuration
        std::string config_file = this->get_parameter("config_file").as_string();
        if (!config_file.empty()) {
            try {
                params_ = loadConfig(config_file);
                RCLCPP_INFO(this->get_logger(), "Loaded config from: %s", config_file.c_str());
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to load config: %s. Using defaults.", e.what());
            }
        }
        printConfig(params_);
        
        // Create ESKF instance
        eskf_ = std::make_unique<ErrorStateKalmanFilter>(params_);
        
        // Setup subscribers
        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        std::string image_topic = this->get_parameter("image_topic").as_string();
        std::string radar_topic = this->get_parameter("radar_topic").as_string();
        
        // IMU subscription with sensor_data QoS (best effort) to match MAVROS
        rclcpp::QoS imu_qos(10);
        imu_qos.best_effort();  // MAVROS uses best_effort reliability
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, imu_qos, 
            std::bind(&ESKFNode::imuCallback, this, std::placeholders::_1));
        
        image_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            image_topic, 10,
            std::bind(&ESKFNode::imageCallback, this, std::placeholders::_1));
        
        radar_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            radar_topic, 10,
            std::bind(&ESKFNode::radarCallback, this, std::placeholders::_1));
        
        // Setup publishers
        std::string pose_topic = this->get_parameter("pose_topic").as_string();
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic, 10);
        
        // Timer for diagnostics (1 Hz)
        diag_timer_ = this->create_wall_timer(
            1s, std::bind(&ESKFNode::diagnosticsCallback, this));
        
        // Compute samples per ESKF update
        samples_per_eskf_ = std::max(1, static_cast<int>(
            std::round(params_.dt_eskf / params_.dt_imu)));
        delay_steps_ = static_cast<int>(std::round(params_.image_delay / params_.dt_eskf));
        
        RCLCPP_INFO(this->get_logger(), "ESKF Node created (waiting for first radar measurement to initialize)");
        RCLCPP_INFO(this->get_logger(), "  IMU topic: %s", imu_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Image topic: %s", image_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Radar topic: %s", radar_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Samples per ESKF: %d", samples_per_eskf_);
        RCLCPP_INFO(this->get_logger(), "  Image delay steps: %d", delay_steps_);
        RCLCPP_WARN(this->get_logger(), "Filter NOT initialized - waiting for first radar measurement...");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Log first IMU message received (once)
        if (!first_imu_received_) {
            first_imu_received_ = true;
            RCLCPP_INFO(this->get_logger(), "First IMU message received");
        }
        
        // Extract measurements and convert from FLU (MAVROS) to FRD (ESKF)
        // FLU: Front-Left-Up (MAVROS convention)
        // FRD: Front-Right-Down (ESKF convention)
        // Conversion: X stays same, Y negated, Z negated
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
                    RCLCPP_INFO(this->get_logger(), "Collecting IMU for static init: %zu/%d samples",
                                init_imu_buffer_.size(), init_buffer_size_);
                }
            }
            return;
        }
        
        // Get timestamp
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        
        // === ZUPT Logic ===
        // Try ZUPT while enabled (before prediction to initialize biases)
        static bool zupt_status_logged = false;
        if (!zupt_status_logged) {
            zupt_status_logged = true;
            RCLCPP_INFO(this->get_logger(), "ZUPT status: enabled=%s", 
                        eskf_->isZUPTEnabled() ? "true" : "false");
        }
        
        if (eskf_->isZUPTEnabled()) {
            if (eskf_->detectZUPT(omega, accel)) {
                // Platform is stationary - apply ZUPT correction
                Vector6d innovation = eskf_->correctZUPT(omega, accel);
                RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(), 1000, "ZUPT correction applied: accel=[%.4f,%.4f,%.4f], gyro=[%.4f,%.4f,%.4f]",
                    innovation(0), innovation(1), innovation(2),
                    innovation(3), innovation(4), innovation(5));
            } else {
                // Movement detected - check if we should permanently disable ZUPT
                // Disable if: radar received AND ZUPT worked at least once AND now moving
                if (eskf_->wasRadarReceived() && eskf_->wasZUPTTriggered()) {
                    eskf_->disableZUPT();
                    RCLCPP_INFO(this->get_logger(), 
                        "ZUPT permanently disabled - platform is moving");
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(), 1000, 
                        "ZUPT not disabled yet: radarReceived=%s, zuptTriggered=%s",
                        eskf_->wasRadarReceived() ? "true" : "false",
                        eskf_->wasZUPTTriggered() ? "true" : "false");
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
            
            // Publish pose
            publishPose(msg->header.stamp);
        }
    }
    
    void imageCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        // Log first image message received (once)
        if (!first_image_received_) {
            first_image_received_ = true;
            RCLCPP_INFO(this->get_logger(), "First image message received: pbar=[%.4f, %.4f]",
                        msg->x, msg->y);
        }
        
        // Don't process image until filter is initialized
        if (!is_initialized_) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Ignoring image measurement - filter not initialized");
            return;
        }
        
        // Convert to pbar measurement
        Vector2d z_pbar(msg->x, msg->y);
        
        // Apply correction with delay
        Vector2d innovation = eskf_->correctImage(z_pbar, delay_steps_);
        
        if (innovation.norm() > 0) {
            RCLCPP_DEBUG(this->get_logger(), "Image correction: innovation = [%.4f, %.4f]",
                        innovation(0), innovation(1));
        }
    }
    
    void radarCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        // Convert to radar measurement (position + velocity = 6D)
        Vector6d z_radar;
        z_radar.head<3>() << msg->x, msg->y, msg->z;
        z_radar.tail<3>().setZero();  // Velocity measurement not available from this topic
        
        // Initialize filter on first radar measurement
        if (!is_initialized_) {
            initializeFilter(z_radar);
            return;
        }
        
        // Notify filter that radar has been received (for ZUPT disable logic)
        eskf_->notifyRadarReceived();
        
        // Apply correction (no delay)
        Vector6d innovation = eskf_->correctRadar(z_radar);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Radar correction: pos innovation = [%.3f, %.3f, %.3f], vel innovation = [%.3f, %.3f, %.3f]",
                    innovation(0), innovation(1), innovation(2),
                    innovation(3), innovation(4), innovation(5));
    }
    
    void initializeFilter(const Vector6d& z_radar) {
        RCLCPP_INFO(this->get_logger(), "=== INITIALIZING ESKF ===");
        RCLCPP_INFO(this->get_logger(), "First radar measurement received:");
        RCLCPP_INFO(this->get_logger(), "  Position: [%.2f, %.2f, %.2f]", 
                    z_radar(0), z_radar(1), z_radar(2));
        RCLCPP_INFO(this->get_logger(), "  Velocity: [%.2f, %.2f, %.2f]", 
                    z_radar(3), z_radar(4), z_radar(5));
        
        // Compute initial attitude from buffered IMU (gravity alignment)
        Quaterniond q_init = computeInitialAttitude();
        
        // Create initial state from radar measurement
        NominalState x_init = NominalState::Zero();
        
        // Set quaternion from gravity alignment
        x_init(0) = q_init.w();
        x_init(1) = q_init.x();
        x_init(2) = q_init.y();
        x_init(3) = q_init.z();
        
        // Set position from radar
        x_init.segment<3>(nominal_idx::PR_START) = z_radar.head<3>();
        
        // Set velocity from radar (currently zero)
        x_init.segment<3>(nominal_idx::VR_START) = z_radar.tail<3>();
        
        // Set pbar to zero (will be corrected by first image measurement)
        x_init.segment<2>(nominal_idx::PBAR_START).setZero();
        
        // Biases start at zero
        x_init.segment<3>(nominal_idx::BGYR_START).setZero();
        x_init.segment<3>(nominal_idx::BACC_START).setZero();
        
        // Reset filter with initial state
        eskf_->reset(x_init, eskf_->getCovariance());
        
        // Mark as initialized
        is_initialized_ = true;
        
        // Notify filter that radar has been received (for ZUPT disable logic)
        eskf_->notifyRadarReceived();
        
        // Clear IMU buffer
        init_imu_buffer_.clear();
        
        RCLCPP_INFO(this->get_logger(), "=== ESKF INITIALIZED ===");
        RCLCPP_INFO(this->get_logger(), "Filter is now running. Processing IMU and sensor data.");
        RCLCPP_INFO(this->get_logger(), "ZUPT is enabled - will attempt stationary detection.");
    }
    
    /**
     * @brief Compute initial attitude from buffered IMU samples using gravity
     * 
     * Estimates roll and pitch by aligning measured gravity with expected gravity.
     * Yaw cannot be determined from gravity alone, so it's set to zero.
     */
    Quaterniond computeInitialAttitude() {
        if (init_imu_buffer_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No IMU samples for static init, using identity");
            return Quaterniond::Identity();
        }
        
        // Average accelerometer readings
        // Note: Accelerometer measures specific force: a_meas = a_true - g
        // When stationary: a_meas = -g, so gravity_body = -a_meas
        Vector3d accel_avg = Vector3d::Zero();
        for (const auto& sample : init_imu_buffer_) {
            accel_avg += sample.accel;
        }
        accel_avg /= static_cast<double>(init_imu_buffer_.size());
        
        // Gravity is negative of accelerometer reading when stationary
        Vector3d gravity_measured = -accel_avg;
        
        // Check variance to verify stationary
        double variance = 0.0;
        for (const auto& sample : init_imu_buffer_) {
            variance += (sample.accel - accel_avg).squaredNorm();
        }
        variance /= static_cast<double>(init_imu_buffer_.size());
        
        RCLCPP_INFO(this->get_logger(), "Static init: %zu samples, variance=%.4f",
                    init_imu_buffer_.size(), variance);
        RCLCPP_INFO(this->get_logger(), "  accel_avg=[%.3f,%.3f,%.3f] -> gravity_body=[%.3f,%.3f,%.3f]",
                    accel_avg(0), accel_avg(1), accel_avg(2),
                    gravity_measured(0), gravity_measured(1), gravity_measured(2));
        
        if (variance > 1.0) {
            RCLCPP_WARN(this->get_logger(), "High variance during static init - platform may not be stationary!");
        }
        
        // Normalize gravity measurement
        Vector3d g = gravity_measured.normalized();
        
        /* ====== ALTERNATIVE: Rodrigues Formula (commented out) ======
         * This aligns vectors but can give ambiguous rotations for pure pitch/roll
         *
        Vector3d z_global(0.0, 0.0, 1.0);  // Expected gravity in FRD frame
        Vector3d v = g.cross(z_global);
        double c = g.dot(z_global);
        double s = v.norm();
        
        Eigen::Matrix3d R;
        if (s < 1e-6) {
            if (c > 0) {
                R = Eigen::Matrix3d::Identity();
            } else {
                R << 1, 0, 0, 0, -1, 0, 0, 0, -1;
            }
        } else {
            Eigen::Matrix3d vx;
            vx << 0, -v(2), v(1),
                  v(2), 0, -v(0),
                 -v(1), v(0), 0;
            R = Eigen::Matrix3d::Identity() + vx + vx * vx * ((1.0 - c) / (s * s));
        }
        Quaterniond q(R);
        * ============================================================ */
        
        // Compute roll and pitch directly from gravity direction
        // In FRD frame: X=forward, Y=right, Z=down
        // When level, g = [0, 0, 1] (gravity points down in +Z)
        // 
        // roll  = atan2(g_y, g_z)  -- rotation around X-axis
        // pitch = atan2(-g_x, sqrt(g_y² + g_z²))  -- rotation around Y-axis
        // yaw is not observable from gravity, set to 0
        
        double roll = std::atan2(g(1), g(2));
        double pitch = std::atan2(-g(0), std::sqrt(g(1)*g(1) + g(2)*g(2)));
        double yaw = 0.0;  // Cannot determine yaw from gravity
        
        RCLCPP_INFO(this->get_logger(), "Initial attitude from gravity: roll=%.2f°, pitch=%.2f°, yaw=%.2f°",
                    roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
        
        // Build rotation matrix from Euler angles (ZYX convention)
        // R = Rz(yaw) * Ry(pitch) * Rx(roll)
        Eigen::AngleAxisd rollAngle(roll, Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Vector3d::UnitZ());
        
        // This gives R_body_to_world (how body is rotated relative to world)
        Quaterniond q = yawAngle * pitchAngle * rollAngle;
        q.normalize();
        
        return q;
    }
    
    void publishPose(const rclcpp::Time& stamp) {
        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = "world";
        
        // Get state
        Quaterniond q = eskf_->getQuaternion();
        Vector3d pos = eskf_->getPosition();
        
        // Set pose
        pose_msg.pose.pose.position.x = pos(0);
        pose_msg.pose.pose.position.y = pos(1);
        pose_msg.pose.pose.position.z = pos(2);
        
        pose_msg.pose.pose.orientation.w = q.w();
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        
        // Set covariance (6x6 for pose: position then orientation)
        auto cov_diag = eskf_->getCovarianceDiagonal();
        // Position covariance (indices 3-5 in error state)
        for (int i = 0; i < 3; ++i) {
            pose_msg.pose.covariance[i * 7] = cov_diag(error_idx::DPR_START + i);
        }
        // Orientation covariance (indices 0-2 in error state)
        for (int i = 0; i < 3; ++i) {
            pose_msg.pose.covariance[(i + 3) * 7] = cov_diag(error_idx::DTHETA_START + i);
        }
        
        pose_pub_->publish(pose_msg);
    }
    
    void diagnosticsCallback() {
        if (!is_initialized_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "ESKF Status: NOT INITIALIZED - waiting for first radar measurement");
            return;
        }
        
        // Get covariance diagonal
        auto P_diag = eskf_->getCovarianceDiagonal();
        
        // Compute trace for each state component
        double P_att = P_diag.segment<3>(error_idx::DTHETA_START).sum();   // Attitude (δθ)
        double P_pos = P_diag.segment<3>(error_idx::DPR_START).sum();      // Position (δpr)
        double P_vel = P_diag.segment<3>(error_idx::DVR_START).sum();      // Velocity (δvr)
        double P_pbar = P_diag.segment<2>(error_idx::DPBAR_START).sum();   // Image (δpbar)
        double P_bgyr = P_diag.segment<3>(error_idx::DBGYR_START).sum();   // Gyro bias (δbgyr)
        double P_bacc = P_diag.segment<3>(error_idx::DBACC_START).sum();   // Accel bias (δbacc)
        
        // Get state estimates
        Quaterniond q = eskf_->getQuaternion();
        Vector3d pos = eskf_->getPosition();
        Vector3d vel = eskf_->getVelocity();
        Vector2d pbar = eskf_->getPbar();
        Vector3d bgyr = eskf_->getGyroBias();
        Vector3d bacc = eskf_->getAccelBias();
        
        // Convert quaternion to Euler angles (ZYX convention: yaw, pitch, roll)
        // Using Eigen's built-in conversion
        Vector3d euler_rad = q.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX order
        Vector3d euler_deg = euler_rad * 180.0 / M_PI;
        // Wrap to [-180, 180] for readability
        for (int i = 0; i < 3; ++i) {
            if (euler_deg(i) > 180.0) euler_deg(i) -= 360.0;
            if (euler_deg(i) < -180.0) euler_deg(i) += 360.0;
        }
        
        // Log all state estimates
        RCLCPP_INFO(this->get_logger(), 
            "ESKF State: att(YPR)=[%.2f,%.2f,%.2f]deg, pos=[%.2f,%.2f,%.2f]m, vel=[%.2f,%.2f,%.2f]m/s",
            euler_deg(0), euler_deg(1), euler_deg(2),
            pos(0), pos(1), pos(2),
            vel(0), vel(1), vel(2));
        
        RCLCPP_INFO(this->get_logger(), 
            "ESKF State: pbar=[%.4f,%.4f], bgyr=[%.5f,%.5f,%.5f]rad/s, bacc=[%.4f,%.4f,%.4f]m/s2",
            pbar(0), pbar(1),
            bgyr(0), bgyr(1), bgyr(2),
            bacc(0), bacc(1), bacc(2));
        
        // Log covariance traces for each state
        RCLCPP_INFO(this->get_logger(), 
            "ESKF Cov: P_att=%.4f, P_pos=%.4f, P_vel=%.4f, P_pbar=%.6f, P_bgyr=%.6f, P_bacc=%.6f",
            P_att, P_pos, P_vel, P_pbar, P_bgyr, P_bacc);
    }
    
    // Members
    ESKFParams params_;
    std::unique_ptr<ErrorStateKalmanFilter> eskf_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr radar_sub_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr diag_timer_;
    
    // Initialization flag
    bool is_initialized_ = false;
    
    // Counters
    int imu_count_ = 0;
    int samples_per_eskf_ = 1;
    int delay_steps_ = 0;
    
    // First message received flags
    bool first_imu_received_ = false;
    bool first_image_received_ = false;
    
    // Static initialization buffer
    struct IMUSample {
        Vector3d omega;
        Vector3d accel;
    };
    std::vector<IMUSample> init_imu_buffer_;
    int init_buffer_size_ = 400;  // ~2 seconds at 200Hz
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ESKFNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
