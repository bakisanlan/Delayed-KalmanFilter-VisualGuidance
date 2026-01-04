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
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10, 
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
        
        RCLCPP_INFO(this->get_logger(), "ESKF Node initialized");
        RCLCPP_INFO(this->get_logger(), "  IMU topic: %s", imu_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Image topic: %s", image_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Radar topic: %s", radar_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Samples per ESKF: %d", samples_per_eskf_);
        RCLCPP_INFO(this->get_logger(), "  Image delay steps: %d", delay_steps_);
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Extract measurements
        Vector3d omega(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z
        );
        Vector3d accel(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z
        );
        
        // Get timestamp
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        
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
        // Convert to radar measurement
        Vector3d z_radar(msg->x, msg->y, msg->z);
        
        // Apply correction (no delay)
        Vector3d innovation = eskf_->correctRadar(z_radar);
        
        RCLCPP_DEBUG(this->get_logger(), "Radar correction: innovation = [%.3f, %.3f, %.3f]",
                    innovation(0), innovation(1), innovation(2));
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
        // Print diagnostics
        auto cov_diag = eskf_->getCovarianceDiagonal();
        double trace = cov_diag.sum();
        
        RCLCPP_INFO(this->get_logger(), 
            "ESKF Status: P trace=%.3f, pos=[%.2f,%.2f,%.2f], vel=[%.2f,%.2f,%.2f]",
            trace,
            eskf_->getPosition()(0), eskf_->getPosition()(1), eskf_->getPosition()(2),
            eskf_->getVelocity()(0), eskf_->getVelocity()(1), eskf_->getVelocity()(2));
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
    
    // Counters
    int imu_count_ = 0;
    int samples_per_eskf_ = 1;
    int delay_steps_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ESKFNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
