/**
 * @file eskf_node_reduced.cpp
 * @brief ROS2 node for the reduced-state ESKF.
 */

#include "eskf_cpp_reduced/eskf_config.hpp"
#include "eskf_cpp_reduced/eskf_core_reduced.hpp"
#include "eskf_cpp_reduced/utils/print.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>

using namespace std::chrono_literals;



class ReducedESKFNode : public rclcpp::Node {
public:
    ReducedESKFNode() : Node("eskf_node_reduced") {
        this->declare_parameter("config_file", "");
        this->declare_parameter("print_level", "INFO");

        eskf::Printer::setPrintLevel(this->get_parameter("print_level").as_string());

        const std::string config_file = this->get_parameter("config_file").as_string();
        if (!config_file.empty()) {
            try {
                params_ = eskf::loadConfig(config_file);
                PRINT_INFO(GREEN "[CONFIG-RED]: Loaded from %s" RESET "\n", config_file.c_str())
            } catch (const std::exception& e) {
                PRINT_WARNING(YELLOW "[CONFIG-RED]: Failed to load: %s. Using defaults." RESET "\n", e.what())
            }
        }
        eskf::printConfig(params_);

        filter_ = std::make_unique<eskf::reduced::ReducedErrorStateKalmanFilter>(params_);

        rclcpp::QoS imu_qos(10);
        imu_qos.best_effort();

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            params_.topic_imu, imu_qos,
            std::bind(&ReducedESKFNode::imuCallback, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            params_.topic_image, 10,
            std::bind(&ReducedESKFNode::imageCallback, this, std::placeholders::_1));

        radar_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            params_.topic_radar, 10,
            std::bind(&ReducedESKFNode::radarCallback, this, std::placeholders::_1));

        interceptor_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            params_.topic_interceptor_odom, imu_qos,
            std::bind(&ReducedESKFNode::interceptorOdomCallback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(params_.topic_odom, 10);
        pbar_pub_ = this->create_publisher<geometry_msgs::msg::Point>(params_.topic_pbar, 10);

        diag_timer_ = this->create_wall_timer(
            1s, std::bind(&ReducedESKFNode::diagnosticsCallback, this));

        samples_per_eskf_ = std::max(1, static_cast<int>(std::round(params_.dt_eskf / params_.dt_imu)));
        delay_steps_ = static_cast<int>(std::round(params_.image_delay / params_.dt_eskf));

        PRINT_INFO(BOLDCYAN "\n[ESKF-RED]: Node initialized" RESET "\n")
        PRINT_INFO(CYAN "  IMU topic:              %s" RESET "\n", params_.topic_imu.c_str())
        PRINT_INFO(CYAN "  Interceptor odom topic: %s" RESET "\n", params_.topic_interceptor_odom.c_str())
        PRINT_INFO(CYAN "  Image topic:            %s" RESET "\n", params_.topic_image.c_str())
        PRINT_INFO(CYAN "  Radar topic:            %s" RESET "\n", params_.topic_radar.c_str())
        PRINT_INFO(CYAN "  Samples/update:         %d" RESET "\n", samples_per_eskf_)
        PRINT_INFO(CYAN "  Image delay:            %d steps" RESET "\n", delay_steps_)
        PRINT_WARNING(YELLOW "[ESKF-RED]: Waiting for interceptor odometry and first radar measurement..." RESET "\n")
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // so the reduced filter keeps using the IMU gyro for omega.
        const eskf::Vector3d omega_flu(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);
        const eskf::Vector3d omega_frd = eskf::math::fluToFrd(omega_flu);

        const double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        if (!is_initialized_ || !interceptor_state_received_) {
            return;
        }

        filter_->accumulateIMU(omega_frd);
        imu_count_++;

        if (imu_count_ >= samples_per_eskf_) {
            const auto avg = filter_->getAveragedIMU(timestamp);
            filter_->predict(avg.omega, interceptor_state_, timestamp);
            imu_count_ = 0;
            prediction_count_++;

            if (last_image_time_ > 0.0) {
                const double time_since_image = timestamp - last_image_time_;
                if (time_since_image > params_.image_timeout_sec && !filter_->isPbarFrozen()) {
                    filter_->setPbarFrozen(true);
                    PRINT_WARNING(YELLOW "[IMAGE-RED]: Timeout (%.1fs) - pbar frozen" RESET "\n", time_since_image)
                }
            }

            publishState(msg->header.stamp);
        }
    }

    void imageCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {

        //debug
        //PRINT_INFO(GREEN "[IMAGE-RED]: Image received" RESET "\n")

        if (!is_initialized_) {
            return;
        }

        if (filter_->isPbarFrozen()) {
            filter_->setPbarFrozen(false);
        }

        last_image_time_ = this->now().seconds();
        const eskf::Vector2d z_pbar(msg->point.x, msg->point.y);
        const eskf::Vector2d innovation = filter_->correctImage(z_pbar, delay_steps_);

        static int image_log_counter = 0;
        if (innovation.norm() > 0.0 && ++image_log_counter % 30 == 0) {
            PRINT_DEBUG(CYAN "[IMAGE-RED]: Innovation = [%.4f, %.4f]" RESET "\n",
                        innovation(0), innovation(1))
        }
    }

    void radarCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        pending_radar_measurement_.setZero();
        pending_radar_measurement_.head<3>() <<
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z;
        pending_radar_measurement_.tail<3>() <<
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z;
        pending_radar_noise_ = buildRadarMeasurementNoise(*msg);
        pending_radar_received_ = true;

        if (!interceptor_state_received_) {
            PRINT_WARNING(YELLOW "[RADAR-RED]: Radar received before interceptor odometry; waiting..." RESET "\n")
            return;
        }

        // print debug for radar measurement
        PRINT_DEBUG(BLUE "[RADAR-RED]: Radar measurement pos=[%.3f, %.3f, %.3f] vel=[%.3f, %.3f, %.3f]" RESET "\n",
                    pending_radar_measurement_(0), pending_radar_measurement_(1), pending_radar_measurement_(2),
                    pending_radar_measurement_(3), pending_radar_measurement_(4), pending_radar_measurement_(5))

        const eskf::reduced::RadarMeasurement target_state =
            computeTargetStateMeasurement(pending_radar_measurement_);

        if (!is_initialized_) {
            initializeFilter(target_state, pending_radar_noise_);
            pending_radar_received_ = false;
            return;
        }

        filter_->notifyRadarReceived();
        const eskf::reduced::RadarMeasurement innovation =
            filter_->correctRadar(target_state, pending_radar_noise_);

        static int radar_log_counter = 0;
        if (++radar_log_counter % 10 == 0) {
            PRINT_DEBUG(BLUE "[RADAR-RED]: Innovation pos=[%.3f, %.3f, %.3f] vel=[%.3f, %.3f, %.3f]" RESET "\n",
                        innovation(0), innovation(1), innovation(2),
                        innovation(3), innovation(4), innovation(5))
        }
    }

    void interceptorOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        const eskf::Vector3d pos_enu(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);

        const eskf::Quaterniond q_enu_flu(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
        const eskf::Quaterniond q_ned_frd =
            eskf::math::convertBodyOrientationEnuFluToNedFrd(q_enu_flu);
        const eskf::RotationMatrix R_b2e = eskf::math::quaternionToRotation(q_ned_frd);

        const eskf::Vector3d vel_enu_local(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z);

        interceptor_state_.position_ned = eskf::math::enuToNed(pos_enu);
        interceptor_state_.velocity_ned = eskf::math::enuToNed(vel_enu_local);
        interceptor_state_.R_b2e = R_b2e;
        interceptor_state_received_ = true;

        if (!is_initialized_ && pending_radar_received_) {
            initializeFilter(
                computeTargetStateMeasurement(pending_radar_measurement_),
                pending_radar_noise_);
            pending_radar_received_ = false;
        }
    }

    eskf::reduced::RadarMeasurement computeTargetStateMeasurement(
        const eskf::reduced::RadarMeasurement& radar_measurement) const {
        if (params_.radar_measurement_is_relative) {
            eskf::reduced::RadarMeasurement target_state = radar_measurement;
            // Relative convention: p_r = p_i - p_t and v_r = v_i - v_t, both in NED.
            target_state.head<3>() = interceptor_state_.position_ned - radar_measurement.head<3>();
            target_state.tail<3>() = interceptor_state_.velocity_ned - radar_measurement.tail<3>();
            return target_state;
        }
        return radar_measurement;
    }

    eskf::reduced::RadarNoise buildRadarMeasurementNoise(const nav_msgs::msg::Odometry& msg) const {
        eskf::reduced::RadarNoise radar_noise = eskf::reduced::RadarNoise::Zero();

        const Eigen::Matrix3d fallback_pos =
            params_.sigma_radar_pos * params_.sigma_radar_pos * Eigen::Matrix3d::Identity();
        const Eigen::Matrix3d fallback_vel =
            params_.sigma_radar_vel * params_.sigma_radar_vel * Eigen::Matrix3d::Identity();

        radar_noise.block<3, 3>(0, 0) = eskf::math::buildCovarianceBlock3x3(
            msg.pose.covariance, fallback_pos);
        radar_noise.block<3, 3>(3, 3) = eskf::math::buildCovarianceBlock3x3(
            msg.twist.covariance, fallback_vel);

        return radar_noise;
    }

    void initializeFilter(const eskf::reduced::RadarMeasurement& z_target,
                          const eskf::reduced::RadarNoise& radar_noise) {
        eskf::reduced::NominalState x_init = eskf::reduced::NominalState::Zero();
        const eskf::Vector3d p_t_init = z_target.head<3>();
        x_init.segment<3>(eskf::reduced::nominal_idx::PT_START) = p_t_init;
        x_init.segment<3>(eskf::reduced::nominal_idx::VT_START) =
            params_.use_vr ? eskf::Vector3d(z_target.tail<3>()) : eskf::Vector3d::Zero();

        // 1. Initialize pbar state directly from geometry
        const eskf::Vector2d pbar_init = eskf::math::computeImageFeatures(
            p_t_init, interceptor_state_, params_.R_b2c, params_.min_depth);
        x_init.segment<2>(eskf::reduced::nominal_idx::PBAR_START) = pbar_init;

        eskf::reduced::ErrorCovariance P_init = filter_->getCovariance();
        const Eigen::Matrix3d P_pos = radar_noise.block<3, 3>(0, 0);
        P_init.block<3, 3>(eskf::reduced::error_idx::DPT_START,
                           eskf::reduced::error_idx::DPT_START) = P_pos;

        if (params_.use_vr) {
            P_init.block<3, 3>(eskf::reduced::error_idx::DVT_START,
                               eskf::reduced::error_idx::DVT_START) =
                radar_noise.block<3, 3>(3, 3);
        }

        // 2. Propagate target position covariance into pbar covariance
        const eskf::Vector3d p_c = eskf::math::computeTargetInCameraFrame(
            p_t_init, interceptor_state_, params_.R_b2c);
        const Eigen::Matrix2d P_pbar_proj = eskf::math::projectPositionCovarianceToPbar(
            P_pos, p_c, interceptor_state_, params_.R_b2c, params_.min_depth);

        P_init.block<2, 2>(eskf::reduced::error_idx::DPBAR_START,
                           eskf::reduced::error_idx::DPBAR_START) += P_pbar_proj;

        filter_->reset(x_init, P_init);
        filter_->notifyRadarReceived();
        is_initialized_ = true;

        PRINT_INFO(BOLDGREEN "\n========== REDUCED ESKF INITIALIZATION ==========" RESET "\n")
        PRINT_INFO(CYAN "[RADAR-RED]: First measurement received" RESET "\n")
        PRINT_INFO(CYAN "  Target position NED: [%.2f, %.2f, %.2f]" RESET "\n",
                   z_target(0), z_target(1), z_target(2))
        PRINT_INFO(CYAN "  Target velocity NED: [%.2f, %.2f, %.2f]" RESET "\n",
                   x_init(eskf::reduced::nominal_idx::VT_START + 0),
                   x_init(eskf::reduced::nominal_idx::VT_START + 1),
                   x_init(eskf::reduced::nominal_idx::VT_START + 2))
        PRINT_INFO(CYAN "  Init pbar:           [%.4f, %.4f]" RESET "\n",
                   pbar_init(0), pbar_init(1))
        PRINT_INFO(CYAN "  Init sigma pos NED: [%.2f, %.2f, %.2f]" RESET "\n",
                   std::sqrt(P_init(eskf::reduced::error_idx::DPT_START + 0,
                                    eskf::reduced::error_idx::DPT_START + 0)),
                   std::sqrt(P_init(eskf::reduced::error_idx::DPT_START + 1,
                                    eskf::reduced::error_idx::DPT_START + 1)),
                   std::sqrt(P_init(eskf::reduced::error_idx::DPT_START + 2,
                                    eskf::reduced::error_idx::DPT_START + 2)))
        PRINT_INFO(CYAN "  Init sigma vel NED: [%.2f, %.2f, %.2f]" RESET "\n",
                   std::sqrt(P_init(eskf::reduced::error_idx::DVT_START + 0,
                                    eskf::reduced::error_idx::DVT_START + 0)),
                   std::sqrt(P_init(eskf::reduced::error_idx::DVT_START + 1,
                                    eskf::reduced::error_idx::DVT_START + 1)),
                   std::sqrt(P_init(eskf::reduced::error_idx::DVT_START + 2,
                                    eskf::reduced::error_idx::DVT_START + 2)))
        PRINT_INFO(CYAN "  Init sigma pbar:     [%.4f, %.4f]" RESET "\n",
                   std::sqrt(P_init(eskf::reduced::error_idx::DPBAR_START + 0,
                                    eskf::reduced::error_idx::DPBAR_START + 0)),
                   std::sqrt(P_init(eskf::reduced::error_idx::DPBAR_START + 1,
                                    eskf::reduced::error_idx::DPBAR_START + 1)))
        PRINT_INFO(GREEN "[ESKF-RED]: Filter RUNNING" RESET "\n")
        PRINT_INFO(BOLDGREEN "=================================================" RESET "\n\n")
    }

    void publishState(const rclcpp::Time& stamp) {
        const eskf::Vector3d pos = filter_->getTargetPosition();
        const eskf::Vector3d vel = filter_->getTargetVelocity();
        const eskf::Vector2d pbar = filter_->getPbar();
        const auto cov_diag = filter_->getCovarianceDiagonal();

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = "ned";
        odom_msg.child_frame_id = "target";

        odom_msg.pose.pose.position.x = pos(0);
        odom_msg.pose.pose.position.y = pos(1);
        odom_msg.pose.pose.position.z = pos(2);
        odom_msg.pose.pose.orientation.w = 1.0;
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = 0.0;

        for (int i = 0; i < 3; ++i) {
            odom_msg.pose.covariance[i * 7] = cov_diag(eskf::reduced::error_idx::DPT_START + i);
            odom_msg.twist.covariance[i * 7] = cov_diag(eskf::reduced::error_idx::DVT_START + i);
        }

        odom_msg.twist.twist.linear.x = vel(0);
        odom_msg.twist.twist.linear.y = vel(1);
        odom_msg.twist.twist.linear.z = vel(2);

        odom_pub_->publish(odom_msg);

        geometry_msgs::msg::Point pbar_msg;
        pbar_msg.x = pbar(0);
        pbar_msg.y = pbar(1);
        pbar_msg.z = 0.0;
        pbar_pub_->publish(pbar_msg);
    }

    void diagnosticsCallback() {
        if (!is_initialized_) {
            PRINT_WARNING(YELLOW "[ESKF-RED]: Waiting for radar/odometry before initialization..." RESET "\n")
            return;
        }

        const auto P_diag = filter_->getCovarianceDiagonal();
        const auto P_pos = P_diag.segment<3>(eskf::reduced::error_idx::DPT_START);
        const auto P_vel = P_diag.segment<3>(eskf::reduced::error_idx::DVT_START);
        const auto P_pbar = P_diag.segment<2>(eskf::reduced::error_idx::DPBAR_START);

        const eskf::Vector3d pos = filter_->getTargetPosition();
        const eskf::Vector3d vel = filter_->getTargetVelocity();
        const eskf::Vector2d pbar = filter_->getPbar();

        const double current_hz = static_cast<double>(prediction_count_);
        prediction_count_ = 0;

        PRINT_INFO(BOLDCYAN "[STATE-RED]:" RESET
                   " pos=" GREEN "[%.2f,%.2f,%.2f]m" RESET
                   " vel=" BLUE "[%.2f,%.2f,%.2f]m/s" RESET
                   " pbar=" MAGENTA "[%.4f,%.4f]" RESET "\n",
                   pos(0), pos(1), pos(2),
                   vel(0), vel(1), vel(2),
                   pbar(0), pbar(1))

        PRINT_INFO(BOLDCYAN "[COV-RED]:" RESET
                   " pos=" GREEN "[%.2f,%.2f,%.2f]" RESET
                   " vel=" BLUE "[%.4f,%.4f,%.4f]" RESET
                   " pbar=" MAGENTA "[%.6f,%.6f]" RESET
                   " | " BOLDREDPURPLE "%.0f Hz" RESET "\n",
                   P_pos(0), P_pos(1), P_pos(2),
                   P_vel(0), P_vel(1), P_vel(2),
                   P_pbar(0), P_pbar(1), current_hz)
    }

    eskf::ESKFParams params_;
    std::unique_ptr<eskf::reduced::ReducedErrorStateKalmanFilter> filter_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr image_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr radar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr interceptor_odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pbar_pub_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    eskf::reduced::InterceptorState interceptor_state_;
    bool interceptor_state_received_ = false;
    bool is_initialized_ = false;
    bool pending_radar_received_ = false;
    eskf::reduced::RadarMeasurement pending_radar_measurement_ =
        eskf::reduced::RadarMeasurement::Zero();
    eskf::reduced::RadarNoise pending_radar_noise_ =
        eskf::reduced::RadarNoise::Identity();

    int imu_count_ = 0;
    int samples_per_eskf_ = 1;
    int delay_steps_ = 0;
    int prediction_count_ = 0;
    double last_image_time_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReducedESKFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
