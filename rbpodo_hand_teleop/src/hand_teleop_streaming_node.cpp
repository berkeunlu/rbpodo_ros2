/**
 * Copyright (c) 2024 Rainbow Robotics
 * 
 * Hand Teleop Streaming Node - Real-time 3D hand mirroring using servo_l
 * 
 * This node uses streaming servo control for smooth, low-latency teleoperation.
 * Your hand position in a 3D cube directly maps to the robot TCP position.
 * 
 * SAFETY: Robot only moves within a STRICTLY bounded workspace from initial position.
 */

#include <chrono>
#include <memory>
#include <mutex>
#include <cmath>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rbpodo_msgs/msg/system_state.hpp"
#include "rbpodo_msgs/srv/set_cartesian_pose_controller_config.hpp"

using std::placeholders::_1;

class HandTeleopStreamingNode : public rclcpp::Node
{
public:
  HandTeleopStreamingNode()
  : Node("hand_teleop_streaming_node"),
    tcp_initialized_(false),
    hand_detected_(false),
    streaming_active_(false),
    hand_lost_count_(0)
  {
    // Parameters
    workspace_size_ = declare_parameter<double>("workspace_size", 0.05);  // 5cm default
    control_rate_ = declare_parameter<double>("control_rate", 30.0);  // 30Hz streaming
    enable_debug_ = declare_parameter<bool>("enable_debug", false);
    
    // Servo control parameters
    servo_t1_ = declare_parameter<double>("servo_t1", 0.08);   // Arrival time (higher = smoother)
    servo_t2_ = declare_parameter<double>("servo_t2", 0.15);   // Hold time
    servo_gain_ = declare_parameter<double>("servo_gain", 0.6); // Speed tracking (lower = smoother)
    servo_alpha_ = declare_parameter<double>("servo_alpha", 0.2);  // Low-pass filter (lower = smoother)
    
    // Position smoothing (exponential moving average, 0.0-1.0, lower = smoother)
    smoothing_factor_ = declare_parameter<double>("smoothing_factor", 0.3);
    
    // Safety clamps
    if (workspace_size_ > 0.15) {
      RCLCPP_WARN(get_logger(), "Workspace clamped to 15cm MAX for safety!");
      workspace_size_ = 0.15;
    }

    RCLCPP_WARN(get_logger(), "========================================");
    RCLCPP_WARN(get_logger(), "3D HAND TELEOPERATION - STREAMING MODE");
    RCLCPP_WARN(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "Workspace: +/- %.1f cm cube", workspace_size_ * 100.0);
    RCLCPP_INFO(get_logger(), "Control rate: %.0f Hz", control_rate_);
    RCLCPP_INFO(get_logger(), "Servo params: t1=%.3f t2=%.3f gain=%.2f alpha=%.2f",
                servo_t1_, servo_t2_, servo_gain_, servo_alpha_);
    RCLCPP_INFO(get_logger(), "Position smoothing: %.2f (lower = smoother)", smoothing_factor_);

    // Subscribe to robot SystemState
    state_sub_ = create_subscription<rbpodo_msgs::msg::SystemState>(
      "/rbpodo_hardware/system_state", 10,
      std::bind(&HandTeleopStreamingNode::stateCallback, this, _1));

    // Subscribe to 3D hand position from MediaPipe bridge
    hand_pos_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/hand/position", 10,
      std::bind(&HandTeleopStreamingNode::handPositionCallback, this, _1));

    hand_detected_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/hand/detected", 10,
      std::bind(&HandTeleopStreamingNode::handDetectedCallback, this, _1));

    // Publisher for servo commands
    servo_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/rbpodo_hardware/servo_cartesian", rclcpp::SensorDataQoS());
    
    // Service client for configuring servo controller
    config_client_ = create_client<rbpodo_msgs::srv::SetCartesianPoseControllerConfig>(
      "/rbpodo_hardware/set_cartesian_pose_controller_config");

    // Control timer for streaming servo commands
    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate_)),
      std::bind(&HandTeleopStreamingNode::controlCallback, this));

    RCLCPP_INFO(get_logger(), "Waiting for robot SystemState...");
    RCLCPP_INFO(get_logger(), "Waiting for eval service...");
  }

private:
  void stateCallback(const rbpodo_msgs::msg::SystemState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // Store current TCP pose
    current_tcp_ = *msg;
    last_state_time_ = this->now();
    robot_state_ = msg->robot_state;

    // Initialize on first message
    if (!tcp_initialized_) {
      initial_x_ = msg->tcp_pos[0];
      initial_y_ = msg->tcp_pos[1];
      initial_z_ = msg->tcp_pos[2];
      initial_rx_ = msg->tcp_pos[3];
      initial_ry_ = msg->tcp_pos[4];
      initial_rz_ = msg->tcp_pos[5];

      target_x_ = initial_x_;
      target_y_ = initial_y_;
      target_z_ = initial_z_;

      tcp_initialized_ = true;

      RCLCPP_INFO(get_logger(), "========================================");
      RCLCPP_INFO(get_logger(), "Initial TCP from robot:");
      RCLCPP_INFO(get_logger(), "  Position: (%.3f, %.3f, %.3f) m", initial_x_, initial_y_, initial_z_);
      RCLCPP_INFO(get_logger(), "  Orientation: (%.1f, %.1f, %.1f) deg", 
                  initial_rx_ * 180.0/M_PI, initial_ry_ * 180.0/M_PI, initial_rz_ * 180.0/M_PI);
      RCLCPP_INFO(get_logger(), "Workspace: +/- %.1f cm 3D cube", workspace_size_ * 100.0);
      RCLCPP_INFO(get_logger(), "========================================");
      RCLCPP_INFO(get_logger(), "Show hand to camera. Move in 3D space!");
      RCLCPP_INFO(get_logger(), "  Hand left/right -> Robot X (forward/back)");
      RCLCPP_INFO(get_logger(), "  Hand depth      -> Robot Y (closer=Y-, farther=Y+)");
      RCLCPP_INFO(get_logger(), "  Hand up/down    -> Robot Z");
      RCLCPP_INFO(get_logger(), "========================================");

      // Configure servo controller
      configureServoController();
    }
  }

  void configureServoController()
  {
    if (!config_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "Config service not available, using defaults");
      return;
    }

    auto request = std::make_shared<rbpodo_msgs::srv::SetCartesianPoseControllerConfig::Request>();
    request->t1 = servo_t1_;
    request->t2 = servo_t2_;
    request->gain = servo_gain_;
    request->alpha = servo_alpha_;

    config_client_->async_send_request(request);
    RCLCPP_INFO(get_logger(), "Configured servo controller");
  }

  void handPositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(hand_mutex_);
    
    // Store normalized hand position [-1, 1]
    hand_x_ = msg->point.x;  // Left/right in camera view
    hand_y_ = msg->point.y;  // Up/down in camera view
    hand_z_ = msg->point.z;  // Depth (forward/back)
    
    last_hand_time_ = this->now();
    hand_lost_count_ = 0;
  }

  void handDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(hand_mutex_);
    hand_detected_ = msg->data;
    if (!hand_detected_) {
      hand_lost_count_++;
    }
  }

  void controlCallback()
  {
    if (!tcp_initialized_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
                           "Waiting for robot SystemState...");
      return;
    }

    // Check for stale state
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if ((this->now() - last_state_time_).seconds() > 1.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Robot state stale!");
        streaming_active_ = false;
        return;
      }
    }

    // Check servo publisher has subscribers (optional, just for logging)
    if (servo_pub_->get_subscription_count() == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No subscribers on servo_cartesian topic");
    }

    // Check hand detection
    bool hand_visible = false;
    double hx = 0, hy = 0, hz = 0;
    {
      std::lock_guard<std::mutex> lock(hand_mutex_);
      hand_visible = hand_detected_ && (this->now() - last_hand_time_).seconds() < 0.5;
      hx = hand_x_;
      hy = hand_y_;
      hz = hand_z_;
    }

    if (!hand_visible) {
      if (streaming_active_ && hand_lost_count_ == 5) {
        RCLCPP_INFO(get_logger(), "Hand lost - robot holds position");
        streaming_active_ = false;
      }
      return;
    }

    // Calculate target position
    // Map hand coordinates to robot workspace:
    // From MediaPipe server:
    //   hx: hand left = negative, hand right = positive (user's view facing camera)
    //   hy: hand up = positive, hand down = negative
    //   hz: hand closer to camera = positive, hand farther = negative
    //
    // Robot coordinate frame:
    //   X: forward/back
    //   Y: left/right (from robot's perspective)
    //   Z: up/down
    //
    // USER-REQUESTED MAPPING:
    //   Hand left/right (X) -> Robot X (forward/back)
    //   Hand depth (Z)      -> Robot Y: closer = Y-, farther = Y+
    //   Hand up/down (Y)    -> Robot Z
    
    double offset_x = -hx * workspace_size_;  // Hand left = robot forward, hand right = robot back
    double offset_y = -hz * workspace_size_;  // Hand closer = Y-, hand farther = Y+
    double offset_z = hy * workspace_size_;   // Hand up = robot up

    // Clamp offsets to workspace bounds
    offset_x = std::clamp(offset_x, -workspace_size_, workspace_size_);
    offset_y = std::clamp(offset_y, -workspace_size_, workspace_size_);
    offset_z = std::clamp(offset_z, -workspace_size_, workspace_size_);

    // Calculate raw target
    double raw_target_x = initial_x_ + offset_x;
    double raw_target_y = initial_y_ + offset_y;
    double raw_target_z = initial_z_ + offset_z;
    
    // Apply smoothing filter (exponential moving average)
    double smoothing = smoothing_factor_;
    target_x_ = smoothing * raw_target_x + (1.0 - smoothing) * target_x_;
    target_y_ = smoothing * raw_target_y + (1.0 - smoothing) * target_y_;
    target_z_ = smoothing * raw_target_z + (1.0 - smoothing) * target_z_;

    // Safety check
    double dist = std::sqrt(offset_x*offset_x + offset_y*offset_y + offset_z*offset_z);
    if (dist > workspace_size_ * 1.8) {  // Diagonal tolerance
      RCLCPP_ERROR(get_logger(), "SAFETY: Target %.3fm exceeds bounds!", dist);
      return;
    }

    // Send servo_l command via eval
    sendServoCommand(target_x_, target_y_, target_z_, initial_rx_, initial_ry_, initial_rz_);
    
    if (!streaming_active_) {
      RCLCPP_INFO(get_logger(), "Streaming control started!");
      streaming_active_ = true;
    }

    if (enable_debug_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200, 
                           "Hand(%.2f,%.2f,%.2f) -> TCP(%.3f,%.3f,%.3f) off(%.1f,%.1f,%.1f)cm",
                           hx, hy, hz, target_x_, target_y_, target_z_,
                           offset_x * 100, offset_y * 100, offset_z * 100);
    }
  }

  void sendServoCommand(double x, double y, double z, double rx, double ry, double rz)
  {
    // Publish servo command as Float64MultiArray
    // Data: [x, y, z, rx, ry, rz] in meters and radians
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.resize(6);
    msg.data[0] = x;
    msg.data[1] = y;
    msg.data[2] = z;
    msg.data[3] = rx;
    msg.data[4] = ry;
    msg.data[5] = rz;
    
    servo_pub_->publish(msg);
  }

  // Parameters
  double workspace_size_;
  double control_rate_;
  bool enable_debug_;
  double servo_t1_, servo_t2_, servo_gain_, servo_alpha_;
  double smoothing_factor_;

  // Robot state
  bool tcp_initialized_;
  double initial_x_{0}, initial_y_{0}, initial_z_{0};
  double initial_rx_{0}, initial_ry_{0}, initial_rz_{0};
  double target_x_{0}, target_y_{0}, target_z_{0};
  rbpodo_msgs::msg::SystemState current_tcp_;
  int robot_state_{0};
  rclcpp::Time last_state_time_;
  std::mutex state_mutex_;

  // Hand tracking
  bool hand_detected_;
  std::atomic<bool> streaming_active_;
  int hand_lost_count_;
  double hand_x_{0}, hand_y_{0}, hand_z_{0};
  rclcpp::Time last_hand_time_;
  std::mutex hand_mutex_;

  // ROS interfaces
  rclcpp::Subscription<rbpodo_msgs::msg::SystemState>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr hand_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hand_detected_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr servo_pub_;
  rclcpp::Client<rbpodo_msgs::srv::SetCartesianPoseControllerConfig>::SharedPtr config_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HandTeleopStreamingNode>());
  rclcpp::shutdown();
  return 0;
}
