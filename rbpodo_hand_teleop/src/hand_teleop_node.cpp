/**
 * Copyright (c) 2024 Rainbow Robotics
 * 
 * Hand Teleop Node - Mimics hand movement using rbpodo native control
 * 
 * SAFETY: Robot only moves within a STRICTLY bounded workspace from initial position.
 * Uses move_l action with ACTUAL TCP pose from robot's SystemState.
 */

#include <chrono>
#include <memory>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc.hpp"
#include "rbpodo_msgs/action/move_l.hpp"
#include "rbpodo_msgs/msg/system_state.hpp"

using std::placeholders::_1;
using MoveL = rbpodo_msgs::action::MoveL;
using GoalHandleMoveL = rclcpp_action::ClientGoalHandle<MoveL>;

class HandTeleopNode : public rclcpp::Node
{
public:
  HandTeleopNode()
  : Node("hand_teleop_node"),
    tcp_initialized_(false),
    hand_detected_(false),
    hand_lost_count_(0),
    goal_active_(false),
    hand_x_(0.0),
    hand_y_(0.0)
  {
    // Parameters - workspace bounds enforced in METERS
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    workspace_size_ = declare_parameter<double>("workspace_size", 0.05);  // 5cm default
    move_speed_ = declare_parameter<double>("move_speed", 0.05);  // 50mm/s - safe speed
    move_accel_ = declare_parameter<double>("move_acceleration", 0.1);
    threshold_low_ = declare_parameter<int>("threshold_low", 200);
    enable_debug_ = declare_parameter<bool>("enable_debug", false);
    control_rate_ = declare_parameter<double>("control_rate", 5.0);  // 5Hz for safety

    // ENFORCE maximum workspace size for safety
    if (workspace_size_ > 0.10) {
      RCLCPP_WARN(get_logger(), "Workspace clamped to 10cm MAX for safety!");
      workspace_size_ = 0.10;
    }

    RCLCPP_WARN(get_logger(), "========================================");
    RCLCPP_WARN(get_logger(), "HAND MIMIC - SAFE BOUNDED CONTROL");
    RCLCPP_WARN(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "Max workspace: +/- %.1f cm", workspace_size_ * 100.0);
    RCLCPP_INFO(get_logger(), "Speed: %.1f mm/s", move_speed_ * 1000.0);

    // Subscribe to robot SystemState for ACTUAL TCP pose
    state_sub_ = create_subscription<rbpodo_msgs::msg::SystemState>(
      "/rbpodo_hardware/system_state", 10,
      std::bind(&HandTeleopNode::stateCallback, this, _1));

    // Subscribe to camera images
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&HandTeleopNode::imageCallback, this, _1));

    // Action client for move_l
    move_l_client_ = rclcpp_action::create_client<MoveL>(this, "/rbpodo_hardware/move_l");

    // Control timer
    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate_)),
      std::bind(&HandTeleopNode::controlCallback, this));

    RCLCPP_INFO(get_logger(), "Waiting for robot SystemState...");
  }

private:
  void stateCallback(const rbpodo_msgs::msg::SystemState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // Store current TCP pose from robot (meters and radians)
    current_x_ = msg->tcp_pos[0];
    current_y_ = msg->tcp_pos[1];
    current_z_ = msg->tcp_pos[2];
    current_rx_ = msg->tcp_pos[3];
    current_ry_ = msg->tcp_pos[4];
    current_rz_ = msg->tcp_pos[5];
    last_state_time_ = this->now();

    // Initialize on first message
    if (!tcp_initialized_) {
      initial_x_ = current_x_;
      initial_y_ = current_y_;
      initial_z_ = current_z_;
      initial_rx_ = current_rx_;
      initial_ry_ = current_ry_;
      initial_rz_ = current_rz_;
      
      last_sent_x_ = initial_x_;
      last_sent_y_ = initial_y_;
      last_sent_z_ = initial_z_;
      
      tcp_initialized_ = true;

      RCLCPP_INFO(get_logger(), "========================================");
      RCLCPP_INFO(get_logger(), "Initial TCP from robot:");
      RCLCPP_INFO(get_logger(), "  Position: (%.3f, %.3f, %.3f) m", initial_x_, initial_y_, initial_z_);
      RCLCPP_INFO(get_logger(), "  Orientation: (%.1f, %.1f, %.1f) deg", 
                  initial_rx_ * 180.0/M_PI, initial_ry_ * 180.0/M_PI, initial_rz_ * 180.0/M_PI);
      RCLCPP_INFO(get_logger(), "Workspace: +/- %.1f cm cube", workspace_size_ * 100.0);
      RCLCPP_INFO(get_logger(), "Show hand to camera to control!");
      RCLCPP_INFO(get_logger(), "========================================");
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat bgr;
    try {
      auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      bgr = cv_ptr->image;
    } catch (const cv_bridge::Exception& e) {
      return;
    }

    if (bgr.empty()) return;

    // Simple brightness-based detection
    cv::Mat gray, mask;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, mask, threshold_low_, 255, cv::THRESH_BINARY);

    cv::Moments m = cv::moments(mask, true);
    double min_area = bgr.cols * bgr.rows * 0.001;

    if (m.m00 < min_area) {
      std::lock_guard<std::mutex> lock(hand_mutex_);
      hand_detected_ = false;
      return;
    }

    // Compute normalized coordinates [-1, 1]
    double cx = m.m10 / m.m00;
    double cy = m.m01 / m.m00;
    double nx = (cx - bgr.cols / 2.0) / (bgr.cols / 2.0);
    double ny = (cy - bgr.rows / 2.0) / (bgr.rows / 2.0);
    nx = std::max(-1.0, std::min(1.0, nx));
    ny = std::max(-1.0, std::min(1.0, ny));

    {
      std::lock_guard<std::mutex> lock(hand_mutex_);
      hand_detected_ = true;
      hand_x_ = nx;
      hand_y_ = ny;
      hand_lost_count_ = 0;
    }

    if (enable_debug_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Hand: (%.2f, %.2f)", nx, ny);
    }
  }

  void controlCallback()
  {
    // Check if we have recent state data
    if (!tcp_initialized_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
                           "Waiting for robot SystemState on /rbpodo_hardware/system_state...");
      return;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if ((this->now() - last_state_time_).seconds() > 1.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Robot state stale!");
        return;
      }
    }

    // Check action server ready
    if (!move_l_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for move_l action server...");
      return;
    }

    // Check hand detection
    bool hand_visible = false;
    double hx = 0, hy = 0;
    {
      std::lock_guard<std::mutex> lock(hand_mutex_);
      hand_visible = hand_detected_;
      hx = hand_x_;
      hy = hand_y_;
      if (!hand_visible) hand_lost_count_++;
    }

    if (!hand_visible) {
      if (hand_lost_count_ == 5) {
        RCLCPP_INFO(get_logger(), "Hand lost - robot holds position");
      }
      return;
    }

    // Skip if goal already in progress
    if (goal_active_.load()) {
      return;
    }

    // Calculate target position RELATIVE to initial, STRICTLY bounded by workspace
    // Hand X (left/right in image) -> Robot Y axis
    // Hand Y (up/down in image) -> Robot Z axis (inverted: up in image = up in world)
    double offset_y = hx * workspace_size_;
    double offset_z = -hy * workspace_size_;  // Negate because image Y is down

    // STRICTLY clamp offsets to workspace bounds
    offset_y = std::max(-workspace_size_, std::min(workspace_size_, offset_y));
    offset_z = std::max(-workspace_size_, std::min(workspace_size_, offset_z));

    double target_x = initial_x_;  // X stays constant
    double target_y = initial_y_ + offset_y;
    double target_z = initial_z_ + offset_z;

    // SAFETY CHECK: Verify target is within bounds
    double dist_from_initial = std::sqrt(
      std::pow(target_x - initial_x_, 2) +
      std::pow(target_y - initial_y_, 2) +
      std::pow(target_z - initial_z_, 2));
    
    if (dist_from_initial > workspace_size_ * 1.5) {  // 1.5x for diagonal tolerance
      RCLCPP_ERROR(get_logger(), "SAFETY: Target %.3fm from initial exceeds bounds! Skipping.", 
                   dist_from_initial);
      return;
    }

    // Check minimum movement (5mm) to avoid jitter
    double dist_from_last = std::sqrt(
      std::pow(target_y - last_sent_y_, 2) + 
      std::pow(target_z - last_sent_z_, 2));
    
    if (dist_from_last < 0.005) return;

    if (enable_debug_) {
      RCLCPP_INFO(get_logger(), "Move: Y=%.3f->%.3f (off=%.1fcm) Z=%.3f->%.3f (off=%.1fcm)", 
                  initial_y_, target_y, offset_y * 100.0,
                  initial_z_, target_z, offset_z * 100.0);
    }

    // Send move_l goal with ACTUAL orientation from robot
    auto goal = MoveL::Goal();
    goal.point[0] = target_x;
    goal.point[1] = target_y;
    goal.point[2] = target_z;
    goal.point[3] = initial_rx_;  // Keep orientation FIXED from initial
    goal.point[4] = initial_ry_;
    goal.point[5] = initial_rz_;
    goal.speed = move_speed_;
    goal.acceleration = move_accel_;
    goal.time_for_waiting_start = 0.5;

    RCLCPP_INFO(get_logger(), "move_l to (%.3f, %.3f, %.3f) orient=(%.1f, %.1f, %.1f)deg",
                target_x, target_y, target_z,
                initial_rx_ * 180.0 / M_PI, initial_ry_ * 180.0 / M_PI, initial_rz_ * 180.0 / M_PI);

    auto send_options = rclcpp_action::Client<MoveL>::SendGoalOptions();
    
    send_options.goal_response_callback = 
      [this](const GoalHandleMoveL::SharedPtr& goal_handle) {
        if (!goal_handle) {
          RCLCPP_WARN(get_logger(), "Goal rejected");
          goal_active_ = false;
        } else {
          goal_active_ = true;
        }
      };

    send_options.result_callback = 
      [this](const GoalHandleMoveL::WrappedResult& result) {
        (void)result;
        goal_active_ = false;
      };

    move_l_client_->async_send_goal(goal, send_options);
    last_sent_x_ = target_x;
    last_sent_y_ = target_y;
    last_sent_z_ = target_z;
  }

  // Parameters
  std::string image_topic_;
  double workspace_size_;
  double move_speed_;
  double move_accel_;
  int threshold_low_;
  bool enable_debug_;
  double control_rate_;

  // Robot state
  bool tcp_initialized_;
  double initial_x_{0}, initial_y_{0}, initial_z_{0};
  double initial_rx_{0}, initial_ry_{0}, initial_rz_{0};
  double current_x_{0}, current_y_{0}, current_z_{0};
  double current_rx_{0}, current_ry_{0}, current_rz_{0};
  double last_sent_x_{0}, last_sent_y_{0}, last_sent_z_{0};
  rclcpp::Time last_state_time_;
  std::mutex state_mutex_;

  // Hand tracking
  bool hand_detected_;
  int hand_lost_count_;
  std::atomic<bool> goal_active_;
  double hand_x_, hand_y_;
  std::mutex hand_mutex_;

  // ROS interfaces
  rclcpp::Subscription<rbpodo_msgs::msg::SystemState>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp_action::Client<MoveL>::SharedPtr move_l_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HandTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
