#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>

using namespace std::chrono_literals;

static inline float wrapPi(float a) {
  return std::atan2(std::sin(a), std::cos(a));
}

struct Waypoint {
  float dx, dy, dphi;
};

class TurnController : public rclcpp::Node {
public:
  TurnController(int scene_number)
      : Node("turn_controller"), scene_number_(scene_number), got_odom_(false),
        wp_reached_(false), init_(true), paused_(false), has_last_time_(false),
        target_wp_(0) {
    twist_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&TurnController::odomCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        50ms, std::bind(&TurnController::executeCallback, this));

    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    SelectWaypoints();
  }

  void stopRobot() {
    auto stop = geometry_msgs::msg::Twist();
    for (int i = 0; i < 5; ++i) {
      twist_pub_->publish(stop);
    }
  }

private:
  void SelectWaypoints() {
    // Waypoints [dx, dy, dphi] in BODY frame.
    switch (scene_number_) {
    case 1: // Simulation
      waypoints_ = {{
          {0.0f, 0.0f, -0.912f},
          {0.0f, 0.0f, 0.7576f},
          {0.0f, 0.0f, 0.8504f},
      }};
      break;

    case 2: // CyberWorld
      waypoints_ = {{
          {0.0f, 0.0f, -0.535f},
          {0.0f, 0.0f, -0.795f},
          {0.0f, 0.0f, 1.311f},
      }};
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid Scene Number: %d",
                   scene_number_);
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const auto &o = msg->pose.pose.orientation;
    tf2::Quaternion q(o.x, o.y, o.z, o.w);
    current_yaw_ = tf2::impl::getYaw(q);
    got_odom_ = true;
  }

  void resetPIDState() {
    prev_err_ = 0.0f;
    integral_err_ = 0.0f;
    d_filt_ = 0.0f;
    has_last_time_ = false;
  }

  void executeCallback() {
    if (!got_odom_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 2000,
                           "Odom data not received!");
      return;
    }

    const rclcpp::Time now = clock_->now();

    if (paused_) {
      if ((now - pause_time_).seconds() >= 2.0) {
        paused_ = false;
        resetPIDState();
      } else {
        auto stop = geometry_msgs::msg::Twist();
        twist_pub_->publish(stop);
        return;
      }
    }

    // Latch target yaw on waypoint change.
    if (wp_reached_ || init_) {
      target_yaw_ = wrapPi(current_yaw_ + waypoints_[target_wp_].dphi);
      init_ = false;
      wp_reached_ = false;
      resetPIDState();
      RCLCPP_INFO(this->get_logger(),
                  "WP %zu target yaw: %.3f rad (current %.3f)", target_wp_ + 1,
                  target_yaw_, current_yaw_);
    }

    // Time step
    double dt = 0.05;
    if (has_last_time_) {
      dt = (now - last_time_).seconds();
    }
    dt = std::clamp(dt, 1e-3, 0.2);
    last_time_ = now;
    has_last_time_ = true;
    const float dtf = static_cast<float>(dt);

    // Yaw error, wrapped to [-pi, pi]
    const float err = wrapPi(target_yaw_ - current_yaw_);

    // Acceptance: yaw only (no translation commanded, so no XY error matters)
    if (std::abs(err) < yaw_tol_) {
      auto stop = geometry_msgs::msg::Twist();
      twist_pub_->publish(stop);
      wp_reached_ = true;
      target_wp_++;

      if (target_wp_ >= waypoints_.size()) {
        RCLCPP_INFO(this->get_logger(), "Final waypoint reached!");
        stopRobot();
        rclcpp::shutdown();
        return;
      }
      pause_time_ = now;
      paused_ = true;
      RCLCPP_INFO(this->get_logger(), "Waypoint %zu reached, pausing...",
                  target_wp_);
      return;
    }

    // Filtered derivative
    const float d_raw = (err - prev_err_) / dtf;
    d_filt_ = alpha_d_ * d_raw + (1.0f - alpha_d_) * d_filt_;

    // PID
    const float omega_unsat = Kp_ * err + Kd_ * d_filt_ + Ki_ * integral_err_;
    const float omega_sat =
        std::clamp(omega_unsat, -max_ang_vel_, max_ang_vel_);

    // Conditional integration anti-windup
    const bool saturated =
        (std::abs(omega_unsat) >= max_ang_vel_) &&
        ((omega_unsat > 0 && err > 0) || (omega_unsat < 0 && err < 0));
    if (!saturated) {
      integral_err_ += err * dtf;
      integral_err_ = std::clamp(integral_err_, -int_limit_, int_limit_);
    }
    prev_err_ = err;

    // Pure rotation — zero linear channels explicitly
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = omega_sat;
    twist_pub_->publish(cmd_vel);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time pause_time_;
  rclcpp::Time last_time_;
  int scene_number_;
  bool got_odom_, wp_reached_, init_, paused_, has_last_time_;
  size_t target_wp_;
  std::shared_ptr<rclcpp::Clock> clock_;

  float current_yaw_{0.0f};
  float target_yaw_{0.0f};
  float prev_err_{0.0f};
  float integral_err_{0.0f};
  float d_filt_{0.0f};
  std::array<Waypoint, 3> waypoints_;

  // PID gains and limits (tuned down from original for real-robot safety)
  const float Kp_ = 1.5f;
  const float Ki_ = 0.0f;
  const float Kd_ = 0.10f;
  const float int_limit_ = 0.5f;
  const float alpha_d_ = 0.2f;
  const float max_ang_vel_ = 0.9f;

  // Acceptance tolerance
  const float yaw_tol_ = 0.03f;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  int scene_number = 2; // default: real
  if (argc > 1) {
    scene_number = std::atoi(argv[1]);
  }

  auto node = std::make_shared<TurnController>(scene_number);
  rclcpp::spin(node);

  node->stopRobot();
  rclcpp::shutdown();
  return 0;
}
