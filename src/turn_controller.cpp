#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "eigen3/Eigen/Dense"
#include "rosidl_runtime_c/message_initialization.h"

using namespace std::chrono_literals;

class TurnController : public rclcpp::Node {
public:
  TurnController(int scene_number)
      : Node("turn_controller"), scene_number_(scene_number) {
    RCLCPP_INFO(get_logger(), "Turn controller node.");

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&TurnController::odom_callback, this, std::placeholders::_1));
    // sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "/rosbot_xl_base_controller/odom", 10,
    //     std::bind(&TurnController::odom_callback, this,
    //               std::placeholders::_1));

    select_waypoints();
  }

  void run() {
    // PID gains
    const double Kp = 0.5, Ki = 0.05, Kd = 0.1;
    // State variables
    double error_phi = 0.0;
    double error_phi_prev = 0.0;
    double integral_phi = 0.0;
    double derivative_phi = 0.0;
    double PID_phi = 0.0;
    // Limits
    const double I_MAX = 1.0; // integral clamp ±
    const double W_MAX = 0.4; // max angular speed (rad/s)

    geometry_msgs::msg::Twist twist;

    // Wait until someone is listening on /cmd_vel
    while (pub_->get_subscription_count() == 0) {
      rclcpp::sleep_for(100ms);
    }

    auto t0 = std::chrono::steady_clock::now();
    double goal_phi = 0.0;

    // Loop over each relative-turn in motions_
    for (auto [_dx, _dy, rel_phi] : motions_) {
      goal_phi += rel_phi;  // accumulate target yaw
      error_phi_prev = 0.0; // reset PID history
      integral_phi = 0.0;

      // Main PID loop: until we reach the angular tolerance
      while (std::abs(goal_phi - phi_) > ang_tol) {
        // ——— timing ———
        auto t1 = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(t1 - t0).count();
        t0 = t1;
        if (dt <= 0.0)
          dt = 1e-3;

        // ——— compute yaw error ———
        error_phi = goal_phi - phi_;

        // ——— integrate (with clamp) ———
        integral_phi = std::clamp(integral_phi + error_phi * dt, -I_MAX, I_MAX);

        // ——— derivative ———
        derivative_phi = (error_phi - error_phi_prev) / dt;

        // ——— PID output ———
        PID_phi = Kp * error_phi + Ki * integral_phi + Kd * derivative_phi;

        // ——— clamp command ———
        PID_phi = std::clamp(PID_phi, -W_MAX, W_MAX);

        // ——— save for next step ———
        error_phi_prev = error_phi;

        twist.linear.x = 0.00;
        twist.linear.y = 0.00;
        twist.angular.z = PID_phi;

        // Publish to /cmd_vel
        pub_->publish(twist);

        rclcpp::spin_some(shared_from_this());
        rclcpp::sleep_for(25ms);

        // ——— debug print ———
        RCLCPP_INFO(get_logger(), "angle_error=%.3f -> w_cmd=%.3f", error_phi,
                    PID_phi);
      }

      // Stop before next segment
      stop();
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::vector<std::tuple<double, double, double>> motions_;
  double phi_;
  double w_, l_, r_;

  double ang_tol = 0.01;

  int scene_number_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract yaw (phi) from quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    phi_ = yaw;
  }

  void stop() {
    geometry_msgs::msg::Twist twist;
    rclcpp::Rate rate(20);
    for (int i = 0; i < 20; ++i) {
      pub_->publish(twist);
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    }
    RCLCPP_INFO(get_logger(), "Stop (zeroed for 0.5 s)");
  }

  //   static double quat_to_yaw(double qz, double qw) {
  //     return 2.0 * std::atan2(qz, qw);
  //   }

  void select_waypoints() {

    switch (scene_number_) {
    case 1: { // Simulation

      double yaw1 = -1.000;
      double yaw2 = -0.150;
      double yaw3 = 0.700;

      motions_ = {{0, 0, yaw1}, {0, 0, yaw2 - yaw1}, {0, 0, yaw3 - yaw2}};
    }; break;

    case 2: { // CyberWorld

      double yaw1 = -0.30;
      double yaw2 = -1.00;
      double yaw3 = 1.00;

      motions_ = {{0.0, 0.0, yaw1}, {0.0, 0.0, yaw2 - yaw1}, {0.0, 0.0, yaw3}};
    } break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid Scene Number: %d",
                   scene_number_);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int scene_number = 1;
  if (argc > 1) {
    scene_number = std::atoi(argv[1]);
  }
  auto node = std::make_shared<TurnController>(scene_number);
  node->run();
  rclcpp::shutdown();
  return 0;
}