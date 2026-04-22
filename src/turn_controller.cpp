#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <thread>
#include <vector>

struct Waypoint {
  double dx;   // Delta x in meters
  double dy;   // Delta y in meters
  double dyaw; // Delta yaw in radians
};

class TurnController : public rclcpp::Node {
public:
  TurnController(int scene_number) : Node("turn_controller") {

    // Setup Waypoints and PID gains depending on scene
    setup_scene(scene_number);

    RCLCPP_INFO(this->get_logger(), "PID gains: Kp=%.3f, Ki=%.3f, Kd=%.3f", Kp_,
                Ki_, Kd_);

    // Create Reentrant Callback Group
    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;

    // Subscriber for Odometry Information
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&TurnController::odom_callback, this, std::placeholders::_1),
        options);

    // Subscriber for IMU Information
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu_broadcaster/imu", 10,
        std::bind(&TurnController::imu_callback, this, std::placeholders::_1),
        options);

    // Publisher for the CMD Vel
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Turn Controller Node Ready!");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // State
  double current_yaw_{0.0};
  double imu_yaw_rate_{0.0};
  double segment_target_yaw_{0.0};
  bool segment_active_{false};
  bool odom_received_{false};
  std::vector<Waypoint> waypoints_;
  size_t current_idx_{0};

  geometry_msgs::msg::Twist twist_;

  // PID
  double Kp_{1.3};
  double Ki_{0.001};
  double Kd_{0.3};
  double sum_I_{0.0};

  // Timing & limits
  const double rate_hz_{40.0}; // ROSbot XL and Sim work on 20 Hz
  const double dt_{1.0 / rate_hz_};
  double max_speed_{3.14}; // Took from ROSBot XL Spec 3.14 rad/s
  const double angular_tolerance{0.01};
  double speed_tolerance_{0.02};

  const double pause_duration_sec_{1.5};
  const int pause_ticks_goal_{
      static_cast<int>(pause_duration_sec_ * rate_hz_)}; // sec * rateHZ
  int pause_ticks_{0};
  bool pausing_{false};

  void setup_scene(const int scene_number) {
    switch (scene_number) {
    case 1: // Simulation
      waypoints_ = {{0.0, 0.0, -0.92},
                    {0.0, 0.0, 0.77},
                    {0.0, 0.0, 0.74},
                    {0.0, 0.0, -0.59}};
      Kp_ = 1.3;
      Ki_ = 0.001;
      Kd_ = 0.3;
      max_speed_ = 3.14;
      speed_tolerance_ = 0.02;
      RCLCPP_INFO(get_logger(),
                  "Scene 1 (Sim): set sim waypoints and PID gains");
      RCLCPP_INFO(this->get_logger(), "Set Simulation Waypoints.");
      break;
    case 2: // CyberWorld
      Kp_ = 1.25;
      Ki_ = 0.001;
      Kd_ = 0.3;
      max_speed_ = 1.4;
      speed_tolerance_ = 0.05;
      waypoints_ = {{0.0, 0.0, -0.6}, {0.0, 0.0, -0.66}, {0.0, 0.0, 1.26}};
      RCLCPP_INFO(get_logger(),
                  "Scene 2 (CyberWorld): set real waypoints and PID gains");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid Scene Number: %d",
                   scene_number);
      rclcpp::shutdown();
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    current_yaw_ = tf2::getYaw(q);

    if (!odom_received_) {
      odom_received_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "First odom: yaw=%.3f. Starting control loop.", current_yaw_);

      // Control Loop Timer 40 Hz, normal for the PID
      control_timer_ = this->create_wall_timer(
          std::chrono::duration<double>(dt_),
          std::bind(&TurnController::control_loop, this), callback_group_);
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_yaw_rate_ = msg->angular_velocity.z; // rad/s, IMU frame z
  }

  void control_loop() {
    // Don't run PID until we know topics are "ready"
    if (!interfaces_ready()) {
      stop();
      return;
    }

    if (pausing_) {
      stop();
      pause_ticks_++;
      if (pause_ticks_ >= pause_ticks_goal_) {
        pausing_ = false;
      }
      return;
    }

    // 1. If no active segment, start one (if waypoints left)
    if (!segment_active_ && current_idx_ < waypoints_.size()) {
      auto wp = waypoints_[current_idx_];

      segment_target_yaw_ = current_yaw_ + wp.dyaw;
      segment_active_ = true;
      sum_I_ = 0.0;

      RCLCPP_INFO(this->get_logger(), "Turning to waypoint %zu (dyaw=%.3f)",
                  current_idx_, wp.dyaw);
    }

    // 2. If no more segments, stop the robot
    if (current_idx_ >= waypoints_.size()) {
      stop();
      RCLCPP_INFO(this->get_logger(), "Trajectory completed.");
      rclcpp::shutdown();
      return;
    }

    // 3. Compute error to current waypoint
    double e_yaw = segment_target_yaw_ - current_yaw_;
    double de_yaw = 0.0 - imu_yaw_rate_;

    // 4. Check if we reached the waypoint
    if (std::abs(e_yaw) < angular_tolerance &&
        std::abs(de_yaw) < speed_tolerance_) {
      RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu.", current_idx_);
      current_idx_++;
      segment_active_ = false;
      stop();
      // start pause
      pausing_ = true;
      pause_ticks_ = 0;
      return;
    }

    // 5. Run our PID controller using eyaw, deyaw
    compute_PID(e_yaw, de_yaw);

    RCLCPP_INFO(this->get_logger(), //  *this->get_clock(), 40,
                "Debug: Error (eyaw=%.3f) | "
                "World Frame Speed: (w_yaw=%.3f)",
                e_yaw, imu_yaw_rate_);

    cmd_vel_publisher_->publish(twist_);
  }

  bool interfaces_ready() {
    // Is anyone listening to /cmd_vel ?
    auto cmd_vel_subs = cmd_vel_publisher_->get_subscription_count();

    if (cmd_vel_subs == 0) {
      sum_I_ = 0.0;
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "No subscribers on /cmd_vel. Holding control loop...");
      return false;
    }

    return true;
  }

  void compute_PID(double e_yaw, double de_yaw) {
    // Update integral value of integral control
    sum_I_ += e_yaw * dt_;

    twist_.angular.z = Kp_ * e_yaw + Kd_ * de_yaw + Ki_ * sum_I_;

    // Limit angular yaw speed
    twist_.angular.z = std::clamp(twist_.angular.z, -max_speed_, max_speed_);
  }

  void stop() {
    twist_.angular.z = 0.0;
    cmd_vel_publisher_->publish(twist_);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Check if a scene number argument is provided
  int scene_number = 1; // Default scene number to simulation
  if (argc > 1) {
    scene_number = std::atoi(argv[1]);
  }

  auto node = std::make_shared<TurnController>(scene_number);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    3);
  executor.add_node(node);
  try {
    executor.spin();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}