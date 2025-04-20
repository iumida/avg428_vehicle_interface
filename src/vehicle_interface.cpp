#include <rclcpp/rclcpp.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_control_msgs/msg/control.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <cmath>
#include <chrono>

class PIDController {
public:
  PIDController(double p, double i, double d)
  : p_gain_(p), i_gain_(i), d_gain_(d), prev_error_(0.0), integral_(0.0) {}

  double compute(double error, double dt) {
    integral_ += error * dt;
    double derivative = (error - prev_error_) / dt;
    double output = p_gain_ * error + i_gain_ * integral_ + d_gain_ * derivative;
    prev_error_ = error;
    return output;
  }

  void reset() {
    prev_error_ = 0.0;
    integral_ = 0.0;
  }

private:
  double p_gain_;
  double i_gain_;
  double d_gain_;
  double prev_error_;
  double integral_;
};

class VehicleInterface : public rclcpp::Node {
public:
  VehicleInterface()
  : Node("vehicle_interface"),
    desired_velocity_(0.0),
    measured_velocity_(0.0),
    desired_steering_(0.0),
    last_can_steering_(0),
    raw_gear_code_(0),
    velocity_pid_(3, 0.6, 0.15)
  {
    // Publishers
    velocity_pub_ = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
      "/vehicle/status/velocity_status", 10);
    gear_pub_ = create_publisher<autoware_vehicle_msgs::msg::GearReport>(
      "/vehicle/status/gear_status", 10);
    turn_pub_ = create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
      "/vehicle/status/turn_indicators_status", 10);
    mode_pub_ = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
      "/vehicle/status/control_mode", 10);
    steering_pub_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
      "/vehicle/status/steering_status", 10);

    // Subscriptions for outgoing commands
    gear_cmd_sub_ = create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
      "/control/command/gear_cmd", 10,
      [this](const autoware_vehicle_msgs::msg::GearCommand::SharedPtr msg) {
        send_gear_to_can(msg->command);
      });
    turn_cmd_sub_ = create_subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
      "/control/command/turn_indicators_cmd", 10,
      [this](const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::SharedPtr msg) {
        send_turn_to_can(msg->command);
      });
    control_cmd_sub_ = create_subscription<autoware_control_msgs::msg::Control>(
      "/control/command/control_cmd", 10,
      [this](const autoware_control_msgs::msg::Control::SharedPtr msg) {
        desired_velocity_ = msg->longitudinal.velocity;
        desired_steering_ = msg->lateral.steering_tire_angle;
        send_steering_to_can(desired_steering_);
      });

    // Timer for velocity control
    velocity_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&VehicleInterface::controlVelocity, this));

    // Setup CAN socket and start RX thread
    setup_can_socket("can0");
    can_thread_ = std::thread(&VehicleInterface::can_rx_loop, this);
  }

  ~VehicleInterface() {
    close(can_socket_);
    if (can_thread_.joinable()) {
      can_thread_.join();
    }
  }

private:
  // CAN
  int can_socket_;
  std::thread can_thread_;

  // ROS publishers & subscribers
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr mode_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_pub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_cmd_sub_;
  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_sub_;
  rclcpp::TimerBase::SharedPtr velocity_timer_;

  // Control variables
  double desired_velocity_;
  double measured_velocity_;
  double desired_steering_;
  uint16_t last_can_steering_;
  uint8_t raw_gear_code_;         // raw byte0 from CAN 0x0A3
  PIDController velocity_pid_;

  void controlVelocity() {
    const double dt = 0.1;

    if (desired_velocity_ == 0.0) {
      velocity_pid_.reset();
      struct can_frame tx_frame{};
      tx_frame.can_id  = 0x075;
      tx_frame.can_dlc = 8;
      std::memset(tx_frame.data, 0, 8);
      tx_frame.data[0] = 0;
      write(can_socket_, &tx_frame, sizeof(tx_frame));
      return;
    }

    double error = desired_velocity_ - measured_velocity_;
    double pid_output = velocity_pid_.compute(error, dt);
    double feedforward = 30.0;
    double throttle_percentage = std::clamp(pid_output + feedforward, 0.0, 100.0);
    uint8_t throttle_value = static_cast<uint8_t>(throttle_percentage);

    struct can_frame tx_frame{};
    tx_frame.can_id  = 0x075;
    tx_frame.can_dlc = 8;
    std::memset(tx_frame.data, 0, 8);
    tx_frame.data[0] = throttle_value;
    write(can_socket_, &tx_frame, sizeof(tx_frame));
  }

  void setup_can_socket(const std::string &iface) {
    struct ifreq ifr{};
    struct sockaddr_can addr{};
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ);
    ioctl(can_socket_, SIOCGIFINDEX, &ifr);
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    struct can_filter filters[5] = {
      {0x101, CAN_SFF_MASK},
      {0x0A3, CAN_SFF_MASK},
      {0x067, CAN_SFF_MASK},
      {0x363, CAN_SFF_MASK},
      {0x43F, CAN_SFF_MASK},
    };
    setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, sizeof(filters));
    bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
  }

  void can_rx_loop() {
    struct can_frame frame;
    uint32_t left_speed_raw = 0;
    uint32_t right_speed_raw = 0;

    while (rclcpp::ok()) {
      if (read(can_socket_, &frame, sizeof(frame)) < 0) {
        continue;
      }

      // Velocity report (ID 0x101)
      if (frame.can_id == 0x101 && frame.can_dlc == 8) {
        left_speed_raw  = (frame.data[3] << 24) | (frame.data[2] << 16)
                        | (frame.data[1] << 8)  | frame.data[0];
        right_speed_raw = (frame.data[7] << 24) | (frame.data[6] << 16)
                        | (frame.data[5] << 8)  | frame.data[4];
        double left_mps  = static_cast<double>(left_speed_raw)  /1000.0/3600.0;
        double right_mps = static_cast<double>(right_speed_raw) /1000.0/3600.0;
        double avg_mps   = (left_mps + right_mps) / 2.0;

        // If raw gear code == 2 (R), make velocity negative
        if (raw_gear_code_ == 2) {
          avg_mps = -avg_mps;
        }
        measured_velocity_ = avg_mps;

        autoware_vehicle_msgs::msg::VelocityReport vel_msg;
        vel_msg.header.stamp          = now();
        vel_msg.header.frame_id       = "base_link";
        vel_msg.longitudinal_velocity = avg_mps;
        vel_msg.lateral_velocity      = 0.0;
        vel_msg.heading_rate          = 0.0;
        velocity_pub_->publish(vel_msg);
      }
      // Gear report (ID 0x0A3)
      else if (frame.can_id == 0x0A3 && frame.can_dlc >= 1) {
        uint8_t val = frame.data[0];
        raw_gear_code_ = val;  // store raw gear code

        uint8_t gear = 0;
        switch (val) {
          case 8:  gear = 22; break;  // P
          case 4:  gear = 1;  break;  // N
          case 1:  gear = 2;  break;  // D
          case 2:  gear = 20; break;  // R
          default: gear = 0;  break;
        }
        autoware_vehicle_msgs::msg::GearReport gear_msg;
        gear_msg.report = gear;
        gear_pub_->publish(gear_msg);
      }
      // Steering report (ID 0x067)
      else if (frame.can_id == 0x067 && frame.can_dlc >= 4) {
        uint16_t raw = (frame.data[3] << 8) | frame.data[2];
        float degree = (raw - 9000) * 0.1f;
        degree = std::clamp(degree, -43.0f, 43.0f);
        float rad = degree * M_PI / 180.0f;
        autoware_vehicle_msgs::msg::SteeringReport str_msg;
        str_msg.steering_tire_angle = rad;
        steering_pub_->publish(str_msg);
      }
      // Control mode (ID 0x363)
      else if (frame.can_id == 0x363 && frame.can_dlc >= 7) {
        uint8_t mode_flag = frame.data[6];
        uint8_t mode = autoware_vehicle_msgs::msg::ControlModeReport::NO_COMMAND;
        if (mode_flag == 0x41) {
          mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
        } else if (mode_flag == 0x82) {
          mode = autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
        }
        autoware_vehicle_msgs::msg::ControlModeReport mode_msg;
        mode_msg.mode = mode;
        mode_pub_->publish(mode_msg);
      }
      // Turn indicators (ID 0x43F)
      else if (frame.can_id == 0x43F && frame.can_dlc >= 7) {
        bool left  = frame.data[5] & 0x01;
        bool right = frame.data[6] & 0x01;
        uint8_t signal = (left && !right) ? 2
                          : (!left && right) ? 3
                          : 1;
        autoware_vehicle_msgs::msg::TurnIndicatorsReport ti_msg;
        ti_msg.report = signal;
        turn_pub_->publish(ti_msg);
      }
    }
  }

  void send_gear_to_can(uint8_t gear_val) {
    uint8_t can_value = 0;
    switch (gear_val) {
      case 22: can_value = 8; break;  // P
      case 20: can_value = 2; break;  // R
      case 1:  can_value = 4; break;  // N
      case 2:  can_value = 1; break;  // D
      default: return;
    }
    struct can_frame tx_frame{};
    tx_frame.can_id  = 0x168;
    tx_frame.can_dlc = 8;
    std::memset(tx_frame.data, 0, 8);
    tx_frame.data[0] = can_value;
    write(can_socket_, &tx_frame, sizeof(tx_frame));
  }

  void send_turn_to_can(uint8_t command) {
    struct can_frame tx_frame{};
    tx_frame.can_id  = 0x43F;
    tx_frame.can_dlc = 8;
    std::memset(tx_frame.data, 0, 8);
    if (command == 2) {
      tx_frame.data[5] = 1;
    } else if (command == 3) {
      tx_frame.data[6] = 1;
    }
    write(can_socket_, &tx_frame, sizeof(tx_frame));
  }

  void send_steering_to_can(float steering_tire_angle) {
    float deg = steering_tire_angle * 180.0f / M_PI;
    deg = std::clamp(deg, -43.0f, 43.0f);
    uint16_t raw = static_cast<uint16_t>(9000 + deg * 10.0f);
    last_can_steering_ = raw;
    struct can_frame tx_frame{};
    tx_frame.can_id  = 0x065;
    tx_frame.can_dlc = 8;
    std::memset(tx_frame.data, 0, 8);
    tx_frame.data[0] = raw & 0xFF;
    tx_frame.data[1] = (raw >> 8) & 0xFF;
    write(can_socket_, &tx_frame, sizeof(tx_frame));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleInterface>());
  rclcpp::shutdown();
  return 0;
}

