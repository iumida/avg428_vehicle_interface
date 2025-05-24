// src/vehicle_interface.cpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
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
#include <algorithm>
#include <string>

using namespace std::chrono_literals;

class PIDController {
public:
  PIDController(double p, double i, double d)
    : p_gain_(p), i_gain_(i), d_gain_(d), prev_error_(0.0), integral_(0.0) {}

  double compute(double error, double dt) {
    integral_ += error * dt;
    double derivative = (error - prev_error_) / dt;
    prev_error_ = error;
    return p_gain_ * error + i_gain_ * integral_ + d_gain_ * derivative;
  }

  void reset() { prev_error_ = integral_ = 0.0; }

private:
  double p_gain_, i_gain_, d_gain_, prev_error_, integral_;
};

class VehicleInterface : public rclcpp::Node {
public:
  VehicleInterface()
  : Node("vehicle_interface"),
    desired_velocity_(0.0),
    measured_velocity_(0.0),
    desired_steering_(0.0),
    current_gear_cmd_(22),
    velocity_pid_(3.0, 0.6, 0.15)
  {
    velocity_pub_ = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 10);
    gear_pub_ = create_publisher<autoware_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", 10);
    turn_pub_ = create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>("/vehicle/status/turn_indicators_status", 10);
    mode_pub_ = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", 10);
    steering_pub_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 10);

    gear_cmd_agx_sub_ = create_subscription<autoware_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", 10,
      [this](const autoware_vehicle_msgs::msg::GearCommand::SharedPtr msg) {
        current_gear_cmd_ = msg->command;
        send_gear_to_can(current_gear_cmd_);
      });

    control_cmd_agx_sub_ = create_subscription<autoware_control_msgs::msg::Control>("/control/command/control_cmd", 10,
      [this](const autoware_control_msgs::msg::Control::SharedPtr msg) {
        desired_velocity_ = std::abs(msg->longitudinal.velocity);
        desired_steering_ = msg->lateral.steering_tire_angle;
        send_steering_to_can(desired_steering_);
      });

    turn_cmd_sub_ = create_subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>("/control/command/turn_indicators_cmd", 10,
      [this](const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::SharedPtr msg) {
        send_turn_to_can(msg->command);
      });

    velocity_timer_ = create_wall_timer(100ms, std::bind(&VehicleInterface::controlVelocity, this));

    setup_can_socket("can0");

    send_gear_to_can(current_gear_cmd_);
    send_steering_to_can(0.0f);
  }

  ~VehicleInterface() override {
    close(can_socket_);
    if (can_thread_.joinable()) can_thread_.join();
  }

private:
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr mode_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_pub_;

  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_agx_sub_;
  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_agx_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_cmd_sub_;
  rclcpp::TimerBase::SharedPtr velocity_timer_;

  int can_socket_{-1};
  std::thread can_thread_;

  double desired_velocity_, measured_velocity_, desired_steering_;
  uint8_t current_gear_cmd_;
  PIDController velocity_pid_;

  void controlVelocity() {
    constexpr double dt = 0.1;
    struct can_frame tx{};
    tx.can_id = 0x075;
    tx.can_dlc = 8;
    std::memset(tx.data, 0, sizeof(tx.data));
    if (desired_velocity_ < 1e-3) {
      velocity_pid_.reset();
      tx.data[0] = 0;
      write(can_socket_, &tx, sizeof(tx));
      return;
    }
    double ff = (current_gear_cmd_ == 20 ? 50.0 : 30.0);
    double error = desired_velocity_ - measured_velocity_;
    double out = velocity_pid_.compute(error, dt) + ff;
    double pct = std::clamp(out, 0.0, 100.0);
    tx.data[0] = static_cast<uint8_t>(pct);
    write(can_socket_, &tx, sizeof(tx));
  }

  void setup_can_socket(const std::string &iface) {
    struct ifreq ifr{};
    struct sockaddr_can addr{};
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ);
    ioctl(can_socket_, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    struct can_filter filters[] = {
      {0x101, CAN_SFF_MASK}, {0x0A3, CAN_SFF_MASK}, {0x067, CAN_SFF_MASK},
      {0x363, CAN_SFF_MASK}, {0x43F, CAN_SFF_MASK}
    };
    setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(filters));
    bind(can_socket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    can_thread_ = std::thread(&VehicleInterface::can_rx_loop, this);
  }

  void can_rx_loop() {
    struct can_frame frame;
    while (rclcpp::ok()) {
      if (read(can_socket_, &frame, sizeof(frame)) < 0) continue;
      if (frame.can_id == 0x101 && frame.can_dlc == 8) {
        uint32_t left = (frame.data[3]<<24)|(frame.data[2]<<16)|(frame.data[1]<<8)|frame.data[0];
        uint32_t right = (frame.data[7]<<24)|(frame.data[6]<<16)|(frame.data[5]<<8)|frame.data[4];
        double left_mps = left / 1000.0 / 3600.0;
        double right_mps = right / 1000.0 / 3600.0;
        double avg = (left_mps + right_mps) / 2.0;
        if (current_gear_cmd_ == 20) avg = -avg;
        measured_velocity_ = avg;
        autoware_vehicle_msgs::msg::VelocityReport m;
        m.header.stamp = now();
        m.header.frame_id = "base_link";
        m.longitudinal_velocity = avg;
        velocity_pub_->publish(m);
      } else if (frame.can_id == 0x0A3 && frame.can_dlc >= 1) {
        uint8_t raw = frame.data[0], gear = 0;
        switch (raw) {
          case 8: gear = 22; break;
          case 4: gear = 1; break;
          case 1: gear = 2; break;
          case 2: gear = 20; break;
        }
        current_gear_cmd_ = gear;
        autoware_vehicle_msgs::msg::GearReport g;
        g.report = gear;
        gear_pub_->publish(g);
      } else if (frame.can_id == 0x067 && frame.can_dlc >= 4) {
        uint16_t raw = (frame.data[3]<<8) | frame.data[2];
        float deg = (static_cast<int>(raw) - 9000) / 10.0f;
        autoware_vehicle_msgs::msg::SteeringReport s;
        s.steering_tire_angle = deg * static_cast<float>(M_PI / 180.0f);
        steering_pub_->publish(s);
      } else if (frame.can_id == 0x363 && frame.can_dlc >= 7) {
        uint8_t flag = frame.data[6];
        uint8_t mode = autoware_vehicle_msgs::msg::ControlModeReport::NO_COMMAND;
        if (flag == 0x41) mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
        else if (flag == 0x82) mode = autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
        autoware_vehicle_msgs::msg::ControlModeReport m;
        m.mode = mode;
        mode_pub_->publish(m);
      } else if (frame.can_id == 0x43F && frame.can_dlc >= 7) {
        bool L = frame.data[5] & 0x01;
        bool R = frame.data[6] & 0x01;
        uint8_t sig = (L && !R) ? 2 : (!L && R) ? 3 : 1;
        autoware_vehicle_msgs::msg::TurnIndicatorsReport t;
        t.report = sig;
        turn_pub_->publish(t);
      }
    }
  }

  void send_gear_to_can(uint8_t gear) {
    uint8_t val = 0;
    switch (gear) {
      case 22: val = 8; break;
      case 1: val = 4; break;
      case 2: val = 1; break;
      case 20: val = 2; break;
    }
    struct can_frame tx{};
    tx.can_id = 0x168;
    tx.can_dlc = 8;
    std::memset(tx.data, 0, 8);
    tx.data[0] = val;
    write(can_socket_, &tx, sizeof(tx));
  }

  void send_steering_to_can(float steering) {
    float deg = steering * 180.0f / static_cast<float>(M_PI);
    uint16_t raw = static_cast<uint16_t>(9000 + deg * 10.0f);
    struct can_frame tx{};
    tx.can_id = 0x065;
    tx.can_dlc = 8;
    std::memset(tx.data, 0, 8);
    tx.data[0] = raw & 0xFF;
    tx.data[1] = (raw >> 8) & 0xFF;
    write(can_socket_, &tx, sizeof(tx));
  }

  void send_turn_to_can(uint8_t cmd) {
    struct can_frame tx{};
    tx.can_id = 0x43F;
    tx.can_dlc = 8;
    std::memset(tx.data, 0, 8);
    if (cmd == 2) tx.data[5] = 1;
    if (cmd == 3) tx.data[6] = 1;
    write(can_socket_, &tx, sizeof(tx));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleInterface>());
  rclcpp::shutdown();
  return 0;
}
