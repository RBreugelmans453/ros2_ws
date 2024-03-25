// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "waveshare_real/waveshare_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

namespace waveshare_real
{

HardWareCommandPub::HardWareCommandPub() : Node("hardware_command_pub")
{
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);
  mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);
}

hardware_interface::CallbackReturn WaveShareHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  command_pub_ = std::make_shared<HardWareCommandPub>();
  cfg_.rear_left_wheel_name = info_.hardware_parameters["rear_left_wheel_name"];
  cfg_.rear_right_wheel_name = info_.hardware_parameters["rear_right_wheel_name"];
  cfg_.front_left_wheel_name = info_.hardware_parameters["front_left_wheel_name"];
  cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("WaveShareHardware"), "PID values not supplied, using defaults.");
  }
  

  wheel_r_l_.setup(cfg_.rear_left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_r_.setup(cfg_.rear_right_wheel_name, cfg_.enc_counts_per_rev);
  wheel_f_l_.setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_f_r_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("WaveShareHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("WaveShareHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("WaveShareHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("WaveShareHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("WaveShareHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> WaveShareHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_l_.name, hardware_interface::HW_IF_POSITION, &wheel_r_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_r_.vel));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_f_l_.name, hardware_interface::HW_IF_POSITION, &wheel_f_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_f_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_f_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_f_r_.name, hardware_interface::HW_IF_POSITION, &wheel_f_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_f_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_f_r_.vel));
  
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> WaveShareHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_r_.cmd));
  
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_f_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_f_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_f_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_f_r_.cmd));
  
  return command_interfaces;
}

hardware_interface::CallbackReturn WaveShareHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("WaveShareHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("WaveShareHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void HardWareCommandPub::send_imu(float& acceX, float& acceY, float& acceZ, float& gyroX, float& gyroY, float& gyroZ, float& magX, float& magY, float& magZ)
{
  //comms_.read_imu_values(acceX, acceY, acceZ, gyroX, gyroY, gyroZ, magX, magY, magZ);
  auto imu_msg = sensor_msgs::msg::Imu();
  imu_msg.header.stamp = rclcpp::Clock().now(); 
  //rclcpp::Node::now();
  imu_msg.header.frame_id = "base_link";
  imu_msg.orientation_covariance = {
    -1, 0, 0, 0, 0, 0, 0, 0, 0};
  imu_msg.angular_velocity.x = gyroX * 0.01745;
  imu_msg.angular_velocity.y = gyroY * 0.01745;
  imu_msg.angular_velocity.z = gyroZ * 0.01745;
  imu_msg.linear_acceleration.x = acceX * 9806.65;
  imu_msg.linear_acceleration.y = acceY * 9806.65;
  imu_msg.linear_acceleration.z = acceZ * 9806.65;
  imu_msg.linear_acceleration_covariance = {
    -1, 0, 0, 0, 0, 0, 0, 0, 0};
  imu_msg.angular_velocity_covariance = {
    -1, 0, 0, 0, 0, 0, 0, 0, 0};
  imu_pub_->publish(imu_msg);

  auto mag_msg = sensor_msgs::msg::MagneticField();
  mag_msg.header.stamp = rclcpp::Clock().now();
  mag_msg.header.frame_id = "base_link";
  mag_msg.magnetic_field.x = magX * 1000000;
  mag_msg.magnetic_field.y = magY * 1000000;
  mag_msg.magnetic_field.z = magZ * 1000000;
  mag_pub_->publish(mag_msg);
}

hardware_interface::CallbackReturn WaveShareHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("WaveShareHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("WaveShareHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn WaveShareHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("WaveShareHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.pid_p > 0)
  {
    comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
  }

  //imu_pub_ = get_name()->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
  RCLCPP_INFO(rclcpp::get_logger("WaveShareHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WaveShareHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("WaveShareHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("WaveShareHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type WaveShareHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }
  
  //comms_.read_encoder_values(wheel_r_l_.enc, wheel_r_r_.enc);
  double delta_seconds = period.seconds();
  float acceX = 0;
  float acceY = 0;
  float acceZ = 0;
  float gyroX = 0;
  float gyroY = 0;
  float gyroZ = 0;
  float magX = 0;
  float magY = 0;
  float magZ = 0;
  //double pos_prev = wheel_r_l_.pos;
  //wheel_r_l_.pos = wheel_r_l_.calc_enc_angle();
 //wheel_r_l_.vel = (wheel_r_l_.pos - pos_prev) / delta_seconds;

  //pos_prev = wheel_r_r_.pos;
  //wheel_r_r_.pos = wheel_r_r_.calc_enc_angle();
 // wheel_r_r_.vel = (wheel_r_r_.pos - pos_prev) / delta_seconds;
  wheel_r_l_.vel = wheel_r_l_.cmd;
  wheel_r_r_.vel = wheel_r_r_.cmd;
  wheel_f_l_.vel = wheel_f_l_.cmd;
  wheel_f_r_.vel = wheel_f_r_.cmd;

  wheel_r_l_.pos += wheel_r_l_.vel * delta_seconds;
  wheel_r_r_.pos += wheel_r_r_.vel * delta_seconds;
  wheel_f_l_.pos += wheel_f_l_.vel * delta_seconds;
  wheel_f_r_.pos += wheel_f_r_.vel * delta_seconds;

  comms_.read_imu_values(acceX, acceY, acceZ, gyroX, gyroY, gyroZ, magX, magY, magZ);
  command_pub_->send_imu(acceX, acceY, acceZ, gyroX, gyroY, gyroZ, magX, magY, magZ);
  return hardware_interface::return_type::OK; 
}

hardware_interface::return_type waveshare_real ::WaveShareHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  int motor_l_counts_per_loop = wheel_r_l_.cmd / wheel_r_l_.rads_per_count / cfg_.loop_rate;
  int motor_r_counts_per_loop = wheel_f_r_.cmd / wheel_f_r_.rads_per_count / cfg_.loop_rate;
  comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);
  return hardware_interface::return_type::OK;
}

}  // namespace waveshare_real

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  waveshare_real::WaveShareHardware, hardware_interface::SystemInterface)