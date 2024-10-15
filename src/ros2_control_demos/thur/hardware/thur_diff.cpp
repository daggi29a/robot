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

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "thur/thur_diff.hpp"
#include "include/serial.hpp"

namespace thur
{
hardware_interface::CallbackReturn ThurSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  serial_port_name_ = info_.hardware_parameters["serial_port"];

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ThurSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ThurSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ThurSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ThurSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ThurSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ThurSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ThurSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ThurSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // RCLCPP_INFO(rclcpp::get_logger("ThurSystemHardware"), "Activating ...please wait...");

  // for (auto i = 0; i < hw_start_sec_; i++)
  // {
  //   rclcpp::sleep_for(std::chrono::seconds(1));
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("ThurSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
  // }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  serial_port_ = std::make_shared<ThurSerialPort>();
  if (serial_port_->open(serial_port_name_) != return_type::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("ThurSystemHardware"), "Trying to activate serial port '%s' ...", serial_port_->to_string());
      RCLCPP_WARN(rclcpp::get_logger("ThurHardware"), "Thur hardware failed to open serial port");
      RCLCPP_INFO(rclcpp::get_logger("ThurSystemHardware"), "Done trying!");

      return hardware_interface::CallbackReturn::SUCCESS;
  }

  RCLCPP_INFO(rclcpp::get_logger("ThurSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThurSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (serial_port_->is_open()) {
          serial_port_->close();
          serial_port_.reset();
  }
  // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // RCLCPP_INFO(rclcpp::get_logger("ThurSystemHardware"), "Deactivating ...please wait...");

  // for (auto i = 0; i < hw_stop_sec_; i++)
  // {
  //   rclcpp::sleep_for(std::chrono::seconds(1));
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("ThurSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
  // }
  // // END: This part here is for exemplary purposes - Please do not copy to your production code

  RCLCPP_INFO(rclcpp::get_logger("ThurSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ThurSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
        RCLCPP_INFO(rclcpp::get_logger("ThurHardware"), "In Read Method!!!s");
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
/*   for (std::size_t i = 0; i < hw_velocities_.size(); i++)
  {
    // Simulate DiffBot wheels's movement as a first-order system
    // Update the joint status: this is a revolute joint without any limit.
    // Simply integrates
    hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];

    RCLCPP_INFO(
      rclcpp::get_logger("ThurSystemHardware"),
      "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
      hw_velocities_[i], info_.joints[i].name.c_str());
  }
 */  // END: This part here is for exemplary purposes - Please do not copy to your production code

      // Make sure we are connected to the serial port
  if (!serial_port_->is_open()) {
        RCLCPP_WARN(rclcpp::get_logger("ThurHardware"), "Thur hardware not connected to serial port");
        return hardware_interface::return_type::ERROR;
    }

    // We currently have an ack response, so read the frames
    std::vector<SerialHdlcFrame> frames;
    serial_port_->read_frames(frames);


  for (size_t i = 0; i < frames.size(); i++) {
      char buff[100];
      int offset = 0;
      for (size_t l = 0; l < frames[i].length; l++) {
          sprintf(&buff[offset], "%02X ", frames[i].data[l]);
          offset += 3;
      }
      RCLCPP_INFO(rclcpp::get_logger("ThurHardware"), "Frame received: %c", buff[0]);
  }

  // TODO: Write to states

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type thur ::ThurSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ThurHardware"), "In Write Method!!!s");

  /*// BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("ThurSystemHardware"), "Writing...");

  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("ThurSystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
      info_.joints[i].name.c_str());

    hw_velocities_[i] = hw_commands_[i];
  }
  RCLCPP_INFO(rclcpp::get_logger("ThurSystemHardware"), "Joints successfully written!");
  */
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  
  u_int8_t message[8];

  if (serial_port_->is_open()){
    message[0] = 1;
    message[1] = 2;
    message[2] = 3;
    message[3] = 4;
    message[4] = 5;
    message[5] = 6;
    message[6] = 7;
    message[7] = 8;


    serial_port_->write_frame(message, 8);

  }
  
  return hardware_interface::return_type::OK;
}

}  // namespace thur

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  thur::ThurSystemHardware, hardware_interface::SystemInterface)
