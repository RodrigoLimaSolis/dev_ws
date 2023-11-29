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

#ifndef SOBOT_DRIVE__DIFFBOT_SYSTEM_HPP_
#define SOBOT_DRIVE__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "sobot_drive/visibility_control.h"
#include "sobot_drive/sobot_comms.hpp"


namespace sobot_drive
{
class SobotDriveHardware : public hardware_interface::SystemInterface
{

//Aqui vamos colocar os outros parametros da nossa API do Sobot
struct Config
{
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  float update_rate = 0;  
  float wheel_separation = 0;
  float wheel_radius = 0;

};


public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SobotDriveHardware);

  SOBOT_DRIVE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  SOBOT_DRIVE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  SOBOT_DRIVE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  SOBOT_DRIVE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  SOBOT_DRIVE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  SOBOT_DRIVE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  SOBOT_DRIVE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Store the command for the simulated robot
  SobotCommns comms_;
  Config cfg_;
  std::vector<double> old_hw_commands_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  float command_later;
  bool flag_current_on;

  float PERIMETRO;
  float PERIMETRO_ENTRE_RODAS;


  enum SOBOT_STATE {
        FORWARD, //0
        BACKWARD, //1
        RIGHT, //2
        LEFT, //3
        RIGHT_DIFF_FORWARD, //4
        LEFT_DIFF_FORWARD, //5
        RIGHT_DIFF_BACKWARD, //6
        LEFT_DIFF_BACKWARD, //7
        PAUSE, //8
        BREAK, //9
    };
    
  SOBOT_STATE sobot_status ;
  SOBOT_STATE old_sobot_status ;
  SOBOT_STATE sobot_movement_status ;
  SOBOT_STATE update_rate ;
  SOBOT_STATE wheel_separation ;
  SOBOT_STATE wheel_radius ;

  };
}  // namespace sobot_drive

#endif  // SOBOT_DRIVE__DIFFBOT_SYSTEM_HPP_
