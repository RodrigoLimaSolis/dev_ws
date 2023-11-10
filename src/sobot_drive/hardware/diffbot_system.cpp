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

#include "sobot_drive/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sobot_drive
{
hardware_interface::CallbackReturn SobotDriveHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }



  //Aqui vamos colocar os outros parametros da nossa API do Sobot
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);

  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  old_hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());;
  
  sobot_movement_status = PAUSE;
  sobot_status = PAUSE;
  old_sobot_status = PAUSE;

  command_later = 0;
  flag_current_on = false;

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SobotDriveHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SobotDriveHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SobotDriveHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SobotDriveHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SobotDriveHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SobotDriveHardware::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> SobotDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn SobotDriveHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"), "Activating ...please wait...");


  if (comms_.connected())
  {
    comms_.disconnect();
  }

  //Configurações Iniciais dos Motores
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  comms_.send_msg("MT0 MC MD0 AT2000 DT2000 V10");
  comms_.send_msg("LT E1 RD00 GR00 BL100");
  comms_.send_msg("MT0 ME1");




  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
    rclcpp::get_logger("SobotDriveHardware"), "%.1f seconds left...", hw_start_sec_ - i);
  }
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

  RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}









hardware_interface::CallbackReturn SobotDriveHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"), "Deactivating ...please wait...");

  comms_.disconnect();

  for (auto i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("SobotDriveHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}






hardware_interface::return_type SobotDriveHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{



  // Rotina para coletar os valores
  std::string  retorno = "";
  retorno = comms_.read_status();
  RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"),
      "Comandos retornados %s",retorno.c_str());

  //MT0 MS D+00000,26
  //Verificar se está indo pra frente ou pra trás
  
    retorno[14] = '.';
    std::stringstream num_status;
    for (int i = 10; i < 17; i++) {
        num_status << retorno[i];
    }
    std::cout << num_status.str();
    float temp_position = 0;
    
    temp_position = std::stof(num_status.str());

    
    if((sobot_movement_status == FORWARD)||(sobot_movement_status == BACKWARD)){
      
      temp_position = temp_position / 1000 / 0.314159265 * 360 * M_PI / 180.0;

      if(retorno[8] == '-') temp_position *= -1;
      hw_velocities_[0] = (temp_position / period.seconds());
      hw_velocities_[1] = (temp_position / period.seconds());

      hw_positions_[0] += temp_position; 
      hw_positions_[1] += temp_position;
      
      }


    if((sobot_movement_status == RIGHT)||(sobot_movement_status == LEFT)){
      
      temp_position = temp_position / 1000 / 0.314159265 * 360 * M_PI / 180.0;

      if(sobot_movement_status == RIGHT){
        hw_velocities_[0] = (temp_position / period.seconds());
        hw_velocities_[1] = ((temp_position * -1) / period.seconds());
        hw_positions_[0] += temp_position;
        hw_positions_[1] += temp_position * -1;
      }
      
      else{
        hw_velocities_[0] = ((temp_position * -1) / period.seconds());
        hw_velocities_[1] = ( temp_position / period.seconds());
        hw_positions_[0] += temp_position * -1;
        hw_positions_[1] += temp_position;
      }
    }


    if((sobot_movement_status == RIGHT_DIFF_FORWARD)||(sobot_movement_status == LEFT_DIFF_FORWARD)){
      
      temp_position = temp_position / 1000 / 0.314159265 * 360 * M_PI / 180.0;
     
      if(sobot_movement_status == RIGHT_DIFF_FORWARD){
        
        hw_velocities_[0] = temp_position / period.seconds();
        hw_positions_[0] += temp_position;
        
        temp_position = temp_position / 1.298850575;
        
        hw_velocities_[1] = temp_position / period.seconds();
        hw_positions_[1] += temp_position;
      }
      else{
        hw_velocities_[1] = temp_position / period.seconds();
        hw_positions_[1] += temp_position;
        
        temp_position = temp_position / 1.298850575;
        
        hw_velocities_[0] = temp_position / period.seconds();
        hw_positions_[0] += temp_position;
      }
    
    }
    if((sobot_movement_status == RIGHT_DIFF_BACKWARD)||(sobot_movement_status == LEFT_DIFF_BACKWARD))
    {
      
      temp_position = temp_position / 1000 / 0.314159265 * 360 * M_PI / 180.0;
      temp_position *= -1;
      
      if(sobot_movement_status == RIGHT_DIFF_FORWARD){
        
        hw_velocities_[0] = temp_position / period.seconds();
        hw_positions_[0] += temp_position;
        
        temp_position = temp_position / 1.298850575;
        
        hw_velocities_[1] = temp_position / period.seconds();
        hw_positions_[1] += temp_position;
      }
      else
      {
        hw_velocities_[1] = temp_position / period.seconds();
        hw_positions_[1] += temp_position;
        
        temp_position = temp_position / 1.298850575;

        
        hw_velocities_[0] = temp_position / period.seconds();
        hw_positions_[0] += temp_position;
      }
    }
  
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  for (std::size_t i = 0; i < hw_velocities_.size(); i++)
  {
    // Simulate DiffBot wheels's movement as a first-order system
    // Update the joint status: this is a revolute joint without any limit.
    // Simply integrates
    
  //hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];

    RCLCPP_INFO(
      rclcpp::get_logger("SobotDriveHardware"),
      "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
      hw_velocities_[i], info_.joints[i].name.c_str());
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type sobot_drive ::SobotDriveHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"), "Writing...");
  
    RCLCPP_INFO(
      rclcpp::get_logger("SobotDriveHardware"), "Hw_commands[0]: %f e o ultimo: %f", 
      hw_commands_[0], command_later);




      // CONTROLE DA ENTRADA DE MOVIMENTAÇÃO
      if((hw_commands_[0] > 0) && (hw_commands_[1] > 0)){

        if (hw_commands_[0] == hw_commands_[1]) sobot_status = FORWARD;
        else if ((hw_commands_[0] < hw_commands_[1])&&(hw_commands_[0] == old_hw_commands_[0])) sobot_status = LEFT_DIFF_FORWARD;
        else if ((hw_commands_[0] > hw_commands_[1])&&(hw_commands_[0] == old_hw_commands_[0])) sobot_status = RIGHT_DIFF_FORWARD;

      } 
      else if ((hw_commands_[0] < 0) && (hw_commands_[1] < 0)){

        if (hw_commands_[0] == hw_commands_[1]) sobot_status = BACKWARD;
        else if ((hw_commands_[0] < hw_commands_[1])&&(hw_commands_[0] == old_hw_commands_[0])) sobot_status = RIGHT_DIFF_BACKWARD;
        else if ((hw_commands_[0] > hw_commands_[1])&&(hw_commands_[0] == old_hw_commands_[0])) sobot_status = LEFT_DIFF_BACKWARD;


      }
      else if (((hw_commands_[0] > 0) && (hw_commands_[1] < 0))
      || ((hw_commands_[0] < 0)&&(hw_commands_[1] > 0)))
      {

        if(hw_commands_[0] > hw_commands_[1]) sobot_status = RIGHT;
        if(hw_commands_[0] < hw_commands_[1]) sobot_status = LEFT;

      }


      // CONTROLE DOS PAUSES
      if((sobot_status == FORWARD)  || (sobot_status == RIGHT_DIFF_FORWARD) || (sobot_status == RIGHT))  {
  
        if(hw_commands_[0] < old_hw_commands_[0]) sobot_status = PAUSE;

      }

      else if((sobot_status == BACKWARD) ||(sobot_status == LEFT) || (sobot_status == RIGHT_DIFF_BACKWARD))  {
  
        if(hw_commands_[0] > old_hw_commands_[0]) sobot_status = PAUSE;

      }
      
      else if(sobot_status == LEFT_DIFF_BACKWARD){ 
       if((hw_commands_[0] > old_hw_commands_[0])&&(hw_commands_[1] > old_hw_commands_[1])) sobot_status = PAUSE;
      }
      
      else if(sobot_status == LEFT_DIFF_FORWARD) if(hw_commands_[1] < old_hw_commands_[1]) sobot_status = PAUSE;


      old_hw_commands_[1] = hw_commands_[1];
      old_hw_commands_[0] = hw_commands_[0];



    // SELECIONANDO TIPO DE MOVIMENTO
    if(old_sobot_status != sobot_status ){

      old_sobot_status = sobot_status;
      switch (sobot_status) {

          case FORWARD:
              sobot_movement_status = FORWARD;
              comms_.send_msg("LT E1 RD00 GR100 BL00");
              comms_.send_msg("MT0 MF");
              break;

          case BACKWARD:
              sobot_movement_status = BACKWARD;            
              comms_.send_msg("LT E1 RD00 GR00 BL100");
              comms_.send_msg("MT0 MB");
              break;

          case RIGHT:
              sobot_movement_status = RIGHT;
              
              comms_.send_msg("LT E1 RD50 GR00 BL100");
              comms_.send_msg("MT0 MC MD0 AT2000 DT2000 V10");
              comms_.send_msg("MT0 MR");
              break;

          case LEFT:
              sobot_movement_status = LEFT;
              
              comms_.send_msg("LT E1 RD00 GR50 BL100");
              comms_.send_msg("MT0 MC MD0 AT2000 DT2000 V10");
              comms_.send_msg("MT0 ML");
              break;

          case RIGHT_DIFF_FORWARD:
              sobot_movement_status = RIGHT_DIFF_FORWARD;
              
              comms_.send_msg("LT E1 RD00 GR100 BL40");
              comms_.send_msg("MT0 MC MD1 RI870 AT2000 DT2000 V10");
              comms_.send_msg("MT0 MR");
              break;

          case LEFT_DIFF_FORWARD:
              sobot_movement_status =LEFT_DIFF_FORWARD ;
              
              comms_.send_msg("LT E1 RD40 GR100 BL00");
              comms_.send_msg("MT0 MC MD1 RI870 AT2000 DT2000 V10"); 
              comms_.send_msg("MT0 ML");
              break;

          case RIGHT_DIFF_BACKWARD:
              sobot_movement_status = RIGHT_DIFF_BACKWARD;
              
              comms_.send_msg("LT E1 RD100 GR50 BL100");
              comms_.send_msg("MT0 MC MD1 RI870 AT2000 DT2000 V10"); 
              comms_.send_msg("MT0 MR-");
              break;

          case LEFT_DIFF_BACKWARD:
              sobot_movement_status = LEFT_DIFF_BACKWARD;
              
              comms_.send_msg("LT E1 RD100 GR00 BL50");
              comms_.send_msg("MT0 MC MD1 RI870 AT2000 DT2000 V10"); 
              comms_.send_msg("MT0 ML-");
              break;
          
          case PAUSE:
              
              comms_.send_msg("LT E1 RD100 GR00 BL50");
              comms_.send_msg("MT0 MP");
              break;


          case BREAK:
              
              comms_.send_msg("LT E1 RD100 GR00 BL00");
              comms_.send_msg("MT0 MB");
              break;
    }
  }

    RCLCPP_INFO(
      rclcpp::get_logger("SobotDriveHardware"), "Sobot Status: %d e Sobot Status Antigo: %d", 
      sobot_status, old_sobot_status);

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"), "Writing...");
  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("SobotDriveHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
      info_.joints[i].name.c_str());

    hw_velocities_[i] = hw_commands_[i];
  }
  RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace sobot_drive

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  sobot_drive::SobotDriveHardware, hardware_interface::SystemInterface)
