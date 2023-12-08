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
  cfg_.update_rate = std::stoi(info_.hardware_parameters["update_rate"]);
  cfg_.wheel_separation = std::stof(info_.hardware_parameters["wheel_separation"]);
  cfg_.wheel_radius = std::stof(info_.hardware_parameters["wheel_radius"]);


  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);

  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  old_hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());


  PERIMETRO = cfg_.wheel_radius * 2 * M_PI;

  
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
  comms_.send_msg("MT0 MC MD0 AT100 DT100 V5");
  comms_.send_msg("WP MT1 WD100,20"); 
  comms_.send_msg("WP MT2 WD99,95"); 
  comms_.send_msg("WP DW260,40"); //280.7-20.3 
  comms_.send_msg("LT E1 RD00 GR00 BL50");
  comms_.send_msg("MT0 E1");



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
  comms_.send_msg("LT E0 RD00 GR00 BL00");

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
        else if (hw_commands_[0] < hw_commands_[1]) sobot_status = LEFT_DIFF_FORWARD;
        else if (hw_commands_[0] > hw_commands_[1]) sobot_status = RIGHT_DIFF_FORWARD;

      } 
      else if ((hw_commands_[0] < 0) && (hw_commands_[1] < 0)){

        if (hw_commands_[0] == hw_commands_[1]) sobot_status = BACKWARD;
        else if (hw_commands_[0] < hw_commands_[1]) sobot_status = RIGHT_DIFF_BACKWARD;
        else if (hw_commands_[0] > hw_commands_[1]) sobot_status = LEFT_DIFF_BACKWARD;


      }
      else if (((hw_commands_[0] > 0) && (hw_commands_[1] < 0))
      || ((hw_commands_[0] < 0)&&(hw_commands_[1] > 0)))
      {

        if(hw_commands_[0] > hw_commands_[1]) sobot_status = RIGHT;
        if(hw_commands_[0] < hw_commands_[1]) sobot_status = LEFT;
      }


      char command_sobot[50];
      float raio_curva = 0;
      float distancia = 0;
      float distancia_left, distancia_right, delta_ang,raio_separacao,delta_ang_graus;

      //Executando o comando
      switch (sobot_status) {

          case FORWARD:
              distancia = (hw_commands_[0] / (2 * M_PI)) ;
              distancia =  distancia * PERIMETRO * 100;
              RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"),"Porcentagem: %f Perimetro: %f",distancia, PERIMETRO);
              sprintf(command_sobot, "MT0 D%d AT000 DT000 V%d", int(distancia),int(distancia));
              comms_.send_msg(command_sobot);
              break;

          case BACKWARD:
              distancia = (hw_commands_[0] / (2 * M_PI)) * PERIMETRO *100;
              sprintf(command_sobot, "MT0 D%d AT000 DT000 V%d", int(distancia),int(distancia)*-1);
              comms_.send_msg(command_sobot);
              break;

          case RIGHT:
              sobot_movement_status = RIGHT;
              distancia = (hw_commands_[0] / (2 * M_PI)) * PERIMETRO; 
              //RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"),"Porcentagem: %f",distancia);
              raio_curva = (distancia * 360 / cfg_.wheel_separation * M_PI)/10; 
              //RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"),"Porcentagem: %f",raio_curva/cfg_.update_rate);
              sprintf(command_sobot, "MT0 D%d,%d%d R AT000 DT000 V%d", int(raio_curva/cfg_.update_rate), int(raio_curva/cfg_.update_rate*10)%10,int(raio_curva/cfg_.update_rate*100)%10,int(distancia*100));
              comms_.send_msg(command_sobot);
              break;

          case LEFT:
              sobot_movement_status = LEFT;
              distancia = ((hw_commands_[0] / (2 * M_PI)) * PERIMETRO) * -1; 
              //RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"),"Porcentagem: %f",distancia);
              raio_curva = (distancia * 360 / cfg_.wheel_separation * M_PI)/10; 
              //RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"),"Porcentagem: %f",raio_curva/cfg_.update_rate);
              sprintf(command_sobot, "MT0 D%d,%d%d L AT000 DT000 V%d", int(raio_curva/cfg_.update_rate), int(raio_curva/cfg_.update_rate*10)%10,int(raio_curva/cfg_.update_rate*100)%10,int(distancia*100));
              comms_.send_msg(command_sobot);
              break;

          case RIGHT_DIFF_FORWARD:
              sobot_movement_status = RIGHT_DIFF_FORWARD;
              distancia_left = ((hw_commands_[0] / (2 * M_PI)) * PERIMETRO); 
              distancia_right = ((hw_commands_[1] / (2 * M_PI)) * PERIMETRO); 

              RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"),"Distancia Left: %f Distancia Rigth: %f",distancia_left, distancia_right);

              delta_ang = (distancia_left-distancia_right)/cfg_.wheel_separation;
              delta_ang_graus = delta_ang*180/M_PI;
              RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"),"Delta Angulo: %f - Graus: %f",delta_ang, delta_ang_graus);

              raio_separacao = distancia_right/delta_ang;
              RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"),"Raio de separação: %f",raio_separacao);
              sprintf(command_sobot, "MT0 D%d,%d%d DF R RI%d V%d",int(delta_ang_graus/cfg_.update_rate), int(delta_ang_graus/cfg_.update_rate*10)%10, int(delta_ang_graus/cfg_.update_rate*100)%10,int(raio_separacao*1000), int(distancia_left*100));
              comms_.send_msg(command_sobot);
              break;

         case LEFT_DIFF_FORWARD:
              sobot_movement_status =LEFT_DIFF_FORWARD ;
              
              distancia_left = ((hw_commands_[0] / (2 * M_PI)) * PERIMETRO); 
              distancia_right = ((hw_commands_[1] / (2 * M_PI)) * PERIMETRO); 

              RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"),"Distancia Left: %f Distancia Rigth: %f",distancia_left, distancia_right);

              delta_ang = (distancia_right - distancia_left)/cfg_.wheel_separation;
              delta_ang_graus = delta_ang*180/M_PI;
              RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"),"Delta Angulo: %f - Graus: %f",delta_ang, delta_ang_graus);

              raio_separacao = distancia_left/delta_ang;
              RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"),"Raio de separação: %f",raio_separacao);
              sprintf(command_sobot, "MT0 D%d,%d%d DF L RI%d V%d",int(delta_ang_graus/cfg_.update_rate), int(delta_ang_graus/cfg_.update_rate*10)%10, int(delta_ang_graus/cfg_.update_rate*100)%10,int(raio_separacao*1000), int(distancia_right*100));
              comms_.send_msg(command_sobot);
              break;

          case RIGHT_DIFF_BACKWARD:
              sobot_movement_status = RIGHT_DIFF_BACKWARD;
              
              distancia_left = ((hw_commands_[0] / (2 * M_PI)) * PERIMETRO)*-1; 
              distancia_right = ((hw_commands_[1] / (2 * M_PI)) * PERIMETRO)*-1; 

              RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"),"Distancia Left: %f Distancia Rigth: %f",distancia_left, distancia_right);

              delta_ang = (distancia_left-distancia_right)/cfg_.wheel_separation;
              delta_ang_graus = delta_ang*180/M_PI;
              RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"),"Delta Angulo: %f - Graus: %f",delta_ang, delta_ang_graus);

              raio_separacao = distancia_right/delta_ang;
              RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"),"Raio de separação: %f",raio_separacao);
              sprintf(command_sobot, "MT0 D-%d,%d%d DF R RI%d V%d",int(delta_ang_graus/cfg_.update_rate), int(delta_ang_graus/cfg_.update_rate*10)%10, int(delta_ang_graus/cfg_.update_rate*100)%10,int(raio_separacao*1000), int(distancia_left*100));
              comms_.send_msg(command_sobot);
              break;

          case LEFT_DIFF_BACKWARD:
              sobot_movement_status = LEFT_DIFF_BACKWARD;
              
              distancia_left = ((hw_commands_[0] / (2 * M_PI)) * PERIMETRO)*-1; 
              distancia_right = ((hw_commands_[1] / (2 * M_PI)) * PERIMETRO)*-1; 

              RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"),"Distancia Left: %f Distancia Rigth: %f",distancia_left, distancia_right);

              delta_ang = (distancia_right - distancia_left)/cfg_.wheel_separation;
              delta_ang_graus = delta_ang*180/M_PI;
              RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"),"Delta Angulo: %f - Graus: %f",delta_ang, delta_ang_graus);

              raio_separacao = distancia_left/delta_ang;
              RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"),"Raio de separação: %f",raio_separacao);
              sprintf(command_sobot, "MT0 D-%d,%d%d DF L RI%d V%d",int(delta_ang_graus/cfg_.update_rate), int(delta_ang_graus/cfg_.update_rate*10)%10, int(delta_ang_graus/cfg_.update_rate*100)%10,int(raio_separacao*1000), int(distancia_right*100));
              comms_.send_msg(command_sobot);
              break;
      }
      
      RCLCPP_INFO(rclcpp::get_logger("SobotDriveHardware"),"Mandando %s, distância: %f, raio: %f",command_sobot, distancia, raio_curva);



    RCLCPP_INFO(
      rclcpp::get_logger("SobotDriveHardware"), "Sobot Status: %d e Sobot Status Antigo: %d", 
      sobot_status, old_sobot_status);

      old_hw_commands_[1] = hw_commands_[1];
      old_hw_commands_[0] = hw_commands_[0];



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
