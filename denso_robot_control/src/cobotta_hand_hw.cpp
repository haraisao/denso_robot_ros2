// Copyright 2020 DENSO WAVE INCORPORATED
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

#include "denso_robot_control/cobotta_hand_hw.hpp"

#include <chrono>
#include <cmath>
#include <thread>
#include <functional>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace denso_robot_control {

/**
  LifeCycle On Init
 */
hardware_interface::CallbackReturn
CobottaHandHW::on_init(const hardware_interface::HardwareInfo & info) {
  RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "***** Start Hardware configuration 1");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "***** Start Hardware configuration 2");
  info_ = info;

  hw_joint_state_ = 0.015;
  hw_joint_velocity_ = 0;
  hw_joint_command_ = 0;
  hw_joint_vel_command_ = 0;
  current_state_ = 0.015;

  const hardware_interface::ComponentInfo & joint = info_.joints[0];
  // RRBotModularJoint has exactly one state and command interface on each joint

  if (joint.command_interfaces.size() != 1)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("CobottaHandHW"),
       "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
      joint.command_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("CobottaHandHW"),
       "Joint '%s' have %s command interfaces found. '%s' expected.",
      joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
      hardware_interface::HW_IF_POSITION);
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("CobottaHandHW"),
       "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
      joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "***** Num of joints %ld", info_.joints.size());
  RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "***** Hardware configured !!");
  return CallbackReturn::SUCCESS;
}

/**
  
 */
std::vector<hardware_interface::StateInterface>
CobottaHandHW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_joint_state_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_velocity_));

  return state_interfaces;
}

/**
  
 */
std::vector<hardware_interface::CommandInterface>
CobottaHandHW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_joint_command_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_vel_command_));

  return command_interfaces;
}

/**
  LifeCycle on Activate
 */
hardware_interface::CallbackReturn
CobottaHandHW::on_activate(const rclcpp_lifecycle::State & previous_state) {
  RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "====================> Starting COBOTTA hand drivers ...");

  std::string node_name = info_.hardware_parameters["node_name"].c_str();
  std::string node_namespace = info_.hardware_parameters["node_namespace"].c_str();
  std::string robot_ip_address = info_.hardware_parameters["ip_address"];
  std::string robot_name = info_.hardware_parameters["robot_name"];


  int ctrl_type = 8;

  int arm_group = 0;
  //arm_group = std::stoi(info_.hardware_parameters["arm_group"]);

  int send_format = 0;
  int recv_format = 2;

  bool verbose = true;
  //std::string str_true = "True";
  //std::string str_verbose = info_.hardware_parameters["verbose"].c_str();
  //verbose = std::equal(str_true.begin(), str_true.end(), str_verbose.begin(), str_verbose.begin() + str_true.length());
 
  if (verbose) {
    RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "*******************************************");
    RCLCPP_INFO(
      rclcpp::get_logger("CobottaHandHW"),
      "***** Verbose mode ON. List of parsed robot arguments :");
    RCLCPP_INFO(
      rclcpp::get_logger("CobottaHandHW"), "***** robot name: %s", robot_name.c_str());
  
    RCLCPP_INFO(
      rclcpp::get_logger("CobottaHandHW"),
      "***** controller type (8 = RC8 ; 9 = RC9): %d", ctrl_type);
    RCLCPP_INFO(
      rclcpp::get_logger("CobottaHandHW"),
      "***** robot ip address: %s", robot_ip_address.c_str());
    RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "***** arm group: %d", arm_group);
    RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "***** send format: %d", send_format);
    RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "***** receive format: %d", recv_format);
    RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "*******************************************");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "Arguments parsed !!");
  }

  drobo_ = std::make_shared<CobottaHandControl>(
    node_name, node_namespace, robot_name, robot_ip_address, ctrl_type,
    arm_group, send_format, recv_format, verbose);

  auto node = rclcpp::Node::make_shared(node_name, node_namespace);
  drobo_->setNode(node);

  SpinNode(node, drobo_);

  RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "System successfully started !!");
  return CallbackReturn::SUCCESS;
}

/**
  LifeCycle: on Deactivate
 */
hardware_interface::CallbackReturn
CobottaHandHW::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
  RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "Stopping robot drivers... ");
  // TODO: do we really need this wait time ??
  //drobo_->Stop();
  std::this_thread::sleep_for(std::chrono::seconds(2));

  RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "System successfully stopped !!");
  return CallbackReturn::SUCCESS;
}

/**
   Read Current position 
 */
hardware_interface::return_type
CobottaHandHW::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */) {
  std::unique_lock<std::mutex> lock_mode(mtx_mode_);
  // read robot current position
  //drobo_->read(pos_interface_);

  if (hw_joint_state_ != current_state_){
    hw_joint_state_ = current_state_;
    hw_joint_velocity_ = 0;

    RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"),
      "******** Call Read %lf,  %lf!!", hw_joint_state_, hw_joint_command_);
  }
  return return_type::OK;
}

/**
   Write Target position 
 */
hardware_interface::return_type
CobottaHandHW::write(const rclcpp::Time & /* time */, const rclcpp::Duration & period) {
  std::unique_lock<std::mutex> lock_mode(mtx_mode_);

  if (current_state_ != hw_joint_command_){
    RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"),
     "******** Call write %lf,  %lf!!", hw_joint_state_, hw_joint_command_);
    if (drobo_){
      drobo_->set_hand_pos(hw_joint_command_);
    }
    current_state_ = hw_joint_command_;
  }
  return return_type::OK;

}

// **********************************************************************************
/**
  ROS2 Massageing
 */
void
CobottaHandHW::SpinNode(rclcpp::Node::SharedPtr& node, CobottaHandControl_Ptr drobo) {
  RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "***** Starting DENSO robot control thread ...");
    std::thread denso_thread([node, drobo]() {
    rclcpp::WallRate loop_rate(125);
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      //drobo->Update();
      loop_rate.sleep();
    }
    rclcpp::shutdown();
  });

  denso_thread.detach();

  RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "***** DENSO robot control thread started !!");
}
}  // namespace denso_robot_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  denso_robot_control::CobottaHandHW,
  hardware_interface::SystemInterface
)
