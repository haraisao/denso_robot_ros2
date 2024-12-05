/**
 * Software License Agreement (MIT License)
 *
 * @copyright Copyright (c) 2015 DENSO WAVE INCORPORATED
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef COBOTTA_HAND_CONTROL__DENSO_ROBOT_CONTROL_HPP_
#define COBOTTA_HAND_CONTROL__DENSO_ROBOT_CONTROL_HPP_

// System
#include <memory>
#include <string>
#include <vector>
#include <limits>

// ros2_control hardware_interface
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/visibility_control.h"
// ROS
#include "rclcpp/macros.hpp"
// Message (std_msgs)
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
// Action
//#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
// DENSO libraries
#include "denso_robot_core/denso_robot_core.h"
#include "denso_robot_core/denso_controller.h"
#include "denso_robot_core/denso_controller_rc8_cobotta.h"
#include "denso_robot_core/denso_robot.h"
#include "denso_robot_core/denso_variable.h"
#include "denso_robot_core_interfaces/msg/user_io.hpp"
#include "denso_robot_core_interfaces/srv/change_mode.hpp"

using namespace denso_robot_core;
using namespace std_msgs;
using hardware_interface::HardwareInfo;
using hardware_interface::return_type;

#define JOINT_MAX (8)


namespace denso_robot_control {

class CobottaHandControl
{
public:
  CobottaHandControl(
    const std::string& node_name, const std::string& node_namespace, const std::string& robot_name,
    const std::string& robot_ip_address, int ctrl_type,
    int arm_group, int send_format, int recv_format, bool verbose);
  virtual ~CobottaHandControl();

  HRESULT Initialize();

  rclcpp::Time getTime() const
  {
    return rclcpp::Clock().now();
  }

  rclcpp::Duration getPeriod() const
  {
    return ctrl_->get_Duration();
  }

  void setNode(rclcpp::Node::SharedPtr& node)
  {
    node_ = node;
  }

  void CallbackCurMode(const std_msgs::msg::UInt32::SharedPtr msg);
  void CallbackHandMove(const std_msgs::msg::UInt32::SharedPtr msg);

  void set_hand_pos(double pos);
  void Start();
  void Stop();
  void Update();

  int m_mode;

private:
  std::string node_name_;
  std::string node_namespace_;
  std::string robot_name_;
  std::string robot_ip_address_;
  int robot_joints_;
  int ctrl_type_;
  int arm_group_;
  int send_format_;
  int recv_format_;
  bool verbose_;
  rclcpp::Time start_time_, prev_time_;
  double cycle_sec_;
  
  int hand_speed_;
  int hand_force_;

  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sub_mode_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sub_hand_move_;
    
  DensoControllerRC8Cobotta_Ptr ctrl_;
  DensoRobotRC8Cobotta_Ptr rob_;
  DensoVariable_Ptr var_err_;

  std::mutex mtx_mode_;

  // ROS2 Node Handle
  rclcpp::Node::SharedPtr node_;

};

typedef std::shared_ptr<CobottaHandControl> CobottaHandControl_Ptr;

}  // namespace denso_robot_control

#endif  // COBOTTA_HAND_CONTROL__DENSO_ROBOT_CONTROL_HPP_
