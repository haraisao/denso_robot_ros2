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

#ifndef ROBOTTA_HAND_CONTROL__DENSO_ROBOT_HW_HPP_
#define ROBOTTA_HAND_CONTROL__DENSO_ROBOT_HW_HPP_


#include "denso_robot_control/cobotta_hand_control.hpp"

namespace denso_robot_control {

class CobottaHandHW : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CobottaHandHW)

  HRESULT Initialize();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void SpinNode(rclcpp::Node::SharedPtr& node, CobottaHandControl_Ptr drobo);

  HardwareInfo info_;
  // Store the commands for the real robot
  std::vector<double> cmd_interface_;
  std::vector<double> prev_cmd_interface_;
  std::vector<double> cmd_vel_interface_;
  std::vector<double> pos_interface_;
  std::vector<double> vel_interface_;
  std::vector<double> eff_interface_;

private:
  CobottaHandControl_Ptr drobo_ = NULL;

  std::mutex mtx_mode_;
  //// for test

  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;
  double current_state_;

  // Store the command for the simulated robot
  double hw_joint_command_;
  double hw_joint_vel_command_;
  double hw_joint_state_;
  double hw_joint_velocity_;
};

}  // namespace denso_robot_control

#endif  // DENSO_HAND_CONTROL__DENSO_ROBOT_HW_HPP_
