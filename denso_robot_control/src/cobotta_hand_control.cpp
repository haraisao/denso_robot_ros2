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

#include "denso_robot_control/cobotta_hand_control.hpp"

#include <chrono>
#include <cmath>
#include <functional>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define RAD_2_DEG(x) ((x)*180.0 / M_PI)
#define DEG_2_RAD(x) ((x) / 180.0 * M_PI)
#define M_2_MM(x) ((x)*1000.0)
#define MM_2_M(x) ((x) / 1000.0)


namespace denso_robot_control
{
  /**
    Constructor
   */
  CobottaHandControl::CobottaHandControl(
    const std::string& node_name, const std::string& node_namespace,
    const std::string& robot_name, const std::string& robot_ip_address, int ctrl_type,
    int arm_group, int send_format, int recv_format, bool verbose)
  : node_name_(node_name), node_namespace_(node_namespace), robot_name_(robot_name),
    robot_ip_address_(robot_ip_address), ctrl_type_(ctrl_type), 
    arm_group_(arm_group), send_format_(send_format), recv_format_(recv_format), verbose_(verbose) 
    {
      m_mode=0;
     Initialize();
  }

  /**
    Deconstructor
   */
  CobottaHandControl::~CobottaHandControl() {
  }

  /**
      Initialize controller
   */
  HRESULT
  CobottaHandControl::Initialize() {
    if (NULL == node_) {
      node_ = rclcpp::Node::make_shared(node_name_, node_namespace_);
    }

    pub_hand_move_ = node_->create_publisher<std_msgs::msg::UInt32>("HandMoveA", 1);

    if (verbose_) {
      RCLCPP_INFO(
        rclcpp::get_logger(node_->get_name()),
        "***** DENSO cobotta hand control node name: %s", node_name_.c_str());
      RCLCPP_INFO(
        rclcpp::get_logger(node_->get_name()),
        "***** DENSO cobotta hand control node namespace: %s", node_namespace_.c_str());
      RCLCPP_INFO(
        rclcpp::get_logger(node_->get_name()),
        "***** DENSO cobotta robot name: %s", robot_name_.c_str());
    }

    RCLCPP_INFO(rclcpp::get_logger(node_->get_name()), "==========> Initialized COBOTTA hand");
    return S_OK;
  }

  /**
   */
  void
  CobottaHandControl::set_hand_pos(double pos) {
    double hand_w = pos*2000;
    std_msgs::msg::UInt32 msg;
    msg.data = (uint32_t)hand_w;
    pub_hand_move_->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "***** Hand pos ... %f", hand_w);
    return;
  }

  /**
   */
  void
  CobottaHandControl::Update() {
  }
  
  /**
   */
  void
  CobottaHandControl::Start() {
    RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "***** Starting DENSO robot_core thread ...");
  }

  /**
  */
  void
  CobottaHandControl::Stop() {
     RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "***** Stopping DENSO robot_core thread ...");
  }

}  // namespace denso_robot_control
