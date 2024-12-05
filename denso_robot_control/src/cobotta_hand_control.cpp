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
    start_time_ = getTime();
    prev_time_ = start_time_;

    std::string filename;
    node_->declare_parameter("denso_config_file", "");
    if(!node_->get_parameter("denso_config_file", filename)) {
      return E_FAIL;
    }
    node_->declare_parameter("hand_speed", 100);
    hand_speed_ = node_->get_parameter("hand_speed").as_int();
    node_->declare_parameter("hand_force", 20.0);
    hand_force_ = node_->get_parameter("hand_force").as_double();

    //node_->declare_parameter("bcap_slave_control_cycle_sec", 0.008);
    //cycle_sec_ = node_->get_parameter("bcap_slave_control_cycle_sec").as_double();

    sub_mode_  =  node_->create_subscription<std_msgs::msg::UInt32>("CurMode", 1,
          std::bind(&CobottaHandControl::CallbackCurMode, this, std::placeholders::_1));
    sub_hand_move_  =  node_->create_subscription<std_msgs::msg::UInt32>("HandMove", 1,
          std::bind(&CobottaHandControl::CallbackHandMove, this, std::placeholders::_1));

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

    if (verbose_) {
      RCLCPP_INFO(rclcpp::get_logger(node_->get_name()), "[DEBUG] Adding hand controller ...");
    }

    ctrl_ = std::make_shared<DensoControllerRC8Cobotta>(
          node_, robot_name_, &m_mode, robot_ip_address_,
          rclcpp::Duration(std::chrono::duration<double>(0.008)));
    HRESULT hr = ctrl_->InitializeBCAP(filename);
    if (FAILED(hr)) {
      RCLCPP_ERROR(rclcpp::get_logger(node_->get_name()), "!!!!!!! Failed to Initialize COBOTTA hand");
      return hr;
    }
    RCLCPP_INFO(rclcpp::get_logger(node_->get_name()), "==========> Initialized COBOTTA hand");
    return S_OK;
  }

  /**
   */
  void
  CobottaHandControl::set_hand_pos(double pos) {
    if(m_mode == 0) { return; }
    double hand_w = pos*2000;
    ctrl_->HandMove(hand_w, hand_speed_);
    RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "***** Hand pos ... %f", hand_w);
    return;
  }
  /**
   */
  void
  CobottaHandControl::CallbackCurMode(const std_msgs::msg::UInt32::SharedPtr msg) {
    m_mode = msg->data;
  }
  /**
   */
  void
  CobottaHandControl::CallbackHandMove(const std_msgs::msg::UInt32::SharedPtr msg) {
    ctrl_->HandMoveAH((double)msg->data, hand_speed_, hand_force_);
  } 

  /**
   */
  void
  CobottaHandControl::Update() {
    ctrl_->Update();
  }
  
  /**
   */
  void
  CobottaHandControl::Start() {
    RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "***** Starting DENSO robot_core thread ...");
    //eng_->Start();
  }

  /**
  */
  void
  CobottaHandControl::Stop() {
     RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "***** Stopping DENSO robot_core thread ...");
    //eng_->Stop();
  }

}  // namespace denso_robot_control
