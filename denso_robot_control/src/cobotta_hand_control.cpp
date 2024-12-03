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
    memset(type_, 0, sizeof(type_));
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

    //node_->declare_parameter("bcap_slave_control_cycle_sec", 0.008);
    //cycle_sec_ = node_->get_parameter("bcap_slave_control_cycle_sec").as_double();

    //joint_.resize(robot_joints_);
    //memset(cmd_, 0, sizeof(cmd_));
    verbose_ = true;
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

    eng_ = std::make_shared<DensoRobotCore>(node_, robot_ip_address_, robot_name_, ctrl_type_);

    if (verbose_) {
      RCLCPP_INFO(
        rclcpp::get_logger(node_->get_name()), "[DEBUG] Initializing b-cap engine ...");
    }

    /**
      Initialize ORiN ports
     */
    HRESULT hr = eng_->Initialize();
    if (FAILED(hr)) {
      RCLCPP_FATAL(
        rclcpp::get_logger(node_->get_name()), "Failed to connect real controller. (%X)", hr);
      return hr;
    }
 #if 1
    if (verbose_) {
      RCLCPP_INFO(rclcpp::get_logger(node_->get_name()), "[DEBUG] Adding hand controller ...");
    }
    double ctrl_cycle_msec = 8;
    int m_mode = 0;
    std::string filename;
    node_->get_parameter("denso_config_file", filename);
    std::cerr << "======= denso_config_file: " << filename << std::endl;
    //ctrl_ = eng_->get_Controller();
    ctrl_ = std::make_shared<DensoControllerRC8Cobotta>(
          node_, robot_name_, &m_mode, robot_ip_address_,
          rclcpp::Duration(std::chrono::duration<double>(ctrl_cycle_msec / 1000.0)));
    HRESULT res = ctrl_->InitializeBCAP(filename);
    std::cerr << "======= InitializeBCAP: " << res << std::endl;
#endif
#if 0
    if (verbose_) {
      RCLCPP_INFO(rclcpp::get_logger(node_->get_name()), "[DEBUG] Adding robot arm ...");
    }

    /// get ORiN interfaces
    DensoRobotRC8Cobotta_Ptr pRob;
    hr = ctrl_->get_Robot(DensoBase::SRV_ACT, &pRob);
    if (FAILED(hr)) {
      RCLCPP_FATAL(
        rclcpp::get_logger(node_->get_name()), "Failed to connect real robot. (%X)", hr);
      return hr;
    }

    rob_ = pRob;

    hr = CheckRobotType();
    if (FAILED(hr)) {
      RCLCPP_FATAL(rclcpp::get_logger(node_->get_name()), "Invalid robot type.");
      return hr;
    }
    rob_->ChangeArmGroup(arm_group_);

    /// Error code monitor
    hr = ctrl_->AddVariable("@ERROR_CODE");
    if (FAILED(hr)) {
      printErrorDescription(hr, "Failed to add @ERROR_CODE object");
      return hr;
    }
    hr = ctrl_->get_Variable("@ERROR_CODE", &var_err_);
    if (FAILED(hr)) {
      printErrorDescription(hr, "Failed to get @ERROR_CODE object");
      return hr;
    }

    rob_->set_SendFormat(send_format_);
    rob_->set_RecvFormat(recv_format_);

    //pub_cur_mode_ = node_->create_publisher<std_msgs::msg::Int32>("CurMode", 1);
    //pub_error_code_ = node_->create_publisher<std_msgs::msg::UInt32>("ErrorCode", 1);
#endif
    RCLCPP_INFO(rclcpp::get_logger(node_->get_name()), "==========> Initialized COBOTTA hand");
    return S_OK;
  }

  /**
   */
  HRESULT
  CobottaHandControl::ChangeModeWithClearError(int mode)
  {
    HRESULT hr = eng_->ChangeMode(mode, mode == DensoRobot::SLVMODE_NONE);
    if (FAILED(hr)) {
      // Clear Error
      HRESULT hres = ctrl_->ExecClearError();
      if (FAILED(hres)) {
        printErrorDescription(hres, "Failed to clear error");
      }
    }
    return hr;
  }

  /**

   */
  HRESULT CobottaHandControl::CheckRobotType() {
    DensoVariable_Ptr p_var;
    VARIANT_Ptr vnt_val(new VARIANT());
    std::string type_name = "@TYPE_NAME";

    HRESULT hr = rob_->AddVariable(type_name);
    if (FAILED(hr)) {
      printErrorDescription(hr, "Failed to add @TYPE_NAME");
      return hr;
    }
    rob_->get_Variable(type_name, &p_var);
    hr = p_var->ExecGetValue(vnt_val);
    if (FAILED(hr)) {
      printErrorDescription(hr, "Failed to get @TYPE_NAME");
      return hr;
    }
    type_name = DensoBase::ConvertBSTRToString(vnt_val->bstrVal);
    RCLCPP_INFO(
      rclcpp::get_logger(node_->get_name()),
      "***** Full robot name: %s", type_name.c_str());
    if (
    strncmp(
      robot_name_.c_str(), type_name.c_str(),
      (robot_name_.length() < type_name.length()) ? robot_name_.length() : type_name.length()))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(node_->get_name()), "Expected robot type is %s , real robot type is %s",
        robot_name_.c_str(), type_name.c_str());
      return E_FAIL;
    }

    return 0;
  }

  /**
  
   */
  bool
  CobottaHandControl::hasError() {
    HRESULT hr;
    VARIANT_Ptr vnt_val(new VARIANT());
    hr = var_err_->ExecGetValue(vnt_val);
    if (SUCCEEDED(hr) && (vnt_val->lVal == 0)) {
      return false;
    }
    return true;
  }

  /**
  
   */
  void
  CobottaHandControl::printErrorDescription(HRESULT error_code,
                                 const std::string& error_message) {
    HRESULT hr;
    std::string error_description;
    if (eng_->get_Mode() == DensoRobot::SLVMODE_NONE) {
      hr = ctrl_->ExecGetErrorDescription(error_code, error_description);
      if (SUCCEEDED(hr)) {
        RCLCPP_FATAL(
          rclcpp::get_logger(node_->get_name()), "%s: %s (%X)",
          error_message.c_str(), error_description.c_str(), error_code);
        return;
      }
    }
    RCLCPP_FATAL(
      rclcpp::get_logger(node_->get_name()), "%s (%X)", error_message.c_str(), error_code);
  }

  /**
   */
  void
  CobottaHandControl::set_hand_pos(double pos) {
    double hand_w = pos*2000;
    ctrl_->HandMove(hand_w);
    RCLCPP_INFO(rclcpp::get_logger("CobottaHandHW"), "***** Hand pos ... %f", hand_w);
    return;
  }


  /**
  
   */
  hardware_interface::return_type
  CobottaHandControl::read(std::vector<double>& pos_interface) {
    std::unique_lock<std::mutex> lock_mode(mtx_mode_);

    return return_type::OK;
  }

  /**
   */
  hardware_interface::return_type
  CobottaHandControl::write(std::vector<double>& cmd_interface,
                     std::vector<double>& prev_cmd_interface, double duration) {
    //if(duration < cycle_sec_/3){ return return_type::OK;}
    std::unique_lock<std::mutex> lock_mode(mtx_mode_);

    return return_type::OK;
  }

  /**
   */
  void
  CobottaHandControl::Update() {
    //ctrl_->Update();
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
