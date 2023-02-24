// Copyright (c) 2020 New Eagle, All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/** \brief This file defines the RaptorDbwCAN class.
 * \copyright Copyright 2021 New Eagle LLC
 * \file raptor_dbw_can.hpp
 */

#ifndef RAPTOR_DBW_CAN__RAPTOR_DBW_CAN_HPP_
#define RAPTOR_DBW_CAN__RAPTOR_DBW_CAN_HPP_

#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
//#include <raptor_pdu_msgs/msg/relay_command.hpp>          // not used.
//#include <raptor_pdu_msgs/msg/relay_state.hpp>            // not used.
#include <raptor_dbw_msgs/msg/accelerator_pedal_cmd.hpp>
#include <raptor_dbw_msgs/msg/accelerator_pedal_report.hpp>
//#include <raptor_dbw_msgs/msg/actuator_control_mode.hpp>  // not dispatched.
#include <raptor_dbw_msgs/msg/brake2_report.hpp>
#include <raptor_dbw_msgs/msg/brake_cmd.hpp>
//#include <raptor_dbw_msgs/msg/brake_report.hpp>           // dispatched, not stated in dbc.
//#include <raptor_dbw_msgs/msg/driver_input_report.hpp>    // dispatched, not stated in dbc.
//#include <raptor_dbw_msgs/msg/exit_report.hpp>            // not dispatched.
//#include <raptor_dbw_msgs/msg/fault_actions_report.hpp>   // dispatched, not stated in dbc.
//#include <raptor_dbw_msgs/msg/gear_cmd.hpp>               // dispatched, not stated in dbc, does not apply.
//#include <raptor_dbw_msgs/msg/gear_report.hpp>            // dispatched, not stated in dbc, does not apply.
//#include <raptor_dbw_msgs/msg/global_enable_cmd.hpp>      // not dispatched.
//#include <raptor_dbw_msgs/msg/gps_reference_report.hpp>   // not dispatched.
//#include <raptor_dbw_msgs/msg/gps_remainder_report.hpp>   // not dispatched.
//#include <raptor_dbw_msgs/msg/hmi_global_enable_report.hpp> // dispatched, not stated in dbc.
//#include <raptor_dbw_msgs/msg/low_voltage_system_report.hpp>// dispatched, not stated in dbc.
//#include <raptor_dbw_msgs/msg/misc_cmd.hpp>               // not dispatched.
//#include <raptor_dbw_msgs/msg/misc_report.hpp>            // dispatched, not stated in dbc. misc_report_do is used instead.
//#include <raptor_dbw_msgs/msg/other_actuators_report.hpp> // dispatched, not stated in dbc.
//#include <raptor_dbw_msgs/msg/steering2_report.hpp>       // dispatched, not stated in dbc.
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>
#include <raptor_dbw_msgs/msg/steering_cmd.hpp>
#include <raptor_dbw_msgs/msg/steering_report.hpp>
//#include <raptor_dbw_msgs/msg/surround_report.hpp>        // dispatched, not stated in dbc.
#include <raptor_dbw_msgs/msg/tire_pressure_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_position_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
//#include <sensor_msgs/msg/imu.hpp>                        // dispatched, not stated in dbc.
#include <sensor_msgs/msg/joint_state.hpp>
//#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
//#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>                          // used for gear shift cmd.

#include <deep_orange_msgs/msg/base_to_car_summary.hpp>
#include <deep_orange_msgs/msg/diagnostic_report.hpp>
#include <deep_orange_msgs/msg/lap_time_report.hpp>
#include <deep_orange_msgs/msg/brake_temp_report.hpp>
#include <deep_orange_msgs/msg/ct_report.hpp>
#include <deep_orange_msgs/msg/misc_report.hpp>
#include <deep_orange_msgs/msg/rc_to_ct.hpp>
// #include <deep_orange_msgs/msg/pos_time.hpp>
#include <deep_orange_msgs/msg/coordinates.hpp>
#include <deep_orange_msgs/msg/pt_report.hpp>
#include <deep_orange_msgs/msg/tire_report.hpp>
#include <deep_orange_msgs/msg/mylaps_report.hpp>

// temp stuff
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>

#include <can_dbc_parser/DbcMessage.hpp>
#include <can_dbc_parser/DbcSignal.hpp>
#include <can_dbc_parser/Dbc.hpp>
#include <can_dbc_parser/DbcBuilder.hpp>

#include <cmath>
#include <string>
#include <vector>

#include "raptor_dbw_can/dispatch.hpp"

using namespace std::chrono_literals;  // NOLINT

using can_msgs::msg::Frame;
using geometry_msgs::msg::TwistStamped;
using sensor_msgs::msg::JointState;
using std_msgs::msg::Empty;
using std_msgs::msg::Int8;

//using raptor_pdu_msgs::msg::RelayCommand;

using raptor_dbw_msgs::msg::AcceleratorPedalCmd;
using raptor_dbw_msgs::msg::AcceleratorPedalReport;
using raptor_dbw_msgs::msg::Brake2Report;
using raptor_dbw_msgs::msg::BrakeCmd;
using raptor_dbw_msgs::msg::SteeringCmd;
using raptor_dbw_msgs::msg::SteeringReport;
using raptor_dbw_msgs::msg::SteeringExtendedReport;
using raptor_dbw_msgs::msg::TirePressureReport;
// using raptor_dbw_msgs::msg::WatchdogStatus;
using raptor_dbw_msgs::msg::WheelPositionReport;
using raptor_dbw_msgs::msg::WheelSpeedReport;

//using deep_orange_msgs::msg::BaseToCarSummary;
using deep_orange_msgs::msg::MiscReport;
using deep_orange_msgs::msg::RcToCt;
using deep_orange_msgs::msg::CtReport;
using deep_orange_msgs::msg::TireReport;
using deep_orange_msgs::msg::PtReport;
using deep_orange_msgs::msg::DiagnosticReport;
using deep_orange_msgs::msg::LapTimeReport;
using deep_orange_msgs::msg::MylapsReport;

namespace raptor_dbw_can
{
/** \brief Class for converting Raptor DBW messages between CAN & ROS */
class RaptorDbwCAN : public rclcpp::Node
{
public:
/** \brief Default constructor.
 * \param[in] options The options for this node
 */
  explicit RaptorDbwCAN(const rclcpp::NodeOptions & options);
  ~RaptorDbwCAN();

private:
/** \brief timer callbacks for publishing tire report at 100Hz
 */
  void timerTireCallback();

/** \brief timer callbacks for publishing pit (engine) report at 100Hz
 */
  void timerPtCallback();

/** \brief timer callback for publishing mylaps report at 10Hz.
 */
  void timerMylapsCallback();

/** \brief Attempt to enable the DBW system.
 * \param[in] msg Enable message (must not be null)
 */
  void recvEnable(const Empty::UniquePtr msg);

/** \brief Attempt to disable the DBW system.
 * \param[in] msg Disable message (must not be null)
 */
  void recvDisable(const Empty::UniquePtr msg);

/** \brief Convert reports received over CAN into ROS messages.
 * \param[in] msg The message received over CAN.
 */
  void recvCAN(const Frame::UniquePtr msg);

/** \brief Convert an Accel Pedal Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvAccelPedalRpt(const Frame& msg);

/** \brief Convert a Brake2 Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvBrake2Rpt(const Frame& msg);

/** \brief Convert a Pit (Engine+Gear) Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN. The message is split into 3 parts.
 */
  void recvDOPtRptPart1(const Frame& msg);
  void recvDOPtRptPart2(const Frame& msg);
  void recvDOPtRptPart3(const Frame& msg);

/** \brief Convert a Diagnostic Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvDODiagRpt(const Frame& msg);

/** \brief Convert a Misc. Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvDOMiscRpt(const Frame& msg);

/** \brief Convert a Race Control to Team Computer Report received over CAN
 * into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvDORcToCtRpt(const Frame& msg);

/** \brief Convert a Lap Time Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvDOLapTimeRpt(const Frame& msg);

/** \brief Convert a Steering Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvSteeringRpt(const Frame& msg);

/** \brief Convert a Steering2 Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvSteeringExtdRpt(const Frame& msg);

/** \brief Convert a Tire Pressure Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 * \param[in] canid The CAN ID of the incoming message.
 * \param[in] which The prefix string indicating which wheel the message corresponds to.
 */
  void recvTirePressureRpt(const Frame& msg,
                          const unsigned int & canid,
                          const std::string & which);

/** \brief Convert a Tire Temperature Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 * \param[in] canid The CAN ID of the incoming message.
 * \param[in] which The prefix string indicating which wheel the message corresponds to.
 * \param[in] part The message is split into four (4) parts.
 * This value is in the range of 1..4.
 */
  void recvTireTempRpt(const Frame& msg,
                      const unsigned int & canid,
                      const std::string & which,
                      const uint8_t & part);

/** \brief Convert a Wheel Speed Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvWheelSpeedRpt(const Frame& msg);

/** \brief Convert a Wheel Strain Gauge Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvWheelStrainGaugeRpt(const Frame& msg);

/** \brief Convert a Wheel Potentialmeter Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvWheelPotentialmeterRpt(const Frame& msg);

/**
 * \brief Convert a MyLaps Report received over CAN into a ROS message.
 * \param[in] msg The message received over CAN.
 */
  void recvMyLapsRptPart1(const Frame& msg);
  void recvMyLapsRptPart2(const Frame& msg);

/** \brief Convert an Accelerator Pedal Command sent as a ROS message into a CAN message.
 * \param[in] msg The message to send over CAN.
 */
  void recvAcceleratorPedalCmd(const AcceleratorPedalCmd::UniquePtr msg);

/** \brief Convert a Brake Command sent as a ROS message into a CAN message.
 * \param[in] msg The message to send over CAN.
 */
  void recvBrakeCmd(const BrakeCmd::UniquePtr msg);

/** \brief Convert a Steering Command sent as a ROS message into a CAN message.
 * \param[in] msg The message to send over CAN.
 */
  void recvSteeringCmd(const SteeringCmd::UniquePtr msg);

  void recvGearShiftCmd(const Int8::UniquePtr msg);
  
  void recvCtCmd(const deep_orange_msgs::msg::CtReport::UniquePtr msg);

  rclcpp::Clock m_clock;
  static constexpr int64_t CLOCK_1_SEC = 1000;  // duration in milliseconds

  // Parameters from launch
  std::string dbw_dbc_file_;
  float max_steer_angle_;

/** \brief Checks faults & overrides to establish DBW control */
  void enableSystem();

/** \brief Disables DBW control */
  void disableSystem();

/** \brief Enumeration of vehicle joints */
  enum ListJoints
  {
    JOINT_FL = 0,   /**< Front left wheel */
    JOINT_FR,       /**< Front right wheel */
    JOINT_RL,       /**< Rear left wheel */
    JOINT_RR,       /**< Rear right wheel */
    JOINT_SL,       /**< Steering left */
    JOINT_SR,       /**< Steering right */
    JOINT_COUNT,    /**< Number of joints */
  };

  JointState joint_state_;

/** \brief Calculates & publishes joint states based on updated steering report.
 *    Overloaded function.
 * \param[in] stamp Updated time stamp
 * \param[in] steering Updated steering report
 */
  void publishJointStates(
    const rclcpp::Time stamp,
    const SteeringReport& steering);

/** \brief Calculates & publishes joint states based on updated wheel speed report.
 *    Overloaded function.
 * \param[in] stamp Updated time stamp
 * \param[in] wheels Updated wheel speed report
 */
  void publishJointStates(
    const rclcpp::Time stamp,
    const WheelSpeedReport& wheels);

  // Ackermann steering
  double acker_wheelbase_;
  double acker_track_;
  double steering_ratio_;

  rclcpp::TimerBase::SharedPtr timer_mylaps_report_;
  rclcpp::TimerBase::SharedPtr timer_tire_report_;
  rclcpp::TimerBase::SharedPtr timer_pt_report_;

  deep_orange_msgs::msg::PtReport pt_report_msg;
  deep_orange_msgs::msg::TireReport tire_report_msg;
  deep_orange_msgs::msg::MylapsReport mylaps_report_msg;
 
  // Subscribed topics
  rclcpp::Subscription<Frame>::SharedPtr sub_can_;
  rclcpp::Subscription<AcceleratorPedalCmd>::SharedPtr sub_accelerator_pedal_;
  rclcpp::Subscription<BrakeCmd>::SharedPtr sub_brake_;
  rclcpp::Subscription<Int8>::SharedPtr sub_gear_shift_cmd_;
  rclcpp::Subscription<SteeringCmd>::SharedPtr sub_steering_;
  rclcpp::Subscription<deep_orange_msgs::msg::CtReport>::SharedPtr sub_ct_report_;

  // Published topics -- Raptor types
  rclcpp::Publisher<Frame>::SharedPtr pub_can_;
  rclcpp::Publisher<AcceleratorPedalReport>::SharedPtr pub_accel_pedal_;
  rclcpp::Publisher<Brake2Report>::SharedPtr pub_brake_2_report_;
  rclcpp::Publisher<JointState>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steering_;
  rclcpp::Publisher<SteeringExtendedReport>::SharedPtr pub_steering_ext_;
  rclcpp::Publisher<TirePressureReport>::SharedPtr pub_tire_pressure_;
  rclcpp::Publisher<WheelSpeedReport>::SharedPtr pub_wheel_speeds_;

  // Published topics -- Deep Orange types
  //rclcpp::Publisher<BaseToCarSummary>::SharedPtr pub_flags_;
  rclcpp::Publisher<MiscReport>::SharedPtr pub_misc_do_;
  rclcpp::Publisher<RcToCt>::SharedPtr pub_rc_to_ct_;
  rclcpp::Publisher<TireReport>::SharedPtr pub_tire_report_;
  rclcpp::Publisher<PtReport>::SharedPtr pub_pt_report_;
  rclcpp::Publisher<MylapsReport>::SharedPtr pub_mylaps_report_;
  rclcpp::Publisher<DiagnosticReport>::SharedPtr pub_diag_report_;
  rclcpp::Publisher<LapTimeReport>::SharedPtr pub_timing_report_;

  NewEagle::Dbc dbwDbc_;

  uint32_t count_;
};

}  // namespace raptor_dbw_can

#endif  // RAPTOR_DBW_CAN__RAPTOR_DBW_CAN_HPP_
