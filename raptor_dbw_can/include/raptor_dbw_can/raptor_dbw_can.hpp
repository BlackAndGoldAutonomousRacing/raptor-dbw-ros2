// Copyright (c) 2020 New Eagle, All rights reserved.
// Copyright (c) 2022-2023 Haoguang Yang, All rights reserved.
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
 * The definition is modified to suit the configuration on the AV21 autonomous Indycar.
 * \author Haoguang Yang
 * \date 07/01/2023
 * \file raptor_dbw_can.hpp
 */

#ifndef RAPTOR_DBW_CAN__RAPTOR_DBW_CAN_HPP_
#define RAPTOR_DBW_CAN__RAPTOR_DBW_CAN_HPP_

// C++ STL
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

// ROS 2
#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <builtin_interfaces/msg/time.hpp>
#include <can_msgs/msg/frame.hpp>
#include <sensor_msgs/msg/imu.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int8.hpp>  // used for gear shift cmd.
// #include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <deep_orange_msgs/msg/base_to_car_summary.hpp>
#include <deep_orange_msgs/msg/brake_temp_report.hpp>
#include <deep_orange_msgs/msg/coordinates.hpp>
#include <deep_orange_msgs/msg/ct_report.hpp>
#include <deep_orange_msgs/msg/diagnostic_report.hpp>
#include <deep_orange_msgs/msg/lap_time_report.hpp>
#include <deep_orange_msgs/msg/misc_report.hpp>
#include <deep_orange_msgs/msg/pt_report.hpp>
#include <deep_orange_msgs/msg/race_control_pos_report.hpp>
#include <deep_orange_msgs/msg/rc_to_ct.hpp>
#include <deep_orange_msgs/msg/tire_report.hpp>

#include <raptor_dbw_msgs/msg/accelerator_pedal_cmd.hpp>
#include <raptor_dbw_msgs/msg/accelerator_pedal_report.hpp>
#include <raptor_dbw_msgs/msg/brake2_report.hpp>
#include <raptor_dbw_msgs/msg/brake_cmd.hpp>
#include <raptor_dbw_msgs/msg/steering_cmd.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>
#include <raptor_dbw_msgs/msg/steering_report.hpp>
#include <raptor_dbw_msgs/msg/tire_pressure_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_position_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>

// #include <raptor_pdu_msgs/msg/relay_command.hpp>          // not used.
// #include <raptor_pdu_msgs/msg/relay_state.hpp>            // not used.

#include <can_dbc_parser/Dbc.hpp>
#include <can_dbc_parser/DbcBuilder.hpp>
#include <can_dbc_parser/DbcMessage.hpp>
#include <can_dbc_parser/DbcSignal.hpp>

#include "raptor_dbw_can/canid_enum.hpp"

using namespace std::chrono_literals;  // NOLINT

using builtin_interfaces::msg::Time;
using can_msgs::msg::Frame;
// using sensor_msgs::msg::JointState;
using sensor_msgs::msg::Imu;
using std_msgs::msg::Empty;
using std_msgs::msg::Int8;
using std_msgs::msg::UInt8MultiArray;

using raptor_dbw_msgs::msg::AcceleratorPedalCmd;
using raptor_dbw_msgs::msg::AcceleratorPedalReport;
using raptor_dbw_msgs::msg::Brake2Report;
using raptor_dbw_msgs::msg::BrakeCmd;
using raptor_dbw_msgs::msg::SteeringCmd;
using raptor_dbw_msgs::msg::SteeringExtendedReport;
using raptor_dbw_msgs::msg::SteeringReport;
using raptor_dbw_msgs::msg::TirePressureReport;
// using raptor_dbw_msgs::msg::WatchdogStatus;
using raptor_dbw_msgs::msg::WheelPositionReport;
using raptor_dbw_msgs::msg::WheelSpeedReport;

// using raptor_pdu_msgs::msg::RelayCommand;

// using deep_orange_msgs::msg::BaseToCarSummary;
using deep_orange_msgs::msg::CtReport;
using deep_orange_msgs::msg::DiagnosticReport;
using deep_orange_msgs::msg::LapTimeReport;
using deep_orange_msgs::msg::MiscReport;
using deep_orange_msgs::msg::PtReport;
using deep_orange_msgs::msg::RaceControlPosReport;
using deep_orange_msgs::msg::RcToCt;
using deep_orange_msgs::msg::TireReport;

namespace raptor_dbw_can {

/** \brief Class for converting Raptor DBW messages between CAN & ROS */
class RaptorDbwCAN : public rclcpp::Node {
 public:
  /** \brief Default constructor.
   * \param[in] options The options for this node
   */
  explicit RaptorDbwCAN(const rclcpp::NodeOptions& options);
  ~RaptorDbwCAN();

 private:
  struct DbcFrameRx {
    NewEagle::DbcMessage* message = NULL;

    DbcFrameRx(const Frame& msg, const MessageID& msgType, NewEagle::Dbc& dbc,
               Time& frame_timestamp) {
      message = dbc.GetMessageById(static_cast<uint32_t>(msgType));
      if (msg.dlc < message->GetDlc()) return;
      frame_timestamp = msg.header.stamp;
      message->SetFrame(msg);
    }

    bool valid() const { return message != NULL; }

    template <typename T>
    DbcFrameRx& operator()(const std::string& dbcField, T& msgField) {
      msgField = static_cast<T>(message->GetSignal(dbcField)->GetResult());
      return *this;
    }
  };

  struct DbcFrameTx {
    NewEagle::DbcMessage* message = NULL;

    DbcFrameTx(const MessageID& msgType, NewEagle::Dbc& dbc) {
      message = dbc.GetMessageById(static_cast<uint32_t>(msgType));
    }

    bool valid() const { return message != NULL; }

    template <typename T>
    DbcFrameTx& operator()(const std::string& dbcField, const T& msgField) {
      message->GetSignal(dbcField)->SetResult(static_cast<double>(msgField));
      return *this;
    }
  };

  /** \brief timer callbacks for publishing tire report at 100Hz
   */
  void timerTireCallback();

  /** \brief timer callbacks for publishing pit (engine) report at 100Hz
   */
  void timerPtCallback();

  /** \brief timer callback for publishing mylaps report at 5Hz.
   */
  void timerMylapsCallback();

  /** \brief Attempt to enable the DBW system.
   * \param[in] msg Enable message (must not be null)
   */
  // void recvEnable(Empty::UniquePtr msg);

  /** \brief Attempt to disable the DBW system.
   * \param[in] msg Disable message (must not be null)
   */
  // void recvDisable(Empty::UniquePtr msg);

  /** \brief Convert reports received over CAN into ROS messages.
   * \param[in] msg The message received over CAN.
   */
  void recvCAN(Frame::UniquePtr msg);

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
  void recvTirePressureRpt(const Frame& msg, const uint32_t& canid, const std::string_view& which,
                           float& pressure, float& gauge);

  /** \brief Convert a Tire Temperature Report received over CAN into a ROS message.
   * \param[in] msg The message received over CAN.
   * \param[in] canid The CAN ID of the incoming message.
   * \param[in] which The prefix string indicating which wheel the message corresponds to.
   * \param[in] part The message is split into four (4) parts.
   * This value is in the range of 1..4.
   */
  void recvTireTempRpt(const Frame& msg, const uint32_t& canid, const std::string_view& which,
                       const uint8_t& part, float* temp);

  /** \brief computes values of TireTemp message for a given array of tire temperatures.
   */
  void postprocessTireTemp();

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

  /**
   * @brief Convert Marelli Report received over CAN into a ROS message.
   *
   */
  void recvMarelliRptPart1(const Frame& msg);
  void recvMarelliRptPart2(const Frame& msg);

  /** \brief Convert an Accelerator Pedal Command sent as a ROS message into a CAN message.
   * \param[in] msg The message to send over CAN.
   */
  void recvAcceleratorPedalCmd(AcceleratorPedalCmd::UniquePtr msg);

  /** \brief Convert a Brake Command sent as a ROS message into a CAN message.
   * \param[in] msg The message to send over CAN.
   */
  void recvBrakeCmd(BrakeCmd::UniquePtr msg);

  /** \brief Convert a Steering Command sent as a ROS message into a CAN message.
   * \param[in] msg The message to send over CAN.
   */
  void recvSteeringCmd(SteeringCmd::UniquePtr msg);

  void recvGearShiftCmd(Int8::UniquePtr msg);

  void recvCtCmd(deep_orange_msgs::msg::CtReport::UniquePtr msg);

  void recvImuCmd(sensor_msgs::msg::Imu::UniquePtr msg);

  void recvDashSwitchesCmd(std_msgs::msg::UInt8MultiArray::UniquePtr msg);

  /** \brief Checks faults & overrides to establish DBW control */
  // void enableSystem();

  /** \brief Disables DBW control */
  // void disableSystem();

  /** \brief Enumeration of vehicle joints */
  // enum ListJoints
  // {
  //   JOINT_FL = 0,   /**< Front left wheel */
  //   JOINT_FR,       /**< Front right wheel */
  //   JOINT_RL,       /**< Rear left wheel */
  //   JOINT_RR,       /**< Rear right wheel */
  //   JOINT_SL,       /**< Steering left */
  //   JOINT_SR,       /**< Steering right */
  //   JOINT_COUNT,    /**< Number of joints */
  // };

  // JointState joint_state_;

  /** \brief Calculates & publishes joint states based on updated steering report.
   *    Overloaded function.
   * \param[in] stamp Updated time stamp
   * \param[in] steering Updated steering report
   */
  // void publishJointStates(const rclcpp::Time stamp, const SteeringReport& steering);

  /** \brief Calculates & publishes joint states based on updated wheel speed report.
   *    Overloaded function.
   * \param[in] stamp Updated time stamp
   * \param[in] wheels Updated wheel speed report
   */
  // void publishJointStates(const rclcpp::Time stamp, const WheelSpeedReport& wheels);

  // Ackermann steering
  // double acker_wheelbase_;
  // double acker_track_;
  // double steering_ratio_;

  enum RaceControlSystems {
    Spoofed = 0,
    MyLaps,
    Marelli,
    RaceControlSystemsCount
  };

  RaceControlSystems current_rc_type_;

  // Parameters from launch
  std::string dbw_dbc_file_;

  NewEagle::Dbc dbwDbc_;

  // IMU feedback -- a RAW IMU is preferred.
  static constexpr double GRAVITY = 9.80665;
  //! Driver Dash Switches States
  static constexpr uint8_t TRACTION_AIM_DEFAULT = 3;
  static constexpr uint8_t TRACTION_RANGE_DEFAULT = 3;
  uint8_t last_driver_traction_range_switch_ = TRACTION_RANGE_DEFAULT;
  uint8_t last_traction_aim_ = TRACTION_AIM_DEFAULT;

  // arrays for raw records of tire temperature.
  static constexpr size_t TIRE_TEMP_SENSORS_PER_WHEEL = 16;
  float tire_temp_raw_array_fl_[TIRE_TEMP_SENSORS_PER_WHEEL] = {-1.};
  float tire_temp_raw_array_fr_[TIRE_TEMP_SENSORS_PER_WHEEL] = {-1.};
  float tire_temp_raw_array_rl_[TIRE_TEMP_SENSORS_PER_WHEEL] = {-1.};
  float tire_temp_raw_array_rr_[TIRE_TEMP_SENSORS_PER_WHEEL] = {-1.};

  float last_acc_pedal_cmd_ = 0.;
  float last_steering_cmd_ = 0.;

  // Class variables for shared message storage
  PtReport pt_report_msg;
  RaceControlPosReport rc_pos_report_msg_;
  RcToCt race_control_cmd_;
  TireReport tire_report_msg;

  // ROS 2 components
  rclcpp::TimerBase::SharedPtr timer_rc_pos_report_;
  rclcpp::TimerBase::SharedPtr timer_tire_report_;
  rclcpp::TimerBase::SharedPtr timer_pt_report_;

  // Subscribed topics
  rclcpp::Subscription<AcceleratorPedalCmd>::SharedPtr sub_accelerator_pedal_;
  rclcpp::Subscription<BrakeCmd>::SharedPtr sub_brake_;
  rclcpp::Subscription<CtReport>::SharedPtr sub_ct_report_;
  rclcpp::Subscription<Frame>::SharedPtr sub_can_;
  rclcpp::Subscription<Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<Int8>::SharedPtr sub_gear_shift_cmd_;
  rclcpp::Subscription<SteeringCmd>::SharedPtr sub_steering_;
  rclcpp::Subscription<UInt8MultiArray>::SharedPtr sub_dash_cmds_;

  // Published topics -- Raptor types
  rclcpp::Publisher<AcceleratorPedalReport>::SharedPtr pub_accel_pedal_;
  rclcpp::Publisher<Brake2Report>::SharedPtr pub_brake_2_report_;
  rclcpp::Publisher<Frame>::SharedPtr pub_can_;
  // rclcpp::Publisher<JointState>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steering_;
  rclcpp::Publisher<SteeringExtendedReport>::SharedPtr pub_steering_ext_;
  rclcpp::Publisher<TirePressureReport>::SharedPtr pub_tire_pressure_;
  rclcpp::Publisher<WheelSpeedReport>::SharedPtr pub_wheel_speeds_;

  // Published topics -- Deep Orange types
  // rclcpp::Publisher<BaseToCarSummary>::SharedPtr pub_flags_;
  rclcpp::Publisher<DiagnosticReport>::SharedPtr pub_diag_report_;
  rclcpp::Publisher<LapTimeReport>::SharedPtr pub_timing_report_;
  rclcpp::Publisher<MiscReport>::SharedPtr pub_misc_do_;
  rclcpp::Publisher<PtReport>::SharedPtr pub_pt_report_;
  rclcpp::Publisher<RaceControlPosReport>::SharedPtr pub_rc_pos_report_;
  rclcpp::Publisher<RcToCt>::SharedPtr pub_rc_to_ct_;
  rclcpp::Publisher<TireReport>::SharedPtr pub_tire_report_;
};

}  // namespace raptor_dbw_can

#endif  // RAPTOR_DBW_CAN__RAPTOR_DBW_CAN_HPP_
