// Copyright (c) 2015-2018, Dataspeed Inc., 2018-2020 New Eagle, All rights reserved.
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

#include "raptor_dbw_can/raptor_dbw_can.hpp"
#include <iostream>

#include <algorithm>
#include <cmath>
#include <string>

namespace raptor_dbw_can
{

RaptorDbwCAN::RaptorDbwCAN(const rclcpp::NodeOptions & options)
: Node("raptor_dbw_can_node", options)
{
  dbw_dbc_file_ = this->declare_parameter<std::string>("dbw_dbc_file");

  // Initializing tire report: set sizes for the arrays
  for (int i = 0; i < 16; i++) 
  {
    tire_report_msg.fl_tire_temperature.push_back(-1.0);
    tire_report_msg.fr_tire_temperature.push_back(-1.0);
    tire_report_msg.rl_tire_temperature.push_back(-1.0);
    tire_report_msg.rr_tire_temperature.push_back(-1.0);
  }

  // Initialize joint state reports and ckermann steering parameters
  acker_wheelbase_ = this->declare_parameter<double>
                        ("ackermann_wheelbase", 2.8498);  // 112.2 inches
  acker_track_ = this->declare_parameter<double>
                        ("ackermann_track", 1.5824);      // 62.3 inches
  steering_ratio_ = this->declare_parameter<double>
                        ("steering_ratio", 14.8);
  // on the steering wheel, in degrees
  max_steer_angle_ = this->declare_parameter<double>
                        ("max_steer_angle", 450.0);

  // Initialize joint states
  joint_state_.position.resize(JOINT_COUNT);
  joint_state_.velocity.resize(JOINT_COUNT);
  joint_state_.effort.resize(JOINT_COUNT);
  joint_state_.name.resize(JOINT_COUNT);
  joint_state_.name[JOINT_FL] = "wheel_fl";   // Front Left
  joint_state_.name[JOINT_FR] = "wheel_fr";   // Front Right
  joint_state_.name[JOINT_RL] = "wheel_rl";   // Rear Left
  joint_state_.name[JOINT_RR] = "wheel_rr";   // Rear Right
  joint_state_.name[JOINT_SL] = "steer_fl";
  joint_state_.name[JOINT_SR] = "steer_fr";

  // Set up Publishers
  pub_can_ = this->create_publisher<Frame>("can_tx", 20);
  pub_accel_pedal_ = this->create_publisher<AcceleratorPedalReport>(
    "accelerator_pedal_report", 20);
  pub_steering_ =
    this->create_publisher<SteeringReport>("steering_report", 20);
  pub_steering_ext_ = this->create_publisher<SteeringExtendedReport>(
    "steering_extended_report", 20);
  pub_wheel_speeds_ = this->create_publisher<WheelSpeedReport>(
    "wheel_speed_report", 20);
  pub_tire_pressure_ = this->create_publisher<TirePressureReport>(
    "tire_pressure_report", 20);
  pub_brake_2_report_ = this->create_publisher<Brake2Report>(
    "brake_2_report", 20);
  pub_joint_states_ = this->create_publisher<JointState>("joint_states", 10);

  //pub_flags_ = this->create_publisher<BaseToCarSummary>("flag_report", 20);
  pub_misc_do_ = this->create_publisher<MiscReport>("misc_report_do", 10);
  pub_rc_to_ct_ = this->create_publisher<RcToCt>("rc_to_ct", 10);
  pub_tire_report_ = this->create_publisher<TireReport>("tire_report", 10);
  pub_pt_report_ = this->create_publisher<PtReport>("pt_report", 10);
  pub_diag_report_ = this->create_publisher<DiagnosticReport>(
    "diag_report", 10);
  pub_timing_report_ = this->create_publisher<LapTimeReport>(
    "lap_time_report", 10);
  pub_mylaps_report_ = this->create_publisher<MylapsReport>(
    "mylaps_report", 10);

  // Set up Subscribers
  sub_can_ = this->create_subscription<Frame>(
    "can_rx", 500, std::bind(&RaptorDbwCAN::recvCAN, this, std::placeholders::_1));

  sub_brake_ = this->create_subscription<BrakeCmd>(
    "brake_cmd", 1,
    std::bind(&RaptorDbwCAN::recvBrakeCmd, this, std::placeholders::_1));

  sub_accelerator_pedal_ = this->create_subscription<AcceleratorPedalCmd>(
    "accelerator_pedal_cmd", 1,
    std::bind(&RaptorDbwCAN::recvAcceleratorPedalCmd, this, std::placeholders::_1));

  sub_steering_ = this->create_subscription<SteeringCmd>(
    "steering_cmd", 1,
    std::bind(&RaptorDbwCAN::recvSteeringCmd, this, std::placeholders::_1));

  sub_gear_shift_cmd_ = this->create_subscription<UInt8>(
    "gear_cmd", 10,
    std::bind(&RaptorDbwCAN::recvGearShiftCmd, this, std::placeholders::_1));

  sub_ct_report_ = this->create_subscription<CtReport>(
    "ct_report", 1,
    std::bind(&RaptorDbwCAN::recvCtCmd, this, std::placeholders::_1));

  dbwDbc_ = NewEagle::DbcBuilder().NewDbc(dbw_dbc_file_);

  // Set up Timers
  timer_tire_report_ = this->create_wall_timer(10ms,
    std::bind(&RaptorDbwCAN::timerTireCallback, this));

  timer_pt_report_ = this->create_wall_timer(10ms,
    std::bind(&RaptorDbwCAN::timerPtCallback, this));

  timer_mylaps_report_ = this->create_wall_timer(100ms,
    std::bind(&RaptorDbwCAN::timerMylapsCallback, this));
}

RaptorDbwCAN::~RaptorDbwCAN()
{
}

void RaptorDbwCAN::recvCAN(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (msg->is_rtr || msg->is_error)
    return;
  
  const std::string fl = "FL";
  const std::string fr = "FR";
  const std::string rl = "RL";
  const std::string rr = "RR";
  const uint8_t part1 = 1;
  const uint8_t part2 = 2;
  const uint8_t part3 = 3;
  const uint8_t part4 = 4;

  switch (static_cast<MessageID>(msg->id)) {
    case MessageID::BASE_TO_CAR_SUMMARY:
      recvDORcToCtRpt(msg);
      break;

    case MessageID::BASE_TO_CAR_TIMING:
      recvDOLapTimeRpt(msg);
      break;

    case MessageID::WHEEL_SPEED_REPORT:
      recvWheelSpeedRpt(msg);
      break;

    case MessageID::BRAKE_PRESSURE_REPORT:
      recvBrake2Rpt(msg);
      break;

    case MessageID::ACCELERATOR_REPORT:
      recvAccelPedalRpt(msg);
      break;

    case MessageID::STEERING_REPORT:
      recvSteeringRpt(msg);
      break;

    case MessageID::MISC_REPORT:
      recvDOMiscRpt(msg);
      break;

    case MessageID::WHEEL_STRAIN_GAUGE:
      recvWheelStrainGaugeRpt(msg);
      break; 

    case MessageID::WHEEL_POTENTIOMETER_DATA:
      recvWheelPotentialmeterRpt(msg);
      break;

    case MessageID::STEERING_REPORT_EXTD:
      recvSteeringExtdRpt(msg);
      break;

    case MessageID::TIRE_PRESSURE_FL:
      recvTirePressureRpt(msg, static_cast<unsigned int>(msg->id), fl);
      break;

    case MessageID::TIRE_PRESSURE_FR:
      recvTirePressureRpt(msg, static_cast<unsigned int>(msg->id), fr);
      break;

    case MessageID::TIRE_PRESSURE_RL:
      recvTirePressureRpt(msg, static_cast<unsigned int>(msg->id), rl);
      break;
    
    case MessageID::TIRE_PRESSURE_RR:
      recvTirePressureRpt(msg, static_cast<unsigned int>(msg->id), rr);
      break;

    case MessageID::TIRE_TEMP_FL_1:
      recvTireTempRpt(msg, static_cast<unsigned int>(msg->id), fl, part1);
      break;

    case MessageID::TIRE_TEMP_FL_2:
      recvTireTempRpt(msg, static_cast<unsigned int>(msg->id), fl, part2);
      break;

    case MessageID::TIRE_TEMP_FL_3:
      recvTireTempRpt(msg, static_cast<unsigned int>(msg->id), fl, part3);
      break;

    case MessageID::TIRE_TEMP_FL_4:
      recvTireTempRpt(msg, static_cast<unsigned int>(msg->id), fl, part4);
      break;

    case MessageID::TIRE_TEMP_FR_1:
      recvTireTempRpt(msg, static_cast<unsigned int>(msg->id), fr, part1);
      break;

    case MessageID::TIRE_TEMP_FR_2:
      recvTireTempRpt(msg, static_cast<unsigned int>(msg->id), fr, part2);
      break;

    case MessageID::TIRE_TEMP_FR_3:
      recvTireTempRpt(msg, static_cast<unsigned int>(msg->id), fr, part3);
      break;

    case MessageID::TIRE_TEMP_FR_4:
      recvTireTempRpt(msg, static_cast<unsigned int>(msg->id), fr, part4);
      break;

    case MessageID::TIRE_TEMP_RL_1:
      recvTireTempRpt(msg, static_cast<unsigned int>(msg->id), rl, part1);
      break;

    case MessageID::TIRE_TEMP_RL_2:
      recvTireTempRpt(msg, static_cast<unsigned int>(msg->id), rl, part2);
      break;

    case MessageID::TIRE_TEMP_RL_3:
      recvTireTempRpt(msg, static_cast<unsigned int>(msg->id), rl, part3);
      break;

    case MessageID::TIRE_TEMP_RL_4:
      recvTireTempRpt(msg, static_cast<unsigned int>(msg->id), rl, part4);
      break;

    case MessageID::TIRE_TEMP_RR_1:
      recvTireTempRpt(msg, static_cast<unsigned int>(msg->id), rr, part1);
      break;

    case MessageID::TIRE_TEMP_RR_2:
      recvTireTempRpt(msg, static_cast<unsigned int>(msg->id), rr, part2);
      break;

    case MessageID::TIRE_TEMP_RR_3:
      recvTireTempRpt(msg, static_cast<unsigned int>(msg->id), rr, part3);
      break;

    case MessageID::TIRE_TEMP_RR_4:
      recvTireTempRpt(msg, static_cast<unsigned int>(msg->id), rr, part4);
      break;

    case MessageID::PT_REPORT_1:
      recvDOPtRptPart1(msg);
      break;

    case MessageID::PT_REPORT_2:
      recvDOPtRptPart2(msg);
      break;

    case MessageID::DIAGNOSTIC_REPORT:
      recvDODiagRpt(msg);
      break;

    case MessageID::PT_REPORT_3:
      recvDOPtRptPart3(msg);
      break;

    default:
      break;
  }
}

void RaptorDbwCAN::recvAccelPedalRpt(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(
    static_cast<uint32_t>(MessageID::ACCELERATOR_REPORT));
  if (msg->dlc < message->GetDlc())
    return;

  message->SetFrame(msg);

  AcceleratorPedalReport out;
  out.header.stamp = msg->header.stamp;
  out.pedal_output  = message->GetSignal("acc_pedal_fdbk")->GetResult();
  out.rolling_counter = message->GetSignal("acc_pedal_fdbk_counter")->GetResult(); 
  pub_accel_pedal_->publish(out);
}

void RaptorDbwCAN::recvSteeringRpt(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(
    static_cast<uint32_t>(MessageID::STEERING_REPORT));
  if (msg->dlc < message->GetDlc())
    return;

  message->SetFrame(msg);
  
  SteeringReport out;
  out.header.stamp = msg->header.stamp;
  out.steering_wheel_angle  = message->GetSignal("steering_motor_ang_avg_fdbk")->GetResult();
  out.rolling_counter = message->GetSignal("steering_motor_fdbk_counter")->GetResult(); 
  pub_steering_->publish(out);
  publishJointStates(msg->header.stamp, out);
}

void RaptorDbwCAN::recvSteeringExtdRpt(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(
    static_cast<uint32_t>(MessageID::STEERING_REPORT_EXTD));
  if (msg->dlc < message->GetDlc())
    return;

  message->SetFrame(msg);
  SteeringExtendedReport out;
  out.header.stamp = msg->header.stamp;
  out.steering_motor_ang_1  = message->GetSignal("steering_motor_ang_1_fdbk")->GetResult();
  out.steering_motor_ang_2  = message->GetSignal("steering_motor_ang_2_fdbk")->GetResult();
  out.steering_motor_ang_3  = message->GetSignal("steering_motor_ang_3_fdbk")->GetResult();
  pub_steering_ext_->publish(out);
}

void RaptorDbwCAN::recvDOLapTimeRpt(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(
    static_cast<uint32_t>(MessageID::BASE_TO_CAR_TIMING));
  if (msg->dlc < message->GetDlc())
    return;
  message->SetFrame(msg);

  LapTimeReport out;
  out.stamp = msg->header.stamp;
  out.laps = message->GetSignal("laps")->GetResult();
  out.lap_time = message->GetSignal("lap_time")->GetResult();
  out.time_stamp = message->GetSignal("time_stamp")->GetResult(); 
  pub_timing_report_->publish(out);
}

void RaptorDbwCAN::recvDORcToCtRpt(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(
    static_cast<uint32_t>(MessageID::BASE_TO_CAR_SUMMARY));
  if (msg->dlc < message->GetDlc())
    return;
  message->SetFrame(msg);

  RcToCt out;
  out.stamp = msg->header.stamp;
  out.rolling_counter = message->GetSignal("base_to_car_heartbeat")->GetResult();
  out.track_flag = message->GetSignal("track_flag")->GetResult();
  out.veh_flag = message->GetSignal("veh_flag")->GetResult(); 
  out.veh_rank = message->GetSignal("veh_rank")->GetResult();
  out.lap_count = message->GetSignal("lap_count")->GetResult();
  out.lap_distance = message->GetSignal("lap_distance")->GetResult();
  pub_rc_to_ct_->publish(out);
}

void RaptorDbwCAN::recvWheelSpeedRpt(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(
    static_cast<uint32_t>(MessageID::WHEEL_SPEED_REPORT));

  if (msg->dlc < message->GetDlc())
    return;

  WheelSpeedReport out;
  out.header.stamp = msg->header.stamp;
  out.front_left  = message->GetSignal("wheel_speed_FL")->GetResult();
  out.front_right = message->GetSignal("wheel_speed_FR")->GetResult();
  out.rear_left = message->GetSignal("wheel_speed_RL")->GetResult();
  out.rear_right = message->GetSignal("wheel_speed_RR")->GetResult();

  pub_wheel_speeds_->publish(out);
  publishJointStates(msg->header.stamp, out);
}

void RaptorDbwCAN::recvTirePressureRpt(const Frame::SharedPtr msg,
                                  const unsigned int & canid,
                                  const std::string & which)
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(canid);
  if (msg->dlc < message->GetDlc())
    return;
  message->SetFrame(msg);
  // fill in which tire: FL, FR, ...
  tire_report_msg.fl_tire_pressure = message->GetSignal(which+"_Tire_Pressure")->GetResult();
  tire_report_msg.fl_tire_pressure_gauge =
    message->GetSignal(which+"_Tire_Pressure_Gauge")->GetResult();
}

void RaptorDbwCAN::recvTireTempRpt(const Frame::SharedPtr msg,
                              const unsigned int & canid,
                              const std::string & which,
                              const uint8_t & part)
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(canid);
  if (msg->dlc < message->GetDlc())
    return;
  message->SetFrame(msg);
  char buf[3];
  // fill in the fields: part1: 0..3, etc., corresponds to dbc signals _01, _02, ..._16.
  // fill in which tire: FL, FR, ...
  for (int index = (part-1)*4; index < part*4; index ++){
    snprintf(buf, 3, "%02d", index+1);
    tire_report_msg.fl_tire_temperature[index] =
        message->GetSignal(which + "_Tire_Temp_" + std::string(buf))->GetResult();
  }
}

void RaptorDbwCAN::recvBrake2Rpt(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(
    static_cast<uint32_t>(MessageID::BRAKE_PRESSURE_REPORT));

  if (msg->dlc < message->GetDlc())
    return;

  Brake2Report out;
  out.header.stamp = msg->header.stamp;
  out.front_brake_pressure = message->GetSignal(
    "brake_pressure_fdbk_front")->GetResult();
  out.rear_brake_pressure  = message->GetSignal(
    "brake_pressure_fdbk_rear")->GetResult();
  out.rolling_counter = message->GetSignal(
    "brk_pressure_fdbk_counter")->GetResult();

  pub_brake_2_report_->publish(out);
}

void RaptorDbwCAN::recvDOMiscRpt(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(
    static_cast<uint32_t>(MessageID::MISC_REPORT));
  if (msg->dlc < message->GetDlc())
    return;

  message->SetFrame(msg);

  MiscReport out;
  out.stamp = msg->header.stamp;
  out.sys_state = message->GetSignal("sys_state")->GetResult(); 
  out.safety_switch_state = message->GetSignal("safety_switch_state")->GetResult(); 
  out.mode_switch_state = message->GetSignal("mode_switch_state")->GetResult();
  out.battery_voltage = message->GetSignal("battery_voltage")->GetResult();
  pub_misc_do_->publish(out);
}

void RaptorDbwCAN::recvDODiagRpt(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(
    static_cast<uint32_t>(MessageID::DIAGNOSTIC_REPORT));
  if (msg->dlc < message->GetDlc())
    return;

  message->SetFrame(msg);

  DiagnosticReport out;
  out.stamp = msg->header.stamp;
  out.sd_system_warning = message->GetSignal("sd_system_warning")->GetResult();
  out.sd_system_failure = message->GetSignal("sd_system_failure")->GetResult();
  out.motec_warning = message->GetSignal("motec_warning")->GetResult();
  out.sd_brake_warning1 = message->GetSignal("sd_brake_warning1")->GetResult();
  out.sd_brake_warning2 = message->GetSignal("sd_brake_warning2")->GetResult();
  out.sd_brake_warning3 = message->GetSignal("sd_brake_warning3")->GetResult();
  out.sd_steer_warning1 = message->GetSignal("sd_steer_warning1")->GetResult();
  out.sd_steer_warning2 = message->GetSignal("sd_steer_warning2")->GetResult();
  out.sd_steer_warning3 = message->GetSignal("sd_steer_warning3")->GetResult();
  out.est1_oos_front_brk = message->GetSignal("est1_oos_front_brk")->GetResult();
  out.est2_oos_rear_brk = message->GetSignal("est2_oos_rear_brk")->GetResult();
  out.est3_low_eng_speed = message->GetSignal("est3_low_eng_speed")->GetResult();
  out.est4_sd_comms_loss = message->GetSignal("est4_sd_comms_loss")->GetResult();
  out.est5_motec_comms_loss = message->GetSignal("est5_motec_comms_loss")->GetResult();
  out.est6_sd_ebrake = message->GetSignal("est6_sd_ebrake")->GetResult();
  out.adlink_hb_lost = message->GetSignal("adlink_hb_lost")->GetResult();
  out.rc_lost = message->GetSignal("rc_lost")->GetResult();

  pub_diag_report_->publish(out);
}

void RaptorDbwCAN::recvDOPtRptPart1(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(
    static_cast<uint32_t>(MessageID::PT_REPORT_1));
  if (msg->dlc < message->GetDlc())
    return;
  message->SetFrame(msg);
  pt_report_msg.stamp = msg->header.stamp;
  pt_report_msg.throttle_position = message->GetSignal("throttle_position")->GetResult();
  pt_report_msg.engine_run_switch_status = message->GetSignal("engine_run_switch")->GetResult();
  pt_report_msg.current_gear = message->GetSignal("current_gear")->GetResult();
  pt_report_msg.engine_rpm = message->GetSignal("engine_speed_rpm")->GetResult();
  pt_report_msg.vehicle_speed_kmph = message->GetSignal("vehicle_speed_kmph")->GetResult();
}

void RaptorDbwCAN::recvDOPtRptPart2(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(
    static_cast<uint32_t>(MessageID::PT_REPORT_2));
  if (msg->dlc < message->GetDlc())
    return;
  message->SetFrame(msg);
  pt_report_msg.fuel_pressure = message->GetSignal("fuel_pressure_kPa")->GetResult();
  pt_report_msg.engine_oil_pressure = message->GetSignal("engine_oil_pressure_kPa")->GetResult();
  pt_report_msg.engine_coolant_temperature = message->GetSignal("coolant_temperature")->GetResult();
  pt_report_msg.transmission_oil_temperature = message->GetSignal("transmission_temperature")->GetResult();
  pt_report_msg.transmission_oil_pressure = message->GetSignal("transmission_pressure_kPa")->GetResult();
}

void RaptorDbwCAN::recvDOPtRptPart3(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(
    static_cast<uint32_t>(MessageID::PT_REPORT_3));
  if (msg->dlc < message->GetDlc())
    return;
  message->SetFrame(msg);
  pt_report_msg.engine_oil_temperature = message->GetSignal("engine_oil_temperature")->GetResult();
  pt_report_msg.wheel_torque_total = message->GetSignal("torque_wheels")->GetResult();
}

void RaptorDbwCAN::recvWheelStrainGaugeRpt(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(
    static_cast<uint32_t>(MessageID::WHEEL_STRAIN_GAUGE));
  if (msg->dlc < message->GetDlc())
    return;
  message->SetFrame(msg); 
  tire_report_msg.fl_wheel_load = message->GetSignal("wheel_strain_gauge_FL")->GetResult();
  tire_report_msg.fr_wheel_load = message->GetSignal("wheel_strain_gauge_FR")->GetResult();
  tire_report_msg.rl_wheel_load = message->GetSignal("wheel_strain_gauge_RL")->GetResult();
  tire_report_msg.rr_wheel_load = message->GetSignal("wheel_strain_gauge_RR")->GetResult();
}

void RaptorDbwCAN::recvWheelPotentialmeterRpt(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(
    static_cast<uint32_t>(MessageID::WHEEL_POTENTIOMETER_DATA));
  if (msg->dlc < message->GetDlc())
    return;
  message->SetFrame(msg); 
  tire_report_msg.stamp = msg->header.stamp;
  tire_report_msg.fl_damper_linear_potentiometer = message->GetSignal("wheel_potentiometer_FL")->GetResult();
  tire_report_msg.fr_damper_linear_potentiometer = message->GetSignal("wheel_potentiometer_FR")->GetResult();
  tire_report_msg.rl_damper_linear_potentiometer = message->GetSignal("wheel_potentiometer_RL")->GetResult();
  tire_report_msg.rr_damper_linear_potentiometer = message->GetSignal("wheel_potentiometer_RR")->GetResult();
}

void RaptorDbwCAN::recvMyLapsRptPart1(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(
    static_cast<uint32_t>(MessageID::MYLAPS_REPORT_1));
  if (msg->dlc < message->GetDlc())
    return;
  message->SetFrame(msg);
  mylaps_report_msg.stamp = msg->header.stamp;
  mylaps_report_msg.speed = message->GetSignal("mylaps_speed")->GetResult();
  mylaps_report_msg.heading = message->GetSignal("mylaps_heading")->GetResult();
}

void RaptorDbwCAN::recvMyLapsRptPart2(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(
    static_cast<uint32_t>(MessageID::MYLAPS_REPORT_2));
  if (msg->dlc < message->GetDlc())
    return;
  message->SetFrame(msg);
  mylaps_report_msg.latitude = message->GetSignal("Latitude")->GetResult();
  mylaps_report_msg.longitude = message->GetSignal("Longitude")->GetResult();
}

void RaptorDbwCAN::recvEnable(const Empty::SharedPtr msg)
{
  if (msg == NULL)
    return;
  enableSystem();
}

void RaptorDbwCAN::recvDisable(const Empty::SharedPtr msg)
{
  if (msg == NULL)
    return;
  disableSystem();
}

void RaptorDbwCAN::recvBrakeCmd(const BrakeCmd::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessage("brake_pressure_cmd");

  message->GetSignal("brake_pressure_cmd")->SetResult(msg->pedal_cmd); 
  
  NewEagle::DbcSignal * cnt = message->GetSignal("brk_pressure_cmd_counter");
  cnt->SetResult(msg->rolling_counter);
  
  Frame frame = message->GetFrame();
  
  pub_can_->publish(frame);
}

void RaptorDbwCAN::recvAcceleratorPedalCmd(
  const AcceleratorPedalCmd::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessage("accelerator_cmd");
  
  message->GetSignal("acc_pedal_cmd")->SetResult(msg->pedal_cmd);
  
  NewEagle::DbcSignal * cnt = message->GetSignal("acc_pedal_cmd_counter");
  cnt->SetResult(msg->rolling_counter);

  Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

void RaptorDbwCAN::recvSteeringCmd(const SteeringCmd::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbwDbc_.GetMessage("steering_cmd");

  double scmd =
    std::max(
      -1.0F * max_steer_angle_,
      std::min(
        max_steer_angle_ * 1.0F, static_cast<float>(
          msg->angle_cmd * 1.0F)));

  message->GetSignal("steering_motor_ang_cmd")->SetResult(scmd);

  message->GetSignal("steering_motor_cmd_counter")->SetResult(msg->rolling_counter);
  
  Frame frame = message->GetFrame();
  
  pub_can_->publish(frame);
}

void RaptorDbwCAN::recvGearShiftCmd(const std_msgs::msg::UInt8::SharedPtr msg) 
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessage("gear_shift_cmd");
  message->GetSignal("desired_gear")->SetResult(msg->data);
  can_msgs::msg::Frame frame = message->GetFrame();
  pub_can_->publish(frame);
}

void RaptorDbwCAN::recvCtCmd(const deep_orange_msgs::msg::CtReport::SharedPtr msg) 
{

  NewEagle::DbcMessage* message = dbwDbc_.GetMessage("ct_report");
  message->GetSignal("track_cond_ack")->SetResult(msg->track_cond_ack); 
  message->GetSignal("veh_sig_ack")->SetResult(msg->veh_sig_ack);
  message->GetSignal("ct_state")->SetResult(msg->ct_state);
  message->GetSignal("ct_state_rolling_counter")->SetResult(msg->rolling_counter);
  message->GetSignal("veh_num")->SetResult(msg->veh_num);
  can_msgs::msg::Frame frame = message->GetFrame();

  pub_can_->publish(frame);
}

void RaptorDbwCAN::timerTireCallback() {
    pub_tire_report_->publish(tire_report_msg);
}

void RaptorDbwCAN::timerPtCallback() {
    pub_pt_report_->publish(pt_report_msg);
}

void RaptorDbwCAN::timerMylapsCallback()
{
    pub_mylaps_report_->publish(mylaps_report_msg);
}

void RaptorDbwCAN::enableSystem()
{
  // placeholder
}

void RaptorDbwCAN::disableSystem()
{
  // placeholder
}

void RaptorDbwCAN::publishJointStates(
  const rclcpp::Time stamp,
  const WheelSpeedReport wheels)
{
  double dt = stamp.seconds() - joint_state_.header.stamp.sec;
  joint_state_.velocity[JOINT_FL] = wheels.front_left;
  joint_state_.velocity[JOINT_FR] = wheels.front_right;
  joint_state_.velocity[JOINT_RL] = wheels.rear_left;
  joint_state_.velocity[JOINT_RR] = wheels.rear_right;

  if (dt < 0.5) {
    for (unsigned int i = JOINT_FL; i <= JOINT_RR; i++) {
      joint_state_.position[i] = fmod(
        joint_state_.position[i] + dt * joint_state_.velocity[i],
        2 * M_PI);
    }
  }
  joint_state_.header.stamp = rclcpp::Time(stamp);
  pub_joint_states_->publish(joint_state_);
}

void RaptorDbwCAN::publishJointStates(
  const rclcpp::Time stamp,
  const SteeringReport steering)
{
  double dt = stamp.seconds() - joint_state_.header.stamp.sec;
  const double L = acker_wheelbase_;
  const double W = acker_track_;
  const double r = L / tan(steering.steering_wheel_angle / steering_ratio_);
  joint_state_.position[JOINT_SL] = atan(L / (r - W / 2));
  joint_state_.position[JOINT_SR] = atan(L / (r + W / 2));

  if (dt < 0.5) {
    for (unsigned int i = JOINT_FL; i <= JOINT_RR; i++) {
      joint_state_.position[i] = fmod(
        joint_state_.position[i] + dt * joint_state_.velocity[i],
        2 * M_PI);
    }
  }
  joint_state_.header.stamp = rclcpp::Time(stamp);
  pub_joint_states_->publish(joint_state_);
}
}  // namespace raptor_dbw_can
