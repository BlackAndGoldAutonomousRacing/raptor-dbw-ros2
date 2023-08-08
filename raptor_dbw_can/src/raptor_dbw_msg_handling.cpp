/**
 * @file raptor_dbw_msg_handling.cpp
 * @author Haoguang Yang (yang1510@purdue.edu)
 * @brief This source implements the conversion between involved CAN frames and ROS 2 messages.
 * @version 0.1
 * @date 2023-07-05
 * 
 * @copyright Copyright (c) 2023 Haoguang Yang
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "raptor_dbw_can/raptor_dbw_can.hpp"

namespace raptor_dbw_can {

void RaptorDbwCAN::recvAccelPedalRpt(const Frame& msg) {
  auto out = std::make_unique<AcceleratorPedalReport>();
  DbcFrameRx dbcFrame{msg, MessageID::ACCELERATOR_REPORT, dbwDbc_, out->header.stamp};
  if (!dbcFrame.valid()) return;
  dbcFrame("acc_pedal_fdbk", out->pedal_output)
          ("acc_pedal_fdbk_counter", out->rolling_counter);
  out->pedal_input = last_acc_pedal_cmd_;
  pub_accel_pedal_->publish(std::move(out));
}

void RaptorDbwCAN::recvSteeringRpt(const Frame& msg) {
  auto out = std::make_unique<SteeringReport>();
  DbcFrameRx dbcFrame{msg, MessageID::STEERING_REPORT, dbwDbc_, out->header.stamp};
  if (!dbcFrame.valid()) return;
  dbcFrame("steering_motor_ang_avg_fdbk", out->steering_wheel_angle)
          ("steering_motor_fdbk_counter", out->rolling_counter);
  out->steering_wheel_angle_cmd = last_steering_cmd_;
  pub_steering_->publish(std::move(out));
  // See comments at the function.
  // publishJointStates(msg.header.stamp, out);
}

void RaptorDbwCAN::recvSteeringExtdRpt(const Frame& msg) {
  auto out = std::make_unique<SteeringExtendedReport>();
  DbcFrameRx dbcFrame{msg, MessageID::STEERING_REPORT_EXTD, dbwDbc_, out->header.stamp};
  if (!dbcFrame.valid()) return;
  dbcFrame("steering_motor_ang_1_fdbk", out->steering_motor_ang_1)
          ("steering_motor_ang_2_fdbk", out->steering_motor_ang_2)
          ("steering_motor_ang_3_fdbk", out->steering_motor_ang_3);
  pub_steering_ext_->publish(std::move(out));
}

void RaptorDbwCAN::recvDOLapTimeRpt(const Frame& msg) {
  auto out = std::make_unique<LapTimeReport>();
  DbcFrameRx dbcFrame{msg, MessageID::BASE_TO_CAR_TIMING, dbwDbc_, out->stamp};
  if (!dbcFrame.valid()) return;
  dbcFrame("laps", out->laps)
          ("lap_time", out->lap_time)
          ("time_stamp", out->session_elapsed_time);
  pub_timing_report_->publish(std::move(out));
}

void RaptorDbwCAN::recvDORcToCtRpt(const Frame& msg) {
  DbcFrameRx dbcFrame{msg, MessageID::BASE_TO_CAR_SUMMARY, dbwDbc_, race_control_cmd_.stamp};
  if (!dbcFrame.valid()) return;
  dbcFrame("base_to_car_heartbeat", race_control_cmd_.rolling_counter)
          ("veh_rank", race_control_cmd_.vehicle_rank)
          ("lap_count", race_control_cmd_.lap_count)
          ("lap_distance", race_control_cmd_.lap_distance)
          ("round_target_speed", race_control_cmd_.round_target_speed);
  if (current_rc_type_ != RaceControlSystems::Marelli) {
    // prevent overwriting shared fields when under different race control systems
    dbcFrame("track_flag", race_control_cmd_.track_flag)
            ("veh_flag", race_control_cmd_.vehicle_flag);
  }
  pub_rc_to_ct_->publish(race_control_cmd_);
}

void RaptorDbwCAN::recvWheelSpeedRpt(const Frame& msg) {
  auto out = std::make_unique<WheelSpeedReport>();
  DbcFrameRx dbcFrame{msg, MessageID::WHEEL_SPEED_REPORT, dbwDbc_, out->header.stamp};
  if (!dbcFrame.valid()) return;
  dbcFrame("wheel_speed_FL", out->front_left)
          ("wheel_speed_FR", out->front_right)
          ("wheel_speed_RL", out->rear_left)
          ("wheel_speed_RR", out->rear_right);
  pub_wheel_speeds_->publish(std::move(out));
  // See comments at the function.
  // publishJointStates(msg.header.stamp, out);
}

void RaptorDbwCAN::recvTirePressureRpt(const Frame& msg, const uint32_t& canid,
                                       const std::string_view& which, float& pressure,
                                       float& gauge) {
  NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(canid);
  if (msg.dlc < message->GetDlc()) return;
  message->SetFrame(msg);
  // fill in which tire: FL, FR, ...
  pressure = message->GetSignal(std::string(which) + "_Tire_Pressure")->GetResult();
  gauge = message->GetSignal(std::string(which) + "_Tire_Pressure_Gauge")->GetResult();
}

void RaptorDbwCAN::recvTireTempRpt(const Frame& msg, const uint32_t& canid,
                                   const std::string_view& which, const uint8_t& part,
                                   float* temp) {
  NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(canid);
  if (msg.dlc < message->GetDlc()) return;
  message->SetFrame(msg);
  char buf[3];
  // fill in the fields: part1: 0..3, etc., corresponds to dbc signals _01, _02, ..._16.
  // fill in which tire: FL, FR, ...
  for (int index = (part - 1) * 4; index < part * 4; index++) {
    snprintf(buf, 3, "%02d", 1 + (index % static_cast<int>(TIRE_TEMP_SENSORS_PER_WHEEL)));
    temp[index] =
        message->GetSignal(std::string(which) + "_Tire_Temp_" + std::string(buf))->GetResult();
  }
}

void RaptorDbwCAN::recvBrake2Rpt(const Frame& msg) {
  auto out = std::make_unique<Brake2Report>();
  DbcFrameRx dbcFrame{msg, MessageID::BRAKE_PRESSURE_REPORT, dbwDbc_, out->header.stamp};
  if (!dbcFrame.valid()) return;
  dbcFrame("brake_pressure_fdbk_front", out->front_brake_pressure)
          ("brake_pressure_fdbk_rear", out->rear_brake_pressure)
          ("brk_pressure_fdbk_counter", out->rolling_counter);
  pub_brake_2_report_->publish(std::move(out));
}

void RaptorDbwCAN::recvDOMiscRpt(const Frame& msg) {
  auto out = std::make_unique<MiscReport>();
  DbcFrameRx dbcFrame{msg, MessageID::MISC_REPORT, dbwDbc_, out->stamp};
  if (!dbcFrame.valid()) return;
  dbcFrame("sys_state", out->sys_state)
          ("safety_switch_state", out->safety_switch_state)
          ("mode_switch_state", out->mode_switch_state)
          ("battery_voltage", out->battery_voltage);
  pub_misc_do_->publish(std::move(out));
}

void RaptorDbwCAN::recvDODiagRpt(const Frame& msg) {
  auto out = std::make_unique<DiagnosticReport>();
  DbcFrameRx dbcFrame{msg, MessageID::DIAGNOSTIC_REPORT, dbwDbc_, out->stamp};
  if (!dbcFrame.valid()) return;
  dbcFrame("sd_system_warning", out->sd_system_warning)
          ("sd_system_failure", out->sd_system_failure)
          ("sd_brake_warning1", out->sd_brake_warning1)
          ("sd_brake_warning2", out->sd_brake_warning2)
          ("sd_brake_warning3", out->sd_brake_warning3)
          ("sd_steer_warning1", out->sd_steer_warning1)
          ("sd_steer_warning2", out->sd_steer_warning2)
          ("sd_steer_warning3", out->sd_steer_warning3)
          ("motec_warning", out->motec_warning)
          ("est1_oos_front_brk", out->est1_oos_front_brk)
          ("est2_oos_rear_brk", out->est2_oos_rear_brk)
          ("est3_low_eng_speed", out->est3_low_eng_speed)
          ("est4_sd_comms_loss", out->est4_sd_comms_loss)
          ("est5_motec_comms_loss", out->est5_motec_comms_loss)
          ("est6_sd_ebrake", out->est6_sd_ebrake)
          ("adlink_hb_lost", out->adlink_hb_lost)
          ("rc_lost", out->rc_lost);
  pub_diag_report_->publish(std::move(out));
}

void RaptorDbwCAN::recvDOPtRptPart1(const Frame& msg) {
  DbcFrameRx dbcFrame{msg, MessageID::PT_REPORT_1, dbwDbc_, pt_report_msg.stamp};
  if (!dbcFrame.valid()) return;
  dbcFrame("throttle_position", pt_report_msg.throttle_position)
          ("engine_run_switch", pt_report_msg.engine_run_switch_status)
          ("current_gear", pt_report_msg.current_gear)
          ("engine_speed_rpm", pt_report_msg.engine_rpm)
          ("vehicle_speed_kmph", pt_report_msg.vehicle_speed_kmph)
          ("engine_state", pt_report_msg.engine_on_status)
          ("gear_shift_status", pt_report_msg.gear_shift_status);
}

void RaptorDbwCAN::recvDOPtRptPart2(const Frame& msg) {
  Time stamp_tmp;
  DbcFrameRx dbcFrame{msg, MessageID::PT_REPORT_2, dbwDbc_, stamp_tmp};
  if (!dbcFrame.valid()) return;
  dbcFrame("fuel_pressure_kPa", pt_report_msg.fuel_pressure)
          ("engine_oil_pressure_kPa", pt_report_msg.engine_oil_pressure)
          ("coolant_temperature", pt_report_msg.engine_coolant_temperature)
          ("transmission_temperature", pt_report_msg.transmission_oil_temperature)
          ("transmission_pressure_kPa", pt_report_msg.transmission_oil_pressure);
}

void RaptorDbwCAN::recvDOPtRptPart3(const Frame& msg) {
  Time stamp_tmp;
  DbcFrameRx dbcFrame{msg, MessageID::PT_REPORT_3, dbwDbc_, stamp_tmp};
  if (!dbcFrame.valid()) return;
  dbcFrame("engine_oil_temperature", pt_report_msg.engine_oil_temperature)
          ("torque_wheels", pt_report_msg.wheel_torque_total)
          ("driver_traction_aim_swicth_fbk", pt_report_msg.driver_traction_aim_switch)
          ("driver_traction_range_switch_fbk", pt_report_msg.driver_traction_range_switch);
  // This reset mechanism synchronizes the publisher timer to the latest receiving cycle.
  timer_pt_report_->reset();
  pub_pt_report_->publish(pt_report_msg);
}

void RaptorDbwCAN::recvWheelStrainGaugeRpt(const Frame& msg) {
  Time stamp_tmp;
  DbcFrameRx dbcFrame{msg, MessageID::WHEEL_STRAIN_GAUGE, dbwDbc_, stamp_tmp};
  if (!dbcFrame.valid()) return;
  // TODO: verify -- these fields may be unused.
  dbcFrame("wheel_strain_gauge_FL", tire_report_msg.fl_wheel_load)
          ("wheel_strain_gauge_FR", tire_report_msg.fr_wheel_load)
          ("wheel_strain_gauge_RL", tire_report_msg.rl_wheel_load)
          ("wheel_strain_gauge_RR", tire_report_msg.rr_wheel_load);
}

void RaptorDbwCAN::recvWheelPotentialmeterRpt(const Frame& msg) {
  DbcFrameRx dbcFrame{msg, MessageID::WHEEL_POTENTIOMETER_DATA, dbwDbc_, tire_report_msg.stamp};
  if (!dbcFrame.valid()) return;
  dbcFrame("wheel_potentiometer_FL", tire_report_msg.fl_damper_linear_potentiometer)
          ("wheel_potentiometer_FR", tire_report_msg.fr_damper_linear_potentiometer)
          ("wheel_potentiometer_RL", tire_report_msg.rl_damper_linear_potentiometer)
          ("wheel_potentiometer_RR", tire_report_msg.rr_damper_linear_potentiometer);
}

void RaptorDbwCAN::recvMyLapsRptPart1(const Frame& msg) {
  Time stamp_tmp;
  DbcFrameRx dbcFrame{msg, MessageID::MYLAPS_REPORT_1, dbwDbc_, stamp_tmp};
  if (!dbcFrame.valid()) return;
  dbcFrame("mylaps_speed", rc_pos_report_msg_.speed)
          ("mylaps_heading", rc_pos_report_msg_.heading);
}

void RaptorDbwCAN::recvMyLapsRptPart2(const Frame& msg) {
  DbcFrameRx dbcFrame{msg, MessageID::MYLAPS_REPORT_2, dbwDbc_, rc_pos_report_msg_.stamp};
  if (!dbcFrame.valid()) return;
  // if (current_rc_type_ == RaceControlSystems::Marelli) return;
  dbcFrame("Latitude", rc_pos_report_msg_.latitude)
          ("Longitude", rc_pos_report_msg_.longitude);
  // This reset mechanism synchronizes the publisher timer to the latest receiving cycle.
  timer_rc_pos_report_->reset();
  pub_rc_pos_report_->publish(rc_pos_report_msg_);
}

void RaptorDbwCAN::recvMarelliRptPart1(const Frame& msg) {
  Time stamp;
  DbcFrameRx dbcFrame{msg, MessageID::MARELLI_REPORT_1, dbwDbc_, stamp};
  if (!dbcFrame.valid()) return;
  dbcFrame("marelli_rc_lte_rssi", race_control_cmd_.lte_rssi)
          ("marelli_rc_base_sync_check", race_control_cmd_.lte_sync_ok);
  if (current_rc_type_ == RaceControlSystems::Marelli){
    // prevent overwriting shared fields when under different race control systems
    dbcFrame("marelli_track_flag", race_control_cmd_.track_flag)
            ("marelli_vehicle_flag_ack", race_control_cmd_.vehicle_flag)
            ("marelli_sector_flag", race_control_cmd_.sector_flag);
    race_control_cmd_.stamp = stamp;
    pub_rc_to_ct_->publish(race_control_cmd_);
  }

  // acknowledge flags
  DbcFrameTx dbcFrameTx{MessageID::CT_REPORT_2, dbwDbc_};
  dbcFrameTx("marelli_track_flag_ack", race_control_cmd_.track_flag)
            ("marelli_vehicle_flag_ack", race_control_cmd_.vehicle_flag)
            ("marelli_sector_flag_ack", race_control_cmd_.sector_flag);
  auto frame = std::make_unique<Frame>(dbcFrameTx.message->GetFrame());
  pub_can_->publish(std::move(frame));
}

void RaptorDbwCAN::recvMarelliRptPart2(const Frame& msg) {
  DbcFrameRx dbcFrame{msg, MessageID::MARELLI_REPORT_2, dbwDbc_, rc_pos_report_msg_.stamp};
  if (!dbcFrame.valid()) return;
  // if (current_rc_type_ != RaceControlSystems::Marelli) return;
  dbcFrame("marelli_gps_lat", rc_pos_report_msg_.latitude)
          ("marelli_gps_long", rc_pos_report_msg_.longitude);
  // This reset mechanism synchronizes the publisher timer to the latest receiving cycle.
  timer_rc_pos_report_->reset();
  pub_rc_pos_report_->publish(rc_pos_report_msg_);
}

// these two functions are currently not used.
/*
void RaptorDbwCAN::recvEnable(Empty::UniquePtr msg) {
  if (msg == NULL) return;
  enableSystem();
}

void RaptorDbwCAN::recvDisable(Empty::UniquePtr msg) {
  if (msg == NULL) return;
  disableSystem();
}
*/

void RaptorDbwCAN::recvBrakeCmd(BrakeCmd::UniquePtr msg) {
  DbcFrameTx dbcFrame{MessageID::BRAKE_PRESSURE_CMD, dbwDbc_};
  dbcFrame("brake_pressure_cmd", msg->pedal_cmd)
          ("brk_pressure_cmd_counter", msg->rolling_counter);
  auto frame = std::make_unique<Frame>(dbcFrame.message->GetFrame());
  pub_can_->publish(std::move(frame));
}

void RaptorDbwCAN::recvAcceleratorPedalCmd(AcceleratorPedalCmd::UniquePtr msg) {
  DbcFrameTx dbcFrame{MessageID::ACCELERATOR_CMD, dbwDbc_};
  dbcFrame("acc_pedal_cmd", msg->pedal_cmd)
          ("acc_pedal_cmd_counter", msg->rolling_counter);
  auto frame = std::make_unique<Frame>(dbcFrame.message->GetFrame());
  pub_can_->publish(std::move(frame));
  last_acc_pedal_cmd_ = msg->pedal_cmd;
}

void RaptorDbwCAN::recvSteeringCmd(SteeringCmd::UniquePtr msg) {
  DbcFrameTx dbcFrame{MessageID::STEERING_CMD, dbwDbc_};
  dbcFrame("steering_motor_ang_cmd", msg->angle_cmd)
          ("steering_motor_cmd_counter", msg->rolling_counter);
  auto frame = std::make_unique<Frame>(dbcFrame.message->GetFrame());
  pub_can_->publish(std::move(frame));
  last_steering_cmd_ = msg->angle_cmd;
}

void RaptorDbwCAN::recvGearShiftCmd(std_msgs::msg::Int8::UniquePtr msg) {
  DbcFrameTx dbcFrame{MessageID::GEAR_SHIFT_CMD, dbwDbc_};
  dbcFrame("desired_gear", msg->data);
  auto frame = std::make_unique<Frame>(dbcFrame.message->GetFrame());
  pub_can_->publish(std::move(frame));
}

void RaptorDbwCAN::recvCtCmd(deep_orange_msgs::msg::CtReport::UniquePtr msg) {
  DbcFrameTx dbcFrame{MessageID::CT_REPORT, dbwDbc_};
  dbcFrame("veh_num", msg->vehicle_id)
          ("ct_state", msg->ct_state)
          ("ct_state_rolling_counter", msg->rolling_counter);
  if (current_rc_type_ != RaceControlSystems::Marelli){
    dbcFrame("track_cond_ack", msg->track_flag_ack)
            ("veh_sig_ack", msg->vehicle_flag_ack);
  }
  auto frame = std::make_unique<Frame>(dbcFrame.message->GetFrame());
  pub_can_->publish(std::move(frame));
}

void RaptorDbwCAN::recvImuCmd(sensor_msgs::msg::Imu::UniquePtr msg) {
  DbcFrameTx dbcFrame{MessageID::CT_VEHICLE_ACC_FEEDBACK, dbwDbc_};
  dbcFrame("long_ct_vehicle_acc_fbk", msg->linear_acceleration.x / GRAVITY)
          ("lat_ct_vehicle_acc_fbk", msg->linear_acceleration.y / GRAVITY)
          ("vertical_ct_vehicle_acc_fbk", (msg->linear_acceleration.z - GRAVITY) / GRAVITY);
  auto frame = std::make_unique<Frame>(dbcFrame.message->GetFrame());
  pub_can_->publish(std::move(frame));
}

void RaptorDbwCAN::recvDashSwitchesCmd(std_msgs::msg::UInt8MultiArray::UniquePtr msg) {
  if (msg->data.size() < 2) return;
  switch (msg->data[0]) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
      last_traction_aim_ = msg->data[0];
      break;
    default:
      last_traction_aim_ = TRACTION_AIM_DEFAULT;
      break;
  }
  switch (msg->data[1]) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
      last_driver_traction_range_switch_ = msg->data[1];
      break;
    default:
      last_driver_traction_range_switch_ = TRACTION_RANGE_DEFAULT;
      break;
  }
  DbcFrameTx dbcFrame{MessageID::DASH_SWITCHES_CMD, dbwDbc_};
  dbcFrame("driver_traction_aim_switch", last_traction_aim_)
          ("driver_traction_range_switch", last_driver_traction_range_switch_);
  auto frame = std::make_unique<Frame>(dbcFrame.message->GetFrame());
  pub_can_->publish(std::move(frame));
}

}   // namespace raptor_dbw_can
