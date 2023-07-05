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

namespace raptor_dbw_can {

RaptorDbwCAN::RaptorDbwCAN(const rclcpp::NodeOptions& options)
    : Node("raptor_dbw_can_node", options) {
  std::string rc_tmp = this->declare_parameter<std::string>("race_control_system", "spoofed");
  if (rc_tmp == "mylaps") current_rc_type_ = RaceControlSystems::MyLaps;
  else if (rc_tmp == "marelli") current_rc_type_ = RaceControlSystems::Marelli;
  else current_rc_type_ = RaceControlSystems::Spoofed;

  dbw_dbc_file_ = this->declare_parameter<std::string>("dbw_dbc_file");
  dbwDbc_ = NewEagle::DbcBuilder().NewDbc(dbw_dbc_file_);

  // We are only populating mean, median, min, and max values of the raw temp records.
  // Initialize the fields.
  for (int i = 0; i < 4; i++) {
    tire_report_msg.fl_tire_temperature.push_back(-1.0);
    tire_report_msg.fr_tire_temperature.push_back(-1.0);
    tire_report_msg.rl_tire_temperature.push_back(-1.0);
    tire_report_msg.rr_tire_temperature.push_back(-1.0);
  }

  /*
  // Initialize joint state reports and ckermann steering parameters
  acker_wheelbase_ =
      this->declare_parameter<double>("ackermann_wheelbase", 2.8498);         // 112.2 inches
  acker_track_ = this->declare_parameter<double>("ackermann_track", 1.5824);  // 62.3 inches
  steering_ratio_ = this->declare_parameter<double>("steering_ratio", 14.8);

  // Initialize joint states
  joint_state_.position.resize(JOINT_COUNT);
  joint_state_.velocity.resize(JOINT_COUNT);
  joint_state_.effort.resize(JOINT_COUNT);
  joint_state_.name.resize(JOINT_COUNT);
  joint_state_.name[JOINT_FL] = "wheel_fl";  // Front Left
  joint_state_.name[JOINT_FR] = "wheel_fr";  // Front Right
  joint_state_.name[JOINT_RL] = "wheel_rl";  // Rear Left
  joint_state_.name[JOINT_RR] = "wheel_rr";  // Rear Right
  joint_state_.name[JOINT_SL] = "steer_fl";
  joint_state_.name[JOINT_SR] = "steer_fr";

  */

  // Set up Publishers
  pub_can_ = this->create_publisher<Frame>("can_tx", 20);
  pub_accel_pedal_ = this->create_publisher<AcceleratorPedalReport>("accelerator_pedal_report", 20);
  pub_steering_ = this->create_publisher<SteeringReport>("steering_report", 20);
  pub_steering_ext_ =
      this->create_publisher<SteeringExtendedReport>("steering_extended_report", 20);
  pub_wheel_speeds_ = this->create_publisher<WheelSpeedReport>("wheel_speed_report", 20);
  pub_tire_pressure_ = this->create_publisher<TirePressureReport>("tire_pressure_report", 20);
  pub_brake_2_report_ = this->create_publisher<Brake2Report>("brake_2_report", 20);
  // pub_joint_states_ = this->create_publisher<JointState>("joint_states", 10);

  // pub_flags_ = this->create_publisher<BaseToCarSummary>("flag_report", 20);
  pub_misc_do_ = this->create_publisher<MiscReport>("misc_report_do", 10);
  // TODO: MIT named their message as RaceControlReport, which is more comprehensive.
  pub_rc_to_ct_ = this->create_publisher<RcToCt>("rc_to_ct", 10);
  pub_tire_report_ = this->create_publisher<TireReport>("tire_report", 10);
  pub_pt_report_ = this->create_publisher<PtReport>("pt_report", 10);
  pub_diag_report_ = this->create_publisher<DiagnosticReport>("diag_report", 10);
  pub_timing_report_ = this->create_publisher<LapTimeReport>("lap_time_report", 10);
  pub_rc_pos_report_ = this->create_publisher<RaceControlPosReport>("rc_pos_report", 10);

  // Set up Timers
  timer_tire_report_ =
      this->create_wall_timer(10ms, std::bind(&RaptorDbwCAN::timerTireCallback, this));

  timer_pt_report_ = this->create_wall_timer(10ms, std::bind(&RaptorDbwCAN::timerPtCallback, this));

  timer_rc_pos_report_ =
      this->create_wall_timer(200ms, std::bind(&RaptorDbwCAN::timerMylapsCallback, this));

  // Set up Subscribers
  sub_can_ = this->create_subscription<Frame>(
      "can_rx", 500, std::bind(&RaptorDbwCAN::recvCAN, this, std::placeholders::_1));

  sub_brake_ = this->create_subscription<BrakeCmd>(
      "brake_cmd", rclcpp::SensorDataQoS(),
      std::bind(&RaptorDbwCAN::recvBrakeCmd, this, std::placeholders::_1));

  sub_accelerator_pedal_ = this->create_subscription<AcceleratorPedalCmd>(
      "accelerator_pedal_cmd", rclcpp::SensorDataQoS(),
      std::bind(&RaptorDbwCAN::recvAcceleratorPedalCmd, this, std::placeholders::_1));

  sub_steering_ = this->create_subscription<SteeringCmd>(
      "steering_cmd", rclcpp::SensorDataQoS(),
      std::bind(&RaptorDbwCAN::recvSteeringCmd, this, std::placeholders::_1));

  sub_gear_shift_cmd_ = this->create_subscription<Int8>(
      "gear_cmd", rclcpp::SensorDataQoS(),
      std::bind(&RaptorDbwCAN::recvGearShiftCmd, this, std::placeholders::_1));

  sub_ct_report_ = this->create_subscription<CtReport>(
      "ct_report", rclcpp::SensorDataQoS(),
      std::bind(&RaptorDbwCAN::recvCtCmd, this, std::placeholders::_1));

  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS(),
      std::bind(&RaptorDbwCAN::recvImuCmd, this, std::placeholders::_1));

  sub_dash_cmds_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "dashboard_cmd", rclcpp::SensorDataQoS(),
      std::bind(&RaptorDbwCAN::recvDashSwitchesCmd, this, std::placeholders::_1));
}

RaptorDbwCAN::~RaptorDbwCAN() {}

void RaptorDbwCAN::recvCAN(Frame::UniquePtr msg) {
  static constexpr std::string_view fl = "FL";
  static constexpr std::string_view fr = "FR";
  static constexpr std::string_view rl = "RL";
  static constexpr std::string_view rr = "RR";
  static constexpr uint8_t part1 = 1;
  static constexpr uint8_t part2 = 2;
  static constexpr uint8_t part3 = 3;
  static constexpr uint8_t part4 = 4;

  if (msg->is_rtr || msg->is_error) return;

  switch (static_cast<MessageID>(msg->id)) {
    case MessageID::BASE_TO_CAR_SUMMARY:
      recvDORcToCtRpt(*msg);
      break;

    case MessageID::BASE_TO_CAR_TIMING:
      recvDOLapTimeRpt(*msg);
      break;

    case MessageID::MYLAPS_REPORT_1:
      recvMyLapsRptPart1(*msg);
      break;

    case MessageID::MYLAPS_REPORT_2:
      recvMarelliRptPart2(*msg);
      break;

    case MessageID::MARELLI_REPORT_1:
      recvMarelliRptPart1(*msg);
      break;

    case MessageID::MARELLI_REPORT_2:
      recvMarelliRptPart2(*msg);
      break;

    case MessageID::WHEEL_SPEED_REPORT:
      recvWheelSpeedRpt(*msg);
      break;

    case MessageID::BRAKE_PRESSURE_REPORT:
      recvBrake2Rpt(*msg);
      break;

    case MessageID::ACCELERATOR_REPORT:
      recvAccelPedalRpt(*msg);
      break;

    case MessageID::STEERING_REPORT:
      recvSteeringRpt(*msg);
      break;

    case MessageID::MISC_REPORT:
      recvDOMiscRpt(*msg);
      break;

    case MessageID::WHEEL_STRAIN_GAUGE:
      recvWheelStrainGaugeRpt(*msg);
      break;

    case MessageID::WHEEL_POTENTIOMETER_DATA:
      recvWheelPotentialmeterRpt(*msg);
      break;

    case MessageID::STEERING_REPORT_EXTD:
      recvSteeringExtdRpt(*msg);
      break;

    case MessageID::TIRE_PRESSURE_FL:
      recvTirePressureRpt(*msg, msg->id, fl, tire_report_msg.fl_tire_pressure,
                          tire_report_msg.fl_tire_pressure_gauge);
      break;

    case MessageID::TIRE_PRESSURE_FR:
      recvTirePressureRpt(*msg, msg->id, fr, tire_report_msg.fr_tire_pressure,
                          tire_report_msg.fr_tire_pressure_gauge);
      break;

    case MessageID::TIRE_PRESSURE_RL:
      recvTirePressureRpt(*msg, msg->id, rl, tire_report_msg.rl_tire_pressure,
                          tire_report_msg.rl_tire_pressure_gauge);
      break;

    case MessageID::TIRE_PRESSURE_RR:
      recvTirePressureRpt(*msg, msg->id, rr, tire_report_msg.rr_tire_pressure,
                          tire_report_msg.rr_tire_pressure_gauge);
      break;

    case MessageID::TIRE_TEMP_FL_1:
      recvTireTempRpt(*msg, msg->id, fl, part1, tire_temp_raw_array_fl_);
      break;

    case MessageID::TIRE_TEMP_FL_2:
      recvTireTempRpt(*msg, msg->id, fl, part2, tire_temp_raw_array_fl_);
      break;

    case MessageID::TIRE_TEMP_FL_3:
      recvTireTempRpt(*msg, msg->id, fl, part3, tire_temp_raw_array_fl_);
      break;

    case MessageID::TIRE_TEMP_FL_4:
      recvTireTempRpt(*msg, msg->id, fl, part4, tire_temp_raw_array_fl_);
      break;

    case MessageID::TIRE_TEMP_FR_1:
      recvTireTempRpt(*msg, msg->id, fr, part1, tire_temp_raw_array_fr_);
      break;

    case MessageID::TIRE_TEMP_FR_2:
      recvTireTempRpt(*msg, msg->id, fr, part2, tire_temp_raw_array_fr_);
      break;

    case MessageID::TIRE_TEMP_FR_3:
      recvTireTempRpt(*msg, msg->id, fr, part3, tire_temp_raw_array_fr_);
      break;

    case MessageID::TIRE_TEMP_FR_4:
      recvTireTempRpt(*msg, msg->id, fr, part4, tire_temp_raw_array_fr_);
      break;

    case MessageID::TIRE_TEMP_RL_1:
      recvTireTempRpt(*msg, msg->id, rl, part1, tire_temp_raw_array_rl_);
      break;

    case MessageID::TIRE_TEMP_RL_2:
      recvTireTempRpt(*msg, msg->id, rl, part2, tire_temp_raw_array_rl_);
      break;

    case MessageID::TIRE_TEMP_RL_3:
      recvTireTempRpt(*msg, msg->id, rl, part3, tire_temp_raw_array_rl_);
      break;

    case MessageID::TIRE_TEMP_RL_4:
      recvTireTempRpt(*msg, msg->id, rl, part4, tire_temp_raw_array_rl_);
      break;

    case MessageID::TIRE_TEMP_RR_1:
      recvTireTempRpt(*msg, msg->id, rr, part1, tire_temp_raw_array_rr_);
      break;

    case MessageID::TIRE_TEMP_RR_2:
      recvTireTempRpt(*msg, msg->id, rr, part2, tire_temp_raw_array_rr_);
      break;

    case MessageID::TIRE_TEMP_RR_3:
      recvTireTempRpt(*msg, msg->id, rr, part3, tire_temp_raw_array_rr_);
      break;

    case MessageID::TIRE_TEMP_RR_4:
      recvTireTempRpt(*msg, msg->id, rr, part4, tire_temp_raw_array_rr_);
      break;

    case MessageID::PT_REPORT_1:
      recvDOPtRptPart1(*msg);
      break;

    case MessageID::PT_REPORT_2:
      recvDOPtRptPart2(*msg);
      break;

    case MessageID::DIAGNOSTIC_REPORT:
      recvDODiagRpt(*msg);
      break;

    case MessageID::PT_REPORT_3:
      recvDOPtRptPart3(*msg);
      break;

    default:
      break;
  }
}

// computes values of TireTemp message for a given array of tire temperatures.
void RaptorDbwCAN::postprocessTireTemp() {
  std::vector<float> fl_temps(tire_temp_raw_array_fl_,
                              tire_temp_raw_array_fl_ + TIRE_TEMP_SENSORS_PER_WHEEL);
  std::vector<float> fr_temps(tire_temp_raw_array_fr_,
                              tire_temp_raw_array_fr_ + TIRE_TEMP_SENSORS_PER_WHEEL);
  std::vector<float> rl_temps(tire_temp_raw_array_rl_,
                              tire_temp_raw_array_rl_ + TIRE_TEMP_SENSORS_PER_WHEEL);
  std::vector<float> rr_temps(tire_temp_raw_array_rr_,
                              tire_temp_raw_array_rr_ + TIRE_TEMP_SENSORS_PER_WHEEL);

  // mean
  tire_report_msg.fl_tire_temperature[0] =
      std::reduce(fl_temps.begin(), fl_temps.end(), 0.) / TIRE_TEMP_SENSORS_PER_WHEEL;
  tire_report_msg.fr_tire_temperature[0] =
      std::reduce(fr_temps.begin(), fr_temps.end(), 0.) / TIRE_TEMP_SENSORS_PER_WHEEL;
  tire_report_msg.fl_tire_temperature[0] =
      std::reduce(rl_temps.begin(), rl_temps.end(), 0.) / TIRE_TEMP_SENSORS_PER_WHEEL;
  tire_report_msg.fr_tire_temperature[0] =
      std::reduce(rr_temps.begin(), rr_temps.end(), 0.) / TIRE_TEMP_SENSORS_PER_WHEEL;

  // median
  std::sort(fl_temps.begin(), fl_temps.end());
  std::sort(fr_temps.begin(), fr_temps.end());
  std::sort(rl_temps.begin(), rl_temps.end());
  std::sort(rr_temps.begin(), rr_temps.end());

  tire_report_msg.fl_tire_temperature[1] = 0.5 * (fl_temps[TIRE_TEMP_SENSORS_PER_WHEEL / 2 - 1] +
                                                  fl_temps[TIRE_TEMP_SENSORS_PER_WHEEL / 2]);
  tire_report_msg.fr_tire_temperature[1] = 0.5 * (fr_temps[TIRE_TEMP_SENSORS_PER_WHEEL / 2 - 1] +
                                                  fr_temps[TIRE_TEMP_SENSORS_PER_WHEEL / 2]);
  tire_report_msg.rl_tire_temperature[1] = 0.5 * (rl_temps[TIRE_TEMP_SENSORS_PER_WHEEL / 2 - 1] +
                                                  rl_temps[TIRE_TEMP_SENSORS_PER_WHEEL / 2]);
  tire_report_msg.rr_tire_temperature[1] = 0.5 * (rr_temps[TIRE_TEMP_SENSORS_PER_WHEEL / 2 - 1] +
                                                  rr_temps[TIRE_TEMP_SENSORS_PER_WHEEL / 2]);

  // min
  tire_report_msg.fl_tire_temperature[2] = fl_temps[0];
  tire_report_msg.fr_tire_temperature[2] = fr_temps[0];
  tire_report_msg.rl_tire_temperature[2] = rl_temps[0];
  tire_report_msg.rr_tire_temperature[2] = rr_temps[0];

  // max
  tire_report_msg.fl_tire_temperature[3] = fl_temps[15];
  tire_report_msg.fr_tire_temperature[3] = fr_temps[15];
  tire_report_msg.rl_tire_temperature[3] = rl_temps[15];
  tire_report_msg.rr_tire_temperature[3] = rr_temps[15];
}

void RaptorDbwCAN::timerTireCallback() {
  postprocessTireTemp();
  pub_tire_report_->publish(tire_report_msg);
}

void RaptorDbwCAN::timerPtCallback() { pub_pt_report_->publish(pt_report_msg); }

void RaptorDbwCAN::timerMylapsCallback() { pub_rc_pos_report_->publish(rc_pos_report_msg_); }

/*
void RaptorDbwCAN::enableSystem() {
  // placeholder
}

void RaptorDbwCAN::disableSystem() {
  // placeholder
}
*/

// Separating joint kinematics calculation from the interface code is better, as it prevents
// coupling of functionalities. A separate odometry integrator node is in charge of this.
/*
void RaptorDbwCAN::publishJointStates(
  const rclcpp::Time stamp,
  const WheelSpeedReport& wheels)
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
  const SteeringReport& steering)
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
*/

}  // namespace raptor_dbw_can
