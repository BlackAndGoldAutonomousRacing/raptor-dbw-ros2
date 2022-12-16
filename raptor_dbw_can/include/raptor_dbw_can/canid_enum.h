/* Enumeration of CAN IDs as provided in the CAN DBC file:
 * ./launch/CAN1-INDY-V11.dbc
 *
 * To re-generate this file, run from the package directory:
 * python ./scripts/dbc_to_c_canid_enum.py ./launch/CAN1-INDY-V11.dbc ./include/raptor_dbw_can/canid_enum.h
 *
 * THIS FILE IS AUTOMATICALLY GENERATED. ANY DIRECT MODIFICATION TO THIS FILE IS
 * STRONGLY DISCOURAGED. YOUR CHANGES TO THIS FILE MAY BE LOST UPON RE-GENERATION.
 */

#ifndef RAPTOR_DBW_CAN__CANID_ENUM_HPP_
#define RAPTOR_DBW_CAN__CANID_ENUM_HPP_

namespace raptor_dbw_can
{
#undef BUILD_ASSERT

/** \brief Enumeration of CAN message IDs */
enum class MessageID
{

BASE_TO_CAR_SUMMARY = 0x4B0,
BASE_TO_CAR_TIMING = 0x4B8,
REST_OF_FIELD = 0x4B9,
WHEEL_SPEED_REPORT = 0x514,
BRAKE_PRESSURE_REPORT = 0x515,
ACCELERATOR_REPORT = 0x516,
STEERING_REPORT = 0x517,
MISC_REPORT = 0x518,
WHEEL_STRAIN_GAUGE = 0x51E,
WHEEL_POTENTIOMETER_DATA = 0x51F,
STEERING_REPORT_EXTD = 0x520,
TIRE_PRESSURE_FL = 0x528,
TIRE_PRESSURE_FR = 0x529,
TIRE_PRESSURE_RL = 0x52A,
TIRE_PRESSURE_RR = 0x52B,
TIRE_TEMP_FL_1 = 0x52C,
TIRE_TEMP_FL_2 = 0x52D,
TIRE_TEMP_FL_3 = 0x52E,
TIRE_TEMP_FL_4 = 0x52F,
TIRE_TEMP_FR_1 = 0x530,
TIRE_TEMP_FR_2 = 0x531,
TIRE_TEMP_FR_3 = 0x532,
TIRE_TEMP_FR_4 = 0x533,
TIRE_TEMP_RL_1 = 0x534,
TIRE_TEMP_RL_2 = 0x535,
TIRE_TEMP_RL_3 = 0x536,
TIRE_TEMP_RL_4 = 0x537,
TIRE_TEMP_RR_1 = 0x538,
TIRE_TEMP_RR_2 = 0x539,
TIRE_TEMP_RR_3 = 0x53A,
TIRE_TEMP_RR_4 = 0x53B,
PT_REPORT_1 = 0x53C,
PT_REPORT_2 = 0x53D,
DIAGNOSTIC_REPORT = 0x53E,
PT_REPORT_3 = 0x53F,
BRAKE_PRESSURE_CMD = 0x578,
ACCELERATOR_CMD = 0x579,
STEERING_CMD = 0x57A,
GEAR_SHIFT_CMD = 0x57B,
CT_REPORT = 0x57C,
POSITION_HEADING = 0x5DC,       // Is this publishing or subscribing?
VELOCITY_ACCELERATION = 0x5DD,  // Is this publishing or subscribing?
MYLAPS_REPORT_2 = 0xE2,
MYLAPS_REPORT_1 = 0xE4,

};

}       // namespace raptor_dbw_can

#endif  // RAPTOR_DBW_CAN__CANID_ENUM_HPP_

