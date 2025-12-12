#ifndef B3C_SENSOR_CONFIG_H_
#define B3C_SENSOR_CONFIG_H_

#include "stdint.h"
#include "bionics_const.h"

#include "B3C_SW_UTIL_Emulator.h"

// Sensor Enumeration
typedef enum 
{
  SYS_INPUT_SM_MB_CH0 = 0,      // 0
  SYS_INPUT_SM_MB_CH1,          // 1
  SYS_INPUT_SM_MB_CH2,          // 2
  SYS_INPUT_SM_MB_CH3,          // 3
  SYS_INPUT_SM_MB_CH4,          // 4
  SYS_INPUT_SM_MB_CH5,          // 5
  SYS_INPUT_SM_MB_CH6,          // 6
  SYS_INPUT_SM_MB_CH7,          // 7
  SYS_INPUT_SHANK_LIN_ACC_X,    // 8
  SYS_INPUT_SHANK_LIN_ACC_Y,    // 9
  SYS_INPUT_SHANK_LIN_ACC_Z,    // 10
  SYS_INPUT_SHANK_GRAV_X,       // 11
  SYS_INPUT_SHANK_GRAV_Y,       // 12
  SYS_INPUT_SHANK_GRAV_Z,       // 13
  SYS_INPUT_CT_ANGLE,           // 14
  SYS_INPUT_CT_FIELD_MAG,       // 15
  SYS_INPUT_CT_TORQUE,          // 16
  SYS_INPUT_KNEE_ANGLE,         // 17
  SYS_INPUT_TORQUE_ACTUAL,      // 18
  SYS_INPUT_MOTOR_CURRENT,      // 19
  SYS_INPUT_MOTOR_TEMP,         // 20
  SYS_INPUT_MOSFET_TEMP,        // 21
  SYS_INPUT_GCS_0,              // 22
  SYS_INPUT_GCS_1,              // 23
  SYS_INPUT_GCS_2,              // 24
  SYS_INPUT_GCS_3,              // 25
  SYS_INPUT_SM_MDB_CH0,         // 26
  SYS_INPUT_SM_MDB_CH1,         // 27
  SYS_INPUT_SM_MDB_CH2,         // 28
  SYS_INPUT_SM_MDB_CH3,         // 29
  SYS_INPUT_SM_EXP_CH0,         // 30
  SYS_INPUT_SM_EXP_CH1,         // 31
  SYS_INPUT_SM_EXP_CH2,         // 32
  SYS_INPUT_SM_EXP_CH3,         // 33
  SYS_INPUT_BAT_TEMPERATURE,    // 34
  SYS_INPUT_BAT_VOLTAGE,        // 35
  SYS_INPUT_BAT_CURRENT,        // 36
  SYS_INPUT_BAT_REMAINING_CAPACITY, // 37
  SYS_INPUT_BAT_STATUS,         // 38
  SYS_INPUT_REL_STATE_OF_CHARGE, // 39
  SYS_INPUT_EXP_TEMP,           // 40
  SYS_NUM_INPUTS                // 41
} systemInputId;

typedef enum 
{
  SYS_SENSOR_MB_ADC = 0,
  SYS_SENSOR_IMU,
  SYS_SENSOR_CT,
  SYS_SENSOR_KA,
  SYS_SENSOR_MOTOR_DRIVE,
  SYS_SENSOR_GCS,
  SYS_SENSOR_MDB_ADC,
  SYS_SENSOR_EXP_ADC,
  SYS_SENSOR_BATTERY,
  SYS_SENSOR_TEMPERATURE,
  SYS_SENSORS_NUM
} systemSensorId;

#endif /* B3C_SENSOR_CONFIG_H_*/
