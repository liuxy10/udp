#ifndef B3C_EMULATOR_CONFIG_H_
#define B3C_EMULATOR_CONFIG_H_

// Sensor Emulation: Variable Index
typedef enum 
{
  EMULATE_SYS_INPUT_SM_MB_CH0 = 0,                      // 0
  EMULATE_SYS_INPUT_SM_MB_CH1,                          // 1
  EMULATE_SYS_INPUT_SM_MB_CH2,                          // 2
  EMULATE_SYS_INPUT_SM_MB_CH3,                          // 3
  EMULATE_SYS_INPUT_SM_MB_CH4,                          // 4
  EMULATE_SYS_INPUT_SM_MB_CH5,                          // 5
  EMULATE_SYS_INPUT_SM_MB_CH6,                          // 6
  EMULATE_SYS_INPUT_SM_MB_CH7,                          // 7
  EMULATE_SYS_INPUT_SHANK_LIN_ACC_X,                    // 8
  EMULATE_SYS_INPUT_SHANK_LIN_ACC_Y,                    // 9
  EMULATE_SYS_INPUT_SHANK_LIN_ACC_Z,                    // 10
  EMULATE_SYS_INPUT_SHANK_GRAV_X,                       // 11
  EMULATE_SYS_INPUT_SHANK_GRAV_Y,                       // 12
  EMULATE_SYS_INPUT_SHANK_GRAV_Z,                       // 13
  EMULATE_SYS_INPUT_CT_ANGLE,                           // 14
  EMULATE_SYS_INPUT_CT_FIELD_MAG,                       // 15
  EMULATE_SYS_INPUT_CT_TORQUE,                          // 16
  EMULATE_SYS_INPUT_KNEE_ANGLE,                         // 17
  EMULATE_SYS_INPUT_TORQUE_ACTUAL,                      // 18
  EMULATE_SYS_INPUT_MOTOR_CURRENT,                      // 19
  EMULATE_SYS_INPUT_MOTOR_TEMP,                         // 20
  EMULATE_SYS_INPUT_MOSFET_TEMP,                        // 21
  EMULATE_SYS_INPUT_GCS_0,                              // 22
  EMULATE_SYS_INPUT_GCS_1,                              // 23
  EMULATE_SYS_INPUT_GCS_2,                              // 24
  EMULATE_SYS_INPUT_GCS_3,                              // 25
  EMULATE_SYS_INPUT_SM_MDB_CH0,                         // 26
  EMULATE_SYS_INPUT_SM_MDB_CH1,                         // 27
  EMULATE_SYS_INPUT_SM_MDB_CH2,                         // 28
  EMULATE_SYS_INPUT_SM_MDB_CH3,                         // 29
  EMULATE_SYS_INPUT_SM_EXP_CH0,                         // 30
  EMULATE_SYS_INPUT_SM_EXP_CH1,                         // 31
  EMULATE_SYS_INPUT_SM_EXP_CH2,                         // 32
  EMULATE_SYS_INPUT_SM_EXP_CH3,                         // 33
  EMULATE_SYS_INPUT_BAT_TEMPERATURE,                    // 34
  EMULATE_SYS_INPUT_BAT_VOLTAGE,                        // 35
  EMULATE_SYS_INPUT_BAT_CURRENT,                        // 36
  EMULATE_SYS_INPUT_BAT_REMAINING_CAPACITY,             // 37
  EMULATE_SYS_INPUT_BAT_STATUS,                         // 38
  EMULATE_SYS_INPUT_REL_STATE_OF_CHARGE,                // 39
  EMULATE_SYS_INPUT_EXP_TEMP,                           // 40
  EMULATE_REACTIVE_AI_CURRENT_ACTIVE_PHASE,             // 41
  EMULATE_REACTIVE_AI_CURRENT_ACTIVE_SUB_PHASE,         // 42
  EMULATE_REACTIVE_AI_CURRENT_ACTIVITY,                 // 43
  EMULATE_REACTIVE_AI_TORQUE,                           // 44
  EMULATE_REACTIVE_AI_KNEE_VELOCITY,                    // 45
  EMULATE_REACTIVE_AI_KNEE_MOTOR_VELOCITY,              // 46
  EMULATE_REACTIVE_AI_ABS_THIGH_ANGLE,                  // 47
  EMULATE_REACTIVE_AI_ABS_SHANK_ANGLE,                  // 48
  EMULATE_REACTIVE_AI_KNEE_ANGLE,                       // 49
  EMULATE_REACTIVE_AI_KNEE_MOTOR_ANGLE,                 // 50
  EMULATE_REACTIVE_AI_SAGITAL_PLANE_OFFSET_ANGLE,       // 51
  EMULATE_REACTIVE_AI_MAX_Y_POS_WITHIN_TRACK_WINDOW,    // 52
  EMULATE_REACTIVE_AI_CURRENT_CONTROL_MODE,             // 53
  EMULATE_REACTIVE_AI_LOAD_CELL,                        // 54
  EMULATE_REACTIVE_AI_CURRENT_CADENCE,                  // 55
  EMULATE_REACTIVE_AI_TOE_OFF_UNLOAD_POINT_FOUND,       // 56
  EMULATE_REACTIVE_AI_KNEE_ANGLE_DUE_TO_CT,             // 57
  EMULATE_REACTIVE_AI_SHANK_ROTATIONAL_VELOCITY,        // 58
  EMULATE_REACTIVE_AI_THIGH_ROTATIONAL_VELOCITY,        // 59
  EMULATE_REACTIVE_AI_BRAKE_SUB_PHASE_REACHED,          // 60
  EMULATE_BASE_SYSTEM_USER_WARNING,                     // 61
  EMULATE_REACTIVE_AI_DRIFT_STATUS_UNKNOWN,             // 62
  EMULATE_CONTROLLER_MODULATION_ENABLED,                // 63
  EMULATE_REACTIVE_AI_STEP_SECTION,                     // 64
  EMULATE_REACTIVE_AI_TOE_LOAD,                         // 65        
  EMULATE_SYS_OUTPUT_TORQUE,                            // 66        
  EMULATE_REACTIVE_AI_DIF_ATAN_GRAV,                    // 67
  EMULATE_TOTAL_NUM_OF_VARIABLES                        // 68
} EEmulateVariables;


#endif /* B3C_EMULATOR_CONFIG_H_*/


