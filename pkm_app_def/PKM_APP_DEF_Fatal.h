#ifndef PKM_APP_DEFFATAL_H
#define PKM_APP_DEFFATAL_H

#include "B3C_SW_DEF_Fatal.h"

#define MOTOR_ENABLE_TIMEOUT 50

#define FATAL_MOTOR_WINDING_TEMPERATURE         140                             // the cutoff limit for the motor temperture to protect the motor windings.
#define FATAL_MOTOR_WINDING_TEMP_COUNTER        50                              // How many readings above the winding temp are allowed before an error is triggered
/* -----------------------------------------------------------------------------
APP ERRORS
---------------------------------------------------------------------------- **/
// context params: Set here the variables whose values you want to store at occurance of an error.
typedef enum _EFatalContextParamType
{
    FATAL_CONTEXT_PARAM_POSITION,        // Current measured position
    FATAL_CONTEXT_PARAM_DEFLECTION,      // Current measured deflection
    FATAL_CONTEXT_PARAM_LOAD,            // Current measured load
    FATAL_CONTEXT_PARAM_DRIVE_CURRENT,   // Current measured drive current
    FATAL_CONTEXT_PARAM_BATT_VOLTAGE,    // Current measured battery voltage
    FATAL_CONTEXT_PARAM_BATT_CURRENT,    // Current measured battery current
    FATAL_CONTEXT_PARAM_BATT_SOC,        // Current measured state of charge
    FATAL_CONTEXT_PARAM_BATT_TEMP,	   // Current measured battery temperature
    FATAL_CONTEXT_PARAM_BOARD_TEMP,      // Current measured board temperature
    FATAL_CONTEXT_PARAM_MOTOR_TEMP,	   // Current measured motor temperature
    FATAL_CONTEXT_PARAM_CONTROL_MODE,    // Current ControlMode
    FATAL_CONTEXT_PARAM_SUBPHASE,        // Current subphase
    FATAL_CONTEXT_PARAM_ACTIVITY,        // Current activity
    FATAL_CONTEXT_PARAM_CADENCE,         // Current measured cadence
    FATAL_CONTEXT_PARAM_GAIN_KP,         // Current gain KP used
    FATAL_CONTEXT_PARAM_GAIN_KD,         // Current gain KD used
    FATAL_CONTEXT_PARAM_GAIN_M,          // Current gain M used
    FATAL_CONTEXT_PARAM_DESIRED_POS,     // Current desired position used
    FATAL_CONTEXT_PARAM_DESIRED_VEL     // Current desired velocity used 
} EFatalContextParamType;

#endif //#define B3C_SW_DEFFATAL_H
