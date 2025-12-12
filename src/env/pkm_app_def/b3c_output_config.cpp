
#include "B3C_HW_TOP_hw_handlers.h"
#include "bionics_const.h"
#include "b3c_output_config.h"
#include <B3C_SW_OUTPUT_MAN.h>

SOutputFunctions SOutputFuncArr[SYS_NUM_OUTPUTS]=
{
      //Init            //SetCalVal                             //SetEngineeringValue           //GetRawValue                   //GetErrorStatus        //IsIdleStatus          //ApplySetpoint                 //enable                //disable               //IsEnabled         
     {&B3C_MDB_Init,    &B3C_MDB_SetCalMotorTorqueToCurrent,    &B3C_MDB_SetTorqueSetpoint,     &B3C_MDB_GetTorqueSetpoint_Raw, NULL,                   B3C_MDB_TransfereDone,  &B3C_MDB_SetTorqueSetpoint,     B3C_MDB_Enable,         B3C_MDB_Disable,        B3C_MDB_IsEnabled}//SYS_OUTPUT_KNEE_MOTOR
}; 
                                                                                                                                                                                                                                          

SOutputConfig SOutputConfigArr [SYS_NUM_OUTPUTS] =
{//SR,      minSetpoint    maxSetpoint  delInputError          cal1          cal2       enabled         comCodeEngineering                              comCodeRaw                                              comCodeStatus        
#ifdef FAST_MOTOR
  {1,          -75,            75,              10,               0,      -175.0F,          true,           BIONICS_VAR_POWER_KNEE_ACTUATOR_SETPOINT,       BIONICS_VAR_POWER_KNEE_ACTUATOR_SETPOINT_RAW,           0  }//SYS_OUTPUT_KNEE_MOTOR -> BIONICS_VAR_KNEE_ACTUATOR_STATUS
#else
  {1,          -75,            75,            10,               0,        -150.0F,          true,           BIONICS_VAR_POWER_KNEE_ACTUATOR_SETPOINT,       BIONICS_VAR_POWER_KNEE_ACTUATOR_SETPOINT_RAW,           0  }//SYS_OUTPUT_KNEE_MOTOR -> BIONICS_VAR_KNEE_ACTUATOR_STATUS
#endif
};


