/** @file
 * @defgroup Össur Power Knee Mainstream
 * @{
 * @ingroup framework
 * @brief Database Format Definitions.
 *
 * @details
 * Declearations of the Hardware Parameters database format.
 *
 * @author Árni Einarsson
 * @date FEB 2018
 *
 * @copyright Copyright (c) 2018 Össur.\n
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval from Össur.
 */
#ifndef PKM_APP_DB_USER_PARAMS_DEF_H
#define PKM_APP_DB_USER_PARAMS_DEF_H

/* -----------------------------------------------------------------------------
Includes
---------------------------------------------------------------------------- **/
#include "PKM_APP_UserParamsDef.h"
#include <B3C_SW_DEF_DateTime.h>

#define CURRENT_USER_PARAMS_DB_VERSION  1

/* -----------------------------------------------------------------------------
Data types
---------------------------------------------------------------------------- **/

// Enumeration of the available User Parameter IDs
typedef enum _EUserParams {
        USER_PARAM_USER_WEIGHT=0,                                               /* weight                               (in kg) */		
	USER_PARAM_THIGH_LENGTH,        				        /* thighLength				(in cm) */
	USER_PARAM_SHANK_LENGTH,					        /* shankLength				(in cm) */
	USER_PARAM_FOOT_LENGTH,						        /* footLength				(in cm) */
        // USER_PARAM_SUP_SPEED_LEVEL,                                             /* SUP target velocity                  (in %) */
	USER_PARAM_PROFILE_ID,						        /* control profile ID                           */
	USER_PARAM_VIBRATION_INTENSITY,                                         /* vibrationIntensity		        (in %) */
	USER_PARAM_STANCE_FLEXION_LEVEL,                                        /* Amount of Stance Flexion in FA/FP    (in %)*/
        USER_PARAM_TOA_TORQUE_LEVEL,                                            /* % decrease ext. torque for TOA in FP (in %) */
        USER_PARAM_FLEX_ANGLE_IN_FA_FP,                                         /* Maximum flexion Angle in FA/FP       (in deg) */
	USER_PARAM_FA_FP_SPEED_LEVEL,                                           /* Thigh rot speed in Stance for FA->FP (in %) */
        USER_PARAM_FA_FP_STEP_BUFFER,                                           /* Number of Fast steps for FA->FP      (in Steps) */
	USER_PARAM_SDN_TORQUE_LEVEL,                                            /* Flex Troque Level for SDN            (in %) */
	USER_PARAM_SDN_RESISTANCE_LEVEL,                                        /* SDN resistance level			(in %) */
	USER_PARAM_SUP_ASSISTANCE_LEVEL,                                        /* SUP Assistance Level                 (in %) */
	USER_PARAM_DIS_RESISTANCE_LEVEL,					/* DIS resistance level			(in %) */
	USER_PARAM_DIS_EXT_SPEED,                                               /* DIS ext. speed in Swing		(in %) */
        USER_PARAM_RAMP_RESISTANCE_LEVEL,                                       /* Ramp resistance level                (in %) */
        USER_PARAM_GEN_ASSISTANCE_LEVEL,                                        /* Amount of assitance in GEN Stance    (in %) */
        USER_PARAM_GEN_FLEXION_ANGLE,						/* Target flexion ang in Swing in GEN   (in deg) */
	USER_PARAM_GEN_FOOT_PLACEMENT_ANGLE,                                    /* Target extension ang in Swing in GEN (in deg) */
        USER_PARAM_RAMP_TORQUE_LEVEL,                                           /* Instant Flex Troque Level for SDN    (in Nm) */
        USER_PARAM_EXERCISE_MODE,                                               /* Exercise mode                                */
        USER_PARAM_DISABLE_STAIR_ASCENT,                                        /* Disable Stair Acent                          */
        USER_PARAM_AMPUTATION_TYPE,                                             /* Amputation type                              */
        USER_PARAM_CALIBRATION_STATUS,                                          /* Calibration status of the device             */
        NB_OF_USER_PARAMS
}EUserParams;

/*------------------------------------------------------------------------------
Name:       SUserParameter
Desc:       User adjustable parameters.
------------------------------------------------------------------------------*/
typedef struct _SUserParameter
{
  int16_t  userParam[NB_OF_USER_PARAMS];
} SUserParameters;


/*------------------------------------------------------------------------------
Name:       SUserParameterDb
Desc:       User adjustable parameters database.
------------------------------------------------------------------------------*/
typedef struct _SUserParameterDb
{
    time_t           timeOfSave;       // Epoch time when database was saved to flash
    SUserParameters  userParams;
} SUserParametersDb;


/*------------------------------------------------------------------------------
Name:       SUserParamMngrInit
Desc:       Init structure for params manager.
------------------------------------------------------------------------------*/
typedef	struct	_SUserParamMngrInit
{
	uint32_t	bionicVarId;					// The bionic communication variable Id
	int16_t         minValue;                                       // The minimum value of the parameter
        int16_t         maxValue;                                       // The maximum value of the parameter
        int16_t         value;                                          // The default value of the parameter
}SUserParamInit;


/* ----------------------------------------------------------------------------
Constants
---------------------------------------------------------------------------- **/
#define USER_PARAM_USER_WEIGHT_DEFAULT_VALUE                    80
#define USER_PARAM_THIGH_LENGTH_DEFAULT_VALUE                   500
#define USER_PARAM_SHANK_LENGTH_DEFAULT_VALUE                   500
#define USER_PARAM_FOOT_LENGTH_DEFAULT_VALUE                    25
// #define USER_PARAM_SUP_SPEED_LEVEL_DEFAULT_VALUE                50
#define USER_PARAM_PROFILE_ID_DEFAULT_VALUE                     1
#define USER_PARAM_VIBRATION_INTENSITY_DEFAULT_VALUE            50
#define USER_PARAM_STANCE_FLEXION_LEVEL_DEFAULT_VALUE           50
#define USER_PARAM_TOA_TORQUE_LEVEL_DEFAULT_VALUE               100
#define USER_PARAM_FLEX_ANGLE_IN_FA_FP_DEFAULT_VALUE            60
#define USER_PARAM_FA_FP_SPEED_LEVEL_DEFAULT_VALUE              50
#define USER_PARAM_FA_FP_STEP_BUFFER_DEFAULT_VALUE              2
#define USER_PARAM_SDN_TORQUE_LEVEL_DEFAULT_VALUE               30
#define USER_PARAM_SDN_RESISTANCE_LEVEL_DEFAULT_VALUE           50
#define USER_PARAM_SUP_ASSISTANCE_LEVEL_DEFAULT_VALUE           50
#define USER_PARAM_DIS_RESISTANCE_LEVEL_DEFAULT_VALUE           50
#define USER_PARAM_DIS_EXT_SPEED_DEFAULT_VALUE                  50
#define USER_PARAM_RAMP_RESISTANCE_LEVEL_DEFAULT_VALUE          50
#define USER_PARAM_GEN_ASSISTANCE_LEVEL_DEFAULT_VALUE           40
#define USER_PARAM_GEN_FLEXION_ANGLE_DEFAULT_VALUE              70
#define USER_PARAM_GEN_FOOT_PLACEMENT_ANGLE_DEFAULT_VALUE       55
#define USER_PARAM_RAMP_TORQUE_LEVEL_DEFAULT_VALUE              30
#define USER_PARAM_EXERCISE_MODE_DEFAULT_VALUE                  0
#define USER_PARAM_DISABLE_STAIR_ASCENT_DEFAULT_VALUE           0
#define USER_PARAM_AMPUTATION_TYPE_DEFAULT_VALUE                1
#define USER_PARAM_CALIBRATION_STATUS_DEFAULT_VALUE             0

extern SUserParamInit SUserParamConf[];


#endif //PKM_APP_DB_USER_PARAMS_DEF_H
