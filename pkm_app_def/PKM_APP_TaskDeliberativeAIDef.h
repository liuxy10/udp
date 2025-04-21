/** @file
 * @defgroup Össur Power Knee Mainstream
 * @{
 * @ingroup framework
 * @brief PKM Definitions.
 *
 * @details
 * Declearations of the Deliberatvive AI structures.
 *
 * @author Árni Einarsson
 * @date Okt 2016
 *
 * @copyright Copyright (c) 2016 Össur.\n
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval from Össur.
 */
#ifndef PKM_APP_TASKDELIBERATIVEAIDEF_H
#define PKM_APP_TASKDELIBERATIVEAIDEF_H

/* -----------------------------------------------------------------------------
Includes
---------------------------------------------------------------------------- **/


/* -----------------------------------------------------------------------------
Constants
---------------------------------------------------------------------------- **/
#define DELIB_AI_TASK_NAME        "TDelibAI"
#define DELIB_AI_TASK_PRIORITY    1
#define DELIB_AI_TASK_NB_OF_MSG   6

/* -----------------------------------------------------------------------------
Data types
---------------------------------------------------------------------------- **/

// Type of Activities selected by the Deliberation A.I.
typedef enum _ESystemActivities
{
    ACTIVITY_FUMBLING_AROUND = 0,	        // FA
    ACTIVITY_FORWARD_PROG,			// FP
    ACTIVITY_DISSIPATION,			// DIS
    ACTIVITY_GENERATION,			// GEN
    ACTIVITY_SIT_DOWN,				// SDN
    ACTIVITY_STAND_UP,				// SUP
    ACTIVITY_SIT,				// SIT
    ACTIVITY_STANDBY,				// STBY
    ACTIVITY_KNEEL,				// KNEEL
    ACTIVITY_FATAL_ERROR,			// ERROR
    NUMBER_OF_ACTIVITY,
    DB_NUMBER_OF_ACTIVITY = NUMBER_OF_ACTIVITY,
    ACTIVITY_EXERCISE = 100
}
ESystemActivities;


// Enumeration of Activity Modes
typedef enum _EActivityMode
{
    ACTIVITY_MODE_FA,    			// FA
    ACTIVITY_MODE_FP,    			// FP
    ACTIVITY_MODE_DIS,      			// DIS
    ACTIVITY_MODE_GEN,      			// GEN
    ACTIVITY_MODE_SDN,				// SDN
    ACTIVITY_MODE_SUP,				// SUP
    ACTIVITY_MODE_SIT,				// SIT
    ACTIVITY_MODE_STBY,				// STBY
    ACTIVITY_MODE_KNEEL,
    ACTIVITY_MODE_ERROR,
    ACTIVITY_MODE_NORMAL,
    ACTIVITY_MODE_EXERCISE,
    NBR_ACTIVITY_MODES
}
EActivityMode;

// Type of Activities selected by the Deliberation A.I.
typedef enum _EControlMode
{
    CONTROL_MODE_NORMAL,
    CONTROL_MODE_STUMBLE,
    CONTROL_MODE_EXT_BUMPER_ACTIVE,
    CONTROL_MODE_FLEX_BUMPER_ACTIVE,
    CONTROL_MODE_STAIR_ASCENT,
    NUMBER_OF_CONTROL_MODES
}
EControlMode;

// Enumeration of Activity Modes
typedef enum _EWalkingActivityModes
{
    WALKING_MODE_LOCKED_KNEE = 0,
    WALKING_MODE_PASSIVE_TOEOFF,
    WALKING_MODE_ACTIVE_TOEOFF,
    WALKING_MODE_PASSIVE_TOEOFF_DEFAULT_STANCE
}EWalkingActivityModes;

// Enumeration of Activity Modes
typedef enum _EWalkingSections
{
    STANCE_OR_NONSTEP_ACTIVE = 0,
    SWING_FLEXION_ACTIVE,
    SWING_EXTENSION_ACTIVE,
}EWalkingSections;


// Enumeration of Profile Ids
typedef enum _EProfileId
{
    PROFILE_ID_UNILATERAL_DYNAMIC_ACTIVE_TOA = 1,
    PROFILE_ID_UNILATERAL_DYNAMIC_PASSIVE_TOA,
    PROFILE_ID_UNILATERAL_MOBILIKNEE_MODERATE,
    PROFILE_ID_UNILATERAL_MOBILIKNEE_CONSERVATIVE,
    PROFILE_ID_UNILATERAL_MOBILIKNEE_LOCKED_KNEE,
    PROFILE_ID_BILATERAL,
    PROFILE_ID_HIP_DISC,
    PROFILE_ID_HIP_DISC_LOW_ACTIVE,
}EProfileId;

// Enumeration of Profile Ids
typedef enum _EAmputationType
{
    AMPUTATION_TYPE_UNILATERAL = 1,
    AMPUTATION_TYPE_HIP_DISC,
    AMPUTATION_TYPE_BILATERAL,
}EAmputationType;


// Enumeration of Activity Modes
typedef enum _EGenerationActivityModes
{
    GEN_MODE_NORMAL = 0,        // USER WILL BE LATCHED IN STAIR MODE UNTIL TIMEOUT OR EXIT TRIGGER. KNEE WILL NEVER ENTER BUMPER AVOIDANCE (FREE SWING)
    GEN_MODE_FREE,              // USER WILL TRANSITION FROM BRAKE TO BUMPER AVOIDANCE
}EGenerationActivityModes;

// Enumeration of Activity Modes
typedef enum _EStandUpActivityModes
{
    STANDUP_MODE_POWER_KNEE = 0,
    STANDUP_MODE_CONSTANT_FORCE,
    STANDUP_MODE_LOCK_FLEXION,
    STANDUP_MODE_LOCK_FLEXION_2,
}EStandUpActivityModes;



#endif //TASKDELIBERATIVEAIDEF_H
