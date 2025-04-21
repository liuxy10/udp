/** @file
 * @defgroup Össur Power Knee Mainstream
 * @{
 * @ingroup Databases
 * @brief Database Format Definitions.
 *
 * @details
 * Declearations of the System Parameters database format.
 *
 * @author Árni Einarsson
 * @date Okt 2016
 *
 * @copyright Copyright (c) 2016 Össur.\n
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval from Össur.
*/
#ifndef PKM_APP_DB_SYSTEM_PARAMS_DEF
#define PKM_APP_DB_SYSTEM_PARAMS_DEF

/* -----------------------------------------------------------------------------
Includes
---------------------------------------------------------------------------- **/
#include "PKM_APP_ReactiveAIDef.h"
#include "PKM_APP_GainSchedulerDef.h"
#include "stdint.h"
#include "B3C_SW_DEF_DateTime.h"
/* ----------------------------------------------------------------------------
Constants
---------------------------------------------------------------------------- **/
#define USER_IDENT_SIZE				                50
#define CURRENT_SYSTEM_PARAMETERS_DB_VERSION           	        1

// Deliberative AI hardcoded parameters (not in System Database)
#define DEF_AI_MAX_CADENCE_FOR_TRANSITION			90.0F		// Maximum cadence (in steps/min) where FP to other activity transition is allowed
// FP/STUMBLE hardcoded parameters (not in System Database)
#define	DEF_AI_STUMBLE_TORQUE_THRESHOLD				50.0F		// Torque at Swing over which STUMBLE FP Mode becomes active
#define	DEF_AI_STUMBLE_MAX_CADENCE				115.0F		// Maximum cadence over which Stumble Recovery is disabled 
#define	DEF_AI_STUMBLE_KNEE_ANGLE_THRESHOLD			10.0F		// Knee angle at toe off under which STUMBLE FP mode becomes inactive
#define DEF_AI_EXT_THRESH_ANG                                   4.0F            // The minimum angle where the control parameters are overwritten to avoid full extension of the knee 
#define DEF_AI_FLEX_THRESH_ANG                                  100.0F          // The minimum angle where the control parameters are overwritten to avoid full extension of the knee 
#define	DEF_AI_STUMBLE_SETTINGS							\
{\
/*	gainKP,	gainKD,	gainM,  delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    minValue */\
   	300.0F,	4.0F,	0.0F,   0.0F,		5.0F,		        0.0F,                   0.0F,    \
}

#define	DEF_AI_EXT_BUMPER_SETTINGS							\
{\
/*	gainKP,	gainKD,	gainM,  delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    minValue */\
   	1000.0F,  5.0F,	0.0F,   0.0F,		DEF_AI_EXT_THRESH_ANG,	0.0F,                   5000.0F,    \
}

#define	DEF_AI_FLEX_BUMPER_SETTINGS							\
{\
/*	gainKP,	gainKD,	gainM,  delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    minValue */\
   	0.0F,	12.0F,	0.0F,   0.025F,		0.0F,		        0.0F,                   0.0F,    \
}

// Navigation hardcoded (not in System Database) parameter values
#define DEF_NAV_INVALID_Z_ENERGY_THRESHOLD          	        5.0F		// Threshold for invalid Z energy in m2/s2
#define DEF_NAV_INVALID_KNEE_ANGLE_THRESHOLD        	        10.0F           // Threshold for invalid knee angle delta (encoder vs gyros) in deg

/* -----------------------------------------------------------------------------
Data types
---------------------------------------------------------------------------- **/


/* -----------------------------------------------------------------------------
Name:   SSubPhaseParameters
Desc:   Structure defining parameters used for the subphase detection
        mechanism
---------------------------------------------------------------------------- **/
typedef struct _SSubPhaseParameters 
{
    bool        activityEnabled;                                                // Flag indicating that the activity is allowed.
    float    	swingToStanceThreshold;                 			// The loadcell threshold limit to transition from swing to stance
    float    	stanceToSwingThreshold;			                        // The loadcell threshold limit to transition from stance to swing
    uint16_t	phaseMinDuration;                    				// Minimum phase duration (in msec)
    uint16_t    phaseTimeoutDuration;                                           // Max time without phase transition before triggering a timeout event.    
    uint8_t     activityControlMode;                                            // specifies the control stragedy used in the activity. 0 means that the default parameters are used
    bool	toeOffAssistEnable;           		                        // Enable the Toe Off Assist
    bool   	brakeEnable;                       	                        // Enable the brake    
    float    	brakePositionTriggerMinCad;        	                        // Knee Angle Position (in deg) at minimal cadence used for brake cadence adaptation
    float    	brakePositionTriggerMaxCad;        	                        // Knee Angle Position (in deg) at maximal cadence used for brake cadence adaptation
    float    	brakeVelocityTrigger;              	                        // Knee Velocity (in deg/sec) used for the brake entering conditions
    float    	brakeExitVelTrigger;               	                        // Knee Velocity (in deg/sec) used for the brake exiting conditions
    bool   	bumperAvoidEnable;                 	                        // Enable the Bumper Avoidance
    float    	bumperAvoidPositionTriggerMinCad;  	                        // Knee Angle Position (in deg) at minimal cadence used for the Bumper Avoidance cadence adaptation
    float    	bumperAvoidPositionTriggerMaxCad;  	                        // Knee Angle Position (in deg) at maximal cadence used for the Bumper Avoidance cadence adaptation
    float    	bumperAvoidVelocityTrigger;        	                        // Knee Velocity (in deg/sec) used for the Bumper Avoidance entering
    float    	bumperAvoidExitVelTrigger;         	                        // Knee Velocity (in deg/sec) used for the Bumper Avoidance exiting
    float    	minCadenceValue;                   	                        // The minimal cadence (in steps/min) used to adapt parameters
    float    	maxCadenceValue;                   	                        // The maximal cadence (in steps/min) used to adapt parameters
    float    	reserved4;
    float    	reserved5;
} 
SSubPhaseParameters;

/* -----------------------------------------------------------------------------
Name:   SReactiveAiParameters
Desc:   Structure defining parameters used by the reactive AI for subphase
	transition detection and event generation to the deliberative AI
---------------------------------------------------------------------------- **/
typedef struct _SReactiveAiParameters
{
    SSubPhaseParameters	subPhase[DB_NUMBER_OF_ACTIVITY];			// Parameters used for subphases detection
    uint16_t            SUP2SDN_stillDurationThreshold ;                         // The time duration(msec) that the thigh or the knee velocity has to be lower than limit to detect "false standups"
    float		SDN2FP_thighAngleThresholdAtToeOff;			// The thigh angle threshold at toe off that determines if the user is seated or still standing 
    float		SDN2DIS_relThighAngleThresholdAtToeOff;			// The relative thigh angle threshold at toe off that determines if the user is going down a step
    float               DIS2FP_RelThighAngleThreshold;                             // The thigh angle threshold that user has to reach in stance to detect that he is no longer in stairs.
    float	        FA2SDN_torqueThresholdInstantanious;                    // The torque (in Nm) needed to apply to trigger SDN
    float	        FA2SDN_torqueThreshold;     				// The torque (in Nm) needed to apply to trigger SDN 
    uint16_t	        FA2SDN_torqueThresholdDuration;				// The time (in msec) that the torque must exceed SDN_torqueThreshold to trigger SDN
    float    	        FA2SDN_thighAngleThreshold;                             // The maximum thigh angle (in deg) that you can trigger SDN from.
    uint16_t            FA2SDN_triggerWindowDuration;                           // The time duration from Activity change or Heel strike that the knee can transition to SIT DOWN. 
    uint16_t	        SIT2SUP_triggerWindowDuration;                          // The time duration from Activity change or Heel strike that the knee can transition to STAND UP. 
    float    	        SIT2SUP_shankAngleThreshold;                            // The minimum shank angle (in deg) that you can trigger SUP from.
    float               SIT2SUP_thighRelativeAngleThreshold;                    // The minimum relative thigh angle (in deg) that the user must perform to trigger stand up
    float               SIT2FA_thighAngleThreshold;                             // The range from vertical in stance that the thigh can be if the knee is to transition from SIT to FA
    float               SIT2FA_kneeAngleThreshold;                              // The max knee angle in stance if the knee is to transition from SIT to FA
    float           	FA2FP_thighAngleThresholdAtToeOff;			// The Absolute Thigh Angle (in deg) at toe off to transition from FA to FP
    float            	FP2FA_thighAngleThresholdAtToeOff;		        // The Absolute Thigh Angle (in deg) at toe off to go from FP to FA
    float               SUP2FA_kneeAngleThreshold;                              // The knee angle limit (in deg) for transitioning from standing up to FA.
    float               SUP2FA_thighAngleThreshold;                             // The thigh angle limit (in deg) for transitioning from standing up to FA.
    float               FA2GEN_thighAngleThreshold;                             // The thigh angle (in deg) that the user has to reach when exerting ext. torque to enter stairs
    float               FA2GEN_kneeAngleThreshold;                              // The knee angle (in deg) that the user has to reach when exerting ext. torque to enter stairs
    float               FA2GEN_torqueThreshold;                                 // The ext. torque (in Nm) that the user has to exert to enter stairs
    uint16_t            FA2GEN_triggerWindowThreshold;                          // The time duration (in msec)from heel strike that the user has to meet the stairs up trigger
    float               GEN2FP_absThighAngleAtBaThreshold;                      // How much more (less lexed hip)the abs thigh angle must be when entering BA subphase in GEN to transition to FP
    float               SDN2DISMinKneeAngleAtToeOff;                            // The min knee angle at toe off when descending stairs.
    float               FA2SIT_thighAngleThreshold;                             // The thigh angle (in deg) threshold that the user meets to transition from FA to SIT when in SWING.
    float               FA2FP_ThighRotVelInStanceThreshold;                     // The thigh rotational speed in stance needed to transition from Fa to Fp. 
    uint8_t             FA2FP_ConsecStepsThighVelThreshold;                     // Number of steps at high speed needed to transition from FA to FP.
    uint8_t             FAFP_TOA_TriggerSensitivity;                              // The sensitivity of the ToA tigger. 
} 
SReactiveAiParameters;

/* -----------------------------------------------------------------------------
Name:   SDeliberativeAiParameters
Desc:   Structure defining parameters used by the Deliberative AI for  
	detecting activity transition at toe off and heel strike
---------------------------------------------------------------------------- **/
typedef struct _SDeliberativeAiParameters 
{
    EActivityMode	activityMode;					        // Activity mode selection forces an activity or selects the user experience level)
    uint8_t            	profileId;
    float            	reserved2;
    float            	reserved3;
    float            	reserved4;
    float            	reserved5;
} 
SDeliberativeAiParameters;

/* -----------------------------------------------------------------------------
Name:       SSchedulerParameters
Desc:       Structure defining parameters used for the scheduler mechanism
---------------------------------------------------------------------------- **/
typedef struct _SSchedulerParameters 
{
    float	        gainKP;              	                                // Proportional gain
    float   	        gainKD;                                                 // Derivative gain
    float   	        gainM;                                                  // Gain to Motor
    float   	        delay;                                                  // Delay for transition to gain (in sec)
    float   	        desiredPosition;                                        // The desired position for the controller (in deg)
    float   	        desiredVelocity;                                        // The desired velocity for the controller (in deg/sec)
    float   	        minValue;                                            // A manual minimum value.
} 
SSchedulerParameters;

/* -----------------------------------------------------------------------------
Name:       SControllerParameters
Desc:       Structure defining parameters used for the controller mechanism
---------------------------------------------------------------------------- **/
typedef struct _SControllerParameters 
{
    float	commandSaturation;		                                // Value of the saturation of the controller force (in Nm) in both directions. A negative value disables saturation.
    float   	commandModulation;		                                // Intensity of the command modulation (used during vibration)
    float   	reserved1;
    float   	reserved2;
    float   	reserved3;
    float   	reserved4;
    float   	reserved5;

} 
SControllerParameters;

/* -----------------------------------------------------------------------------
Name:       SStandbyModeParameters
Desc:       Structure defining parameters used for the standby mode mechanism
---------------------------------------------------------------------------- **/
typedef struct _SStandbyModeParameters 
{
    float	gyroThreshold;			                                // Gyro minimum value (in g) to wake up from idle mode
    float   	loadcellThreshold;		                                // Load cell minimum value to wake up from idle mode
    float   	reserved1;
    float   	reserved2;
    float   	reserved3;
    float   	reserved4;
    float   	reserved5;

} 
SStandbyModeParameters;

/* -----------------------------------------------------------------------------
Name:       SNavigationParameters
Desc:       Structure defining parameters used for the inertial navigation 
            mechanism
---------------------------------------------------------------------------- **/
typedef struct _SNavigationParameters 
{
    float   	thighLength;              	                                // The distance (in m) measured from hip to knee
    float   	shankLength;              	                                // The distance (in m) measured from floor to knee
    float   	kneeToShankIMULength;      	                                // The distance (in m) measured from knee to shank IMU sensor board
    float   	heelToShankIMULength;                                           // The distance (in m) measured from shank IMU sensor board to ankle
    float       footSize;                                                       // The size of the foot.
    float   	userWeight;                                                     // weight of the user in kg.
    float   	reserved2;
    float   	reserved3;
    float   	reserved4;
    float   	reserved5;
}
SNavigationParameters;

/* -----------------------------------------------------------------------------
Name:       SSystemParameters
Desc:       Structure defining parameters used for the inertial navigation 
            mechanism.  The default values are defined by SYS_PARAMS_INIT_TBL.
---------------------------------------------------------------------------- **/
typedef struct _SSystemParameters 
{
    SReactiveAiParameters  		reactiveAI;
    SDeliberativeAiParameters           deliberativeAI;
    SSchedulerParameters   		gainScheduler[NB_OF_SUBPHASES][DB_NUMBER_OF_ACTIVITY][NB_OF_ADAPTATION_POINTS];
    SControllerParameters  		controller;
    SStandbyModeParameters 		standbyMode;
    SNavigationParameters  		navigation;
}
SSystemParameters;

// Constant define to be used when creating the table of SSystemParameters defaults
// Example: const SSystemParameters CParamsManager::smSysParamsInitTbl = SYS_PARAMS_INIT_TBL;
#define	SYS_PARAMS_INIT_TBL	\
{\
	/* SReactiveAiParameters  reactiveAI */\
	{\
          /* SSubPhaseParameters	subPhase[ACTIVITY_FUMBLING_AROUND] */\
          true,                                                                 /* bool         activityEnabled*/\
          50.0F,                                                                /* float        swingToStanceThreshold*/\
          30.0F,                                    	                        /* float        stanceToSwingThreshold*/\
          200,                                                                   /* uint16_t     phaseMinDuration*/\
          1500,                                                                 /* uint16_t     phaseTimeoutDuration*/\
          0,                                                                    /* uint8_t      activityControlMode*/\
          true,								        /* bool	        toeOffAssistEnable */\
          true,								        /* bool	        brakeEnable */\
          45.0F,								/* float	brakePositionTriggerMinCad              (in deg) */\
          60.0F,								/* float	brakePositionTriggerMaxCad              (in deg) */\
          50.0F,								/* float	brakeVelocityTrigger                    (in deg/sec) */\
          -140.0F,							        /* float	brakeExitVelTrigger                     (in deg/sec) */\
          true,								        /* bool	        bumperAvoidEnable */\
          18.0F,								/* float	bumperAvoidPositionTriggerMinCad        (in deg) */\
          22.0F,								/* float	bumperAvoidPositionTriggerMaxCad        (in deg) */\
          -70.0F,							        /* float	bumperAvoidVelocityTrigger              (in deg/sec) */\
          100.0F,								/* float	bumperAvoidExitVelTrigger               (in deg/sec) */\
          60.0F,								/* float	minCadenceValue				(in steps/min) */\
          115.0F,								/* float	maxCadenceValue				(in steps/min) */\
          -60.0F,								/* float        reserved4 */\
          50.0F,								/* float        reserved5 */\
\
          /* SSubPhaseParameters	subPhase[ACTIVITY_FORWARD_PROG] */\
          true,                                                                 /* bool         activityEnabled*/\
          50.0F,                                                                /* float        swingToStanceThreshold*/\
          30.0F,                                    	                        /* float        stanceToSwingThreshold*/\
          200,                                                                  /* uint16_t     phaseMinDuration*/\
          1350,                                                                 /* uint16_t     phaseTimeoutDuration*/\
          0,                                                                    /* uint8_t      controlMode*/\
          true,								        /* bool	        toeOffAssistEnable */\
          true,								        /* bool	        brakeEnable */\
          40.0F,								/* float	brakePositionTriggerMinCad              (in deg) */\
          55.0F,								/* float	brakePositionTriggerMaxCad              (in deg) */\
          50.0F,								/* float	brakeVelocityTrigger                    (in deg/sec) */\
          -140.0F,							        /* float	brakeExitVelTrigger                     (in deg/sec) */\
          true,								        /* bool	        bumperAvoidEnable */\
          17.0F,								/* float	bumperAvoidPositionTriggerMinCad        (in deg) */\
          20.0F,								/* float	bumperAvoidPositionTriggerMaxCad        (in deg) */\
          -50.0F,								/* float	bumperAvoidVelocityTrigger              (in deg/sec) */\
          999.0F,								/* float	bumperAvoidExitVelTrigger               (in deg/sec) */\
          60.0F,								/* float	minCadenceValue */\
          120.0F,								/* float	maxCadenceValue */\
          -60.0F,  							        /* float        reserved4 */\
          50.0F,							        /* float        reserved5 */\
\
          /* SSubPhaseParameters	subPhase[ACTIVITY_DISSIPATION] */\
          true,                                                                 /* bool         activityEnabled*/\
          50.0F,                                                                /* float        swingToStanceThreshold*/\
          30.0F,                                    	                        /* float        stanceToSwingThreshold*/\
          50,                                                                   /* uint16_t     phaseMinDuration*/\
          2000,                                                                 /* uint16_t     phaseTimeoutDuration*/\
          0,                                                                    /* uint8_t      activityControlMode*/\
          false,								/* bool	        toeOffAssistEnable */\
          false,							        /* bool	        brakeEnable */\
          55.0F,								/* float	brakePositionTriggerMinCad              (in deg) */\
          55.0F,								/* float	brakePositionTriggerMaxCad              (in deg) */\
          5.0F,		        						/* float	brakeVelocityTrigger                    (in deg/sec) */\
          1.0F,	        							/* float	brakeExitVelTrigger                     (in deg/sec) */\
          true, 								/* bool	        bumperAvoidEnable */\
          20.0F,								/* float	bumperAvoidPositionTriggerMinCad        (in deg) */\
          20.0F,								/* float	bumperAvoidPositionTriggerMaxCad        (in deg) */\
          -1.0F,								/* float	bumperAvoidVelocityTrigger              (in deg/sec) */\
          200.0F,								/* float	bumperAvoidExitVelTrigger               (in deg/sec) */\
          60.0F,								/* float	minCadenceValue */\
          115.0F,								/* float	maxCadenceValue */\
          0.0F,					        			/* float        reserved4 */\
          0.0F,						        		/* float        reserved5 */\
\
          /* SSubPhaseParameters	subPhase[ACTIVITY_GENERATION] */\
          true,                                                                 /* bool         activityEnabled*/\
          40.0F,                                                                /* float        swingToStanceThreshold*/\
          25.0F,                                    	                        /* float        stanceToSwingThreshold*/\
          350,                                                                   /* uint16_t     phaseMinDuration*/\
          2500,                                                                 /* uint16_t     phaseTimeoutDuration*/\
          0,                                                                    /* uint8_t      activityControlMode*/\
          true, 							        /* bool	        toeOffAssistEnable */\
          false,								/* bool	        brakeEnable */\
          50.0F,								/* float	brakePositionTriggerMinCad		(in deg) */\
          35.0F,								/* float	brakePositionTriggerMaxCad		(in deg) */\
          10.0F,								/* float	brakeVelocityTrigger			(in deg/sec) */\
          0.0F,								        /* float	brakeExitVelTrigger			(in deg/sec) */\
          true,								        /* bool	        bumperAvoidEnable */\
          105.0F,								/* float	bumperAvoidPositionTriggerMinCad	(in deg) */\
          105.0F,								/* float	bumperAvoidPositionTriggerMaxCad	(in deg) */\
          -70.0F,								/* float	bumperAvoidVelocityTrigger		(in deg/sec) */\
          200.0F,								/* float	bumperAvoidExitVelTrigger		(in deg/sec) */\
          40.0F,								/* float	minCadenceValue */\
          90.0F,								/* float	maxCadenceValue */\
          0.0F,	        							/* float        reserved4 */\
          0.0F, 								/* float        reserved5 */\
\
          /* SSubPhaseParameters	subPhase[ACTIVITY_SIT_DOWN] */\
          true,                                                                 /* bool         activityEnabled*/\
          50.0F,                                                                /* float        swingToStanceThreshold*/\
          30.0F,                                    	                        /* float        stanceToSwingThreshold*/\
          50,                                                                   /* uint16_t     phaseMinDuration*/\
          3000,                                                                 /* uint16_t     phaseTimeoutDuration*/\
          0,                                                                    /* uint8_t      activityControlMode*/\
          true,								        /* bool	        toeOffAssistEnable */\
          false,								/* bool	        brakeEnable */\
          120.0F,								/* float	brakePositionTriggerMinCad              (in deg) */\
          120.0F,								/* float	brakePositionTriggerMaxCad              (in deg) */\
          100.0F,								/* float	brakeVelocityTrigger                    (in deg/sec) */\
          0.0F,								        /* float	brakeExitVelTrigger                     (in deg/sec) */\
          false,								/* bool	        bumperAvoidEnable */\
          8.0F,								        /* float	bumperAvoidPositionTriggerMinCad        (in deg) */\
          8.0F,			        					/* float	bumperAvoidPositionTriggerMaxCad        (in deg) */\
          -100.0F,			        				/* float	bumperAvoidVelocityTrigger              (in deg/sec) */\
          0.0F,					        			/* float	bumperAvoidExitVelTrigger               (in deg/sec) */\
          60.0F,								/* float	minCadenceValue				(in steps/min) */\
          115.0F,								/* float	maxCadenceValue				(in steps/min) */\
          0.0F,	        							/* float        reserved4 */\
          0.0F,		        						/* float        reserved5 */\
\
          /* SSubPhaseParameters	subPhase[ACTIVITY_STAND_UP] */\
          true,                                                                 /* bool         activityEnabled*/\
          50.0F,                                                                /* float        swingToStanceThreshold*/\
          30.0F,                                    	                        /* float        stanceToSwingThreshold*/\
          50,                                                                   /* uint16_t     phaseMinDuration*/\
          3000,                                                                 /* uint16_t     phaseTimeoutDuration*/\
          0,                                                                    /* uint8_t      activityControlMode*/\
          true, 	        						/* bool	        toeOffAssistEnable */\
          false,								/* bool 	brakeEnable */\
          120.0F,								/* float	brakePositionTriggerMinCad              (in deg) */\
          120.0F,								/* float	brakePositionTriggerMaxCad              (in deg) */\
          100.0F,								/* float	brakeVelocityTrigger                    (in deg/sec) */\
          0.0F,							        	/* float	brakeExitVelTrigger                     (in deg/sec) */\
          true,  								/* bool	        bumperAvoidEnable */\
          15.0F,							        /* float	bumperAvoidPositionTriggerMinCad        (in deg) */\
          15.0F,	        						/* float	bumperAvoidPositionTriggerMaxCad        (in deg) */\
          100.0F, 		        					/* float	bumperAvoidVelocityTrigger              (in deg/sec) */\
          0.0F,			        				/* float	bumperAvoidExitVelTrigger               (in deg/sec) */\
          60.0F,								/* float	minCadenceValue				(in steps/min) */\
          115.0F,								/* float	maxCadenceValue				(in steps/min) */\
          0.0F,                                                                /* float        reserved4 */\
          0.0F,								        /* float        reserved5 */\
\
          /* SSubPhaseParameters	subPhase[ACTIVITY_SIT] */\
          true,                                                                 /* bool         activityEnabled*/\
          30.0F,                                                                /* float        swingToStanceThreshold*/\
          20.0F,                                    	                        /* float        stanceToSwingThreshold*/\
          50,                                                                   /* uint16_t     phaseMinDuration*/\
          1500,                                                                 /* uint16_t     phaseTimeoutDuration*/\
          0,                                                                    /* uint8_t      activityControlMode*/\
          false,								/* bool	        toeOffAssistEnable */\
          false,								/* bool	        brakeEnable */\
          120.0F,								/* float	brakePositionTriggerMinCad              (in deg) */\
          120.0F,								/* float	brakePositionTriggerMaxCad              (in deg) */\
          100.0F,								/* float	brakeVelocityTrigger                    (in deg/sec) */\
          0.0F  ,								/* float	brakeExitVelTrigger                     (in deg/sec) */\
          false,								/* bool	        bumperAvoidEnable */\
          8.0F,	        							/* float	bumperAvoidPositionTriggerMinCad        (in deg) */\
          8.0F,		        						/* float	bumperAvoidPositionTriggerMaxCad        (in deg) */\
          -100.0F,		        					/* float	bumperAvoidVelocityTrigger              (in deg/sec) */\
          0.0F,				        				/* float	bumperAvoidExitVelTrigger               (in deg/sec) */\
          60.0F,								/* float	minCadenceValue				(in steps/min) */\
          115.0F,								/* float	maxCadenceValue				(in steps/min) */\
          0.0F,								        /* float        reserved4 */\
          0.0F,							        	/* float        reserved5 */\
\
          /* SSubPhaseParameters	subPhase[ACTIVITY_STANDBY] */\
          true,                                                                 /* bool         activityEnabled*/\
          50.0F,                                                                /* float        swingToStanceThreshold*/\
          30.0F,                                    	                        /* float        stanceToSwingThreshold*/\
          50,                                                                   /* uint16_t     phaseMinDuration*/\
          1500,                                                                 /* uint16_t     phaseTimeoutDuration*/\
          0,                                                                    /* uint8_t      activityControlMode*/\
          false,								/* bool	        toeOffAssistEnable */\
          false,								/* bool	        brakeEnable */\
          30.0F,								/* float	brakePositionTriggerMinCad              (in deg) */\
          30.0F,								/* float	brakePositionTriggerMaxCad              (in deg) */\
          100.0F,								/* float	brakeVelocityTrigger                    (in deg/sec) */\
          0.0F, 								/* float	brakeExitVelTrigger                     (in deg/sec) */\
          false,								/* bool	        bumperAvoidEnable */\
          40.0F,								/* float	bumperAvoidPositionTriggerMinCad        (in deg) */\
          40.0F,								/* float	bumperAvoidPositionTriggerMaxCad        (in deg) */\
          -100.0F,      							/* float	bumperAvoidVelocityTrigger              (in deg/sec) */\
          20.0F,								/* float	bumperAvoidExitVelTrigger               (in deg/sec) */\
          60.0F,								/* float	minCadenceValue				(in steps/min) */\
          115.0F,								/* float	maxCadenceValue				(in steps/min) */\
          0.0F,					        			/* float        reserved4 */\
          0.0F,						        		/* float        reserved5 */\
\
          /* SSubPhaseParameters	subPhase[ACTIVITY_KNEEL] */\
          true,                                                                 /* bool         activityEnabled*/\
          50.0F,                                                                /* float        swingToStanceThreshold*/\
          30.0F,                                    	                        /* float        stanceToSwingThreshold*/\
          50,                                                                   /* uint16_t     phaseMinDuration*/\
          1500,                                                                 /* uint16_t     phaseTimeoutDuration*/\
          0,                                                                    /* uint8_t      activityControlMode*/\
          false,								/* bool	        toeOffAssistEnable */\
          false,							        /* bool	        brakeEnable */\
          115.0F,								/* float	brakePositionTriggerMinCad              (in deg) */\
          115.0F,								/* float	brakePositionTriggerMaxCad              (in deg) */\
          150.0F,								/* float	brakeVelocityTrigger                    (in deg/sec) */\
          0.0F,				        				/* float	brakeExitVelTrigger                     (in deg/sec) */\
          false,				        			/* bool	        bumperAvoidEnable */\
          5.0F,						        		/* float	bumperAvoidPositionTriggerMinCad        (in deg) */\
          5.0F,							        	/* float	bumperAvoidPositionTriggerMaxCad        (in deg) */\
          -250.0F,							        /* float	bumperAvoidVelocityTrigger              (in deg/sec) */\
          0.0F, 								/* float	bumperAvoidExitVelTrigger               (in deg/sec) */\
          60.0F,								/* float	minCadenceValue				(in steps/min) */\
          115.0F,								/* float	maxCadenceValue				(in steps/min) */\
          0.0F,				        				/* float        reserved4 */\
          0.0F,					        			/* float        reserved5 */\
\
          /* SSubPhaseParameters	subPhase[ACTIVITY_FATAL_ERROR] */\
          true,                                                                 /* bool         activityEnabled*/\
          50.0F,                                                                /* float        swingToStanceThreshold*/\
          30.0F,                                    	                        /* float        stanceToSwingThreshold*/\
          50,                                                                   /* uint16_t     phaseMinDuration*/\
          1500,                                                                 /* uint16_t     phaseTimeoutDuration*/\
          0,                                                                    /* uint8_t      activityControlMode*/\
          false,								/* bool	        toeOffAssistEnable */\
          false,								/* bool	        brakeEnable */\
          30.0F,								/* float	brakePositionTriggerMinCad              (in deg) */\
          30.0F,								/* float	brakePositionTriggerMaxCad              (in deg) */\
          100.0F,								/* float	brakeVelocityTrigger                    (in deg/sec) */\
          0.0F,								        /* float	brakeExitVelTrigger                     (in deg/sec) */\
          false,								/* bool	        bumperAvoidEnable */\
          40.0F,								/* float	bumperAvoidPositionTriggerMinCad        (in deg) */\
          40.0F,								/* float	bumperAvoidPositionTriggerMaxCad        (in deg) */\
          -100.0F,						        	/* float	bumperAvoidVelocityTrigger              (in deg/sec) */\
          20.0F,								/* float	bumperAvoidExitVelTrigger               (in deg/sec) */\
          60.0F,								/* float	minCadenceValue				(in steps/min) */\
          115.0F,								/* float	maxCadenceValue				(in steps/min) */\
          0.0F,							        	/* float        reserved4 */\
          0.0F,							        	/* float        reserved5 */\
\
          50,                                                                   /* uint16_t    	SUP2SDN_stillDurationThreshold          (in msec)*/\
          -50.0F,								/* float	SDN2FP_thighAngleThresholdAtToeOff      (in negative deg) */\
          5.0F,		        					        /* float	SDN2DIS_relThighAngleThresholdAtToeOff  (in negative deg) */\
          20.0F,							        /* float	DIS2FP_RelThighAngleThreshold           (in negative deg) */\
          40.0F,                                                                /* float	FA2SDN_torqueThresholdInstantanious     (in Nm) */\
          40.0F,                                                                /* float	FA2SDN_torqueThreshold                  (in Nm) */\
          400,                                                                  /* uint16_t	FA2SDN_torqueThresholdDuration		(in msec) */\
          0,                                                                    /* float    	FA2SDN_thighAngleThreshold              (in deg) */\
          3000,                                                                 /* uint16_t    	FA2SDN_triggerWindowDuration            (in msec)*/\
          2000,                                                                 /* uint16_t     SIT2SUP_triggerWindowDuration           (in msec)*/\
          -5.0,                                                                 /* float    	SIT2SUP_shankAngleThreshold             (in deg) */\
          10.0,                                                                 /* float    	SIT2SUP_thighRelativeAngleThreshold     (in deg) */\
          30.0,                                                                 /* float    	SIT2FA_thighAngleThreshold              (in deg) */\
          15.0,                                                                 /* float    	SIT2FA_kneeAngleThreshold               (in deg) */\
          3.0,                                                                  /* float        FA2FP_thighAngleThresholdAtToeOff	(in deg) */\
          3.0,                                                                  /* float        FP2FA_thighAngleThresholdAtToeOff	(in deg) */\
          15.0,                                                                  /* float        SUP2FA_kneeAngleThreshold		(in deg) */\
          30.0,                                                                 /* float        SUP2FA_thighAngleThreshold		(in deg) */\
          -30.0F,                                                               /* float        FA2GEN_thighAngleThreshold              (in deg) */\
          35.0F,                                                                /* float        FA2GEN_kneeAngleThreshold               (in deg) */\
          -3.0F,                                                                /* float        FA2GEN_torqueThreshold                  (in Nm) */\
          2000,                                                                 /* uint16_t     FA2GEN_triggerWindowThreshold           (in mSec) */\
          20.0F,                                                                /* float        GEN2FP_absThighAngleAtBaThreshold       (in deg) */\
          30.0F,								/* float        SDN2DISMinKneeAngleAtToeOff             (in deg) */\
          -50.0F,								/* float        FA2SIT_thighAngleThreshold              (in deg) */\
          45.0F,								/* float        FA2FP_ThighRotVelInStanceThreshold      (in deg/sec) */\
          2,    								/* uint8_t      FA2FP_ConsecStepsThighVelThreshold      (steps) */\
          0,    								/* uint8_t      FAFP_TOA_TriggerSensitivity             (%) */\
    },\
\
    /* SDeliberativeAiParameters    deliberativeAI */\
    {\
    	ACTIVITY_MODE_NORMAL,                                                   /* EActivityMode activityMode */\
    	PROFILE_ID_UNILATERAL_DYNAMIC_ACTIVE_TOA,                               /* uint8_t        profileId */\
    	0.0F,		        						/* float        reserved2 */\
    	0.0F,		              						/* float        reserved3 */\
    	0.0F,		        						/* float        reserved4 */\
    	0.0F,		        						/* float        reserved5 */\
    },\
\
	/* SSchedulerParameters   gainScheduler[][][]	Note: all values are float */\
	{\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_REJECTION][ACTIVITY_FUMBLING_AROUND][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	0.0F,		-60.0F,	        0.0F,		0.0F,			0.0F,                   0.0F,           /* MIN_CADENCE */\
    	0.0F,	0.0F,		-60.0F,	        0.0F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_REJECTION][ACTIVITY_FORWARD_PROG][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	0.0F,		-60.0F,	        0.0F,		0.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	0.0F,		-60.0F,	        0.0F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_REJECTION][ACTIVITY_DISSIPATION][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	20.0F,		-50.0F,	        0.05F,		0.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	20.0F,		-50.0F,	        0.05F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_REJECTION][ACTIVITY_GENERATION][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	300.0F,	5.0F,		-10.0F,		0.010F,		6.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	300.0F,	5.0F,		-10.0F,		0.005F,		6.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_REJECTION][ACTIVITY_SIT_DOWN][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	20.0F,	        -50.0F,	        0.10F,		0.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	20.0F,		-50.0F,	        0.10F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_REJECTION][ACTIVITY_STAND_UP][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	300.0F,	0.0F,		0.0F,		0.1F,		6.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	300.0F,	0.0F,		0.0F,		0.1F,		6.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_REJECTION][ACTIVITY_SIT][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	0.0F,		300.0F,		0.1F,		40.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	0.0F,		300.0F,		0.1F,		40.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_REJECTION][ACTIVITY_STANDBY][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	0.0F,		300.0F,		0.0F,		0.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	0.0F,		300.0F,		0.0F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_REJECTION][ACTIVITY_KNEEL][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	10.0F,		300.0F,		0.005F,		0.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	10.0F,		300.0F,		0.005F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_REJECTION][ACTIVITY_FATAL_ERROR][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	0.0F,		0.0F,	        0.0F,		0.0F,			0.0F,                   -500.0F,           /* MIN_CADENCE */\
    	0.0F,	0.0F,		0.0F,	        0.0F,		0.0F,			0.0F,                   -500.0F,           /* MAX_CADENCE */\
\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_TOE_OFF_ASSIST][ACTIVITY_FUMBLING_AROUND][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	0.0F,		400.0F,		0.005F,		0.0F,			0.0F,                0.0F,              /* MIN_CADENCE */\
    	0.0F,	0.0F,		400.0F,		0.005F,		0.0F,			0.0F,                0.0F,              /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_TOE_OFF_ASSIST][ACTIVITY_FORWARD_PROG][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	40.0F,	0.1F,		0.0F,		0.005F,		90.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	60.0F,	0.1F,		0.0F,		0.005F,		90.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_TOE_OFF_ASSIST][ACTIVITY_DISSIPATION][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	0.0F,		0.0F,		0.0F,		0.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	0.0F,		0.0F,		0.0F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_TOE_OFF_ASSIST][ACTIVITY_GENERATION][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	100.0F,	40.0F,		-80.0F,		0.0F,		8.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	100.0F,	40.0F,		-80.0F,		0.0F,		8.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_TOE_OFF_ASSIST][ACTIVITY_SIT_DOWN][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	40.0F,	0.1F,		0.0F,		0.005F,		90.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	60.0F,	0.1F,		0.0F,		0.005F,		90.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_TOE_OFF_ASSIST][ACTIVITY_STAND_UP][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	40.0F,		0.0F,		0.1F,		8.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	40.0F,		0.0F,		0.1F,		8.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_TOE_OFF_ASSIST][ACTIVITY_SIT][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	20.0F,	1.0F,		0.0F,		0.005F,		90.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	20.0F,	1.0F,		0.0F,		0.005F,		90.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_TOE_OFF_ASSIST][ACTIVITY_STANDBY][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	0.0F,		0.0F,		0.0F,		0.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	0.0F,		0.0F,		0.0F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_TOE_OFF_ASSIST][ACTIVITY_KNEEL][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	5.0F,		0.0F,		0.005F,		0.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	5.0F,		0.0F,		0.005F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_TOE_OFF_ASSIST][ACTIVITY_FATAL_ERROR][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
        0.0F,	0.0F,		-81.0F,	        0.0F,		0.0F,			0.0F,                   0.0F,           /* MIN_CADENCE */\
    	0.0F,	0.0F,		-81.0F,	        0.0F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BUMPER_AVOIDANCE][ACTIVITY_FUMBLING_AROUND][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	100.0F,	12.0F,		0.0F,		0.015F,		8.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	175.0F,	15.0F,		0.0F,		0.015F,		12.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BUMPER_AVOIDANCE][ACTIVITY_FORWARD_PROG][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	100.0F,	12.0F,		0.0F,		0.005F,		8.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	175.0F,	15.0F,		0.0F,		0.005F,		12.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BUMPER_AVOIDANCE][ACTIVITY_DISSIPATION][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	250.0F,	10.0F,		-91.0F,	        0.0F,		8.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	250.0F,	10.0F,		-91.0F,	        0.0F,		8.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BUMPER_AVOIDANCE][ACTIVITY_GENERATION][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	100.0F,	8.0F,		0.0F,		0.0F,		60.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	100.0F,	8.0F,		0.0F,		0.0F,		60.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BUMPER_AVOIDANCE][ACTIVITY_SIT_DOWN][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	250.0F,	20.0F,		0.0F,		0.015F,		8.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	250.0F,	20.0F,		0.0F,		0.015F,		8.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BUMPER_AVOIDANCE][ACTIVITY_STAND_UP][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	250.0F,	20.0F,		0.0F,		0.015F,		8.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	250.0F,	20.0F,		0.0F,		0.015F,		8.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BUMPER_AVOIDANCE][ACTIVITY_SIT][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	250.0F,	20.0F,		0.0F,		0.015F,		10.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	250.0F,	20.0F,		0.0F,		0.015F,		10.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BUMPER_AVOIDANCE][ACTIVITY_STANDBY][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	0.0F,		0.0F,		0.0F,		0.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	0.0F,		0.0F,		0.0F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BUMPER_AVOIDANCE][ACTIVITY_KNEEL][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	30.0F,		0.0F,		0.005F,		10.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	30.0F,		0.0F,		0.005F,		10.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BUMPER_AVOIDANCE][ACTIVITY_FATAL_ERROR][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	0.0F,		-91.0F,	        0.0F,		0.0F,			0.0F,                   0.0F,           /* MIN_CADENCE */\
    	0.0F,	0.0F,		-91.0F,	        0.0F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_FOLLOWING][ACTIVITY_FUMBLING_AROUND][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	0.0F,		400.0F,		0.1F,		0.0F,			0.0F,                   0.0F,           /* MIN_CADENCE */\
    	0.0F,	0.0F,		400.0F,		0.1F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_FOLLOWING][ACTIVITY_FORWARD_PROG][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	0.0F,		400.0F,		0.05F,		0.0F,			0.0F,                   0.0F,           /* MIN_CADENCE */\
    	0.0F,	0.0F,		400.0F,		0.2F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_FOLLOWING][ACTIVITY_DISSIPATION][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	30.0F,	0.0F,		0.0F,		0.0F,		10.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	30.0F,	0.0F,		0.0F,		0.0F,		10.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_FOLLOWING][ACTIVITY_GENERATION][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	125.0F,	0.0F,		0.0F,		0.0F,		90.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	125.0F,	0.0F,		0.0F,		0.0F,		90.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_FOLLOWING][ACTIVITY_SIT_DOWN][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	3.0F,		200.0F,		0.1F,		40.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	3.0F,		200.0F,		0.1F,		40.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_FOLLOWING][ACTIVITY_STAND_UP][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	300.0F,	0.0F,		0.0F,		0.1F,		6.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	300.0F,	0.0F,		0.0F,		0.1F,		6.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_FOLLOWING][ACTIVITY_SIT][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	0.0F,		200.0F,		0.1F,		40.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	0.0F,		200.0F,		0.1F,		40.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_FOLLOWING][ACTIVITY_STANDBY][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	0.0F,		300.0F,		0.0F,		0.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	0.0F,		300.0F,		0.0F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_FOLLOWING][ACTIVITY_KNEEL][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	3.0F,		300.0F,		0.250F,		0.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	3.0F,		300.0F,		0.250F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_FORCE_FOLLOWING][ACTIVITY_FATAL_ERROR][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	0.0F,		0.0F,	        0.0F,		0.0F,			0.0F,                   -500.0F,           /* MIN_CADENCE */\
    	0.0F,	0.0F,		0.0F,	        0.0F,		0.0F,			0.0F,                   -500.0F,           /* MAX_CADENCE */\
\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BRAKE][ACTIVITY_FUMBLING_AROUND][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	12.0F,		0.0F,		0.150F,		0.0F,			-250.0F,                0.0F,           /* MIN_CADENCE */\
    	0.0F,	14.0F,		0.0F,		0.050F,		0.0F,			-400.0F,                0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BRAKE][ACTIVITY_FORWARD_PROG][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	17.0F,		0.0F,		0.150F,		0.0F,			-250.0F,                0.0F,           /* MIN_CADENCE */\
    	0.0F,	20.0F,		0.0F,		0.050F,		0.0F,			-300.0F,                0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BRAKE][ACTIVITY_DISSIPATION][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	2.0F,		0.0F,		0.0F,		0.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	2.0F,		0.0F,		0.0F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BRAKE][ACTIVITY_GENERATION][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	75.0F,	4.0F,		0.0F,		0.0F,		60.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	75.0F,	4.0F,		0.0F,		0.0F,		60.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BRAKE][ACTIVITY_SIT_DOWN][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	6.0F,		0.0F,		0.050F,		0.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	6.0F,		0.0F,		0.050F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BRAKE][ACTIVITY_STAND_UP][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	6.0F,		0.0F,		0.050F,		0.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	6.0F,		0.0F,		0.050F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BRAKE][ACTIVITY_SIT][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	6.0F,		0.0F,		0.050F,		0.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	6.0F,		0.0F,		0.050F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BRAKE][ACTIVITY_STANDBY][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	0.0F,		0.0F,		0.0F,		0.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	0.0F,		0.0F,		0.0F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BRAKE][ACTIVITY_KNEEL][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	30.0F,		0.0F,		0.005F,		0.0F,			0.0F,                   0.0F,		/* MIN_CADENCE */\
    	0.0F,	30.0F,		0.0F,		0.005F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    /* SSchedulerParameters   gainScheduler[SUBPHASE_BRAKE][ACTIVITY_FATAL_ERROR][] */\
    /*	gainKP,  gainKD,        gainM,          delay(sec),     Pos_target(deg),        Vel_target(deg/sec),    MinValue */\
    	0.0F,	0.0F,		-71.0F,	        0.0F,		0.0F,			0.0F,                   0.0F,           /* MIN_CADENCE */\
    	0.0F,	0.0F,		-71.0F,	        0.0F,		0.0F,			0.0F,                   0.0F,           /* MAX_CADENCE */\
    },\
\
    /* SControllerParameters  controller */\
    {\
    	100.0F,								        /* float	commandSaturation			(in Nm) */\
    	20.0F,								        /* float        commandModulation			(in Nm) */\
    	0.0F,								        /* float        reserved1 */\
    	0.0F,								        /* float        reserved2 */\
    	0.0F,								        /* float        reserved3 */\
    	0.0F,								        /* float        reserved4 */\
        0.0F								        /* float        reserved5 */\
    },\
\
    /* SStandbyModeParameters standbyMode */\
    {\
    	0.5F,								        /* float	gyroThreshold				(in g) */\
    	500.0F,								        /* float	loadcellThreshold */\
    	0.0F,								        /* float	reserved1 */\
    	0.0F,								        /* float        reserved2 */\
    	0.0F,								        /* float        reserved3 */\
    	0.0F,								        /* float        reserved4 */\
        0.0F								        /* float        reserved5 */\
    },\
\
    /* SNavigationParameters  navigation */\
    {\
    	0.42F,								        /* float        thighLength 				(in m) */\
        0.50F,								        /* float        thighLength 				(in m) */\
    	0.215F,								        /* float        kneeToShankIMULength 		        (in m) */\
    	0.305F,								        /* float        heelToShankIMULength 		        (in m) */\
    	0.27F,								        /* float        footSize 				(in m) */\
    	80.0F,								        /* float        userWeight                              (in kg)*/\
    	0.0F,			        					/* float        reserved2 */\
    	0.0F,		        						/* float        reserved3 */\
    	0.0F,	        							/* float        reserved4 */\
    	0.0F       								/* float        reserved5 */\
    }\
}
#endif // PKM_APP_DB_SYSTEM_PARAMS_DEF
