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
#ifndef PKM_APP_DB_APPLICATION_STATS_DEF_H
#define PKM_APP_DB_APPLICATION_STATS_DEF_H

#define CURRENT_APPLICATION_STATS_DB_VERSION                 1

/* -----------------------------------------------------------------------------
Data types
---------------------------------------------------------------------------- **/

// Enumeration of the available User Parameter IDs
typedef enum _EApplicationStatsParams {
        APPLICATION_STATS_PEAK_TORQUE=0,                                        /* Peak recorded torque                 (in Nm)  */		
        APPLICATION_STATS_MOTOR_DIST_COVERED,                                   /* Accumulated rotation of motor        (in deg)*/
        APPLICATION_STATS_MOTOR_TEMP_WARNINGS,
        APPLICATION_STATS_MOTOR_TEMP_ERRORS,
        APPLICATION_STATS_TOA_CANCEL_THIGH,
        APPLICATION_STATS_TOA_CANCEL_TOELOAD,
        APPLICATION_STATS_RESERVED_3,
        APPLICATION_STATS_RESERVED_4,
        APPLICATION_STATS_RESERVED_5,
        APPLICATION_STATS_RESERVED_6,
        APPLICATION_STATS_RESERVED_7,
        APPLICATION_STATS_RESERVED_8,
        NB_OF_APPLICATION_STATS_PARAMS
}EApplicationStatsParams;

/*------------------------------------------------------------------------------
Name:       SApplicationStatsParameterDb
Desc:       Applicaiton statistics database
------------------------------------------------------------------------------*/
typedef struct _SApplicationStatsDb
{
  float ApplicationStat[NB_OF_APPLICATION_STATS_PARAMS];
} SApplicationStatsDb;



#endif //PKM_APP_DB_APPLICATION_STATS_DEF_H
