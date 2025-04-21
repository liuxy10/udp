/** @file
 * @defgroup Össur Power Knee Mainstream
 * @{
 * @ingroup DataBases
 * @brief Database Format Definitions.
 * @details
 * Declearations of the Biometrics database format.
 * @author Árni Einarsson
 * @date Okt 2016
 *
 * @copyright Copyright (c) 2016 Össur.\n
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval from Össur.
 */
#ifndef PKM_APP_DB_BIOMETRIC_PARAMS_DEF_H
#define PKM_APP_DB_BIOMETRIC_PARAMS_DEF_H

/* -----------------------------------------------------------------------------
Includes
---------------------------------------------------------------------------- **/
#include "B3C_SW_DEF_KneeBiometricStats.h"
#include <stdint.h>
/* ----------------------------------------------------------------------------
Constants
---------------------------------------------------------------------------- **/
// The version of the different databases
#define CURRENT_BIOMETRIC_STATS_DB_VERSION 2

/* -----------------------------------------------------------------------------
Data types
---------------------------------------------------------------------------- **/
/*------------------------------------------------------------------------------
Name:       EBiomStatsDbVar
Desc:       This enum is necessary to set the values in the diagnostic application.
------------------------------------------------------------------------------*/
typedef enum _EBiomStatsDbVar
{
    BIOM_NUMBER_OF_PARAMS = B3C_KNEE_BIOM_NUMBER_OF_PARAMS
} EBiomStatsDbVar;

/*------------------------------------------------------------------------------
Name:       SBiometricStatsDb
Desc:       Biometric Data cumulated from Database Initialization (life of product)
            in the case of the Absolute Database, and from the last set point in
            the case of the Relative Database.  These statistics are updated by
            the Main Application and forwarded to the User Application in order
            to be displayed.
------------------------------------------------------------------------------*/
typedef struct _SBiometricStatsDb
{
    // B3C Biometric Stats
    SB3CKneeBiometricStats b3cBiometricStats;

    // PKM Biometric Stats
    

} SBiometricStatsDb;

#endif // PKM_APP_DB_BIOMETRIC_PARAMS_DEF_H