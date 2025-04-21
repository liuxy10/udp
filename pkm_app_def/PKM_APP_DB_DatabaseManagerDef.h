/** @file
 * @defgroup OB3C Össur Bionic Common Core Components
 * @{
 * @ingroup framework
 * @brief Database Definitions.
 *
 * @details
 * Various definitions of the databases in the system. Here the developer can set
 * the size of the defined databases, their location on the flash, etc. 
 *
 * @author Árni Einarsson
 * @date Okt 2016
 *
 * @copyright Copyright (c) 2016 Össur.\n
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval from Össur.
 */
#ifndef PKM_APP_DB_DATABASE_MANAGER_DEF_H
#define PKM_APP_DB_DATABASE_MANAGER_DEF_H

/* -----------------------------------------------------------------------------
Includes
---------------------------------------------------------------------------- **/

#include "B3C_SW_DB_Database.h"
#include "B3C_SW_DB_DatabaseManagerDef.h"
#include "PKM_APP_DB_BiometricParamsDef.h"
#include "PKM_APP_DB_HardwareParamsDef.h"
#include "PKM_APP_DB_SystemParamsDef.h"
#include "PKM_APP_DB_UserParamsDef.h"
#include "PKM_APP_DB_ApplicationStatsDef.h"

/* ----------------------------------------------------------------------------
Constants
---------------------------------------------------------------------------- **/

/* ----------------------------------------------------------------------------
 ENTER HERE THE SIZE RESERVED FOR THE DATABASES DEFINED IN THE APPLICATION.  
THE SIZE SHOULD EXCEED THE DEFINED SIZE OF THE DATABASE + HEADER+FOOTER
 ----------------------------------------------------------------------------*/
#define DB_ABS_BIOM_STATS_MAX_SIZE        0x200
#define DB_SYSTEM_PARAMS_MAX_SIZE         0x1000
#define DB_HW_PARAMS_MAX_SIZE             0x100
#define DB_USER_PARAMS_MAX_SIZE           0x100
#define DB_REL_BIOM_STATS_MAX_SIZE        0x200
#define DB_PKM_APP_STATS_MAX_SIZE         0x100

/* ----------------------------------------------------------------------------
 ENTER HERE THE FLASH LOCATION OF THE DATABASE(PRIMARY AND SECONDARY)
 MAKE SURE THAT THE DATABASE MAX SIZE IS ACCOUNTED FOR. 
 ----------------------------------------------------------------------------*/

#define DB_ABS_BIOM_STATS_PRIM_BANK     0x3400
#define DB_ABS_BIOM_STATS_SEC_BANK      (DB_ABS_BIOM_STATS_PRIM_BANK    + DB_ABS_BIOM_STATS_MAX_SIZE)  
#define DB_SYSTEM_PARAMS_PRIM_BANK      (DB_ABS_BIOM_STATS_SEC_BANK     + DB_ABS_BIOM_STATS_MAX_SIZE)
#define DB_SYSTEM_PARAMS_SEC_BANK       (DB_SYSTEM_PARAMS_PRIM_BANK     + DB_SYSTEM_PARAMS_MAX_SIZE)
#define DB_HW_PARAMS_PRIM_BANK          (DB_SYSTEM_PARAMS_SEC_BANK      + DB_SYSTEM_PARAMS_MAX_SIZE)
#define DB_HW_PARAMS_SEC_BANK           (DB_HW_PARAMS_PRIM_BANK         + DB_HW_PARAMS_MAX_SIZE)
#define DB_USER_PARAMS_PRIM_BANK        (DB_HW_PARAMS_SEC_BANK          + DB_HW_PARAMS_MAX_SIZE)
#define DB_USER_PARAMS_SEC_BANK         (DB_USER_PARAMS_PRIM_BANK       + DB_USER_PARAMS_MAX_SIZE)
#define DB_REL_BIOM_STATS_PRIM_BANK     (DB_USER_PARAMS_SEC_BANK        + DB_USER_PARAMS_MAX_SIZE)
#define DB_REL_BIOM_STATS_SEC_BANK      (DB_REL_BIOM_STATS_PRIM_BANK    + DB_REL_BIOM_STATS_MAX_SIZE)
#define DB_PKM_APP_STATS_PRIM_BANK      (DB_REL_BIOM_STATS_SEC_BANK     + DB_REL_BIOM_STATS_MAX_SIZE)
#define DB_PKM_APP_STATS_SEC_BANK       (DB_PKM_APP_STATS_PRIM_BANK     + DB_PKM_APP_STATS_MAX_SIZE)
#define DB_ABS_BIOM_STATS_RUNTIME_BANK  (DB_PKM_APP_STATS_SEC_BANK      + DB_PKM_APP_STATS_MAX_SIZE)
#define DB_REL_BIOM_STATS_RUNTIME_BANK  (DB_ABS_BIOM_STATS_RUNTIME_BANK + DB_ABS_BIOM_STATS_MAX_SIZE)
#define DB_USER_PARAMS_RUNTIME_BANK     (DB_REL_BIOM_STATS_RUNTIME_BANK + DB_REL_BIOM_STATS_MAX_SIZE)

/* -----------------------------------------------------------------------------
ENTER HERE ALL THE ENUMERATION OF THE DATABASES DEFINED IN THE APPLICATION
---------------------------------------------------------------------------- **/
typedef enum _EPkmAppDatabase
{
   DB_ABS_BIOM_STATS = TOTAL_NUM_OF_B3C_DB,
   DB_SYSTEM_PARAMS,
   DB_HW_PARAMS,
   DB_USER_PARAMS,
   DB_REL_BIOM_STATS,
   DB_PKM_APP_STATS,
   DB_ABS_BIOM_STATS_RUNTIME,
   DB_REL_BIOM_STATS_RUNTIME,
   DB_USER_PARAMS_RUNTIME,
   TOTAL_NUM_OF_DB
} EPkmAppDatabase;

/* -----------------------------------------------------------------------------
ENTER HERE CONFIGURATION OF THE DATABASES. 
MAKE SURE TO KEEP THE SAME ORDER AS THE ENUMERATION IN EDatabase.
---------------------------------------------------------------------------- **/
#ifdef DIAG_APP
static	SDatabaseDescription	sPkmAppDataBaseCfgTbl[(uint8_t)TOTAL_NUM_OF_DB-(uint8_t)TOTAL_NUM_OF_B3C_DB] =
{
 //primaryBankOffset,                   secondaryBankOffset,            maxSizeOfDatabase,              sizeOfDatabase,                         Entries,        dualBank,       writeAccessAllowed      runtime         databaseVersion
  {DB_ABS_BIOM_STATS_PRIM_BANK,         DB_ABS_BIOM_STATS_SEC_BANK,     DB_ABS_BIOM_STATS_MAX_SIZE,     sizeof(SBiometricStatsDb),              1,              true,           true,                   false,          CURRENT_BIOMETRIC_STATS_DB_VERSION},
  {DB_SYSTEM_PARAMS_PRIM_BANK,          DB_SYSTEM_PARAMS_SEC_BANK,      DB_SYSTEM_PARAMS_MAX_SIZE,      sizeof(SSystemParameters),              1,              true,           true,                   false,          CURRENT_SYSTEM_PARAMETERS_DB_VERSION},
  {DB_HW_PARAMS_PRIM_BANK,              DB_HW_PARAMS_SEC_BANK,          DB_HW_PARAMS_MAX_SIZE,          sizeof(SHardwareParametersDb),          1,              true,           true,                   false,          CURRENT_HW_PARAMS_DB_VERSION},
  {DB_USER_PARAMS_PRIM_BANK,            DB_USER_PARAMS_SEC_BANK,        DB_USER_PARAMS_MAX_SIZE,        sizeof(SUserParametersDb),              1,              true,           true,                   false,          CURRENT_USER_PARAMS_DB_VERSION},
  {DB_REL_BIOM_STATS_PRIM_BANK,         DB_REL_BIOM_STATS_SEC_BANK,     DB_REL_BIOM_STATS_MAX_SIZE,     sizeof(SBiometricStatsDb),              1,              true,           true,                   false,          CURRENT_BIOMETRIC_STATS_DB_VERSION},
  {DB_PKM_APP_STATS_PRIM_BANK,          DB_PKM_APP_STATS_SEC_BANK,      DB_PKM_APP_STATS_MAX_SIZE,      sizeof(SApplicationStatsDb),            1,              true,           true,                   false,          CURRENT_APPLICATION_STATS_DB_VERSION},
  {DB_ABS_BIOM_STATS_RUNTIME_BANK,      0,                              DB_ABS_BIOM_STATS_MAX_SIZE,     sizeof(SBiometricStatsDb),              1,              false,          true,                   true,           CURRENT_BIOMETRIC_STATS_DB_VERSION},
  {DB_REL_BIOM_STATS_RUNTIME_BANK,      0,                              DB_REL_BIOM_STATS_MAX_SIZE,     sizeof(SBiometricStatsDb),              1,              false,          true,                   true,           CURRENT_BIOMETRIC_STATS_DB_VERSION},
  {DB_USER_PARAMS_RUNTIME_BANK,         0,                              DB_USER_PARAMS_MAX_SIZE,        sizeof(SUserParametersDb),              1,              false,          true,                   true,           CURRENT_USER_PARAMS_DB_VERSION}
};
#else
static	SDatabaseDescription	sPkmAppDataBaseCfgTbl[(uint8_t)TOTAL_NUM_OF_DB-(uint8_t)TOTAL_NUM_OF_B3C_DB] =
{
 //primaryBankOffset,              secondaryBankOffset,            maxSizeOfDatabase,              sizeOfDatabase,                         Entries,        dualBank,       writeAccessAllowed           runtime         databaseVersion
  {DB_ABS_BIOM_STATS_PRIM_BANK,         DB_ABS_BIOM_STATS_SEC_BANK,     DB_ABS_BIOM_STATS_MAX_SIZE,     sizeof(SBiometricStatsDb),              1,              true,           true,                   false,          CURRENT_BIOMETRIC_STATS_DB_VERSION},
  {DB_SYSTEM_PARAMS_PRIM_BANK,          DB_SYSTEM_PARAMS_SEC_BANK,      DB_SYSTEM_PARAMS_MAX_SIZE,      sizeof(SSystemParameters),              1,              true,           true,                   false,          CURRENT_SYSTEM_PARAMETERS_DB_VERSION},
  {DB_HW_PARAMS_PRIM_BANK,              DB_HW_PARAMS_SEC_BANK,          DB_HW_PARAMS_MAX_SIZE,          sizeof(SHardwareParametersDb),          1,              true,           false,                  false,          CURRENT_HW_PARAMS_DB_VERSION},
  {DB_USER_PARAMS_PRIM_BANK,            DB_USER_PARAMS_SEC_BANK,        DB_USER_PARAMS_MAX_SIZE,        sizeof(SUserParametersDb),              1,              true,           true,                   false,          CURRENT_USER_PARAMS_DB_VERSION},
  {DB_REL_BIOM_STATS_PRIM_BANK,         DB_REL_BIOM_STATS_SEC_BANK,     DB_REL_BIOM_STATS_MAX_SIZE,     sizeof(SBiometricStatsDb),              1,              true,           true,                   false,          CURRENT_BIOMETRIC_STATS_DB_VERSION},
  {DB_PKM_APP_STATS_PRIM_BANK,          DB_PKM_APP_STATS_SEC_BANK,      DB_PKM_APP_STATS_MAX_SIZE,      sizeof(SApplicationStatsDb),            1,              true,           true,                   false,          CURRENT_APPLICATION_STATS_DB_VERSION},
  {DB_ABS_BIOM_STATS_RUNTIME_BANK,      0,                              DB_ABS_BIOM_STATS_MAX_SIZE,     sizeof(SBiometricStatsDb),              1,              false,          true,                   true,           CURRENT_BIOMETRIC_STATS_DB_VERSION},
  {DB_REL_BIOM_STATS_RUNTIME_BANK,      0,                              DB_REL_BIOM_STATS_MAX_SIZE,     sizeof(SBiometricStatsDb),              1,              false,          true,                   true,           CURRENT_BIOMETRIC_STATS_DB_VERSION},
  {DB_USER_PARAMS_RUNTIME_BANK,         0,                              DB_USER_PARAMS_MAX_SIZE,        sizeof(SUserParametersDb),              1,              false,          true,                   true,           CURRENT_USER_PARAMS_DB_VERSION}
};
#endif

#endif // PKM_APP_DB_DATABASE_MANAGER_DEF_H
