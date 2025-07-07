/** @file
 * @defgroup OB3C Össur Bionic Common Core Components
 * @{
 * @ingroup framework
 * @brief FPGA General information .
 *
 * @details
 *      User defines for the Bionic Communication
 *
 * @author Sunneva Osk Palmarsdottir
 * @date February 2019
 *
 * @copyright Copyright (c) 2016 Össur.\n
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval from Össur.
**/
#ifndef B3C_SW_BIONIC_COMM_DEF_H
#define B3C_SW_BIONIC_COMM_DEF_H

/* ----------------------------------------------------------------------------
Includes
--------------------------------------------------------------------------- **/

/* ----------------------------------------------------------------------------
Constants
--------------------------------------------------------------------------- **/

#ifdef MAIN_APP /* -- Main application defines -- */
#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_BASE_VAR                       70
#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_KNEE_VAR                       50
#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_LEG_VAR                        50
#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_HIP_VAR                        5
#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_PKM_VAR                        85
#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_RHEO_VAR                       60
#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_APO_VAR                        30

#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_BASE_FUNCTIONS                 60
#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_LEG_FUNCTIONS                  32
#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_KNEE_FUNCTIONS                 32

#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_NOTIFY                         10
#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_LOGGING                        32

#define B3C_BIONIC_COMM_LOG_BUFFER_SIZE                                 0x150000
#define B3C_BIONIC_COMM_TX_BULK_BUFFER_SIZE                             2048
#define B3C_BIONIC_COMM_RX_BULK_BUFFER_SIZE                             512


#elif DIAG_APP /* -- Diagnostic application defines -- */
#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_BASE_VAR                       55
#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_KNEE_VAR                       20
#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_LEG_VAR                        10
#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_PKM_VAR                        17 // EXP ADCs

#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_BASE_FUNCTIONS                 60
#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_LEG_FUNCTIONS                  32
#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_KNEE_FUNCTIONS                 32

#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_NOTIFY                         10
#define B3C_BIONIC_COMM_NUM_OF_SUPPORTED_LOGGING                        32

#define B3C_BIONIC_COMM_LOG_BUFFER_SIZE                                 0x150000
#define B3C_BIONIC_COMM_TX_BULK_BUFFER_SIZE                             2048
#define B3C_BIONIC_COMM_RX_BULK_BUFFER_SIZE                             512
#endif

/* ----------------------------------------------------------------------------
Data types
--------------------------------------------------------------------------- **/

/* ----------------------------------------------------------------------------
Classes
--------------------------------------------------------------------------- **/

#endif  //B3C_SW_BIONIC_COMM_DEF_H
