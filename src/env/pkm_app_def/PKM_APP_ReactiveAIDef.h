/** @file
 * @defgroup Össur Power Knee Mainstream
 * @{
 * @ingroup framework
 * @brief PKM Definitions.
 *
 * @details
 * Declearations of the Reactive AI structures.
 *
 * @author Árni Einarsson
 * @date Okt 2016
 *
 * @copyright Copyright (c) 2016 Össur.\n
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval from Össur.
 */
#ifndef PKM_APP_REACTIVEAIDEF_H
#define PKM_APP_REACTIVEAIDEF_H

/* -----------------------------------------------------------------------------
Includes
---------------------------------------------------------------------------- **/
#include "PKM_APP_TaskDeliberativeAIDef.h"

/* ----------------------------------------------------------------------------
Constants
---------------------------------------------------------------------------- **/

/* -----------------------------------------------------------------------------
Data types
---------------------------------------------------------------------------- **/

// Locomotion basic phases
typedef enum _EBasicPhases{
    PHASE_STANCE,         // Stance
    PHASE_SWING,          // Swing
    NB_OF_PHASES          // Do not modify this value
} EBasicPhases;

// Locomotion basic phases
typedef enum _ESubphases{
    SUBPHASE_FORCE_REJECTION,
    SUBPHASE_TOE_OFF_ASSIST,
    SUBPHASE_BUMPER_AVOIDANCE,
    SUBPHASE_FORCE_FOLLOWING,
    SUBPHASE_BRAKE,
    NB_OF_SUBPHASES          // Do not modify this value
} ESubphases;


#endif // PKM_APP_REACTIVEAIDEF_H
