/** @file
 * @defgroup Össur Power Knee Mainstream
 * @{
 * @ingroup framework
 * @brief PKM Definitions.
 *
 * @details
 * Declearations of the Gain scheduler parameters.
 *
 * @author Árni Einarsson
 * @date Okt 2016
 *
 * @copyright Copyright (c) 2016 Össur.\n
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval from Össur.
 */
#ifndef PKM_APP_GAINSCHEDULERDEF_H
#define PKM_APP_GAINSCHEDULERDEF_H

/* -----------------------------------------------------------------------------
Includes
---------------------------------------------------------------------------- **/

/* ----------------------------------------------------------------------------
Constants
---------------------------------------------------------------------------- **/

/* -----------------------------------------------------------------------------
Data types
---------------------------------------------------------------------------- **/
typedef enum _ESchedulerAdaptationPoint
{
    MIN_CADENCE,
    MAX_CADENCE,
    NB_OF_ADAPTATION_POINTS
}
ESchedulerAdaptationPoint;

#endif //PKM_APP_GAINSCHEDULERDEF_H
