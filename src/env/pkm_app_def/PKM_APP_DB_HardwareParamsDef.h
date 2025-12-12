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
 * @date Okt 2016
 *
 * @copyright Copyright (c) 2016 Össur.\n
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval from Össur.
 */
#ifndef PKM_APP_DB_HARDWARE_PARAMS_DEF_H
#define PKM_APP_DB_HARDWARE_PARAMS_DEF_H

/* -----------------------------------------------------------------------------
Includes
---------------------------------------------------------------------------- **/

/* ----------------------------------------------------------------------------
Constants
---------------------------------------------------------------------------- **/

// The version of the different databases
#define CURRENT_HW_PARAMS_DB_VERSION                    1

typedef enum _EHardwareParamType // ** Update Description in hwParamStrTable **
{
    HW_PARAM_IMU_ANGLE_OFFSET_SAGITAL = 0,      //0
    HW_PARAM_IMU_ANGLE_OFFSET_FRONTAL,          //1
    HW_PARAM_CT_ANGLE_GAIN,                     //2     
    HW_PARAM_CT_ANGLE_OFFSET,                   //3
    HW_PARAM_CT_TORQUE_COEFF_1,                 //4 
    HW_PARAM_CT_TORQUE_COEFF_2,                 //5
    HW_PARAM_KA_ANGLE_GAIN,                     //6
    HW_PARAM_KA_ANGLE_OFFSET,                   //7
    HW_PARAM_MOTOR_TEMP_GAIN,                   //8
    HW_PARAM_MOTOR_TEMP_OFFSET,                 //9
    HW_PARAM_MOTOR_MOSFET_TEMP_GAIN,            //10
    HW_PARAM_MOTOR_MOSFET_TEMP_OFFSET,          //11
    HW_PARAM_GCS0_GAIN,                         //12
    HW_PARAM_GCS0_OFFSET,                       //13
    HW_PARAM_GCS0_TEMP_COMP_CONFIG,             //14
    HW_PARAM_GCS1_GAIN,                         //15
    HW_PARAM_GCS1_OFFSET,                       //16
    HW_PARAM_GCS1_TEMP_COMP_CONFIG,             //17
    HW_PARAM_GCS2_GAIN,                         //18
    HW_PARAM_GCS2_OFFSET,                       //19                  
    HW_PARAM_GCS2_TEMP_COMP_CONFIG,             //20
    HW_PARAM_GCS3_GAIN,                         //21
    HW_PARAM_GCS3_OFFSET,                       //22
    HW_PARAM_GCS3_TEMP_COMP_CONFIG,             //23
    HW_PARAM_MB_ADC_CH7_GAIN,                   //24
    HW_PARAM_MB_ADC_CH7_OFFSET,                 //25
    HW_PARAM_MOTOR_OUTPUT_GAIN,                 //26
    HW_PARAM_MOTOR_OUTPUT_OFFSET,               //27
    HW_PARAM_GCS0_TEMP_COMP_OFFSET,             //28
    HW_PARAM_GCS0_TEMP_COMP_GAIN,               //29
    HW_PARAM_GCS0_TEMP_COMP_WRAP_OFFSET,        //30
    HW_PARAM_GCS1_TEMP_COMP_OFFSET,             //31
    HW_PARAM_GCS1_TEMP_COMP_GAIN,               //32
    HW_PARAM_GCS1_TEMP_COMP_WRAP_OFFSET,        //33
    HW_PARAM_GCS2_TEMP_COMP_OFFSET,             //34
    HW_PARAM_GCS2_TEMP_COMP_GAIN,               //35
    HW_PARAM_GCS2_TEMP_COMP_WRAP_OFFSET,        //36
    HW_PARAM_GCS3_TEMP_COMP_OFFSET,             //37
    HW_PARAM_GCS3_TEMP_COMP_GAIN,               //38
    HW_PARAM_GCS3_TEMP_COMP_WRAP_OFFSET,        //39
    HW_PARAM_MOTOR_MAX_TORQUE,                  //40
    HW_PARAM_RESEVED_2,                         //41
    HW_PARAM_RESEVED_3,                         //42
    HW_PARAM_RESEVED_4,                         //43
    HW_PARAM_RESEVED_5,                         //44
    HW_PARAM_RESEVED_6,                         //45
    HW_PARAM_RESEVED_7,                         //46
    HW_PARAM_RESEVED_8,                         //47
    HW_PARAM_RESEVED_9,                         //48
    HW_PARAM_RESEVED_10,                        //49
    NB_OF_HW_PARAMS
}EHardwareParamType;

/*------------------------------------------------------------------------------
Name:       SHardwareParametersDb
Desc:       Gain and offset to by applied to convert sensor acquisition to
            engineering value.
            Useful for calibrating the sensor of the platform.
            These parameters are to be updated by the Diagnostic Application.
            To be forwarded to the User Application to be retrieved.
------------------------------------------------------------------------------*/
typedef struct _SHardwareParametersDb
{
    float	hwParam[NB_OF_HW_PARAMS];
} SHardwareParametersDb;


#endif //PKM_APP_DB_HARDWARE_PARAMS_DEF_H
