#ifndef PKM_APP_DEF_DEVPARAMS_H_
#define PKM_APP_DEF_DEVPARAMS_H_

#include "B3C_SW_DB_DevelopmentParamsDef.h"

typedef enum _EDevelopmentParamVectorInputConfigType
{
  DEVP_VECTOR_INPUT_CONFIG_CURRENT_CONTROL = 0,
  DEVP_VECTOR_INPUT_CONFIG_POS_CONTROL,
  DEVP_VECTOR_INPUT_CONFIG_VEL_CONTROL,
  DEVP_VECTOR_INPUT_CONFIG_POS_AND_VEL_CONTROL
} EDevelopmentParamVectorInputConfigType;

typedef enum _EDevelopmentParamDisableDriveType
{
  DEVP_DRIVE_ENABLED = 0,
  DEVP_DRIVE_DISABLED
} EDevelopmentParamDisableDriveType;

typedef enum _EDevelopmentParamDisableOutputType
{
  DEVP_OUTPUT_ENABLED = 0,
  DEVP_OUTPUT_DISABLED
} EDevelopmentParamDisableOutputType;

EDevelopmentParamType DEVP_VECTOR_INPUT_GAIN_KP = DEVP_PARAM_4;
EDevelopmentParamType DEVP_VECTOR_INPUT_GAIN_KD = DEVP_PARAM_5;

#endif /* PKM_APP_DEF_DEVPARAMS_H_*/