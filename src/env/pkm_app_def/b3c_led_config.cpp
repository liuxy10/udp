#include "bionics_const.h"
#include "stdlib.h"
#include "B3C_HW_TOP_hw_handlers.h"
#include "b3c_led_config.h"
#include <B3C_SW_UI_LED.h>

uint8_t defaultPattern[1] = {0};
uint8_t blink[6] = {50, 50, 50, 0, 0, 0};
uint8_t breathing[64] = {49,46,42,38,34,31,27,24,21,18,15,13,11,9,7,5,4,3,2,1,0,0,0,0,1,2,3,4,5,7,9,11,13,15,18,21,24,27,31,34,38,42,46,49,54,58,63,68,73,78,83,89,94,99,99,94,89,83,78,73,68,63,58,54};
uint8_t blink_fast[20] = {50,50,50,50,50,50,50,50,50,50,0,0,0,0,0,0,0,0,0,0};
uint8_t steady[1] = {50};
uint8_t steadyDim[1] = {4};
uint8_t blink_fast_dim[20] = {4,4,4,4,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0};

uint8_t red[1] = {0};
uint8_t green[1] = {1};
uint8_t blue[1] = {2};
uint8_t yellow[1] = {3};
uint8_t orange[1] = {4};
uint8_t white[1] = {5};
uint8_t rainbow[5] = {0,4,3,2,1};


SLedFunctions SLedFuncArr[NUM_OF_LED]=
//Init                   //SetLED                      //TriggerScan                  //Disable         //Enable               
{
    {&B3C_UI_Init,       &B3C_UI_LED_0,               &B3C_UI_LED_UpdateTrigger,      NULL,             NULL},//LED_NR_0
    {NULL,               &B3C_UI_LED_1,               NULL,                           NULL,             NULL},//LED_NR_1
    {NULL,               &B3C_UI_LED_2,               NULL,                           NULL,             NULL},//LED_NR_2
    {NULL,               &B3C_UI_LED_3,               NULL,                           NULL,             NULL},//LED_NR_3
    {NULL,               &B3C_UI_LED_4,               NULL,                           NULL,             NULL},//LED_NR_4
    {NULL,               &B3C_UI_LED_5,               NULL,                           NULL,             NULL},//LED_NR_5    
}; 

SLedConfig SLedConfigArr [NUM_OF_LED] =
{// Group                Inuse        SR                       timeout   TimeoutEn         currentColorFrame     currentPatternFrame color                               Frame   
  { LED_INDICATOR,       true,        LED_250_MS_SAMPLE_RATE,  0,        false,            0,                    0,                  &SLedColorVectorArr[GREEN],         &SLedPatternVectorArr[LED_PATTERN_BLINKING]},//LED_NR_0
  { LED_APPLICATION,     false,       LED_SAMPLE_RATE,         0,        false,            0,                    0,                  &SLedColorVectorArr[GREEN],         &SLedPatternVectorArr[LED_PATTERN_STEADY]},//LED_NR_1
  { LED_APPLICATION,     false,       LED_SAMPLE_RATE,         0,        false,            0,                    0,                  &SLedColorVectorArr[GREEN],         &SLedPatternVectorArr[LED_PATTERN_STEADY]},//LED_NR_2
  { LED_APPLICATION,     false,       LED_SAMPLE_RATE,         0,        false,            0,                    0,                  &SLedColorVectorArr[GREEN],         &SLedPatternVectorArr[LED_PATTERN_STEADY]},//LED_NR_3
  { LED_APPLICATION,     false,       LED_SAMPLE_RATE,         0,        false,            0,                    0,                  &SLedColorVectorArr[GREEN],         &SLedPatternVectorArr[LED_PATTERN_STEADY]},//LED_NR_4
  { LED_APPLICATION,     false,       LED_SAMPLE_RATE,         0,        false,            0,                    0,                  &SLedColorVectorArr[GREEN],         &SLedPatternVectorArr[LED_PATTERN_STEADY]},//LED_NR_5
};


SLedVector SLedPatternVectorArr [NUM_OF_PATTERN] =
{// Id                                  Size                     Vector    
  {LED_PATTERN_BLINKING,                sizeof(blink),          &blink[0]},            //Blinking 
  {LED_PATTERN_STEADY,                  sizeof(steady),         &steady[0]},           //Steady
  {LED_PATTERN_BLINKING_FAST,           sizeof(blink_fast),     &blink_fast[0]},       //Blinking fast
  {LED_PATTERN_STEADY_DIM,              sizeof(steadyDim),      &steadyDim[0]},        //SteadyDim
  {LED_PATTERN_BLINKING_FAST_DIM,       sizeof(blink_fast_dim), &blink_fast_dim[0]},   //Blinking fast dim
};

SLedVector SLedColorVectorArr [NUM_OF_COLOR] =
{// Id          Size                    Vector    
  {RED,         sizeof(red),            &red[0]},       //Red
  {GREEN,       sizeof(green),          &green[0]},     //Green
  {BLUE,        sizeof(blue),           &blue[0]},      //Blue
  {YELLOW,      sizeof(yellow),         &yellow[0]},    //Yellow
  {ORANGE,      sizeof(orange),         &orange[0]},    //Orange
  {WHITE,       sizeof(white),          &white[0]},     //White
  {RAINBOW,     sizeof(rainbow),        &rainbow[0]}    // Rainbow
};
