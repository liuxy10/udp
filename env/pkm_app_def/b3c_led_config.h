#ifndef B3C_LED_CONFIG_H_
#define B3C_LED_CONFIG_H_

#include "stdint.h"
#include "bionics_const.h"

#define LED_SAMPLE_RATE              1 // 50ms
#define LED_250_MS_SAMPLE_RATE       5 // 250ms

// LED Enumeration
typedef enum 
{
  LED_NR_0 = 0,              // 0
  LED_NR_1,                  // 1
  LED_NR_2,                  // 2
  LED_NR_3,                  // 3
  LED_NR_4,                  // 4
  LED_NR_5,                  // 5
  NUM_OF_LED                 // 6
} systemLedId;

// LED Group
typedef enum 
{
  LED_INDICATOR = 0,                // 0
  LED_APPLICATION,                  // 1
  NUM_OF_APPLICATION                // 2
} ledApplication;

// LED Color
typedef enum 
{
  RED,                        // 0
  GREEN,                      // 1
  BLUE,                       // 2
  YELLOW,                     // 3
  ORANGE,                     // 4
  WHITE,                      // 5
  RAINBOW,                    // 6
  NUM_OF_COLOR                // 7
} systemColorId;


// LED Pattern
typedef enum 
{
  LED_PATTERN_BLINKING,                 // 0
  LED_PATTERN_STEADY,                   // 1
  LED_PATTERN_BLINKING_FAST,            // 2
  LED_PATTERN_STEADY_DIM,               // 3
  LED_PATTERN_BLINKING_FAST_DIM,        // 4
  NUM_OF_PATTERN                        // 5
} systemPatternId;

#endif /* B3C_LED_CONFIG_H_*/