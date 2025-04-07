/**
  * @file    cube_patterns.h
  * @brief   LED Cube Pattern Definitions and Handlers
  */

#ifndef __CUBE_PATTERNS_H
#define __CUBE_PATTERNS_H

#include "stm32f4xx_hal.h"
#include "led_cube.h"

/* Pattern constants */
#define PATTERN_COUNT        10     // Total number of patterns
#define PATTERN_STATIC       0      // Static pattern mode
#define PATTERN_DYNAMIC      1      // Dynamic pattern mode

/* Functions for pattern management */
void CUBE_Patterns_Init(void);
void CUBE_Patterns_ProcessAnimation(void);
void CUBE_Patterns_SetMode(uint8_t mode);
void CUBE_Patterns_NextPattern(void);
uint8_t CUBE_Patterns_GetMode(void);
uint8_t CUBE_Patterns_GetCurrentPattern(void);

/* Static pattern functions */
void CUBE_Patterns_ShowStaticPattern(uint8_t patternIndex);

/* Individual pattern functions - accessible for testing */
void pattern_PlaneWave(void);
void pattern_Rain(void);
void pattern_Sphere(uint32_t step);
void pattern_Spiral(void);
void pattern_FireEffect(void);
void pattern_BouncingBall(void);
void pattern_Snake3D(void);
void pattern_Ripples(void);
void pattern_Helix(void);
void pattern_Fountain(void);

#endif /* __CUBE_PATTERNS_H */