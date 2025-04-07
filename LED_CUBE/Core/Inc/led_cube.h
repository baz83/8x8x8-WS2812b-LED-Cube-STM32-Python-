#ifndef __LED_CUBE_H
#define __LED_CUBE_H

#include "stm32f4xx_hal.h"
#include "ws2812b.h"

// LED Cube dimensions
#define CUBE_SIZE 8
#define PLANE_COUNT 8
#define LEDS_PER_PLANE 64

// Protocol definitions
#define CUBE_START_MARKER 0xA5  // Frame start marker
#define CUBE_END_MARKER   0x5A  // Frame end marker
#define READY_FOR_DATA    0xAA  // STM32 ready to receive signal

// Function prototypes
void LED_Cube_Init(void);
void LED_Cube_Process(void);
void LED_Cube_UpdateVoxel(uint8_t x, uint8_t y, uint8_t z, uint8_t state);
void LED_Cube_SetPlaneData(uint8_t plane, uint8_t* data);
void LED_Cube_SetFullData(uint8_t* data, uint16_t size);
void LED_Cube_Clear(void);

extern uint8_t cube_data[PLANE_COUNT][CUBE_SIZE][CUBE_SIZE];

#endif /* __LED_CUBE_H */