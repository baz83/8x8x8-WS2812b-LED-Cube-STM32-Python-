#include "led_cube.h"
#include "ws2812b.h"
#include <string.h>

// LED Cube data buffer - [z][y][x] format (z = plane)
uint8_t cube_data[PLANE_COUNT][CUBE_SIZE][CUBE_SIZE];

// Main framebuffer for all planes - 8 planes * 64 LEDs * 3 colors (RGB)
uint8_t cube_framebuffer[PLANE_COUNT * LEDS_PER_PLANE * 3];

// Animation state
static uint32_t rainbowOffset = 0;
static float hueOffset = 0.0f;
static float brightness = 0.7f;

static uint32_t cubeHSVtoRGB(uint8_t h, uint8_t s, uint8_t v);

extern volatile uint8_t currentMode;
#define MODE_STL 1  // Match the definition in main.c

static uint32_t frame_count = 0;
static uint32_t last_fps_time = 0;
static float current_fps = 0;

/**
 * @brief Calculate and return the current refresh rate
 * @return Current refresh rate in frames per second
 */
float LED_Cube_GetRefreshRate(void) {
    return current_fps;
}

/**
 * @brief Maps (x,y) coordinates to serpentine index on a plane
 * @param x: X coordinate (0-7)
 * @param y: Y coordinate (0-7)  
 * @return Index in the serpentine pattern (0-63)
 */
static uint8_t mapToSerpentine(uint8_t x, uint8_t y) {
    // For even rows (0,2,4,6), indices go left to right
    if (y % 2 == 0) {
        return y * CUBE_SIZE + x;
    } 
    // For odd rows (1,3,5,7), indices go right to left
    else {
        return y * CUBE_SIZE + (CUBE_SIZE - 1 - x);
    }
}

/**
 * @brief Initialize the LED cube
 */
void LED_Cube_Init(void) {
    // Clear the cube data
    memset(cube_data, 0, sizeof(cube_data));
    
    // Clear the framebuffer
    memset(cube_framebuffer, 0, sizeof(cube_framebuffer));
    
    // Initialize each plane (8 channels)
    for (uint8_t i = 0; i < 8; i++) {
        ws2812b.item[i].channel = i;
        ws2812b.item[i].frameBufferPointer = &cube_framebuffer[i * 64 * 3]; // Point to appropriate section
        ws2812b.item[i].frameBufferSize = 64 * 3; // 64 LEDs × 3 bytes each
    }
    
    // Initialize the WS2812B hardware
    ws2812b_init();
}

/**
 * @brief Update a single voxel in the cube
 * @param x: X coordinate (0-7)
 * @param y: Y coordinate (0-7)
 * @param z: Z coordinate (0-7) - plane
 * @param state: 0 = off, 1 = on
 */
void LED_Cube_UpdateVoxel(uint8_t x, uint8_t y, uint8_t z, uint8_t state) {
    if (x < CUBE_SIZE && y < CUBE_SIZE && z < PLANE_COUNT) {
        cube_data[z][y][x] = state ? 1 : 0;
    }
}

/**
 * @brief Set data for an entire plane
 * @param plane: Plane index (0-7)
 * @param data: 64-byte array of LED states (0/1)
 */
void LED_Cube_SetPlaneData(uint8_t plane, uint8_t* data) {
    if (plane < PLANE_COUNT) {
        uint8_t x, y;
        for (y = 0; y < CUBE_SIZE; y++) {
            for (x = 0; x < CUBE_SIZE; x++) {
                uint8_t serpIndex = mapToSerpentine(x, y);
                cube_data[plane][y][x] = data[serpIndex] ? 1 : 0;
            }
        }
    }
}

/**
 * @brief Set data for the entire cube
 * @param data: Buffer containing LED states (one bit per LED)
 * @param size: Size of the data buffer
 */
void LED_Cube_SetFullData(uint8_t* data, uint16_t size) {
    uint8_t z, y, x;
    uint16_t bitIndex = 0;
    
    // Clear the cube first
    LED_Cube_Clear();
    
    // Process each byte of data
    for (uint16_t byteIndex = 0; byteIndex < size; byteIndex++) {
        uint8_t currentByte = data[byteIndex];
        
        // Process each bit in the byte
        for (uint8_t bit = 0; bit < 8; bit++) {
            // Extract one bit
            uint8_t state = (currentByte >> (7 - bit)) & 0x01;
            
            // Calculate coordinates
            z = bitIndex / (CUBE_SIZE * CUBE_SIZE);
            uint16_t remainder = bitIndex % (CUBE_SIZE * CUBE_SIZE);
            y = remainder / CUBE_SIZE;
            x = remainder % CUBE_SIZE;
            
            // Update the voxel if within bounds
            if (z < PLANE_COUNT && y < CUBE_SIZE && x < CUBE_SIZE) {
                cube_data[z][y][x] = state;
            }
            
            bitIndex++;
            // If we've processed all voxels, stop
            if (bitIndex >= PLANE_COUNT * CUBE_SIZE * CUBE_SIZE) {
                break;
            }
        }
    }
}

/**
 * @brief Clear the LED cube
 */
void LED_Cube_Clear(void) {
    memset(cube_data, 0, sizeof(cube_data));
}

/**
 * @brief Apply color wheel effect to a value
 */
static uint32_t Wheel(uint8_t WheelPos) {
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85) {
        return ((uint32_t)(255 - WheelPos * 3) << 16) | ((uint32_t)(0) << 8) | (WheelPos * 3);
    }
    if (WheelPos < 170) {
        WheelPos -= 85;
        return ((uint32_t)(0) << 16) | ((uint32_t)(WheelPos * 3) << 8) | (255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return ((uint32_t)(WheelPos * 3) << 16) | ((uint32_t)(255 - WheelPos * 3) << 8) | (0);
}

/**
 * @brief Apply brightness to a color component
 */
static uint8_t applyBrightness(uint8_t colorValue) {
    return (uint8_t)(colorValue * brightness);
}

/**
 * @brief Process the LED cube - generate colors and update LEDs
 */
void LED_Cube_Process(void) {
    uint8_t z, y, x;
    
    // Only update if the previous transfer is complete
    if (!ws2812b.transferComplete) {
        return;
    }
    
    // Update rainbow effect offset for animation
    // rainbowOffset += 1;
    hueOffset += 0.5f;  // Smaller values = slower transitions
    if (hueOffset >= 256.0f) {
        hueOffset = 0.0f;
    }

    // Update the framebuffer based on current cube data
    for (z = 0; z < PLANE_COUNT; z++) {
        for (y = 0; y < CUBE_SIZE; y++) {
            for (x = 0; x < CUBE_SIZE; x++) {
                // Map 3D coordinates to the 1D framebuffer index
                uint8_t serpIndex = mapToSerpentine(x, y);
                uint16_t ledIndex = (z * LEDS_PER_PLANE) + serpIndex;
                
                // Calculate color based on position and pattern
                if (cube_data[z][y][x]) {
                    // If LED is on, set it to a rainbow color
                    // uint32_t color = Wheel((serpIndex * 32 + z * 16 + rainbowOffset) % 256);
                    uint32_t color;
                    if (currentMode == MODE_STL) {
                        // Orange color for STL mode
                        color = 0x8CFF00;  // GRB format
                    } else {
                        // Rainbow colors for animation mode (existing code)
                        uint8_t hue = (uint8_t)((serpIndex * 8 + z * 4 + (uint8_t)hueOffset) % 256);
                        color = cubeHSVtoRGB(hue, 255, 255);
                    }
                    // Apply brightness
                    uint8_t r = applyBrightness((color >> 16) & 0xFF);
                    uint8_t g = applyBrightness((color >> 8) & 0xFF);
                    uint8_t b = applyBrightness(color & 0xFF);
                    
                    // Update framebuffer (GRB ordering for WS2812B)
                    cube_framebuffer[ledIndex*3]   = g; // Green
                    cube_framebuffer[ledIndex*3+1] = r; // Red
                    cube_framebuffer[ledIndex*3+2] = b; // Blue
                } else {
                    // If LED is off, set to black
                    cube_framebuffer[ledIndex*3]   = 0; // G
                    cube_framebuffer[ledIndex*3+1] = 0; // R
                    cube_framebuffer[ledIndex*3+2] = 0; // B
                }
            }
        }
    }
    
    // Trigger the transfer
    ws2812b.startTransfer = 1;
    
        // Count frame
    frame_count++;
    
    // Calculate FPS every second
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_fps_time >= 1000) {
        current_fps = (float)frame_count * 1000.0f / (current_time - last_fps_time);
        
        // Reset counters
        frame_count = 0;
        last_fps_time = current_time;
    }
    
    // Handle the transfer
    ws2812b_handle();
}

/**
 * @brief Convert HSV color to RGB
 * @param h: Hue (0-255)
 * @param s: Saturation (0-255)
 * @param v: Value/Brightness (0-255)
 * @return RGB color as 32-bit value (0x00RRGGBB)
 */
static uint32_t cubeHSVtoRGB(uint8_t h, uint8_t s, uint8_t v) {
    uint8_t region, remainder, p, q, t;
    uint8_t r, g, b;
    
    if (s == 0) {
        r = g = b = v;
        return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    }
    
    region = h / 43;
    remainder = (h - (region * 43)) * 6; 
    
    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;
    
    switch (region) {
        case 0:  r = v; g = t; b = p; break;
        case 1:  r = q; g = v; b = p; break;
        case 2:  r = p; g = v; b = t; break;
        case 3:  r = p; g = q; b = v; break;
        case 4:  r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
    }
    
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}