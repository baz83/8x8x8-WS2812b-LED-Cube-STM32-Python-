/**
  * @file    cube_patterns.c
  * @brief   LED Cube Pattern Implementations
  */

#include "cube_patterns.h"
#include "led_cube.h"
#include <math.h>
#include <stdlib.h>

/* Private variables for pattern management */
static uint8_t currentPattern = 0;       // Current pattern index
static uint8_t patternMode = PATTERN_STATIC;    // Current pattern mode
static uint32_t animationStep = 0;       // Animation step counter
static uint32_t lastAnimationUpdate = 0; // Last animation time
static uint32_t animationSpeed = 100;    // Animation speed in ms

/**
  * @brief  Initialize cube pattern functionality
  * @retval None
  */
void CUBE_Patterns_Init(void)
{
    // Set initial values
    currentPattern = 0;
    patternMode = PATTERN_STATIC;
    animationStep = 0;
    lastAnimationUpdate = 0;
    animationSpeed = 100;
    
    // Show initial static pattern
    CUBE_Patterns_ShowStaticPattern(currentPattern);
}

/**
  * @brief  Process animation frames when in dynamic mode
  * @retval None
  */
void CUBE_Patterns_ProcessAnimation(void)
{
    uint32_t currentTime = HAL_GetTick();
    
    // Only update dynamic patterns at specified intervals
    if (patternMode == PATTERN_DYNAMIC && 
        (currentTime - lastAnimationUpdate >= animationSpeed)) {
        
        lastAnimationUpdate = currentTime;
        animationStep++;  // Advance animation frame
        
        // Update the cube based on current pattern
        switch (currentPattern) {
            case 0:
                pattern_PlaneWave();
                break;
            case 1:
                pattern_Rain();
                break;
            case 2:
                pattern_Sphere(animationStep);
                break;
            case 3:
                pattern_Spiral();
                break;
            case 4:
                pattern_FireEffect();
                break;
            case 5:
                pattern_BouncingBall();
                break;
            case 6:
                pattern_Snake3D();
                break;
            case 7:
                pattern_Ripples();
                break;
            case 8:
                pattern_Helix();
                break;
            case 9:
                pattern_Fountain();
                break;
            default:
                CUBE_Patterns_ShowStaticPattern(0); // Default to basic pattern
                break;
        }
    }
}

/**
  * @brief  Set the pattern mode (static or dynamic)
  * @param  mode: PATTERN_STATIC or PATTERN_DYNAMIC
  * @retval None
  */
void CUBE_Patterns_SetMode(uint8_t mode)
{
    if (mode <= PATTERN_DYNAMIC) {
        patternMode = mode;
        animationStep = 0; // Reset animation steps when mode changes
    }
}

/**
  * @brief  Advance to the next pattern
  * @retval None
  */
void CUBE_Patterns_NextPattern(void)
{
    currentPattern = (currentPattern + 1) % PATTERN_COUNT;
    animationStep = 0; // Reset animation steps when pattern changes
    
    if (patternMode == PATTERN_STATIC) {
        // For static patterns, update immediately
        CUBE_Patterns_ShowStaticPattern(currentPattern);
    }
}

/**
  * @brief  Get current pattern mode
  * @retval Pattern mode (PATTERN_STATIC or PATTERN_DYNAMIC)
  */
uint8_t CUBE_Patterns_GetMode(void)
{
    return patternMode;
}

/**
  * @brief  Get current pattern index
  * @retval Current pattern index (0-PATTERN_COUNT-1)
  */
uint8_t CUBE_Patterns_GetCurrentPattern(void)
{
    return currentPattern;
}

/**
  * @brief  Display a static pattern
  * @param  patternIndex: Index of pattern to display
  * @retval None
  */
void CUBE_Patterns_ShowStaticPattern(uint8_t patternIndex)
{
    // Clear cube first
    LED_Cube_Clear();
    
    switch(patternIndex) {
        case 0: // Alternating planes (checkerboard in Z)
            for (uint8_t z = 0; z < PLANE_COUNT; z++) {
                for (uint8_t y = 0; y < CUBE_SIZE; y++) {
                    for (uint8_t x = 0; x < CUBE_SIZE; x++) {
                        cube_data[z][y][x] = (z % 2 == 0) ? 1 : 0;
                    }
                }
            }
            break;
            
        case 1: // Horizontal planes only (even ones)
            for (uint8_t z = 0; z < PLANE_COUNT; z += 2) {
                for (uint8_t y = 0; y < CUBE_SIZE; y++) {
                    for (uint8_t x = 0; x < CUBE_SIZE; x++) {
                        cube_data[z][y][x] = 1;
                    }
                }
            }
            break;
            
        case 2: // Border/Shell cube
            for (uint8_t z = 0; z < PLANE_COUNT; z++) {
                for (uint8_t y = 0; y < CUBE_SIZE; y++) {
                    for (uint8_t x = 0; x < CUBE_SIZE; x++) {
                        // Set voxels on the outer edges
                        if (x == 0 || x == CUBE_SIZE-1 || 
                            y == 0 || y == CUBE_SIZE-1 || 
                            z == 0 || z == PLANE_COUNT-1) {
                            cube_data[z][y][x] = 1;
                        }
                    }
                }
            }
            break;
            
        case 3: // Diagonal plane
            for (uint8_t z = 0; z < PLANE_COUNT; z++) {
                for (uint8_t y = 0; y < CUBE_SIZE; y++) {
                    for (uint8_t x = 0; x < CUBE_SIZE; x++) {
                        if (x + y + z <= CUBE_SIZE + 2) {
                            cube_data[z][y][x] = 1;
                        }
                    }
                }
            }
            break;
            
        case 4: // X pattern (diagonal crosses in each plane)
            for (uint8_t z = 0; z < PLANE_COUNT; z++) {
                for (uint8_t y = 0; y < CUBE_SIZE; y++) {
                    for (uint8_t x = 0; x < CUBE_SIZE; x++) {
                        if (x == y || x == (CUBE_SIZE-1-y)) {
                            cube_data[z][y][x] = 1;
                        }
                    }
                }
            }
            break;
            
        case 5: // Cube inside cube
            for (uint8_t z = 1; z < PLANE_COUNT-1; z++) {
                for (uint8_t y = 1; y < CUBE_SIZE-1; y++) {
                    for (uint8_t x = 1; x < CUBE_SIZE-1; x++) {
                        if (x == 1 || x == CUBE_SIZE-2 || 
                            y == 1 || y == CUBE_SIZE-2 || 
                            z == 1 || z == PLANE_COUNT-2) {
                            cube_data[z][y][x] = 1;
                        }
                    }
                }
            }
            // Also set outer cube shell
            CUBE_Patterns_ShowStaticPattern(2);
            break;
            
        case 6: // Random voxels
            for (uint8_t z = 0; z < PLANE_COUNT; z++) {
                for (uint8_t y = 0; y < CUBE_SIZE; y++) {
                    for (uint8_t x = 0; x < CUBE_SIZE; x++) {
                        cube_data[z][y][x] = (rand() % 5 == 0) ? 1 : 0;
                    }
                }
            }
            break;
            
        case 7: // Diagonal lines across cube
            for (uint8_t i = 0; i < CUBE_SIZE; i++) {
                for (uint8_t j = 0; j < CUBE_SIZE; j++) {
                    cube_data[i][j][i] = 1;       // Corner to corner diagonal
                    cube_data[i][j][CUBE_SIZE-1-i] = 1; // Other diagonal
                    cube_data[i][i][j] = 1;       // Another diagonal
                    cube_data[i][CUBE_SIZE-1-i][j] = 1; // Fourth diagonal
                }
            }
            break;
            
        case 8: // 3D cross (3 lines crossing at center)
            for (uint8_t i = 0; i < CUBE_SIZE; i++) {
                // X axis line
                cube_data[CUBE_SIZE/2][CUBE_SIZE/2][i] = 1;
                
                // Y axis line
                cube_data[CUBE_SIZE/2][i][CUBE_SIZE/2] = 1;
                
                // Z axis line
                cube_data[i][CUBE_SIZE/2][CUBE_SIZE/2] = 1;
            }
            break;
            
        case 9: // Full cube (all on)
        default:
            for (uint8_t z = 0; z < PLANE_COUNT; z++) {
                for (uint8_t y = 0; y < CUBE_SIZE; y++) {
                    for (uint8_t x = 0; x < CUBE_SIZE; x++) {
                        cube_data[z][y][x] = 1;
                    }
                }
            }
            break;
    }
}

/**
  * @brief  Sine wave that travels through the cube
  * @retval None
  */
void pattern_PlaneWave(void) {
    LED_Cube_Clear();
    
    // Calculate wave position based on animation step
    float phase = (float)animationStep / 5.0f;
    
    // Create wave traveling along z-axis
    for (uint8_t z = 0; z < PLANE_COUNT; z++) {
        // Calculate sine value for this z position
        float sine_val = sinf(phase + ((float)z / (float)PLANE_COUNT) * 2.0f * 3.14159f);
        
        // Map sine (-1 to 1) to position in cube (0 to CUBE_SIZE-1)
        int8_t y_pos = (int8_t)roundf((sine_val + 1.0f) * (CUBE_SIZE-1) / 2.0f);
        
        // Draw a horizontal line at the calculated height
        for (uint8_t x = 0; x < CUBE_SIZE; x++) {
            if (y_pos >= 0 && y_pos < CUBE_SIZE) {
                cube_data[z][y_pos][x] = 1;
            }
        }
    }
}

/**
  * @brief  Rain effect - drops falling from top to bottom
  * @retval None
  */
void pattern_Rain(void) {
    // Move all voxels down one level
    for (uint8_t z = 0; z < PLANE_COUNT; z++) {
        for (uint8_t x = 0; x < CUBE_SIZE; x++) {
            for (uint8_t y = 0; y < CUBE_SIZE-1; y++) {
                cube_data[z][y][x] = cube_data[z][y+1][x];
            }
        }
    }
    
    // Create new raindrops at top layer (randomly)
    if (animationStep % 3 == 0) {  // Only add new drops every 3 steps
        for (uint8_t z = 0; z < PLANE_COUNT; z++) {
            for (uint8_t x = 0; x < CUBE_SIZE; x++) {
                // 10% chance of a new raindrop
                cube_data[z][CUBE_SIZE-1][x] = (rand() % 10 == 0) ? 1 : 0;
            }
        }
    }
}

/**
  * @brief  Expanding/contracting sphere
  * @param  step: Animation step for timing
  * @retval None
  */
void pattern_Sphere(uint32_t step) {
    LED_Cube_Clear();
    
    // Calculate sphere radius based on animation step (oscillating)
    float max_radius = CUBE_SIZE/2.0f - 0.5f;
    float radius = max_radius * (sinf((float)step / 15.0f) + 1.0f) / 2.0f + 0.5f;
    
    float center_x = (CUBE_SIZE-1) / 2.0f;
    float center_y = (CUBE_SIZE-1) / 2.0f;
    float center_z = (PLANE_COUNT-1) / 2.0f;
    
    // Draw the sphere
    for (uint8_t z = 0; z < PLANE_COUNT; z++) {
        for (uint8_t y = 0; y < CUBE_SIZE; y++) {
            for (uint8_t x = 0; x < CUBE_SIZE; x++) {
                // Calculate distance from center
                float dx = (float)x - center_x;
                float dy = (float)y - center_y;
                float dz = (float)z - center_z;
                float distance = sqrtf(dx*dx + dy*dy + dz*dz);
                
                // If distance is within 0.5 of radius, light the voxel
                if (fabsf(distance - radius) < 0.5f) {
                    cube_data[z][y][x] = 1;
                }
            }
        }
    }
}

/**
  * @brief  Spiral pattern rotating through the cube
  * @retval None
  */
void pattern_Spiral(void) {
    LED_Cube_Clear();
    
    float center_x = (CUBE_SIZE-1) / 2.0f;
    float center_y = (CUBE_SIZE-1) / 2.0f;
    float phase = (float)animationStep / 10.0f;
    
    // Draw spiral through all z-planes
    for (uint8_t z = 0; z < PLANE_COUNT; z++) {
        float z_phase = phase + ((float)z / (float)PLANE_COUNT) * 3.14159f;
        
        // Draw an expanding spiral
        for (float r = 0.0f; r < CUBE_SIZE/2.0f; r += 0.25f) {
            // Convert polar to cartesian coordinates
            float angle = z_phase + r * 1.5f;
            float x = center_x + r * cosf(angle);
            float y = center_y + r * sinf(angle);
            
            // Only draw if coordinates are within bounds
            if (x >= 0 && x < CUBE_SIZE && y >= 0 && y < CUBE_SIZE) {
                cube_data[z][(int)y][(int)x] = 1;
            }
        }
    }
}

/**
  * @brief  Fire effect rising from bottom of cube
  * @retval None
  */
void pattern_FireEffect(void) {
    // Shift all layers up
    for (uint8_t z = PLANE_COUNT-1; z > 0; z--) {
        for (uint8_t y = 0; y < CUBE_SIZE; y++) {
            for (uint8_t x = 0; x < CUBE_SIZE; x++) {
                if ((rand() % 10) < 8) { // 80% chance to propagate upward
                    cube_data[z][y][x] = cube_data[z-1][y][x];
                } else {
                    cube_data[z][y][x] = 0; // 20% chance flame dies
                }
            }
        }
    }
    
    // Create new flames at bottom
    for (uint8_t y = 0; y < CUBE_SIZE; y++) {
        for (uint8_t x = 0; x < CUBE_SIZE; x++) {
            // Center is hotter (more likely to have flame)
            float center_x = (CUBE_SIZE-1) / 2.0f;
            float center_y = (CUBE_SIZE-1) / 2.0f;
            float dx = fabsf((float)x - center_x);
            float dy = fabsf((float)y - center_y);
            float distance = sqrtf(dx*dx + dy*dy);
            
            // More likely to have fire near center
            int chance = 90 - (int)(distance * 20.0f);
            if (chance < 20) chance = 20;
            cube_data[0][y][x] = ((rand() % 100) < chance) ? 1 : 0;
        }
    }
}

/**
  * @brief  Bouncing ball animation with simulated physics
  * @retval None
  */
void pattern_BouncingBall(void) {
    LED_Cube_Clear();
    
    // Ball position (with sub-voxel precision)
    static float ball_x = 2.0f;
    static float ball_y = 2.0f;
    static float ball_z = 4.0f;
    
    // Ball velocity
    static float vel_x = 0.12f;
    static float vel_y = 0.09f;
    static float vel_z = 0.14f;
    
    // Update position
    ball_x += vel_x;
    ball_y += vel_y;
    ball_z += vel_z;
    
    // Apply gravity to z-velocity
    vel_z -= 0.01f;
    
    // Check for collisions with walls and bounce
    if (ball_x < 0 || ball_x >= CUBE_SIZE-0.5f) {
        vel_x = -vel_x * 0.9f; // Bounce with damping
        ball_x = (ball_x < 0) ? 0 : CUBE_SIZE-0.51f; // Adjust position
    }
    
    if (ball_y < 0 || ball_y >= CUBE_SIZE-0.5f) {
        vel_y = -vel_y * 0.9f; // Bounce with damping
        ball_y = (ball_y < 0) ? 0 : CUBE_SIZE-0.51f; // Adjust position
    }
    
    if (ball_z < 0 || ball_z >= PLANE_COUNT-0.5f) {
        vel_z = -vel_z * 0.9f; // Bounce with damping
        ball_z = (ball_z < 0) ? 0 : PLANE_COUNT-0.51f; // Adjust position
        
        // Floor bounce sound effect (if we had sound)
        if (ball_z < 0.1f) {
            // Play sound here if we had audio
        }
    }
    
    // Draw the ball (3x3x3 cube for better visibility)
    for (int8_t dz = -1; dz <= 1; dz++) {
        for (int8_t dy = -1; dy <= 1; dy++) {
            for (int8_t dx = -1; dx <= 1; dx++) {
                int8_t x = (int8_t)ball_x + dx;
                int8_t y = (int8_t)ball_y + dy;
                int8_t z = (int8_t)ball_z + dz;
                
                // Only draw if within bounds
                if (x >= 0 && x < CUBE_SIZE && 
                    y >= 0 && y < CUBE_SIZE && 
                    z >= 0 && z < PLANE_COUNT) {
                    cube_data[z][y][x] = 1;
                }
            }
        }
    }
}

/**
  * @brief  3D snake moving through the cube
  * @retval None
  */
void pattern_Snake3D(void) {
    LED_Cube_Clear();

    // Length of the snake
    #define SNAKE_LENGTH 16
    
    // Snake segments (x,y,z positions)
    static int8_t snake_x[SNAKE_LENGTH];
    static int8_t snake_y[SNAKE_LENGTH];
    static int8_t snake_z[SNAKE_LENGTH];
    
    // Initialize snake if this is first call
    static uint8_t initialized = 0;
    if (!initialized) {
        for (uint8_t i = 0; i < SNAKE_LENGTH; i++) {
            snake_x[i] = CUBE_SIZE/2;
            snake_y[i] = CUBE_SIZE/2;
            snake_z[i] = i % PLANE_COUNT;
        }
        initialized = 1;
    }
    
    // Move snake body (each segment follows the one in front)
    for (int8_t i = SNAKE_LENGTH-1; i > 0; i--) {
        snake_x[i] = snake_x[i-1];
        snake_y[i] = snake_y[i-1];
        snake_z[i] = snake_z[i-1];
    }
    
    // Move head in a 3D sine pattern
    float t = (float)animationStep * 0.1f;
    snake_x[0] = (int8_t)(CUBE_SIZE/2 + sinf(t) * (CUBE_SIZE/2-1));
    snake_y[0] = (int8_t)(CUBE_SIZE/2 + sinf(t*1.3f) * (CUBE_SIZE/2-1));
    snake_z[0] = (int8_t)(PLANE_COUNT/2 + sinf(t*1.7f) * (PLANE_COUNT/2-1));
    
    // Draw the snake
    for (uint8_t i = 0; i < SNAKE_LENGTH; i++) {
        // Calculate brightness based on segment position (head is brightest)
        // In binary display, all segments are just on
        
        // Make sure coordinates are within bounds
        if (snake_x[i] >= 0 && snake_x[i] < CUBE_SIZE &&
            snake_y[i] >= 0 && snake_y[i] < CUBE_SIZE &&
            snake_z[i] >= 0 && snake_z[i] < PLANE_COUNT) {
            cube_data[snake_z[i]][snake_y[i]][snake_x[i]] = 1;
        }
    }
}

/**
  * @brief  Expanding ripples from center of cube
  * @retval None
  */
void pattern_Ripples(void) {
    LED_Cube_Clear();

    // Generate new ripple occasionally
    static uint32_t ripple_start = 0;
    if (animationStep % 40 == 0) {
        ripple_start = animationStep;
    }
    
    // Calculate ripple radius
    float elapsed = (float)(animationStep - ripple_start);
    float radius = elapsed * 0.2f;
    
    // Center of the cube
    float center_x = (CUBE_SIZE-1) / 2.0f;
    float center_y = (CUBE_SIZE-1) / 2.0f;
    float center_z = (PLANE_COUNT-1) / 2.0f;
    
    // Draw the ripple (a sphere shell)
    for (uint8_t z = 0; z < PLANE_COUNT; z++) {
        for (uint8_t y = 0; y < CUBE_SIZE; y++) {
            for (uint8_t x = 0; x < CUBE_SIZE; x++) {
                float dx = (float)x - center_x;
                float dy = (float)y - center_y;
                float dz = (float)z - center_z;
                float distance = sqrtf(dx*dx + dy*dy + dz*dz);
                
                // Draw only points at the current ripple radius
                if (fabsf(distance - radius) < 0.7f) {
                    cube_data[z][y][x] = 1;
                }
            }
        }
    }
}

/**
  * @brief  Rotating helical pattern
  * @retval None
  */
void pattern_Helix(void) {
    LED_Cube_Clear();

    // Helix parameters
    float angle = (float)animationStep * 0.1f;
    float height_scale = 5.0f; // Controls how tight the helix is
    float radius = CUBE_SIZE/3.0f;
    
    // Center of the cube base
    float center_x = (CUBE_SIZE-1) / 2.0f;
    float center_y = (CUBE_SIZE-1) / 2.0f;
    
    // Draw two opposing helices
    for (float t = 0; t < 10.0f; t += 0.25f) {
        // First helix
        float x1 = center_x + radius * cosf(angle + t);
        float y1 = center_y + radius * sinf(angle + t);
        float z1 = (t / height_scale) * PLANE_COUNT;
        
        // Second helix (180 degrees offset)
        float x2 = center_x + radius * cosf(angle + t + 3.14159f);
        float y2 = center_y + radius * sinf(angle + t + 3.14159f);
        float z2 = (t / height_scale) * PLANE_COUNT;
        
        // Draw the points if they're in bounds
        if (x1 >= 0 && x1 < CUBE_SIZE && y1 >= 0 && y1 < CUBE_SIZE && z1 >= 0 && z1 < PLANE_COUNT) {
            cube_data[(int)z1][(int)y1][(int)x1] = 1;
        }
        
        if (x2 >= 0 && x2 < CUBE_SIZE && y2 >= 0 && y2 < CUBE_SIZE && z2 >= 0 && z2 < PLANE_COUNT) {
            cube_data[(int)z2][(int)y2][(int)x2] = 1;
        }
    }
}

/**
  * @brief  Fountain effect spraying from the bottom
  * @retval None
  */
void pattern_Fountain(void) {
    // Maximum number of particles in the fountain
    #define MAX_PARTICLES 30
    
    // Particle structure
    typedef struct {
        float x, y, z;    // Position
        float vx, vy, vz; // Velocity
        uint8_t active;   // Is this particle active?
        uint16_t age;     // Age of the particle
    } Particle;
    
    // Particle array (static so it persists between calls)
    static Particle particles[MAX_PARTICLES];
    static uint8_t initialized = 0;
    
    // Initialize particles if needed
    if (!initialized) {
        for (uint8_t i = 0; i < MAX_PARTICLES; i++) {
            particles[i].active = 0;
        }
        initialized = 1;
    }
    
    // Clear the cube
    LED_Cube_Clear();
    
    // Create new particles
    if (animationStep % 3 == 0) { // Every few frames, spawn new particles
        for (uint8_t i = 0; i < MAX_PARTICLES; i++) {
            if (!particles[i].active) {
                particles[i].x = CUBE_SIZE / 2.0f;
                particles[i].y = CUBE_SIZE / 2.0f;
                particles[i].z = 0;
                
                // Random velocity with upward bias
                particles[i].vx = ((float)(rand() % 100) / 100.0f - 0.5f) * 0.3f;
                particles[i].vy = ((float)(rand() % 100) / 100.0f - 0.5f) * 0.3f;
                particles[i].vz = ((float)(rand() % 100) / 100.0f) * 0.4f + 0.1f;
                
                particles[i].active = 1;
                particles[i].age = 0;
                break; // Only create one new particle per frame
            }
        }
    }
    
    // Update and draw all particles
    for (uint8_t i = 0; i < MAX_PARTICLES; i++) {
        if (particles[i].active) {
            // Update position
            particles[i].x += particles[i].vx;
            particles[i].y += particles[i].vy;
            particles[i].z += particles[i].vz;
            
            // Apply gravity
            particles[i].vz -= 0.03f;
            
            // Age the particle
            particles[i].age++;
            
            // Fix and complete the fountain pattern
            if (particles[i].x < 0 || particles[i].x >= CUBE_SIZE ||
                particles[i].y < 0 || particles[i].y >= CUBE_SIZE ||
                particles[i].z < 0 || particles[i].z >= PLANE_COUNT ||
                particles[i].age > 50) {
                particles[i].active = 0;    
                continue; // Add semicolon here
            }

            // Draw the particle if it's still active
            int px = (int)particles[i].x;
            int py = (int)particles[i].y;
            int pz = (int)particles[i].z;
            cube_data[pz][py][px] = 1;
        }
    }
}
