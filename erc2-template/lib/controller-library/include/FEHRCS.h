#ifndef FEHRCS_H
#define FEHRCS_H

#include <Arduino.h>

#include "../private_include/FEHESP32.h"

class FEHRCS
{
public:
    /**
     * @brief Setup system used to select RCS course
     *
     * You must be in range of the course to be able to successfully initialize.
     *
     */
    void InitializeTouchMenu(const char *team_key);

    /**
     * @brief Get course number corresponding to current region
     *
     * Get course number corresponding to current region
     *
     * @return int 1 (regions A-D), 2 (regions E-H)
     */
    int CurrentCourse();

    /**
     * @brief Get current region's corresponding letter
     *
     * Get current region's corresponding letter from
     * set { A, B, C, D, E, F, G, H}
     *
     * @return char Region letter RCS was initialized to
     */
    char CurrentRegionLetter();

    /**
     * @brief Get time remaining for a match
     *
     * Begins transmitting time remaining with 90 seconds left in the match.
     *
     * @return int Number of seconds remaining
     */
    int Time();

    // Objective functions:
    /**
     * @brief Get correct lever for task
     *
     * Get correct lever for task
     *
     * @return int 0 (left lever), 1 (middle lever), 2 (right lever)
     */
    int GetLever();

    /**
     * @brief Tells you if a lever has been flipped down but not up
     *
     * @return int 1 if lever has been flipped down, but not up. 0 otherwise
     */
    int isLeverFlipped();

    /**
     * @brief Tells you if the window is fully open or closed
     *
     * @return int 1 if window is fully open, 0 if closed
     */
    int isWindowOpen();

private:
    /// @brief Handler for RCS packets from ESP32
    static void handleRCSPacket(uint8_t rcsType, uint8_t value);
    void Initialize(char region, const char *team_key);

    bool initialized = false;
    char _region = 'z';

    volatile int _correctLever = 0; // Initialize this to a valid lever value so it cannot ALWAYS be used to figure out if the start light has gone off
    volatile int _leverFlipped = 0;
    volatile int _dualSliderStatus = 0;

    volatile int _time = 0;

    volatile bool hasLever = false;
};

// Declare RCS as an external variable
extern FEHRCS RCS;

#endif
