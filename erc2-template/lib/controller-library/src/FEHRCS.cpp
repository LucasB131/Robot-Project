/**
 * @file FEHRCS.cpp
 * @author Jayson Clark
 * @brief Support for the Robot Communication System (RCS) on the FEH Shield.
 *
 */

#include <FEH.h>

#include "../private_include/FEHInternal.h"
#include "../private_include/scheduler.h"
#include "../private_include/FEHESP32.h"

#define REGION_COUNT 8

// Global instance of FEHRCS
FEHRCS RCS;

/**
 * @brief Initializes the touch menu and starts the FEHRCS system.
 *
 * This function is intended to use FEHLCD to allow the user to select the region.
 * Currently, it defaults to region 'A'.
 *
 * @param team_key The team key used for authentication or identification.
 */
void FEHRCS::InitializeTouchMenu(const char *team_key)
{
    int cancel = 1;
    int c = 0, d = 0, n;
    int x, y;
    char region;

    FEHIcon::Icon regions_title[1];
    char regions_title_label[1][20] = {"Select RCS Region"};

    FEHIcon::Icon regions[REGION_COUNT];
    char regions_labels[12][20] = {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"};

    FEHIcon::Icon confirm_title[1];
    char confirm_title_label[1][20] = {""};

    FEHIcon::Icon confirm[2];
    char confirm_labels[2][20] = {"Ok", "Cancel"};

    while (cancel)
    {
        c = 0;
        d = 0;
        LCD.Clear(BLACK);

        int regionLabelRowCount = REGION_COUNT / 4;

        FEHIcon::DrawIconArray(regions_title, 1, 1, 1, 201, 1, 1, regions_title_label, BLACK, WHITE);
        FEHIcon::DrawIconArray(regions, regionLabelRowCount, 4, 40, 2, 1, 1, regions_labels, WHITE, WHITE);

        // Wait for region selection
        while (!c)
        {
            if (LCD.Touch(&x, &y))
            {
                for (n = 0; n < REGION_COUNT; n++)
                {
                    if (regions[n].Pressed(x, y, 0))
                    {
                        regions[n].WhilePressed(x, y);
                        c = n + 1; // c now holds the index (1-indexed)
                    }
                }
            }
        }

        // Compute region character and update the confirm title without switch-case
        region = 'A' + c - 1;
        sprintf(confirm_title_label[0], "Choice: %c", region);

        LCD.Clear(BLACK);
        FEHIcon::DrawIconArray(confirm_title, 1, 1, 60, 201, 1, 1, confirm_title_label, BLACK, WHITE);
        FEHIcon::DrawIconArray(confirm, 1, 2, 60, 60, 1, 1, confirm_labels, WHITE, WHITE);

        // Wait for confirmation selection
        while (!d)
        {
            if (LCD.Touch(&x, &y))
            {
                for (n = 0; n < 2; n++)
                {
                    if (confirm[n].Pressed(x, y, 0))
                    {
                        confirm[n].WhilePressed(x, y);
                        d = n + 1;
                    }
                }
            }
        }

        // Set cancel based on selection: Ok (d==1) ends the loop, Cancel (d==2) restarts it.
        cancel = (d == 1) ? 0 : 1;
    }

    LCD.Clear();

    // TODO: Implement region selection using FEHLCD touch menu
    Initialize(region, team_key);
}

/**
 * @brief Initializes the FEHRCS system with the specified region and team key.
 *
 * This function resets the ESP32, establishes serial communication, waits for the ESP32
 * to signal readiness, and sends the necessary credentials and server information.
 *
 * @param region The selected course region to connect to.
 * @param team_key The team key used for authentication or identification.
 */
void FEHRCS::Initialize(char region, const char *team_key)
{

    // Check that region is in the range A-H
    if (region < 'A' || region > 'H')
    {
        _fatalError("Invalid region selected: " + region);
    }

    // Send region info to ESP32
    LCD.WriteLine("Sending region info.");

    FEHESP32::sendChar(region);

    LCD.Write("Sending team key: ");
    for (int i = 0; i < 9; i++)
    {
        LCD.Write(team_key[i]);
        FEHESP32::sendChar(team_key[i]);
    }

    LCD.WriteLine();

    // Set internal region variable
    _region = region;
    initialized = true;

    // Set up RCS packet handler with ESP32
    FEHESP32::setRCSHandler(handleRCSPacket);
}

// /**
//  * @brief Main loop function for FEHRCS.
//  *
//  * Continuously checks for incoming data from the ESP32 and processes it.
//  *
//  * TODO: this is meant to go in the main loop() function. we probably don't want to use the loop()
//  * function of the arduino? Maybe anytime data is sent by the esp32 it should interrupt us,
//  * we quickly handle it. Not sure yet.
//  *
//  */
// void FEHRCS::Loop()
// {
//     if (FEHESP32::available())
//     {
//     }

//     delay(10);
// }

int FEHRCS::CurrentCourse()
{
    if (!initialized)
    {
        _fatalError("FEHRCS not initialized and FEHRCS::CurrentCourse() called.");
    }
    return (int)(_region - 'A');
}

char FEHRCS::CurrentRegionLetter()
{
    if (!initialized)
    {
        _fatalError("FEHRCS not initialized and FEHRCS::CurrentRegionLetter() called.");
    }
    return _region;
}

void FEHRCS::handleRCSPacket(uint8_t rcsType, uint8_t value)
{
    switch (rcsType)
    {
        case RCS_OBJECTIVE:
            RCS._correctLever = value;
            RCS.hasLever = true;
            break;
            
        case RCS_LEVER:
            RCS._leverFlipped = value;
            break;
            
        case RCS_SLIDER:
            RCS._dualSliderStatus = value;
            break;
            
        case RCS_TIME:
            RCS._time = value;
            break;
            
        default:
            // Unknown RCS type - ignore
            break;
    }
}

int FEHRCS::GetLever()
{
    if (!initialized)
    {
        _fatalError("FEHRCS not initialized and FEHRCS::GetLever() called.");
    }
    return _correctLever;
}

int FEHRCS::isLeverFlipped()
{
    if (!initialized)
    {
        _fatalError("FEHRCS not initialized and FEHRCS::isLeverFlipped() called.");
    }
    if (RCS._leverFlipped == 1)
    {
        return 1;
    }
    return 0;
}

int FEHRCS::isWindowOpen()
{
    if (!initialized)
    {
        _fatalError("FEHRCS not initialized and FEHRCS::isWindowOpen() called.");
    }
    if (RCS._dualSliderStatus == 2)
    {
        return 1;
    }
    return 0;
}

// returns the match time in seconds
int FEHRCS::Time()
{
    if (!initialized)
    {
        _fatalError("FEHRCS not initialized and FEHRCS::Time() called.");
    }
    return (int)RCS._time;
}