/**
 * FEHBuzzer.h
 */

#ifndef FEHBUZZER_H
#define FEHBUZZER_H

/**
 * @brief Access to the buzzer
 *
 * Allows the controller to create sound
 */
class FEHBuzzer
{
public:
    /**
     * @brief Enumeration of all 88 frequencies on a piano
     *
     * Enumeration of all 88 frequencies on a piano <br/>
     * Letter = note <br/>
     * 's' = sharp <br/>
     * 'f' = flat <br/>
     * Number = octave
     *
     */
    typedef enum
    {
        NOTE_C8 = 4186,
        NOTE_B7 = 3951,
        NOTE_AS7 = 3729,
        NOTE_BF7 = 3729,
        NOTE_A7 = 3520,
        NOTE_GS7 = 3322,
        NOTE_AF7 = 3322,
        NOTE_G7 = 3136,
        NOTE_FS7 = 2960,
        NOTE_GF7 = 2960,
        NOTE_F7 = 2794,
        NOTE_E7 = 2637,
        NOTE_DS7 = 2489,
        NOTE_EF7 = 2489,
        NOTE_D7 = 2349,
        NOTE_CS7 = 2217,
        NOTE_DF7 = 2217,
        NOTE_C7 = 2093,
        NOTE_B6 = 1976,
        NOTE_AS6 = 1865,
        NOTE_BF6 = 1865,
        NOTE_A6 = 1760,
        NOTE_GS6 = 1661,
        NOTE_AF6 = 1661,
        NOTE_G6 = 1568,
        NOTE_FS6 = 1480,
        NOTE_GF6 = 1480,
        NOTE_F6 = 1397,
        NOTE_E6 = 1319,
        NOTE_DS6 = 1245,
        NOTE_EF6 = 1244,
        NOTE_D6 = 1175,
        NOTE_CS6 = 1109,
        NOTE_DF6 = 1109,
        NOTE_C6 = 1047,
        NOTE_B5 = 988,
        NOTE_AS5 = 932,
        NOTE_BF5 = 932,
        NOTE_A5 = 880,
        NOTE_GS5 = 831,
        NOTE_AF5 = 831,
        NOTE_G5 = 784,
        NOTE_FS5 = 740,
        NOTE_GF5 = 740,
        NOTE_F5 = 698,
        NOTE_E5 = 659,
        NOTE_DS5 = 622,
        NOTE_EF5 = 622,
        NOTE_D5 = 587,
        NOTE_CS5 = 554,
        NOTE_DF5 = 554,
        NOTE_C5 = 523,
        NOTE_B4 = 494,
        NOTE_AS4 = 466,
        NOTE_BF4 = 466,
        NOTE_A4 = 440,
        NOTE_GS4 = 415,
        NOTE_AF4 = 415,
        NOTE_G4 = 392,
        NOTE_FS4 = 370,
        NOTE_GF4 = 370,
        NOTE_F4 = 349,
        NOTE_E4 = 330,
        NOTE_DS4 = 311,
        NOTE_EF4 = 311,
        NOTE_D4 = 294,
        NOTE_CS4 = 277,
        NOTE_DF4 = 277,
        NOTE_C4 = 261,
        NOTE_B3 = 247,
        NOTE_AS3 = 233,
        NOTE_BF3 = 233,
        NOTE_A3 = 220,
        NOTE_GS3 = 208,
        NOTE_AF3 = 208,
        NOTE_G3 = 196,
        NOTE_FS3 = 185,
        NOTE_GF3 = 185,
        NOTE_F3 = 175,
        NOTE_E3 = 165,
        NOTE_DS3 = 156,
        NOTE_EF3 = 156,
        NOTE_D3 = 147,
        NOTE_CS3 = 139,
        NOTE_DF3 = 139,
        NOTE_C3 = 131,
        NOTE_B2 = 123,
        NOTE_AS2 = 117,
        NOTE_BF2 = 117,
        NOTE_A2 = 110,
        NOTE_GS2 = 104,
        NOTE_AF2 = 104,
        NOTE_G2 = 98,
        NOTE_FS2 = 92,
        NOTE_GF2 = 92,
        NOTE_F2 = 87,
        NOTE_E2 = 82,
        NOTE_DS2 = 78,
        NOTE_EF2 = 78,
        NOTE_D2 = 73,
        NOTE_CS2 = 69,
        NOTE_DF2 = 69,
        NOTE_C2 = 65,
        NOTE_B1 = 62,
        NOTE_AS1 = 58,
        NOTE_BF1 = 58,
        NOTE_A1 = 55,
        NOTE_GS1 = 52,
        NOTE_AF1 = 52,
        NOTE_G1 = 49,
        NOTE_FS1 = 46,
        NOTE_GF1 = 46,
        NOTE_F1 = 44,
        NOTE_E1 = 41,
        NOTE_DS1 = 39,
        NOTE_EF1 = 39,
        NOTE_D1 = 37,
        NOTE_CS1 = 35,
        NOTE_DF1 = 35,
        NOTE_C1 = 33,
        NOTE_B0 = 31,
        NOTE_AS0 = 29,
        NOTE_BF0 = 29,
        NOTE_A0 = 28
    } stdnote;

    /**
     * @brief Beeps for 500 miliseconds at a frequency of 1000 Hz
     */
    void Beep();

    /**
     * @brief Beeps infinitely at a frequency of 1000 Hz
     */
    void Buzz();

    /**
     * @brief Beeps for a user specified amount of time (in milliseconds) at a frequency of 1000 Hz
     *
     * @param int Amount of time (milliseconds)
     */
    void Buzz(int);

    /**
     * @brief Beeps infinitely at a user specfied frequency
     *
     * @param int Frequency
     */
    void Tone(int);

    /**
     * @brief Beeps for a user specified amount of time (in milliseconds) at a a user specfied frequency

     * @param int Frequency
     * @param int Amount of time (milliseconds)
     */
    void Tone(int, int);

    /**
     * @brief Turns off buzzer
     *
     * Turns off buzzer
     *
     */
    void Off();

    /**
     * @brief Construct a new FEHBuzzer object
     *
     * Contains enumeration of all 88 frequencies on a piano
     *
     */
    FEHBuzzer();
};

extern FEHBuzzer Buzzer;

#endif // FEHBUZZER_H