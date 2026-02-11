/**
 * FEHIO.h
 */

#ifndef FEHIO_H
#define FEHIO_H

#include <stdint.h>
#define ENCODER_USE_INTERRUPTS
#include <Encoder.h>

/**
 * @brief API to be used with the 16 Student I/O Pins on the controller.
 *
 * Only 8-14 can be used for DigitalEncoder.
 *
 */

class FEHIO
{
public:
    /* NOTE TO DEVELOPERS: STUDENT IO PIN NAMING DOES NOT CORRESPOND TO PORT NAMING */
    typedef enum
    {
        Pin0 = 0,
        Pin1,
        Pin2,
        Pin3,
        Pin4,
        Pin5,
        Pin6,
        Pin7,
        Pin8,
        Pin9,
        Pin10,
        Pin11,
        Pin12,
        Pin13,
        Pin14,
        Pin15,
    } FEHIOPin;
};

/**
 * @brief Use any of 16 Student I/O pins as a digital input.
 *
 */
class DigitalInputPin
{
public:
    DigitalInputPin(FEHIO::FEHIOPin pin, bool usePullup = true);
    /**
     * @brief Returns the value of the DigitalInputPin.
     *  Recall that most of our digital sensors use pull-up resistors, so they return a zero when engaged.
     *
     * @return true the value of the associated DigitalInputPin
     */
    bool Value();

private:
    uint8_t _arduinoPin;
};


class DigitalOutputPin
{
public:
    DigitalOutputPin(FEHIO::FEHIOPin pin);
    /**
    * @brief Sets the value of the DigitalOutputPin.
    *
    * @param value
    *    True or false value of the DigitalOutputPin
    */
    void Write(bool value);

    /**
    * @brief Returns the value of the DigitalOutputPin.
    *
    * @return boolean value of associated DigitalOutputPin
    */
    bool Status();

    /**
    * @brief Switch the value of the DigitalOutputPin from true to false, or vice versa
    *
    */
    void Toggle();




private:
    uint8_t _arduinoPin;
};


/**
 * @brief Use any of the Student I/O pins A0-A7 and B0-B5 as an analog input.
 *
 */
class AnalogInputPin
{
public:
    AnalogInputPin(FEHIO::FEHIOPin pin, bool usePullup = false);
    /**
     * @brief Returns the value of the AnalogInputPin
     *
     * @return the value (0 to 5V) of the associated analog input pin
     */
    float Value();

private:
    uint8_t _arduinoPin;
};

class DigitalEncoder
{
public:
    DigitalEncoder(FEHIO::FEHIOPin pin);
    /**
     * @brief Returns the number of encoder ticks since the last call to ResetCount().
     *
     * @return the number of encoder ticks since the last call to ResetCount()
     */
    int Counts();

    /**
     * @brief Resets the count of the encoder to zero.
     *
     */
    void ResetCounts();

private:
    uint8_t _arduinoPin;
    uint8_t _portKpin;
};

class DigitalQuadratureEncoder
{
public:
    DigitalQuadratureEncoder(FEHIO::FEHIOPin pin1, FEHIO::FEHIOPin pin2);
    /**
     * @brief Returns the number of encoder ticks since the last call to ResetCount().
     * @note This is just a wrapper class for PJRC's Encoder library.
     * @return the number of encoder ticks since the last call to ResetCount()
     */
    int Counts();

    /**
     * @brief Resets the count of the encoder to zero.
     *
     */
    void ResetCounts();

private:
    Encoder *_encoder;
};

#endif // FEHIO_H