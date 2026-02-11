/**
 * @file FEHESP32.h
 * @author Jayson Clark
 * @brief ESP32 co-processor communication driver for the ERC2
 *
 * This class provides an interface to communicate with the ESP32 co-processor
 * on the ERC2 Shield. The ESP32 handles RCS (Robot Communication System)
 * Communication uses a binary packet protocol over UART.
 */

#ifndef FEHESP32_H
#define FEHESP32_H

#include <Arduino.h>

//=============================================================================
// PROTOCOL CONFIGURATION
//=============================================================================

/// @brief Timeout for ESP32 serial communication in milliseconds
#define ESP_SERIAL_TIMEOUT 5000

/// @brief Maximum size of packet data payload in bytes
#define MAX_PACKET_DATA_SIZE 64

//=============================================================================
// BINARY PROTOCOL CONSTANTS
//=============================================================================

/// @brief Magic byte that starts every packet (0xFE)
#define PACKET_MAGIC_BYTE 0xFE

/// @brief Byte that terminates every packet (newline)
#define PACKET_TERMINATOR 0x0A

//=============================================================================
// COMMAND DEFINITIONS
//=============================================================================

/// @brief Status update command
#define CMD_STATUS 0x01

/// @brief Debug message command
#define CMD_DEBUG 0x02

/// @brief Firmware version command
#define CMD_FW_VERSION 0x03

/// @brief Robot Communication System (RCS) data command
#define CMD_RCS 0x10

//=============================================================================
// ESP32 STATUS VALUES
//=============================================================================

/// @brief ESP32 application is running normally
#define STATUS_APP_RUNNING 0x01

/// @brief ESP32 is downloading an update
#define STATUS_DOWNLOADING 0x02

/// @brief ESP32 is rebooting
#define STATUS_REBOOT 0x03

/// @brief ESP32 is flashing firmware
#define STATUS_FLASHING 0x04

/// @brief ESP32 is connecting to WiFi
#define STATUS_CONNECTING 0x05

/// @brief ESP32 is starting up (initial boot, not actually sent by esp32)
#define STATUS_STARTING 0x10

/// @brief ESP32 successfully connected to WiFi
#define STATUS_WIFI_CONNECTED 0x11

/// @brief ESP32 failed to connect to server
#define STATUS_SERVER_FAILED 0x12

/// @brief ESP32 firmware update failed
#define STATUS_UPDATE_FAILED 0x13

/// @brief ESP32 firmware update succeeded
#define STATUS_UPDATE_SUCCESS 0x14

/// @brief ESP32 encountered an error
#define STATUS_ERROR 0xFF

//=============================================================================
// RCS (ROBOT COMMUNICATION SYSTEM) TYPES
//=============================================================================

/// @brief RCS objective/mission identifier
#define RCS_OBJECTIVE 0x01

/// @brief RCS lever position value
#define RCS_LEVER 0x02

/// @brief RCS slider position value
#define RCS_SLIDER 0x03

/// @brief RCS run time value
#define RCS_TIME 0x04

//=============================================================================
// PACKET PARSING STATE MACHINE
//=============================================================================

/**
 * @brief State machine states for binary packet parsing
 */
enum PacketState
{
    WAIT_MAGIC,  ///< Waiting for magic byte (0xFE)
    WAIT_CMD,    ///< Waiting for command byte
    READ_DATA,   ///< Reading data payload
    PACKET_READY ///< Complete packet received and ready to process
};

//=============================================================================
// HARDWARE PIN DEFINITIONS
//=============================================================================

/**
 * @brief Pin mappings for ESP32 interface on ATmega2560
 */
enum ESP32_PINS
{
    EN = 22,      ///< ESP32 enable pin (power control)
    BOOT_SEL = 2, ///< ESP32 boot mode select (interrupt pin for kill signal)
    CS = 40,      ///< ESP32 chip select for SPI (currently unused)
    SPARE = 39,   ///< Spare GPIO used for factory reset trigger
    GPIO2 = 50    ///< ESP32 GPIO2 (general purpose)
};

//=============================================================================
// FEHESP32 CLASS
//=============================================================================

/**
 * @brief ESP32 co-processor communication interface
 *
 * Static class that manages communication with the ESP32 co-processor on the
 * ERC2 Shield. Provides methods for sending/receiving data, processing binary
 * packets, and handling RCS (Robot Control System) updates.
 *
 * @note This is a static-only class and cannot be instantiated
 */
class FEHESP32
{
public:
    //=========================================================================
    // PUBLIC STATUS VARIABLES
    //=========================================================================

    /// @brief True when ESP32 is ready and application is running
    static bool isReady;

    /// @brief ESP32 firmware major version number
    static uint8_t firmwareMajor;

    /// @brief ESP32 firmware minor version number
    static uint8_t firmwareMinor;

    /// @brief ESP32 firmware patch version number
    static uint8_t firmwarePatch;

    //=========================================================================
    // INITIALIZATION
    //=========================================================================

    /**
     * @brief Configure ESP32 control pins
     *
     * Initializes all GPIO pins connected to the ESP32 and sets them to
     * their default states. The ESP32 is powered off after this call.
     * Must be called before begin().
     */
    static void setup();

    /**
     * @brief Initialize ESP32 communication
     *
     * Powers on the ESP32, initializes UART communication at the specified
     * baud rate, and sets up the packet processing scheduler. Also enables
     * the kill signal interrupt on the BOOT_SEL pin.
     *
     * @param baudRate UART baud rate (default: 115200)
     */
    static void begin(unsigned long baudRate = 115200);

    /**
     * @brief Trigger ESP32 factory reset
     *
     * Forces the ESP32 to perform a factory reset by holding the SPARE pin
     * high during boot. This will force the ESP32 to boot into the updater
     * partition, downloading new.
     *
     * @warning This will disconnect the ESP32 from WiFi and reset all settings
     */
    static void factoryReset();

    //=========================================================================
    // DATA TRANSMISSION METHODS
    //=========================================================================

    /**
     * @brief Send a string to ESP32 without newline
     *
     * Transmits a string to the ESP32 via UART. No terminating newline
     * character is added.
     *
     * @param data String to send
     */
    static void send(const String &data);

    /**
     * @brief Send a string to ESP32 with newline
     *
     * Transmits a string to the ESP32 via UART with a terminating newline
     * character appended.
     *
     * @param data String to send
     */
    static void sendln(const String &data);

    /**
     * @brief Send a single byte to ESP32
     *
     * Transmits one byte to the ESP32 via UART.
     *
     * @param byte Byte value to send
     */
    static void sendByte(uint8_t byte);

    /**
     * @brief Send a single character to ESP32
     *
     * Transmits one character to the ESP32 via UART.
     *
     * @param c Character to send
     */
    static void sendChar(char c);

    //=========================================================================
    // DATA RECEPTION METHODS
    //=========================================================================

    /**
     * @brief Read one character from ESP32
     *
     * Blocking call that waits until a character is available in the receive
     * buffer, then returns it.
     *
     * @return Character received from ESP32
     * @warning This function blocks until data is available
     */
    static char readChar();

    /**
     * @brief Check if data is available from ESP32
     *
     * Non-blocking check to see if there are any bytes in the receive buffer.
     *
     * @return true if data is available, false otherwise
     */
    static bool available();

    //=========================================================================
    // PACKET PROCESSING
    //=========================================================================

    /**
     * @brief Process incoming binary packets from ESP32
     *
     * Called periodically by the scheduler to parse incoming bytes from the
     * ESP32 and assemble them into complete binary packets. When a complete
     * packet is received, the appropriate handler is called.
     *
     * This method implements a state machine parser for the binary protocol.
     */
    static void processPackets();

    //=========================================================================
    // RCS (ROBOT COMMUNICATION SYSTEM) INTERFACE
    //=========================================================================

    /**
     * @brief Register callback for RCS data packets
     *
     * Sets a callback function that will be called whenever an RCS packet
     * is received from the ESP32. The callback receives the RCS data type
     * and value.
     *
     * @param handler Function pointer with signature: void(uint8_t rcsType, uint8_t value)
     *
     * @note Only one handler can be registered at a time
     * @note Pass nullptr to unregister the current handler
     */
    static void setRCSHandler(void (*handler)(uint8_t rcsType, uint8_t value));

    //=========================================================================
    // STATUS & VERSION QUERIES
    //=========================================================================

    /**
     * @brief Get human-readable status string
     *
     * Returns a string describing the current ESP32 status based on the
     * most recent status packet received.
     *
     * @return Pointer to static status string (e.g., "ESP32: Ready")
     */
    static const char *getStatusString();

    /**
     * @brief Get firmware version string
     *
     * Returns the ESP32 firmware version in "major.minor.patch" format.
     *
     * @return Pointer to static version string (e.g., "1.2.3")
     */
    static const char *getFirmwareVersionString();

    /**
     * @brief Get the current numeric status code.
     *
     * Returns the last status byte received from the ESP32. Matches
     * STATUS_* defines in this header (e.g., STATUS_DOWNLOADING).
     */
    static uint8_t getStatusCode();

private:
    //=========================================================================
    // PRIVATE STATE VARIABLES
    //=========================================================================

    /// @brief Callback function for RCS data packets
    static void (*_rcsHandler)(uint8_t rcsType, uint8_t value);

    /// @brief Current state of the packet parser state machine
    static PacketState _packetState;

    /// @brief Command byte of the packet currently being parsed
    static uint8_t _currentCommand;

    /// @brief Buffer for packet data payload
    static uint8_t _packetData[MAX_PACKET_DATA_SIZE];

    /// @brief Current write position in packet data buffer
    static uint8_t _dataIndex;

    /// @brief Most recent status value received from ESP32
    static uint8_t _currentStatus;

    //=========================================================================
    // PRIVATE PACKET HANDLING METHODS
    //=========================================================================

    /**
     * @brief Process a complete binary packet
     *
     * Called when a complete packet has been received. Dispatches to the
     * appropriate handler based on the command byte.
     */
    static void processBinaryPacket();

    /**
     * @brief Handle status update packet
     *
     * Processes a status packet from the ESP32, updating the isReady flag
     * and storing the current status.
     *
     * @param statusValue Status code received
     */
    static void handleStatusPacket(uint8_t statusValue);

    /**
     * @brief Handle debug message packet
     *
     * Processes a debug message packet from the ESP32 and prints it to
     * the serial console.
     *
     * @param data Pointer to debug message data
     * @param length Length of debug message in bytes
     */
    static void handleDebugPacket(const uint8_t *data, uint8_t length);

    /**
     * @brief Handle RCS data packet
     *
     * Processes an RCS data packet and forwards it to the registered
     * callback handler if one exists.
     *
     * @param rcsType Type of RCS data (objective, lever, slider, time)
     * @param value RCS value
     */
    static void handleRCSPacket(uint8_t rcsType, uint8_t value);

    /**
     * @brief Handle firmware version packet
     *
     * Processes a firmware version packet and updates the version variables.
     *
     * @param major Major version number
     * @param minor Minor version number
     * @param patch Patch version number
     */
    static void handleFirmwareVersion(uint8_t major, uint8_t minor, uint8_t patch);

    //=========================================================================
    // DELETED CONSTRUCTORS
    //=========================================================================

    /// @brief Prevent instantiation of this static-only class
    FEHESP32() = delete;

    /// @brief Prevent copying of this static-only class
    FEHESP32(const FEHESP32 &) = delete;

    /// @brief Prevent assignment of this static-only class
    FEHESP32 &operator=(const FEHESP32 &) = delete;
};

#endif // FEHESP32_H
