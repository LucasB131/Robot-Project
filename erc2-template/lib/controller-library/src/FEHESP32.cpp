/**
 * @file FEHESP32.cpp
 * @author Jayson Clark
 * @brief ESP32 co-processor communication driver implementation
 *
 * This file implements the FEHESP32 class which manages communication with
 * the ESP32 co-processor on the ERC2 Shield. It includes:
 * - Custom UART implementation with larger buffers
 * - Binary packet protocol parser
 * - Power management and initialization
 * - RCS data forwarding
 */

#include "../private_include/FEHInternal.h"
#include "../private_include/FEHESP32.h"
#include "../private_include/scheduler.h"
#include <avr/sfr_defs.h>
#include <wiring_private.h>
#include <FEHLCD.h>
#include <util/atomic.h>

//=============================================================================
// FORWARD DECLARATIONS
//=============================================================================

/// @brief Wrapper function for packet processing scheduled by the task scheduler
static void processPacketsWrapper();

//=============================================================================
// CUSTOM UART IMPLEMENTATION
//=============================================================================
//
// We implement our own UART driver because the Arduino HardwareSerial library
// has fixed buffer sizes (64 bytes) which are insufficient for high-speed
// communication with the ESP32. This custom implementation uses 256-byte
// buffers to prevent data loss.
//
// The implementation is based on the Arduino Core HardwareSerial code but
// modified to work specifically with USART1 on the ATmega2560.
//=============================================================================

//-----------------------------------------------------------------------------
// USART1 Register Definitions (ATmega2560)
//-----------------------------------------------------------------------------

#define USART_RX_vect USART1_RX_vect ///< USART1 receive complete interrupt vector
#define UBRRH UBRR1H                 ///< USART1 baud rate register high byte
#define UBRRL UBRR1L                 ///< USART1 baud rate register low byte
#define UCSRA UCSR1A                 ///< USART1 control and status register A
#define UCSRB UCSR1B                 ///< USART1 control and status register B
#define UCSRC UCSR1C                 ///< USART1 control and status register C
#define UDR UDR1                     ///< USART1 data register

//-----------------------------------------------------------------------------
// UART Buffer Configuration
//-----------------------------------------------------------------------------

/// @brief Size of receive buffer (must be power of 2 for efficient modulo)
#define RX_BUFFER_SIZE 256

/// @brief Size of transmit buffer (must be power of 2 for efficient modulo)
#define TX_BUFFER_SIZE 256

//-----------------------------------------------------------------------------
// UART Buffer Variables
//-----------------------------------------------------------------------------

/// @brief Circular receive buffer
volatile static uint8_t rx_buffer[RX_BUFFER_SIZE];

/// @brief Receive buffer write index (updated by ISR)
volatile static uint16_t rx_buffer_head = 0;

/// @brief Receive buffer read index (updated by main code)
volatile static uint16_t rx_buffer_tail = 0;

/// @brief Circular transmit buffer
volatile static uint8_t tx_buffer[TX_BUFFER_SIZE];

/// @brief Transmit buffer write index (updated by main code)
volatile static uint16_t tx_buffer_head = 0;

/// @brief Transmit buffer read index (updated by ISR)
volatile static uint16_t tx_buffer_tail = 0;

/// @brief Flag indicating if data has been written to UART
volatile static bool _written = false;

//=============================================================================
// FEHESP32 STATIC MEMBER INITIALIZATION
//=============================================================================

// Public status variables
bool FEHESP32::isReady = false;
uint8_t FEHESP32::firmwareMajor = 0;
uint8_t FEHESP32::firmwareMinor = 0;
uint8_t FEHESP32::firmwarePatch = 0;

// Private state variables
void (*FEHESP32::_rcsHandler)(uint8_t, uint8_t) = nullptr;
uint8_t FEHESP32::_currentStatus = STATUS_STARTING;
PacketState FEHESP32::_packetState = WAIT_MAGIC;
uint8_t FEHESP32::_currentCommand = 0;
uint8_t FEHESP32::_packetData[MAX_PACKET_DATA_SIZE];
uint8_t FEHESP32::_dataIndex = 0;

//=============================================================================
// CUSTOM UART LOW-LEVEL FUNCTIONS
//=============================================================================

/**
 * @brief Initialize USART1 for ESP32 communication
 *
 * Configures USART1 with the specified baud rate and format. Attempts to use
 * double-speed mode (U2X) for better baud rate accuracy, falling back to
 * normal mode if necessary.
 *
 * @param baud Desired baud rate (e.g., 115200)
 * @param config UART configuration byte (data bits, parity, stop bits)
 */
static void esp_serial_begin(unsigned long baud, byte config)
{
    // Calculate baud rate setting for double-speed mode (U2X = 1)
    // Formula: UBRR = (F_CPU / (8 * BAUD)) - 1
    uint16_t baud_setting = (F_CPU / 4 / baud - 1) / 2;
    UCSRA = 1 << U2X0;

    // Special case: 57600 baud requires normal mode for bootloader compatibility
    // Also: baud_setting must fit in 12 bits (max 4095)
    if (((F_CPU == 16000000UL) && (baud == 57600)) || (baud_setting > 4095))
    {
        // Switch to normal speed mode (U2X = 0)
        // Formula: UBRR = (F_CPU / (16 * BAUD)) - 1
        UCSRA = 0;
        baud_setting = (F_CPU / 8 / baud - 1) / 2;
    }

    // Configure baud rate registers
    UBRRH = baud_setting >> 8;   // High byte
    UBRRL = baud_setting & 0xFF; // Low byte

    // Configure frame format (data bits, parity, stop bits)
    UCSRC = config;

    // Enable UART receiver and transmitter
    sbi(UCSRB, RXEN0);  // Enable RX
    sbi(UCSRB, TXEN0);  // Enable TX
    sbi(UCSRB, RXCIE0); // Enable RX complete interrupt
    cbi(UCSRB, UDRIE0); // Disable data register empty interrupt (enabled when sending)
}

/**
 * @brief Check if data is available in receive buffer
 *
 * Calculates the number of bytes available to read from the circular receive
 * buffer by computing the difference between head and tail indices.
 *
 * @return Number of bytes available (0 if empty)
 */
static bool esp_serial_available(void)
{
    return ((unsigned int)(RX_BUFFER_SIZE + rx_buffer_head - rx_buffer_tail)) % RX_BUFFER_SIZE;
}

/**
 * @brief Read one byte from receive buffer
 *
 * Removes and returns the next byte from the circular receive buffer. This
 * is not synchronized with the ISR, but since we only read tail and the ISR
 * only writes head, no critical section is needed.
 *
 * @return Byte value (0-255) or -1 if buffer is empty
 */
static int esp_serial_read(void)
{
    // Check if buffer is empty
    if (rx_buffer_head == rx_buffer_tail)
    {
        return -1;
    }

    // Read byte from buffer and advance tail pointer
    unsigned char c = rx_buffer[rx_buffer_tail];
    rx_buffer_tail = (uint16_t)(rx_buffer_tail + 1) % RX_BUFFER_SIZE;
    return c;
}

/**
 * @brief Handle USART data register empty interrupt
 *
 * Called when the UART transmit data register is empty and ready for the next
 * byte. Sends the next byte from the transmit buffer, or disables the interrupt
 * if the buffer is empty.
 *
 * @note This is called from the UDRE ISR
 */
static void esp_tx_udr_empty_irq(void)
{
    // Read next byte from transmit buffer
    unsigned char c = tx_buffer[tx_buffer_tail];
    tx_buffer_tail = (tx_buffer_tail + 1) % TX_BUFFER_SIZE;

    // Write byte to UART data register
    UDR = c;

    // Clear the TXC (transmit complete) bit by writing 1 to it
    // This ensures flush() won't return until transmission is complete
    // Preserve U2X0 and MPCM0 bits, clear others
#ifdef MPCM0
    UCSRA = ((UCSRA) & ((1 << U2X0) | (1 << MPCM0))) | (1 << TXC0);
#else
    UCSRA = ((UCSRA) & ((1 << U2X0))) | (1 << TXC0);
#endif

    // If buffer is now empty, disable the data register empty interrupt
    if (tx_buffer_head == tx_buffer_tail)
    {
        cbi(UCSRB, UDRIE0);
    }
}

/**
 * @brief Write one byte to UART transmit buffer
 *
 * Adds a byte to the circular transmit buffer and enables the data register
 * empty interrupt to begin transmission. If the transmit buffer and UART data
 * register are both empty, writes directly to the data register for better
 * performance.
 *
 * This function blocks if the transmit buffer is full, waiting for the ISR
 * to free up space.
 *
 * @param c Byte to transmit
 * @return 1 on success
 */
static size_t esp_write(uint8_t c)
{
    _written = true;

    // Optimization: if buffer is empty and UART is ready, write directly
    // This significantly improves throughput at high baud rates (>500kbps)
    // by avoiding interrupt overhead
    if (tx_buffer_head == tx_buffer_tail && bit_is_set(UCSRA, UDRE0))
    {
        // Write to UDR and clear TXC atomically to prevent race conditions
        // with the ISR that could cause flush() to hang or return early
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            UDR = c;
#ifdef MPCM0
            UCSRA = ((UCSRA) & ((1 << U2X0) | (1 << MPCM0))) | (1 << TXC0);
#else
            UCSRA = ((UCSRA) & ((1 << U2X0))) | (1 << TXC0);
#endif
        }
        return 1;
    }

    // Calculate next buffer position
    uint16_t next_head = (tx_buffer_head + 1) % TX_BUFFER_SIZE;

    // Wait for space in buffer if full
    while (next_head == tx_buffer_tail)
    {
        if (bit_is_clear(SREG, SREG_I))
        {
            // Interrupts are disabled - manually call ISR handler if UART is ready
            // This prevents deadlock when interrupts are disabled
            if (bit_is_set(UCSRA, UDRE0))
            {
                esp_tx_udr_empty_irq();
            }
        }
        // else: ISR will free up space automatically
    }

    // Add byte to buffer
    tx_buffer[tx_buffer_head] = c;

    // Update head pointer and enable UDRE interrupt atomically
    // This prevents the ISR from running between updating the head pointer
    // and enabling the interrupt, which could cause a byte to be missed
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        tx_buffer_head = next_head;
        sbi(UCSRB, UDRIE0); // Enable data register empty interrupt
    }

    return 1;
}

/**
 * @brief USART1 receive complete interrupt service routine
 *
 * Called automatically by hardware when a byte is received on USART1. Reads
 * the byte from the UART data register and adds it to the circular receive
 * buffer if there's space available.
 *
 * Bytes with parity errors are discarded. If the buffer is full, the incoming
 * byte is dropped (data loss).
 */
ISR(USART_RX_vect)
{
    if (bit_is_clear(UCSRA, UPE0))
    {
        // No parity error - read byte from UART
        unsigned char c = UDR;

        // Calculate next buffer position
        uint16_t next_head = (unsigned int)(rx_buffer_head + 1) % RX_BUFFER_SIZE;

        // Only store byte if buffer has space
        // If next_head == tail, buffer would overflow, so drop the byte
        if (next_head != rx_buffer_tail)
        {
            rx_buffer[rx_buffer_head] = c;
            rx_buffer_head = next_head;
        }
        // else: buffer full - byte is lost
    }
    else
    {
        // Parity error detected - read and discard byte
        // Reading UDR clears the error flag
        (void)UDR;
    }
}

//=============================================================================
// FEHESP32 CLASS IMPLEMENTATION
//=============================================================================

//-----------------------------------------------------------------------------
// Initialization & Power Management
//-----------------------------------------------------------------------------

void FEHESP32::setup()
{
    // Configure all ESP32 interface pins
    pinMode(ESP32_PINS::CS, OUTPUT);      // SPI chip select (future use)
    pinMode(ESP32_PINS::EN, OUTPUT);      // ESP32 enable/power control
    pinMode(ESP32_PINS::BOOT_SEL, INPUT); // Boot mode select (also kill signal input)
    pinMode(ESP32_PINS::SPARE, OUTPUT);   // Factory reset trigger
    pinMode(ESP32_PINS::GPIO2, OUTPUT);   // General purpose IO

    // Set SPARE low to prevent factory reset on boot
    // ESP32 checks this pin during startup - HIGH triggers factory reset
    digitalWrite(ESP32_PINS::SPARE, LOW);

    // Power off ESP32 until begin() is called
    digitalWrite(ESP32_PINS::EN, LOW);
}

void FEHESP32::begin(unsigned long baudRate)
{
    // Reset all status variables to initial state
    isReady = false;
    firmwareMajor = 0;
    firmwareMinor = 0;
    firmwarePatch = 0;

    // Initialize USART1 for ESP32 communication
    // Default: 115200 baud, 8 data bits, no parity, 1 stop bit
    esp_serial_begin(baudRate, SERIAL_8N1);

    // Ensure ESP32 starts in a known state
    digitalWrite(ESP32_PINS::EN, LOW);    // Power off
    digitalWrite(ESP32_PINS::SPARE, LOW); // Disable factory reset trigger
    digitalWrite(ESP32_PINS::CS, HIGH);   // Deassert SPI chip select

    // Power on the ESP32
    digitalWrite(ESP32_PINS::EN, HIGH);

    // Wait for ESP32 power to stabilize before enabling interrupt
    // The BOOT_SEL interrupt must not be enabled while ESP32 is off
    delay(100);

    // Attach kill signal interrupt
    // ESP32 pulls BOOT_SEL low when RCS commands a kill
    attachInterrupt(digitalPinToInterrupt(ESP32_PINS::BOOT_SEL), _kill_esp, FALLING);

    // Schedule periodic packet processing (every 10ms)
    // This runs in the background to parse incoming ESP32 data
    scheduleEvent(processPacketsWrapper, SCHEDULER_MS_TO_TICKS(10));
}

void FEHESP32::factoryReset()
{
    // Clear all status variables
    isReady = false;
    firmwareMajor = 0;
    firmwareMinor = 0;
    firmwarePatch = 0;

    // Assert factory reset trigger
    // ESP32 firmware checks this pin during boot - HIGH = perform factory reset
    digitalWrite(ESP32_PINS::SPARE, HIGH);

    // Power cycle the ESP32 with reset trigger asserted
    digitalWrite(ESP32_PINS::EN, LOW);  // Power off
    delay(100);                         // Allow power to drain
    digitalWrite(ESP32_PINS::EN, HIGH); // Power on - ESP32 will detect SPARE high

    // Wait for ESP32 to complete factory reset and boot
    // This takes approximately 1-2 seconds
    delay(1500);

    // Release factory reset trigger
    digitalWrite(ESP32_PINS::SPARE, LOW);
}

//-----------------------------------------------------------------------------
// Data Transmission Methods
//-----------------------------------------------------------------------------

void FEHESP32::sendByte(uint8_t byte)
{
    esp_write(byte);
}

void FEHESP32::sendChar(char c)
{
    esp_write((uint8_t)c);
}

void FEHESP32::send(const String &data)
{
    // Send each character of the string
    for (size_t i = 0; i < data.length(); i++)
    {
        sendChar(data[i]);
    }
}

void FEHESP32::sendln(const String &data)
{
    // Send string followed by newline
    send(data);
    sendChar('\n');
}

//-----------------------------------------------------------------------------
// Data Reception Methods
//-----------------------------------------------------------------------------

char FEHESP32::readChar()
{
    // Block until data is available
    while (!esp_serial_available())
    {
        // Busy wait - consider adding timeout in future
    }

    return (char)esp_serial_read();
}

bool FEHESP32::available()
{
    return esp_serial_available() > 0;
}

//-----------------------------------------------------------------------------
// Packet Processing
//-----------------------------------------------------------------------------

void FEHESP32::processPackets()
{
    // Process all available bytes in the receive buffer
    while (available())
    {
        uint8_t byte = (uint8_t)esp_serial_read();

        // State machine for binary packet parsing
        // Packet format: [MAGIC_BYTE][COMMAND][DATA...][TERMINATOR]
        switch (_packetState)
        {
        case WAIT_MAGIC:
            // Looking for packet start marker (0xFE)
            if (byte == PACKET_MAGIC_BYTE)
            {
                _packetState = WAIT_CMD;
                _dataIndex = 0; // Reset data buffer
            }
            // else: ignore byte, keep waiting for magic
            break;

        case WAIT_CMD:
            // Next byte after magic is the command ID
            _currentCommand = byte;
            _packetState = READ_DATA;
            break;

        case READ_DATA:
            // Read data bytes until terminator or buffer full
            if (byte == PACKET_TERMINATOR)
            {
                // Complete packet received - process it
                _packetState = PACKET_READY;
                processBinaryPacket();
                _packetState = WAIT_MAGIC; // Ready for next packet
            }
            else if (_dataIndex < MAX_PACKET_DATA_SIZE)
            {
                // Add byte to data buffer
                _packetData[_dataIndex++] = byte;
            }
            else
            {
                // Buffer overflow - packet too large, abort parsing
                // This shouldn't happen with well-formed packets
                _packetState = WAIT_MAGIC;
                _dataIndex = 0;
            }
            break;

        case PACKET_READY:
            // Should never reach here - processBinaryPacket() resets to WAIT_MAGIC
            _packetState = WAIT_MAGIC;
            break;
        }
    }
}

void FEHESP32::processBinaryPacket()
{
    // Dispatch packet to appropriate handler based on command ID
    switch (_currentCommand)
    {
    case CMD_STATUS:
        // Status update (1 byte: status code)
        if (_dataIndex >= 1)
        {
            handleStatusPacket(_packetData[0]);
        }
        break;

    case CMD_DEBUG:
        // Debug message (variable length: ASCII text)
        handleDebugPacket(_packetData, _dataIndex);
        break;

    case CMD_RCS:
        // RCS data (2 bytes: type, value)
        if (_dataIndex >= 2)
        {
            handleRCSPacket(_packetData[0], _packetData[1]);
        }
        break;

    case CMD_FW_VERSION:
        // Firmware version (3 bytes: major, minor, patch)
        if (_dataIndex >= 3)
        {
            handleFirmwareVersion(_packetData[0], _packetData[1], _packetData[2]);
        }
        break;

    default:
        // Unknown command - silently ignore
        // Could log this for debugging in the future
        break;
    }
}

//-----------------------------------------------------------------------------
// Packet Handler Methods
//-----------------------------------------------------------------------------

void FEHESP32::handleStatusPacket(uint8_t statusValue)
{
    // Update internal status
    _currentStatus = statusValue;

    // Set ready flag only when ESP32 application is fully running
    // Other status values indicate transitional states (booting, updating, etc.)
    isReady = (statusValue == STATUS_APP_RUNNING);
}

void FEHESP32::handleDebugPacket(const uint8_t *data, uint8_t length)
{
    // Forward debug messages from ESP32 to ATmega serial console
    // This is useful for debugging ESP32 firmware issues
    Serial.print("ESP32 DEBUG: ");
    for (uint8_t i = 0; i < length; i++)
    {
        Serial.write(data[i]);
    }
    Serial.println();
}

void FEHESP32::handleRCSPacket(uint8_t rcsType, uint8_t value)
{
    // Forward RCS data to registered callback handler
    // If no handler is registered, silently discard the data
    if (_rcsHandler != nullptr)
    {
        _rcsHandler(rcsType, value);
    }
}

void FEHESP32::handleFirmwareVersion(uint8_t major, uint8_t minor, uint8_t patch)
{
    // Store firmware version for later queries
    firmwareMajor = major;
    firmwareMinor = minor;
    firmwarePatch = patch;
}

//-----------------------------------------------------------------------------
// RCS Interface
//-----------------------------------------------------------------------------

void FEHESP32::setRCSHandler(void (*handler)(uint8_t, uint8_t))
{
    // Register user callback for RCS data packets
    // Pass nullptr to unregister
    _rcsHandler = handler;
}

//-----------------------------------------------------------------------------
// Status & Version Query Methods
//-----------------------------------------------------------------------------

const char *FEHESP32::getFirmwareVersionString()
{
    // Static buffer persists across function calls
    static char versionBuffer[16];

    // Format version as "major.minor.patch"
    snprintf(versionBuffer, sizeof(versionBuffer), "%d.%d.%d",
             firmwareMajor, firmwareMinor, firmwarePatch);

    return versionBuffer;
}

const char *FEHESP32::getStatusString()
{
    // Convert numeric status code to human-readable string
    // Used for displaying ESP32 state on LCD or serial console
    switch (_currentStatus)
    {
    case STATUS_APP_RUNNING:
        return "ESP32: Ready";
    case STATUS_DOWNLOADING:
        return "ESP32: Downloading...";
    case STATUS_FLASHING:
        return "ESP32: Flashing...";
    case STATUS_REBOOT:
        return "ESP32: Rebooting...";
    case STATUS_CONNECTING:
        return "ESP32: Connecting WiFi...";
    case STATUS_STARTING:
        return "ESP32: Starting up...";
    case STATUS_WIFI_CONNECTED:
        return "ESP32: WiFi connected";
    case STATUS_SERVER_FAILED:
        return "ESP32: Server unavailable";
    case STATUS_UPDATE_FAILED:
        return "ESP32: Update failed";
    case STATUS_UPDATE_SUCCESS:
        return "ESP32: Update successful";
    case STATUS_ERROR:
        return "ESP32: Error";
    default:
        return "ESP32: Unknown status";
    }
}

uint8_t FEHESP32::getStatusCode()
{
    return _currentStatus;
}

//=============================================================================
// SCHEDULER INTEGRATION
//=============================================================================

/**
 * @brief Wrapper function for periodic packet processing
 *
 * This function is called by the task scheduler every 10ms to process incoming
 * packets from the ESP32. It calls the packet processor and then reschedules
 * itself for the next execution.
 *
 * @note This runs in the main loop context, not in an ISR
 */
static void processPacketsWrapper()
{
    // Process any available packets
    FEHESP32::processPackets();

    // Reschedule for next execution (10ms = ~156 scheduler ticks at 64µs/tick)
    scheduleEvent(processPacketsWrapper, SCHEDULER_MS_TO_TICKS(10));
}
