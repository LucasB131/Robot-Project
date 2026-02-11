# FEH Controller Gen 2 Firmware

Libraries for the FEH Robot Controller Gen 2, based off the Arduino Mega 2560 with a custom-developed shield.  
The API is is designed to replicate the Proteus API as closely as possible.

The directory `lib/controller-library/private_include` ensures that headers in `private_include` are not visible to students.
Include flags in `lib/controller-library/library.json` cannot be used for private includes because `library.json` include flags are propagated outwards to programs that depend on the library.  

**IF YOU MODIFY ANY OF THIS MOTOR DRIVER/SERVO PWM CODE, PLEASE USE AN OSCILLOSCOPE TO VERIFY FUNCTIONALITY.**

## Timer Allocation
Timers have been strategically allocated to maximize functionality, and libraries modified to make them control a specific timer.
| Timer | Counter Bits | Timer Use                                                      | Controlled by... |
|-------|--------------|----------------------------------------------------------------|------------------|
| 0     |     8-bit    | Reserved for Arduino API millis(), micros() and delay()        | Arduino library  |
| 1     |    16-bit    | Servo control                                                  | Servo.h library  |
| 2     |     8-bit    | Buzzer                                                         | FEH.cpp library  |
| 3     |    16-bit    | Motor PWM                                                      | FEH.cpp library  |
| 4     |    16-bit    | Low-frequency scheduled events (incl. automated health checks) | FEH.cpp library  |
| 5     |    16-bit    | Motor PWM                                                      | FEH.cpp library  |

**For information on how to write code for AVR timers, please see the application note below.**
> *AVR130: Setup and Use of AVR Timers.*  
> https://ww1.microchip.com/downloads/en/AppNotes/Atmel-2505-Setup-and-Use-of-AVR-Timers_ApplicationNote_AVR130.pdf

## ESP32 Communication Protocol

The shield includes an ESP32 co-processor that handles wireless communication for the Robot Communication System (RCS). Communication between the ATmega2560 and ESP32 uses a compact binary protocol over UART.

### Binary Packet Format
```
0xFE [CMD_BYTE] [DATA_BYTES...] 0x0A
```
- **Magic Byte**: `0xFE` - Prevents accidental command triggering from bootloader/noise
- **Command Byte**: Identifies the packet type
- **Data Bytes**: Variable length payload (command-specific)
- **Terminator**: `0x0A` (\n) - End of packet marker

### Command Types

#### Status Commands (CMD = 0x01)
**Format**: `0xFE 0x01 [STATUS_VALUE] 0x0A`

Reports ESP32 operational status for display during initialization.

| Status Value | Meaning | Description |
|--------------|---------|-------------|
| `0x01` | APP_RUNNING | ESP32 application is running normally |
| `0x02` | DOWNLOADING | ESP32 is downloading OTA firmware update |
| `0x03` | REBOOT | ESP32 is rebooting |
| `0x04` | FLASHING | ESP32 is flashing new firmware |
| `0x05` | CONNECTING | ESP32 is establishing network connection |
| `0x10` | STARTING | ESP32 is starting up |
| `0x11` | WIFI_CONNECTED | ESP32 has connected to WiFi |
| `0x12` | SERVER_FAILED | ESP32 cannot reach update server |
| `0x13` | UPDATE_FAILED | ESP32 firmware update failed |
| `0x14` | UPDATE_SUCCESS | ESP32 firmware update completed successfully |
| `0xFF` | ERROR | ESP32 encountered an error |

**Example**: `0xFE 0x01 0x02 0x0A` = ESP32 is downloading update

#### Debug Commands (CMD = 0x02)
**Format**: `0xFE 0x02 [MESSAGE_BYTES...] 0x0A`

Debug messages from ESP32 are forwarded to Arduino Serial output for debugging.

**Example**: `0xFE 0x02 'H' 'e' 'l' 'l' 'o' 0x0A` → Serial prints "ESP32 DEBUG: Hello"

#### RCS Commands (CMD = 0x10)
**Format**: `0xFE 0x10 [RCS_TYPE] [VALUE] 0x0A`

Robot Communication System data packets containing competition information.

| RCS Type | Value Meaning | Description |
|----------|---------------|-------------|
| `0x01` (OBJECTIVE) | 0=left, 1=middle, 2=right | Which lever to flip for current objective |
| `0x02` (LEVER) | 0=not flipped, 1=flipped | Current state of competition lever |
| `0x03` (SLIDER) | 0=closed, 1=partial, 2=open | Current state of competition window |
| `0x04` (TIME) | 0-255 seconds | Match time remaining |

**Examples**:
- Objective (middle lever): `0xFE 0x10 0x01 0x01 0x0A`
- Lever flipped: `0xFE 0x10 0x02 0x01 0x0A`  
- Time 85 seconds: `0xFE 0x10 0x04 0x55 0x0A`

#### Firmware Version (CMD = 0x03)
**Format**: `0xFE 0x03 [MAJOR] [MINOR] 0x0A`

ESP32 firmware version information.

**Example**: `0xFE 0x03 0x02 0x01 0x0A` = Version 2.1

### Packet Processing

The ESP32 library automatically processes incoming packets every 10ms using Timer 4 scheduling:
- **State Machine Parser**: Robust parsing with buffer overflow protection
- **Automatic Routing**: Status/Debug packets handled internally, RCS packets forwarded to RCS library
- **Error Recovery**: Invalid packets are discarded and parser resets automatically
