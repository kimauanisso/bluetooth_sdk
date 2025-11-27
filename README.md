# 📡 bt_kmn - Bluetooth Command Library for Raspberry Pi Pico

A lightweight C library for Raspberry Pi Pico that provides UART-based Bluetooth communication with an automatic command registration system using linker sections.

---

## 📋 Features

- ✅ **Simple UART Bluetooth Setup** - Easy initialization with custom baud rate and pins
- ✅ **Automatic Command Registration** - Commands registered at compile-time using macros
- ✅ **Zero Runtime Overhead** - Command discovery via linker sections (no dynamic allocation)
- ✅ **Flexible Command System** - Support for commands with or without parameters
- ✅ **C and C++ Compatible** - Works seamlessly with both languages
- ✅ **AT Commands Support** - Send AT commands to configure Bluetooth modules
- ✅ **Non-blocking Communication** - Efficient UART reading without blocking

---

## 🚀 Quick Start

### Installation

1. Add as a submodule to your Pico project:
```bash
git submodule add <repository-url> bluetooth_sdk
```

2. Include in your `CMakeLists.txt`:
```cmake
add_subdirectory(bluetooth_sdk)

target_link_libraries(your_project
    bt_kmn
    # ... other libraries
)
```

### Basic Usage

```cpp
#include "bt_kmn/bluetooth.h"
#include "bt_kmn/commands.h"

// Configure Bluetooth
BluetoothConfig bt_config = {
    .uart = uart1,
    .baudrate = 9600,
    .rx_pin = 9,
    .tx_pin = 8
};

int main() {
    stdio_init_all();
    
    // Initialize Bluetooth
    Bluetooth_Setup(bt_config);
    
    // Main loop - check for incoming commands
    while (true) {
        Bluetooth_ReadMessage();
        sleep_ms(10);
    }
}
```

---

## 📚 API Reference

### Core Functions

#### `Bluetooth_Setup(BluetoothConfig config)`
Initializes UART pins and Bluetooth communication.

**Parameters**:
- `config`: Configuration structure with UART instance, baud rate, and pin numbers

**Example**:
```c
BluetoothConfig config = {
    .uart = uart1,
    .baudrate = 9600,
    .rx_pin = 9,
    .tx_pin = 8
};
Bluetooth_Setup(config);
```

---

#### `Bluetooth_SendMessage(const char *message)`
Sends a string message over Bluetooth.

**Parameters**:
- `message`: Null-terminated string to send

**Example**:
```c
Bluetooth_SendMessage("Robot started!\n");
```

---

#### `Bluetooth_ReadMessage()`
Reads incoming data from UART and automatically dispatches commands.

**Behavior**:
- Non-blocking - returns immediately if no data
- Reads until `\n`, `\r`, or `\0` terminator
- Automatically calls registered command handlers
- Max command length: 128 bytes

**Example**:
```c
while (true) {
    Bluetooth_ReadMessage();  // Check for commands
    // Your code here
}
```

---

#### `Bluetooth_AtCommand(const char *command)`
Sends an AT command to the Bluetooth module (for configuration).

**Parameters**:
- `command`: AT command string (without `\r\n` - added automatically)

**Example**:
```c
Bluetooth_AtCommand("AT+NAME=MyRobot");  // Set BT name
Bluetooth_AtCommand("AT+BAUD4");         // Set 9600 baud
```

---

#### `Bluetooth_InterruptInit(irq_handler_t handler)`
Sets up UART interrupt for advanced use cases (optional).

**Parameters**:
- `handler`: Function pointer to interrupt handler

**Note**: Not required for basic usage with `Bluetooth_ReadMessage()`.

---

## 🎯 Command System

### Registering Commands

The library uses the `BT_COMMAND` macro to automatically register commands at compile-time.

#### Method 1: Separate Declaration (Recommended)

```cpp
// Forward declare command function
void my_command_handler(const char *params);

// Register command
BT_COMMAND(MYCOMMAND, my_command_handler)

// Implement command
void my_command_handler(const char *params) {
    if (params == NULL) {
        Bluetooth_SendMessage("No parameters\n");
    } else {
        Bluetooth_SendMessage("Received: ");
        Bluetooth_SendMessage(params);
        Bluetooth_SendMessage("\n");
    }
}
```

#### Method 2: Inline Definition

```cpp
BT_COMMAND_DEFINE(STATUS) {
    Bluetooth_SendMessage("System OK\n");
}
```

### Command Format

Commands sent via Bluetooth follow these formats:

| Format | Example | Description |
|--------|---------|-------------|
| `COMMAND\n` | `START\n` | Simple command without parameters |
| `COMMAND=params\n` | `SPEED=500\n` | Command with parameters (delimiter: `=`) |

**Note**: Parameters can contain spaces. Everything after `=` is passed to the handler.

### Parameter Parsing

The library passes the full parameter string to your handler. You're responsible for parsing:

```cpp
void cmd_pid(const char *params) {
    if (params == NULL) {
        // No parameters provided
        return;
    }
    
    float kp, ki, kd;
    int parsed = sscanf(params, "%f %f %f", &kp, &ki, &kd);
    
    if (parsed == 3) {
        // Successfully parsed 3 floats
        printf("Kp=%.2f Ki=%.2f Kd=%.2f\n", kp, ki, kd);
    }
}

BT_COMMAND(PID, cmd_pid)
```

**Usage**: Send `PID=0.2 0.01 0.005\n` via Bluetooth

---

## 🔧 How It Works

### Linker Sections Magic

The library uses GCC's linker sections to automatically collect all registered commands at compile-time:

```cpp
#define BT_COMMAND(name, method)                              \
  __attribute__((used))                                       \
  __attribute__((section("BT_COMMANDS")))                     \
  static const Command CONCAT(__bt_command_, name) = {        \
      .command = #name,                                       \
      .method = method                                        \
  };
```

**What happens**:
1. Each `BT_COMMAND` creates a `Command` struct in the `BT_COMMANDS` linker section
2. The linker automatically creates `__start_BT_COMMANDS` and `__stop_BT_COMMANDS` symbols
3. `__run_command()` iterates through this section to find and execute commands

**Benefits**:
- Zero runtime overhead
- No dynamic memory allocation
- Commands discovered automatically
- Type-safe at compile-time

---

## 📖 Complete Example

```cpp
#include <stdio.h>
#include "pico/stdlib.h"
#include "bt_kmn/bluetooth.h"
#include "bt_kmn/commands.h"

// Global state
bool robot_enabled = false;
int motor_speed = 0;

// START command
void cmd_start(const char *params) {
    robot_enabled = true;
    Bluetooth_SendMessage("Robot STARTED\n");
    printf("Robot started\n");
}

// STOP command
void cmd_stop(const char *params) {
    robot_enabled = false;
    motor_speed = 0;
    Bluetooth_SendMessage("Robot STOPPED\n");
    printf("Robot stopped\n");
}

// SPEED command (with parameters)
void cmd_speed(const char *params) {
    if (params == NULL) {
        // Query mode
        char msg[32];
        snprintf(msg, sizeof(msg), "Speed: %d\n", motor_speed);
        Bluetooth_SendMessage(msg);
    } else {
        // Set mode
        motor_speed = atoi(params);
        char msg[32];
        snprintf(msg, sizeof(msg), "Speed set to %d\n", motor_speed);
        Bluetooth_SendMessage(msg);
    }
}

// Register all commands
BT_COMMAND(START, cmd_start)
BT_COMMAND(STOP, cmd_stop)
BT_COMMAND(SPEED, cmd_speed)

int main() {
    stdio_init_all();
    
    // Configure Bluetooth
    BluetoothConfig bt_config = {
        .uart = uart1,
        .baudrate = 9600,
        .rx_pin = 9,
        .tx_pin = 8
    };
    
    Bluetooth_Setup(bt_config);
    sleep_ms(2000);
    
    printf("Bluetooth ready. Send commands via BT:\n");
    printf("  START     - Start robot\n");
    printf("  STOP      - Stop robot\n");
    printf("  SPEED     - Query speed\n");
    printf("  SPEED=500 - Set speed to 500\n");
    
    // Main loop
    while (true) {
        // Check for Bluetooth commands
        Bluetooth_ReadMessage();
        
        // Your robot logic here
        if (robot_enabled) {
            // Move robot at motor_speed...
        }
        
        sleep_ms(10);
    }
}
```

---

## 🔌 Supported Bluetooth Modules

Tested with:
- ✅ **HC-05** - Fully compatible
- ✅ **HC-06** - Fully compatible
- ✅ **JDY-31** - Compatible (tested)
- ⚠️ **HM-10** - Should work (BLE, untested)

### Module Configuration

Configure your module before use (via AT commands or separate tool):

```c
// Example: Configure HC-05
Bluetooth_AtCommand("AT");              // Test connection
Bluetooth_AtCommand("AT+NAME=MyRobot"); // Set device name
Bluetooth_AtCommand("AT+BAUD4");        // 9600 baud (default)
Bluetooth_AtCommand("AT+PIN1234");      // Set pairing PIN
```

**Note**: Most modules require entering AT mode (hold button/pin high during power-on).

---

## ⚙️ Configuration

### Buffer Size

The max command length is 128 bytes. To change:

```c
// In bluetooth.c
#define MAX_COMMAND_SIZE 256  // Increase if needed
```

### Command Delimiter

Commands and parameters are separated by `=` by default. To change:

```c
// In commands.c
#define COMMAND_DELIMITER " "  // Use space instead
```

---

## 🧪 Testing

### Serial Terminal Test

Use any serial terminal to test commands:

```bash
# Linux/Mac
screen /dev/ttyUSB0 9600

# Windows
# Use PuTTY or TeraTerm on COM port
```

Send test commands:
```
START
SPEED=500
STOP
```

### Python Test Script

```python
import serial
import time

ser = serial.Serial('COM3', 9600, timeout=1)
time.sleep(2)  # Wait for connection

# Send commands
ser.write(b'START\n')
print(ser.readline().decode())  # Read response

ser.write(b'SPEED=500\n')
print(ser.readline().decode())

ser.close()
```

---

## 🐛 Troubleshooting

### Commands Not Recognized

**Symptom**: Receiving "Command not found" messages

**Solutions**:
1. Verify command name matches exactly (case-sensitive)
2. Check that `BT_COMMAND` is called at file scope (not inside functions)
3. Ensure the file containing commands is compiled and linked
4. Check linker script has `BT_COMMANDS` section defined

### No Response from Module

**Symptom**: No data received from Bluetooth

**Solutions**:
1. Verify TX/RX pins are correctly connected (TX → RX, RX → TX)
2. Check baud rate matches module configuration (default 9600)
3. Ensure module is powered (LED blinking = discoverable/connected)
4. Test with AT commands: `Bluetooth_AtCommand("AT");`

### Garbled Characters

**Symptom**: Receiving corrupted data

**Solutions**:
1. Check baud rate mismatch (both sides must match)
2. Verify voltage levels (use level shifter for 5V modules)
3. Add delay after `Bluetooth_Setup()` (module initialization)
4. Check for electrical noise (shorten wires, add decoupling caps)

### Buffer Overflow

**Symptom**: "Max command length reached" messages

**Solutions**:
1. Reduce command length
2. Increase `MAX_COMMAND_SIZE` in `bluetooth.c`
3. Split long commands into multiple shorter ones

---

## 📁 Project Structure

```
bluetooth_sdk/
├── CMakeLists.txt          # Build configuration
├── README.md               # This file
│
├── include/
│   └── bt_kmn/
│       ├── bluetooth.h     # Core Bluetooth API
│       └── commands.h      # Command system macros
│
└── src/
    ├── bluetooth.c         # UART and message handling
    └── commands.c          # Command parsing and dispatch
```

---

## 🔗 Integration with Pico SDK

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.13)

# Include Pico SDK
include(pico_sdk_import.cmake)

project(my_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Add bt_kmn library
add_subdirectory(bluetooth_sdk)

# Your executable
add_executable(my_project
    main.cpp
)

target_link_libraries(my_project
    pico_stdlib
    hardware_uart
    bt_kmn              # Link the library
)

pico_add_extra_outputs(my_project)
```

---

## 📝 Advanced Usage

### Custom Command Dispatcher

If you need more control over command processing:

```cpp
void Bluetooth_ReadMessage() {
    // ... (read from UART into buffer)
    
    // Custom pre-processing
    if (buffer[0] == '#') {
        handle_special_command(buffer);
        return;
    }
    
    // Standard command dispatch
    __run_command(buffer);
    
    // Custom post-processing
    log_command_execution(buffer);
}
```

### External Commands (from C++)

```cpp
extern "C" {
    // Declare command functions
    void robot_start_command(const char *params);
    void robot_stop_command(const char *params);
}

// C++ implementation with access to class members
void robot_start_command(const char *params) {
    myRobot.start();  // Call C++ class method
    Bluetooth_SendMessage("Started\n");
}

// Register (must be in extern "C" scope)
BT_COMMAND(START, robot_start_command)
```

---

## 🤝 Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Follow existing code style (`.clang-format` provided)
4. Test thoroughly
5. Submit a pull request

---

## 📚 See Also

- [Raspberry Pi Pico SDK Documentation](https://www.raspberrypi.com/documentation/pico-sdk/)
- [UART Hardware Documentation](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf)

---

**Last Updated**: November 2025
