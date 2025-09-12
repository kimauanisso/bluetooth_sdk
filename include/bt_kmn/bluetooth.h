#include "hardware/uart.h"
#include "hardware/irq.h"

typedef struct BluetoothConfig {
    uart_inst_t* uart;
    uint baudrate;
    uint rx_pin;
    uint tx_pin;
} BluetoothConfig;

void Bluetooth_Setup(BluetoothConfig config);
void Bluetooth_InterruptInit(irq_handler_t handler);

void Bluetooth_AtCommand(const char* command);

void Bluetooth_SendMessage(const char *message);
void Bluetooth_ReadMessage();