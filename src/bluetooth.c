#include "bt_kmn/bluetooth.h"
#include "bt_kmn/commands.h"

#include "stdbool.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/time.h"
#include "stdbool.h"

#define MAX_COMMAND_SIZE 128
uint8_t bt_buffer[MAX_COMMAND_SIZE];

static BluetoothConfig _config;

void Bluetooth_Setup(const BluetoothConfig config){
    _config = config;
    gpio_init(_config.rx_pin);
    gpio_init(_config.tx_pin);

    gpio_set_function(_config.rx_pin, GPIO_FUNC_UART);
    gpio_set_function(_config.tx_pin, GPIO_FUNC_UART);

    uart_init(_config.uart, _config.baudrate);
}

void Bluetooth_InterruptInit(irq_handler_t handler){
    const int irq = _config.uart == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(irq, handler);
    irq_set_enabled(irq, true);
}

void Bluetooth_AtCommand(const char* command){
    uart_puts(_config.uart, command);
    uart_puts(_config.uart, "\r\n");

void Bluetooth_SendMessage(const char *message) {
  uart_puts(_config.uart, message);
}

void Bluetooth_ReadMessage() {
  int count = 0;
  while (uart_is_readable(_config.uart)) {
    uint8_t ch = uart_getc(_config.uart);
    bt_buffer[count++] = ch;
    if (count > MAX_COMMAND_SIZE) {
      Bluetooth_SendMessage("Max command length reached\n");
      return;
    }
  }
  if (count == 0) {
    return;
  }
  bt_buffer[count] = '\0';
  Bluetooth_SendMessage("Running command...\n");
  __run_command(bt_buffer);
}