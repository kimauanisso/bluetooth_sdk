#include "bt_kmn/bluetooth.h"

#include "stdbool.h"
#include "hardware/gpio.h"
#include "pico/time.h"

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

    sleep_ms(1000);
}

void Bluetooth_SendMessage(const char* message){
    uart_puts(_config.uart, message);
}