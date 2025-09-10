#include "hardware/gpio.h"
#include "pico/time.h"

#include "bt_kmn/bluetooth.h"

BluetoothConfig btConfig = {
    .baudrate = 9600, .rx_pin = 13, .tx_pin = 12, .uart = uart0
};

void Configure_BluetoothModule() {
  Bluetooth_AtCommand("AT+NAME=bt_kmn_test");
  Bluetooth_AtCommand("AT+PSWD=123");
  Bluetooth_AtCommand("AT+RESET");
}

int main() {
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

  Bluetooth_Setup(btConfig);

  while (true) {

    Bluetooth_SendMessage("Hello World!\n");
  }
  return 0;
}
