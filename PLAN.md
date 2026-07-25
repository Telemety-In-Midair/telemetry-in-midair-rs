# Plan

ESP32-C6 connected over UART to the WIO-E5.

WIO-E5 either programmed over SWD or UART from ESP.

WIO-E5 and ESP32C6 talk over UART. The ESP and WIO should be able to cordinate power usage to allow avoiding using all radios at once. 

The WIO-E5 reads from the MAX-M10N-10B over UART. The GPS coordinates will be sent over LORA (915 MHz). The coordinates will also be sent to the ESP32c6 over UART and logged to the SD.

The radio is configured by TOML files that can be stored on the sd card and/or sent over UART by the ESP32C6.

The ESP32C6 communicates over BLE to an external app for configuration. The WIO-E5 and MAX-M10N-10B can be turned off or slept by BLE. The ESP can be put into a sleep over BLE. Waking up on a interval to check for wake over BLE.

The SD Card logs should be readable by a phone or computer.

The WIO-E5 blinks D6 on LORA RX, D5 on LORA TX.

The ESP32C6 blinks D2 very quickly on firmware upload to the WIO. Blinks on sleep wake interval. Blinks quickly on info received from phone.

The SD card is optional to run. The ESP will cache the latest data (LORA RSSI/GPS coords/time). This data will be sent once ble is connected.

All LoRa traffic is transmission. A node is a leaf by default and hears every
other node in direct range; one configured as a repeater retransmits what
it hears, extending coverage past a single radio horizon.

