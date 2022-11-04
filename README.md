# UGV_Firmware
Firmware for boat-emulating ground autonomous vehicles

Broken into two parts

## UGV-ESP
- Websocket handling between MCU and server
- Passing relevant data from websocket, to RPI Pico over UART

## UGV-PICO
- Main controller for UGV
- State estimation
- Control loop
- Path following
