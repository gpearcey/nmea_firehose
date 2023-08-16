# NMEA Firehose Device

Part of the [CyberBoat](https://cyberboat.gitbook.io/cyberboat/) project. See the [CyberBoat T Connector Instructions](https://cyberboat.gitbook.io/cyberboat/cyberboat/nmea-t-connector/t-connector-instructions) for setup and usage. 

The NMEA Firehose Device can send any PGN message to a NMEA2000 network. Used for debugging purposes in the CyberBoat project. 

## Requirements
### ESP-IDF
Follow this guide to setup ESP-IDF to build and flash your esp32 board. This poject was completed with esp version 5.0.2. Other versions are likely compatible, but not tested.


## Hardware

* ESP32-C6
* [CAN Transceiver](https://www.waveshare.com/sn65hvd230-can-board.htm)
* NMEA2000 Cabling

## Wiring

| ESP32       | CAN TRX     |
| ----------- | ----------- |
| GPIO 22     | CAN TX      |
| GPIO 23     | CAN RX      |
| 3V3         | 3.3V        |
| GND         | GND         | 

You can change the wiring between ESP32 and CAN TRX in main/main.cpp. 

Connect CAN H and CAN L on CAN TRX to the NMEA2000 network. 

`
tNMEA2000_esp32c6 NMEA2000(GPIO_NUM_22, GPIO_NUM_23);
`

## Set Message

You can set all parts of the message main/main.cpp in the ```N2K_send_task()``` function:

    msg.controller_number = 0;
    msg.PGN = 129026;
    msg.source = 15;
    msg.priority = 2;
    msg.data_length_bytes = 8;
    msg.data[0] = 0x00;
    msg.data[1] = 0x02;
    msg.data[2] = 0x04;
    msg.data[3] = 0x06;
    msg.data[4] = 0x08;
    msg.data[5] = 0x0a;
    msg.data[6] = 0x0c;
    msg.data[7] = 0x0e;


## Install, Build, and Run

Clone the repo  with its submodules:

`
$ git clone --recurse-submodules git@github.com:gpearcey/nmea_firehose.git
`

### Enable Status Printing 

Status printing periodically prints information about task counts and execution time. This is helpful for debugging. 

To use the status printing:
 * To use this function, you need to configure some settings in menuconfig. 
 * You must enable FreeRTOS to collect runtime stats under ``` Component Config -> FreeRTOS -> Kernel -> configGENERATE_RUN_TIME_STATS```
   
 * You must also choose the clock source for run time stats configured under ```Component Config -> FreeRTOS -> Port -> Choose the clock source for runtime stats```
 * The esp_timer should be selected by default. 
 * This option will affect the time unit resolution in which the statistics are measured with respect to.

If you don't want to use status printing, comment out the status task in the `app_main()` function in main/main.cpp. 

### Build and Run

```
$ cd nmea_firehose
$ idf.py build
$ idf.py flash
```
