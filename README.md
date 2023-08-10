# NMEA Firehose Device

Part of the [CyberBoat](https://cyberboat.gitbook.io/cyberboat/) project. See the [CyberBoat T Connector Instructions](https://cyberboat.gitbook.io/cyberboat/cyberboat/nmea-t-connector/t-connector-instructions) for setup and usage. 

Clone the repo  with its submodules:

`
$ git clone --recurse-submodules git@github.com:gpearcey/nmea_firehose.git
`

To use the status printing:
 * To use this function, you need to configure some settings in menuconfig. 
 * You must enable FreeRTOS to collect runtime stats under ``` Component Config -> FreeRTOS -> Kernel -> configGENERATE_RUN_TIME_STATS```
   
 * You must also choose the clock source for run time stats configured under ```Component Config -> FreeRTOS -> Port -> Choose the clock source for runtime stats```
 * The esp_timer should be selected by default. 
 * This option will affect the time unit resolution in which the statistics are measured with respect to.

