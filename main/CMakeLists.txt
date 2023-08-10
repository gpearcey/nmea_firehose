idf_component_register(SRCS "main.cpp" 
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver freertos NMEA2000 NMEA2000_esp32-c6 wamr)

