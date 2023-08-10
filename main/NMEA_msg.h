/**
 * @file NMEA_msg.h
 * 
 * @brief Represents a NMEA 2000 messages
 * 
 * Message structure is as follows:
 * 
 * * controller number
 * * PGN
 * * source 
 * * priority
 * * data_length_bytes 
 * * data array
 * 
*/
#ifndef NMEA_MSG_H
#define NMEA_MSG_H

#include <vector>


struct NMEA_msg {
    uint8_t controller_number;
    static const int MaxDataLen=223;
    uint32_t PGN : 18;
    uint8_t source;
    uint8_t priority : 3;
    int data_length_bytes;
    uint8_t data[MaxDataLen];
};

#endif //NMEA_MSG_Hcode 