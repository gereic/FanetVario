/*!
 * @file BlueFly.h
 *
 *
 */

#ifndef __BLUEFLY_H__
#define __BLUEFLY_H__


#include <HardwareSerial.h>
#include "MicroNMEA.h"
#define BLUEFLY_MAXRECBUFFER 255

class BlueFly {
public:
    BlueFly(); //constructor
    bool begin(uint8_t SerialNumber,uint8_t RxPin, uint8_t TxPin);
    void writeToSerial(String s);
    void run(void); //has to be called cyclic
    MicroNMEA nmea;
    uint16_t nmeaCount = 0;
protected:
    HardwareSerial *pBlueFlySerial;    
    char lineBuffer[BLUEFLY_MAXRECBUFFER];
    char nmeaBuffer[BLUEFLY_MAXRECBUFFER];
    uint8_t recBufferIndex;
};
#endif