/*!
 * @file BlueFly.h
 *
 *
 */

#ifndef __BLUEFLY_H__
#define __BLUEFLY_H__


#include <HardwareSerial.h>
#include "MicroNMEA.h"
#include <nmeaout.h>

#define BLUEFLY_MAXRECBUFFER 255

class BlueFly {
public:
    BlueFly(); //constructor
    bool begin(uint8_t SerialNumber,uint32_t baudrate,uint8_t RxPin, uint8_t TxPin,NmeaOut *_pNmeaOut);
    void writeToSerial(String s);
    void run(void); //has to be called cyclic
    float getAlt();
    float getClimb();
    MicroNMEA nmea;
    uint16_t nmeaCount = 0;
protected:
    float alt;
    float climb;
    void checkLine(String s);
    void checkLXWP0(String s);
    int getStringValue(String s,String *sRet,unsigned int fromIndex,String keyword,String delimiter);
    NmeaOut *pNmeaOut;  
    HardwareSerial *pBlueFlySerial;    
    char lineBuffer[BLUEFLY_MAXRECBUFFER];
    char nmeaBuffer[BLUEFLY_MAXRECBUFFER];
    uint8_t recBufferIndex;
};
#endif