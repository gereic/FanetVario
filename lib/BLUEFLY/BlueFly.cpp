/*!
 * @file BlueFly.cpp
 *
 *
 */

#include "BlueFly.h"

BlueFly::BlueFly(){
    pBlueFlySerial = NULL;
}


bool BlueFly::begin(uint8_t SerialNumber,uint8_t RxPin, uint8_t TxPin,NmeaOut *_pNmeaOut){
    pBlueFlySerial = new HardwareSerial(SerialNumber);
    pBlueFlySerial->begin(115200, SERIAL_8N1, RxPin, TxPin);
    recBufferIndex = 0;
    nmea.setBuffer(nmeaBuffer, sizeof(nmeaBuffer));
    pNmeaOut = _pNmeaOut;
    return true;
}

void BlueFly::writeToSerial(String s){
    pBlueFlySerial->print(s);
}

void BlueFly::run(void){    
    //uint32_t tAct = millis();
    while (pBlueFlySerial->available()){
        if (recBufferIndex >= BLUEFLY_MAXRECBUFFER) recBufferIndex = 0; //Buffer overrun
        lineBuffer[recBufferIndex] = pBlueFlySerial->read();
        if (nmea.process(lineBuffer[recBufferIndex])){ //process char
            nmeaCount ++;
        };
        if (lineBuffer[recBufferIndex] == '\n'){
            //Serial.print("length=");Serial.println(recBufferIndex);
            lineBuffer[recBufferIndex] = 0; //zero-termination
            pNmeaOut->write(String(lineBuffer) + "\r\n");
            recBufferIndex = 0;
        }else{
            if (lineBuffer[recBufferIndex] != '\r'){
                recBufferIndex++;
            }
        }        
        //String line = pBlueFlySerial->readStringUntil('\n');
        //Serial.println(line);
    }
}

