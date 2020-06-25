/*!
 * @file BlueFly.cpp
 *
 *
 */

#include "BlueFly.h"

BlueFly::BlueFly(){
    pBlueFlySerial = NULL;
}

int BlueFly::getStringValue(String s,String *sRet,unsigned int fromIndex,String keyword,String delimiter){
  *sRet = "";
  int pos = s.indexOf(keyword,fromIndex);
  if (pos < 0) return -1; //not found
  pos += keyword.length();
  int pos2 = 0;
  if (delimiter.length() > 0){
    pos2 = s.indexOf(delimiter,pos);
  }else{
      pos2 = s.length();
  }
  if (pos2 < 0) return -1; //not found
  *sRet = s.substring(pos,pos2);
  return pos2;
}

bool BlueFly::begin(uint8_t SerialNumber,uint8_t RxPin, uint8_t TxPin,NmeaOut *_pNmeaOut){
    pBlueFlySerial = new HardwareSerial(SerialNumber);
    pBlueFlySerial->begin(115200, SERIAL_8N1, RxPin, TxPin);
    recBufferIndex = 0;
    nmea.setBuffer(nmeaBuffer, sizeof(nmeaBuffer));
    pNmeaOut = _pNmeaOut;
    return true;
}

float BlueFly::getAlt(){
    return alt;
}
float BlueFly::getClimb(){
    return climb;
}

void BlueFly::writeToSerial(String s){
    pBlueFlySerial->print(s);
}

void BlueFly::checkLXWP0(String s){
    String s1;
    int ret = 0;
    ret = getStringValue(s,&s1,ret,"$LXWP0,",",");
    if (ret < 0){
        return;
    }
    //Serial.println(s1);
    ret = getStringValue(s,&s1,ret,",",",");
    if (ret < 0){
        return;
    }
    //Serial.println(s1);
    ret = getStringValue(s,&s1,ret,",",",");
    if (ret < 0){
        return;
    }
    alt = s1.toFloat();
    //Serial.println(s1);
    ret = getStringValue(s,&s1,ret,",",",");
    if (ret < 0){
        return;
    }
    climb = s1.toFloat();
    //Serial.println(s1);
}

void BlueFly::checkLine(String s){
    if (s.startsWith("$LXWP0")){
        checkLXWP0(s);
    }
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
            checkLine(String(lineBuffer) + "\r\n");
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

