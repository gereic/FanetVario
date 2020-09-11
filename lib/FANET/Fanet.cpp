/*!
 * @file Fanet.cpp
 *
 *
 */

#include "Fanet.h"

Fanet::Fanet(){
    pFanetSerial = NULL;
    initCount = 0;
}


void Fanet::setNMEAOUT(NmeaOut *_pNmeaOut){
    pNmeaOut = _pNmeaOut;
}

bool Fanet::begin(uint8_t SerialNumber,uint8_t RxPin, uint8_t TxPin,uint8_t ResetPin){
    _ResetPin = ResetPin;
    pFanetSerial = new HardwareSerial(SerialNumber);
    pFanetSerial->begin(115200, SERIAL_8N1, RxPin, TxPin);
    pinMode(ResetPin,OUTPUT);
    digitalWrite(_ResetPin, HIGH);
    delay(500); //wait Hardware ready
    resetModule();
    initCount = 0;
    //Serial.println("check receiver version");
    pFanetSerial->print("#DGV\n");
    tCheckModuleState = millis();
    tWeather = millis();  
    _PilotName = "";  
    _myData.aircraftType = eFanetAircraftType::PARA_GLIDER; //default Paraglider
    recBufferIndex = 0;
    bInitOk = false;
    pNmeaOut = NULL;
    return true;
}


void Fanet::setPilotname(String name){
    _PilotName = name;
}

void Fanet::setAircraftType(eFanetAircraftType type){
    _myData.aircraftType = type;
}

void Fanet::sendPilotName(uint32_t tAct){
    static uint32_t tPilotName = millis();
    if (((tAct - tPilotName) >= 240000) && (bInitOk)){
        tPilotName = tAct;
        if (_PilotName.length() > 0){
            String sSend = "#FNT 2,00,0000,1,0,";
            uint8_t arSend[32];
            uint8_t i;
            uint8_t myLen = _PilotName.length();
            _PilotName.toCharArray((char *)&arSend[0],sizeof(arSend));
            sSend += getHexFromByte(myLen) + ",";
            for (i = 0 ;i < myLen; i++){
                sSend += getHexFromByte(arSend[i]);
            }   
            sSend += "\n"; 
            //Serial.print("len=");Serial.println(myLen);
            //Serial.println("sending fanet-name");
            //Serial.print(sSend);
            pFanetSerial->print(sSend);
        }
    }
}

void Fanet::resetModule(){
    digitalWrite(_ResetPin, LOW);
    delay(500);
    digitalWrite(_ResetPin, HIGH);
    delay(500);
}

void Fanet::initModule(uint32_t tAct){
    static uint32_t timeout = millis();
    bool btimeout = false;
    if ((tAct - timeout) >= 1000){
        btimeout = true;
    }
    switch (initCount)
    {
    case 0:
        bDGVOk = false;
        pFanetSerial->print("#DGV\n"); //get module-version
        timeout = tAct; //reset timeout
        initCount++;
        break;
    case 1:
        if (btimeout){
            initCount--; //back on step ask module again
        }
        if (bDGVOk){
            bFNAOk = false;
            //Serial.print("#FNA\n");
            pFanetSerial->print("#FNA\n"); //get module-addr
            timeout = tAct; //reset timeout
            initCount++;
        }
        break;
    case 2:
        if (btimeout){
            initCount--; //back on step ask module again
        }
        if (bFNAOk){
            bFAXOk = false;
            pFanetSerial->print("#FAX\n"); //flarm expiration
            timeout = tAct; //reset timeout
            initCount++;
        }
        break;
    case 3:
        if (btimeout){
            initCount--; //back on step ask module again
        }
        if (bFAXOk){
            bFNCOk = false;
            //Serial.print("#FNC 1,1\n");
            pFanetSerial->print("#FNC 1,1\n"); //PG, online tracking
            timeout = tAct; //reset timeout
            initCount++;
        }
        break;
    case 4:
        if (btimeout){
            initCount--; //back on step ask module again
        }
        if (bFNCOk){
            bDGPOk = false;
            //Serial.print("#DGP 1\n");
            pFanetSerial->print("#DGP 1\n"); //Enable receiver
            timeout = tAct; //reset timeout
            initCount++;
        }
        break;
    case 5:
        if (btimeout){
            initCount--; //back on step ask module again
        }
        if (bDGPOk){
            bFAPOk = false;
            //Serial.print("#FAP 1\n");
            pFanetSerial->print("#FAP 1\n"); //Enable FLARM
            timeout = tAct; //reset timeout
            initCount++;
        }
        break;
    case 6:
        if (btimeout){
            initCount--; //back on step ask module again
        }
        if (bFAPOk){
            initCount = 100; //we are ready !!
            //Serial.println("**** INIT OK ****");
            bInitOk = true;
        }
        break;
    
    default:
        break;
    }
}

String Fanet::getFlarmExp(void){
    return FlarmExp;
}

String Fanet::getAircraftType(eFanetAircraftType type){
  if (type == eFanetAircraftType::PARA_GLIDER){
    return "PARA_GLIDER";
  }else if (type == eFanetAircraftType::HANG_GLIDER){
    return "HANG_GLIDER";
  }else if (type == eFanetAircraftType::BALLOON){
    return "BALLOON";
  }else if (type == eFanetAircraftType::GLIDER){
    return "GLIDER";
  }else if (type == eFanetAircraftType::POWERED_AIRCRAFT){
    return "POWERED_AIRCRAFT";
  }else if (type == eFanetAircraftType::HELICOPTER_ROTORCRAFT){
    return "HELICOPTER_ROTORCRAFT";
  }else if (type == eFanetAircraftType::UAV){
    return "UAV";
  }
  return "UNKNOWN";

}

void Fanet::getMyID(String line){
    _myData.DevId = line.substring(5,7) + line.substring(8,12);
    //Serial.print("*******myID=");
    //Serial.println(_myData.DevId);

}

void Fanet::getVersion(String line){
    sVersion = line;
    log_i("Version=%s",sVersion.c_str());
}

void Fanet::getFAX(String line){
    String s1;
    int ret = 0;
    FlarmExp = "";
    ret = getStringValue(line,&s1,ret,"#FAX ",",");
    if (ret >= 0){
        FlarmExp = String(atoi(s1.c_str()) + 1900);
    }
    ret = getStringValue(line,&s1,ret,",",",");
    if (ret >= 0){
        FlarmExp += "-" + String(atoi(s1.c_str()));
    }
    ret = getStringValue(line,&s1,ret,",","");
    if (ret >= 0){
        FlarmExp += "-" + String(atoi(s1.c_str()));
    }
    Serial.println(FlarmExp);
}

bool Fanet::initOk(void){
    return bInitOk;
}

void Fanet::DecodeLine(String line){
    //Serial.println(line);
    if (line.startsWith("#DGV")){
        getVersion(line.substring(5));
        bDGVOk = true;
    }else if (line.startsWith("#FNA")){
        getMyID(line);
        bFNAOk = true;
    }else if (line.startsWith("#FAX")){
        getFAX(line);
        bFAXOk = true;
    }else if (line.startsWith("#FNR OK")){
        bFNCOk = true;
    }else if (line.startsWith("#DGR OK")){
        bDGPOk = true;
    }else if (line.startsWith("#FAR OK")){
        bFAPOk = true;
    }else if (line.startsWith("#FNF")){
        //we have received tracking-information
        CheckReceivedPackage(line);
    }else{
        Serial.println("UNKNOWN MSG:");
        Serial.println(line);
    }

}

String Fanet::getMyDevId(void){
    return _myData.DevId;
}

int Fanet::updateModule(String filename){
    int ret =  xmodem_transmit(pFanetSerial, filename.c_str(),_ResetPin);
    SPIFFS.remove(filename.c_str()); // remove file
    log_i("ret=%d",ret);
    return ret;
}

void Fanet::run(void){    
    uint32_t tAct = millis();
    initModule(tAct);
    while (pFanetSerial->available()){
        if (recBufferIndex >= FANET_MAXRECBUFFER) recBufferIndex = 0; //Buffer overrun
        lineBuffer[recBufferIndex] = pFanetSerial->read();
        if (lineBuffer[recBufferIndex] == '\n'){
            //Serial.print("length=");Serial.println(recBufferIndex);
            lineBuffer[recBufferIndex] = 0; //zero-termination
            //Serial.println(lineBuffer);
            //log_i("%s",lineBuffer);
            DecodeLine(String(lineBuffer));    
            recBufferIndex = 0;
        }else{
            if (lineBuffer[recBufferIndex] != '\r'){
                recBufferIndex++;
            }
        }    
        
        
        //String line = pFanetSerial->readStringUntil('\n');
        //Serial.println(line);
        //DecodeLine(line);
    }
    /*
    if ((tAct - tCheckModuleState) >= 10000){
        pFanetSerial->print("#DGP\n"); //Check Receiver state
        tCheckModuleState = tAct;
    }
    */
    sendPilotName(tAct);
}

void Fanet::sendWeather(void){
    String sSend = "#FNT 4,00,0000,1,0,A,A0965E44687A0A7F4B96\n";
    Serial.println("sending weather-data");
    Serial.println(sSend);
    pFanetSerial->print(sSend);
}

void Fanet::CheckReceivedPackage(String line){
    String s1;
    String devId = "";
    int ret;
    ret = getStringValue(line,&s1,0,"#FNF ",",");
    //Serial.print("ret=");
    //Serial.println(ret);
    //Serial.print("src_manufacturer=");
    //Serial.println(s1);
    
    if (s1.length() < 2) s1 = "0" + s1;
    devId = s1;
    /*
    if (s1 == "7"){
        devId = "F1";
    }else{
        devId = s1;
    }
    */
    ret = getStringValue(line,&s1,ret,",",",");
    if (s1.length() < 4) s1 = "0" + s1;
    devId += s1;
    //Serial.print("src_id=");
    //Serial.println(s1);

    ret = getStringValue(line,&s1,ret,",",",");
    //Serial.print("broadcast=");
    //Serial.println(s1);

    ret = getStringValue(line,&s1,ret,",",",");
    //Serial.print("signature=");
    //Serial.println(s1);

    ret = getStringValue(line,&s1,ret,",",",");
    //Serial.print("type=");
    //Serial.println(s1);
    uint8_t msgType = s1.toInt();
    ret = getStringValue(line,&s1,ret,",",",");
    //Serial.print("length=");
    //Serial.println(s1);
    long x = strtol(s1.c_str(),NULL,16) * 2;
    //Serial.println(x);
    String payload = line.substring(ret+1);
    //Serial.println(payload.length());
    if (msgType == 1){
        if (payload.length() == x){
            actTrackingData.DevId = devId;
            getTrackingInfo(payload,uint16_t(x));                
        }else{
            Serial.println("length not ok");
        }        
        //Serial.println(line); //directly to serial out
        /*
        if (pNmeaOut != NULL){
            pNmeaOut->write(line + "\r\n"); //directly to serial out
        }
        */
    }else if ((msgType == 2) || (msgType == 3) || (msgType == 4)){
        //Type 2 --> Device-Name
        //Type 3 --> MSG
        //Type 4 --> weather-data
        if (pNmeaOut != NULL){
            pNmeaOut->write(line + "\r\n"); //directly to serial out
        }
    }
}

String Fanet::getHexFromByte(uint8_t val){
    char myHexString[3];

    myHexString[0] = (val >> 4) + 0x30;
    if (myHexString[0] > 0x39) myHexString[0] +=7;


    myHexString[1] = (val & 0x0f) + 0x30;
    if (myHexString[1] > 0x39) myHexString[1] +=7;   
    myHexString[2] = 0;
    return String(myHexString);; 
}

int Fanet::getByteFromHex(char in[]){
  int tens;
  int digits;
   
  if (!isxdigit(in[0]) || !isxdigit(in[1]))   // Valid hex digit character?
    return -1;

  in[0] = toupper(in[0]);   // Use upper case
  in[1] = toupper(in[1]);
 
  tens = in[0] >= 'A' ? (in[0] - 'A' + 10) : in[0] - '0';
  digits = in[1] >= 'A' ? (in[1] - 'A' + 10) : in[1] - '0';
  return tens * 16 + digits;
}

void Fanet::getTrackingInfo(String line,uint16_t length){
    char arPayload[23];
    
    int32_t lat_i = 0;
    int32_t lon_i = 0;

    /*
    if (length > 22){
        Serial.println("length > 22 --> abort: ");
        Serial.println(length);
        return;
    }
    */
    line.toCharArray(arPayload,sizeof(arPayload));

    ((uint8_t*)&lat_i)[0] = getByteFromHex(&arPayload[0]);
    ((uint8_t*)&lat_i)[1] = getByteFromHex(&arPayload[2]);
    ((uint8_t*)&lat_i)[2] = getByteFromHex(&arPayload[4]);

    ((uint8_t*)&lon_i)[0] = getByteFromHex(&arPayload[6]);
    ((uint8_t*)&lon_i)[1] = getByteFromHex(&arPayload[8]);
    ((uint8_t*)&lon_i)[2] = getByteFromHex(&arPayload[10]);

    actTrackingData.lat = (float) lat_i / 93206.0f;
    actTrackingData.lon = (float) lon_i / 46603.0f;

    uint16_t Type = (uint16_t(getByteFromHex(&arPayload[14])) << 8) + uint16_t(getByteFromHex(&arPayload[12]));
    uint16_t altitude = Type & 0x7FF;
    if  (Type & 0x0800){
        altitude *= 4;
    } 
    actTrackingData.altitude = altitude;
    actTrackingData.aircraftType = (eFanetAircraftType)((Type >> 12) & 0x07);

    uint16_t speed = uint16_t(getByteFromHex(&arPayload[16]));
    if (speed & 0x80){
        speed = (speed & 0x80) * 5;
    }else{
        speed = (speed & 0x80);
    }
    actTrackingData.speed = float(speed) * 0.5;

    int8_t climb = int8_t(getByteFromHex(&arPayload[18]));
    if (climb & 0x80){
        climb = (climb & 0x80) * 5;
    }else{
        climb = (climb & 0x80);
    }
    actTrackingData.climb = float(climb) * 0.1;

    actTrackingData.heading = float(getByteFromHex(&arPayload[20])) * 360 / 255;
    if (actTrackingData.heading  < 0) actTrackingData.heading += 360.0;
    newData = true;
}
bool Fanet::newTrackingDataAvaiable(void){
    return newData;
}

bool Fanet::getTrackingData(trackingData *tData){
    bool bRet = newData;
    *tData = actTrackingData; //copy tracking-data
    newData = false; //clear new Data flag
    return bRet;
}

String Fanet::getStringValue(String s,String keyword){
  String sRet = "";
  int pos = s.indexOf(keyword);
  if (pos < 0) return sRet; //not found
  sRet = s.substring(pos + keyword.length());
  return sRet;
}

int Fanet::getStringValue(String s,String *sRet,unsigned int fromIndex,String keyword,String delimiter){
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
void Fanet::writeStateData2FANET(stateData *tData){
    String sFNS = "#FNS " + String(tData->lat,6) + ","
                 + String(tData->lon,6) + ","
                 + String(tData->altitude,3) + "," 
                 + String(int(tData->speed)) + "," 
                 + String(tData->climb,2) + "," 
                 + String(tData->heading,2) + "," 
                 + String(tData->year) + "," 
                 + String(tData->month-1) + "," 
                 + String(tData->day)+ "," 
                 + String(tData->hour) +  "," 
                 + String(tData->minute) +  "," 
                 + String(tData->second) + "," 
                 + String(tData->geoIdAltitude,3) + ",0\n";
//                 + String(tData->geoIdAltitude,3) + ","
//                 + String(tData->turnrate,1) + "\n"; //todo check geoid-height
    //Serial.println(sFNS);
    //log_i("%s",sFNS.c_str());
    _myData.altitude = tData->altitude;
    _myData.climb = tData->climb;
    _myData.heading = tData->heading;
    _myData.lat = tData->lat;
    _myData.lon = tData->lon;
    _myData.speed = tData->speed;
    if (bInitOk){
        pFanetSerial->print(sFNS);
    }
}

void Fanet::writeTrackingData2FANET(trackingData *tData){
    uint8_t arSend[15];
    int i;
    String sSend = "#FNT 1,";    
    sSend += tData->DevId.substring(0,2);
    sSend += ",";
    sSend += tData->DevId.substring(2,6);
    sSend += ",1,0,B,";
    encodeTrackingData(*tData,arSend);
    for (i = 0 ;i < 11; i++){
        sSend += getHexFromByte(arSend[i]);
    }    
    Serial.println("sending tracking-data to FANET");
    Serial.println(sSend);
    pFanetSerial->print(sSend);
}

void Fanet::coord2payload_absolut(float lat, float lon, uint8_t *buf)
{
	if(buf == NULL)
		return;

	int32_t lat_i = roundf(lat * 93206.0f);
	int32_t lon_i = roundf(lon * 46603.0f);

	buf[0] = ((uint8_t*)&lat_i)[0];
	buf[1] = ((uint8_t*)&lat_i)[1];
	buf[2] = ((uint8_t*)&lat_i)[2];

	buf[3] = ((uint8_t*)&lon_i)[0];
	buf[4] = ((uint8_t*)&lon_i)[1];
	buf[5] = ((uint8_t*)&lon_i)[2];
}

void Fanet::encodeTrackingData(trackingData tData,uint8_t *buf){
    //first 6 Bytes are the position
    coord2payload_absolut(tData.lat,tData.lon,buf);
    uint16_t type = 0x8000; //always online tracking on
    type += ((uint16_t)tData.aircraftType) << 12;
    if (tData.altitude > 0x7FF){
        type += 0x800; //altidude * 4
        type += (tData.altitude / 4);
    }else{
        type += tData.altitude;
    }  
    buf[6] = uint8_t(type);
    buf[7] = uint8_t(type >> 8);    
    //speed
    if (tData.speed > 0x7F){
        buf[8] = 0x80 + (uint8_t)((uint16_t)(round(tData.speed)) / 5);
    }else{
        buf[8] = (uint8_t)(round(tData.speed));
    }
    //climb
    buf[9] = 0;
    //heading
     buf[10] = (uint8_t)(round(tData.heading * 255 / 360));
    
}

void Fanet::printAircraftType(eFanetAircraftType type){
  Serial.print("Aircraft-type=");
  if (type == eFanetAircraftType::PARA_GLIDER){
    Serial.println("PARA_GLIDER");
  }else if (type == eFanetAircraftType::HANG_GLIDER){
    Serial.println("HANG_GLIDER");
  }else if (type == eFanetAircraftType::BALLOON){
    Serial.println("BALLOON");
  }else if (type == eFanetAircraftType::GLIDER){
    Serial.println("GLIDER");
  }else if (type == eFanetAircraftType::POWERED_AIRCRAFT){
    Serial.println("POWERED_AIRCRAFT");
  }else if (type == eFanetAircraftType::HELICOPTER_ROTORCRAFT){
    Serial.println("HELICOPTER_ROTORCRAFT");
  }else if (type == eFanetAircraftType::UAV){
    Serial.println("UAV");
  }else{
      Serial.println("UNKNOWN");
  }
}

bool Fanet::getMyTrackingData(trackingData *tData){
    *tData = _myData;
    return true;
}

void Fanet::printFanetData(trackingData tData){
    Serial.print("id=");
    Serial.println(tData.DevId);
    printAircraftType(tData.aircraftType);
    Serial.print("lat=");
    Serial.println(tData.lat,5);
    Serial.print("lon=");
    Serial.println(tData.lon,5);
    Serial.print("alt=");
    Serial.println(tData.altitude);
    Serial.print("heading=");
    Serial.println(tData.heading);
    Serial.print("speed=");
    Serial.println(tData.speed,2);
    Serial.print("climb=");
    Serial.println(tData.climb,2);

}