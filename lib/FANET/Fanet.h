/*!
 * @file fanet.h
 *
 *
 */

#ifndef __FANET_H__
#define __FANET_H__


#include <HardwareSerial.h>
#include <string.h>
#include <nmeaout.h>
#include <xmodem.h>

#define FANET_MAXRECBUFFER 255
#define FANET_DEBUG
#define NAME_SEND_TIME 240000

enum class eFanetAircraftType {
  UNKNOWN = 0,
  PARA_GLIDER = 1,
  HANG_GLIDER = 2,
  BALLOON = 3,
  GLIDER = 4,
  POWERED_AIRCRAFT = 5,
  HELICOPTER_ROTORCRAFT = 6,
  UAV = 7
};

typedef struct {
  String DevId;
  float lat; //latitude
  float lon; //longitude
  float altitude; //altitude [m]
  eFanetAircraftType aircraftType; //
  float speed; //km/h
  float climb; //m/s
  float heading; //deg
} trackingData;

typedef struct {
  float lat; //latitude
  float lon; //longitude
  float altitude; //altitude [m]
  float geoIdAltitude; //altitude [m]
  float speed; //km/h
  float climb; //m/s
  float heading; //deg
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
  float turnrate;
} stateData;

class Fanet {
public:
    Fanet(); //constructor
    bool begin(uint8_t SerialNumber,uint8_t RxPin, uint8_t TxPin,uint8_t ResetPin,int8_t BootPin);
    String getMyDevId(void);
    String getFlarmExp(void);    
    void setNMEAOUT(NmeaOut *_pNmeaOut);
    void setPilotname(String name);
    void setAircraftType(eFanetAircraftType type);
    void run(void); //has to be called cyclic
    bool newTrackingDataAvaiable(void);
    bool getTrackingData(trackingData *tData);
    bool getMyTrackingData(trackingData *tData);
    void writeTrackingData2FANET(trackingData *tData);
    void printFanetData(trackingData tData);
    void writeStateData2FANET(stateData *tData);
    int updateModule(String filename);
    void setFlyingState(bool flying);
    String getAircraftType(eFanetAircraftType type);
    bool initOk(void);
    String sVersion;
protected:
    String FlarmExp;
    HardwareSerial *pFanetSerial;  
    NmeaOut *pNmeaOut;  
    uint8_t initCount;
    void initModule(uint32_t tAct);
    void DecodeLine(String line);
    void CheckReceivedPackage(String line);
    void getTrackingInfo(String line,uint16_t length);
    int getByteFromHex(char in[]);
    String getHexFromByte(uint8_t val);
    uint32_t tCheckModuleState;
    String getStringValue(String s,String keyword);
    int getStringValue(String s,String *sRet,unsigned int fromIndex,String keyword,String delimiter);
    trackingData actTrackingData;
    bool newData = false;
    uint8_t _ResetPin;
    int8_t _BootPin;
    String _PilotName;
    void sendWeather(void);
    uint32_t tWeather;
    void coord2payload_absolut(float lat, float lon, uint8_t *buf);
    void encodeTrackingData(trackingData tData,uint8_t *buf);
    void printAircraftType(eFanetAircraftType type);
    void resetModule();
    void sendPilotName(uint32_t tAct);
    void getMyID(String line);
    void getVersion(String line);
    void getFAX(String line);

    char lineBuffer[FANET_MAXRECBUFFER];
    uint8_t recBufferIndex;
    trackingData _myData;
    bool bDGVOk;
    bool bFNAOk;
    bool bFNCOk;
    bool bDGPOk;
    bool bFAPOk;
    bool bFAXOk;
    bool bInitOk;
    bool bSetFlyingState;
    bool _flyingState = false;
    
};



#endif