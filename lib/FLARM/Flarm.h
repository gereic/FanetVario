/*!
 * @file Flarm.h
 *
 *
 */

#ifndef __FLARM_H__
#define __FLARM_H__


#include <HardwareSerial.h>
#include "CalcTools.h"

enum class eFlarmAircraftType {
  UNKNOWN = 0,
  GLIDER_MOTOR_GLIDER = 1,
  TOW_PLANE = 2,
  HELICOPTER_ROTORCRAFT = 3,
  SKYDIVER = 4,
  DROP_PLANE_SKYDIVER = 5,
  HANG_GLIDER = 6,
  PARA_GLIDER = 7,
  AIRCRAFT_RECIPROCATING_ENGINE = 8,
  AIRCRAFT_JET_TURBO_ENGINE = 9,
  RESERVED10 = 10,
  BALLOON = 11,
  AIRSHIP = 12,
  UAV = 13,
  RESERVED14 = 14,
  STATIC_OBJECT = 15
};


typedef struct {
  String DevId;
  float lat; //latitude
  float lon; //longitude
  uint16_t altitude; //altitude [m]
  eFlarmAircraftType aircraftType; //
  float speed; //km/h
  float climb; //m/s
  float heading; //deg
} FlarmtrackingData;



class Flarm {
public:
    Flarm(); //constructor
    bool begin();
    void run(void); //has to be called cyclic
    void writeFlarmData(FlarmtrackingData *myData,FlarmtrackingData *movePilotData);
protected:
    void writeDataPort(uint32_t tAct);
    String addChecksum(String s);
    String getHexFromByte(uint8_t val);
    String getHexFromByte1(uint8_t val);
};
#endif