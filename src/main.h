#include <Arduino.h>
#include "Fanet.h"

#ifndef __MAIN_H__
#define __MAIN_H__

#define MAIN_FIRMWARE_VERSION "0.9"
#define MAIN_IDENT            "FANETVARIO-"
#define HOSTNAME            "FANETVARIO-"


struct SettingsData{
  String ssid;
  String password;
  String PilotName;
  eFanetAircraftType AircraftType;
  bool bSwitchWifiOff3Min;
  uint32_t wifiDownTime;
};

struct NMEAStatudData{
  float lat; //latitude
  float lon; //longitude
  uint8_t numSat; //number ob sattelites
  bool GPSFix; //gps fixed
};

struct StatusData{
  NMEAStatudData NMEAStat;
};

#endif
