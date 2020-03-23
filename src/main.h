#include <Arduino.h>
#include "Fanet.h"
#include "nmeaout.h"

#ifndef __MAIN_H__
#define __MAIN_H__

#define MAIN_FIRMWARE_VERSION "0.9"
#define MAIN_IDENT            "FANETVARIO-"
#define HOSTNAME            "FANETVARIO-"




struct SettingsData{
  String myDevId; //my device-ID
  String FlarmExp; //expiration of Flarm
  String ssid; //WIFI SSID
  String password; //WIFI PASSWORD
  String PilotName; //Pilotname
  eFanetAircraftType AircraftType; //Aircrafttype
  bool bSwitchWifiOff3Min; //switch off wifi after 3min.
  uint32_t wifiDownTime;
  String UDPServerIP; //UDP-IP-Adress for sending Pakets
  uint16_t UDPSendPort; //Port of udp-server
  eNMEAOUTPUT NMEAOUTPUT; //output for NMEA sentences
};

struct NMEAStatusData{
  float lat; //latitude
  float lon; //longitude
  uint8_t numSat; //number ob sattelites
  bool GPSFix; //gps fixed
};

struct StatusData{
  NMEAStatusData NMEAStat;
};

#endif
