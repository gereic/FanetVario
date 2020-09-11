#include <Arduino.h>
#include "Fanet.h"
#include "nmeaout.h"
#include "tools.h"

#ifndef __MAIN_H__
#define __MAIN_H__

#define VERSION "V1.0.0"
#define APPNAME "GXFANET"

#define BLE_LOW_HEAP 10000
#define MAX_BLE_LOW_HEAP_TIME 30000


//defines for wifi connect
#define WIFI_CONNECT_NONE 0
#define WIFI_CONNECT_ONCE 1
#define WIFI_CONNECT_ALWAYS 2

#define WIFI_RECONNECT_TIME 60000



struct WifiSettings{
  String appw; //access-point-Password
  String ssid; //WIFI SSID
  String password; //WIFI PASSWORD
  uint8_t connect; //1 connect to wifi, 2 connect to wifi and try to stay connected
  uint32_t tWifiStop; //time after wifi will be stopped to save energy 0 --> never
};



struct SettingsData{
  String myDevId; //my device-ID
  String fanetVersion; //Version of FANET-Module
  String FlarmExp; //expiration of Flarm
  String PilotName; //Pilotname
  eFanetAircraftType AircraftType; //Aircrafttype
  WifiSettings wifi;
  String UDPServerIP; //UDP-IP-Adress for sending Pakets
  uint16_t UDPSendPort; //Port of udp-server
  eNMEAOUTPUT NMEAOUTPUT; //output for NMEA sentences
  uint8_t OutputBLE; //output also via BLE ??
};

struct NMEAStatusData{
  float lat; //latitude
  float lon; //longitude
  uint8_t numSat; //number ob sattelites
  bool GPSFix; //gps fixed
};

struct StatusData{
  String myIP; //my IP-Adress
  uint8_t wifiStat;
  NMEAStatusData NMEAStat;
  uint8_t bluetoothStat;
  uint8_t GPS_Fix;
  uint32_t tLoop; //current Loop-time
  uint32_t tMaxLoop; //max Loop-time
  uint8_t UpdateFanetModule;
};

#endif
