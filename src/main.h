#include <Arduino.h>
#include "Fanet.h"
#include "nmeaout.h"
#include "tools.h"

#ifndef __MAIN_H__
#define __MAIN_H__

#define VERSION "V2.0.0"
#define APPNAME "GXFANET"

#define NUMBUTTONS 2
#define LONGPRESSTIME 250

#define MAXSTRING 255
#define BLE_LOW_HEAP 10000
#define MAX_BLE_LOW_HEAP_TIME 30000
#define DISPLAY_UPDATE_RATE 500

#define MIN_FLIGHT_SPEED 15.0 //min speed for flying-detection 
// > at least for 5sec --> takeoff
// < at least for 60sec --> landing
#define MIN_FLIGHT_TIME 5000
#define MIN_GROUND_TIME 60000


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
  float BattVoltOffs = 0.0; //offset for Battery-multiplier
  eFanetAircraftType AircraftType; //Aircrafttype
  WifiSettings wifi;
  String UDPServerIP; //UDP-IP-Adress for sending Pakets
  uint16_t UDPSendPort; //Port of udp-server
  eNMEAOUTPUT NMEAOUTPUT; //output for NMEA sentences
  uint8_t OutputBLE; //output also via BLE ??
  unsigned long baudRate;
};

struct NMEAStatusData{
  float lat; //latitude
  float lon; //longitude
  uint8_t numSat; //number ob sattelites
  bool GPSFix; //gps fixed
};

struct statusGPS{
  uint8_t NumSat;
  uint8_t Fix;
  double Lat;
  double Lon;
  float alt;
  float speed;
  float course;
  float geoidAlt;
};

struct batStatus{
  bool charging;  
  uint16_t voltage; //battery-voltage 1/1000V
  uint8_t percent; //battery-percent
};

struct StatusData{
  String myIP; //my IP-Adress
  uint8_t wifiStat;
  statusGPS gps;
  NMEAStatusData NMEAStat;
  batStatus battery;
  uint8_t bluetoothStat;
  bool flying;
  uint32_t flightTime; //flight-time in sek.
  uint8_t GPS_Fix;
  uint32_t tLoop; //current Loop-time
  uint32_t tMaxLoop; //max Loop-time
  uint8_t UpdateFanetModule;
  bool bPowerOff;
};

#endif
