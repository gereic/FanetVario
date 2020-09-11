#include "fileOps.h"

Preferences preferences;  

void load_configFile(SettingsData* pSetting){
    log_i("LOAD CONFIG FILE");
    preferences.begin("settings", false);                         //Ordner settings anlegen/verwenden
    pSetting->PilotName = preferences.getString("PILOTNAME","Gerald Eichler");
    pSetting->AircraftType = (eFanetAircraftType)preferences.getUChar("AIRCRAFTTYPE",1);
    pSetting->wifi.connect = preferences.getUChar("WIFI_CONNECT",0); //
    pSetting->wifi.ssid = preferences.getString("WIFI_SSID","");
    pSetting->wifi.password = preferences.getString("WIFI_PW","");
    pSetting->wifi.tWifiStop = preferences.getUInt("Time_WIFI_Stop",0);
    pSetting->wifi.appw = preferences.getString("APPW","12345678");
    pSetting->UDPServerIP = preferences.getString("UDP_SERVER",""); //UDP-IP-Adress for sending Pakets
    pSetting->UDPSendPort = preferences.getUInt("UDP_PORT",10110); //Port of udp-server
    pSetting->NMEAOUTPUT = (eNMEAOUTPUT)preferences.getUChar("NMEAOUTPUT",0); //output for NMEA sentences
    pSetting->OutputBLE = preferences.getUChar("OUT_BLE",0); //output also to BLE
    preferences.end(); 
}

void write_configFile(SettingsData* pSetting){
    log_i("WRITE CONFIG FILE");
    preferences.begin("settings", false);                         //Ordner settings anlegen/verwenden
    preferences.putString("PILOTNAME",pSetting->PilotName);
    preferences.putUChar("AIRCRAFTTYPE",uint8_t(pSetting->AircraftType));
    preferences.putString("APPW",pSetting->wifi.appw);
    preferences.putUChar("WIFI_CONNECT",pSetting->wifi.connect); //
    preferences.putString("WIFI_SSID",pSetting->wifi.ssid);
    preferences.putString("WIFI_PW",pSetting->wifi.password);
    preferences.putUInt("Time_WIFI_Stop",pSetting->wifi.tWifiStop);    
    preferences.putString("UDP_SERVER",pSetting->UDPServerIP); //UDP-IP-Adress for sending Pakets
    preferences.putUInt("UDP_PORT",pSetting->UDPSendPort); //Port of udp-server
    preferences.putUChar("NMEAOUTPUT",(uint8_t)pSetting->NMEAOUTPUT); //output for NMEA sentences
    preferences.putUChar("OUT_BLE",pSetting->OutputBLE); //output also to BLE
    preferences.end(); 
    ESP.restart();
}