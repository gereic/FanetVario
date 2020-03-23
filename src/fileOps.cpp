#include "fileOps.h"

Preferences preferences;  

void load_configFile(void){
    Serial.println("LOAD CONFIG FILE");
    preferences.begin("settings", false);                         //Ordner settings anlegen/verwenden
    setting.PilotName = preferences.getString("PILOTNAME","Gerald Eichler");
    setting.ssid = preferences.getString("WIFI_SSID","WLAN_EICHLER");
    setting.password = preferences.getString("WIFI_PW","magest172");
    setting.AircraftType = (eFanetAircraftType)preferences.getUChar("AIRCRAFTTYPE",1);
    setting.bSwitchWifiOff3Min = preferences.getBool("SWOFF3MIN",true);
    setting.UDPServerIP = preferences.getString("UDP_SERVER",""); //UDP-IP-Adress for sending Pakets
    setting.UDPSendPort = preferences.getUInt("UDP_PORT",10110); //Port of udp-server
    setting.NMEAOUTPUT = (eNMEAOUTPUT)preferences.getUChar("NMEAOUTPUT",0); //output for NMEA sentences
    preferences.end(); 
}

void write_configFile(void){
    Serial.println("WRITE CONFIG FILE");
    preferences.begin("settings", false);                         //Ordner settings anlegen/verwenden
    preferences.putString("PILOTNAME",setting.PilotName);
    preferences.putString("WIFI_SSID",setting.ssid);
    preferences.putString("WIFI_PW",setting.password);
    preferences.putUChar("AIRCRAFTTYPE",uint8_t(setting.AircraftType));
    preferences.putBool("SWOFF3MIN",setting.bSwitchWifiOff3Min);    
    preferences.putString("UDP_SERVER",setting.UDPServerIP); //UDP-IP-Adress for sending Pakets
    preferences.putUInt("UDP_PORT",setting.UDPSendPort); //Port of udp-server
    preferences.putUChar("NMEAOUTPUT",(uint8_t)setting.NMEAOUTPUT); //output for NMEA sentences
    preferences.end(); 
    ESP.restart();
}