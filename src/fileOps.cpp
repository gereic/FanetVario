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
    preferences.end(); 
    ESP.restart();
}