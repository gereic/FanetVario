#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include "time.h"
#include <math.h>
#include <HardwareSerial.h>
#include "BlueFly.h"
#include "Flarm.h"
#include "CalcTools.h"
#include "WebHelper.h"
#include "fileOps.h"

#include "main.h"


/***************  Configuration Begin *******************/



#define CON2WIFI

String host_name = HOSTNAME;

#define OTAPROGRAMMING

/***************  Configuration End *********************/

#define ARDUINO_RUNNING_CORE0 0
#define ARDUINO_RUNNING_CORE1 1

#define LED_BUILTIN 2
#define PPS_PIN 4

#define Serial_MAXRECBUFFER 255
char SerialRecBuffer[Serial_MAXRECBUFFER];
uint8_t SerialRecBufferIndex;

BlueFly blueFly;
Flarm flarm;
bool ppsTriggered;
struct SettingsData setting;
Fanet fanet;
NmeaOut nmeaout;

/*
static union {
  uint8_t efuse_mac[6];
  uint64_t chipmacid;
};
*/
IPAddress local_IP(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
/**
 * Default WiFi connection information.
 *
 */
const char* ap_default_psk = "12345678"; ///< Default PSK.


WebServer server(80);

bool WifiConnectOk = true;

/*
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;
*/

void web_task(void *pvParameters);

void printSettings(){
  Serial.println("**** SETTINGS ****");
  Serial.print("WIFI SSID=");Serial.println(setting.ssid);
  Serial.print("WIFI PW=");Serial.println(setting.password);
  Serial.print("Aircraft=");Serial.println(fanet.getAircraftType(setting.AircraftType));
  Serial.print("Pilotname=");Serial.println(setting.PilotName);
  Serial.print("Switch WIFI OFF after 3 min=");Serial.println(setting.bSwitchWifiOff3Min);
  Serial.print("Wifi-down-time=");Serial.println(setting.wifiDownTime/1000.);
  Serial.print("UDP_SERVER=");Serial.println(setting.UDPServerIP);
  Serial.print("UDP_PORT=");Serial.println(setting.UDPSendPort);
  Serial.print("NMEA OUTPUT=");
  if (setting.NMEAOUTPUT == eNMEAOUTPUT::UDP_OUT){
    Serial.println("UDP");
  }else if (setting.NMEAOUTPUT == eNMEAOUTPUT::BLUETOOTH_OUT){
    Serial.println("BLUETOOTH");
  }else{
    Serial.println("SERIAL");
  }
  
}

void IntPPS(){
  ppsTriggered = true;
}

/*
static uint32_t ESP32_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  return (uint32_t) efuse_mac[5]        | (efuse_mac[4] << 8) | \
                   (efuse_mac[3] << 16) | (efuse_mac[2] << 24);
#else
  return (SOFTRF_ADDRESS & 0xFFFFFFFFU );
#endif // SOFTRF_ADDRESS 
}
*/

void setupWifi(){
  //chipmacid = ESP.getEfuseMac();
  //host_name += String((ESP32_getChipId() & 0xFFFFFF), HEX);
  

  //if ((setting.ssid.length() > 0) && (setting.password.length() > 0)){
    WiFi.mode(WIFI_STA);
    delay(10);
    if (WiFi.SSID() != setting.ssid || WiFi.psk() != setting.password)
    {
      Serial.println(F("WiFi config changed."));

      // ... Try to connect to WiFi station.
      WiFi.begin(setting.ssid.c_str(), setting.password.c_str());

      // ... Pritn new SSID
      Serial.print(F("new SSID: "));
      Serial.println(WiFi.SSID());

    }
    else
    {
      // ... Begin with sdk config.
      WiFi.begin();
    }
    WiFi.setHostname(host_name.c_str());
    Serial.println("Hostname: " + host_name);
    Serial.println(F("Wait for WiFi connection."));
    uint32_t wifiTimeout = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiTimeout < 10000) {
      delay(500);
      Serial.print(".");
    }Serial.println("");
  //}
  if(WiFi.status() == WL_CONNECTED){
    // ... print IP Address
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());
  } else{
    Serial.println(F("Can not connect to WiFi station. Go into AP mode."));
    // Go into software AP mode.
    WiFi.mode(WIFI_AP);
    delay(10);
    //WiFi.setHostname(host_name.c_str());
    Serial.print(F("Setting soft-AP configuration ... "));
    Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ?
      F("Ready") : F("Failed!"));
    Serial.print(F("Setting soft-AP ... "));
    Serial.println(WiFi.softAP(host_name.c_str(), ap_default_psk) ?
      F("Ready") : F("Failed!"));    
    Serial.print(F("IP address: "));
    Serial.println(WiFi.softAPIP());
  }
  /*use mdns for host name resolution*/
  if (!MDNS.begin(host_name.c_str())) { //http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    //while (1) {
    //  delay(1000);
    //}
  }
  WifiConnectOk = true;
  Web_setup();
  /* ********************** OTA Programming *******************/
  #ifdef OTAPROGRAMMING
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();  
  #endif  
  //}
  
  //move Webinterface to core 1, so nothing is interrupted
  xTaskCreatePinnedToCore(web_task, "web_task", 6500, NULL, 1, NULL, ARDUINO_RUNNING_CORE1);  
  Serial.println("WIFI-Setup finished");
}

void setup() {
  // put your setup code here, to run once:
  // Set pin mode
  pinMode(LED_BUILTIN,OUTPUT);
  ppsTriggered = false;
  pinMode(PPS_PIN,INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), IntPPS, FALLING);

  Serial.begin(115200);

  load_configFile(); //load configuration

  //setting.bSwitchWifiOff3Min = false;
  if ((setting.bSwitchWifiOff3Min) && (setting.NMEAOUTPUT != eNMEAOUTPUT::UDP_OUT)){
    setting.wifiDownTime = 180000;
  }else{
    setting.wifiDownTime = 0; //don't switch off wifi in case of udp output
  }
  printSettings();
  

  fanet.begin(1,16,17,15); //Hardwareserial, rxPin, txPin, Reset-Pin
  while (!fanet.initOk()){
    fanet.run(); //call run to get DevId
  }
  setting.myDevId = fanet.getMyDevId();
  setting.FlarmExp = fanet.getFlarmExp();
  host_name += setting.myDevId; //String((ESP32_getChipId() & 0xFFFFFF), HEX);

  setupWifi();


  delay(500);
  nmeaout.begin(setting.NMEAOUTPUT,setting.UDPServerIP,setting.UDPSendPort,host_name);
  fanet.setNMEAOUT(&nmeaout); //Hardwareserial, rxPin, txPin, Reset-Pin)
  fanet.setPilotname(setting.PilotName); //set name of Pilot
  fanet.setAircraftType(setting.AircraftType);
  blueFly.begin(2,23,22,&nmeaout); //Hardwareserial, rxPin, txPin
  flarm.begin(&nmeaout);

}

void web_task(void *pvParameters) {
  //delay(5000);
  while (1){
    if  (WifiConnectOk){
      Web_loop();
      #ifdef OTAPROGRAMMING
      ArduinoOTA.handle();
      #endif
    }
    if (( millis() > setting.wifiDownTime) && (setting.wifiDownTime!=0)){
      setting.wifiDownTime=0;
      WifiConnectOk=false;
      esp_wifi_set_mode(WIFI_MODE_NULL);
      esp_wifi_stop();
      //Serial.println("******************WEBCONFIG Setting - WIFI STOPPING************************* ");
    }
    delay(1);
	}
 }

void BlinkLed(uint32_t tAct){
  static uint32_t tLed = millis();
  if ((tAct - tLed) >= 500){
      tLed = tAct;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void sendFanetStatus(){
  static long oldAlt = 0;
  stateData tFanetData;  
  tFanetData.lat = blueFly.nmea.getLatitude() / 1000000.;
  tFanetData.lon = blueFly.nmea.getLongitude() / 1000000.;
	long alt = 0;
  blueFly.nmea.getAltitude(alt);
  if (oldAlt == 0){
    oldAlt = alt;
  }

  tFanetData.altitude = alt/1000.;
  long geoIdAlt = 0;
  blueFly.nmea.getGeoIdAltitude(geoIdAlt);
  tFanetData.geoIdAltitude = geoIdAlt/1000.;
  tFanetData.speed = blueFly.nmea.getSpeed()*1.852/1000.;
  tFanetData.climb = (alt - oldAlt)/1000.;
  oldAlt = alt;
  tFanetData.heading = blueFly.nmea.getCourse()/1000.;
	tFanetData.year = blueFly.nmea.getYear();
	tFanetData.month = blueFly.nmea.getMonth();
	tFanetData.day = blueFly.nmea.getDay();
	tFanetData.hour = blueFly.nmea.getHour();
	tFanetData.minute = blueFly.nmea.getMinute();
	tFanetData.second = blueFly.nmea.getSecond();
  fanet.writeStateData2FANET(&tFanetData);
}

eFlarmAircraftType Fanet2FlarmAircraft(eFanetAircraftType aircraft){
  switch (aircraft)
  {
  case eFanetAircraftType::UNKNOWN:
    return eFlarmAircraftType::UNKNOWN;
  case eFanetAircraftType::PARA_GLIDER:
    return eFlarmAircraftType::PARA_GLIDER;
  case eFanetAircraftType::HANG_GLIDER:
    return eFlarmAircraftType::HANG_GLIDER;
  case eFanetAircraftType::BALLOON:
    return eFlarmAircraftType::BALLOON;
  case eFanetAircraftType::GLIDER:
    return eFlarmAircraftType::GLIDER_MOTOR_GLIDER;
  case eFanetAircraftType::POWERED_AIRCRAFT:
    return eFlarmAircraftType::TOW_PLANE;
  case eFanetAircraftType::HELICOPTER_ROTORCRAFT:
    return eFlarmAircraftType::HELICOPTER_ROTORCRAFT;
  case eFanetAircraftType::UAV:
    return eFlarmAircraftType::UAV;
  }
  return eFlarmAircraftType::UNKNOWN;
}

void Fanet2FlarmData(trackingData *FanetData,FlarmtrackingData *FlarmDataData){
  FlarmDataData->aircraftType = Fanet2FlarmAircraft(FanetData->aircraftType);
  FlarmDataData->altitude = FanetData->altitude;
  FlarmDataData->climb = FanetData->climb;
  FlarmDataData->DevId = FanetData->DevId;
  FlarmDataData->heading = FanetData->heading;
  FlarmDataData->lat = FanetData->lat;
  FlarmDataData->lon = FanetData->lon;
  FlarmDataData->speed = FanetData->speed;
}

void loop() {
  trackingData tFanetData;  
  trackingData myFanetData;  
  FlarmtrackingData myFlarmData;
  FlarmtrackingData PilotFlarmData;
  uint32_t tAct = millis();
  static uint32_t tSerialRec = millis();
  BlinkLed(tAct);
  fanet.run();
  if (fanet.getTrackingData(&tFanetData)){
    fanet.getMyTrackingData(&myFanetData);
    Fanet2FlarmData(&myFanetData,&myFlarmData);
    Fanet2FlarmData(&tFanetData,&PilotFlarmData);
    //fanet.printFanetData(tFanetData);
    //fanet.printFanetData(myFanetData);
    flarm.writeFlarmData(&myFlarmData,&PilotFlarmData);
  }
  blueFly.run();
  flarm.run();
  //send Data directly to bluefly for configuration !!
  if (Serial.available()){
    if (SerialRecBufferIndex == 0){
      tSerialRec = tAct;
    }
    if (SerialRecBufferIndex >= FANET_MAXRECBUFFER) SerialRecBufferIndex = 0; //Buffer overrun
    SerialRecBuffer[SerialRecBufferIndex] = Serial.read(); //read char from serial Line
    if (SerialRecBuffer[SerialRecBufferIndex] == '\n'){
      //Serial.print("length=");Serial.println(recBufferIndex);
      SerialRecBuffer[SerialRecBufferIndex] = 0; //zero-termination
      //Serial.println(lineBuffer);
      blueFly.writeToSerial(String(SerialRecBuffer));
      SerialRecBufferIndex = 0; //reset buffer
    }else{
      if (SerialRecBuffer[SerialRecBufferIndex] != '\r'){
        SerialRecBufferIndex++;
      }
    }    
  }
  if (((tAct - tSerialRec) > 100) && (SerialRecBufferIndex > 0)){
    SerialRecBuffer[SerialRecBufferIndex] = 0; //zero-termination
    blueFly.writeToSerial(String(SerialRecBuffer));
    SerialRecBufferIndex = 0; //reset Buffer
  }
  
  if (ppsTriggered){
    ppsTriggered = false;
    //Serial.println("PPS triggered");
    sendFanetStatus();
  }
  nmeaout.run();
  /*
  if (WifiConnectOk){
    //server.handleClient();
    Web_loop();
    #ifdef OTAPROGRAMMING
    ArduinoOTA.handle();
    #endif
  }
  */
}