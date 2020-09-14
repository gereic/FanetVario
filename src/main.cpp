#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>
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
#include <ble.h>

#include "main.h"


/***************  Configuration Begin *******************/

String host_name = "";

/***************  Configuration End *********************/

const char compile_date[] = __DATE__ " " __TIME__;

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
struct StatusData status;
bool WebUpdateRunning = false;
Fanet fanet;
NmeaOut nmeaout;

IPAddress local_IP(192,168,4,1);
IPAddress gateway(192,168,4,250);
IPAddress subnet(255,255,255,0);

bool WifiConnectOk = true;
static RTC_NOINIT_ATTR uint8_t startOption;


unsigned long ble_low_heap_timer=0;
String ble_data="";
bool ble_mutex=false;


TaskHandle_t xHandleBackground = NULL;
TaskHandle_t xHandleBluetooth = NULL;


void taskBackGround(void *pvParameters);
void taskBluetooth(void *pvParameters);
void WiFiEvent(WiFiEvent_t event);
void sendData2Client(String data);
void listSpiffsFiles();

void listSpiffsFiles(){
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while(file){  
    log_i("FILE: %s len=%d",file.name(),file.size());
    file = root.openNextFile();
  }  
}


void sendData2Client(String data){
  if (data.length() <= 0) return; //nothing to send
  if (setting.OutputBLE){ //output over ble-connection
    if (xHandleBluetooth){
      if ((ble_data.length() + data.length()) <512){
        while(ble_mutex){
          delay(100);
        };
        ble_data=ble_data+data;
      }else{
        if (!ble_mutex){
          ble_data="";
        }
      }
    }else{
      ble_data = "";
      ble_mutex = false;
    }
  }
}


void printSettings(){
  log_i("**** SETTINGS ****");
  log_i("Access-point password=%s",setting.wifi.appw.c_str());
  log_i("WIFI connect=%d",setting.wifi.connect);
  log_i("WIFI SSID=%s",setting.wifi.ssid.c_str());
  log_i("WIFI PW=%s",setting.wifi.password.c_str());
  log_i("Wifi-down-time=%d",setting.wifi.tWifiStop);
  log_i("Aircraft=%s",fanet.getAircraftType(setting.AircraftType).c_str());
  log_i("Pilotname=%s",setting.PilotName.c_str());
  //Serial.print("Switch WIFI OFF after 3 min=");Serial.println(setting.bSwitchWifiOff3Min);
  //Serial.print("Wifi-down-time=");Serial.println(setting.wifiDownTime/1000.);
  log_i("UDP_SERVER=%s",setting.UDPServerIP.c_str());
  log_i("UDP_PORT=%d",setting.UDPSendPort);
  log_i("OUTPUT TO BLE=%d",setting.OutputBLE);
  /*
  Serial.print("NMEA OUTPUT=");
  if (setting.NMEAOUTPUT == eNMEAOUTPUT::UDP_OUT){
    Serial.println("UDP");
  }else if (setting.NMEAOUTPUT == eNMEAOUTPUT::BLUETOOTH_OUT){
    Serial.println("BLUETOOTH");
  }else{
    Serial.println("SERIAL");
  }
  */
  
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

void WiFiEvent(WiFiEvent_t event){
  switch(event){
    case SYSTEM_EVENT_AP_START:
      log_d("AP started. IP: [%s]", WiFi.softAPIP().toString().c_str() );
      break;
    case SYSTEM_EVENT_AP_STOP:
      log_d("AP Stopped");
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      log_d("WiFi Client Disconnected");
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      log_d("SYSTEM_EVENT_AP_STACONNECTED");
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      //log_i("SYSTEM_EVENT_STA_GOT_IP!!!!!!!!!!!!");
      status.myIP = WiFi.localIP().toString();
      log_i("my IP=%s",status.myIP.c_str());
      break;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
      log_i("SYSTEM_EVENT_AP_STAIPASSIGNED!!!!!!!!!!!!");
      break;

    default:
      log_d("Unhandled WiFi Event: %d", event );
      break;
  }
}



void setupWifi(){
  while(host_name.length() == 0){
    delay(100); //wait until we have the devid
  }
  status.wifiStat = 0;
  WiFi.mode(WIFI_OFF);
  //delay(500);
  WiFi.persistent(false);
  WiFi.onEvent(WiFiEvent);
  log_i("Setting soft-AP ... ");
  //if (WiFi.softAP(host_name.c_str(), setting.appw.c_str(),rand() % 12 + 1,0,2)){
  if (WiFi.softAP(host_name.c_str(), setting.wifi.appw.c_str())){
    log_i("Ready");
  }else{
    log_i("Failed!");
  }
  delay(10);
  log_i("Setting soft-AP configuration ... ");
  if(WiFi.softAPConfig(local_IP, gateway, subnet)){
    log_i("Ready");
  }else{
    log_i("Failed!");
  }
  delay(10);

  log_i("hostname=%s",host_name.c_str());
  WiFi.setHostname(host_name.c_str());
  //now configure access-point
  //so we have wifi connect and access-point at same time
  //we connecto to wifi
  if (setting.wifi.connect != WIFI_CONNECT_NONE){
    //esp_wifi_set_auto_connect(true);
    log_i("Try to connect to WiFi ...");
    WiFi.status();
    WiFi.mode(WIFI_MODE_APSTA);
    //WiFi.mode(WIFI_STA);
    if ((WiFi.SSID() != setting.wifi.ssid || WiFi.psk() != setting.wifi.password)){
      // ... Try to connect to WiFi station.
      WiFi.begin(setting.wifi.ssid.c_str(), setting.wifi.password.c_str());
      delay(2000);
    } else {
      // ... Begin with sdk config.
      WiFi.begin();
      delay(2000);
    }
    log_i("Wait for WiFi connection.");
    uint32_t wifiTimeout = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiTimeout < 10000) {
      delay(500);
    }    
  }
  status.wifiStat = 1;
  log_i("my APIP=%s",local_IP.toString().c_str());
  Web_setup();
}

void setup() {
  // put your setup code here, to run once:
  // Set pin mode
  pinMode(LED_BUILTIN,OUTPUT);
  ppsTriggered = false;
  pinMode(PPS_PIN,INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), IntPPS, FALLING);

  Serial.begin(115200);


    log_e("error");
  
  log_i("SDK-Version=%s",ESP.getSdkVersion());
  log_i("CPU-Speed=%d",ESP.getCpuFreqMHz());
  log_i("Total heap: %d", ESP.getHeapSize());
  log_i("Free heap: %d", ESP.getFreeHeap());
  log_i("Free PSRAM: %d", ESP.getFreePsram());
  log_i("compiled at %s",compile_date);
  log_i("current free heap: %d, minimum ever free heap: %d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());

  //esp_sleep_wakeup_cause_t reason = print_wakeup_reason(); //print reason for wakeup
  //print_wakeup_reason(); //print reason for wakeup
  esp_reset_reason_t reason = esp_reset_reason();
  if (reason != ESP_RST_SW) {
    startOption = 0;
  }
  log_i("startOption=%d",startOption);

    // Make sure we can read the file system
  if( !SPIFFS.begin(true)){
    log_e("Error mounting SPIFFS");
    while(1);
  }

  load_configFile(&setting); //load configuration

  //listSpiffsFiles();


  //setting.bSwitchWifiOff3Min = false;
  //setting.NMEAOUTPUT = eNMEAOUTPUT::BLUETOOTH_OUT;
  setting.NMEAOUTPUT = eNMEAOUTPUT::SERIAL_OUT;
  printSettings();
  
  xTaskCreatePinnedToCore(taskBackGround, "taskBackGround", 6500, NULL, 5, &xHandleBackground, ARDUINO_RUNNING_CORE1); //background task
  xTaskCreatePinnedToCore(taskBluetooth, "taskBluetooth", 4096, NULL, 7, &xHandleBluetooth, ARDUINO_RUNNING_CORE1);

  fanet.begin(1,17,16,5); //Hardwareserial, rxPin, txPin, Reset-Pin
  while (!fanet.initOk()){
    fanet.run(); //call run to get DevId
  }
  setting.myDevId = fanet.getMyDevId();
  setting.FlarmExp = fanet.getFlarmExp();
  setting.fanetVersion = fanet.sVersion;
  host_name = APPNAME "-" + setting.myDevId; //String((ESP32_getChipId() & 0xFFFFFF), HEX);

  //setupWifi();

  delay(1000);
  //fanet.updateModule("/fanet.xlb");
  //esp_restart();

  delay(500);
  nmeaout.begin(setting.NMEAOUTPUT,setting.UDPServerIP,setting.UDPSendPort,host_name);
  fanet.setNMEAOUT(&nmeaout); //Hardwareserial, rxPin, txPin, Reset-Pin)
  fanet.setPilotname(setting.PilotName); //set name of Pilot
  fanet.setAircraftType(setting.AircraftType);
  blueFly.begin(2,23,22,&nmeaout); //Hardwareserial, rxPin, txPin
  flarm.begin(&nmeaout);


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
  static float oldTurnrate = 0.0;
  stateData tFanetData;  
  tFanetData.lat = blueFly.nmea.getLatitude() / 1000000.;
  tFanetData.lon = blueFly.nmea.getLongitude() / 1000000.;
	long alt = 0;
  if (!blueFly.nmea.getAltitude(alt)){
    alt = long(blueFly.getAlt() * 1000);
    //Serial.println("alt not valid");
  }
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
	tFanetData.year = blueFly.nmea.getYear() - 1900;
	tFanetData.month = blueFly.nmea.getMonth();
	tFanetData.day = blueFly.nmea.getDay();
	tFanetData.hour = blueFly.nmea.getHour();
	tFanetData.minute = blueFly.nmea.getMinute();
	tFanetData.second = blueFly.nmea.getSecond();
  tFanetData.turnrate = tFanetData.heading - oldTurnrate;
  oldTurnrate = tFanetData.heading;
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
  static uint32_t tLoop = millis();
  static uint32_t tSerialRec = millis();

  status.tLoop = tAct - tLoop;
  tLoop = tAct;
  if (status.tMaxLoop < status.tLoop) status.tMaxLoop = status.tLoop;

  BlinkLed(tAct);
  fanet.run();
  if (status.UpdateFanetModule){
    if (fanet.updateModule("/fanet.xlb") == 0){
      esp_restart();
    }
    status.UpdateFanetModule = 0;
  }
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
    //log_i("PPS triggered");
    sendFanetStatus();
  }
  nmeaout.run();
  sendData2Client(nmeaout.getSendData());
}

void taskBackGround(void *pvParameters){
  static uint32_t tLife = millis();
  static uint32_t tWifiCheck = millis();
  static uint8_t counter = 0;
  static uint32_t warning_time=0;


  if (startOption != 0){ //we start wifi
    log_i("stop task");
    vTaskDelete(xHandleBackground);
    return;
  }

  setupWifi();
  while (1){
    uint32_t tAct = millis();
    if ((setting.wifi.connect == 2) && (WiFi.status() != WL_CONNECTED) && (status.wifiStat)){
      if (timeOver(tAct,tWifiCheck,WIFI_RECONNECT_TIME)){
        tWifiCheck = tAct;
        log_i("WiFi not connected. Try to reconnect");
        WiFi.disconnect();
        WiFi.mode(WIFI_OFF);
        WiFi.persistent(false);
        WiFi.mode(WIFI_MODE_APSTA);
        WiFi.begin(setting.wifi.ssid.c_str(), setting.wifi.password.c_str());        
      }
    }
    if (xPortGetMinimumEverFreeHeapSize()<5000)
    {
      log_e( "*****LOOP current free heap: %d, minimum ever free heap: %d ******", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
      log_e("System Low on Memory - xPortGetMinimumEverFreeHeapSize < 2KB");
      log_e("ESP Restarting !");
      esp_restart();
    }

    if  (status.wifiStat){
      Web_loop();
    }
    if (( tAct > (setting.wifi.tWifiStop * 1000)) && (setting.wifi.tWifiStop!=0) && (!WebUpdateRunning)){
      log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
      Web_stop();
      WiFi.softAPdisconnect(true);
      WiFi.disconnect();
      WiFi.mode(WIFI_MODE_NULL);
      //WiFi.forceSleepBegin(); //This also works
      setting.wifi.tWifiStop=0;
      status.wifiStat=0;
      esp_wifi_set_mode(WIFI_MODE_NULL);
      esp_wifi_stop();
      log_i("******************WEBCONFIG Setting - WIFI STOPPING*************************");
      log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
      if (setting.OutputBLE){
        startOption = 1; //start without wifi, but with bluetooth enabled
        ESP.restart(); //we restart without wifi
      }
    }
    delay(1);
	}
}

void taskBluetooth(void *pvParameters) {

	// BLEServer *pServer;

  while(host_name.length() == 0){
    delay(100); //wait until we have the devid
  }
  if (setting.OutputBLE){
    if (startOption == 1){
      //startBluetooth(); //start bluetooth
    }else{
      log_i("stop task");
      vTaskDelete(xHandleBluetooth);
      return;    
    }
  }else{
    //stop bluetooth-controller --> save some memory
    esp_bt_controller_disable();
    esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
    //log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
    log_i("stop task");
    vTaskDelete(xHandleBluetooth);
    return;    
  }

  if (setting.OutputBLE){
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);    
    //log_i("currHeap:%d,minHeap:%d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());    
    start_ble(host_name);
    status.bluetoothStat = 1;
	 while (1)
	 {
	   // only send if we have more than 31k free heap space.
	   if (xPortGetFreeHeapSize()>BLE_LOW_HEAP)
	   {
		   ble_low_heap_timer = millis();
		   if (ble_data.length()>0)
           {
			   ble_mutex=true;
			   BLESendChunks(ble_data);
			   ble_data="";
			   ble_mutex=false;
           }
	   }
	   else
	   {
		   log_d( " BLE congested - Waiting - Current free heap: %d, minimum ever free heap: %d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
		   ble_mutex=true;
		   log_d("CLEARING BLEOUT");
		   ble_data="";
		   ble_mutex=false;
	   }
	   vTaskDelay(100);
	 }
  }

}
