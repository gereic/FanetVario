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
#include <AceButton.h>

#include "main.h"
#include "TimeDefs.h"

#if defined(SSD1306)
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <oled.h>
#endif


/***************  Configuration Begin *******************/

String host_name = "";

/***************  Configuration End *********************/

//const char compile_date[] = __DATE__ " " __TIME__;
String compile_date = String(BUILD_DATE_YEAR_CH0) + String(BUILD_DATE_YEAR_CH1) + String(BUILD_DATE_YEAR_CH2) + String(BUILD_DATE_YEAR_CH3) + "-" + String(BUILD_DATE_MONTH_CH0) + String(BUILD_DATE_MONTH_CH1) + "-" + String(BUILD_DATE_DAY_CH0) + String(BUILD_DATE_DAY_CH1) + "T" + String(BUILD_TIME_HOUR_CH0) + String(BUILD_TIME_HOUR_CH1) + ":" + String(BUILD_TIME_MIN_CH0) + String(BUILD_TIME_MIN_CH1) + ":" + String(BUILD_TIME_SEC_CH0) + String(BUILD_TIME_SEC_CH1);

#define ARDUINO_RUNNING_CORE0 0
#define ARDUINO_RUNNING_CORE1 1

#if defined(SSD1306)
//Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);
//Oled display;
#endif



int8_t pinExtCtrl = -1;
int8_t pinAdcCtrl = -1;
int8_t pinBat = -1;

#ifdef Heltec_Wifi_V3
#define PIN_LED 35
#define PPS_PIN 5
#define FANET_RX 3
#define FANET_TX 4
#define FANET_RST 2
#define FANET_BOOT 46
#define GPS_RX 42
#define GPS_TX 45
#define PIN_BAT 1
#define PIN_DISPLAY_SDA 17
#define PIN_DISPLAY_SCL 18
#define PIN_DISPLAY_RST 21

#else
#define PIN_LED 2
#define PPS_PIN 4
#define FANET_RX 17
#define FANET_TX 16
#define FANET_RST 5
#define GPS_RX 23
#define GPS_TX 22
#endif

#define Serial_MAXRECBUFFER 255
char SerialRecBuffer[Serial_MAXRECBUFFER];
uint8_t SerialRecBufferIndex;
int8_t PinADCVoltage = -1;
float adcVoltageMultiplier = 0.0;
uint16_t battFull = 4050;
uint16_t battEmpty = 3300;

TwoWire *pI2cOne = &Wire1;

BlueFly blueFly;
Flarm flarm;
volatile bool ppsTriggered;
volatile uint32_t tPPS = millis();
struct SettingsData setting;
struct StatusData status;
bool WebUpdateRunning = false;
bool bPowerOff = false;
Fanet fanet;
NmeaOut nmeaout;


//buttons
struct myButtons {
  int8_t PinButton = -1;
  uint8_t state = 0;
};
ace_button::AceButton buttons[NUMBUTTONS];
myButtons sButton[NUMBUTTONS];

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
TaskHandle_t xHandleOled = NULL;


void taskBackGround(void *pvParameters);
void taskBluetooth(void *pvParameters);
void taskOled(void *pvParameters);
void WiFiEvent(WiFiEvent_t event);
void sendData2Client(String data);
void listSpiffsFiles();
int addChecksum(char *buffer, size_t size);
bool sendCmd2NMEA(const char* s,const char* sRet,uint32_t tTimeout);
bool setupQuetelGps(void);
bool readBatt(uint32_t tAct);
float readBattvoltage();
void checkFlyingState(uint32_t tAct);
void handleEvent(ace_button::AceButton* button, uint8_t eventType, uint8_t buttonState);
void powerOff();

void powerOff(){
  bPowerOff = true;
  delay(100);
  log_i("wait until all tasks are stopped");
  eTaskState tOled = eDeleted;
  eTaskState tBackground = eDeleted;
  eTaskState tBluetooth = eDeleted;
  while(1){
    //wait until all tasks are stopped
    if (xHandleBackground != NULL) tBackground = eTaskGetState(xHandleBackground);
    if (xHandleBluetooth != NULL) tBluetooth = eTaskGetState(xHandleBluetooth);
    if (xHandleOled != NULL) tOled = eTaskGetState(xHandleOled);
    if (((tBackground == eDeleted) || (tBackground == eReady)) && 
        ((tBluetooth == eDeleted) || (tBluetooth == eReady)) &&
        ((tOled == eDeleted) || (tOled == eReady))) break; //now all tasks are stopped
    log_i("Background=%d,Bluetooth=%d,oled=%d",tBackground,tBluetooth,tOled);
    delay(1000);
  }
  digitalWrite(PIN_LED,HIGH);
  log_i("switch power-supply off");  
  delay(100);
  if ((sButton[0].PinButton >= 0)){
    while (!digitalRead(sButton[0].PinButton)){
      log_i("wait until power-button is released");
      delay(500); //wait 500ms
    }
    uint64_t mask = uint64_t(1) << uint64_t(sButton[0].PinButton); //prg-Button has a pull-up-resistor --> Pin has to go low, if pressed
    esp_sleep_enable_ext1_wakeup(mask,ESP_EXT1_WAKEUP_ALL_LOW); //wait until power is back again
  }
  

  //Serial.end();
  //esp_wifi_stop();
  //#ifdef BLUETOOTH
  #ifndef S3CORE
  esp_bluedroid_disable();
  esp_bluedroid_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  esp_bt_mem_release(ESP_BT_MODE_BTDM);
  #endif
  //#endif
  if (pinExtCtrl >= 0) digitalWrite(pinExtCtrl,HIGH); //switch ext-power off
  if (pinAdcCtrl >= 0) digitalWrite(pinAdcCtrl,HIGH); //switch adc-control off

  esp_deep_sleep_start();
}


// The event handler for the button.
void handleEvent(ace_button::AceButton* button, uint8_t eventType, uint8_t buttonState) {

  // Print out a message for all events.
  /*
  Serial.print(F("handleEvent(): eventType: "));
  Serial.print(eventType);
  Serial.print(F("; buttonState: "));
  Serial.println(buttonState);
  */

  // Get the LED pin
  uint8_t id = button->getId();

  // Control the LED only for the Pressed and Released events.
  // Notice that if the MCU is rebooted while the button is pressed down, no
  // event is triggered and the LED remains off.
  switch (eventType) {
    case ace_button::AceButton::kEventClicked:
      sButton[id].state = eventType;
      log_i("button %d clicked",id);
      break;
    case ace_button::AceButton::kEventLongPressed:
      sButton[id].state = eventType;
      //log_i("button %d long pressed",id);
      break;
    case ace_button::AceButton::kEventDoubleClicked:
      sButton[id].state = eventType;
      log_i("button %d double clicked",id);
      break;
  }
}

void checkFlyingState(uint32_t tAct){
  static uint32_t tOk = millis();
  static uint32_t tFlightTime = millis();
  if (status.flying){
    //flying
    if ((!status.gps.Fix) || (status.gps.speed < MIN_FLIGHT_SPEED)){
      if (timeOver(tAct,tOk,MIN_GROUND_TIME)){
        status.flying = false;
        fanet.setFlyingState(status.flying);
      }
    }else{
      tOk = tAct;
    }
    if (timeOver(tAct,tFlightTime,1000)){
      //1 sec. over --> inc. flight-time
      status.flightTime ++; //inc. flight-Time
      tFlightTime += 1000;
    }
  }else{
    //on ground
    if ((status.gps.Fix) && (status.gps.NumSat >= 4) && (status.gps.speed > MIN_FLIGHT_SPEED)){ //minimum 4 satelites to get accurade position and speed
      if (timeOver(tAct,tOk,MIN_FLIGHT_TIME)){
        tFlightTime = tAct;
        status.flightTime = 0;
        status.flying = true;
        fanet.setFlyingState(status.flying);
      }
    }else{
      tOk = tAct;
    }
  }

}



float readBattvoltage(){
  // multisample ADC
  const byte NO_OF_SAMPLES = 5;
  uint32_t adc_reading = 0;
  float vBatt = 0.0;
  if (PinADCVoltage < 0){
    return 0.0; //not voltage-pin defined !!
  }

  analogRead(PinADCVoltage); // First measurement has the biggest difference on my board, this line just skips the first measurement
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    uint16_t thisReading = analogRead(PinADCVoltage);
    adc_reading += thisReading;
  }
  adc_reading /= NO_OF_SAMPLES;

  vBatt = (adcVoltageMultiplier * float(adc_reading) / 1023.0f * 3.3f) + setting.BattVoltOffs;
  //log_i("adc=%d, vADC=%.3f, vBatt=%.3f",adc_reading,float(adc_reading) / 1023.0f * 3.3f,vBatt);
  return vBatt;
}

bool readBatt(uint32_t tAct){
  static uint32_t tBatt = millis() - 5000;
  if ((tAct - tBatt) >= 5000){
    tBatt = tAct;
    status.battery.voltage = uint16_t(readBattvoltage()*1000);
    status.battery.percent = scale(status.battery.voltage,battEmpty,battFull,0,100);
    return true; //bat-status readed
  }else{
    return false;
  }
}


int addChecksum(char *buffer, size_t size){
    uint8_t chk = 0;
    uint16_t tSize = 0;   
    while(*buffer){
        buffer++; //skip first char
        chk ^= *buffer;
        tSize++;
    }
    //Serial.printf("size=%d,tSize=%d,",size,tSize);
    tSize += snprintf(buffer,size-tSize,"*%02X\r\n",chk);
    //Serial.printf("tSize2=%d\r\n",tSize);
    return tSize;
}

bool sendCmd2NMEA(const char* s,const char* sRet,uint32_t tTimeout = 2000){
  char retBuffer[MAXSTRING];
  char buffer[MAXSTRING];
  int pos = 0;
  pos += snprintf(&buffer[pos],MAXSTRING-pos,s);
  pos = addChecksum(buffer,MAXSTRING);
  pos = 0;
  pos += snprintf(&retBuffer[pos],MAXSTRING-pos,sRet);
  pos = addChecksum(retBuffer,MAXSTRING);

  //log_i("%s",buffer);
  while(Serial2.available()){
    Serial2.read();
  }
  //Serial.print("tx=");
  //Serial.print(buffer);
  Serial2.write(&buffer[0]);  
  //Serial.print("sRet=");
  //Serial.print(retBuffer);  
  uint32_t tstart = millis();
  //wait for response
  pos = 0;
  buffer[pos] = 0;
  char c;
  while((millis() - tstart) < tTimeout){
    while(Serial2.available()){
      c = Serial2.read();
      //Serial.printf("%c",c);
      if (c == '$') pos = 0;
      buffer[pos] = c;
      pos++;
      if (pos >= MAXSTRING) pos = 0;
      buffer[pos] = 0; //zero-termination !!      
      if (c == '\n'){
        //Serial.print("rx=");
        //Serial.print(buffer);
        if (strcmp(retBuffer,buffer) == 0){
          //log_i("successful transmitted");
          //Serial.println(buffer);
          return true;
        }
      }
    }
    delay(1);
  }
  //Serial.println(buffer);
  return false;
}

bool setupQuetelGps(void){
  log_i("************ config GPS *************");
  Serial2.begin(9600, SERIAL_8N1,GPS_RX,GPS_TX);
  delay(500);
  if (!sendCmd2NMEA("$PMTK353,1,0,1,0,0","$PMTK001,353,3,1,0,1,0,0,13")){ //enable GPS & Galileo
    Serial2.end();
    log_e("error configure GPS");
    return false;
  }
  if (!sendCmd2NMEA("$PQTXT,W,0,0","$PQTXT,W,OK")){ //set GPTXT to off
    Serial2.end();
    log_e("error configure GPS");
    return false;
  }
  if (!sendCmd2NMEA("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0","$PMTK001,314,3")){ //set nmea-output to GPRMC and GPGGA
    Serial2.end();
    log_e("error configure GPS");
    return false;
  }
  log_i("GPS-successfully configured");
  return true;  
}

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
  digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  ppsTriggered = true;
  tPPS = millis();
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
    case ARDUINO_EVENT_WIFI_READY:
      log_i("wifi ready");
      break;
    case ARDUINO_EVENT_WIFI_STA_START:
      log_i("station start");
      break;
    case ARDUINO_EVENT_WIFI_STA_STOP:
      log_i("station stop");
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      log_i("station connected to AP");
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      log_i("station lost connection to AP");
      break; 
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      status.myIP = WiFi.localIP().toString();
      log_i("station got IP from connected AP IP:%s",status.myIP.c_str());
      break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:          
      log_i("STA LOST IP");
      break;
    case ARDUINO_EVENT_WIFI_AP_START:
      log_i("AP started. IP: [%s]", WiFi.softAPIP().toString().c_str());
      break;
    case ARDUINO_EVENT_WIFI_AP_STOP:
      log_i("soft-AP stop");
      break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      log_i("a station connected to ESP32 soft-AP");
      break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      log_i("a station disconnected from ESP32 soft-AP");
      break;
    case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
      log_i("soft-AP assign an IP to a connected station");
      break;
    default:
      log_e("Unhandled WiFi Event: %d", event );
      break;
  }
  /*
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
  */
}

void setupSoftAp(){
  log_i("Set soft-AP settings");
  WiFi.softAP(host_name.c_str(), setting.wifi.appw.c_str());
  WiFi.softAPConfig(local_IP, gateway, subnet);
}

void setupWifi(){
  while(host_name.length() == 0){
    delay(100); //wait until we have the devid
  }
  status.wifiStat = 0;
  //setting.wifi.ssid = "WLAN_EICHLER";
  //setting.wifi.password = "magest172";
  WiFi.disconnect(true,true);
  WiFi.mode(WIFI_MODE_NULL);  
  WiFi.persistent(false);
  WiFi.onEvent(WiFiEvent);
  WiFi.config(IPADDR_ANY, IPADDR_ANY, IPADDR_ANY,IPADDR_ANY,IPADDR_ANY); // call is only a workaround for bug in WiFi class
  WiFi.setHostname(host_name.c_str());  
  if (setting.wifi.connect != WIFI_CONNECT_NONE){
    //esp_wifi_set_auto_connect(true);
    log_i("Try to connect to WiFi ...");
    WiFi.status();
    WiFi.mode(WIFI_MODE_STA);
    //setupSoftAp();
    if ((WiFi.SSID() != setting.wifi.ssid || WiFi.psk() != setting.wifi.password)){
      // ... Try to connect to WiFi station.
      WiFi.begin(setting.wifi.ssid.c_str(), setting.wifi.password.c_str());
    } else {
      // ... Begin with sdk config.
      WiFi.begin();
    }
    //WiFi.setTxPower(WIFI_POWER_19_5dBm); //maximum wifi-power
    uint32_t tStart = millis();    
    while(WiFi.status() != WL_CONNECTED){
      if (timeOver(millis(),tStart,30000)){ //wait max. 30seconds
        break;
      }
      delay(100);
    }
    if (WiFi.status() == WL_CONNECTED){
      log_i("wifi connected");
    }else{
      log_i("wifi not connected --> switch on AP");
      WiFi.disconnect();
      WiFi.mode(WIFI_MODE_AP); //start AP
      setupSoftAp();
    }  
  }else{
    WiFi.mode(WIFI_MODE_AP);
    setupSoftAp();
  }  
  log_i("hostname=%s",host_name.c_str());  
  log_i("my APIP=%s",local_IP.toString().c_str());
  if((WiFi.getMode() & WIFI_MODE_STA)){
    WiFi.setSleep(false); //disable power-save-mode !! will increase ping-time
    WiFi.setTxPower(WIFI_POWER_19_5dBm); //maximum wifi-power
  }

  status.wifiStat = 1;
  Web_setup();
}

void scanNetworks(){
  // Set WiFi to station mode and disconnect from an AP if it was previously connected.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  while(1){
    Serial.println("Scan start");
 
    // WiFi.scanNetworks will return the number of networks found.
    int n = WiFi.scanNetworks();
    Serial.println("Scan done");
    if (n == 0) {
        Serial.println("no networks found");
    } else {
        Serial.print(n);
        Serial.println(" networks found");
        Serial.println("Nr | SSID                             | RSSI | CH | Encryption");
        for (int i = 0; i < n; ++i) {
            // Print SSID and RSSI for each network found
            Serial.printf("%2d",i + 1);
            Serial.print(" | ");
            Serial.printf("%-32.32s", WiFi.SSID(i).c_str());
            Serial.print(" | ");
            Serial.printf("%4d", WiFi.RSSI(i));
            Serial.print(" | ");
            Serial.printf("%2d", WiFi.channel(i));
            Serial.print(" | ");
            switch (WiFi.encryptionType(i))
            {
            case WIFI_AUTH_OPEN:
                Serial.print("open");
                break;
            case WIFI_AUTH_WEP:
                Serial.print("WEP");
                break;
            case WIFI_AUTH_WPA_PSK:
                Serial.print("WPA");
                break;
            case WIFI_AUTH_WPA2_PSK:
                Serial.print("WPA2");
                break;
            case WIFI_AUTH_WPA_WPA2_PSK:
                Serial.print("WPA+WPA2");
                break;
            case WIFI_AUTH_WPA2_ENTERPRISE:
                Serial.print("WPA2-EAP");
                break;
            case WIFI_AUTH_WPA3_PSK:
                Serial.print("WPA3");
                break;
            case WIFI_AUTH_WPA2_WPA3_PSK:
                Serial.print("WPA2+WPA3");
                break;
            case WIFI_AUTH_WAPI_PSK:
                Serial.print("WAPI");
                break;
            default:
                Serial.print("unknown");
            }
            Serial.println();
            delay(10);
        }
    }
    Serial.println("");
 
    // Delete the scan result to free memory for code below.
    WiFi.scanDelete();
    delay(5000);    
  }

}

void setup() {
  // put your setup code here, to run once:
  // Set pin mode
  for (uint8_t i = 0; i < NUMBUTTONS; i++) {
    //init buttons
    sButton[i].PinButton = -1;
  }  
  #ifdef Heltec_Wifi_V3
    pinExtCtrl = 36;
    pinAdcCtrl = 37;  
    pinMode(pinExtCtrl,OUTPUT);
    digitalWrite(pinExtCtrl,LOW); //switch external power on
    pinMode(pinAdcCtrl,OUTPUT);
    digitalWrite(pinAdcCtrl,LOW);//switch ADC-Control to measure Batt-voltage
    delay(500); //wait power is stable
    analogReadResolution(10); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
    PinADCVoltage = 1; //GPIO1 batt voltage
    pinMode(PinADCVoltage, INPUT); //set pin ADC-Voltage as input
    adcVoltageMultiplier = 5.4f;
    sButton[0].PinButton = 0;
  #endif

  for (uint8_t i = 0; i < NUMBUTTONS; i++) {
    // initialize built-in LED as an output
    if (sButton[i].PinButton >= 0){
      //set pin for button as input
      pinMode(sButton[i].PinButton, INPUT_PULLUP);
      // initialize the corresponding AceButton
      buttons[i].init(sButton[i].PinButton, HIGH, i);
    }
  }
  // Configure the ButtonConfig with the event handler, and enable all higher
  // level events.
  ace_button::ButtonConfig* buttonConfig = ace_button::ButtonConfig::getSystemButtonConfig();
  buttonConfig->setEventHandler(handleEvent);
  buttonConfig->setFeature(ace_button::ButtonConfig::kFeatureClick);
  buttonConfig->setFeature(ace_button::ButtonConfig::kFeatureDoubleClick);
  buttonConfig->setFeature(ace_button::ButtonConfig::kFeatureLongPress);
  buttonConfig->setFeature(ace_button::ButtonConfig::kFeatureRepeatPress);
  buttonConfig->setDebounceDelay(20); //set debounce-delay to 20ms
  buttonConfig->setLongPressDelay(1000); //set long-press-delay to 500ms
  buttonConfig->setClickDelay(500); //set click-delay to 200ms

  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED,HIGH);
  delay(500);
  digitalWrite(PIN_LED,LOW);
  ppsTriggered = false;
  pinMode(PPS_PIN,INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), IntPPS, FALLING);

  Serial.begin(115200);

  //setupDisplay();

  log_i("SDK-Version=%s",ESP.getSdkVersion());
  log_i("CPU-Speed=%d",ESP.getCpuFreqMHz());
  log_i("Total heap: %d", ESP.getHeapSize());
  log_i("Free heap: %d", ESP.getFreeHeap());
  log_i("Free PSRAM: %d", ESP.getFreePsram());
  log_i("compiled at %s",compile_date);
  log_i("current free heap: %d, minimum ever free heap: %d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
  log_i("Wifi-Mac-Adr=%s",WiFi.macAddress().c_str());

  //scanNetworks();

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
  log_i("spiffs total bytes:%d",SPIFFS.totalBytes());

  //listSpiffsFiles();


  //setting.bSwitchWifiOff3Min = false;
  //setting.NMEAOUTPUT = eNMEAOUTPUT::BLUETOOTH_OUT;
  setting.NMEAOUTPUT = eNMEAOUTPUT::SERIAL_OUT;
  printSettings();
  
  xTaskCreatePinnedToCore(taskBackGround, "taskBackGround", 6500, NULL, 5, &xHandleBackground, ARDUINO_RUNNING_CORE1); //background task
  xTaskCreatePinnedToCore(taskBluetooth, "taskBluetooth", 4096, NULL, 7, &xHandleBluetooth, ARDUINO_RUNNING_CORE1);
  xTaskCreatePinnedToCore(taskOled, "taskOled", 6500, NULL, 8, &xHandleOled, ARDUINO_RUNNING_CORE1); //background Oled

  #ifdef FANET_BOOT
    fanet.begin(1,FANET_RX,FANET_TX,FANET_RST,FANET_BOOT); //Hardwareserial, rxPin, txPin, Reset-Pin  
  #else
    fanet.begin(1,FANET_RX,FANET_TX,FANET_RST,-1); //Hardwareserial, rxPin, txPin, Reset-Pin  
  #endif
  fanet.setAircraftType(setting.AircraftType); //we have to set Aircraft-type before init module
  uint32_t tStart = millis();
  while (!fanet.initOk()){
    fanet.run(); //call run to get DevId
    if ((millis() - tStart) >= 3000){

      break; //timeout --> go further to give firmware update a chance
    }
  }
  setting.myDevId = fanet.getMyDevId();
  setting.FlarmExp = fanet.getFlarmExp();
  setting.fanetVersion = fanet.sVersion;
  if(setting.myDevId.length() == 0){
    setting.myDevId = "XXXXXX";
  }
  host_name = APPNAME "-" + setting.myDevId; //String((ESP32_getChipId() & 0xFFFFFF), HEX);

  //setupWifi();

  delay(1000);
  //fanet.updateModule("/fanet.xlb");
  //esp_restart();

  delay(500);
  #ifdef Heltec_Wifi_V3
  //delay(1000);
  for (int i = 0;i < 3; i++){
    if (setupQuetelGps()){
      break;
    }
  }
  //setupQuetelGps();
  //delay(1000);
  #endif

  nmeaout.begin(setting.NMEAOUTPUT,setting.UDPServerIP,setting.UDPSendPort,host_name);  
  fanet.setNMEAOUT(&nmeaout); //Hardwareserial, rxPin, txPin, Reset-Pin)
  fanet.setPilotname(setting.PilotName); //set name of Pilot  
  blueFly.begin(2,setting.baudRate,GPS_RX,GPS_TX,&nmeaout); //Hardwareserial, rxPin, txPin
  flarm.begin(&nmeaout);


}

void taskOled(void *pvParameters){
  log_i("start Oled-task");
  Oled display;
  pI2cOne->begin(PIN_DISPLAY_SDA, PIN_DISPLAY_SCL);
  display.begin(pI2cOne,PIN_DISPLAY_RST);
  while(1){
    display.run();
    delay(10);
    if ((WebUpdateRunning) || (bPowerOff)) break;
  }
  if (WebUpdateRunning) display.webUpdate();
  if (bPowerOff) display.end();
  log_i("stop task");
  vTaskDelete(xHandleOled);
}


void BlinkLed(uint32_t tAct){
  static uint32_t tLed = millis();
  if ((tAct - tLed) >= 500){
      tLed = tAct;
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  }
}

bool sendFanetStatus(){
  static long oldAlt = 0;
  static float oldTurnrate = 0.0;  
  //msg has to sent max 100ms after pps !!
  //if (!blueFly.nmea.isValid()) return;
  /*
  if ((millis() - tPPS) < 400){
    return false;
  }
  */
  if (!blueFly.nmea.isNewMsgValid()) return false;
  //Serial.println("OK");
  status.gps.NumSat = blueFly.nmea.getNumSatellites(); 
  status.gps.Fix = 1;
  status.gps.Lat = blueFly.nmea.getLatitude() / 1000000.;
  status.gps.Lon = blueFly.nmea.getLongitude() / 1000000.;
	long alt = 0;
  if (!blueFly.nmea.getAltitude(alt)){
    alt = long(blueFly.getAlt() * 1000);
    //Serial.println("alt not valid");
  } 
  if (oldAlt == 0){
    oldAlt = alt;
  }   
  status.gps.alt = alt/1000.; 
  long geoIdAlt = 0;
  blueFly.nmea.getGeoIdAltitude(geoIdAlt);
  status.gps.geoidAlt = geoIdAlt/1000.;  

  if (status.gps.NumSat < 4){
    status.gps.speed = 0;
    status.gps.course = 0;
    return false;
  } 
  status.gps.speed = blueFly.nmea.getSpeed()*1.852/1000.; //speed in cm/s --> we need km/h
  status.gps.course = blueFly.nmea.getCourse()/1000.;  
  //Serial.println("new GPS Data avaiable");
  stateData tFanetData;  
  tFanetData.lat = status.gps.Lat;
  tFanetData.lon = status.gps.Lon;



  tFanetData.altitude = status.gps.alt;
  tFanetData.geoIdAltitude = status.gps.geoidAlt;
  tFanetData.speed = status.gps.speed;
  tFanetData.climb = (alt - oldAlt)/1000.;
  oldAlt = alt;
  tFanetData.heading = status.gps.course;
	tFanetData.year = blueFly.nmea.getYear() - 1900;
	tFanetData.month = blueFly.nmea.getMonth();
	tFanetData.day = blueFly.nmea.getDay();
	tFanetData.hour = blueFly.nmea.getHour();
	tFanetData.minute = blueFly.nmea.getMinute();
	tFanetData.second = blueFly.nmea.getSecond();
  tFanetData.turnrate = tFanetData.heading - oldTurnrate;
  oldTurnrate = tFanetData.heading;
  fanet.writeStateData2FANET(&tFanetData);
  //Serial.println("sendFanetMsg");
  return true;
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

/*
bool timeOver(uint32_t tAct,uint32_t timestamp,uint32_t tTime){
    uint32_t tDiff = tAct - timestamp;
    if ((tDiff) >= tTime){
        return true;
    }else{
        return false;
    }
}
*/

void loop() {
  trackingData tFanetData;  
  trackingData myFanetData;  
  FlarmtrackingData myFlarmData;
  FlarmtrackingData PilotFlarmData;
  uint32_t tAct = millis();
  static uint32_t tLoop = millis();
  static uint32_t tSerialRec = millis();
  //static uint32_t tPPS =  millis();

  status.tLoop = tAct - tLoop;
  tLoop = tAct;
  if (status.tMaxLoop < status.tLoop) status.tMaxLoop = status.tLoop;

  //BlinkLed(tAct);
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
    //Serial.println("**************** PPS triggered *******************");
    blueFly.nmea.clearNewMsgValid(); //clear flag msg is valid
  }
  if (timeOver(millis(),tPPS,10000)){
    //no pps for more then 10sec. --> clear GPS-state
    status.gps.Fix = 0;
    status.gps.speed = 0.0;
    status.gps.Lat = 0.0;
    status.gps.Lon = 0.0;
    status.gps.alt = 0.0;
    status.gps.geoidAlt = 0.0;
    status.gps.course = 0.0;
    status.gps.NumSat = 0;    
  }
  nmeaout.run();
  sendData2Client(nmeaout.getSendData());  

  //sendFanetStatus(); //~20ms after pps we have GPRMC and we send the message
  if (sendFanetStatus()){
    log_i("timediff sendFlarm=%d",millis()-tPPS); //~20ms
  }
  checkFlyingState(millis());
  readBatt(millis());
  for (uint8_t i = 0; i < NUMBUTTONS; i++) {
    if (sButton[i].PinButton >= 0){
      buttons[i].check();
    }
  }
  //check Button 0
  if (sButton[0].state == ace_button::AceButton::kEventClicked){
    //log_v("Short Press IRQ");
    //setting.screenNumber ++;
    //bShowBattPower = true; //show battery-state again
    //if (setting.screenNumber > MAXSCREENS) setting.screenNumber = 0;
    //write_screenNumber(); //save screennumber in File
  }else if (sButton[0].state == ace_button::AceButton::kEventLongPressed){
    //log_v("Long Press IRQ");
    status.bPowerOff = true;
  }else if (sButton[0].state == ace_button::AceButton::kEventDoubleClicked){
    //log_v("double clicked IRQ"); --> switch wifi off or on
    //log_i("double clicked wifi on/off");
    //userled.setState(gxUserLed::off); //switch to state off --> so state is working
    //if (status.bWifiOn){
    //  userled.setState(gxUserLed::showWifiDis);
    //  wifiCMD = 10; //switch off wifi
    //}else{
    //  userled.setState(gxUserLed::showWifiEn);
    //  wifiCMD = 11; //switch on wifi
    //}
  }
  sButton[0].state =  0;  
  if (status.bPowerOff){
    powerOff(); //power off, when battery is empty !!
  }  
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
    if (bPowerOff) break;
  }
  log_i("stop task");
  vTaskDelete(xHandleBackground);
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
     if (bPowerOff) break;
	 }   
  }
  log_i("stop task");
  vTaskDelete(xHandleBluetooth);
}
