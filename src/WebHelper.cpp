#include <WebHelper.h>
#include "./web/website.h"

// Globals
AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(1337);
int led_state = 0;
char msg_buf[500];
#define MAXCLIENTS 10
uint8_t clientPages[MAXCLIENTS];

void sendPageHeader(uint8_t client_num);

void sendPageHeader(uint8_t client_num){
  StaticJsonDocument<400> doc;
  doc.clear();
  doc["myDevId"] = setting.myDevId;
  doc["appname"] = String(APPNAME "-" VERSION);
  doc["buildDate"] = "build:" + compile_date + " sdk:" + String(ESP.getSdkVersion());
  doc["pilot"] = "pilot: " + setting.PilotName + " [" + setting.myDevId + "]";
  serializeJson(doc, msg_buf);
  webSocket.sendTXT(client_num, msg_buf);
}

/***********************************************************
 * Functions
 */

// Callback: receiving any WebSocket message
void onWebSocketEvent(uint8_t client_num,
                      WStype_t type,
                      uint8_t * payload,
                      size_t length) {
  StaticJsonDocument<500> doc;                      //Memory pool
  JsonObject root = doc.to<JsonObject>();
  DeserializationError error;
  uint8_t value = 0;
  // Figure out the type of WebSocket event
  switch(type) {

    // Client has disconnected
    case WStype_DISCONNECTED:
      if (client_num < MAXCLIENTS) clientPages[client_num] = 0;
      log_i("[%u] Disconnected!", client_num);
      break;

    // New client has connected
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(client_num);
        log_i("[%u] Connection from ", client_num);
        //log_i("%s",ip.toString());        
      }
      break;

    // Handle text messages from client
    case WStype_TEXT:

      // Print out raw message
      
      log_i("[%u] Received text: %s", client_num, payload);      
      error = deserializeJson(doc, payload);
      if (error) {   //Check for errors in parsing
        log_i("deserializeJson() failed: %s",error.c_str());
        return;
    
      }
      if (root.containsKey("page")){
        value = doc["page"];                    //Get value of sensor measurement
        if (client_num < MAXCLIENTS) clientPages[client_num] = value;
        log_i("page=%d",value);
        sendPageHeader(client_num);
        doc.clear();
        if (clientPages[client_num] == 1){ //info
          doc["myDevId"] = setting.myDevId;
          doc["compiledate"] = String(compile_date);
          doc["fVersion"] = setting.fanetVersion;
          doc["fExp"] = setting.FlarmExp;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
        }else if (clientPages[client_num] == 10){ //full settings
          doc.clear();
          doc["PilotName"] = setting.PilotName;
          doc["type"] = (uint8_t)setting.AircraftType;
          doc["appw"] = setting.wifi.appw;
          doc["wificonnect"] = (uint8_t)setting.wifi.connect;
          doc["ssid"] = setting.wifi.ssid;
          doc["password"] = setting.wifi.password;
          doc["wifioff"] = setting.wifi.tWifiStop;
          doc["outble"] = setting.OutputBLE;
          doc["baudrate"] = setting.baudRate;
          serializeJson(doc, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
        }
      }else if (root.containsKey("save")){
        //save settings-page
        value = doc["save"];
        if (value == 1){
          //general settings-page          
          SettingsData newSetting = setting;
          if (root.containsKey("PilotName")) newSetting.PilotName = doc["PilotName"].as<String>();
          if (root.containsKey("type")) newSetting.AircraftType = (eFanetAircraftType)doc["type"].as<uint8_t>();
          if (root.containsKey("appw")) newSetting.wifi.appw = doc["appw"].as<String>();          
          if (root.containsKey("wificonnect")) newSetting.wifi.connect = (bool)doc["wificonnect"].as<uint8_t>();
          if (root.containsKey("ssid")) newSetting.wifi.ssid = doc["ssid"].as<String>();
          if (root.containsKey("password")) newSetting.wifi.password = doc["password"].as<String>();
          if (root.containsKey("wifioff")) newSetting.wifi.tWifiStop = doc["wifioff"].as<uint32_t>();
          if (root.containsKey("outble")) newSetting.OutputBLE = doc["outble"].as<uint8_t>();
          if (root.containsKey("baudrate")) newSetting.baudRate = doc["baudrate"].as<uint32_t>();
          log_i("write config-to file --> rebooting");
          //delay(10);
          write_configFile(&newSetting);

        }
      }      
      break;

    // For everything else: do nothing
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      break;
  }
}

String processor(const String& var){
  String sRet = "";
  //log_i("%s",var.c_str());
  if(var == "SOCKETIP"){
    return status.myIP;
  }else if (var == "APPNAME"){
    return APPNAME;
  }else if (var == "PILOT"){
    return setting.PilotName;
  }else if (var == "VERSION"){
    return VERSION;
  }else if (var == "BUILD"){
    return String(compile_date);
  }
    
  return "";
}

// Callback: send 404 if requested file does not exist
void onPageNotFound(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  log_e("[%s] HTTP GET request of %s",remote_ip.toString().c_str(),request->url().c_str());
  request->send(404, "text/plain", "Not found");
}

static int restartNow = false;

static void handle_update_progress_cb(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  static uint8_t fileType = 0;
  static File file;
  uint32_t free_space = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
  if (!index){
    //Serial.print("Total bytes:    "); Serial.println(SPIFFS.totalBytes());
    //Serial.print("Used bytes:     "); Serial.println(SPIFFS.usedBytes());
    //Serial.println(filename);
    //Serial.println("Update");
    //log_i("stopping standard-task");
    //vTaskDelete(xHandleStandard); //delete standard-task
    WebUpdateRunning = true;
    delay(500); //wait 1 second until tasks are stopped
    //Update.runAsync(true);
    if (filename == "spiffs.bin"){
      fileType = 0;
      if (!Update.begin(0x30000,U_SPIFFS)) {
        Update.printError(Serial);
      }
    }else if (filename == "firmware.bin"){
      fileType = 0;
      if (!Update.begin(free_space,U_FLASH)) {
        Update.printError(Serial);
      }
    }else if (filename == "fanet.xlb"){
      fileType = 1;
      log_i("upload file %s",filename.c_str());
      file = SPIFFS.open("/" + filename,FILE_WRITE);
    }else{
      fileType = 2;
    }
  }
  if (fileType == 0){
    //it is an firmware-update or spiffs update
    if (Update.write(data, len) != len) {
      Update.printError(Serial);
    }

    if (final) {
      if (!Update.end(true)){
        Update.printError(Serial);
      } else {
        restartNow = true;//Set flag so main loop can issue restart call
        Serial.println("Update complete");      
      }
    }
  }else if (fileType == 1){
    if (file){
      //log_i("write %d bytes",len);
      file.write(data,len);
      if (final) {      
        file.close();
        log_i("file upload complete");
        if (filename == "fanet.xlb"){
          log_i("start update fanet-module");
          status.UpdateFanetModule = 1; //now we can update the fanet-module
        }
      }
    }else{
      log_i("file upload error");
    }
  }else{
    if (final){
      log_i("error unknown filetype");
    }
  }
}

void loadFromFlash(AsyncWebServerRequest *request,const uint8_t * content, size_t len,String dataType = "text/html") {
  AsyncWebServerResponse *response = request->beginResponse_P(200, dataType,content, len);
  response->addHeader("Content-Encoding", "gzip");
  request->send(response);   
}

void Web_setup(void){
  for (int i = 0;i < MAXCLIENTS;i++) clientPages[i] = 0;
  // On HTTP request for root, provide index.html file
  server.on("/fwupdate", HTTP_GET, [](AsyncWebServerRequest *request){
    //request->send(SPIFFS, request->url() + ".html", "text/html",false,processor);
    loadFromFlash(request,fwupdate_html_gz,fwupdate_html_gz_len);
  });
  // handler for the /update form POST (once file upload finishes)
  server.on("/fwupdate", HTTP_POST, [](AsyncWebServerRequest *request){
      request->send(200);
    }, handle_update_progress_cb);

  server.on("/fullsettings.html", HTTP_GET, [](AsyncWebServerRequest *request){
    //request->send(SPIFFS, request->url(), "text/html",false,processor);
    loadFromFlash(request,fullsettings_html_gz,fullsettings_html_gz_len);
  });
  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *request){
    //request->send(SPIFFS, request->url(), "text/html",false,processor);
    loadFromFlash(request,index_html_gz,index_html_gz_len);
  });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    //request->send(SPIFFS, "/index.html", "text/html",false,processor);
    loadFromFlash(request,index_html_gz,index_html_gz_len);
  });
  server.on("/info.html", HTTP_GET, [](AsyncWebServerRequest *request){
    //request->send(SPIFFS, request->url(), "text/html",false,processor);
    loadFromFlash(request,info_html_gz,info_html_gz_len);
  });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    //request->send(SPIFFS, request->url(), "text/css");
    loadFromFlash(request,style_css_gz,style_css_gz_len,"text/css");
  });
  server.on("/scripts.js", HTTP_GET, [](AsyncWebServerRequest *request){
    loadFromFlash(request,scripts_js_gz,scripts_js_gz_len,"text/javascript");
  });  

  // Handle requests for pages that do not exist
  server.onNotFound(onPageNotFound);

  // Start web server
  server.begin();

  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

void Web_stop(void){
  webSocket.close();
  server.end();
}

void Web_loop(void){
  static uint32_t tLife = millis();
  static uint16_t counter = 0;
  static uint32_t tRestart = millis();
  uint32_t tAct = millis();
  long alt = 0;
  // Look for and handle WebSocket data
  webSocket.loop();
  if ((tAct - tLife) >= 500){
    tLife = tAct;
    StaticJsonDocument<300> doc;                      //Memory pool
    doc.clear();
    doc["counter"] = counter;
    doc["gpsFix"] = String(status.gps.Fix);
    doc["gpsNumSat"] = String(status.gps.NumSat);
    doc["gpslat"] = String(status.gps.Lat,6);
    doc["gpslon"] = String(status.gps.Lon,6);
    doc["gpsalt"] = String(status.gps.alt,2);
    doc["tLoop"] = status.tLoop;
    doc["tMaxLoop"] = status.tMaxLoop;
    doc["freeHeap"] = xPortGetFreeHeapSize();
    doc["fHeapMin"] = xPortGetMinimumEverFreeHeapSize();
    doc["battV"] = String(status.battery.voltage/1000.0);
    doc["battPerc"] = status.battery.percent;

    /*
    doc["vBatt"] = String((float)status.vBatt/1000.,2);
    doc["gpsFix"] = status.GPS_Fix;
    doc["gpsNumSat"] = status.GPS_NumSat;
    doc["gpslat"] = String(status.GPS_Lat,6);
    doc["gpslon"] = String(status.GPS_Lon,6);
    doc["gpsAlt"] = String(status.GPS_alt,1);
    doc["gpsSpeed"] = String(status.GPS_speed,2);
    doc["climbrate"] = String(status.ClimbRate,1);
    doc["fanetTx"] = status.fanetTx;
    doc["fanetRx"] = status.fanetRx;
    doc["tLoop"] = status.tLoop;
    doc["tMaxLoop"] = status.tMaxLoop;
    */
    serializeJson(doc, msg_buf);
    for (int i = 0;i <MAXCLIENTS;i++){
      if (clientPages[i] == 1){
        log_d("Sending to [%u]: %s", i, msg_buf);
        webSocket.sendTXT(i, msg_buf);
      }
    }
    counter++;
  }
  if (restartNow){
    if ((tAct - tRestart) >= 1000){
      ESP.restart();
    }    
  }else{
    tRestart = tAct;
  }
}