#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "main.h"
#include "Fanet.h"
#include "BlueFly.h"
#include "fileOps.h"
#include <Update.h>


extern struct SettingsData setting;
extern struct StatusData status;
extern bool WebUpdateRunning;
extern Fanet fanet;
extern BlueFly blueFly;
extern String compile_date;

void Web_setup(void);
void Web_stop(void);
void Web_loop(void);
