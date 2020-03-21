#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include "main.h"
#include "Fanet.h"
#include "BlueFly.h"
#include "fileOps.h"

extern WebServer server;
extern struct SettingsData setting;
extern Fanet fanet;
extern BlueFly blueFly;

void Web_setup(void);
void Web_loop(void);
