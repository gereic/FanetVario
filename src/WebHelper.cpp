#include "WebHelper.h"
/*
 * Login page
 */
const char* loginIndex = 
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>ESP32 Login Page</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<td>Username:</td>"
        "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='admin' && form.pwd.value=='admin')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";
 
/*
 * Server Index Page
 */
 
const char* serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";

void status() {
  String s;
  s = "<style>.bu {background-color: #d7d7d7;border: none;color: black;padding: 8px 32px;\
  text-decoration: none;font-size: 18px;margin: 4px 2px;}\
  body{font: normal 12px Verdana, Arial, sans-serif;background-color:#e6e7e8}</style>\
  <table border=0 cellpadding=6><tr><td><h1 align=center>FANET-VARIO status</h1>";
  //s += "<tr><td>received NMEA-lines</td><td>" + String(blueFly.nmea.getMessageID()) + "</td></tr>";
  s += "<tr><td>device-ID</td><td>" + setting.myDevId + "</td></tr>";
  s += "<tr><td>Flarm-exp</td><td>" + setting.FlarmExp + "</td></tr>";
  s += "<tr><td>count satellites</td><td>" + String(blueFly.nmea.getNumSatellites()) + "</td></tr>";
  s += "<tr><td>sat-system</td><td>";
  // Navigation system, N=GNSS, P=GPS, L=GLONASS, A=Galileo, '\0'=none
  switch (blueFly.nmea.getNavSystem())
  {
  case 'N':
    s += "GNSS";
    break;
  case 'P':
    s += "GPS";
    break;
  case 'L':
    s += "GLONASS";
    break;
  case 'A':
    s += "Galileo";
    break;
  }
  s += "</td></tr>";
  s += "<tr><td>GPS-status</td><td>";
  if (blueFly.nmea.isValid()){
    s += "FIX OK";
  }else{
    s += "no FIX";
  }
  s += "</td></tr>";
  s += "<tr><td align=left><input type=button onClick=\"location.href='/config'\" value='Settings'></td></tr>";
  //s += "<tr><td colspan=3><table><tr><td><form method=post action=\"config\"><input type=hidden name=work1 value=please1><input type=submit class=\"bu\" value='Configure'></form></td>";
  s += "</table>";

 server.send ( 200, "text/html", s.c_str() );
}

void configure() {
  String s;
  s = "<style>.bu {background-color: #d7d7d7;border: none;color: black;padding: 8px 32px;\
  text-decoration: none;font-size: 18px;margin: 4px 2px;}\
  body{font: normal 12px Verdana, Arial, sans-serif;background-color:#e6e7e8}</style>\
  <table border=0 cellpadding=6><tr><td><h1 align=center>Configure FANET-VARIO</h1>\
  <form method=post action=\"savesettings\">";
  s += "<tr><td>WIFI SSID</td><td><input class=bu type=text name=ssid value=\"" + setting.ssid + "\"></td></tr>";
  s += "<tr><td>WIFI Password</td><td><input class=bu type=text name=password value=\"" + setting.password + "\"></td></tr>";
  s += "<tr><td>Pilot Name</td><td><input class=bu type=text name=PilotName value=\"" + setting.PilotName + "\"></td></tr>";
  s += "<tr><td>Wifi Off(wifi off after 3 mins to save power<br>Leave no if you dont know what this means)</td>\
        <td><select class=bu name=SwitchWifiOff>\
        <option value=1 ";
  if (setting.bSwitchWifiOff3Min) s += "selected";
  s += ">yes</option><option value=0 ";
  if (!setting.bSwitchWifiOff3Min) s += "selected";
  s += ">no</option></select></td></tr>";

  s += "<tr><td>Aircraft Type</td><td><select class=bu name=AircraftType>";
  for (int i = 1; i <= 7; i++){
    s += "<option value=" + String(i);
    if ((int)setting.AircraftType == i) s += " selected";
    s += ">" + fanet.getAircraftType((eFanetAircraftType)i);
  }
  s += "</select>";
  s += "<tr><td>UDP Server-IP</td><td><input class=bu type=text name=udpserver value=\"" + setting.UDPServerIP + "\"></td></tr>";
  s += "<tr><td>UDP Port</td><td><input class=bu type=text name=udpport value=\"" + String(setting.UDPSendPort) + "\"></td></tr>";
  s += "<tr><td>NMEA OUTPUT</td><td><select class=bu name=nmeaoutput>";
  s += "<option value=0 ";
  if (setting.NMEAOUTPUT == eNMEAOUTPUT::SERIAL_OUT) s += "selected";
  s += ">SERIAL</option>";
  s += "<option value=1 ";
  if (setting.NMEAOUTPUT == eNMEAOUTPUT::UDP_OUT) s += "selected";
  s += ">UDP</option>";  
  /*
  s += "<option value=2 ";
  if (setting.NMEAOUTPUT == eNMEAOUTPUT::BLUETOOTH_OUT) s += "selected";
  s += ">BLUETOOTH</option>";
  s += "<option value=3 ";
  if (setting.NMEAOUTPUT == eNMEAOUTPUT::BLE_OUT) s += "selected";
  s += ">BLE</option>";
  */
  s += "</select></td></tr>";


  s += "<tr><td><input type=submit class=bu value=Submit></form></td></tr>";
  //s += "<tr><td><a href=/>Return to FANET-VARIO Home</a></td><tr></a>"
  s += "<tr><td align=left><input type=button onClick=\"location.href='/'\" value='HOME'></td></tr>";
  s += "</table>";

 server.send ( 200, "text/html", s.c_str() );
}

void savesettings(){
  for ( uint8_t i = 0; i < server.args(); i++ ){
    if ( server.argName(i) == "ssid"){
      setting.ssid = server.arg(i);
    }
    if ( server.argName(i) == "password"){
      setting.password = server.arg(i);
    }
    if ( server.argName(i) == "PilotName"){
      setting.PilotName = server.arg(i);
    }
    if ( server.argName(i) == "AircraftType"){
      setting.AircraftType = (eFanetAircraftType)atoi(server.arg(i).c_str());
    }
    if ( server.argName(i) == "SwitchWifiOff"){
      setting.bSwitchWifiOff3Min = (bool)atoi(server.arg (i).c_str());
    }
    if ( server.argName(i) == "udpserver"){
      setting.UDPServerIP = server.arg(i);
    }
    if ( server.argName(i) == "udpport"){
      setting.UDPSendPort = atoi(server.arg(i).c_str());
    }
    if ( server.argName(i) == "nmeaoutput")
    {
      setting.NMEAOUTPUT = (eNMEAOUTPUT)atoi(server.arg(i).c_str());
    }
  }
  server.send ( 200, "text/html", "<html><head>\
	<style>.bu {background-color: #d7d7d7;border: none;color: black;padding: 8px 32px;\
	text-decoration: none;font-size: 18px;margin: 4px 2px;}\
	body{font: normal 12px Verdana, Arial, sans-serif;background-color:#e6e7e8}</style>\
	<h1>FANET-VARIO Rebooting<br><br></h1>Please reconnect to the Access Point again<br><br><a href=/>Reconnect to FANET_VARIO home page</a></h2>" );
  delay(2000);
  write_configFile(); //write settings in config-file

}

void Web_setup()
{
    /*
    server.on("/", HTTP_GET, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", loginIndex);
    });
    */
   Serial.println("WEBSETUP");
   server.on ( "/", HTTP_ANY, status );
    server.on ( "/config", HTTP_ANY, configure );
    server.on ( "/status", HTTP_ANY, status );
    server.on("/serverIndex", HTTP_GET, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", serverIndex);
    });
    //handling save settings
    server.on ( "/savesettings", HTTP_POST, savesettings );
    //handling uploading firmware file 
    server.on("/update", HTTP_POST, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
      ESP.restart();
    }, []() {
      HTTPUpload& upload = server.upload();
      if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("Update: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        // flashing firmware to ESP
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) { //true to set the size to the current progress
          Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        } else {
          Update.printError(Serial);
        }
      }
    });
    server.begin();

}



void Web_loop()
{
  //dnsServer.processNextRequest();
  yield();
  //DEBUG_SERIAL_UART_MAX("[%ld] - After DNS Process\r\n", millis());
  server.handleClient();
  yield();
  // DEBUG_SERIAL_UART_MAX("[%ld] - Ending Webloop\r\n", millis());
}
