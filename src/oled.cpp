#include "oled.h"
#include "tools.h"
#include "icons.h"
#include "icons2.h"
#include "Fanet.h"

Oled::Oled(){
}

bool Oled::begin(TwoWire *pi2c,int8_t rst){
  pinRst = rst;
  oldScreenNumber = 0;
  if (display == NULL){
    #ifdef SH1106G
    display = new Adafruit_SH1106G(128, 64, pi2c);
    #else
    display = new Adafruit_SSD1306(128, 64, pi2c);
    #endif
  }
  PowerOn();
  display->setTextColor(WHITE);
  display->clearDisplay();
  display->display();
  display->drawXBitmap(0,2,G_Logo_bits,G_Logo_width,G_Logo_height,WHITE);
  display->display();
  delay(1000);
  display->drawXBitmap(30,2,X_Logo_bits,X_Logo_width,X_Logo_height,WHITE);
  display->display();
  delay(1000);
  display->drawXBitmap(69,2,AirCom_Logo_bits,AirCom_Logo_width,AirCom_Logo_height,WHITE);
  display->setTextSize(1);
  display->setCursor(85,55);
  display->print(VERSION);
  display->setTextColor(WHITE);
  display->setTextSize(1);
  display->setCursor(0,55);
  display->print(setting.myDevId.c_str());
  display->display();
  delay(1000);
  //if (setting.gs.SreenOption == eScreenOption::ALWAYS_OFF){
  //  PowerOff();
  //}else{
    display->clearDisplay();
    display->display();
  //}
  return true;
}

void Oled::PowerOn(void){
  if (bDisplayOn){
    return; //already on
  } 
  //reset OLED display via software
  if (pinRst >= 0){
    log_i("Heltec-board");
    pinMode(pinRst, OUTPUT);
    digitalWrite(pinRst, LOW);
    delay(100);
    digitalWrite(pinRst, HIGH);
    delay(100);
  }  
  //initialize OLED
  #ifdef SH1106G
  if(!display->begin(0x3C,false)) { // Address 0x3C for 128x64
  #else
  if(!display->begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) { // Address 0x3C for 128x64
  #endif
    log_e("display allocation failed");
    for(;;); // Don't proceed, loop forever
  }
  //set display to full contrast
  #ifndef SH1106G
  display->ssd1306_command(SSD1306_SETCONTRAST);
  display->ssd1306_command(0xFF);
  #endif
  //added the possibility to invert BW .. whould be nice to put it in the settings TODO
  display->invertDisplay(0);
  bDisplayOn = true;
}

void Oled::PowerOff(void){
  if (!bDisplayOn){ 
    return; //already off
  }
  display->clearDisplay();
  display->display();
  log_i("set display to off");
  #ifndef SH1106G
  display->ssd1306_command(0x8D); //into charger pump set mode
  display->ssd1306_command(0x10); //turn off charger pump
  display->ssd1306_command(0xAE); //set OLED sleep  
  #endif
  bDisplayOn = false;
}

void Oled::webUpdate(void){
  display->clearDisplay();
  display->setTextSize(2);
  display->setCursor(10,5);
  display->print("FW-UPDATE");
  display->setCursor(10,30);
  display->print("wait...");
  display->display();
}

void Oled::drawAircraftType(int16_t x, int16_t y, uint8_t AircraftType){
  switch (AircraftType)
  {
  case (uint8_t)eFanetAircraftType::PARA_GLIDER :
      display->drawXBitmap(x,y, Paraglider16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  case (uint8_t)eFanetAircraftType::HANG_GLIDER :
      display->drawXBitmap(x,y, Hangglider16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  case (uint8_t)eFanetAircraftType::BALLOON :
      display->drawXBitmap(x,y, Ballon16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  case (uint8_t)eFanetAircraftType::GLIDER :
      display->drawXBitmap(x,y, Sailplane16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  case (uint8_t)eFanetAircraftType::POWERED_AIRCRAFT :
      display->drawXBitmap(x,y, Airplane16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  case (uint8_t)eFanetAircraftType::HELICOPTER_ROTORCRAFT :
      display->drawXBitmap(x,y, Helicopter16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  case (uint8_t)eFanetAircraftType::UAV :
      display->drawXBitmap(x,y, UAV16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  
  default:
      display->drawXBitmap(x,y, UFO16_bits, 16, 16, WHITE);   //GxEPD_BLACK);
      break;
  }
}

void Oled::drawSatCount(int16_t x, int16_t y,uint8_t value){
    display->setTextSize(1);
    if (value == 0){
        display->drawXBitmap(x, y,gpsoff_bits,  16, 16, WHITE);
    }else{
        display->drawXBitmap(x, y,gpsOn_bits,  16, 16, WHITE);
        display->setCursor(x+18,y+4);
        display->print(String(value));
    }

}

void Oled::drawflying(int16_t x, int16_t y, bool flying){
    if (flying){
        display->drawXBitmap(x, y,flying_bits,  16, 16, WHITE);
    }else{
        display->drawXBitmap(x, y,not_flying_bits,  16, 16, WHITE);
    }
}

void Oled::drawBluetoothstat(int16_t x, int16_t y){
    if (status.bluetoothStat == 1){
     display->drawXBitmap(x,y,BT_bits,8,10,WHITE);
    }else if (status.bluetoothStat == 2){
      display->fillRect(x,y,8,10,WHITE);
      display->drawXBitmap(x,y,BT_bits,8,10,BLACK);
    }
}

void Oled::drawBatt(int16_t x, int16_t y,uint8_t value){
  static uint8_t DrawValue = 0;
  if (value == 255){
      DrawValue = (DrawValue + 1) %5; 
  }else{
      DrawValue = value / 25;
  }
  //log_i("%d",DrawValue);
  display->fillRect(x,y,17,8,WHITE);
  switch (DrawValue)
  {
  case 1:
      display->drawBitmap(x, y, bat1icons, 17, 8, BLACK);   //GxEPD_BLACK);    
      break;
  case 2:
      display->drawBitmap(x, y, bat2icons, 17, 8, BLACK);   //GxEPD_BLACK);    
      break;
  case 3:
      display->drawBitmap(x, y, bat3icons, 17, 8, BLACK);   //GxEPD_BLACK);    
      break;
  case 4:
      display->drawBitmap(x, y, bat4icons, 17, 8, BLACK);   //GxEPD_BLACK);    
      break;
  default:
      display->drawBitmap(x, y, bat0icons, 17, 8, BLACK);   //GxEPD_BLACK);    
      break;
  }
}

void Oled::printGPSData(uint32_t tAct){
  String s = "";
  display->clearDisplay();
  display->setTextSize(2);
  display->setCursor(0,0);
  drawAircraftType(0,0,uint8_t(setting.AircraftType));
  
  drawSatCount(18,0,(status.gps.NumSat > 9) ? 9 : status.gps.NumSat);
  drawflying(67,0,status.flying);
  //drawWifiStat(status.wifiSTA.state);
  drawBluetoothstat(101,0);
  drawBatt(111, 0,(status.battery.charging) ? 255 : status.battery.percent);

  display->setTextSize(3);

  //display->setCursor(0,20);
  //display->print(setStringSize(String(status.vario.ClimbRate,1) + "ms",7));

  display->setTextSize(2);

  display->setCursor(0,46);
  display->print(setStringSize(String(status.gps.alt,0) + "m",4));

  display->setCursor(65,46);
  display->print(setStringSize(String(status.gps.speed,0) + "kh",5));

  display->display();

}

String Oled::setStringSize(String s,uint8_t sLen){
  uint8_t actLen = (uint8_t)s.length();
  String sRet = "";
  for (uint8_t i = actLen;i < sLen;i++){
    sRet += " ";
  }
  sRet += s;
  return sRet;
}

void Oled::DrawAngleLine(int16_t x,int16_t y,int16_t length,float deg){
  int16_t xStart;
  int16_t yStart;
  int16_t xEnd;
  int16_t yEnd;
  float rads;
  rads = deg2rad(deg);
  xStart=(int)roundf(((sin(rads) * length/2) * 1) + x);
  yStart=(int)roundf(((cos(rads) * length/2) * -1) + y);
  xEnd=(int)roundf(((sin(rads) * length/2) * -1) + x);
  yEnd=(int)roundf(((cos(rads) * length/2) * 1) + y);  
  display->drawLine(xStart,yStart,xEnd,yEnd,WHITE);
  //log_i("x=%i,y=%i,deg=%0.1f,X-Start=%i,Y-Start=%i,X-End=%i,Y-End=%i",x,y,deg,xStart,yStart,xEnd,yEnd);
}

void Oled::drawSignal(int16_t x, int16_t y,uint8_t strength) {
  if ((strength <= 9) && (strength >= 3)){
      display->drawBitmap(x,y,signal_1, SIGNALWIDTH, SIGNALHEIGHT, BLACK,WHITE);
  }else if ((strength >= 10) && (strength <= 14)){
      display->drawBitmap(x,y,signal_2, SIGNALWIDTH, SIGNALHEIGHT, BLACK,WHITE);
  }else if ((strength >= 15) && (strength <= 19)){
      display->drawBitmap(x,y,signal_3, SIGNALWIDTH, SIGNALHEIGHT, BLACK,WHITE);
  }else if ((strength >= 19) && (strength <= 30)){
      display->drawBitmap(x,y,signal_4, SIGNALWIDTH, SIGNALHEIGHT, BLACK,WHITE);
  }else{
      display->drawBitmap(x,y,signal_0, SIGNALWIDTH, SIGNALHEIGHT, BLACK,WHITE);
  }
}

void Oled::run(void){
  uint32_t tAct = millis();
  static uint32_t tDisplay = millis();
  if (timeOver(tAct,tDisplay,DISPLAY_UPDATE_RATE)){
    tDisplay = tAct;
    printGPSData(tAct);          
  }
}

void Oled::end(void){
  display->setTextColor(WHITE);
  display->clearDisplay();
  display->drawXBitmap(0,2,G_Logo_bits,G_Logo_width,G_Logo_height,WHITE);
  display->drawXBitmap(30,2,X_Logo_bits,X_Logo_width,X_Logo_height,WHITE);
  display->drawXBitmap(69,2,AirCom_Logo_bits,AirCom_Logo_width,AirCom_Logo_height,WHITE);
  display->setTextSize(1);
  display->setCursor(85,55);
  display->print(VERSION);
  display->setCursor(0,55);
  display->print(setting.myDevId);
  display->display();
  delay(1000);
  display->clearDisplay();
  display->drawXBitmap(0,2,G_Logo_bits,G_Logo_width,G_Logo_height,WHITE);
  display->drawXBitmap(30,2,X_Logo_bits,X_Logo_width,X_Logo_height,WHITE);
  display->display();
  delay(1000);
  display->clearDisplay();
  display->drawXBitmap(0,2,G_Logo_bits,G_Logo_width,G_Logo_height,WHITE);
  display->display();
  delay(1000);
  PowerOff();
}