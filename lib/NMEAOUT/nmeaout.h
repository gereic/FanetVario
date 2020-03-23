#ifndef __NMEAOUT_H__
#define __NMEAOUT_H__


#include <Arduino.h>
#include <WiFiUdp.h>
#include <BluetoothSerial.h> //Header File for Serial Bluetooth, will be added by default into Arduino


enum class eNMEAOUTPUT {
  SERIAL_OUT = 0,
  UDP_OUT = 1,
  BLUETOOTH_OUT = 2,
  BLE_OUT = 3,
};

class NmeaOut {
public:
    NmeaOut(); //constructor
    bool begin();
    bool begin(eNMEAOUTPUT dev,String udpIP,uint16_t udpPort,String hostname);
    void write(String s);
    void run(void); //has to be called cyclic
protected:
    eNMEAOUTPUT _outputDevice = eNMEAOUTPUT::SERIAL_OUT;
    String _udpIP = "";
    uint16_t _udpPort = 10110;
    WiFiUDP udp;
    BluetoothSerial SerialBT;
    String sSend;

};

#endif