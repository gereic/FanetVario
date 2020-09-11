#ifndef __NMEAOUT_H__
#define __NMEAOUT_H__


#include <Arduino.h>
#include <WiFiUdp.h>
#include <WiFi.h>

//#include <BluetoothSerial.h> //Header File for Serial Bluetooth, will be added by default into Arduino

/*
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

extern BLEServer *pServer;
extern BLECharacteristic * pTxCharacteristic;
extern bool deviceConnected;
extern bool oldDeviceConnected;
extern uint8_t txValue;
*/

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

/*
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
*/

enum class eNMEAOUTPUT {
  SERIAL_OUT = 0,
  UDP_OUT = 1,
  BLUETOOTH_OUT = 2,
  BLE_OUT = 3,
};

/*
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};
*/


class NmeaOut {
public:
    NmeaOut(); //constructor
    bool begin();
    bool begin(eNMEAOUTPUT dev,String udpIP,uint16_t udpPort,String hostname);
    void write(String s);
    void run(void); //has to be called cyclic
    String getSendData(void);
protected:
    eNMEAOUTPUT _outputDevice = eNMEAOUTPUT::SERIAL_OUT;
    String _udpIP = "";
    uint16_t _udpPort = 10110;
    WiFiUDP udp;
    String sData = "";
    //BluetoothSerial SerialBT;
    //String sSend;

};

#endif