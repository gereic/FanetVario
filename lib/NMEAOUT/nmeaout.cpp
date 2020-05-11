#include "nmeaout.h"

/*
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  Serial.print("BT Callback triggered:");
  Serial.println(event);
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected");
  }

  if(event == ESP_SPP_CLOSE_EVT ){
    Serial.println("Client disconnected");
  }
}
*/

NmeaOut::NmeaOut(){
}


bool NmeaOut::begin(){
    _outputDevice = eNMEAOUTPUT::SERIAL_OUT;
    _udpPort = 0;
    return true;
}

bool NmeaOut::begin(eNMEAOUTPUT dev,String udpIP,uint16_t udpPort,String hostname){
    _outputDevice = dev;
    _udpPort = udpPort;
    _udpIP = udpIP;
    switch (_outputDevice)
    {
    case eNMEAOUTPUT::UDP_OUT:
        udp.begin(_udpPort);
        break;
    case eNMEAOUTPUT::BLUETOOTH_OUT:   
        //SerialBT.register_callback(callback);
        //SerialBT.begin(hostname);
        //sSend = "";
        /*
        // Create the BLE Device
        BLEDevice::init("UART Service");

        // Create the BLE Server
        pServer = BLEDevice::createServer();
        pServer->setCallbacks(new MyServerCallbacks());

        // Create the BLE Service
        BLEService *pService = pServer->createService(SERVICE_UUID);

        // Create a BLE Characteristic
        pTxCharacteristic = pService->createCharacteristic(
                                                CHARACTERISTIC_UUID_TX,
                                                BLECharacteristic::PROPERTY_NOTIFY
                                            );
                            
        pTxCharacteristic->addDescriptor(new BLE2902());

        BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                                                    CHARACTERISTIC_UUID_RX,
                                                    BLECharacteristic::PROPERTY_WRITE
                                                );

        pRxCharacteristic->setCallbacks(new MyCallbacks());

        // Start the service
        pService->start();

        // Start advertising
        pServer->getAdvertising()->start();
        Serial.println("Waiting a client connection to notify...");
        */
        break;
    default:
        break;
    }
    return true;
}

void NmeaOut::write(String s){
    unsigned int bufferSize = s.length();
    uint8_t data[bufferSize+1];
    switch (_outputDevice)
    {
    case eNMEAOUTPUT::UDP_OUT:
        udp.beginPacket(_udpIP.c_str(),_udpPort);
        s.toCharArray((char *)data,bufferSize+1);
        udp.write(&data[0],bufferSize );
        udp.endPacket();  
        break; 

    case eNMEAOUTPUT::BLUETOOTH_OUT:   
        
        /*
        if (SerialBT.hasClient()){
          SerialBT.print("Lenght=");
          SerialBT.println(s.length());
        }
        */
        
       //sSend += s;
       /*
       if (sSend.length() == 0){
           sSend = s;
       } 
       */      
        //SerialBT.write(s.c_str());
        break;
    
    default:
        Serial.print(s);
        break;
    }


}

void NmeaOut::run(void){
    /*
    static uint32_t tSend = millis();
    uint32_t tAct = millis();
    if (_outputDevice == eNMEAOUTPUT::BLUETOOTH_OUT){
        if (SerialBT.hasClient()){
            if ((tAct - tSend) >= 250){
                SerialBT.print(sSend);
                sSend = "";
                tSend = tAct;
            }
        }else{
            sSend = "";
        }*/
        //if (SerialBT.available()){
        //    Serial.write(SerialBT.read()); //write to Serial-device
        //}
        /*
        if (deviceConnected) {
            pTxCharacteristic->setValue(&txValue, 1);
            pTxCharacteristic->notify();
            txValue++;
            delay(10); // bluetooth stack will go into congestion, if too many packets are sent
        }

        // disconnecting
        if (!deviceConnected && oldDeviceConnected) {
            delay(500); // give the bluetooth stack the chance to get things ready
            pServer->startAdvertising(); // restart advertising
            Serial.println("start advertising");
            oldDeviceConnected = deviceConnected;
        }
        // connecting
        if (deviceConnected && !oldDeviceConnected) {
            // do stuff here on connecting
            oldDeviceConnected = deviceConnected;
        }
        */
    //}
    /*
    if ((tAct - tSend) >= 100){
        if (SerialBT.hasClient()){
            //SerialBT.print("Dast ist ein TEST von Gerald\r\n");
            SerialBT.print(sSend);
            sSend = "";
            tSend = tAct;
        }
    }
    */
    
} //has to be called cyclic
