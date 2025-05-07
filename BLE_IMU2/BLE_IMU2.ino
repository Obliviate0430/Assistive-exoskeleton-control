/**
 * A BLE client example that is rich in capabilities.
 * There is a lot new capabilities implemented.
 * author unknown
 * updated by chegewara
 */

#include "BLEDevice.h"
//#include "BLEScan.h"

// The remote service we wish to connect to.
static BLEUUID serviceUUID("0000ffe5-0000-1000-8000-00805f9a34fb");
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID("0000ffe4-0000-1000-8000-00805f9a34fb");

// The remote service we wish to connect to.
static BLEUUID serviceUUID2("0000ffe5-0000-1000-8000-00805f9a34fb"); //根据第2个IMU实际serviceUUID确定是否需要修改
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID2("0000ffe4-0000-1000-8000-00805f9a34fb"); //根据第2个IMU实际charUUID确定是否需要修改

static boolean doConnect1 = false;
static boolean doConnect2 = false;
static boolean connected1 = false;
static boolean connected2 = false;
static boolean doScan1 = false;
static boolean doScan2 = false;
static BLERemoteCharacteristic* pRemoteCharacteristic1;
static BLERemoteCharacteristic* pRemoteCharacteristic2;
static BLEAdvertisedDevice* myDevice1;
static BLEAdvertisedDevice* myDevice2;
int led = 8;
static int count = 0;

static void notifyCallback1(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial1.write(0X67); //第1个IMU数据包头部增加了标志位0X67，STM32串口中断接收代码需相应修改
    Serial1.write(pData, length);
    // Serial1.println();
    
    if(count % 6 == 0)
      digitalWrite(led, HIGH);   // turn the LED on 
    else
      digitalWrite(led, LOW);  
    count++; 
}
static void notifyCallback2(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial1.write(0X68); //第2个IMU数据包头部增加了标志位0X68，STM32串口中断接收代码需相应修改
    Serial1.write(pData, length);
    // Serial1.println();

    if(count % 6 == 3)
      digitalWrite(led, HIGH);   // turn the LED on 
    else
      digitalWrite(led, LOW);  
    count++; 
}

class MyClientCallback1 : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected1 = false;
    Serial1.println("onDisconnect1");
  }
};
class MyClientCallback2 : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected2 = false;
    Serial1.println("onDisconnect2");
  }
};

bool connectToServer1() {
    // Serial1.print("Forming a connection to ");
    // Serial1.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    // Serial1.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback1());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice1);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    // Serial1.println(" - Connected to server");
    pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      // Serial1.print("Failed to find our service UUID: ");
      // Serial1.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    // Serial1.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic1 = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic1 == nullptr) {
      // Serial1.print("Failed to find our characteristic UUID: ");
      // Serial1.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial1.println(" - Found our characteristic1");

    // Read the value of the characteristic.
    if(pRemoteCharacteristic1->canRead()) {
      std::string value = pRemoteCharacteristic1->readValue();
      // Serial1.print("The characteristic value was: ");
      // Serial1.println(value.c_str());
    }

    if(pRemoteCharacteristic1->canNotify())
      pRemoteCharacteristic1->registerForNotify(notifyCallback1);

    connected1 = true;
    return true;
}
bool connectToServer2() {
    // Serial1.print("Forming a connection to ");
    // Serial1.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    // Serial1.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback2());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice2);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    // Serial1.println(" - Connected to server");
    pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID2);
    if (pRemoteService == nullptr) {
      Serial1.print("Failed to find our service2 UUID: ");
      Serial1.println(serviceUUID2.toString().c_str());
      pClient->disconnect();
      return false;
    }
    // Serial1.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic2 = pRemoteService->getCharacteristic(charUUID2);
    if (pRemoteCharacteristic2 == nullptr) {
      Serial1.print("Failed to find our characteristic2 UUID: ");
      Serial1.println(charUUID2.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial1.println(" - Found our characteristic2");

    // Read the value of the characteristic.
    if(pRemoteCharacteristic2->canRead()) {
      std::string value = pRemoteCharacteristic2->readValue();
      Serial1.print("The characteristic2 value was: ");
      Serial1.println(value.c_str());
    }

    if(pRemoteCharacteristic2->canNotify())
      pRemoteCharacteristic2->registerForNotify(notifyCallback2);

    connected2 = true;
    return true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // Serial1.print("BLE Advertised Device found: ");
    // Serial1.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    // if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) 
    if (advertisedDevice.haveName() && (advertisedDevice.getName()=="WT901BLE67"))
    {
      myDevice1 = new BLEAdvertisedDevice(advertisedDevice);
      doConnect1 = true;
      doScan1 = true;

    } // Found our server
    if (advertisedDevice.haveName() && (advertisedDevice.getName()=="WT901BLE68"))//根据第2个IMU实际名称确定是否需要修改
    {
      myDevice2 = new BLEAdvertisedDevice(advertisedDevice);
      doConnect2 = true;
      doScan2 = true;

    } // Found our server
    if(doConnect1 && doConnect2)
    {
      BLEDevice::getScan()->stop();
    }
  } // onResult
}; // MyAdvertisedDeviceCallbacks


void setup() {  
  Serial1.begin(115200, SERIAL_8N1, 1, 0); //使用默认串口Serial时报错，改为Serial1
  Serial1.println("Starting Arduino BLE Client application...");
  BLEDevice::init("BLE-Client");
  
  pinMode(led, OUTPUT);
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
} // End of setup.


// This is the Arduino main loop function.
void loop() {
  // Serial1.print("connected1, connected2: ");
  // Serial1.print(connected1);
  // Serial1.print(connected2);
  // Serial1.println();
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect1 == true) {
    if (connectToServer1()) {
      Serial1.println("We are now connected to the BLE Server1.");
    } else {
      Serial1.println("We have failed to connect to the server1; there is nothin more we will do.");
    }
    doConnect1 = false;
  }
  if (doConnect2 == true) {
    if (connectToServer2()) {
      Serial1.println("We are now connected to the BLE Server2.");
    } else {
      Serial1.println("We have failed to connect to the server2; there is nothin more we will do.");
    }
    doConnect2 = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected1 && connected2) {
    // String newValue = "Time since boot: " + String(millis()/1000);
    // Serial1.println("Setting new characteristic value to \"" + newValue + "\"");
    
    // Set the characteristic's value to be the array of bytes that is actually a string.
    // pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
  }else if(doScan1 || doScan2){
    BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
  }
  
  delay(1000); // Delay a second between loops.
} // End of loop
