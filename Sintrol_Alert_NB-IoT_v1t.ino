
#include "BluetoothSerial.h"
 #define _TASK_TIMECRITICAL

#include <TaskScheduler.h>
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif


#include "REG_SINTROL_203.h"
#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include "HardwareSerial_NB_BC95.h"

#include <esp_now.h>
#include <WiFi.h>


HardwareSerial modbus(2);
HardwareSerial_NB_BC95 AISnb;

BluetoothSerial SerialBT;

// Global copy of slave
#define NUMSLAVES 10
esp_now_peer_info_t slaves[NUMSLAVES] = {};
int SlaveCnt = 0;

#define CHANNEL 3
#define PRINTSCANRESULTS 0

const int RED_PIN =  4;// RED LED
const int ORG_PIN = 15;// ORANGE LED
const int GRE_PIN = 33;// GREEN LED
 
String deviceToken = "Vi93LtG0OwpEEuxfK9pN";
String serverIP = "103.27.203.83"; // Your Server IP;
String serverPort = "9956"; // Your Server Port;
String json = "";
int32_t RSSIWiFi;
int TIMESEND = 10;
ModbusMaster node;
void t2CallgetProbe();
void t1CallcheckAlert();
void t3CallsendViaNBIOT();
//TASK
Task t2(1000, TASK_FOREVER, &t2CallgetProbe);
Task t1(5100, TASK_FOREVER, &t1CallcheckAlert);
Task t3(5000, TASK_FOREVER, &t3CallsendViaNBIOT);
Scheduler runner;

//const long interval = 1000;  //millisecond
//unsigned long previousMillis = 0;

boolean initBase = true;
//const long low = 2;
const long medium = 3;
const long high = 5;

long baseline = 0;
long baselineXmedium = 0;
long baselineXhigh = 0;
 
 
long resultDust = 0;

const int bufferSize = 10;
long circular[bufferSize];
int count = 0;
double avgDust = 0;
double avg = 0;
signal meta ;



void t2CallgetProbe() {     // Update read all data
  //    delay(1000);                              // เคลียบัสว่าง
  for (char i = 0; i < Total_of_Reg ; i++) {
    DATA_METER [i] = Read_Meter_float(ID_meter, Reg_addr[i]);//แสกนหลายตัวตามค่า ID_METER_ALL=X
  }
}

void addReading(long reading) {
  //  Serial.print("add:");
  //  Serial.println(reading);
  circular[count] = reading;
  Serial.print("count:");

  count++;
  Serial.println(count);
 


}

long average() {
  long sum = 0;
  for (int i = 0; i < bufferSize; i++) {
    sum += circular[i];

  }
  int divider = count;
  count = 0;
  memset(circular, 0, sizeof(10));
  long avg = (long) (sum / divider);
  if (initBase) {
      baseline = avg;
      baselineXmedium = baseline * medium;
      baselineXhigh = baseline * high; 
      initBase = false;
  }
  return avg;
}


float Read_Meter_float(char addr , uint16_t  REG) {
  unsigned int i = 0;
  uint32_t j, result;
  uint16_t data[2];
  uint32_t value = 0;
  node.begin(255, modbus);
  result = node.readInputRegisters (REG, 2); ///< Modbus function 0x04 Read Input Registers
  delay(400);
  if (result == node.ku8MBSuccess) {
    for (j = 0; j < 2; j++)
    {
      data[j] = node.getResponseBuffer(j);
      //      Serial.println(data[j]);
    }
    resultDust = getResult(data[1], data[0]);
    addReading(resultDust);
    Serial.println(resultDust);
    return i;
  } else {
    Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug
//    delay(1000);
    return 0;
  }
}




// Init ESP Now with fallback
void initESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    //     initESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// Scan for slaves in AP mode
void scanForSlave() {
  int8_t scanResults = WiFi.scanNetworks();
  //reset slaves
  memset(slaves, 0, sizeof(slaves));
  SlaveCnt = 0;
  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      RSSIWiFi = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

//      if (PRINTSCANRESULTS) {
//        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSIWiFi); Serial.print(")"); Serial.println("");
//      }
//      delay(10);
      // Check if the current device starts with `deviceName`
      if (SSID.equals("dust001")) {
        // SSID of interest
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSIWiFi); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];

        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x%c",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slaves[SlaveCnt].peer_addr[ii] = (uint8_t) mac[ii];
          }
        }
        slaves[SlaveCnt].channel = CHANNEL; // pick a channel
        slaves[SlaveCnt].encrypt = 0; // no encryption
        SlaveCnt++;
      }
    }
  }

  if (SlaveCnt > 0) {
    Serial.print(SlaveCnt); Serial.println(" Slave(s) found, processing..");
  } else {
    Serial.println("No Slave Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
   // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (SlaveCnt > 0) { // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    manageSlave();
    // pair success or already paired
    // Send data to device
    Serial.print("Send to slave "); Serial.println(avgDust);

  } else {
    // No slave found to process
    Serial.println("No slave found");
  }
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
void manageSlave() {
  if (SlaveCnt > 0) {
    for (int i = 0; i < SlaveCnt; i++) {
      const esp_now_peer_info_t *peer = &slaves[i];
      const uint8_t *peer_addr = slaves[i].peer_addr;
      Serial.print("Processing: ");
      for (int ii = 0; ii < 6; ++ii ) {
        Serial.print((uint8_t) slaves[i].peer_addr[ii], HEX);
        if (ii != 5) Serial.print(":");
      }
      Serial.print(" Status: ");
      // check if the peer exists
      bool exists = esp_now_is_peer_exist(peer_addr);
      if (exists) {
        // Slave already paired.
        Serial.println("Already Paired");
      } else {
        // Slave not paired, attempt pair
        esp_err_t addStatus = esp_now_add_peer(peer);
        if (addStatus == ESP_OK) {
          // Pair success
          Serial.println("Pair success");
        } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
          // How did we get so far!!
          Serial.println("ESPNOW Not Init");
        } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
          Serial.println("Add Peer - Invalid Argument");
        } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
          Serial.println("Peer list full");
        } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
          Serial.println("Out of memory");
        } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
          Serial.println("Peer Exists");
        } else {
          Serial.println("Not sure what happened");
        }
        //        delay(100);
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
  }
}


uint8_t data = 0;
// send data
void sendData(int level) {
  data = level;
  for (int i = 0; i < SlaveCnt; i++) {
    const uint8_t *peer_addr = slaves[i].peer_addr;
    if (i == 0) { // print only for first slave
      Serial.print("--->Sending: ");
      Serial.println(data);
    }
    esp_err_t result = esp_now_send(peer_addr, &data, sizeof(data));
    Serial.print("Send Status: ");
    if (result == ESP_OK) {
      Serial.println("Success");
    } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
      // How did we get so far!!
      Serial.println("ESPNOW not Init.");
    } else if (result == ESP_ERR_ESPNOW_ARG) {
      Serial.println("Invalid Argument");
    } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
      Serial.println("Internal Error");
    } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
      Serial.println("ESP_ERR_ESPNOW_NO_MEM");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
      Serial.println("Peer not found.");
    } else {
      Serial.println("Not sure what happened");
    }
    delay(100);
  }
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
//  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  //  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}



//**************************************************************************************************************
void setup() {

  Serial.begin(115200);
  SerialBT.begin("Sintrol001"); //Bluetooth device name
  SerialBT.println("Sintrol001");
  modbus.begin(38400, SERIAL_8N1, 16, 17);
  Serial.println(F("Starting... Sintrol Monitor"));
  SerialBT.println(F("Starting... Sintrol Monitor"));

  // communicate with Modbus slave ID 1 over Serial (port 2)
  node.begin(ID_meter, modbus);

  Serial.println();
  Serial.println(F("***********************************"));

  WiFi.mode(WIFI_STA);
  Serial.println("ESPNow/Multi-Slave/Master");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  // Init ESPNow with a fallback logic
  initESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);


  runner.init();
  Serial.println("Initialized scheduler");

  runner.addTask(t1);
  Serial.println("added t1");
  runner.addTask(t2);
  Serial.println("added t2");
  runner.addTask(t3);
  Serial.println("added t3");
  delay(2000);
  t1.enable();  Serial.println("Enabled t1");
  t2.enable();  Serial.println("Enabled t2");
  t3.enable();  Serial.println("Enabled t3");
  AISnb.debug = true;
  AISnb.setupDevice(serverPort);
  deviceToken = AISnb.getNCCID();
  delay(4000);
  if(deviceToken.length() <1 )
    ESP.restart();
  String ip1 = AISnb.getDeviceIP();
  scanForSlave();
  pinMode(RED_PIN, OUTPUT);
  pinMode(ORG_PIN, OUTPUT);
  pinMode(GRE_PIN, OUTPUT);
 
  delay(1000);
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(ORG_PIN, HIGH);
  digitalWrite(GRE_PIN, HIGH);
  delay(4000);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(ORG_PIN, LOW);
  digitalWrite(GRE_PIN, LOW);
}

void loop() {
  runner.execute();
}

void t3CallsendViaNBIOT () {
  avgDust = average();

  meta = AISnb.getSignal();  
//  Serial.print("baseline:");     Serial.println(baseline);
//  Serial.print("avg:");     Serial.println(avgDust);
//  Serial.print("RSSI:"); Serial.println(meta.rssi);
  

  String json = "";
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);
  json.concat("\",\"dust\":");
  json.concat(avgDust);
  json.concat(",\"rssi\":");
  json.concat(meta.rssi);
  json.concat(",\"csq\":");
  json.concat(meta.csq);
  json.concat(",\"base\":");
  json.concat(baseline);
  json.concat(",\"1\":");
  json.concat(baselineXmedium);
  json.concat(",\"2\":");
  json.concat(baselineXhigh);
  json.concat(",\"wifi\":");
  json.concat(RSSIWiFi);

  json.concat("}");
  Serial.println(json);
  SerialBT.println(json);

  UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);
  UDPReceive resp = AISnb.waitResponse();
//  Serial.print("rssi:");
//  Serial.println(meta.rssi);
//  SerialBT.print("rssi:");
//  SerialBT.println(meta.rssi);
}
void greenOff() {
  Serial.println("Green OFF");
  digitalWrite(GRE_PIN, LOW);
}

void greenOn() {
  Serial.println("Green ON");
  digitalWrite(GRE_PIN, HIGH);
   digitalWrite(ORG_PIN, LOW);
   digitalWrite(RED_PIN, LOW);
}

void orangeOn() {
  Serial.println("Orange ON");
  digitalWrite(ORG_PIN, HIGH);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GRE_PIN, LOW);
}
void orangeOff() {
  Serial.println("Orange OFF");
  digitalWrite(ORG_PIN, LOW);
}

void redOn() {
  Serial.println("Red ON");
  digitalWrite(RED_PIN, HIGH);
   digitalWrite(GRE_PIN, LOW);
   digitalWrite(ORG_PIN, LOW);
}
void redOff() {
  Serial.println("Red OFF");
  digitalWrite(RED_PIN, LOW);
}

void soundOn() {
  Serial.println("Sound ON");

}
void soundOff() {
  Serial.println("Sound OFF");
}

void t1CallcheckAlert() {
  
 Serial.println("checkAlert()");

  if (avgDust < baselineXmedium) {
    greenOn();
    scanForSlave();
    sendData(1);
    Serial.println("Low");
  } else if ((avgDust >  baselineXmedium) && (avgDust <  baselineXhigh)) {
    orangeOn();
    scanForSlave();
    sendData(2);
    Serial.println("Alarm medium");
  } else if (avgDust > baselineXhigh) {
    redOn();
    scanForSlave();
    sendData(3);
    Serial.println("Alert High");
  }


}


String decToHex(int decValue) {

  String hexString = String(decValue, HEX);
  return hexString;
}


unsigned int hexToDec(String hexString) {

  unsigned int decValue = 0;
  int nextInt;

  for (int i = 0; i < hexString.length(); i++) {

    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);

    decValue = (decValue * 16) + nextInt;
  }

  return decValue;
}

long getResult( unsigned int x_high, unsigned int x_low)
{
  String hex2 = "";
  hex2.concat(decToHex(x_low));
  hex2.concat(decToHex(x_high));
  //  Serial.print("hex:");  Serial.println(hex2);  Serial.print("dec:");
  //  Serial.println(hexToDec(hex2));                                                               //rightmost 8 bits
  long dust = hexToDec(hex2);
  return dust;
}
