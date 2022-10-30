#include <LIDARLite.h>
#include <DynamixelSerial.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>


#define START_RECEIVING_BYTE  0x25
#define CMD_TO_SWEEP          0x11
#define END_BYTE              0x0D

const char* ssid = "BKI_VGV_Device";
const char* password = "1234567890";

const char* udpAddr = "192.168.0.12";
const int udpPort = 3333;

bool connected = false;
bool sweeping_done = false;

byte checksum;

float lidardeg_f;
short lidardeg_s;

IPAddress local_IP(192, 168, 0, 111);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiManager wm;
WiFiUDP udp;

LIDARLite garminv3;

float lidar_m;
short lidar_cm;
uint8_t packetBuf[8];

void setup() {
    WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
     
    if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
    }
      
    WiFi.begin(ssid, password);
    udp.begin(local_IP, udpPort); 
    garminv3.begin(0, true);
    garminv3.configure(0);
    Serial.begin(115200);
    
    Dynamixel.setSerial(&Serial2); // &Serial - Arduino UNO/NANO/MICRO, &Serial1, &Serial2, &Serial3 - Arduino Mega
    Dynamixel.begin(1000000,5);  // Inicialize the servo at 1 Mbps and Pin Control 2
    Dynamixel.setEndless(1,OFF);
    //reset settings - wipe credentials for testing
    //wm.resetSettings();

    wm.setConfigPortalBlocking(false);
    wm.setConfigPortalTimeout(60);
    //automatically connect using saved credentials if they exist
    //If connection fails it starts an access point with the specified name
    if(wm.autoConnect("BKI-VGV-Length")){
        Serial.println("connected...yeey :)");
    }
    else {
        Serial.println("Configportal running");
    }
}

void loop() {
  wm.process();

  memset(packetBuf, 0, sizeof(packetBuf));
  int packet_size = udp.parsePacket();
  
  if(packet_size > 0){
    udp.read(packetBuf, packet_size);
    Serial.print("Contents : ");
    Serial.println((char*)packetBuf);
    
    if(packetBuf[0] == 0x25 && packetBuf[packet_size-1] == 0x0D){
      switch (packetBuf[1]){
        case 0x11:
          sweeping_done = false;
          short startdeg = packetBuf[2];
          short enddeg = packetBuf[3]; 
          Serial.print("Start Deg : ");
          Serial.print(startdeg);
          Serial.print("End Deg : ");
          Serial.println(enddeg);
          if(startdeg < enddeg){
            lidardeg_f = startdeg;
            while(!sweeping_done){
              setPan(lidardeg_f);
              delay(20);
              // lidar_m = getLidarData();
              lidar_cm = garminv3.distance();
              if(lidar_cm == 1){
                lidar_cm = 99999;
              }
              lidardeg_s = lidardeg_f * 10;
              Serial.print("Sweeping...");
              Serial.print("Lidar Deg = ");
              Serial.print(lidardeg_f);
              Serial.print("Lidar Distance : ");
              Serial.println(lidar_cm);
              memset(packetBuf, 0, sizeof(packetBuf));
              packetBuf[0] = 0x33;
              memcpy(packetBuf + 1, &lidardeg_s, 2);
              memcpy(packetBuf + 3, &lidar_cm, 2);
              checksum = lidardeg_s ^ lidar_cm;
              memcpy(packetBuf + 5, &checksum, 2);
              packetBuf[7] = END_BYTE;
              udp.beginPacket(udp.remoteIP(), udpPort);
              udp.write(packetBuf, sizeof(packetBuf));
              udp.endPacket();

              if(lidardeg_f == enddeg){
                sweeping_done = true;
                Serial.println("SWEEPING DONE");
                setPan(150);
              }
              lidardeg_f += 0.5;
              delay(30);
            }            
          }else{
            Serial.println("Invalid START & END DEG");
          }
          
          break;
      }
    }
  }

    delay(100);
;}


int angle2raw(float angle){
  int raw;
  raw = (angle * 1023 / 300);
  return raw;
}

// void setPan(int angle){
//   int raw_cmd;
//   if(angle < 70) angle = 70;
//   if(angle > 230) angle = 230;

//   raw_cmd = angle2raw(angle);
//   Dynamixel.move(1, raw_cmd);
//   Dynamixel.ledStatus(1, ON);
//   Dynamixel.action();
// }

void setPan(float angle){
  int raw_cmd;
  if(angle < 70) angle = 70;
  if(angle > 230) angle = 230;

  raw_cmd = angle2raw(angle);
  Serial.println(raw_cmd);
  Dynamixel.move(1, raw_cmd);
  Dynamixel.ledStatus(1, ON);
  Dynamixel.action();
}
