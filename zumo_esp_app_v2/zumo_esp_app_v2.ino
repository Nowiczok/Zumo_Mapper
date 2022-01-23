#include <AsyncUDP.h>
//#include <HardwareSerial.h>

// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <RPLidar.h>
#include <WiFi.h>
#include "ESPAsyncWebServer.h"

#define RPLIDAR_MOTOR 12
#define POINT_BUFF_LEN 2000

#define LeftMotor 12
#define RightMotor 13
#define LeftMotorDirection 14
#define RightMotorDirection 27

#define RX0 3
#define TX0 1

#define LED_PIN 26
const char* ssid = "AndroidAP";
const char* password = "Tj11qa05";

AsyncWebServer server(80);

RPLidar lidar;
float distanceBuff[POINT_BUFF_LEN];
float angleBuff[POINT_BUFF_LEN];
bool dataStartAcquisition = false;
bool getData = false;
int buffCount = 0;
int speed_zumo = 3;

String Forward()
{
    Serial.write('F');
    Serial.write(0x00);
    Serial.write(0x0A);
    return "Forward";
}

String Back()
{
    Serial.write('B');
    Serial.write(0x00);
    Serial.write(0x0A);
    return "Back";
}

String Left()
{
    Serial.write('L');
    Serial.write(0x00);
    Serial.write(0x0A);
    return "Left";
}

String Right()
{
    Serial.write('R');
    Serial.write(0x00);
    Serial.write(0x0A);
    return "Right";
}

String Stop()
{
    Serial.write('S');
    Serial.write(0x00);
    Serial.write(0x0A);
    return "STOP";
}

String SpeedUp()
{
  if(speed_zumo < 10)
  {
    speed_zumo += 1;
    return "speed up";
  }
  return "error";
}

String SpeedDown()
{
  if(speed_zumo >1)
  {
    speed_zumo -= 1;
    return "speed up";
  }
  return "error";
}

String getScan() 
{
  String toSend = "";
  for(int i = 0; i < buffCount; i++)
  {
    toSend += String(distanceBuff[i]) + ";" + String(angleBuff[i]) + ";";
  }
  return toSend;
}

String startAcquisition() 
{
  dataStartAcquisition = true;
  return "Starting";
}
                                             
void setup() 
{

    //Serial.begin(115200);
    Serial.begin(28800);
    //Serial1.begin(28800, SERIAL_8N1, RX0, TX0);
  
  //wifi config
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) 
   {
     delay(500);
     Serial.print(".");
   }
   IPAddress IP = WiFi.localIP();
   Serial.print("IP address: ");
   Serial.println(IP);
   
   pinMode(LED_PIN, OUTPUT);
   server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request)
   {
    request->send_P(200, "text/plain", startAcquisition().c_str());
   });
   
   server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request)
   {
    request->send_P(200, "text/plain", getScan().c_str());
   });
   
   server.on("/forward", HTTP_GET, [](AsyncWebServerRequest *request)
   {
    request->send_P(200, "text/plain", Forward().c_str());
   });
   
   server.on("/right", HTTP_GET, [](AsyncWebServerRequest *request)
   {
    request->send_P(200, "text/plain", Right().c_str());
   });

   server.on("/left", HTTP_GET, [](AsyncWebServerRequest *request)
   {
    request->send_P(200, "text/plain", Left().c_str());
   });

   server.on("/back", HTTP_GET, [](AsyncWebServerRequest *request)
   {
    request->send_P(200, "text/plain", Back().c_str());
   });

   server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request)
   {
    request->send_P(200, "text/plain", Stop().c_str());
   });

   server.on("/speed_up", HTTP_GET, [](AsyncWebServerRequest *request)
   {
    request->send_P(200, "text/plain", SpeedUp().c_str());
   });

   server.on("/speed_down", HTTP_GET, [](AsyncWebServerRequest *request)
   {
    request->send_P(200, "text/plain", SpeedDown().c_str());
   });

  server.begin();
  
  //lidar config
  lidar.begin(Serial2); 
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  digitalWrite(RPLIDAR_MOTOR, LOW);
}

void loop() 
{
  
  if (IS_OK(lidar.waitPoint())) 
  {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement

    if(startBit)
    {
      if(dataStartAcquisition)
      {
        dataStartAcquisition = false;
        getData = true;
        buffCount = 0;
      }else
      {
        getData = false;
      }
    }
    
    if(getData)
    {
      if(buffCount < POINT_BUFF_LEN)
      {
        distanceBuff[buffCount] = distance;
        angleBuff[buffCount] = angle;
        buffCount++;
      }
    }
     
  } else 
  {
    digitalWrite(RPLIDAR_MOTOR, LOW); //stop the rplidar motor
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) // try to detect RPLIDAR...
    {
       lidar.startScan(); // detected...
       digitalWrite(RPLIDAR_MOTOR, HIGH); // start motor rotating at max allowed speed
       delay(1000);
    }
  }
}
