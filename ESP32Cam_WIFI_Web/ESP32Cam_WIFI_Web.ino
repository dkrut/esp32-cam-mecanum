#include "esp_camera.h"
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define CAMERA_MODEL_AI_THINKER

#define LED   4
#define RXD2 14
#define TXD2 13

void CameraWebServer_init();

WiFiServer server(100);


extern int gpLed =  4; // Light

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // prevent brownouts by silencing them

  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.setDebugOutput(true);
  Serial.println();

  CameraWebServer_init();

  pinMode(gpLed, OUTPUT); //Light
  ledcSetup(7, 5000, 8);
  ledcAttachPin(gpLed, 7);  //pin4 is LED

  server.begin();

  for (int i = 0; i < 5; i++) 
  {
    ledcWrite(7, 10); // flash led
    delay(50);
    ledcWrite(7, 0);
    delay(50);
  }
}

void loop() 
{
  //delay(1000);
  //Serial.printf("RSSi: %ld dBm\n", WiFi.RSSI());
}
