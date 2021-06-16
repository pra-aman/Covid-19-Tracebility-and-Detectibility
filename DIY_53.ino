#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;
#include <U8g2lib.h>
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#include "MAX30100_PulseOximeter.h"
PulseOximeter pox;
void onBeatDetected()
{
    Serial.println("Beat!");
    u8g2.setCursor(75, 42);
    u8g2.print("Beat!");
    
}

void setup() 
{
  Serial.begin(115200);
  Serial.println("Initialize MPU6050");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(100);
  }
  ///////////////////
  u8g2.begin(); 
  //////////////////
  Serial.println("Initialize MPU6050");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
 if (!pox.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }


    // pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);


    pox.setOnBeatDetectedCallback(onBeatDetected);
}

void loop()
{
  float temp = mpu.readTemperature();
  int symp=0;
  Serial.print(" Temp = ");
  Serial.print(temp);
  Serial.println(" *C");
  pox.update();
  u8g2.clearBuffer();
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_pxplusibmvga9_tf);
    u8g2.setCursor(0, 12);
    u8g2.print("Beat=");
    u8g2.print(pox.getHeartRate());
    u8g2.print("Bps");

    u8g2.setCursor(0, 26);
    u8g2.print("O2=");
    u8g2.print(pox.getSpO2());
    u8g2.print("%");
    
    
    u8g2.setCursor(0, 42);
    u8g2.print("T=");
    u8g2.print(mpu.readTemperature());
    u8g2.print(char(176));
    u8g2.print("c");
     
      if (temp>38){
    u8g2.setCursor(0, 49);
    u8g2.print("Fever Detected");
    Serial.println("fever");
    Serial.println ("Might be suspect");
    u8g2.setCursor(97, 26);
    u8g2.print("S1+");
  
    }
    if (pox.getSpO2()<75){
    u8g2.setCursor(0, 55);
    u8g2.print("breath shortness");

    u8g2.setCursor(88, 26);
    u8g2.print("S2+");
      
    }
   
   
    
  
    pox.update();
         } 
    while ( u8g2.nextPage() );
    pox.update();
    delay(100);
    }
