#include <NMEAGPS.h>
#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

#define FONA_RX 5
#define FONA_TX 4
#define FONA_RST 11
#define I2C_ADDRESS 0x3C
#define RST_PIN -1
SSD1306AsciiWire oled;
int buzzerpin=13;
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

#define gpsPort Serial
#define GPS_PORT_NAME "Serial"
#define DEBUG_PORT Serial

#if !defined( NMEAGPS_PARSE_RMC ) &  \
    !defined( NMEAGPS_PARSE_GGA ) &  \
    !defined( NMEAGPS_PARSE_GLL )
  #error You must uncomment at least one of NMEAGPS_PARSE_RMC, NMEAGPS_PARSE_GGA or NMEAGPS_PARSE_GLL in NMEAGPS_cfg.h!
#endif

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

#if !defined( GPS_FIX_LOCATION )
  #error You must uncomment GPS_FIX_LOCATION in GPSfix_cfg.h!
#endif

char replybuffer[255];
String commands="";
char fonaNotificationBuffer[64];          //for notifications from the FONA
char smsBuffer[250];

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
NMEAGPS gps;

// The base location, in degrees * 10,000,000
//NeoGPS::Location_t base( 0.000221,0.000117 ); 
NeoGPS::Location_t home( 0.002651,0.000241);
NeoGPS::Location_t restrict1( 0.001651,0.000241); 
NeoGPS::Location_t restrict2( 0.001651,0.000241); 
NeoGPS::Location_t restrict3( 0.001651,0.000241); 
void setup()
{
  DEBUG_PORT.begin(9600);
  DEBUG_PORT.println( F("NMEAdistance.ino started.") );
  DEBUG_PORT.println( F("Looking for GPS device on " GPS_PORT_NAME) );
  pinMode(13,INPUT);
  gpsPort.begin(9600);
   fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while(1);
  }
  Serial.println(F("FONA is OK"));

  // Print SIM card IMEI number.
  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("SIM card IMEI: "); Serial.println(imei);
  }

  fonaSerial->print("AT+CNMI=2,1\r\n");  //set up the FONA to send a +CMTI notification when an SMS is received

  Serial.println("FONA Ready");
   oled.begin(&Adafruit128x64, I2C_ADDRESS);
  //oled.setFont(TimesNewRoman16_bold);
 oled.setFont(Callibri11_bold); 

} // setup

void loop()
{
  
  while (gps.available( gpsPort )) {
gps_fix fix = gps.read(); // save the latest
    // When we have a location, calculate how far away we are from the base location.
 
      float range1 = fix.location.DistanceMiles( home );
      float range2 = fix.location.DistanceMiles( restrict1 );
      float range3 = fix.location.DistanceMiles( restrict2 );
      float range4 = fix.location.DistanceMiles( restrict3 );

      DEBUG_PORT.print( F("Range:1 ") );
      DEBUG_PORT.print( range1 );
      DEBUG_PORT.print( F("Range:2 ") );
      DEBUG_PORT.print( range2 );
      DEBUG_PORT.print( F("Range:3") );
      DEBUG_PORT.print( range3 );
      DEBUG_PORT.println( "range in miles" );
      
      oled.clear();
      oled.println(" Distance from safe Zone");
      oled.print(range1);
      oled.print("miles\n");
      oled.print((range1*1.609));
      oled.print(" km\n");
 
   if (((range2*1.609)<0.19) or((range3*1.609)<0.1) or ((range4*1.609)<0.1)){
      oled.println("In infected area");
      if (!fona.sendSMS("7979952235", "Hey, Ashwini has entred in infected area")) {
        Serial.println(F("Failed"));
      } else {
        Serial.println(F("Sent!"));
        delay(4000);
      }
  }
   
   if (((range2*1.609)>0.19)){
       digitalWrite(13,HIGH);
       oled.println("roaming outside home");
     if (!fona.sendSMS("7979952235", "Hey, This man going out of home ")) {
        Serial.println(F("Failed"));
      } else {
        Serial.println(F("Sent!"));
        delay(4000);
      }
  }else{
    
  digitalWrite(13,LOW);
  delay(300);
}
  }
  
} // loop
