/*
 *  This sketch sends data via HTTP GET requests to data.sparkfun.com service.
 *
 *  You need to get streamId and privateKey at data.sparkfun.com and paste them
 *  below. Or just customize this script to talk to other HTTP servers.
 *  ESP8266 Arduino example
 */
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <TM1637Display.h>
#include <pins_arduino.h>
#include <EncoderStepCounter.h>
#include "wifi_credentials.h"

#define ENCODER_PIN1 D6
#define ENCODER_INT1 digitalPinToInterrupt(ENCODER_PIN1)
#define ENCODER_PIN2 D7
#define ENCODER_INT2 digitalPinToInterrupt(ENCODER_PIN2)

// Create instance for one full step encoder
//EncoderStepCounter encoder(ENCODER_PIN1, ENCODER_PIN2);
// Use the following for half step encoders
//EncoderStepCounter encoder(ENCODER_PIN1, ENCODER_PIN2, HALF_STEP);

const long utcOffsetInSeconds = (-8 * 60 * 60);
//For UTC -8.00 : -8 * 60 * 60 : -18000

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);



// Module connection pins (Digital Pins)

#define led D4
#define CLK D0


// The amount of time (in milliseconds) between tests
#define TEST_DELAY   2000

const uint8_t SEG_DONE[] = {
	SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
	SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
	SEG_D | SEG_E | SEG_F | SEG_G,            // F
  0                           // 
	};

 TM1637Display display(CLK, D3);
 TM1637Display display2(CLK, D5);

void connect_to_wifi();
void seven_segment_display();
void seven_segment_display2();
void clock_seven_segment_display();
void interrupt() ;

void setup()
{
   // Initialize encoder
  //encoder.begin();
  // Initialize interrupts
  // attachInterrupt(ENCODER_INT1, interrupt, CHANGE);
  // attachInterrupt(ENCODER_INT2, interrupt, CHANGE);
  Serial.begin(115200);
  delay(10);
  pinMode(led, OUTPUT);
  connect_to_wifi();
}

// Call tick on every change interrupt
// void interrupt() {
//   encoder.tick();
// }

// This is an example on how to change a "long" variable
// with the library. With every loop the value is added
// and then cleared in the encoder library
signed long position = 0;
void loop() {
  // signed char pos = encoder.getPosition();
  // if (pos != 0) {
  //   position += pos;
  //   encoder.reset();
  //   Serial.println(position);
  // }
 


  delay(1000);
  clock_seven_segment_display();
  digitalWrite(led, !digitalRead(led)); //toggle the on-board led
  // seven_segment_display2();
  // seven_segment_display();
  switch(WiFi.status()){
    case WL_NO_SSID_AVAIL :
    Serial.println("No SSID ");
    break;
    case WL_SCAN_COMPLETED :
    Serial.println(" WL_SCAN_COMPLETED ");
    break;
    case WL_CONNECTED :
    Serial.println("WL_CONNECTED ");
    break;
    case WL_CONNECT_FAILED :
    Serial.println("WL_CONNECT_FAILED ");
    break;
    case WL_CONNECTION_LOST :
    Serial.println("WL_CONNECTION_LOST ");
    break;
    case WL_DISCONNECTED :
    Serial.println("WL_DISCONNECTED ");
    break;
    case WL_IDLE_STATUS :
    Serial.println("WL_IDLE_STATUS ");
    break;
    case WL_NO_SHIELD :
    Serial.println("WL_NO_SHIELD"); 
    break;

  }
  /*
  WL_IDLE_STATUS      = 0,
    WL_NO_SSID_AVAIL    = 1,
    WL_SCAN_COMPLETED   = 2,
    WL_CONNECTED        = 3,
    WL_CONNECT_FAILED   = 4,
    WL_CONNECTION_LOST  = 5,
    WL_DISCONNECTED     = 6
    */
}

void connect_to_wifi()
{
  // We start by connecting to a WiFi network
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }

}

void flash_led() {
  delay (1000);
  digitalWrite(led, !digitalRead(led)); //toggle the on-board led
  
  timeClient.update();

  Serial.print(daysOfTheWeek[timeClient.getDay()]);
  Serial.print(", ");
  Serial.print(timeClient.getHours());
  Serial.print(":");
  Serial.print(timeClient.getMinutes());
  Serial.print(":");
  Serial.println(timeClient.getSeconds());
  Serial.println(timeClient.getFormattedTime());

  

}


void clock_seven_segment_display()
{
   timeClient.update();

  Serial.print(daysOfTheWeek[timeClient.getDay()]);
  Serial.print(", ");
  Serial.print(timeClient.getHours());
  Serial.print(":");
  Serial.print(timeClient.getMinutes());
  Serial.print(":");
  Serial.println(timeClient.getSeconds());
  Serial.println(timeClient.getFormattedTime());

  int hour=timeClient.getHours();
  int minutes=timeClient.getMinutes();

  int displayValue=(hour*100)+minutes;

  //display.setBrightness(0x0f);
  display.setBrightness(0x02);
  int colon =0;
  if(digitalRead(led)){
    colon=0xff;
  }
   display.showNumberDecEx(displayValue, colon, true);
 

}

void seven_segment_display()
{
  int k;
  uint8_t data[] = { 0xff, 0xff, 0xff, 0xff };
  uint8_t blank[] = { 0x00, 0x00, 0x00, 0x00 };
  display.setBrightness(0x0f);

  // All segments on
  display.setSegments(data);
  flash_led();

  // Selectively set different digits
  data[0] = display.encodeDigit(0);
  data[1] = display.encodeDigit(1);
  data[2] = display.encodeDigit(2);
  data[3] = display.encodeDigit(3);
  display.setSegments(data);
  flash_led();

  /*
  for(k = 3; k >= 0; k--) {
	display.setSegments(data, 1, k);
	flash_led();
	}
  */

  display.clear();
  display.setSegments(data+2, 2, 2);
  flash_led();

  display.clear();
  display.setSegments(data+2, 2, 1);
  flash_led();

  display.clear();
  display.setSegments(data+1, 3, 1);
  flash_led();


  // Show decimal numbers with/without leading zeros
  display.showNumberDec(0, false); // Expect: ___0
  flash_led();
  display.showNumberDec(0, true);  // Expect: 0000
  flash_led();
	display.showNumberDec(1, false); // Expect: ___1
	flash_led();
  display.showNumberDec(1, true);  // Expect: 0001
  flash_led();
  display.showNumberDec(301, false); // Expect: _301
  flash_led();
  display.showNumberDec(301, true); // Expect: 0301
  flash_led();
  display.clear();
  display.showNumberDec(14, false, 2, 1); // Expect: _14_
  flash_led();
  display.clear();
  display.showNumberDec(4, true, 2, 2);  // Expect: 04__
  flash_led();
  display.showNumberDec(-1, false);  // Expect: __-1
  flash_led();
  display.showNumberDec(-12);        // Expect: _-12
  flash_led();
  display.showNumberDec(-999);       // Expect: -999
  flash_led();
  display.clear();
  display.showNumberDec(-5, false, 3, 0); // Expect: _-5_
  flash_led();
  display.showNumberHexEx(0xf1af);        // Expect: f1Af
  flash_led();
  display.showNumberHexEx(0x2c);          // Expect: __2C
  flash_led();
  display.showNumberHexEx(0xd1, 0, true); // Expect: 00d1
  flash_led();
  display.clear();
  display.showNumberHexEx(0xd1, 0, true, 2); // Expect: d1__
  flash_led();
	// Run through all the dots
	for(k=0; k <= 4; k++) {
		display.showNumberDecEx(0, (0x80 >> k), true);
		flash_led();
	}

  // Brightness Test
  for(k = 0; k < 4; k++)
	data[k] = 0xff;
  for(k = 0; k < 7; k++) {
    display.setBrightness(k);
    display.setSegments(data);
    flash_led();
  }
  
  // On/Off test
  for(k = 0; k < 4; k++) {
    display.setBrightness(7, false);  // Turn off
    display.setSegments(data);
    flash_led();
    display.setBrightness(7, true); // Turn on
    display.setSegments(data);
    flash_led();  
  }

 
  // Done!
  display.setSegments(SEG_DONE);

  
}

void seven_segment_display2()
{
  int k;
  uint8_t data[] = { 0xff, 0xff, 0xff, 0xff };
  uint8_t blank[] = { 0x00, 0x00, 0x00, 0x00 };
  display2.setBrightness(0x0f);

  // All segments on
  display2.setSegments(data);
  flash_led();

  // Selectively set different digits
  data[0] = display2.encodeDigit(0);
  data[1] = display2.encodeDigit(1);
  data[2] = display2.encodeDigit(2);
  data[3] = display2.encodeDigit(3);
  display2.setSegments(data);
  flash_led();

  /*
  for(k = 3; k >= 0; k--) {
	display2.setSegments(data, 1, k);
	flash_led();
	}
  */

  display2.clear();
  display2.setSegments(data+2, 2, 2);
  flash_led();

  display2.clear();
  display2.setSegments(data+2, 2, 1);
  flash_led();

  display2.clear();
  display2.setSegments(data+1, 3, 1);
  flash_led();


  // Show decimal numbers with/without leading zeros
  display2.showNumberDec(0, false); // Expect: ___0
  flash_led();
  display2.showNumberDec(0, true);  // Expect: 0000
  flash_led();
	display2.showNumberDec(1, false); // Expect: ___1
	flash_led();
  display2.showNumberDec(1, true);  // Expect: 0001
  flash_led();
  display2.showNumberDec(301, false); // Expect: _301
  flash_led();
  display2.showNumberDec(301, true); // Expect: 0301
  flash_led();
  display2.clear();
  display2.showNumberDec(14, false, 2, 1); // Expect: _14_
  flash_led();
  display2.clear();
  display2.showNumberDec(4, true, 2, 2);  // Expect: 04__
  flash_led();
  display2.showNumberDec(-1, false);  // Expect: __-1
  flash_led();
  display2.showNumberDec(-12);        // Expect: _-12
  flash_led();
  display2.showNumberDec(-999);       // Expect: -999
  flash_led();
  display2.clear();
  display2.showNumberDec(-5, false, 3, 0); // Expect: _-5_
  flash_led();
  display2.showNumberHexEx(0xf1af);        // Expect: f1Af
  flash_led();
  display2.showNumberHexEx(0x2c);          // Expect: __2C
  flash_led();
  display2.showNumberHexEx(0xd1, 0, true); // Expect: 00d1
  flash_led();
  display2.clear();
  display2.showNumberHexEx(0xd1, 0, true, 2); // Expect: d1__
  flash_led();
  
	// Run through all the dots
	for(k=0; k <= 4; k++) {
		display2.showNumberDecEx(0, (0x80 >> k), true);
		flash_led();
	}

  // Brightness Test
  for(k = 0; k < 4; k++)
	data[k] = 0xff;
  for(k = 0; k < 7; k++) {
    display2.setBrightness(k);
    display2.setSegments(data);
    flash_led();
  }
  
  // On/Off test
  for(k = 0; k < 4; k++) {
    display2.setBrightness(7, false);  // Turn off
    display2.setSegments(data);
    flash_led();
    display2.setBrightness(7, true); // Turn on
    display2.setSegments(data);
    flash_led();  
  }

 
  // Done!
  display2.setSegments(SEG_DONE);

  
}