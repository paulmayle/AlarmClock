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
EncoderStepCounter encoder(ENCODER_PIN1, ENCODER_PIN2);
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
#define TEST_DELAY 2000

const uint8_t SEG_DONE[] = {
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, // O
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, // O
    SEG_D | SEG_E | SEG_F | SEG_G,                 // F
    0                                              //
};

const uint8_t SEG_DASH[] = {
    SEG_G,
    SEG_G,
    SEG_G,
    SEG_G};

TM1637Display display(CLK, D3);
TM1637Display display2(CLK, D5);

void connect_to_wifi();
void seven_segment_display();
void seven_segment_display2();
void clock_seven_segment_display();
ICACHE_RAM_ATTR void interrupt();

bool connected = false;

void setup()
{
  // Initialize encoder
  encoder.begin();
  // Initialize interrupts
  attachInterrupt(ENCODER_INT1, interrupt, CHANGE);
  attachInterrupt(ENCODER_INT2, interrupt, CHANGE);
  Serial.begin(115200);
  delay(10);
  pinMode(led, OUTPUT);
  connect_to_wifi();
}

// Call tick on every change interrupt
ICACHE_RAM_ATTR void interrupt()
{
  encoder.tick();
}

// This is an example on how to change a "long" variable
// with the library. With every loop the value is added
// and then cleared in the encoder library
signed long position = 0;
void loop()
{
  signed char pos = encoder.getPosition();
  if (pos != 0)
  {
    position += pos;
    encoder.reset();
    Serial.println(position);
  }

  delay(1000);
  clock_seven_segment_display();
  digitalWrite(led, !digitalRead(led)); //toggle the on-board led
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

void clock_seven_segment_display()
{
  int displayValue = 0;

  display.setBrightness(0x0f);

  switch (WiFi.status())
  {
  case WL_NO_SSID_AVAIL:
    Serial.println("No SSID ");
    connected = false;
    break;
  case WL_SCAN_COMPLETED:
    Serial.println(" WL_SCAN_COMPLETED ");
    break;
  case WL_CONNECTED:
    Serial.println("WL_CONNECTED ");
    if (connected == false)
    {
      timeClient.forceUpdate();
      connected = true;
    }
    timeClient.update();

    break;
  case WL_CONNECT_FAILED:
    Serial.println("WL_CONNECT_FAILED ");
    connected = false;

    break;
  case WL_CONNECTION_LOST:
    Serial.println("WL_CONNECTION_LOST ");
    connected = false;

    break;
  case WL_DISCONNECTED:
    Serial.println("WL_DISCONNECTED ");
    connected = false;

    break;
  case WL_IDLE_STATUS:
    Serial.println("WL_IDLE_STATUS ");
    break;
  case WL_NO_SHIELD:
    Serial.println("WL_NO_SHIELD");
    break;
  }

  int colon = 0;
  if (digitalRead(led))
  {
    colon = 0xff;
  }
  //display.showNumberDecEx(0, colon, false);

  if (connected == false)
  {
    display.setSegments(SEG_DASH);
  }
  else
  {
    int hour = timeClient.getHours();
    int minutes = timeClient.getMinutes();

    Serial.print(daysOfTheWeek[timeClient.getDay()]);
    Serial.print(", ");
    Serial.print(hour);
    Serial.print(":");
    Serial.print(minutes);
    Serial.print(":");
    Serial.println(timeClient.getSeconds());
    Serial.println(timeClient.getFormattedTime());
    displayValue = (hour * 100) + minutes;
    display.showNumberDecEx(displayValue, colon, true);
  }
}
