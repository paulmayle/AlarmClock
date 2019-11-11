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

#define ENCODER_PIN1 D7
#define ENCODER_INT1 digitalPinToInterrupt(ENCODER_PIN1)
#define ENCODER_PIN2 D6
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
    SEG_D | SEG_E | SEG_F | SEG_G,                 // F
    SEG_D | SEG_E | SEG_F | SEG_G,                 // F
    0                                              //
};

const uint8_t SEG_DASH[] = {
    SEG_G,
    SEG_G,
    SEG_G,
    SEG_G};

TM1637Display clockDisplay(CLK, D3);
TM1637Display alarmDisplay(CLK, D5);

void connect_to_wifi();

void clock_seven_segment_display();
void alarm_seven_segment_display(uint hour, uint minute);
void setAlarm(int clicks);
bool isWifiConnected();
void check_alarm(int hour, int minute);
ICACHE_RAM_ATTR void interrupt();

unsigned long startTime = 0;
unsigned long delayTime = 1000; // delay of 1000mS
unsigned long triggerTime;

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
  unsigned long endTime = millis();
  triggerTime = endTime + delayTime; // set the next trigger time
  startTime = endTime;
  // This seems to CRASH it!!
 // digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
}

// Call tick on every change interrupt
ICACHE_RAM_ATTR void interrupt()
{
  encoder.tick();
}




int alarmHour = 7;
int alarmMinute = 0;
ulong lastAlarmUpdate;

void setAlarm(int clicks)
{
  if (clicks != 0)
  {
    ulong update = millis();

    if (lastAlarmUpdate + 25 > update)
    {
      // less than 25mS since the last update speed up the change
      clicks *= 15;
    }
    else
    {
      // no point in setting the alarm to the minute - nearest 5 mins will do
      clicks *= 5;
    }
    alarmMinute = (alarmMinute / 5) * 5; // make sure we are on a 5 minute boundry
    lastAlarmUpdate = update;            // store the update time for the speed of rotation timer
    Serial.print("clicks ");
    Serial.println(clicks);
    // just in case we get too many to keep the calculations simple and avoid the time changeing too much in a single increment we can limit it to 45 minute changes.
    if (clicks > 45)
    {
      clicks = 45;
    }
    if (clicks < -45)
    {
      clicks = -45;
    }
    alarmMinute += clicks;
    encoder.reset();
    if (alarmMinute > 59)
    {
      alarmMinute -= 60;
      if (++alarmHour > 23)
      {
        alarmHour = 0;
      }
    }
    if (alarmMinute < 0)
    {
      alarmMinute += 60;
      if (--alarmHour < 0)
      {
        alarmHour = 23;
      }
    }
    Serial.print("Alarm time ");
    Serial.print(alarmHour);
    Serial.print(":");
    Serial.println(alarmMinute);
  }
  alarm_seven_segment_display(alarmHour, alarmMinute);
}

void loop()
{
  signed char pos = encoder.getPosition();

  setAlarm(pos);
  

  unsigned long endTime = millis();
  if (endTime < startTime)
  {
    // timer has wrapped around
    triggerTime = endTime + delayTime; // set the next trigger time
    startTime = endTime;
  }
  if (endTime > triggerTime)
  {
    triggerTime = endTime + delayTime; // set the next trigger time
    startTime = endTime;
    clock_seven_segment_display();

   
  }
}

void check_alarm(int hour, int minute){
  if(hour==alarmHour){
    if(minute==alarmMinute||minute==alarmMinute+1||minute==alarmMinute+2||minute==alarmMinute+3||minute==alarmMinute+4){
    digitalWrite(led, LOW); //toggle the on-board led
     Serial.print("Alarm is sounding ");
    }else{
      digitalWrite(led, HIGH); //toggle the on-board led
    }
  }

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


bool colon=false;
void clock_seven_segment_display()
{
  
  clockDisplay.setBrightness(0x0f);
  colon=!colon;

  if (isWifiConnected())
  {
    int hour = timeClient.getHours();
    int minutes = timeClient.getMinutes();

    check_alarm(hour,minutes);

    // Serial.print(daysOfTheWeek[timeClient.getDay()]);
    // Serial.print(", ");
    // Serial.print(hour);
    // Serial.print(":");
    // Serial.print(minutes);
    // Serial.print(":");
    // Serial.println(timeClient.getSeconds());
    // Serial.println(timeClient.getFormattedTime());
  
    clockDisplay.showNumberDecEx(((hour * 100) + minutes), colon ? 0xff : 0x00, true);
  }
  else
  {
    clockDisplay.setSegments(SEG_DASH);
  }
}

void alarm_seven_segment_display(uint hour, uint minute)
{
  alarmDisplay.setBrightness(0xff);
  alarmDisplay.showNumberDecEx(((hour * 100) + minute), 0xff, true);
}

bool connected = false;
bool isWifiConnected()
{
  
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
    //Serial.println("WL_CONNECTED ");
    if (connected == false)
    {
      Serial.println("We have just (re) connected so force an update to the NTP time ");
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
  return connected;
}
