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
#include <EEPROM.h>
#include <EncoderStepCounter.h>
#include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`
#include "images.h"
#include <TimeLib.h>
#include "OLEDDisplayUi.h"
#include "wifi_credentials.h"

#define ENCODER_PIN1 D7
#define ENCODER_INT1 digitalPinToInterrupt(ENCODER_PIN1)
#define ENCODER_PIN2 D6
#define ENCODER_INT2 digitalPinToInterrupt(ENCODER_PIN2)

#define ALARM_HOURS_STORE 0
#define ALARM_MINUTE_STORE 1

// We have a status line for messages
#define STATUS_X 120 // Centred on this
#define STATUS_Y 65

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
// ===== OLED =====
String twoDigits(int digits);
void clockOverlay(OLEDDisplay *display, OLEDDisplayUiState *state);
void analogClockFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y);
void digitalClockFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y);
static uint8_t conv2d(const char *p);

// ==== END OLED ====

unsigned long startTime = 0;
unsigned long delayTime = 1000; // delay of 1000mS
unsigned long triggerTime;

int alarmHour;
int alarmMinute;

//========================OLED DISPALY ======================================

// OLED ===
SSD1306Wire display(0x3c, D2, D1);

OLEDDisplayUi ui(&display);

int screenW = 128;
int screenH = 64;
int clockCenterX = screenW / 2;
int clockCenterY = ((screenH - 16) / 2) + 16; // top yellow part is 16 px height
int clockRadius = 23;

// This array keeps function pointers to all frames
// frames are the single views that slide in
FrameCallback frames[] = {analogClockFrame, digitalClockFrame};

// how many frames are there?
int frameCount = 2;

// Overlays are statically drawn on top of a frame eg. a clock
OverlayCallback overlays[] = {clockOverlay};
int overlaysCount = 1;

//============================== END OLED ================================================

void setup()
{
  EEPROM.begin(10);
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
  alarmHour = EEPROM.read(ALARM_HOURS_STORE);
  alarmMinute = EEPROM.read(ALARM_MINUTE_STORE);
  Serial.print("Read hours (");
  Serial.print(alarmHour);
  Serial.print(") and minutes (");
  Serial.print(alarmMinute);
  Serial.println(") for the alarm ");
  Serial.print("Store size ");
  Serial.println(EEPROM.length());

  // ================ OLED =============
  //------------------------------oled-----=========================

  // The ESP is capable of rendering 60fps in 80Mhz mode
  // but that won't give you much time for anything else
  // run it in 160Mhz mode or just set it to 30 fps
  ui.setTargetFPS(10);

  // Customize the active and inactive symbol
 
  ui.disableAllIndicators();

  
  ui.disableAutoTransition();
  ui.transitionToFrame(1);
  // Add frames
  ui.setFrames(frames, frameCount);

  // Add overlays
  ui.setOverlays(overlays, overlaysCount);

  // Initialising the UI will init the display too.
  ui.init();

  display.flipScreenVertically();
}

// Call tick on every change interrupt
ICACHE_RAM_ATTR void interrupt()
{
  encoder.tick();
}

ulong lastAlarmUpdate;
bool notSaved;

void setAlarm(int clicks)
{
  if (clicks != 0)
  {
    ulong update = millis();
    notSaved = true;

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

  if (notSaved && millis() > lastAlarmUpdate + 20000)
  {
    // It has been 20 seconds since the alarm was set so save the new value
    Serial.print("Save hours (");
    Serial.print(alarmHour);
    Serial.print(") and minutes (");
    Serial.print(alarmMinute);
    Serial.println(") for the alarm ");
    notSaved = false;
    EEPROM.write(ALARM_HOURS_STORE, alarmHour);
    EEPROM.write(ALARM_MINUTE_STORE, alarmMinute);
    EEPROM.commit();
  }
}

void saveAlarmTime(int time)
{
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

  //-------oled-------

  int remainingTimeBudget = ui.update();

  if (remainingTimeBudget > 0)
  {
    // You can do some work here
    // Don't do stuff if you are below your
    // time budget.
    delay(remainingTimeBudget);
  }
}

void check_alarm(int hour, int minute)
{
  if (hour == alarmHour)
  {
    if (minute == alarmMinute || minute == alarmMinute + 1 || minute == alarmMinute + 2 || minute == alarmMinute + 3 || minute == alarmMinute + 4)
    {
      digitalWrite(led, LOW); //toggle the on-board led
      Serial.print("Alarm is sounding ");
    }
    else
    {
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

bool colon = false;
void clock_seven_segment_display()
{

  clockDisplay.setBrightness(0x0f);
  colon = !colon;

  if (isWifiConnected())
  {
    int hour = timeClient.getHours();
    int minutes = timeClient.getMinutes();
    int secs = timeClient.getSeconds();

    check_alarm(hour, minutes);

    setTime(hour, minutes, timeClient.getSeconds(), timeClient.getDay(), 1, 2019);

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

//======OLED=======

// utility function for digital clock display: prints leading 0
String twoDigits(int digits)
{
  if (digits < 10)
  {
    String i = '0' + String(digits);
    return i;
  }
  else
  {
    return String(digits);
  }
}

void clockOverlay(OLEDDisplay *display, OLEDDisplayUiState *state)
{
}

void analogClockFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
  //  ui.disableIndicator();

  // Draw the clock face
  //  display->drawCircle(clockCenterX + x, clockCenterY + y, clockRadius);
  display->drawCircle(clockCenterX + x, clockCenterY + y, 2);
  //
  //hour ticks
  for (int z = 0; z < 360; z = z + 30)
  {
    //Begin at 0° and stop at 360°
    float angle = z;
    angle = (angle / 57.29577951); //Convert degrees to radians
    int x2 = (clockCenterX + (sin(angle) * clockRadius));
    int y2 = (clockCenterY - (cos(angle) * clockRadius));
    int x3 = (clockCenterX + (sin(angle) * (clockRadius - (clockRadius / 8))));
    int y3 = (clockCenterY - (cos(angle) * (clockRadius - (clockRadius / 8))));
    display->drawLine(x2 + x, y2 + y, x3 + x, y3 + y);
  }

  // display second hand
  float angle = second() * 6;
  angle = (angle / 57.29577951); //Convert degrees to radians
  int x3 = (clockCenterX + (sin(angle) * (clockRadius - (clockRadius / 5))));
  int y3 = (clockCenterY - (cos(angle) * (clockRadius - (clockRadius / 5))));
  display->drawLine(clockCenterX + x, clockCenterY + y, x3 + x, y3 + y);
  //
  // display minute hand
  angle = minute() * 6;
  angle = (angle / 57.29577951); //Convert degrees to radians
  x3 = (clockCenterX + (sin(angle) * (clockRadius - (clockRadius / 4))));
  y3 = (clockCenterY - (cos(angle) * (clockRadius - (clockRadius / 4))));
  display->drawLine(clockCenterX + x, clockCenterY + y, x3 + x, y3 + y);
  //
  // display hour hand
  angle = hour() * 30 + int((minute() / 12) * 6);
  angle = (angle / 57.29577951); //Convert degrees to radians
  x3 = (clockCenterX + (sin(angle) * (clockRadius - (clockRadius / 2))));
  y3 = (clockCenterY - (cos(angle) * (clockRadius - (clockRadius / 2))));
  display->drawLine(clockCenterX + x, clockCenterY + y, x3 + x, y3 + y);

  // digital display
  String timenow = String(hour()) + ":" + twoDigits(minute()) + ":" + twoDigits(second());
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_16);
  display->drawString(clockCenterX + x, 0, timenow);
}

void digitalClockFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
  String timenow = String(hour()) + ":" + twoDigits(minute()) + ":" + twoDigits(second());
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_24);
  display->drawString(clockCenterX + x, clockCenterY + y, timenow);
}

// static uint8_t conv2d(const char *p)
// {
//   uint8_t v = 0;
//   if ('0' <= *p && *p <= '9')
//     v = *p - '0';
//   return 10 * v + *++p - '0';
// }
