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
#include <Time.h>
#include <Timezone.h>
#include "OLEDDisplayUi.h"
#include "wifi_credentials.h"
extern "C"
{
#include "user_interface.h"
}

os_timer_t myTimer;

bool soundAlarm = false;

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
//NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
// Try it without the timezone offset
NTPClient timeClient(ntpUDP, "pool.ntp.org");

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
//static uint8_t conv2d(const char *p);
time_t getLocalTime();

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
int clockCenterX = screenW / 2 - 32;
int clockCenterY = ((screenH - 16) / 2) + 16; // top yellow part is 16 px height
int clockRadius = 23;

int oledCenterX = screenW / 2;
int oledCenterY = ((screenH - 16) / 2) + 16; // top yellow part is 16 px height

// This array keeps function pointers to all frames
// frames are the single views that slide in
FrameCallback frames[] = {analogClockFrame, digitalClockFrame};

// how many frames are there?
int frameCount = 2;

// Overlays are statically drawn on top of a frame eg. a clock
OverlayCallback overlays[] = {clockOverlay};
int overlaysCount = 1;

// // This array keeps function pointers to all frames
// // frames are the single views that slide in
// FrameCallback frames[] = {analogClockFrame, digitalClockFrame};

// // how many frames are there?
// int frameCount = 2;

// // Overlays are statically drawn on top of a frame eg. a clock
// OverlayCallback overlays[] = {clockOverlay};
// int overlaysCount = 1;

// ================================= Date stuff =========================
//String date;
String t;
const char *days[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
const char *months[] = {"January", "February", "March", "April", "May", "June", "July", "August", "Sepember", "October", "November", "December"};

// timer interupt // start of timerCallback
void timerCallback(void *pArg)
{

  if (soundAlarm)
  {
    digitalWrite(led, !digitalRead(led)); //on-board led is the alarm
  }

} // End of timerCallback

void user_init(void)
{
  /*
  os_timer_setfn - Define a function to be called when the timer fires

void os_timer_setfn(
      os_timer_t *pTimer,
      os_timer_func_t *pFunction,
      void *pArg)

Define the callback function that will be called when the timer reaches zero. The pTimer parameters is a pointer to the timer control structure.

The pFunction parameters is a pointer to the callback function.

The pArg parameter is a value that will be passed into the called back function. The callback function should have the signature:
void (*functionName)(void *pArg)

The pArg parameter is the value registered with the callback function.
*/

  os_timer_setfn(&myTimer, timerCallback, NULL);

  /*
      os_timer_arm -  Enable a millisecond granularity timer.

void os_timer_arm(
      os_timer_t *pTimer,
      uint32_t milliseconds,
      bool repeat)

Arm a timer such that is starts ticking and fires when the clock reaches zero.

The pTimer parameter is a pointed to a timer control structure.
The milliseconds parameter is the duration of the timer measured in milliseconds. The repeat parameter is whether or not the timer will restart once it has reached zero.

*/

  os_timer_arm(&myTimer, 1, true);
} // End of user_init

//==================================================================================

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

  // You can change the transition that is used
  // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
  ui.setFrameAnimation(SLIDE_LEFT);

  // Add frames
  ui.setFrames(frames, frameCount);

  // Add overlays
  ui.setOverlays(overlays, overlaysCount);

  // Initialising the UI will init the display too.
  ui.init();

  // ============== Only use single screen on OLED ================
  ui.disableAutoTransition();
  ui.transitionToFrame(0);
  // Add frames

  display.flipScreenVertically();

  //digitalWrite(led, LOW); //on-board led is the alarm

  // timer
  user_init();
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

  yield();
}

void check_alarm(int hour, int minute)
{
  uint8_t alarmState = HIGH;
  if (hour == alarmHour && (minute == alarmMinute || minute == alarmMinute + 1 || minute == alarmMinute + 2 || minute == alarmMinute + 3 || minute == alarmMinute + 4))
  {
    alarmState = LOW;
    Serial.print("Alarm is sounding ");
    soundAlarm = true;
  }
  else
  {
    soundAlarm = false;
    digitalWrite(led, HIGH);
  }

  //digitalWrite(led, alarmState); //on-board led is the alarm
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

bool blankTillWiFi = true;
bool colon = false;
void clock_seven_segment_display()
{

  time_t local = getLocalTime();

  clockDisplay.setBrightness(0x0f);
  colon = !colon;

  int hourT = hour(local);
  int minutes = minute(local);
  //int secs = second(local) ;

  if (isWifiConnected())
  {
    blankTillWiFi = false;
    check_alarm(hourT, minutes);
    //setTime(hourT, minutes, timeClient.getSeconds(), timeClient.getDay(), 1, 2019);
    clockDisplay.showNumberDecEx(((hourT * 100) + minutes), colon ? 0xff : 0x00, true);
  }
  else
  {
    // if we just booted but haven't get got the correct time show dashes
    if (blankTillWiFi)
    {
      clockDisplay.setSegments(SEG_DASH);
    }
    else
    {
      // we did have the correct time but lost wiFi so keep displaying it, but stop flashing the colon
      colon = true;
      setTime(hourT, minutes, timeClient.getSeconds(), timeClient.getDay(), 1, 2019);
      clockDisplay.showNumberDecEx(((hourT * 100) + minutes), colon ? 0xff : 0x00, true);
    }
  }
}

int lastSeconds = 0;
void digitalClockFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
  // time_t local = getLocalTime();
  // //String timenow = String(hour()) + ":" + twoDigits(minute()) + ":" + twoDigits(second());

  // // now format the Time variables into strings with proper names for month, day etc
  // date = ""; // clear the variables
  // date += days[weekday(local) - 1];
  // date += ", ";
  // date += months[month(local) - 1];
  // date += " ";
  // date += day(local);
  // date += ", ";
  // date += year(local);

  // // format the time 24 hour clock
  // t = ""; // clear the time
  // if(hour(local) < 10)
  //   t += "0";
  // t += hour(local);
  // t += ":";
  // if (minute(local) < 10) // add a zero if minute is under 10
  //   t += "0";
  // t += minute(local);

  // display->setTextAlignment(TEXT_ALIGN_CENTER);
  // display->setFont(ArialMT_Plain_10);
  // display->drawString(oledCenterX + x, oledCenterY + y - 16, t);
  // display->drawString(oledCenterX + x, oledCenterY + y, date);

  // display->setTextAlignment(TEXT_ALIGN_CENTER);
  // display->setFont(ArialMT_Plain_16);
  // display->drawString(oledCenterX + x, 0, timenow);
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

time_t getLocalTime()
{

  unsigned long epochTime = timeClient.getEpochTime();

  // convert received time stamp to time_t object
  time_t local, utc;
  utc = epochTime;

  // Then convert the UTC UNIX timestamp to local time
  TimeChangeRule usPDT = {"PDT", Second, Sun, Mar, 2, -420}; //UTC - 7 hours - change this as needed
  TimeChangeRule usPST = {"PST", First, Sun, Nov, 2, -480};  //UTC - 8 hours - change this as needed
  Timezone usPacific(usPDT, usPST);
  local = usPacific.toLocal(utc);
  return local;
}

void analogClockFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{

  time_t local = getLocalTime();

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
  float angle = second(local) * 6;
  angle = (angle / 57.29577951); //Convert degrees to radians
  int x3 = (clockCenterX + (sin(angle) * (clockRadius - (clockRadius / 5))));
  int y3 = (clockCenterY - (cos(angle) * (clockRadius - (clockRadius / 5))));
  display->drawLine(clockCenterX + x, clockCenterY + y, x3 + x, y3 + y);
  //
  // display minute hand
  angle = minute(local) * 6;
  angle = (angle / 57.29577951); //Convert degrees to radians
  x3 = (clockCenterX + (sin(angle) * (clockRadius - (clockRadius / 4))));
  y3 = (clockCenterY - (cos(angle) * (clockRadius - (clockRadius / 4))));
  display->drawLine(clockCenterX + x, clockCenterY + y, x3 + x, y3 + y);
  //
  // display hour hand
  angle = hour(local) * 30 + int((minute() / 12) * 6);
  angle = (angle / 57.29577951); //Convert degrees to radians
  x3 = (clockCenterX + (sin(angle) * (clockRadius - (clockRadius / 2))));
  y3 = (clockCenterY - (cos(angle) * (clockRadius - (clockRadius / 2))));
  display->drawLine(clockCenterX + x, clockCenterY + y, x3 + x, y3 + y);

  

  // now format the Time variables into strings with proper names for month, day etc
  String displayWeekDay = days[weekday(local) - 1];
  String displayMonth = months[month(local) - 1];
  displayMonth += " " + day(local);
  String displayDate = "";
  displayDate += day(local);
  String displayYear = "";
  displayYear += year(local);

  // format the time 24 hour clock
  t = ""; // clear the time
  if (hour(local) < 10)
    t += "0";
  t += hour(local);
  t += ":";
  if (minute(local) < 10) // add a zero if minute is under 10
    t += "0";
  t += minute(local);
  t += ":";
  if (second(local) < 10)
    t += "0";
  t += second(local);

  // digital display
  //String timenow = String(hour()) + ":" + twoDigits(minute()) + ":" + twoDigits(second());
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_16);
  //display->drawString(oledCenterX + x, 0, timenow);
  // Yellow top bar
  display->drawString(oledCenterX + x - 20, oledCenterY + y - 40, t);
  display->drawString(oledCenterX + x + 34, oledCenterY + y - 40, displayDate);
  // blue body
  display->setFont(ArialMT_Plain_10);
  display->drawString(oledCenterX + x + 28, oledCenterY + y - 20, displayMonth);
  display->drawString(oledCenterX + x + 28, oledCenterY + y - 5, displayWeekDay);
  display->drawString(oledCenterX + x + 28, oledCenterY + y + 10, displayYear);
}
