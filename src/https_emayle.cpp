
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <TM1637Display.h>
#include <pins_arduino.h>
#include <EEPROM.h>
#include <EncoderStepCounter.h>
#include "SSD1306Wire.h"
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

// Module connection pins (Digital Pins)
#define CLK D0
#define OLED_CLOCK D1
#define OLED_DATA D2
#define LED_CLOCK D3
#define LED_ONBOARD D4
#define LED_ALARM D5
#define ENCODER_PIN2 D6
#define ENCODER_PIN1 D7

#define ADC A0

#define ENCODER_INT1 digitalPinToInterrupt(ENCODER_PIN1)
#define ENCODER_INT2 digitalPinToInterrupt(ENCODER_PIN2)

// Store the alarm time in nvram (eeprom simulator)
#define ALARM_HOURS_STORE 0
#define ALARM_MINUTE_STORE 1
#define ALARM_STATE_STORE 2
#define BRIGHTNESS_STORE 3

// yellow part of oled uses first 16 pixels
#define STATUS_X 120 // Centred on this
#define STATUS_Y 65

#define BEEP_DELAY 15 // start alarm with a 15 second delay between beeps.

// Create instance for one full step encoder
EncoderStepCounter encoder(ENCODER_PIN1, ENCODER_PIN2);

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

/* Define the segments 

  AAAAAA
 F      B 
 F      B 
 F      B 
  GGGGGG
 E      C
 E      C
 E      C
  DDDDDD  
*/

const uint8_t SEG_BEEP[] = {
    SEG_C | SEG_D | SEG_E | SEG_F | SEG_G, // b
    SEG_A | SEG_D | SEG_E | SEG_F | SEG_G, // E
    SEG_A | SEG_D | SEG_E | SEG_F | SEG_G, // E
    SEG_A | SEG_B | SEG_E | SEG_F | SEG_G  // p
};

const uint8_t SEG_HOLD[] = {
    SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,         // H
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, // O
    SEG_D | SEG_E | SEG_F,                         // L
    SEG_B | SEG_C | SEG_D | SEG_E | SEG_G          // d
};

const uint8_t SEG_OFF[] = {
    0,                                             //
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, // O
    SEG_A | SEG_E | SEG_F | SEG_G,                 // F
    SEG_A | SEG_E | SEG_F | SEG_G                 // F
    
};

const uint8_t SEG_OFF_LJ[] = {
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, // O
    SEG_A | SEG_E | SEG_F | SEG_G,                 // F
    SEG_A | SEG_E | SEG_F | SEG_G,                 // F
    0                                              //
};

const uint8_t SEG_DASH[] = {
    SEG_G,
    SEG_G,
    SEG_G,
    SEG_G};

TM1637Display clockDisplay(CLK, LED_CLOCK);
TM1637Display alarmDisplay(CLK, LED_ALARM);

// ********  Prototypes *************************
void connect_to_wifi();

void clock_seven_segment_display();
void alarm_seven_segment_display(uint hour, uint minute);
void alarm_seven_segment_display_beep();
void setAlarm();
bool isWifiConnected();
void check_alarm(int hour, int minute);
ICACHE_RAM_ATTR void interrupt();
String twoDigits(int digits);
void clockOverlay(OLEDDisplay *display, OLEDDisplayUiState *state);
void dateTimeFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y);
void setupFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y);
void brightnessFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y);

void processMenuSelection();
void processBrightnessMenu();
void processMainMenu();
void checkForButtonPress();
boolean wasButtonPressed();
void exitMenu();
boolean isTimeToBeep(unsigned long delayAlarmTimeOff); // old version 
boolean isTimeToBeep();

time_t getLocalTime();
boolean isTimeToUpdateDisplay();
int getSelection(int max, int selection);
void proessMenuSelection();

//*****************************************************

unsigned long startTime = 0;
unsigned long delayTime = 1000; // delay of 1000mS

unsigned long startAlarmTime = 0;
unsigned long delayAlarmTimeOn = 50; // delay of 100mS
//unsigned long delayAlarmTimeOff = 1000; // delay of 100mS

int alarmHour;
int alarmMinute;

boolean snooze;
boolean inSetupMode;
int menuDisplayed = 0;
int ledBrightness;
boolean alarmOff;
boolean settingBrightness;
boolean buttonPressed;

enum menuState
{
  ALARM_OFF,
  BRIGHTNESS,
  EXIT
};

enum menu
{
  MAIN_MENU,
  BRIGHTNESS_MENU
};

//========================OLED DISPALY ======================================

// OLED ===
SSD1306Wire display(0x3c, OLED_DATA, OLED_CLOCK);

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
FrameCallback frames[] = {dateTimeFrame, setupFrame, brightnessFrame};

// how many frames are there?
int frameCount = 3;

// Overlays are statically drawn on top of a frame eg. a clock
OverlayCallback overlays[] = {clockOverlay};
int overlaysCount = 1;

// // This array keeps function pointers to all frames
// // frames are the single views that slide in
// FrameCallback frames[] = {dateTimeFrame, setupFrame};

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
const char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// timer interupt // start of timerCallback
void timerCallback(void *pArg)
{

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
  pinMode(LED_ONBOARD, OUTPUT);
  connect_to_wifi();

  alarmHour = EEPROM.read(ALARM_HOURS_STORE);
  alarmMinute = EEPROM.read(ALARM_MINUTE_STORE);
  alarmOff = EEPROM.read(ALARM_STATE_STORE);
  ledBrightness = EEPROM.read(BRIGHTNESS_STORE);
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

  // timer
  user_init();
}

int maxMenuSelection = 2;
int menuSelection = 0;
int count = BEEP_DELAY;
void loop()
{
  //-------oled-------
  int remainingTimeBudget = ui.update();
  if (remainingTimeBudget > 0)
  {
    // You can do some work here
    // Don't do stuff if you are below your
    // time budget.

    checkForButtonPress();
    if (wasButtonPressed())
    {

      // Button pressed
      if (inSetupMode)
      {
        processMenuSelection();
      }
      else
      {
        // if the alarm is sounding we can silence it
        if (soundAlarm)
        {
          snooze = true;
          soundAlarm = false;
        }
        else
        {
          inSetupMode = true;
          ui.transitionToFrame(1);
          maxMenuSelection = 2;
          menuSelection = 0;
        }
      }
    }
    //Serial.print("analogue: ");
    //Serial.println(v);
    // see if the user has turned the encoder to set the alarm

    if (inSetupMode)
    {
      // get the encoder selection of the menu item
      int tempSelection = menuSelection;
      menuSelection = getSelection(maxMenuSelection, menuSelection);
      if (tempSelection != menuSelection) // the selection has changed
      {
        switch (menuDisplayed)
        {
        case menu::MAIN_MENU:
          break;
        case menu::BRIGHTNESS_MENU:
          ledBrightness = menuSelection;
          alarm_seven_segment_display(alarmHour, alarmMinute); // so update the brightness
          clock_seven_segment_display();
          break;
        }
      }
    }
    else
    {
      // not in setup mode so just set the alarm time
      setAlarm();
    }

    // If a second has elapsed update the time on the 7 segment display
    if (isTimeToUpdateDisplay())
      clock_seven_segment_display();

    /*
  If the alarm has been triggered and we want to sound the alarm
  Check is time to beep to sound intermittently 
*/

    if (soundAlarm)
    {

      if (isTimeToBeep())
      {
        digitalWrite(LED_ONBOARD, LOW);
      }
      else
      {
        digitalWrite(LED_ONBOARD, HIGH);
      }
    }
    else
    {
      count = BEEP_DELAY;
      digitalWrite(LED_ONBOARD, HIGH);
    }

    // delay(remainingTimeBudget);
  }
}

boolean wasButtonPressed()
{
  if (buttonPressed)
  {
    buttonPressed = false;
    return true;
  }
  return false;
}

void checkForButtonPress()
{

  // read the ADC to determine which buttons are pressed
  int v = analogRead(ADC);
  if (v < 50)
  {
    delay(10);
    while (analogRead(ADC) < 50)
    {
      delay(10);
    }
    delay(50);
    // Button pressed
    buttonPressed = true;
  }
}

void processMenuSelection()
{

  switch (menuDisplayed)
  {
  case menu::MAIN_MENU:
    processMainMenu();
    break;

  case menu::BRIGHTNESS_MENU:
    processBrightnessMenu();
    break;
  }
}

/*
Called when button is pressed to save the new LRD brightness level
*/
void processBrightnessMenu()
{
  ledBrightness = menuSelection;
  menuDisplayed = menu::MAIN_MENU;
  inSetupMode = false;
  //menuDisplayed = menu::MAIN_MENU;
  ui.transitionToFrame(0);
  maxMenuSelection = 2;
  EEPROM.write(BRIGHTNESS_STORE, ledBrightness);
  EEPROM.commit();
}

void exitMenu()
{
  inSetupMode = false;
  menuDisplayed = menu::MAIN_MENU;
  ui.transitionToFrame(0);
}

void processMainMenu()
{
  switch (menuSelection)
  {
  case menuState::EXIT:
    //menuItem = 0;
    exitMenu();
    break;
  case menuState::BRIGHTNESS:
    ui.transitionToFrame(2);
    //  settingBrightness = true;
    maxMenuSelection = 7;
    menuSelection = ledBrightness;
    menuDisplayed = menu::BRIGHTNESS_MENU;

    break;
  case menuState::ALARM_OFF:
    alarmOff = !alarmOff;
    //  Serial.print("Alarm is ");
    // Serial.println(alarmOff ? "OFF" : "ON");
    alarm_seven_segment_display(alarmHour, alarmMinute);
    EEPROM.write(ALARM_STATE_STORE, alarmOff);
    EEPROM.commit();
    exitMenu();

    break;
  }
}

int getSelection(int max, int selection)
{
  signed char clicks = encoder.getPosition();

  if (clicks != 0)
  {
    Serial.print("Incoming selection ");
    Serial.println(selection);
    Serial.print("clicks ");
    Serial.println(clicks);
    encoder.reset();
    selection += clicks;
    if (selection < 0)
      selection = 0;
    if (selection > max)
      selection = max;

    Serial.print("Outgoing selection ");
    Serial.println(selection);
  }
  return selection;
}

// Call tick on every change interrupt
ICACHE_RAM_ATTR void interrupt()
{
  encoder.tick();
}

ulong lastAlarmUpdate;
bool notSaved;

void setAlarm()
{
  signed char clicks = encoder.getPosition();
  if (clicks != 0)
  {
    encoder.reset();
    if (alarmOff)
    {
      // change to active and save the state
      alarmOff = false;
      EEPROM.write(ALARM_STATE_STORE, alarmOff);
      EEPROM.commit();
    }

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
  if (soundAlarm)
  {
    alarm_seven_segment_display_beep();
  }
  else
  {
    alarm_seven_segment_display(alarmHour, alarmMinute);
  }

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

/**
 * This will return true once a second, so the colon flasshes at the correct rate.
 * */

boolean isTimeToUpdateDisplay()
{
  unsigned long currentTime = millis();
  if (currentTime < startTime)
  {
    // timer has wrapped around
    startTime = currentTime;
  }
  else
  {
    if (currentTime > startTime + delayTime)
    {
      startTime = currentTime;
      return true;
    }
  }
  return false;
}

/* 
This version replaced the version that takes a delay value
We already know that it is time to sound the alarm at this point - we are just deciding if we should shoind the beep - of is this the quiet time between beeps
To sound a 'humane' beep we start slow and build up
 */

boolean soundedThisPeriod=false;
boolean isTimeToBeep(){

  boolean soundTheAlarm=false;
  time_t local = getLocalTime();
  int hours = hour(local);
  int minutes = minute(local);
  int seconds = second(local);

  int timeSinceStart=minutes-alarmMinute;  // get the time that the alarm has been sounding from 0 to 5 minutes


  switch(timeSinceStart){
    // minites 1 and 2
      case 0 :
      case 1 :
      if(seconds>=0 && seconds <10){    // give ourselves a 10 second window to sound the alarm in case the wifi call is blocking 
        if(!soundedThisPeriod){
        soundTheAlarm=true;
      }
        soundedThisPeriod=true;
      }else{
        soundedThisPeriod=false;
      }
      break;
      // minute 3
      case 2 :
      if((seconds>=0 && seconds <10) || (seconds==15 && seconds <25)||(seconds==30 && seconds <40)|| (seconds==45 && seconds <55) ){
        if(!soundedThisPeriod){
        soundTheAlarm=true;
      }
        soundedThisPeriod=true;
      }else{
        soundedThisPeriod=false;
      }
      break;
      // minute 4
      case 3 :
       if((seconds % 5) ==0 ){    // sound every 5 seconds 
        if(!soundedThisPeriod){
        soundTheAlarm=true;
      }
        soundedThisPeriod=true;
      }else{
        soundedThisPeriod=false;
      }
      break;
      // minute 5 is the last chance to ramp it up!
      case 4 :

       if((seconds % 2) == 0 && seconds < 20  ){    // sound every 2 seconds . for 10 seconds
        if(!soundedThisPeriod){
        soundTheAlarm=true;
      }
        soundedThisPeriod=true;
      }else{
        soundedThisPeriod=false;
      }


     if(seconds >= 30 && seconds <=35 ){    // final attempt 5 seconds solid on
        soundTheAlarm=true;
      }else{
        soundedThisPeriod=false;
      }
     
      break;
      
    }
  
  return soundTheAlarm;

}

boolean isTimeToBeep(unsigned long delayAlarmTimeOff)
{

  

  unsigned long currentTime = millis();
  if (currentTime < startAlarmTime)
  {
    // timer has wrapped around
    startAlarmTime = currentTime;
  }
  else
  {
    if (count == BEEP_DELAY)
    {
      --count;
      startAlarmTime = currentTime;
      return true;
    }

    if (currentTime > startAlarmTime + (delayAlarmTimeOff * count))
    {
      if (--count < 1)
        count = 1;
      startAlarmTime = currentTime;
      return true;
    }
  }
  return false;
}

/*
Should be called every second from the depilay time update
*/
void check_alarm(int hour, int minute)
{
  if (hour == alarmHour && (minute == alarmMinute || minute == alarmMinute + 1 || minute == alarmMinute + 2 || minute == alarmMinute + 3 || minute == alarmMinute + 4))
  {
    if (!snooze && !alarmOff)
    // set the sound alarm flag. This doesn't mean the alarm will acutally sound, just that it is within the 5 minute sound period
      soundAlarm = true;
  }
  else
  {
    snooze = false;
    soundAlarm = false;
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


/**
 * Called every second to update the display and check the alarm
 *
 */

bool blankTillWiFi = true;
bool colon = false;
void clock_seven_segment_display()
{

  time_t local = getLocalTime();

  if (soundAlarm)
  {
    clockDisplay.setBrightness(7);
  }
  else
  {
    clockDisplay.setBrightness(ledBrightness);
  }

  colon = !colon;

  int hourT = hour(local);
  int minutes = minute(local);
 

  bool showLeadingZero=(hourT==0);

  if (isWifiConnected())
  {
    blankTillWiFi = false;
    check_alarm(hourT, minutes);
    clockDisplay.showNumberDecEx(((hourT * 100) + minutes), colon ? 0xff : 0x00, showLeadingZero);
  }
  else
  {
    // if we just booted but haven't get got the correct time show dashes
    if (blankTillWiFi)
    {
      clockDisplay.setSegments(SEG_HOLD);
    }
    else
    {
      // we did have the correct time but lost wiFi so keep displaying it, but stop flashing the colon
      setTime(hourT, minutes, timeClient.getSeconds(), timeClient.getDay(), 1, 2019);
      clockDisplay.showNumberDecEx(((hourT * 100) + minutes), 0xff, showLeadingZero);
    }
  }
}

int lastSeconds = 0;

/*

This is the OLED Setup screen 



*/

void alarm_seven_segment_display(uint hour, uint minute)
{
  alarmDisplay.setBrightness(ledBrightness);
  bool showLeadingZero=(hour==0);
  if (!alarmOff)
  {
    alarmDisplay.showNumberDecEx(((hour * 100) + minute), 0xff, showLeadingZero);
  }
  else
  {
    alarmDisplay.setSegments(SEG_OFF);
  }
}

void alarm_seven_segment_display_beep()
{
  alarmDisplay.setBrightness(7);
  if (colon)
  {
    alarmDisplay.setSegments(SEG_BEEP);
  }
  else
  {
    alarmDisplay.clear();
  }
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
  //Serial.println("overlay called");

  //display->drawCircle(clockCenterX , clockCenterY , 9);
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

void setupFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{

  //Serial.println("Setup called");
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_16);
  if (alarmOff)
  {
    display->drawString(0 + x, 0 + y, " Alarm ON ");
  }
  else
  {
    display->drawString(0 + x, 0 + y, " Alarm OFF ");
  }
  display->drawString(0 + x, 20 + y, " Brightness ");
  display->drawString(0 + x, 40 + y, " Exit");

  /*
  Draw the box around the selected menu item
*/

  int x2 = 0;
  int y2 = 0;
  int w = 100;
  int h = 17;

  switch (menuSelection)
  {
  case 0:
    x2 = 0;
    y2 = 0;
    break;
  case 1:
    x2 = 0;
    y2 = 20;
    break;
  case 2:
    x2 = 0;
    y2 = 40;
    break;
  }
  display->drawRect(x2, y2, w, h);
}

void brightnessFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{

  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_16);
  display->drawString(5 + x, 20 + y, "1");
  display->drawString(25 + x, 20 + y, "2");
  display->drawString(45 + x, 20 + y, "3");
  display->drawString(65 + x, 20 + y, "4");
  display->drawString(5 + x, 40 + y, "5");
  display->drawString(25 + x, 40 + y, "6");
  display->drawString(45 + x, 40 + y, "7");
  display->drawString(65 + x, 40 + y, "8");
  display->drawString(0 + x, 0 + y, "Brightness");

  /*
  Draw the box around the selected menu item
*/

  int x2 = 0;
  int y2 = 0;
  int w = 20;
  int h = 20;

  switch (menuSelection)
  {
  case 0:
    x2 = 0;
    y2 = 20;
    break;
  case 1:
    x2 = 20;
    y2 = 20;
    break;
  case 2:
    x2 = 40;
    y2 = 20;
    break;
  case 3:
    x2 = 60;
    y2 = 20;
    break;

    // row 2
  case 4:
    x2 = 0;
    y2 = 40;
    break;
  case 5:
    x2 = 20;
    y2 = 40;
    break;
  case 6:
    x2 = 40;
    y2 = 40;
    break;
  case 7:
    x2 = 60;
    y2 = 40;
    break;
  }
  display->drawRect(x2, y2, w, h);
}

void dateTimeFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
  //Serial.println("clock display called");

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

  //hour ticks
  for (int z = 0; z < 360; z = z + 90)
  {
    //Begin at 0° and stop at 360°
    float angle = z;
    angle = (angle / 57.29577951); //Convert degrees to radians
    int x2 = (clockCenterX + (sin(angle) * clockRadius));
    int y2 = (clockCenterY - (cos(angle) * clockRadius));
    int x3 = (clockCenterX + (sin(angle) * (clockRadius - (clockRadius / 4))));
    int y3 = (clockCenterY - (cos(angle) * (clockRadius - (clockRadius / 4))));
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
