// GEO Cookstove Sensor Code
// Respira Bien 2022-2023
// Arduino code for the cookstove sensor
// This file contains setup() and loop(), as well as functions for initializing the sensors

#include <Arduino.h>
#include <U8g2lib.h>
#include <SD.h>       // Used to read/write files to the SD card
#include <SPI.h>      // Used to communicate with the SD card reader
#include <RTClib.h>   // Library for the real time clock
#include <String.h>   // Required to use string data type
#include "PM.h"       // Header file with PM functions/data
#include "CO2.h"      // Header file with CO2 functions/data
#include "CO.h"       // Header file with CO functions/data
#include <LowPower.h> // Library to enable sleep mode
#include <String.h>

/*
 * Pin configuration:
 * GND -> GND
 * VCC -> 5V
 * SCK -> D13
 * SDA -> D11
 * RES -> D8
 * DC  -> D9
 * CS  -> D10
 */

U8G2_SSD1309_128X64_NONAME0_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);

// PIN NUMBERS
#define RED 24        // Pin number for Red in RGB LED
#define GREEN 26      // Pin number for Green in RGB LED
#define BLUE 22       // Pin number for Blue in RGB LED
#define FAN 6         // Pin number for MOSFET (controls power to the fan)
#define CS 53         // Pin number for SD card reader
#define INTERRUPT 18  // Pin number for the RTC interrupt
#define SELECT 2      // Pin number for Select Button
#define TOGGLE 3      // Pin number for Toggle Button

// OBJECTS
RTC_PCF8523 rtc;      // Real time clock object
PM pm;
CO2 co2;
CO co;
File myFile;          


// GLOBAL VARIABLES
String fileName = "datos.csv";
String pm2_5;
String pm10;
double ppmCO = 0;
double ppmCO2 = 0;
double co2raw = 0;
double coraw = 0;
volatile bool togglePushed = false;
volatile bool selectPushed = false;
volatile bool menuArrowState = false;
volatile int menuArrowPos = 25;
volatile bool measureArrowState = false;
volatile int measureArrowPos = 65;
volatile int optionsArrowPos = 20;
volatile int waitDebounce = 0;
volatile int measureTime = 150; //Every 12 is one second


// Calibrated parameters for mapping the CO and CO2 values 
double calibratedInterceptCO = -431.29;
double calibratedSlopeCO = 30.036;
double calibratedInterceptCO2 = -254.05;
double calibratedSlopeCO2 = 2.1045;

// Enumeration of state variable for the state machine
enum state {menu, wait, measure, record, error, options};
enum state currentState;

void setup(void) 
{
  // Setup Terminal
  Serial.begin(9600);
  Serial3.begin(9600);  // Serial3 will be used to communicate with the PM sensor
  Serial.println("Respira Bien Cookstove Sensor");
  Serial.println();

  // Setup text
  u8g2.begin();
  u8g2.setFont(u8g2_font_tinytim_tr);  // choose a suitable font
  // Find fonts from: https://github.com/olikraus/u8g2/wiki/fntlistallplain#u8g2-font-list

  // Start Screen 
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_8x13_mr);  // choose a suitable font
  u8g2.drawStr(20,25,"Bienvenidos");  // write something to the internal memory
  u8g2.sendBuffer();          // transfer internal memory to the display 
  delay(2000);
  u8g2.clearBuffer();
  u8g2.sendBuffer();

  // Setup buttons
  pinMode(SELECT, INPUT);
  pinMode(TOGGLE, INPUT);
  pinMode(INTERRUPT, INPUT_PULLUP); // Initialize the interrupt pin with the internal pull up resistor in the Arduino
  pinMode(FAN, OUTPUT);             // Configures MOSFET pin as OUTPUT
  pinMode(RED, OUTPUT);             // Configures RED pin as OUTPUT
  pinMode(BLUE, OUTPUT);            // Configures BLUE pin as OUTPUT
  pinMode(GREEN, OUTPUT);           // Configures GREEN pin as OUTPUT
  digitalWrite(FAN, LOW);           // Sets MOSFET pin to LOW

//  attachInterrupt(digitalPinToInterrupt(INTERRUPT), isr, FALLING);  // Connects the RTC SQW pin to the MCU interrupt pin
  attachInterrupt(digitalPinToInterrupt(SELECT), isrSelect, FALLING); 
  attachInterrupt(digitalPinToInterrupt(TOGGLE), isrToggle, FALLING);

  initSensors(false, true, true, true, true);
  Serial.println("Sensors initialized successfully");
  currentState = menu;

  digitalWrite(FAN, HIGH);

//  String dateDay = (char*)rtc.now().day();
//  String dateMonth = (char*)rtc.now().month();
//  String dateYear = (char*)rtc.now().year();
//  String dateHour = (char*)rtc.now().hour();
//  String dateMin = (char*)rtc.now().minute();
//  String dateSec = (char*)rtc.now().second();
//
//  fileName = dateDay + '_' + dateMonth + '_' + dateYear + '_' + dateHour + '_' + dateMin + '_' + dateSec;

  bool makeHeader = printHeader();
  if (makeHeader) { Serial.println("made header for file"); }
  else { Serial.println("failed to make header for file"); }
}

void loop(void) 
{
  static int fanTimer = 0;
  static bool writeSuccess = false;
  static bool SDCardSuccess = false;
//  togglePushed = false;
//  selectPushed = false;
  
  // Perform state update (Mealy).
  switch(currentState) 
  { 
    case menu:
      //Serial.println("Menu");
      if (selectPushed) 
      {
        if (menuArrowState) 
        { 
          currentState = options; 
          clearScreen();
        }
        else 
        { 
          currentState = measure;
          LED_GREEN();
          waitDebounce = 0; 
          clearScreen();
        }
        selectPushed = false;
      }
      break;
    case wait:
      //Serial.println("Wait");
      if (selectPushed) 
      {
        if (measureArrowState) 
        { 
          LED_OFF();
          currentState = menu;
          measureArrowPos = 65;
          measureArrowState = false;
          clearScreen();
        }
        else {}
        selectPushed = false;
      }
      else if (waitDebounce > measureTime) 
      {
        LED_OFF();
        waitDebounce = 0;
        currentState = measure;
      }
      else 
      {
        currentState = wait;
      }
      break;
    case measure:
      //Serial.println("Measure");
      if (selectPushed) 
      {
        if (measureArrowState) 
        { 
          currentState = menu;
          measureArrowPos = 65;
          measureArrowState = false;
          clearScreen();
        }
        else {}
        selectPushed = false;
      }
      else 
      {
        currentState = record; 
      }
      break;
    case record:
      //Serial.println("Record");
      if (selectPushed) 
      {
        if (measureArrowState) 
        { 
          currentState = menu;
          measureArrowPos = 65;
          measureArrowState = false;
          clearScreen();
        }
        else {}
        selectPushed = false;
      }
      if (writeSuccess) {
        LED_GREEN();
        currentState = wait;
      }
      else {
        clearScreen();
        currentState = error;
      }
      break;
    case error:
      if (SDCardSuccess && printHeader()) {
        clearScreen();
        LED_OFF();
        currentState = measure;
      }
      break;
    case options:
//      if (selectPushed) 
//      {
//        selectPushed = false;
//        if (optionsArrowPos == 85) 
//        { 
//          Serial.println("Enter Menu State");
//          optionsArrowPos = 20;
//          menuArrowPos = 25;
//          clearScreen();
//          currentState = menu; 
//        }
//      }
//      break;
    default:
      break;
  }

  // Perform state action (Moore).
  switch(currentState) 
  { 
    case menu:
      //Serial.println("State: Menu");
      printMenuScreen(menuArrowPos);
      if (togglePushed) 
      {
        if (menuArrowState) 
        { 
          menuArrowState = false; 
          menuArrowPos = 25;
        }
        else 
        { 
          menuArrowState = true;
          menuArrowPos = 45;
        }
        togglePushed = false;
      }
      break;
    case wait:
      printMeasureScreen(measureArrowPos, ppmCO, ppmCO2, "0");//pm2_5);
      measureWaitButtons();
      waitDebounce++;
      break;
    case measure:
      measureWaitButtons();
      //pm.measure();         // PM sensor reads measurements, variables in PM library will contain the results
      //pm2_5 = pm.pm2_5;     // Concentration of PM that is 2.5micrometers
      //pm10 = pm.pm10;       // Concentration of PM that is 10micrometers
      co2raw = co2.measure();
      coraw = co.measure();
      ppmCO = (coraw * calibratedSlopeCO) + calibratedInterceptCO;           // Concentration of CO in parts per million
      ppmCO2 = (co2raw * calibratedSlopeCO2) + calibratedInterceptCO2;   // Concentration of CO2 in parts per million
//      Serial.println("CO2 Raw main file:");
//      Serial.println(co2raw, DEC);
//      Serial.println("CO2 main file:");
//      Serial.println(ppmCO2, DEC);
      printMeasureScreen(measureArrowPos, ppmCO, ppmCO2, "0");//pm2_5);
      break;
    case record:
      printMeasureScreen(measureArrowPos, ppmCO, ppmCO2, "0");//pm2_5);
      measureWaitButtons();
      writeSuccess = writeToFile(rtc.now(), ppmCO, coraw, ppmCO2, co2raw, "0","0");//pm2_5, pm10);
      break;
    case error:
      LED_RED();
      printError();
      SDCardSuccess = SD.begin();
      break;
    case options:
      printOptions(optionsArrowPos);
      optionsButtons();
      break;
    default:
      LED_MAGENTA();
      break;
  }
}

void measureWaitButtons() 
{
  if (togglePushed) 
  {
    if (measureArrowState) 
    { 
      measureArrowState = false; 
      measureArrowPos = 65;
    }
    else 
    { 
      measureArrowState = true;
      measureArrowPos = 85;
    }
    togglePushed = false;
  }
}

void optionsButtons() 
{
  if (togglePushed) 
      {
        togglePushed = false;
        if (optionsArrowPos == 20) { optionsArrowPos = 55; }
        else if (optionsArrowPos == 55) { optionsArrowPos = 92; }
        else if (optionsArrowPos == 92) { optionsArrowPos = 65; }
        else if (optionsArrowPos == 65) { optionsArrowPos = 85; }
        else if (optionsArrowPos == 85) { optionsArrowPos = 20; }
      }
  if (selectPushed) 
      {
        selectPushed = false;
        if (optionsArrowPos == 20) { measureTime = 150; Serial.println("10 sec");}
        else if (optionsArrowPos == 55) { measureTime = (150*3) + (15*5); Serial.println("30 sec");}
        else if (optionsArrowPos == 92) { measureTime = (150*6) + (15*12); Serial.println("1 min");}
        else if (optionsArrowPos == 85) 
        { 
          Serial.println("Enter Menu State");
          optionsArrowPos = 20;
          menuArrowPos = 25;
          menuArrowState = false;
          clearScreen();
          currentState = menu; 
        }
      }
}

// Initializes all of the sensors
void initSensors(bool pmInit, bool coInit, bool co2Init, bool rtcInit, bool sdInit) 
{
  Serial.println("sdInit");
  bool sd = 0;
  if (sdInit) 
  {
    sd = SD.begin();
  }
  Serial.println(sd);
  Serial.println("pmInit");
  if (pmInit) 
  {
    pm.reset_measurement();               // Opens communication with the PM sensor
  }
  Serial.println("co2Init");
  if (co2Init) 
  {
    co2.scd30.begin();                    // Intializes CO2 sensor
    co2.scd30.setMeasurementInterval(1);  // Fastest communication time with CO2 Sensor
  }
  Serial.println("coInit");
  if (coInit) 
  {
    
  }
  Serial.println("rtcInit");
  if (rtcInit) 
  {
    rtc.begin();  // Initializes real time clock, uses RTC library
    if (!rtc.isrunning()) 
    {
      Serial.println("RTC is not running! Setting __DATE__ and __TIME__ to the date and time of last compile.");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Will set the real time clock to the time from the computer system
    }
    
    // If you wish to reset the date and time to the time of 
    // last compile, uncomment this line of code. Otherwise,
    // It will only get reset if the RTC stops running e.g.
    // the battery dies, etc. 
    
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); 
  }
}

/*
 * Interrupt service routine:
 * 
 * When the attachInterrupt() function is called in the sleep state, 
 * this function is specified as the interrupt service routine (isr).
 * This means that when the RTC sends the interrupt signal, it calls 
 * this function right when the MCU wakes up and then continues 
 * execution where it left off (in the sleep state action).
 * 
 */
void isr() 
{
  Serial.println("MCU is now awake");
}

void isrToggle() 
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200) 
  {
    Serial.println("Pressed Toggle Button");
    togglePushed = true;
  }
  last_interrupt_time = interrupt_time;
  
}

void isrSelect() 
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200) 
  {
    Serial.println("Pressed Select Button");
    selectPushed = true;
  }
  last_interrupt_time = interrupt_time;
}
