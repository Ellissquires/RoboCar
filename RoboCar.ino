// RoboThing

#include <WiFi.h>
#include <aREST.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "IOExpander.h"
#include <Adafruit_GFX.h>       // core graphics library
#include "Adafruit_HX8357.h"    // tft display local hacked version
#include "Adafruit_STMPE610.h"  // touch screen local hacked version

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
 
// And connect 2 DC motors to port M3 & M4 !
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(4);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(3);

// Create aREST instance
aREST rest = aREST();

// The port to listen for incoming TCP connections 
#define LISTEN_PORT           80

// Create an instance of the server
WiFiServer server(LISTEN_PORT);

// Movement functions
int stop(String message);
int forward(String message);
int right(String message);
int left(String message);
int backward(String message);

// Battery Management settings
byte BM_I2Cadd = 0x6b;
byte BM_Watchdog = 0x05; // Charge Termination/Timer Control Register
byte BM_OpCon    = 0x07; // Misc Operation Control Register
byte BM_Status   = 0x08; // System Status Register 
byte BM_Version  = 0x0a; // Vender / Part / Revision Status Register 
bool DBG = false; // debug switch
#define D(args...)   { if(DBG) { Serial.printf(args); Serial.println(); } }
class Chunk {
public:
  int begin();  // initialisation
  void test();  // validation
};

// Screen display
// the LCD and touch screen
#define TFT_DC   33
Adafruit_HX8357 tft =  Adafruit_HX8357(IOExpander::LCD_CS, TFT_DC, IOExpander::LCD_RESET);
Adafruit_STMPE610 ts = Adafruit_STMPE610(IOExpander::TOUCH_CS);
// calibration data for converting raw touch data to the screen coordinates
#define TS_MINX 3800
#define TS_MAXX 100
#define TS_MINY 100
#define TS_MAXY 3750
// a test set of boxes
class Menu: public Chunk {
public:
  void init() {
    tft.setTextSize(3);
    tft.setCursor(65, 110);
    tft.setTextColor(HX8357_BLACK);
    tft.print("Screen Test");
    tft.setCursor(120, 170);
    tft.setTextColor(HX8357_RED);
    tft.print("Red");  
    tft.setCursor(120, 200);
    tft.setTextColor(HX8357_GREEN);
    tft.print("Green");
    tft.setCursor(120, 230);
    tft.setTextColor(HX8357_BLUE);
    tft.print("Blue");
    tft.fillRect(110, 320, 100, 40 , HX8357_BLACK);
    tft.setCursor(120, 330);
    tft.setTextColor(HX8357_WHITE);
    tft.print("NEXT!");
    tft.fillRect(30, 30, 70, 70 , HX8357_GREEN);
    tft.fillRect(220, 30, 70, 70 , HX8357_RED);
    tft.fillRect(30, 380, 70, 70 , HX8357_BLUE);
    tft.fillRect(220, 380, 70, 70 , HX8357_CYAN);
  }
  
  void fail() {
    tft.fillScreen(HX8357_WHITE);
    tft.setTextSize(3);
    tft.setCursor(65, 110);
    tft.setTextColor(HX8357_BLUE);
    tft.print("!!  FAIL  !!");
    tft.setCursor(65, 250);
  }
};
Menu m;

void setup() {
  Wire.setClock(100000); // TODO higher rates trigger an IOExpander bug
  Wire.begin();  
  IOExpander::begin();
  // Start Serial
  Serial.begin(115200);
  IOExpander::digitalWrite(IOExpander::BACKLIGHT, LOW);
  tft.begin(HX8357D);
  tft.fillScreen(HX8357_WHITE);
  m.init();
  IOExpander::digitalWrite(IOExpander::BACKLIGHT, HIGH);

  if(! ts.begin()) {
    D("failed to start touchscreen controller");
    m.fail();
    tft.println("TOUCH");
    delay(3000);
  } else {
    D("touchscreen started");
  }
  
  // Power Switch Stuff
  
  uint8_t inputPwrSw = IOExpander::digitalRead(IOExpander::POWER_SWITCH);

  bool powerGood = // bit 2 of status register indicates if USB connected
    bitRead(getRegister(BM_I2Cadd, BM_Status), 2);

  if(!inputPwrSw) {  // when power switch off
    if(!powerGood) { // and usb unplugged we go into shipping mode
      setShipping(true);
    } else { // power switch off and usb plugged in we sleep
      esp_sleep_enable_timer_wakeup(1000000); // sleep time is in uSec
      esp_deep_sleep_start();
    }
  }

  
  // Init motor shield
  AFMS.begin();  

  // Functions          
  rest.function("stop", stop);
  rest.function("forward", forward);
  rest.function("left", left);
  rest.function("right", right);
  rest.function("backward", backward);
      
  // Give name and ID to device
  rest.set_id("1");
  rest.set_name("robot");
  
  // Connect to WiFi
  WiFi.begin("PLUSNET-QNS3","432b2a3c37");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
 
  // Start the server
  server.begin();
  Serial.println("Server started");
  
  // Print the IP address
  Serial.println(WiFi.localIP());
  
}

void loop() {
  uint8_t inputPwrSw = IOExpander::digitalRead(IOExpander::POWER_SWITCH);

  bool powerGood = // bit 2 of status register indicates if USB connected
    bitRead(getRegister(BM_I2Cadd, BM_Status), 2);

  if(!inputPwrSw) {  // when power switch off...
    if(!powerGood) { // and usb unplugged we go into shipping mode
      setShipping(true);
    } else { // power switch is off and usb plugged in: we sleep
      esp_sleep_enable_timer_wakeup(1000000); // sleep time is in uSec
      esp_deep_sleep_start();
    }
  }
  
  // Handle REST calls
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
  while(!client.available()){
    delay(1);
  }
  rest.handle(client);
}

int stop(String command) {
  D("Stop boi");
  L_MOTOR->setSpeed(0);
  L_MOTOR->run( RELEASE );
 
  R_MOTOR->setSpeed(0);
  R_MOTOR->run( RELEASE );
}

int forward(String command) {
  D("Move boi");
  L_MOTOR->setSpeed(255);
  L_MOTOR->run( FORWARD );
 
  R_MOTOR->setSpeed(255);
  R_MOTOR->run( FORWARD );
}

int left(String command) {
  D("Left boi");
  L_MOTOR->setSpeed(100);
  L_MOTOR->run( BACKWARD );
 
  R_MOTOR->setSpeed(100);
  R_MOTOR->run( FORWARD );
}

int right(String command) {
  D("Right boi");
  L_MOTOR->setSpeed(100);
  L_MOTOR->run( FORWARD );
 
  R_MOTOR->setSpeed(100);
  R_MOTOR->run( BACKWARD );
}

int backward(String command) {
  D("Back boi");
  L_MOTOR->setSpeed(150);
  L_MOTOR->run( BACKWARD );
 
  R_MOTOR->setSpeed(150);
  R_MOTOR->run( BACKWARD );
}

void setShipping(bool value) {
  byte result;
  if(value) {
    result=getRegister(BM_I2Cadd, BM_Watchdog);  // state of timing register
    bitClear(result, 5);                         // clear bit 5...
    bitClear(result, 4);                         // and bit 4 to disable...
    setRegister(BM_I2Cadd, BM_Watchdog, result); // WDT (REG05[5:4] = 00)

    result=getRegister(BM_I2Cadd, BM_OpCon);     // operational register
    bitSet(result, 5);                           // set bit 5 to disable...
    setRegister(BM_I2Cadd, BM_OpCon, result);    // BATFET (REG07[5] = 1)
  } else {
    result=getRegister(BM_I2Cadd, BM_Watchdog);  // state of timing register
    bitClear(result, 5);                         // clear bit 5...
    bitSet(result, 4);                           // and set bit 4 to enable...
    setRegister(BM_I2Cadd, BM_Watchdog, result); // WDT (REG05[5:4] = 01)

    result=getRegister(BM_I2Cadd, BM_OpCon);     // operational register
    bitClear(result, 5);                         // clear bit 5 to enable...
    setRegister(BM_I2Cadd, BM_OpCon, result);    // BATFET (REG07[5] = 0)
  }
}

void setRegister(byte address, byte reg, byte value) {
  write8(address, reg, value);
}

void write8(byte address, byte reg, byte value) {
  Wire.beginTransmission(address);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();
}

byte read8(byte address, byte reg) {
  byte value;
  Wire.beginTransmission(address);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();
  return value;
}

byte getRegister(byte address, byte reg) {
  byte result;
  result=read8(address, reg);
  return result;
}
