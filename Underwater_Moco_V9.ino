// Motion control and general testing of multiple motors
// This is the main sourcecode to test CANbus control of the MyActuator RMD-X series with V???? firmware
// Uses Arduino Uno with the Sparkfun CAN-BUS shield
//
// National Geographic Photo Engineering
//
// Matt Norman
// May 20, 2022

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <mcp_can.h>
#include "EmojisHANC.h"

// SAMD core //
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL SerialUSB
#else
#define SERIAL Serial
#endif

//# defines and constants //
#define CONTROL_PERIOD 10         // in MS
#define SCREEN_REFRESH_PERIOD 50  // in MS
#define DIAL_PERIOD 250
#define NOISE_TOLERANCE_JOYSTICK 5
#define NOISE_TOLERANCE_DIAL 15
#define MAX_ACCELERATION 10000  // Max acceleration parameter 1 dps/s range 50 - 80000
#define MAX_SPEED 5000          // Max speed parameter 1 dps/LSB -/+ 32767
#define MAX_TORQUE 500          // Max torque parameter 1 dps/LSB -/+ 32767
#define SCREEN_WIDTH 128        // OLED display width, in pixels
#define SCREEN_HEIGHT 64        // OLED display height, in pixels
#define OLED_RESET -1           // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C     ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define TILT_ID 0x142           // Defult ID 0x141 + 1, reset in GUI
#define PAN_ID 0x143            // Defult ID 0x141 + 1, Reset in GUI
#define XTR1 0x141              // Defult ID 0x141 + 1, reset in GUI
#define XTR2 0x144              // Defult ID 0x141 + 1, Reset in GUI
#define XTR3 0x145              // Defult ID 0x141 + 1, reset in GUI
#define DIAL_INDEX 128          //1023/8 = ~128
#define SENSATIVITY_CONST 4     //used in control loop to adjust system sensativity
#define PROFILE_CONST 0.25      ////used in control loop to adjust system responce to input
unsigned long controlTimer = 0;
unsigned long screenRefreshTimer = 0;
unsigned long currentTime = 0;
const int panPin = A0;
const int tiltPin = A1;
const int profilePin = A2;
const int sensePin = A3;
const int SPI_CS_PIN = 10;  // cs pin = 10 for Uno = 53 for Mega //

// obj prototypes

MCP_CAN CAN(SPI_CS_PIN);  // Set CS pin //

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// func prototypes //

double mapVal(double, double, double, double, double);
double mapProfile(double, double, double, double);
const uint8_t* stufEmoji(uint8_t);
int sendReceiveBuffer(uint16_t, unsigned char*, unsigned char*);
uint8_t checkInput();
uint8_t samplePin(uint8_t, int32_t*);
int32_t checkSoftStop(uint16_t, double, int32_t, int32_t, int32_t, double, double);
int32_t readPosition(uint16_t);
int32_t readAcceleration(uint16_t);
int32_t homeMotor(uint16_t);
int32_t readErrorState(uint16_t);
void initalizeMotor(uint16_t);
void brakeOffMotor(uint16_t);
void printBuffer(unsigned char*);
uint32_t readFirmwareVersion(uint16_t);
void resetMotor(uint16_t);
void runMotor(uint16_t);
void setSpeed(uint16_t, uint32_t);
void setTorque(uint16_t, uint32_t);
void setAcceleration(uint16_t, uint32_t);
void setupOLED(uint8_t);
void spdCommand(uint16_t, uint32_t);
void stopMotor(uint16_t);
void zeroMotor(uint16_t);

// global variables //
unsigned char len = 0;
unsigned char buf[8];
uint32_t reductionRatio = 36;  //gear box reduction 1:36 for RMD X6
uint16_t msgID = 0x140;        //140+ID
int32_t panLock = 135;         //Pan Axis movment stops in deg
int32_t tiltLock = 90;         //Tilt Axis movment stops in deg
int32_t angControl = 9000;     //angle, motor reduces by 0.01 deg/LSB
int32_t currentPosTilt = 0;    //current position of motor
int32_t currentPosPan = 0;     //current position of motor
int32_t newPosTilt = 0;        //next position of motor
int32_t newPosPan = 0;         //next position of motor
int32_t profile = 0;           //acceleration profile
int32_t sense = 0;             //input sensativity
int32_t tiltIn = 0;
int32_t panIn = 0;
int32_t homePan = 0;
int32_t homeTilt = 0;
int32_t homeAxis = 0;
uint8_t senseSetting = 0;
uint8_t profileSetting = 0;

// runs once //
void setup() {
  SERIAL.begin(9600);

  setupOLED(0x3C);

  while (CAN_OK != CAN.begin(CAN_1000KBPS))  // init can bus : baudrate = 1000k
  {
    SERIAL.println("CAN BUS Shield init fail");
    SERIAL.println(" Init CAN BUS Shield again");
    delay(250);
  }
  SERIAL.println("CAN BUS Shield init ok!");

  initalizeMotor(TILT_ID);
  initalizeMotor(PAN_ID);
  initalizeMotor(XTR1);
  initalizeMotor(XTR2);
  initalizeMotor(XTR3);

  homeMotor(XTR1);
  homeMotor(XTR2);
  homeMotor(XTR3);

  oled.clearDisplay();
  oled.drawBitmap(0, 0, stuffEmoji(2), 128, 64, WHITE);
  oled.display();
  homeTilt = homeMotor(TILT_ID);

  oled.clearDisplay();
  oled.drawBitmap(0, 0, stuffEmoji(3), 128, 64, WHITE);
  oled.display();
  homePan = homeMotor(PAN_ID);



  checkInput();
}

// runs continuous //
void loop() {

  currentTime = millis();

  if (samplePin(sensePin, &sense) || samplePin(profilePin, &profile)) {
    senseSetting = 1 + (sense / DIAL_INDEX);
    profileSetting = 1 + (profile / DIAL_INDEX);

    SERIAL.print("Sensitivity: ");
    SERIAL.println(senseSetting);
    SERIAL.print("Profile: ");
    SERIAL.println(profileSetting);

    // SERIAL.print("Pan: ");
    // SERIAL.println(checkSoftStop(PAN_ID, panIn, panLock, homePan, MAX_TORQUE, senseSetting, profileSetting));
    // SERIAL.print("Tilt: ");
    // SERIAL.println(checkSoftStop(TILT_ID, tiltIn, tiltLock, homeTilt, MAX_TORQUE, senseSetting, profileSetting));
  }

  if (samplePin(tiltPin, &tiltIn) || samplePin(panPin, &panIn)) {

    // SERIAL.print("Pan: ");
    // SERIAL.println(checkSoftStop(PAN_ID, panIn, panLock, homePan, MAX_TORQUE, senseSetting, profileSetting));
    // SERIAL.print("Tilt: ");
    // SERIAL.println(checkSoftStop(TILT_ID, tiltIn, tiltLock, homeTilt, MAX_TORQUE, senseSetting, profileSetting));
    // SERIAL.print("Sensitivity: ");
    // SERIAL.println(senseSetting);
    // SERIAL.print("Profile: ");
    // SERIAL.println(profileSetting);

    setTorque(PAN_ID, checkSoftStop(PAN_ID, panIn, panLock, homePan, MAX_TORQUE, senseSetting, profileSetting));
    setTorque(TILT_ID, checkSoftStop(TILT_ID, tiltIn, tiltLock, homeTilt, MAX_TORQUE, senseSetting, profileSetting));
  }


  if (currentTime - screenRefreshTimer >= SCREEN_REFRESH_PERIOD) {
    screenRefreshTimer = currentTime;

    oled.clearDisplay();
    oled.setTextSize(2);               // Normal 1:1 pixel scale
    oled.setTextColor(SSD1306_WHITE);  // Draw white text
    oled.setCursor(0, 0);              // Start at top-left corner
    oled.print(F("Pan| "));
    oled.println(panIn);
    oled.setCursor(0, 20);
    oled.print(F("Tilt| "));
    oled.println(tiltIn);
    oled.setCursor(0, 40);
    oled.print(F("Power| "));
    oled.println(readErrorState(TILT_ID));
    oled.display();
  }
}


// Functions //

uint8_t checkInput() {
  uint8_t pinState = 0;

  pinState |= samplePin(panPin, &panIn);
  pinState |= samplePin(tiltPin, &tiltIn);
  pinState |= samplePin(profilePin, &profile);
  pinState |= samplePin(sensePin, &sense);

  return pinState;
}

uint8_t samplePin(uint8_t pin, int32_t* oldSample) {

  // grab new sample
  int32_t newSample = analogRead(pin);

  // check if new sample is above noise threshold
  if (pin == A0 || pin == A1) {
    if (abs(newSample - *oldSample) > NOISE_TOLERANCE_JOYSTICK) {
      // update oldSample
      *oldSample = newSample;

      return 1;
    }
  }

  if (pin == A2 || pin == A3) {
    if (abs(newSample - *oldSample) > NOISE_TOLERANCE_DIAL) {
      // update oldSample
      *oldSample = newSample;

      return 1;
    }
  }

  return 0;
}

double mapVal(double valIn, double oldMin, double oldMax, double newMin, double newMax) {
  double oldRange = oldMax - oldMin;
  double newRange = newMax - newMin;
  double valOut = 0.0;

  valOut = (((valIn - oldMin) * newRange) / oldRange) + newMin;

  return valOut;
}

double mapProfile(double valIn, double maxVal, double sense, double profile) {
  double valOut = 0;
  double tempVal = valIn;

  valIn = abs(mapVal(valIn, 0, 1023, -100, 100));

  if (valIn <= 3.0) {
    valOut = 0;
  } else if (valIn > 3 && valIn <= 25.0) {
    valOut = (sense / SENSATIVITY_CONST) * pow(valIn, (PROFILE_CONST * profile));
  } else if (valIn > 25 && valIn <= 95.0) {
    valOut = (sense / SENSATIVITY_CONST) * valIn;
  } else if (valIn > 95.0) {
    valOut = maxVal;
  }

  if (valOut >= maxVal) {
    valOut = maxVal;
  }

  if (tempVal <= 512) {
    valOut = -valOut;
  }

  return valOut;
}

int32_t checkSoftStop(uint16_t canID, double valIn, int32_t stop, int32_t maxVal, int32_t homePosition, double sense, double profile) {
  int32_t currentPosition = readPosition(canID);  // in 1deg/lsb
  int32_t negStop = homePosition - stop;
  int32_t posStop = homePosition + stop;
  int32_t valOut = 0;

  // if (currentPosition <= negStop) {
  //   // setSpeed(canID, 0);
  //   setPosition(canID, (negStop + 1));
  //   valOut = 0;
  // } else if (currentPosition >= posStop) {
  //   // setSpeed(canID, 0);
  //   setPosition(canID, (posStop - 1));
  //   valOut = 0;
  // } else {
  valOut = mapProfile(valIn, maxVal, sense, profile);
  // }
  return valOut;
}

void initalizeMotor(uint16_t canID) {

  SERIAL.print(canID, HEX);
  SERIAL.print(" firmware version| ");
  SERIAL.println(readFirmwareVersion(canID));
  // runMotor(canID);
  // delay(5);
  // brakeOffMotor(canID);
  // delay(5);
  // zeroMotor(canID);
  // delay(5);
  // resetMotor(canID);
  // delay(250);
  // setAcceleration(canID, MAX_ACCELERATION);
  // delay(5);
  // resetMotor(canID);
  // delay(250);
}

int sendReceiveBuffer(uint16_t canId, unsigned char* buf_in, unsigned char* buf_out) {
  int success_flag = 0;  //Set success=]\-flag to zero
  int try_index = 3;     //Attempt send receive buffer 3 times
  buf_out[0] = 0x9C;
  buf_out[1] = 0x00;
  buf_out[2] = 0x00;
  buf_out[3] = 0x00;
  buf_out[4] = 0x00;
  buf_out[5] = 0x00;
  buf_out[6] = 0x00;
  buf_out[7] = 0x00;

  // if (buf_in[0] == 0x76)  //excepion as 0x76 does not recieve a return
  // {
  //   success_flag = 1;
  //   buf_out[0] = 0x76;
  //   return (success_flag);
  // }

  while ((buf_out[0] == 0x9C) && (try_index--)) {
    CAN.sendMsgBuf(canId, 0, 8, buf_in);
    delay(10);
    if (CAN_MSGAVAIL == CAN.checkReceive())  // check if data coming
    {
      CAN.readMsgBuf(&len, buf_out);  // read data,  len: data length, buf: data buf
      unsigned long canId = CAN.getCanId();
      if (buf_out[0] != 0X9C) {
        success_flag = 1;
      }
    }
  }
  return (success_flag);  //Return a zero if message not received
}

void printBuffer(unsigned char* buf) {
  SERIAL.print(buf[0], HEX);
  SERIAL.print("\t");
  SERIAL.print(buf[1], HEX);
  SERIAL.print("\t");
  SERIAL.print(buf[2], HEX);
  SERIAL.print("\t");
  SERIAL.print(buf[3], HEX);
  SERIAL.print("\t");
  SERIAL.print(buf[4], HEX);
  SERIAL.print("\t");
  SERIAL.print(buf[5], HEX);
  SERIAL.print("\t");
  SERIAL.print(buf[6], HEX);
  SERIAL.print("\t");
  SERIAL.println(buf[7], HEX);
}

uint32_t readFirmwareVersion(uint16_t canID) {
  unsigned char buf_in[8];
  unsigned char buf_out[8];
  uint32_t ver = 0;

  //Read Firmware Version
  buf_in[0] = 0xB2;
  buf_in[1] = 0x00;
  buf_in[2] = 0x00;
  buf_in[3] = 0x00;
  buf_in[4] = 0x00;
  buf_in[5] = 0x00;
  buf_in[6] = 0x00;
  buf_in[7] = 0x00;

  sendReceiveBuffer(canID, buf_in, buf_out);

  ver = ((uint32_t)buf_out[7] << 24) | ((uint32_t)buf_out[6] << 16) | ((uint32_t)buf_out[5] << 8) | ((uint32_t)buf_out[4]);

  return ver;
}

void runMotor(uint16_t canID) {
  unsigned char buf_in[8];
  unsigned char buf_out[8];
  //Motor ruunning command, legacy from v2.0??
  buf_in[0] = 0x88;
  buf_in[1] = 0x00;
  buf_in[2] = 0x00;
  buf_in[3] = 0x00;
  buf_in[4] = 0x00;
  buf_in[5] = 0x00;
  buf_in[6] = 0x00;
  buf_in[7] = 0x00;

  sendReceiveBuffer(canID, buf_in, buf_out);
}

void brakeOffMotor(uint16_t canID) {
  unsigned char buf_in[8];
  unsigned char buf_out[8];
  //Brake Off Command
  buf_in[0] = 0x77;
  buf_in[1] = 0x00;
  buf_in[2] = 0x00;
  buf_in[3] = 0x00;
  buf_in[4] = 0x00;
  buf_in[5] = 0x00;
  buf_in[6] = 0x00;
  buf_in[7] = 0x00;

  sendReceiveBuffer(canID, buf_in, buf_out);
}

void brakeOnMotor(uint16_t canID) {
  unsigned char buf_in[8];
  unsigned char buf_out[8];
  //Brake Off Command
  buf_in[0] = 0x78;
  buf_in[1] = 0x00;
  buf_in[2] = 0x00;
  buf_in[3] = 0x00;
  buf_in[4] = 0x00;
  buf_in[5] = 0x00;
  buf_in[6] = 0x00;
  buf_in[7] = 0x00;

  sendReceiveBuffer(canID, buf_in, buf_out);
}

void zeroMotor(uint16_t canID) {
  unsigned char buf_in[8];
  unsigned char buf_out[8];
  // set current position as zero
  buf_in[0] = 0x64;
  buf_in[1] = 0x00;
  buf_in[2] = 0x00;
  buf_in[3] = 0x00;
  buf_in[4] = 0x00;
  buf_in[5] = 0x00;
  buf_in[6] = 0x00;
  buf_in[7] = 0x00;

  sendReceiveBuffer(canID, buf_in, buf_out);
}

void resetMotor(uint16_t canID) {
  unsigned char buf_in[8];
  unsigned char buf_out[8];
  // motor reset, necassary for zero position write to take effect
  buf_in[0] = 0x76;
  buf_in[1] = 0x00;
  buf_in[2] = 0x00;
  buf_in[3] = 0x00;
  buf_in[4] = 0x00;
  buf_in[5] = 0x00;
  buf_in[6] = 0x00;
  buf_in[7] = 0x00;

  sendReceiveBuffer(canID, buf_in, buf_out);
}

void setTorque(uint16_t canID, uint32_t torque) {
  unsigned char buf_in[8];
  unsigned char buf_out[8];

  // if (torque == 0) {
  //   brakeOnMotor(canID);
  // } else {
  //   brakeOffMotor(canID);
  // }

  //Send incremental torque
  buf_in[0] = 0xA1;
  buf_in[1] = 0x00;
  buf_in[2] = 0x00;
  buf_in[3] = 0x00;
  buf_in[4] = torque;
  buf_in[5] = torque >> 8;
  buf_in[6] = 0x00;
  buf_in[7] = 0x00;

  sendReceiveBuffer(canID, buf_in, buf_out);
}

void setSpeed(uint16_t canID, uint32_t spd) {
  unsigned char buf_in[8];
  unsigned char buf_out[8];

  if (spd == 0) {
    stopMotor(canID);
  }

  //Send incremental speed
  buf_in[0] = 0xA2;
  buf_in[1] = 0x00;
  buf_in[2] = 0x00;
  buf_in[3] = 0x00;
  buf_in[4] = spd;
  buf_in[5] = spd >> 8;
  buf_in[6] = spd >> 16;
  buf_in[7] = spd >> 24;

  sendReceiveBuffer(canID, buf_in, buf_out);
}

void setAcceleration(uint16_t canID, uint32_t acc) {
  unsigned char buf_in[8];
  unsigned char buf_out[8];

  //Send incremental acceleration
  buf_in[0] = 0x43;
  buf_in[1] = 0x00;
  buf_in[2] = 0x00;
  buf_in[3] = 0x00;
  buf_in[4] = acc;
  buf_in[5] = acc >> 8;
  buf_in[6] = acc >> 16;
  buf_in[7] = acc >> 24;

  sendReceiveBuffer(canID, buf_in, buf_out);

  resetMotor(canID);
  delay(500);
}

void stopMotor(uint16_t canID) {
  unsigned char buf_in[8];
  unsigned char buf_out[8];

  //stop motor movment
  buf_in[0] = 0x81;
  buf_in[1] = 0x00;
  buf_in[2] = 0x00;
  buf_in[3] = 0x00;
  buf_in[4] = 0x00;
  buf_in[5] = 0x00;
  buf_in[6] = 0x00;
  buf_in[7] = 0x00;

  sendReceiveBuffer(canID, buf_in, buf_out);
}

void forceStopMotor(uint16_t canID) {
  unsigned char buf_in[8];
  unsigned char buf_out[8];

  //stop motor movment
  buf_in[0] = 0x80;
  buf_in[1] = 0x00;
  buf_in[2] = 0x00;
  buf_in[3] = 0x00;
  buf_in[4] = 0x00;
  buf_in[5] = 0x00;
  buf_in[6] = 0x00;
  buf_in[7] = 0x00;

  sendReceiveBuffer(canID, buf_in, buf_out);
}

void readMode(uint16_t canID) {
  unsigned char buf_in[8];
  unsigned char buf_out[8];

  //stop motor movment
  buf_in[0] = 0x70;
  buf_in[1] = 0x00;
  buf_in[2] = 0x00;
  buf_in[3] = 0x00;
  buf_in[4] = 0x00;
  buf_in[5] = 0x00;
  buf_in[6] = 0x00;
  buf_in[7] = 0x00;

  sendReceiveBuffer(canID, buf_in, buf_out);
}

int32_t homeMotor(uint16_t canID) {
  int32_t tempHome[4];
  // motor homing sequence

  setPosition(canID, 180);
  tempHome[0] = readPosition(canID);
  delay(1000);
  setPosition(canID, 190);
  delay(50);
  setPosition(canID, 180);
  delay(50);
  tempHome[1] = readPosition(canID);
  setPosition(canID, 170);
  delay(50);
  setPosition(canID, 180);
  delay(50);
  tempHome[2] = readPosition(canID);

  tempHome[3] = (tempHome[0] + tempHome[1] + tempHome[2]) / 3;

  SERIAL.print("Home Complete ");
  SERIAL.println(canID, HEX);

  return tempHome[3];
}

void setPosition(uint16_t canID, uint32_t position) {
  unsigned char buf_in[8];
  unsigned char buf_out[8];
  uint32_t newPosition = 0;

  newPosition = position * reductionRatio * 100;

  //Send incremental position
  buf_in[0] = 0xA4;
  buf_in[1] = 0x00;
  buf_in[2] = MAX_SPEED;
  buf_in[3] = MAX_SPEED >> 8;
  buf_in[4] = newPosition;
  buf_in[5] = newPosition >> 8;
  buf_in[6] = newPosition >> 16;
  buf_in[7] = newPosition >> 24;

  while ((readPosition(canID) < position * 0.99) || (readPosition(canID) > position * 1.01)) {
    sendReceiveBuffer(canID, buf_in, buf_out);
    // SERIAL.println("");
    // SERIAL.print("Set Pos ");
    // SERIAL.println(position);
    // SERIAL.print("Current Pos ");
    // SERIAL.println(readPosition(canID));
    // SERIAL.print("left Pos ");
    // SERIAL.println(position * 0.999);
    // SERIAL.print("Right Pos ");
    // SERIAL.println(position * 1.001);
  }
}

int32_t readPosition(uint16_t canID) {
  unsigned char buf_in[8];
  unsigned char buf_out[8];
  int32_t pos = 0;

  //read current position
  buf_in[0] = 0x92;
  buf_in[1] = 0x00;
  buf_in[2] = 0x00;
  buf_in[3] = 0x00;
  buf_in[4] = 0x00;
  buf_in[5] = 0x00;
  buf_in[6] = 0x00;
  buf_in[7] = 0x00;

  sendReceiveBuffer(canID, buf_in, buf_out);

  pos = ((uint32_t)buf_out[7] << 24) | ((uint32_t)buf_out[6] << 16) | ((uint32_t)buf_out[5] << 8) | ((uint32_t)buf_out[4]);
  pos = pos / 100 / reductionRatio;

  return pos;
}

int32_t readAcceleration(uint16_t canID) {
  unsigned char buf_in[8];
  unsigned char buf_out[8];
  int32_t acc = 0;

  //read current position
  buf_in[0] = 0x42;
  buf_in[1] = 0x00;
  buf_in[2] = 0x00;
  buf_in[3] = 0x00;
  buf_in[4] = 0x00;
  buf_in[5] = 0x00;
  buf_in[6] = 0x00;
  buf_in[7] = 0x00;

  sendReceiveBuffer(canID, buf_in, buf_out);

  acc = ((uint32_t)buf_out[7] << 24) | ((uint32_t)buf_out[6] << 16) | ((uint32_t)buf_out[5] << 8) | ((uint32_t)buf_out[4]);

  return acc;
}

int32_t readErrorState(uint16_t canID) {
  unsigned char buf_in[8];
  unsigned char buf_out[8];
  int8_t temp = 0;
  int8_t brake = 0;
  int16_t voltage = 0;
  int16_t errStat = 0;
  int16_t torqueCurrent = 0;
  int16_t motorSpeed = 0;
  int16_t outputAngle = 0;

  //read error frame one
  buf_in[0] = 0x9A;
  buf_in[1] = 0x00;
  buf_in[2] = 0x00;
  buf_in[3] = 0x00;
  buf_in[4] = 0x00;
  buf_in[5] = 0x00;
  buf_in[6] = 0x00;
  buf_in[7] = 0x00;

  sendReceiveBuffer(canID, buf_in, buf_out);

  temp = buf_out[1];
  brake = buf_out[3];
  voltage = (((uint16_t)buf_out[5] << 8) | ((uint16_t)buf_out[4]));
  errStat = (((uint16_t)buf_out[7] << 8) | ((uint16_t)buf_out[6]));

  //read error frame two
  buf_in[0] = 0x9C;
  buf_in[1] = 0x00;
  buf_in[2] = 0x00;
  buf_in[3] = 0x00;
  buf_in[4] = 0x00;
  buf_in[5] = 0x00;
  buf_in[6] = 0x00;
  buf_in[7] = 0x00;

  sendReceiveBuffer(canID, buf_in, buf_out);

  torqueCurrent = (((uint16_t)buf_out[3] << 8) | ((uint16_t)buf_out[2]));
  motorSpeed = (((uint16_t)buf_out[5] << 8) | ((uint16_t)buf_out[4]));
  outputAngle = (((uint16_t)buf_out[7] << 8) | ((uint16_t)buf_out[6]));

  return voltage;

  // SERIAL.println("");
  // SERIAL.print("Motor Temp: ");
  // SERIAL.println(temp);
  // SERIAL.print("Brake State: ");
  // SERIAL.println(brake);
  // SERIAL.print("Voltage: ");
  // SERIAL.println(voltage);
  // SERIAL.print("Error State: ");
  // SERIAL.println(errStat);
  // SERIAL.print("Torque Current: ");
  // SERIAL.println(torqueCurrent);
  // SERIAL.print("Motor Speed: ");
  // SERIAL.println(motorSpeed);
  // SERIAL.print("Output Angle: ");
  // SERIAL.println(outputAngle);
}

void setupOLED(uint8_t ID) {
  if (!oled.begin(SSD1306_SWITCHCAPVCC, ID)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true)
      ;
  }

  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(40, 0);
  oled.print(F("HANC"));
  oled.setCursor(45, 20);
  oled.print(F("Ver: "));
  oled.setCursor(40, 40);
  oled.print(F("2.50"));
  oled.display();

  delay(750);
}


const uint8_t* stuffEmoji(uint8_t slc) {
  switch (slc) {
    case 1:
      return RIGHT_FISH;
      break;
    case 2:
      return LEFT_FISH;
      break;
    case 3:
      return CAMERA;
      break;
    case 4:
      return MOTOR;
      break;
    default:
      break;
  }
}

// END FILE
