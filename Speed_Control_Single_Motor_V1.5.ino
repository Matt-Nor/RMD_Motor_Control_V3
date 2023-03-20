// Motion control and general testing of multiple motors
// This is the main sourcecode to test CANbus control of the MyActuator RMD-X series with V3 firmware
// Uses Arduino Mega with the Sparkfun CAN-BUS shield
//
// National Geographic Photo Engineering
//
// Matt Norman
// May 20, 2022

#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

// SAMD core //
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL SerialUSB
#else
#define SERIAL Serial
#endif

//# defines and constants //
#define CONTROL_PERIOD 10         // in MS
#define SCREEN_REFRESH_PERIOD 50  // in MS
#define NOISE_TOLERANCE 5
#define MAX_ACCELERATION 60000  // Max acceleration parameter 1 dps/s range 50 - 80000
#define MAX_SPEED 100000        // Max speed parameter 1 dps/LSB -/+ 32767
#define MAX_TORQUE 2000         // Max torque parameter 1 dps/LSB -/+ 32767
#define SCREEN_WIDTH 128        // OLED display width, in pixels
#define SCREEN_HEIGHT 64        // OLED display height, in pixels
#define OLED_RESET -1           // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C     ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define MOTOR_ID 0X140
unsigned long controlTimer = 0;
unsigned long screenRefreshTimer = 0;
unsigned long currentTime = 0;
const int panPin = A0;
const int tiltPin = A1;

// cs pin = 10 for Uno = 53 for Mega //
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);  // Set CS pin //

// func prototypes //

double mapVal(double, double, double, double, double);
const uint8_t* stufEmoji(uint8_t);
int sendReceiveBuffer(uint16_t, unsigned char*, unsigned char*);
uint8_t checkInput();
uint8_t samplePin(uint8_t, int32_t*);
int32_t checkSoftStop(uint16_t, int8_t, int32_t, int32_t);
int32_t readPosition(uint16_t);
int32_t readAcceleration(uint16_t);
int32_t homeMotor(uint16_t);
int32_t readErrorState(uint16_t);
uint32_t readFirmwareVersion(uint16_t);
void initalizeMotor(uint16_t);
void brakeOffMotor(uint16_t);
void printBuffer(unsigned char*);
void resetMotor(uint16_t);
void runMotor(uint16_t);
void setSpeed(uint16_t, uint32_t);
void setAcceleration(uint16_t, uint32_t);
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
int32_t accelerationIn = 0;    //current acceleration of motor
int32_t senseIn = 0;           //current damping of motor
int32_t tiltIn = 0;
int32_t panIn = 0;
int32_t homePan = 0;
int32_t homeTilt = 0;
int32_t homeAxis = 0;
double sense = 0;  // Movment damping

// runs once //
void setup() {
  SERIAL.begin(115200);

  while (CAN_OK != CAN.begin(CAN_1000KBPS))  // init can bus : baudrate = 1000k
  {
    SERIAL.println("CAN BUS Shield init fail");
    SERIAL.println(" Init CAN BUS Shield again");
    delay(250);
  }
  SERIAL.println(" ");
  SERIAL.println("CAN BUS Shield init ok!");

  // initalizeMotor(0X142);

  for (int i = 1; i <= 8; ++i) {
    initalizeMotor(MOTOR_ID + i);
  }

  checkInput();
}

// runs continuous //
void loop() {

  // currentTime = millis();

  // if (currentTime - controlTimer >= CONTROL_PERIOD) {
  //   controlTimer = currentTime;

  //   // setSpeed(0x142, checkSoftStop(0x142, panPin, panLock, homePan));
  //   setSpeed(0x145, (mapVal(analogRead(panPin), 0, 1023, -1000000, 1000000)));
  // }


  // if (currentTime - screenRefreshTimer >= SCREEN_REFRESH_PERIOD) {
  //   screenRefreshTimer = currentTime;

  //   oled.clearDisplay();
  //   oled.setTextSize(2);               // Normal 1:1 pixel scale
  //   oled.setTextColor(SSD1306_WHITE);  // Draw white text
  //   oled.setCursor(0, 0);              // Start at top-left corner
  //   oled.print(F("Pan| "));
  //   oled.println(readPosition(MOTOR_ID));
  //   oled.setCursor(0, 40);
  //   oled.print(F("Spd |"));
  //   oled.println(readErrorState(MOTOR_ID));
  //   oled.display();
  // }
}


// Functions //

uint8_t checkInput() {
  uint8_t pinState = 0;

  pinState |= samplePin(panPin, &panIn);
  pinState |= samplePin(tiltPin, &tiltIn);

  return pinState;
}

uint8_t samplePin(uint8_t pin, int32_t* oldSample) {

  // grab new sample
  int32_t newSample = analogRead(pin);

  // check if new sample is above noise threshold
  if (abs(newSample - *oldSample) > NOISE_TOLERANCE) {
    // update oldSample
    *oldSample = newSample;

    return 1;
  }

  return 0;
}

double mapVal(double valIn, double oldMin, double oldMax, double newMin, double newMax) {
  double oldRange = oldMax - oldMin;
  double newRange = newMax - newMin;
  double valOut = 0.0;

  valOut = (((valIn - oldMin) * newRange) / oldRange) + newMin;

  if (valIn >= 500.0 && valIn <= 550.0) {
    valOut = 0.0;
  }

  if (valOut > newMax) {
    valOut = newMax;
  }

  if (valOut < newMin) {
    valOut = newMin;
  }

  return valOut;
}

void initalizeMotor(uint16_t canID) {
  SERIAL.print(canID, HEX);
  SERIAL.print(" firmware version| ");
  SERIAL.print(readFirmwareVersion(canID));


  // brakeOffMotor(canID);
  // delay(5);
  readMode(canID);
  delay(5);


  if (readFirmwareVersion(canID) != 0) {
    // homeMotor(canID);
    // SERIAL.print ("FLAG");
    setPosition(canID, 90);
    delay(1000);
    setPosition(canID, 180);
    delay(1000);
    setPosition(canID, 45);
    delay(1000);
  }
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

int sendReceiveBuffer(uint16_t canId, unsigned char* buf_in, unsigned char* buf_out) {
  int success_flag = 0;  //Set success flag to zero
  int try_index = 3;     //Attempt send receive buffer 5 times
  buf_out[0] = 0x88;
  buf_out[1] = 0x00;
  buf_out[2] = 0x00;
  buf_out[3] = 0x00;
  buf_out[4] = 0x00;
  buf_out[5] = 0x00;
  buf_out[6] = 0x00;
  buf_out[7] = 0x00;

  if (buf_in[0] == 0x76)  //excepion as 0x76 does not recieve a return
  {
    success_flag = 1;
    buf_out[0] = 0x76;
    return (success_flag);
  }

  while ((buf_out[0] == 0x88) && (try_index--)) {
    CAN.sendMsgBuf(canId, 0, 8, buf_in);
    delay(25);
    if (CAN_MSGAVAIL == CAN.checkReceive())  // check if data coming
    {
      CAN.readMsgBuf(&len, buf_out);  // read data,  len: data length, buf: data buf
      unsigned long canId = CAN.getCanId();
      if (buf_out[0] != 0x88) {
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

void runMotor(uint16_t canID) {
  unsigned char buf_in[8];
  unsigned char buf_out[8];
  //Motor ruunning command, legacy from v2.0??
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

void setTorque(uint16_t canID, uint32_t spd) {
  unsigned char buf_in[8];
  unsigned char buf_out[8];

  // if (spd == 0) {
  //   stopMotor(canID);
  // }

  //Send incremental torque
  buf_in[0] = 0xA1;
  buf_in[1] = 0x00;
  buf_in[2] = 0x00;
  buf_in[3] = 0x00;
  buf_in[4] = spd;
  buf_in[5] = spd >> 8;
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

  SERIAL.print("   Mode| ");
  SERIAL.println(buf[7], HEX);
}

int32_t homeMotor(uint16_t canID) {
  int32_t tempHome[4];
  // motor homing sequence

  setPosition(canID, 180);
  tempHome[0] = readPosition(canID);
  delay(1000);
  setPosition(canID, 185);
  delay(50);
  setPosition(canID, 180);
  delay(50);
  tempHome[1] = readPosition(canID);
  setPosition(canID, 175);
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

  newPosition = position * 100 * reductionRatio;

  //Send incremental position
  buf_in[0] = 0xA3;
  buf_in[1] = 0x00;
  buf_in[2] = 0x00;
  buf_in[3] = 0x00;
  buf_in[4] = newPosition;
  buf_in[5] = newPosition >> 8;
  buf_in[6] = newPosition >> 16;
  buf_in[7] = newPosition >> 24;

  // while ((readPosition(canID) < position * 0.999) || (readPosition(canID) > position * 1.001)) {
  sendReceiveBuffer(canID, buf_in, buf_out);
  // delay(5);
  SERIAL.print("\t Angle| ");
  SERIAL.println(buf[7], HEX);

  // SERIAL.println("");
  // SERIAL.print("Set Pos ");
  // SERIAL.println(position);
  // SERIAL.print("Current Pos ");
  // SERIAL.println(readPosition(canID));
  // SERIAL.print("left Pos ");
  // SERIAL.println(position * 0.999);
  // SERIAL.print("Right Pos ");
  // SERIAL.println(position * 1.001);
  // }
}

int32_t checkSoftStop(uint16_t canID, int8_t pin, int32_t stop, int32_t homePosition) {
  int32_t currentPosition = readPosition(canID);  // in 1deg/lsb
  int32_t negStop = homePosition - stop;
  int32_t posStop = homePosition + stop;
  int32_t speed = 0;

  if (currentPosition <= negStop) {
    // setSpeed(canID, 0);
    setPosition(canID, (negStop + 1));
    speed = 0;
  } else if (currentPosition >= posStop) {
    // setSpeed(canID, 0);
    setPosition(canID, (posStop - 1));
    speed = 0;
  } else {
    speed = mapVal(analogRead(pin), 0, 1023, -MAX_SPEED, MAX_SPEED);
  }
  return speed;
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

// END FILE
