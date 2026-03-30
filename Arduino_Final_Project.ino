/*
Reads sensor inputs and outputs them to a PLC program.
Then receives instructions from PLC to control a DC motor and robot arm.
*/

/*
NOTES:
- The following libraries need to be installed from the library manager
  - Modbus-Arduino (NOT ArduinoModbus or Modbus)
  - Modbus-Serial 
- Don't use the serial communication pins (RX/TX), pin 0 and 1
- The IR sensors used have a normally HIGH logic level (i.e. when it senses something it goes active LOW), 
  for the sake of simplfying logic the state of these sensors are inverted via convertDigitalInput()
- Each used pin has a corresponding register for use in CODESYS, but it
  doesn't necessarily need to be used in the CODESYS program, it's just designed that way for convenience
  - UNUSED REGISTERS
    - con2A, con1A, conEN in favor of controlling them via staMotorCW, staMotorCCW
*/

/*
CODESYS INTEGRATION:
- This video goes through the process 
  https://www.youtube.com/watch?v=UpzWkuqIYgE
- The terms Master-Slave also refers to the terms Client-Server
- MAKE SURE:
  - COM port is correct (Matches the COM port used in Arduino program under "Tools")
  - Baud rate is correct
  - Parity is set to none
  - 8 data bits, 1 stop bit
  - Server address in Modbus Server matches ID in Arduino code
- SETTING UP REGISTERS:
  - Match the channel bit number of CODESYS variables with the registerNum of the Arduino variables
  - Digital input:  Read Discrete Inputs
                    Length 10
  - Digital output: Write Multiple Coils
                    Length 8
  - Analog input:   Read Input Register
                    Length n/a
  - Analog ouput:   Write Single Register
                    Length 1
*/
#include <ModbusSerial.h>

const unsigned long BAUD = 115200; //Baud rate
const byte ID = 1; //ID used for communications
const int TXPIN = -1; //Normally disabled, used to control RX/TX pins when connected to a RS485 driver

#define MySerial Serial //Defines serial port used

ModbusSerial mb (MySerial, ID, TXPIN); //Defines modbus object

enum class RegisterType //Determines type of register for each pin
{
  ireg, //Analog input
  hreg, //Analog output
  ists, //Digital input
  coil  //Digital output
};

/*
These robot instructions are here for reference for testing purposes and clarification
*/
// enum RobotInstruction
// {
//   fault = 0,
//   task1 = 1,
//   task2pos1 = 2,
//   task2pos2 = 3,
//   task3 = 4,
//   NULL5 = 5,
//   task2pos3 = 6,
//   NULL7 = 7,
//   holding = 8,
//   tool1 = 9,
//   tool2 = 10,
//   NULL11 = 11,
//   tool3 = 12,
//   NULL13 = 13,
//   NULL14 = 14,
//   home = 15,
// };

//Holds pin information for CODESYS to access inputs and send outputs from/to specified pins
struct Connection
{
  int pin; 
  //Corresponding number to be entered into the map data in CODESYS
  //Each registerType has it's own set of registerNum
  int registerNum; 
  //Corresponds to the channel that map data is to be entered in CODESYS
  RegisterType registerType;
};

//Holds digital outputs (coils) from CODESYS
struct Status
{
  bool state;
  int registerNum;
};

struct Sensor 
{
  Connection con;
  bool state;
};

struct Motor
{
  //Holds connection data to L293D pins
  Connection con2A;
  Connection con1A;
  Connection conEN;
  //Deterimes direction that motor is running
  Status staMotorCW;
  Status staMotorCCW;
  int speed; //Motor speed value between 0-255
};

const int NUM_ROBOT_BITS = 4;
struct Robot
{
  Connection inputConBit[NUM_ROBOT_BITS];
  Connection outputConBit[NUM_ROBOT_BITS];
  int inputStateBit[NUM_ROBOT_BITS]; 
  int outputStateBit[NUM_ROBOT_BITS];
};

//Initalize sensor objects
Sensor sensorSprayer =  {.con = {.pin = 2, .registerNum = 0, .registerType = RegisterType::ists}, .state = HIGH};
Sensor sensorSqueegee = {.con = {.pin = 3, .registerNum = 1, .registerType = RegisterType::ists}, .state = HIGH};
Sensor sensorCloth =    {.con = {.pin = 4, .registerNum = 2, .registerType = RegisterType::ists}, .state = HIGH};
Sensor sensorPosMin =   {.con = {.pin = 5, .registerNum = 3, .registerType = RegisterType::ists}, .state = LOW};
Sensor sensorPosMax =   {.con = {.pin = 6, .registerNum = 4, .registerType = RegisterType::ists}, .state = LOW};
Sensor sensorInfrared = {.con = {.pin = 7, .registerNum = 5, .registerType = RegisterType::ists}, .state = LOW};

//Initialize motor object
Motor motorWindow = 
{
.con2A = {.pin = 8, .registerNum = 0, .registerType = RegisterType::coil}, 
.con1A = {.pin = 9, .registerNum = 1, .registerType = RegisterType::coil}, 
.conEN = {.pin = 10, .registerNum = 0, .registerType = RegisterType::hreg}, //Connect to PWM pin
.staMotorCW = {.state = false, .registerNum = 6}, //Coil
.staMotorCCW = {.state = false, .registerNum = 7}, //Coil
.speed = 0
};

//Initialize robot object
Robot robotWindow = 
{
.inputConBit = {
  {.pin = 23, .registerNum = 6, .registerType = RegisterType::ists},
  {.pin = 25, .registerNum = 7, .registerType = RegisterType::ists},
  {.pin = 27, .registerNum = 8, .registerType = RegisterType::ists},
  {.pin = 29, .registerNum = 9, .registerType = RegisterType::ists}
  }, 
.outputConBit = {
  {.pin = 22, .registerNum = 2, .registerType = RegisterType::coil},
  {.pin = 24, .registerNum = 3, .registerType = RegisterType::coil},
  {.pin = 26, .registerNum = 4, .registerType = RegisterType::coil},
  {.pin = 28, .registerNum = 5, .registerType = RegisterType::coil}
  }, 
.inputStateBit = {LOW}, .outputStateBit = {LOW}
};

//Const speed for motor operation (Range 0-255)
const int MOTOR_RUN_SPEED = 10;
//For tracking motor delay
unsigned long motorMillis = 0;
const unsigned long MOTOR_DELAY = 700;

//Sets up pins and registers
void setupConnection(Connection c);
//Store input data
void readDigitalInput();
//Converts LOWs to HIGHs and HIGHs to LOWs, for the IR sensor
int convertDigitalInput(Sensor sensorObject);
//Send state of motor's motion
void sendOutputMotor();
//Send state of robot's motion
void sendOutputRobot();

void setup() {
  //Set up baud rate
  MySerial.begin(BAUD);
  mb.config(BAUD);

  //Sets up pins and registers
  setupConnection(sensorSprayer.con);
  setupConnection(sensorSqueegee.con);
  setupConnection(sensorCloth.con);
  setupConnection(sensorPosMin.con);
  setupConnection(sensorPosMax.con);
  setupConnection(sensorInfrared.con);
  setupConnection(motorWindow.con2A);
  setupConnection(motorWindow.con1A);
  setupConnection(motorWindow.conEN);
  for (int i = 0; i < NUM_ROBOT_BITS; i++)
  {
    setupConnection(robotWindow.inputConBit[i]);
    setupConnection(robotWindow.outputConBit[i]);
  }

  //Sets up registers for Status objects
  mb.addCoil(motorWindow.staMotorCW.registerNum);
  mb.addCoil(motorWindow.staMotorCCW.registerNum);

  motorMillis = millis();
}

void loop() {
  //Task that performs all operations on MODBUS
  mb.task(); 

  readDigitalInput();
  sendOutputMotor();
  sendOutputRobot();
}

void setupConnection(Connection c)
{
  if (c.registerType == RegisterType::ists) //Digital input
  {
    pinMode(c.pin, INPUT);
    mb.addIsts(c.registerNum);
  }
  if (c.registerType == RegisterType::coil) //Digital output
  {
    pinMode(c.pin, OUTPUT);
    mb.addCoil(c.registerNum);
  }
  if (c.registerType == RegisterType::ireg) //Analog input
  {
    pinMode(c.pin, INPUT);
    mb.addIreg(c.registerNum);
  }
  if (c.registerType == RegisterType::hreg) //Analog output
  {
    pinMode(c.pin, OUTPUT);
    mb.addHreg(c.registerNum);
  }
}

void readDigitalInput()
{
  //Stores input state values
  sensorSprayer.state = convertDigitalInput(sensorSprayer);
  sensorSqueegee.state = convertDigitalInput(sensorSqueegee);
  sensorCloth.state = convertDigitalInput(sensorCloth);
  sensorPosMin.state = convertDigitalInput(sensorPosMin);
  sensorPosMax.state = convertDigitalInput(sensorPosMax);

  sensorInfrared.state = digitalRead(sensorInfrared.con.pin);

/*
Code for testing without the IR sensors due to their normally HIGH output.
Highlight the entire function and use CTRL + K + U to uncomment it all, CTRL + K + C to comment it all
*/
  // sensorSprayer.state = digitalRead(sensorSprayer.con.pin);
  // sensorSqueegee.state = digitalRead(sensorSqueegee.con.pin);
  // sensorCloth.state = digitalRead(sensorCloth.con.pin);
  // sensorPosMin.state = digitalRead(sensorPosMin.con.pin);
  // sensorPosMax.state = digitalRead(sensorPosMax.con.pin);

  //Sets states of the digital inputs registers for CODESYS
  mb.Ists(sensorSprayer.con.registerNum, sensorSprayer.state);
  mb.Ists(sensorSqueegee.con.registerNum, sensorSqueegee.state);
  mb.Ists(sensorCloth.con.registerNum, sensorCloth.state);
  mb.Ists(sensorPosMin.con.registerNum, sensorPosMin.state);
  mb.Ists(sensorPosMax.con.registerNum, sensorPosMax.state);
  mb.Ists(sensorInfrared.con.registerNum, sensorInfrared.state);

  for (int i = 0; i < NUM_ROBOT_BITS; i++)
  {
    robotWindow.inputStateBit[i] = digitalRead(robotWindow.inputConBit[i].pin);
    mb.Ists(robotWindow.inputConBit[i].registerNum, robotWindow.inputStateBit[i]);
  }
}

int convertDigitalInput(Sensor sensorObject)
{
  int state = digitalRead(sensorObject.con.pin) == HIGH ? LOW : HIGH;
  return state;
}

void sendOutputRobot()
{
  for (int i = 0; i < NUM_ROBOT_BITS; i++)
  {
    //Update the output sent to the robot from CODESYS
    robotWindow.outputStateBit[i] = mb.Coil(robotWindow.outputConBit[i].registerNum);
    digitalWrite(robotWindow.outputConBit[i].pin, robotWindow.outputStateBit[i]);
  }
}

/*
This is another version of sendOutputRobot() to test Arduino to Robot Arm communication
Highlight the entire function and use CTRL + K + U to uncomment it all, CTRL + K + C to comment it all
*/
// void sendOutputRobot()
// {
//   //Edit the value depending on the instruction number you want to test
//   robotInstruction = 0;
//   for (int i = 0; i < NUM_ROBOT_BITS; i++)
//   {
//     //Performs a bitwise AND with the instruction num and bit number i determine if that bit is HIGH or LOW
//     robotWindow.outputStateBit[i] = (robotInstruction & (1<<i)) == (1<<i) ? HIGH : LOW;
//     digitalWrite(robotWindow.outputConBit[i].pin, robotWindow.outputStateBit[i]);
//   }
// }

void sendOutputMotor()
{
  bool prevStateCW = motorWindow.staMotorCW.state;
  bool prevStateCCW = motorWindow.staMotorCW.state;

  //Update the motor's Status objects from CODESYS
  motorWindow.staMotorCW.state = mb.Coil(motorWindow.staMotorCW.registerNum);
  motorWindow.staMotorCCW.state = mb.Coil(motorWindow.staMotorCCW.registerNum);
  //Delay motor's operation when it changes directions
  if (motorWindow.staMotorCW.state != prevStateCW && motorWindow.staMotorCCW.state != prevStateCCW)
  {
    motorMillis = millis();
  } 

  //Operates if not currently delayed
  if (millis() - motorMillis > MOTOR_DELAY)
  {
    //CW
    if (motorWindow.staMotorCW.state == true && motorWindow.staMotorCCW.state == false)
    {
      analogWrite(motorWindow.conEN.pin, MOTOR_RUN_SPEED);
      digitalWrite(motorWindow.con2A.pin, true);
      digitalWrite(motorWindow.con1A.pin, false);
    }
    //CCW
    else if (motorWindow.staMotorCCW.state == true && motorWindow.staMotorCW.state == false)
    {
      analogWrite(motorWindow.conEN.pin, MOTOR_RUN_SPEED);
      digitalWrite(motorWindow.con2A.pin, false);
      digitalWrite(motorWindow.con1A.pin, true);
    }
    //Not moving or error where CW and CCW are both true
    else
    {
      analogWrite(motorWindow.conEN.pin, 0);
      digitalWrite(motorWindow.con2A.pin, false);
      digitalWrite(motorWindow.con1A.pin, false);
    }
  }
  else
  {
    analogWrite(motorWindow.conEN.pin, 0);
    digitalWrite(motorWindow.con2A.pin, false);
    digitalWrite(motorWindow.con1A.pin, false);
  }
}
