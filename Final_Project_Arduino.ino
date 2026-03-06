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
- The proximity sensors used have a normally HIGH logic level, for the sake of simplfying logic
  the state of these sensors are inverted via convertDigitalInput()
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

enum RobotInstruction
{
  fault = 0,
  task1 = 1,
  task2pos1 = 2,
  task2pos2 = 3,
  task3 = 4,
  NULL5 = 5,
  task2pos3 = 6,
  NULL7 = 7,
  holding = 8,
  tool1 = 9,
  tool2 = 10,
  NULL11 = 11,
  tool3 = 12,
  NULL13 = 13,
  NULL14 = 14,
  home = 15,
};

//Holds information for communication between CODESYS, Arduino IDE, and Arduino board
struct Connection
{
  int pin; 
  //Corresponding number to be entered into the map data in CODESYS
  //Each registerType has it's own set of registerNum
  int registerNum; 
  //Corresponds to the channel that map data is to be entered in CODESYS
  RegisterType registerType;
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
  int speed; //Motor speed value between 0-255
  int direction; //direction = 1 is CW, direction = -1 is CCW
};

const int NUM_ROBOT_BITS = 4;
struct Robot
{
  Connection inputConBit[NUM_ROBOT_BITS];
  Connection outputConBit[NUM_ROBOT_BITS];
  int inputStateBit[NUM_ROBOT_BITS]; 
  int outputStateBit[NUM_ROBOT_BITS];
  int instruct;
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
.conEN = {.pin = 10, .registerNum = 0, .registerType = RegisterType::hreg}, 
.speed = 0, .direction = 1
};

//Initialize robot object
Robot robotWindow = 
{
.inputConBit = {
  {.pin = 11, .registerNum = 6, .registerType = RegisterType::ists},
  {.pin = 12, .registerNum = 7, .registerType = RegisterType::ists},
  {.pin = 13, .registerNum = 8, .registerType = RegisterType::ists},
  {.pin = 14, .registerNum = 9, .registerType = RegisterType::ists}
  }, 
.outputConBit = {
  {.pin = 15, .registerNum = 2, .registerType = RegisterType::coil},
  {.pin = 16, .registerNum = 3, .registerType = RegisterType::coil},
  {.pin = 17, .registerNum = 4, .registerType = RegisterType::coil},
  {.pin = 18, .registerNum = 5, .registerType = RegisterType::coil}
  }, 
.inputStateBit = {LOW}, .outputStateBit = {LOW}, .instruct = 0
};

//Const speed for motor operation
const int MOTOR_RUN_SPEED = 255;

//////THESE ARE FOR TESTING
unsigned long motorMillis = 0;
const unsigned long MOTOR_DELAY = 700;
//////

//Sets up pins and registers
void setupConnection(Connection c);
//Store input data
void readDigitalInput();
//Converts LOWs to HIGHs and HIGHs to LOWs, for the proximity sensor
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
  robotWindow.instruct = 0;
  for (int i = 0; i < NUM_ROBOT_BITS; i++)
  {
    robotWindow.outputStateBit[i] = mb.Coil(robotWindow.outputConBit[i].registerNum);
    digitalWrite(robotWindow.outputConBit[i].pin, robotWindow.outputStateBit[i]);
    robotWindow.instruct = robotWindow.outputStateBit[i] == true ? robotWindow.instruct + (1<<i) : robotWindow.instruct;
  }
}

void sendOutputMotor()
{
  analogWrite(motorWindow.conEN.pin, mb.Hreg(motorWindow.conEN.registerNum));
  digitalWrite(motorWindow.con2A.pin, mb.Coil(motorWindow.con2A.registerNum));
  digitalWrite(motorWindow.con1A.pin, mb.Coil(motorWindow.con1A.registerNum));
}