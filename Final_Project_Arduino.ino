/*
Reads sensor inputs and outputs them to a PLC program.
Then receives instructions from PLC to control a DC motor and robot arm.
*/
enum MotorInstruction
{
  stop = 0,
  moveCW = 1,
  moveCCW = 2
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

struct Sensor 
{
  int pin;
  //NOTE the sensors used have normally HIGH outputs, when it senses something, it goes to LOW
  bool state;
};

struct Motor
{
  //Holds pins connected to L293D pins
  int pin2A;
  int pin1A;
  int pinEN;
  //Motor speed value between 0-255
  int speed;
  //direction = 1 is CW, direction = -1 is CCW
  int direction;
  int instruct;
};

const int NUM_ROBOT_BITS = 4;
struct Robot
{
  int inputPinBit[NUM_ROBOT_BITS];
  int outputPinBit[NUM_ROBOT_BITS];
  int inputStateBit[NUM_ROBOT_BITS]; 
  int outputStateBit[NUM_ROBOT_BITS];
  int instruct;
};

//Initalize sensor objects
Sensor sensorSprayer = {.pin = 2, .state = HIGH};
Sensor sensorSqueegee = {.pin = 3, .state = HIGH};
Sensor sensorCloth = {.pin = 4, .state = HIGH};
Sensor sensorPosMin = {.pin = 5, .state = LOW};
Sensor sensorPosMax = {.pin = 6, .state = LOW};
Sensor sensorInfrared {.pin = 7, .state = LOW};

//Initialize motor object
Motor motorWindow = {.pin2A = 7, .pin1A = 8, .pinEN = 9, .speed = 0, .direction = 1, .instruct = 0};
//Const speed for motor operation
const int MOTOR_RUN_SPEED = 255;

//////THESE ARE FOR TESTING
unsigned long motorMillis = 0;
const unsigned long MOTOR_DELAY = 700;

unsigned long robotInstructMillis = 0;
const unsigned long ROBOT_INSTRUCT_DELAY = 200;
//////

//Initialize robot object
Robot robotWindow = {.inputPinBit = {10, 11, 12, 13}, .outputPinBit = {14, 15, 16, 17}, .inputStateBit = {LOW}, .outputStateBit = {LOW}, .instruct = 0};

//Store input data
void readInput();
//Converts LOWs to HIGHs and HIGHs to LOWs
//Used to set active sensor state output as HIGH, ad inactive sensor state output as LOW
//because the sensor output is reversed
int convertDigitalInput(Sensor sensorObject);
//Send stored sensor input data to PLC
void sendOutputDataPLC();
//Set states of motor's outputs
void setOutputMotor();
//Set states of robot's outputs
void setOutputRobot();
//Send state of motor's motion
void sendOutputMotor();
//Send state of robot's motion
void sendOutputRobot();

void setup() {
  pinMode(sensorSprayer.pin, INPUT);
  pinMode(sensorSqueegee.pin, INPUT);
  pinMode(sensorCloth.pin, INPUT);
  pinMode(sensorPosMin.pin, INPUT);
  pinMode(sensorPosMax.pin, INPUT);
  pinMode(sensorInfrared.pin, INPUT);
  pinMode(motorWindow.pin2A, OUTPUT);
  pinMode(motorWindow.pin1A, OUTPUT);
  pinMode(motorWindow.pinEN, OUTPUT);

  for (int i = 0; i < NUM_ROBOT_BITS; i++)
  {
    pinMode(robotWindow.inputPinBit[i], INPUT);
    pinMode(robotWindow.outputPinBit[i], OUTPUT);
  }
}

void loop() {
  readInput();
  setOutputMotor();
  sendOutputMotor();
  setOutputRobot();
  sendOutputRobot();
}

void readInput()
{
  sensorSprayer.state = convertDigitalInput(sensorSprayer);
  sensorSqueegee.state = convertDigitalInput(sensorSqueegee);
  sensorCloth.state = convertDigitalInput(sensorCloth);
  sensorPosMin.state = convertDigitalInput(sensorPosMin);
  sensorPosMax.state = convertDigitalInput(sensorPosMax);

  sensorInfrared.state = digitalRead(sensorInfrared.pin);
  
  for (int i = 0; i < NUM_ROBOT_BITS; i++)
  {
    robotWindow.inputStateBit[i] = digitalRead(robotWindow.inputPinBit[i]);
  }
}

int convertDigitalInput(Sensor sensorObject)
{
  int state = digitalRead(sensorObject.pin) == HIGH ? LOW : HIGH;
  return state;
}

void setOutputMotor()
{
  if (sensorPosMin.state == HIGH && sensorPosMax.state == HIGH)
  {
    motorWindow.speed = 0;
  }
  //Min sensor is active
  else if (sensorPosMin.state == HIGH)
  {
    motorWindow.speed = MOTOR_RUN_SPEED;
    if (motorWindow.direction == -1)
    {
      motorMillis = millis();
    }
    motorWindow.direction = 1;
    
  }
  //Max sensor is active
  else if (sensorPosMax.state == HIGH)
  {
    motorWindow.speed = MOTOR_RUN_SPEED;
    if (motorWindow.direction == 1)
    {
      motorMillis = millis();
    }
    motorWindow.direction = -1;
  }
  //No sensors are active
  else
  {
    motorWindow.speed = MOTOR_RUN_SPEED;
  }
}

void setOutputRobot()
{
  /////THIS IS FOR TESTING
  if (millis() - robotInstructMillis > ROBOT_INSTRUCT_DELAY)
  {
    robotInstructMillis = millis();

    if (robotWindow.instruct >= 15)
    {
      robotWindow.instruct = 0;
    }
    else
    {
      robotWindow.instruct = robotWindow.instruct + 1;
    }
  }
  /////
  
  //Converts integer value of robot's .instruct into 4-bit binary value
  for (int i = 0; i < NUM_ROBOT_BITS; i++)
  {
    //Performs a Bitwise AND operation on each individual bit of the 4-bit robot instruction number
    robotWindow.outputStateBit[i] = (robotWindow.instruct & (1<<i)) == (1<<i) ? HIGH : LOW;
  }
}

void sendOutputMotor()
{
  //////For testing purposes
  if (millis() - motorMillis > MOTOR_DELAY)
  {
    if (motorWindow.direction == 1)
    {
      digitalWrite(motorWindow.pin2A, HIGH);
      digitalWrite(motorWindow.pin1A, LOW);
    }
    else if (motorWindow.direction == -1)
    {
      digitalWrite(motorWindow.pin2A, LOW);
      digitalWrite(motorWindow.pin1A, HIGH);
    }
    //Set fan speed
    analogWrite(motorWindow.pinEN, motorWindow.speed);
  }
  else
  {
    digitalWrite(motorWindow.pin2A, LOW);
    digitalWrite(motorWindow.pin1A, LOW);
  }
  //////
}

void sendOutputRobot()
{
  for (int i = 0; i < NUM_ROBOT_BITS; i++)
  {
    digitalWrite(robotWindow.outputPinBit[i], robotWindow.outputStateBit[i]);
  }
}