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
  int pinBit[NUM_ROBOT_BITS];
  int stateBit[NUM_ROBOT_BITS]; 
  int instruct;
};


//Initalize sensor objects
Sensor sensorSprayer = {.pin = 2, .state = HIGH};
Sensor sensorSqueegee = {.pin = 3, .state = HIGH};
Sensor sensorCloth = {.pin = 4, .state = HIGH};
Sensor sensorPosMin = {.pin = 5, .state = LOW};
Sensor sensorPosMax = {.pin = 6, .state = LOW};

//Initialize motor object
Motor motorWindow = {.pin2A = 7, .pin1A = 8, .pinEN = 9, .speed = 0, .direction = 1, .instruct = 0};
//Const speed for motor operation
const int MOTOR_RUN_SPEED = 200;

//////THESE ARE FOR TESTING
unsigned long motorMillis = 0;
const unsigned long MOTOR_DELAY = 1000;

unsigned long robotInstructMillis = 0;
const unsigned long ROBOT_INSTRUCT_DELAY = 200;
//////

//Initialize robot object
Robot robotWindow = {.pinBit = {10, 11, 12, 13}, .stateBit = {LOW}, .instruct = 0};

//Store sensor input data
void readInputSensor();
//Send stored sensor input data to PLC
void sendOutputDataPLC();
//Set states of motor's outputs
void setOutputMotor(Motor motorObject);
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
  pinMode(motorWindow.pin2A, OUTPUT);
  pinMode(motorWindow.pin1A, OUTPUT);
  pinMode(motorWindow.pinEN, OUTPUT);
}

void loop() {
  readInputSensor();
  setOutputMotor();
  sendOutputMotor();
  setOutputRobot();
  sendOutputRobot();
}

void readInputSensor()
{
  sensorSprayer.state = digitalRead(sensorSprayer.pin);
  sensorSqueegee.state = digitalRead(sensorSqueegee.pin);
  sensorCloth.state = digitalRead(sensorCloth.pin);
  sensorPosMin.state = digitalRead(sensorPosMin.pin);
  sensorPosMax.state = digitalRead(sensorPosMax.pin);
}

void setOutputMotor()
{
  //NOTE the sensors used have normally HIGH outputs, when it senses something, it goes to LOW
  //Both Min and Max sensors are active
  if (sensorPosMin.state == LOW && sensorPosMax.state == LOW)
  {
    motorWindow.speed = 0;
  }
  //Min sensor is active
  else if (sensorPosMin.state == LOW)
  {
    motorWindow.speed = MOTOR_RUN_SPEED;
    if (motorWindow.direction == -1)
    {
      motorMillis = millis();
    }
    motorWindow.direction = 1;
    
  }
  //Max sensor is active
  else if (sensorPosMax.state == LOW)
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
    //Left shift operation (<<) shifts the binary value each loop iteration: 1(0001), 2(0010), 4(0100), 8(1000)
    //i.e. (14 & (1<<3)) == (1<<3) expressed in binary is (1110 & 1000) == (1000)
    //Sets the respective stateBit based on the state of the bit checked
    robotWindow.stateBit[i] = (robotWindow.instruct & (1<<i)) == (1<<i) ? HIGH : LOW;
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
    digitalWrite(robotWindow.pinBit[i], robotWindow.stateBit[i]);
  }
}