/*
Reads sensor inputs and outputs them to a PLC program.
Then receives instructions from PLC to control a DC motor and robot arm.
*/

struct Sensor 
{
  int pin;
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
};

struct Robot
{
  int signalInput1;
};

//Initalize sensor objects
Sensor sensorSprayer = {.pin = 2, .state = HIGH};
Sensor sensorSqueegee = {.pin = 3, .state = HIGH};
Sensor sensorCloth = {.pin = 4, .state = HIGH};
Sensor sensorPosMin = {.pin = 5, .state = LOW};
Sensor sensorPosMax = {.pin = 6, .state = LOW};

//Initialize motor object
Motor motorWindow = {.pin2A = 7, .pin1A = 8, .pinEN = 9, .speed = 0, .direction = 1};

//////THESE ARE FOR TESTING
unsigned long motorMillis = 0;
const unsigned long MOTOR_DELAY = 1000;
//////

//Store sensor input data
void readInputSensor();
//Send stored sensor input data to PLC
void sendOutputDataPLC();
//Set states of outputs
void setOutputInstruction();
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
}

void readInputSensor()
{
  sensorSprayer.state = digitalRead(sensorSprayer.pin);
  sensorSqueegee.state = digitalRead(sensorSqueegee.pin);
  sensorCloth.state = digitalRead(sensorCloth.pin);
  sensorPosMin.state = digitalRead(sensorPosMin.pin);
  sensorPosMax.state = digitalRead(sensorPosMax.pin);
}

void setOutputInstruction()
{
  if (sensorPosMin.state == HIGH && sensorPosMax.state == HIGH)
  {
    motorWindow.speed = 0;
  }
  else if (sensorPosMin.state == HIGH)
  {
    motorWindow.speed = 200;
    if (motorWindow.direction == -1)
    {
      motorMillis = millis();
    }
    motorWindow.direction = 1;
    
  }
  else if (sensorPosMax.state == HIGH)
  {
    motorWindow.speed = 200;
    if (motorWindow.direction == 1)
    {
      motorMillis = millis();
    }
    motorWindow.direction = -1;
  }
  else
  {
    motorWindow.speed = 200;
  }
}

void sendOutputMotor()
{
  //////
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