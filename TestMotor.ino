//Const speed for motor operation (Range 0-255)
const int MOTOR_RUN_SPEED = 255;
//For tracking motor delay
unsigned long motorMillis = 0;
const unsigned long MOTOR_DELAY = 700;

unsigned long runMillis = 0;
bool direction = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  motorMillis = millis();
  runMillis = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (millis() - motorMillis > MOTOR_DELAY)
  {
    if (millis() - runMillis < 1000)
    {
      if (direction == true)
      {
        analogWrite(10, MOTOR_RUN_SPEED);
        digitalWrite(8, true);
        digitalWrite(9, false);
      }
      else
      {
        analogWrite(10, MOTOR_RUN_SPEED);
        digitalWrite(8, false);
        digitalWrite(9, true);
      }
    }
    else
    {
      direction = direction == true ? false : true;
      motorMillis = millis();
    }
  }
  else
  {
    runMillis = millis();
  }
}
