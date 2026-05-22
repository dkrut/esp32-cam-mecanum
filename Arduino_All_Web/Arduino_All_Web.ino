//www.kuongshun.com

#include    <Servo.h>

#define MOTOR_PIN           9
#define PWM1_PIN            5
#define PWM2_PIN            6
#define SHCP_PIN            2                  // The displacement of the clock
#define EN_PIN              7                  // Can make control
#define DATA_PIN            8                  // Serial data
#define STCP_PIN            4                  // Memory register clock            
#define Trig_PIN            12
#define Echo_PIN            13
#define LEFT_LINE_TRACJING      A2
#define CENTER_LINE_TRACJING    A1
#define right_LINE_TRACJING     A0

Servo MOTORservo;

const int Forward       = 163;                 // forward
const int Backward      = 92;                  // back
const int Turn_Left     = 106;                 // left translation
const int Turn_Right    = 149;                 // Right translation
const int Top_Left      = 34;                  // Upper left mobile
const int Bottom_Left   = 72;                  // Lower left mobile
const int Top_Right     = 129;                 // Upper right mobile
const int Bottom_Right  = 20;                  // The lower right move
const int Stop          = 0;                   // stop
const int Contrarotate  = 83;                  // Counterclockwise rotation
const int Clockwise     = 172;                 // Rotate clockwise
const int Model1        = 25;                  // model1
const int Model2        = 26;                  // model2
const int Model3        = 27;                  // model3
const int Model4        = 28;                  // model4
const int Speed         = 29;
const int Servo         = 30; 
const int MotorLeft     = 230;                 // servo turn left
const int MotorRight    = 231;                 // servo turn right

int Left_Tra_Value;
int Center_Tra_Value;
int Right_Tra_Value;
int Black_Line = 400;
int speeds = 250;
int leftDistance = 0;
int middleDistance = 0;
int rightDistance = 0;

byte RX_package[4] = {0};
uint16_t angle = 90;
byte Model = Stop;
byte val = 0;
char model_var = 0;
int UT_distance = 0;
int OA_mark=0;
void setup()
{
  Serial.setTimeout(10);
  Serial.begin(115200);

  MOTORservo.attach(MOTOR_PIN);

  pinMode(SHCP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(STCP_PIN, OUTPUT);
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);

  pinMode(Trig_PIN, OUTPUT);
  pinMode(Echo_PIN, INPUT);

  pinMode(LEFT_LINE_TRACJING, INPUT);
  pinMode(CENTER_LINE_TRACJING, INPUT);
  pinMode(right_LINE_TRACJING, INPUT);

  MOTORservo.write(angle);

  Motor(Stop, 0);
}

void loop()
{
  RXpack_func();
  switch (model_var)
  {
    case 0:
      model1_func(val);
      break;
    case 1:
      model2_func();      // OA model
      break;
    case 2:
      model3_func();      // follow model
      break;
    case 3:
      model4_func();      // Tracking model
      break;
    case 4:
      Servo_Move();
      break;
  }

}

void model1_func(byte values)
{
  switch (values)
  {
    case Stop:
      Motor(Stop, 0);
      break;
    case Forward:
      Motor(Forward, speeds);
      break;
    case Backward:
      Motor(Backward, speeds);
      break;
    case Turn_Left:
      Motor(Turn_Left, speeds);
      break;
    case Turn_Right:
      Motor(Turn_Right, speeds);
      break;
    case Top_Left:
      Motor(Top_Left, speeds);
      break;
    case Top_Right:
      Motor(Top_Right, speeds);
      break;
    case Bottom_Left:
      Motor(Bottom_Left, speeds);
      break;
    case Bottom_Right:
      Motor(Bottom_Right, speeds);
      break;
    case Clockwise:
      Motor(Clockwise, speeds);
      break;
    case Contrarotate:
      Motor(Contrarotate, speeds);
      break;
    default:
      // Serial.println(".");
      Model = 0;
      Motor(Stop, 0);
      break;
  }
}

void model2_func()
{
  static int oa_step = 0;
  static unsigned long step_start = 0;
  
  if (model_var != 1) { oa_step = 0; return; }
  
  unsigned long now = millis();
  middleDistance = SR04(Trig_PIN, Echo_PIN);
  
  switch (oa_step) {
    case 0:
      if (middleDistance <= 15) {
        Motor(Stop, 0);
        step_start = now;
        oa_step = 1;
      } else {
        Motor(Forward, speeds);
      }
      break;

    case 1:
      if (now - step_start >= 100) {
        Motor(Backward, speeds - 70);
        step_start = now;
        oa_step = 2;
      }
      break;

    case 2:
      if (now - step_start >= 100) {
        Motor(Clockwise, speeds);
        step_start = now;
        oa_step = 3;
      }
      break;

    case 3:
      if (now - step_start >= 200) {
        Motor(Stop, 0);
        step_start = now;
        oa_step = 4;
      }
      break;

    case 4:
      if (now - step_start >= 200) {
        rightDistance = SR04(Trig_PIN, Echo_PIN);
        step_start = now;
        oa_step = 5;
      }
      break;

    case 5:
      if (now - step_start >= 200) {
        if (rightDistance > 15) {
          oa_step = 0;
        } else if (OA_mark == 0) {
          OA_mark++;
          oa_step = 0;
        } else {
          Motor(Contrarotate, speeds);
          step_start = now;
          oa_step = 6;
        }
      }
      break;

    case 6:
      if (now - step_start >= 600) {
        Motor(Stop, 0);
        step_start = now;
        oa_step = 7;
      }
      break;

    case 7:
      if (now - step_start >= 200) {
        leftDistance = SR04(Trig_PIN, Echo_PIN);
        step_start = now;
        oa_step = 8;
      }
      break;

    case 8:
      if (now - step_start >= 200) {
        if (rightDistance > leftDistance) {
          Motor(Backward, speeds - 70);
          step_start = now;
          oa_step = 9;
        } else {
          Motor(Contrarotate, speeds);
          step_start = now;
          oa_step = 11;
        }
      }
      break;

    case 9:
      if (now - step_start >= 500) {
        Motor(Clockwise, speeds);
        step_start = now;
        oa_step = 10;
      }
      break;

    case 10:
      if (now - step_start >= 500) {
        OA_mark = 0;
        oa_step = 0;
      }
      break;

    case 11:
      if (now - step_start >= 200) {
        Motor(Stop, 0);
        step_start = now;
        oa_step = 12;
      }
      break;

    case 12:
      if (now - step_start >= 200) {
        leftDistance = SR04(Trig_PIN, Echo_PIN);
        step_start = now;
        oa_step = 13;
      }
      break;

    case 13:
      if (now - step_start >= 200) {
        if (leftDistance < 25) {
          Motor(Contrarotate, speeds);
          step_start = now;
          oa_step = 11;
        } else {
          Motor(Contrarotate, speeds);
          step_start = now;
          oa_step = 14;
        }
      }
      break;

    case 14:
      if (now - step_start >= 100) {
        OA_mark = 0;
        oa_step = 0;
      }
      break;
  }
}

void model3_func()      // follow model
{
  MOTORservo.write(90);
  UT_distance = SR04(Trig_PIN, Echo_PIN);
  if (UT_distance < 15)
  {
    Motor(Backward, speeds-50);
  }
  else if (15 <= UT_distance && UT_distance <= 20)
  {
    Motor(Stop, 0);
  }
  else if (20 <= UT_distance && UT_distance <= 25)
  {
    Motor(Forward, speeds-70);
  }
  else if (25 <= UT_distance && UT_distance <= 50)
  {
    Motor(Forward, speeds-30);
  }
  else
  {
    Motor(Stop, 0);
  }
}

void model4_func()      // tracking model
{
  MOTORservo.write(90);
  Left_Tra_Value = analogRead(LEFT_LINE_TRACJING);
  Center_Tra_Value = analogRead(CENTER_LINE_TRACJING);
  Right_Tra_Value = analogRead(right_LINE_TRACJING);
  if (Left_Tra_Value > Black_Line && Center_Tra_Value <= Black_Line && Right_Tra_Value > Black_Line) 
  {
    Motor(Forward, 180);
  } 
  else if (Left_Tra_Value < Black_Line && Center_Tra_Value < Black_Line && Right_Tra_Value < Black_Line) 
  {
    Motor(Forward, 130);
  } 
  else if (Left_Tra_Value >= Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line) 
  {
    Motor(Clockwise, 220);
  } 
  else if (Left_Tra_Value >= Black_Line && Center_Tra_Value < Black_Line && Right_Tra_Value < Black_Line) 
  {
    Motor(Clockwise, 160);
  } 
  else if (Left_Tra_Value < Black_Line && Center_Tra_Value < Black_Line && Right_Tra_Value >= Black_Line) 
  {
    Motor(Contrarotate, 160);
  } 
  else if (Left_Tra_Value < Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value >= Black_Line) 
  {
    Motor(Contrarotate, 220);
  } 
  else if (Left_Tra_Value >= Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value >= Black_Line) 
  {
    Motor(Forward, 130);
  }
}

void Servo_Move()  //servo
{
  static uint16_t current_angle = 90;
  static uint16_t target_angle = 90;
  static unsigned long last_move_time = 0;
  const int STEP_DELAY = 10;
  const int HYSTERESIS = 1;
  
  uint16_t safe_angle = (angle == 0) ? 1 : angle;
  if (safe_angle == 255) return;
  
  if (safe_angle >= 1 && safe_angle <= 180) {
    if (abs((int16_t)safe_angle - (int16_t)target_angle) > HYSTERESIS) {
      target_angle = safe_angle;
    }
  }
  
  if (current_angle != target_angle) {
    if (millis() - last_move_time >= STEP_DELAY) {
      if (current_angle < target_angle) {
        current_angle++;
        if (current_angle > target_angle) current_angle = target_angle;
      } else if (current_angle > target_angle) {
        current_angle--;
        if (current_angle < target_angle) current_angle = target_angle;
      }
      
      current_angle = constrain(current_angle, 1, 180);
      MOTORservo.write(current_angle);
      last_move_time = millis();
    }
  }
}

void Motor(int Dir, int Speed)      // motor drive
{
  digitalWrite(EN_PIN, LOW);
  analogWrite(PWM1_PIN, Speed);
  analogWrite(PWM2_PIN, Speed);

  digitalWrite(STCP_PIN, LOW);
  shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, Dir);
  digitalWrite(STCP_PIN, HIGH);
}

float SR04(int Trig, int Echo)      // ultrasonic measured distance
{
  static float filtered = 0;
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  float distance = pulseIn(Echo, HIGH, 23200) / 58.00;

  if (distance == 0 || distance > 400) return filtered;

  filtered = filtered * 0.7 + distance * 0.3;
  return filtered;
}

void RXpack_func()  //Receive data
{
  if (Serial.available() > 0)
  {
    if (Serial.readBytes(RX_package, 4) == 4)
    {
      if (RX_package[0] == 0xAB && RX_package[3] == 0xFF)     // The header and tail of the packet are verified
      {
        //Serial.write(RX_package, 4);
        Model = RX_package[1];
        val = RX_package[2];
        if (Model == Speed)
        {
          speeds = val;
        }
        else if (Model == Model1)
        {
          model_var = 0;
        }
        else if (Model == Model2)
        {
          model_var = 1;
        }
        else if (Model == Model3)
        {
          model_var = 2;
        }
        else if (Model == Model4)
        {
          model_var = 3;
        }
        else if (Model == Servo)
        {
          angle = val;
          model_var = 4;
        }
      }
    }
  }
}
