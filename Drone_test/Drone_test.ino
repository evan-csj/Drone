#include <Wire.h>
#include <Servo.h>

#define ROLL  0
#define PITCH 1
#define YAW   2

Servo ESC1, ESC2, ESC3, ESC4;
float pwm1 = 0, pwm2 = 0, pwm3 = 0, pwm4 = 0, pwm = 950;

int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;

int r1 = 0, r2 = 0, r3 = 0, r4 = 0;
bool stopFlag = false;
float voltageSensor, voltage;
const float factor = 11.87/2.36;
const float vcc = 5.13;

float Acceleration_angle[3];
float Gyro_angle[3];
float Total_angle[3];
float setPoints[3] = {0, 0, 0};
float error[3];
float prevError[3];

float elapsedTime, time, timePrev;
int i;
float rad2deg = 180/M_PI;
float PID;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;

double kp = 3.55;
double ki = 0.005;
double kd = 2.05;

double throttle;
char command;

void setup() {
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
  Serial3.begin(115200);
  
  ESC1.attach( 8,1000,2000);
  ESC2.attach( 9,1000,2000);
  ESC3.attach(10,1000,2000);
  ESC4.attach(11,1000,2000);

  time = micros();
  
  ESC1.writeMicroseconds(pwm);
  ESC2.writeMicroseconds(pwm);
  ESC3.writeMicroseconds(pwm);
  ESC4.writeMicroseconds(pwm);
  delay(7000);
}

void loop() {
  timePrev = time;
  time = micros();
  elapsedTime = (time - timePrev) / 1000000;

  if( Serial3.available() ) // if data is available to read
     {
         command = Serial3.read();
         switch(command){
          case 'A':
            if(pwm < 1200) pwm += 50;
            else if(pwm < 1450) pwm += 10;
            else pwm += 1;
            stopFlag = false;
            break;
          case 'V':
            if(pwm < 1000) pwm = 950;
            else if(pwm <= 1200) pwm -= 50;
            else if(pwm <= 1450) pwm -= 10;
            else pwm -= 1;
            break;
//          case 'C':
//            setPoints[ROLL] = 0;
//            setPoints[PITCH] = 0;
//            break;
          case 'S': stopFlag = true; break;
//          case 'F': setPoints[PITCH] += 10; break;
//          case 'B': setPoints[PITCH] -= 10; break;
//          case 'L': setPoints[ROLL] += 10; break;
//          case 'R': setPoints[ROLL] -= 10; break;
          case 11: r1++; break;
          case 10: r1--; break;
          case 21: r2++; break;
          case 20: r2--; break;
          case 31: r3++; break;
          case 30: r3--; break;
          case 41: r4++; break;
          case 40: r4--; break;
//          case 'P': 
//            K += 10; PIDUpdate(); break;
//          case 'I': Ti += 10; PIDUpdate(); break;
//          case 'D': Td += 0.1; PIDUpdate(); break;
//          case 'p': 
//            if(K > 0){
//              K -= 10;
//              PIDUpdate(); 
//            }
//            break;
//          case 'i': Ti -= 10; PIDUpdate(); break;
//          case 'd':
//            if(Td > 0){
//              Td -= 0.1; 
//              PIDUpdate(); 
//            }
//            break;  
         }
     }

  if(pwm > 950 && stopFlag){
    pwm -= 1;
    if(pwm < 950) pwm = 950;
  }
     
  ESC1.writeMicroseconds(pwm1);
  ESC3.writeMicroseconds(pwm3);
  
  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);

  Acc_rawX = Wire.read()<<8 | Wire.read();
  Acc_rawY = Wire.read()<<8 | Wire.read();
  Acc_rawZ = Wire.read()<<8 | Wire.read();

  Acceleration_angle[ROLL] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad2deg;
  Acceleration_angle[PITCH] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad2deg;
  Acceleration_angle[YAW] = atan(sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2))/(Acc_rawZ/16384.0))*rad2deg;

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);

  Gyr_rawX = Wire.read()<<8 | Wire.read();
  Gyr_rawY = Wire.read()<<8 | Wire.read();
  Gyr_rawZ = Wire.read()<<8 | Wire.read();

  Gyro_angle[ROLL]  = Gyr_rawX/131.0;
  Gyro_angle[PITCH] = Gyr_rawY/131.0;
  Gyro_angle[YAW]   = Gyr_rawZ/131.0;

  Total_angle[ROLL]  = 0.98*(Total_angle[ROLL]  + Gyro_angle[ROLL]*elapsedTime)  + 0.02*Acceleration_angle[ROLL];
  Total_angle[PITCH] = 0.98*(Total_angle[PITCH] + Gyro_angle[PITCH]*elapsedTime) + 0.02*Acceleration_angle[PITCH];
  Total_angle[YAW]   = 0.98*(Total_angle[YAW]   + Gyro_angle[YAW]*elapsedTime)   + 0.02*Acceleration_angle[YAW];

  error[PITCH] = Total_angle[PITCH] - setPoints[PITCH];
  
  pid_p = kp*error[PITCH];
  
  if(-3 < error[PITCH] < 3){
    pid_i = pid_i + ki*error[PITCH];
  }

  pid_d = kd*((error[PITCH] - prevError[PITCH])/elapsedTime);

  PID = pid_p + pid_i + pid_d;
  PID = minMax(PID, -1000, 1000);

  throttle = pwm;

  pwm1 = throttle - PID;
  pwm3 = throttle + PID;

  pwm1 = minMax(pwm1, 1000, 2000);
  pwm3 = minMax(pwm3, 1000, 2000);

  voltageSensor = analogRead(A0);
  voltage = voltageSensor / 1024 * vcc * factor * 100 + 5;

  Serial3.print("R: ");
  Serial3.print(Total_angle[ROLL]);
  Serial3.print(" P: ");
  Serial3.println(Total_angle[PITCH]);
  Serial3.print(" Y: ");
  Serial3.println(Total_angle[YAW]);

  Serial3.print("R1234: ");
  Serial3.print(r1);
  Serial3.print(" ");
  Serial3.print(r2);
  Serial3.print(" ");
  Serial3.print(r3);
  Serial3.print(" ");
  Serial3.println(r4);
    
  Serial3.print("PWM: ");
  Serial3.println(pwm);
  Serial3.println(pwm1);
  Serial3.println(pwm3);
  
  Serial3.print("V: ");
  Serial3.println(voltage/100);
  Serial3.println('~');
}

float minMax(float value, float min, float max) {
  if (value > max) {
      value = max;
  } else if (value < min) {
      value = min;
  }
  return value;
}
