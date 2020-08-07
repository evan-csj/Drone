// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <Servo.h>

Servo ESC1, ESC2, ESC3, ESC4;
float pwm1 = 0, pwm2 = 0, pwm3 = 0, pwm4 = 0;
int pwm = 950;
int r1 = 15, r2 = 6, r3 = -15, r4 = 4;
char command;
bool stopFlag = false;
float Crotor;
float voltageSensor, voltage;
const float factor = 11.87/2.36;
const float vcc = 5.13;

float Ixx = 0.0118;
float Iyy = 0.0118;
float Izz = 0.0225;

float m = 1.1;
float g = 9.8;
float b = 1.84E-7;
float k = 3.24E-8;
float L = 0.160;

float thrust;
float yaw_pid;
float pitch_pid;
float roll_pid;
float z_pid;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
// MPU6050 mpu(0x69); // <-- use for AD0 high

#define OUTPUT_READABLE_YAWPITCHROLL

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 gyro;
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float setPoints[3] = {0, 0, 0};
float errors[3];
float dErrors[3] = {0, 0, 0};
float iErrors[3] = {0, 0, 0};
float preErrors[3] = {0, 0, 0};

float K = 0;
float Ti = 0;
float Td = 0;

float KpRoll = K;
float KiRoll = 0;
float KdRoll = K*Td;

//float KpRoll = 1.3;
//float KiRoll = 0.04;
//float KdRoll = 18;

float KpPitch = KpRoll;
float KiPitch = KiRoll;
float KdPitch = KdRoll;

float KpYaw = 0;
float KiYaw = 0;
float KdYaw = 0;

unsigned long StartTime;
unsigned long CurrentTime;
float dt = 0.009;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    Serial3.begin(115200);
    Serial.begin(9600);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize Serial3 communication
    // (9600 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    while (!Serial3); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // wait for ready
    while (Serial3.available() && Serial3.read()); // empty buffer

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(73);
    mpu.setYGyroOffset(-63);
    mpu.setZGyroOffset(-25);
    mpu.setZAccelOffset(1506);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    ESC1.attach(8,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
    ESC2.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
    ESC3.attach(11,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
    ESC4.attach(10,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
     StartTime = micros();

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
          case 'C':
            setPoints[ROLL] = 0;
            setPoints[PITCH] = 0;
            break;
          case 'S': stopFlag = true; break;
          case 'F': setPoints[PITCH] += 10; break;
          case 'B': setPoints[PITCH] -= 10; break;
          case 'L': setPoints[ROLL] += 10; break;
          case 'R': setPoints[ROLL] -= 10; break;
          case 11: r1++; break;
          case 10: r1--; break;
          case 21: r2++; break;
          case 20: r2--; break;
          case 31: r3++; break;
          case 30: r3--; break;
          case 41: r4++; break;
          case 40: r4--; break;
          case 'P': 
            K += 1; PIDUpdate(); break;
          case 'I': Ti += 10; PIDUpdate(); break;
          case 'D': Td += 0.1; PIDUpdate(); break;
          case 'p': 
            if(K > 0){
              K -= 1;
              PIDUpdate(); 
            }
            break;
          case 'i': Ti -= 10; PIDUpdate(); break;
          case 'd':
            if(Td > 0){
              Td -= 0.1; 
              PIDUpdate(); 
            }
            break;  
         }
     }

     // Rotor Protection
     if(abs(ypr[PITCH]) + abs(ypr[ROLL]) > 0.9){
      pwm = 950;
     }
     
     if(pwm > 950 && stopFlag){
      pwm -= 1;
      if(pwm < 950) pwm = 950;
     }

    ESC1.writeMicroseconds(pwm1 + r1);
//    ESC2.writeMicroseconds(pwm2 + r2);
    ESC3.writeMicroseconds(pwm3 + r3);
//    ESC4.writeMicroseconds(pwm4 + r4);

//    ESC1.writeMicroseconds(0);
    ESC2.writeMicroseconds(0);
//    ESC3.writeMicroseconds(0);
    ESC4.writeMicroseconds(0);

//    ESC1.writeMicroseconds(pwm + r1);
//    ESC2.writeMicroseconds(pwm + r2);
//    ESC3.writeMicroseconds(pwm + r3);
//    ESC4.writeMicroseconds(pwm + r4);

    voltageSensor = analogRead(A0);
    voltage = voltageSensor / 1024 * vcc * factor * 100 + 5;
    
    YPR();
    YPRError();
    PIDController();
    
    Serial3.print("Y: ");
    Serial3.println(ypr[YAW] * 180/M_PI);
    Serial3.print("P: ");
    Serial3.println(ypr[PITCH] * 180/M_PI);
    Serial3.print("R: ");
    Serial3.println(ypr[ROLL] * 180/M_PI);

    Serial3.print("R1234: ");
    Serial3.print(r1);
    Serial3.print(" ");
    Serial3.print(r2);
    Serial3.print(" ");
    Serial3.print(r3);
    Serial3.print(" ");
    Serial3.println(r4);

    Serial3.print("PID: ");
    Serial3.print(KpRoll);
    Serial3.print(" ");
    Serial3.print(KiRoll);
    Serial3.print(" ");
    Serial3.println(KdRoll);
    
    Serial3.print("PWM: ");
    Serial3.println(pwm);

    Serial3.print("V: ");
    Serial3.println(voltage/100);

    Serial3.print("Loop time: ");
    Serial3.println(micros() - StartTime);
    Serial3.println('~');
//    while((micros() - StartTime) < (dt*1000000)){}
}

void YPR() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            ypr[YAW] = -ypr[YAW];
            ypr[ROLL] = -ypr[ROLL];
        #endif

        // blink LED to indicate activity
//        blinkState = !blinkState;
//        digitalWrite(LED_PIN, blinkState);
    }
}

void YPRError() {
    errors[YAW]   = setPoints[YAW]   - ypr[YAW];
    errors[PITCH] = setPoints[PITCH] - ypr[PITCH];
    errors[ROLL]  = setPoints[ROLL]  - ypr[ROLL];

//    errors[YAW]   = ypr[YAW]   - setPoints[YAW];
//    errors[PITCH] = ypr[PITCH] - setPoints[PITCH];
//    errors[ROLL]  = ypr[ROLL]  - setPoints[ROLL];
    
    iErrors[YAW]   += errors[YAW];
    iErrors[PITCH] += errors[PITCH];
    iErrors[ROLL]  += errors[ROLL];

//    iErrors[YAW]   = minMax(iErrors[YAW],   -400/KiYaw,   400/KiYaw);
//    iErrors[PITCH] = minMax(iErrors[PITCH], -400/KiPitch, 400/KiPitch);
//    iErrors[ROLL]  = minMax(iErrors[ROLL],  -400/KiRoll,  400/KiRoll);

    dErrors[YAW]   = errors[YAW]   - preErrors[YAW];
    dErrors[PITCH] = errors[PITCH] - preErrors[PITCH];
    dErrors[ROLL]  = errors[ROLL]  - preErrors[ROLL];

//    prepreErrors[YAW]   = preErrors[YAW];
//    prepreErrors[PITCH] = preErrors[PITCH];
//    prepreErrors[ROLL]  = preErrors[ROLL];

    preErrors[YAW]   = errors[YAW];
    preErrors[PITCH] = errors[PITCH];
    preErrors[ROLL]  = errors[ROLL];
}

void PIDController() {
//    Crotor    = 0.0058*pwm - 3.9 + 0.16*(voltage - 1110)/100;
//    Crotor    = minMax(Crotor, 3, 4);
    Crotor    = 1;
    thrust    = Crotor*pwm;
    yaw_pid   = 0;
    pitch_pid = 0;
    roll_pid  = 0;
    z_pid     = 4*(thrust)*(thrust);

    // Initialize motor commands with throttle
//    pwm1 = thrust;
//    pwm2 = thrust;
//    pwm3 = thrust;
//    pwm4 = thrust;

    // Do not calculate anything if throttle is 0
    if (thrust >= 950) {
        // PID = e.Kp + ∫e.Ki + Δe.Kd
//        yaw_pid   = (errors[YAW]   * KpYaw)   + (iErrors[YAW]   * KiYaw)   * dt + (dErrors[YAW]   * KdYaw)   /dt;
        pitch_pid = (errors[PITCH] * KpPitch) + (iErrors[PITCH] * KiPitch) * dt + (dErrors[PITCH] * KdPitch) /dt;
        roll_pid  = (errors[ROLL]  * KpRoll)  + (iErrors[ROLL]  * KiRoll)  * dt + (dErrors[ROLL]  * KdRoll)  /dt;

        // Keep values within acceptable range. TODO export hard-coded values in variables/const
//        yaw_pid   = minMax(yaw_pid, -400, 400);
//        pitch_pid = minMax(pitch_pid, -400, 400);
//        roll_pid  = minMax(roll_pid, -400, 400);

        // Calculate pulse duration for each ESC
        pwm1 = - 2*pitch_pid + yaw_pid + thrust;
        pwm2 = - 2*roll_pid  - yaw_pid + thrust;
        pwm3 = + 2*pitch_pid + yaw_pid + thrust;
        pwm4 = + 2*roll_pid  - yaw_pid + thrust;

//        pwm1 = 0.5*sqrt( + roll_pid + pitch_pid - yaw_pid + z_pid);
//        pwm2 = 0.5*sqrt( - roll_pid + pitch_pid + yaw_pid + z_pid);
//        pwm3 = 0.5*sqrt( - roll_pid - pitch_pid - yaw_pid + z_pid);
//        pwm4 = 0.5*sqrt( + roll_pid - pitch_pid + yaw_pid + z_pid);
    }
    
    pwm1 = minMax(pwm1, 950, 2000);
    pwm2 = minMax(pwm2, 950, 2000);
    pwm3 = minMax(pwm3, 950, 2000);
    pwm4 = minMax(pwm4, 950, 2000);
}

void PIDUpdate(){
  KpRoll = K;
  KiRoll = 0;
  KdRoll = K*Td;
  
  KpPitch = KpRoll;
  KiPitch = KiRoll;
  KdPitch = KdRoll;
  return 0;
}

float minMax(float value, float min, float max) {
    if (value > max) {
        value = max;
    } else if (value < min) {
        value = min;
    }

    return value;
}
