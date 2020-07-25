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
int potValue = 950;
char command;
float pwm1, pwm2, pwm3, pwm4;
int startFlag = 0;
float m = 1.1;
float g = 9.8;
float b = 1.98E-7;
float k = 7.78E-6;

float thrust;
float yaw_pid;
float pitch_pid;
float roll_pid;
float z_pid;

Servo ESC1, ESC2, ESC3, ESC4; //Name the Servo

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
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float setPoints[3] = {0, 0, 0};
float errors[3];
float dErrors[3] = {0, 0, 0};
float iErrors[3] = {0, 0, 0};
float preErrors[3] = {0, 0, 0};
// float prepreErrors[3] = {0, 0, 0};

float K = 256215;
float Ti = 1;
float Td = 5;

float KpRoll = K;
float KiRoll = K/Ti;
float KdRoll = K*Td;

float Kp[3] = {0, KpRoll*2, KpRoll};    // P coefficients in that order : Yaw, Pitch, Roll
float Ki[3] = {0, KiRoll*2, KiRoll};    // I coefficients in that order : Yaw, Pitch, Roll
float Kd[3] = {0, KdRoll*2, KdRoll};        // D coefficients in that order : Yaw, Pitch, Roll

unsigned long StartTime;
unsigned long CurrentTime;
float dt = 0.004;

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
    Serial3.begin(9600);
    Serial.begin(9600);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize Serial3 communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    // Serial3.begin(38400);
    while (!Serial3); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // wait for ready
    while (Serial3.available() && Serial3.read()); // empty buffer

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-82);
    mpu.setYGyroOffset(64);
    mpu.setZGyroOffset(-143);
    mpu.setZAccelOffset(1508);

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

    ESC1.attach( 9,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
    ESC2.attach(10,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
    ESC3.attach(11,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
    ESC4.attach(12,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
     StartTime = micros();

     if( Serial3.available() ) // if data is available to read
     {
         command = Serial3.read();
         Serial.println(command);
         if(command == 'U'){
          potValue += 50;
         }else if(command == 'D'){
          potValue -= 50;
         }else if(command == 'S'){
          potValue = 950;
         }
     }
    
//    ESC1.writeMicroseconds(potValue);
//    ESC2.writeMicroseconds(potValue + 55);
//    ESC3.writeMicroseconds(potValue + 185);
//    ESC4.writeMicroseconds(potValue + 200);

    ESC1.writeMicroseconds(pwm1);
    ESC2.writeMicroseconds(0);
    ESC3.writeMicroseconds(pwm3);
    ESC4.writeMicroseconds(0);

    YPR();
    YPRError();
    PIDController();

    Serial3.print("Yaw: ");
    Serial3.println(ypr[0] * 180/M_PI);
    Serial3.print("Pitch: ");
    Serial3.println(ypr[1] * 180/M_PI);
    Serial3.print("Roll: ");
    Serial3.println(ypr[2] * 180/M_PI);

    Serial3.print("Yaw PID: ");
    Serial3.println(yaw_pid);
    Serial3.print("Pitch PID: ");
    Serial3.println(pitch_pid);
    Serial3.print("Roll PID: ");
    Serial3.println(roll_pid);
    
    Serial3.print("Throttle: ");
    Serial3.println(potValue);
    
    Serial3.print("PWM1: ");
    Serial3.println(pwm1);
    Serial3.print("PWM2: ");
    Serial3.println(pwm2);
    Serial3.print("PWM3: ");
    Serial3.println(pwm3);
    Serial3.print("PWM4: ");
    Serial3.println(pwm4);

    Serial3.println('~');
    
    while((micros() - StartTime) < dt){}
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
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

void YPRError() {
    errors[YAW]   = ypr[YAW]   - setPoints[YAW];
    errors[PITCH] = ypr[PITCH] - setPoints[PITCH];
    errors[ROLL]  = ypr[ROLL]  - setPoints[ROLL];

    iErrors[YAW]   += errors[YAW];
    iErrors[PITCH] += errors[PITCH];
    iErrors[ROLL]  += errors[ROLL];

//    iErrors[YAW]   = minMax(iErrors[YAW],   -400/Ki[YAW],   400/Ki[YAW]);
//    iErrors[PITCH] = minMax(iErrors[PITCH], -400/Ki[PITCH], 400/Ki[PITCH]);
//    iErrors[ROLL]  = minMax(iErrors[ROLL],  -400/Ki[ROLL],  400/Ki[ROLL]);

    dErrors[YAW]   = errors[YAW]   - preErrors[YAW];
    dErrors[PITCH] = errors[PITCH] - preErrors[PITCH];
    dErrors[ROLL]  = errors[ROLL]  - preErrors[ROLL];

    // prepreErrors[YAW]   = preErrors[YAW];
    // prepreErrors[PITCH] = preErrors[PITCH];
    // prepreErrors[ROLL]  = preErrors[ROLL];

    preErrors[YAW]   = errors[YAW];
    preErrors[PITCH] = errors[PITCH];
    preErrors[ROLL]  = errors[ROLL];
}

void PIDController() {
    thrust    = potValue;
    yaw_pid   = 0;
    pitch_pid = 0;
    roll_pid  = 0;
    z_pid     = 4*thrust*thrust;

    // Initialize motor commands with throttle
    pwm1 = thrust;
    pwm2 = thrust;
    pwm3 = thrust;
    pwm4 = thrust;

    // Do not calculate anything if throttle is 0
    if (thrust >= 1000) {
        // PID = e.Kp + ∫e.Ki + Δe.Kd
       yaw_pid   = (errors[YAW]   * Kp[YAW])   + (iErrors[YAW]   * Ki[YAW])   * dt + (dErrors[YAW]   * Kd[YAW])   /dt;
       pitch_pid = (errors[PITCH] * Kp[PITCH]) + (iErrors[PITCH] * Ki[PITCH]) * dt + (dErrors[PITCH] * Kd[PITCH]) /dt;
       roll_pid  = (errors[ROLL]  * Kp[ROLL])  + (iErrors[ROLL]  * Ki[ROLL])  * dt + (dErrors[ROLL]  * Kd[ROLL])  /dt;

       // yaw_pid   = yaw_pid +   Kp[YAW]   * (errors[YAW]   - preErrors[YAW])   + Ki[YAW]   * dt * errors[YAW]   + Kd[YAW]   * (errors[YAW]   - 2*preErrors[YAW]   + prepreErrors[YAW])   /dt;
       // pitch_pid = pitch_pid + Kp[PITCH] * (errors[PITCH] - preErrors[PITCH]) + Ki[PITCH] * dt * errors[PITCH] + Kd[PITCH] * (errors[PITCH] - 2*preErrors[PITCH] + prepreErrors[PITCH]) /dt;
       // roll_pid  = roll_pid +  Kp[ROLL]  * (errors[ROLL]  - preErrors[ROLL])  + Ki[ROLL]  * dt * errors[ROLL]  + Kd[ROLL]  * (errors[ROLL]  - 2*preErrors[ROLL]  + prepreErrors[ROLL])  /dt;

        // Keep values within acceptable range. TODO export hard-coded values in variables/const
//        yaw_pid   = minMax(yaw_pid, -400, 400);
//        pitch_pid = minMax(pitch_pid, -400, 400);
//        roll_pid  = minMax(roll_pid, -400, 400);

        // Calculate pulse duration for each ESC
//        pwm1 = thrust + roll_pid + pitch_pid - yaw_pid;
//        pwm2 = thrust - roll_pid + pitch_pid + yaw_pid;
//        pwm3 = thrust - roll_pid - pitch_pid - yaw_pid;
//        pwm4 = thrust + roll_pid - pitch_pid + yaw_pid;

        pwm1 = 0.5*sqrt( + roll_pid + pitch_pid - yaw_pid + z_pid);
        pwm2 = 0.5*sqrt( - roll_pid + pitch_pid + yaw_pid + z_pid);
        pwm3 = 0.5*sqrt( - roll_pid - pitch_pid - yaw_pid + z_pid);
        pwm4 = 0.5*sqrt( + roll_pid - pitch_pid + yaw_pid + z_pid);
    }

    // Prevent out-of-range-values
    pwm1 = minMax(pwm1, 950, 2000);
    pwm2 = minMax(pwm2, 950, 2000);
    pwm3 = minMax(pwm3, 950, 2000);
    pwm4 = minMax(pwm4, 950, 2000);
}

float minMax(float value, float min, float max) {
    if (value > max) {
        value = max;
    } else if (value < min) {
        value = min;
    }

    return value;
}
