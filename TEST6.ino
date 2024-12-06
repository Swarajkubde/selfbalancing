#include "I2Cdev.h"
#include <PID_v1.h> 
#include "MPU6050_6Axis_MotionApps20.h" 
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

Quaternion q;           
VectorFloat gravity;    
float ypr[3];           

/*********Tune these 4 values for your BOT*********/
double setpoint = 180.5; // Adjust as necessary
double Kp = 36;        // Increased value
double Kd = 1.9;       
double Ki = 145;       
/******End of values setting*********/

double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false;

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Serial.begin(115200);
    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(2497);
    mpu.setYGyroOffset(-388);
    mpu.setZGyroOffset(-223);
    mpu.setZAccelOffset(32767);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    pinMode(6, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
    analogWrite(6, LOW);
    analogWrite(9, LOW);
    analogWrite(10, LOW);
    analogWrite(11, LOW);
}

void loop() {
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {
        pid.Compute();
        Serial.print(input); Serial.print(" =>"); Serial.println(output);

        if (input > 150 && input < 200) { 
            if (output > 0) 
                Forward(); 
            else if (output < 0) 
                Reverse(); 
        } else 
            Stop(); 
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        input = ypr[1] * 180 / M_PI + 180;
    }
    Serial.print("Input: "); Serial.print(input);
Serial.print(" Output: "); Serial.print(output);
Serial.print(" Setpoint: "); Serial.println(setpoint);

}

void Forward() {
    int motorSpeed = constrain(output * 2, 0, 255); // Increase speed
    analogWrite(6, motorSpeed);
    analogWrite(9, 0);
    analogWrite(10, motorSpeed);
    analogWrite(11, 0);
    Serial.print("F");
}

void Reverse() {
    int motorSpeed = constrain(output * -2, 0, 255); // Increase speed
    analogWrite(6, 0);
    analogWrite(9, motorSpeed);
    analogWrite(10, 0);
    analogWrite(11, motorSpeed);
    Serial.print("R");
}

void Stop() {
    analogWrite(6, 0);
    analogWrite(9, 0);
    analogWrite(10, 0);
    analogWrite(11, 0);
    Serial.print("S");
}
