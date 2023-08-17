#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_ABS_SPEED 30

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

//PID
double originalSetpoint = 172.50;  
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

                                        //OUR PID Constansts
                                        
double Kp = 35;     
double Kd = 2.0;  
double Ki = 180;  
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.6;
double motorSpeedFactorRight = 0.5;

                                      //MOTOR CONTROLLER                     
int IN1 = 6;
int IN2 = 7;
int IN3 = 9;
int IN4 = 8;
int ENA = 5;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
 mpuInterrupt = true;
}


void setup()
{
 // join I2C bus (I2Cdev library doesn't do this automatically)
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 Wire.begin();
 TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
 #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
 Fastwire::setup(400, true);
 #endif

 mpu.initialize();

 devStatus = mpu.dmpInitialize();

 // supply  gyro offsets here, scaled for min sensitivity
 mpu.setXGyroOffset(220);
 mpu.setYGyroOffset(76);
 mpu.setZGyroOffset(-85);
 mpu.setZAccelOffset(1788); 


 if (devStatus == 0)
 {
 // turn on the DMP, now that it's ready
 mpu.setDMPEnabled(true);

 // enable Arduino interrupt detection
 attachInterrupt(0, dmpDataReady, RISING);
 mpuIntStatus = mpu.getIntStatus();

 // set our DMP Ready flag so the main loop() function knows it's okay to use it
 dmpReady = true;

 // get expected DMP packet size for later comparison
 packetSize = mpu.dmpGetFIFOPacketSize();
 
 //setup PID
 pid.SetMode(AUTOMATIC);
 pid.SetSampleTime(10);
 pid.SetOutputLimits(-255, 255); 
 }
 else
 {
 
 Serial.print(F("DMP Initialization failed (code "));
 Serial.print(devStatus);
 Serial.println(F(")"));
 }
}


void loop()
{
 
 if (!dmpReady) return;

 
 while (!mpuInterrupt && fifoCount < packetSize)
 {
 //no mpu data - performing PID calculations and output to motors 
 pid.Compute();
 motorController.move(output, MIN_ABS_SPEED);
 
 }


 mpuInterrupt = false;
 mpuIntStatus = mpu.getIntStatus();

 
 fifoCount = mpu.getFIFOCount();

 
 if ((mpuIntStatus & 0x10) || fifoCount == 1024)
 {

 mpu.resetFIFO();
 Serial.println(F("FIFO overflow!"));

 
 }
 else if (mpuIntStatus & 0x02)
 {
 
 while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();


 mpu.getFIFOBytes(fifoBuffer, packetSize);
 
 fifoCount -= packetSize;

 mpu.dmpGetQuaternion(&q, fifoBuffer);
 mpu.dmpGetGravity(&gravity, &q);
 mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
 input = ypr[1] * 180/M_PI + 180;
 }
}
