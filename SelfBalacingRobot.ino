#define version_0.66

#include <I2Cdev.h>
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <helper_3dmath.h>
#include <PID_v1.h>                              
#include <digitalIOPerformance.h>               
#define DIGITALIO_NO_INTERRUPT_SAFETY
#define DIGITALIO_NO_MIX_ANALOGWRITE

#define RESTRICT_PITCH

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
int16_t gyro[3]; 
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#define CW   1
#define CCW  2

#define LED 13
#define SPD_INT_L 3//1 // encoder kiri 1
#define SPD_PUL_L 11 // encoder kiri 2
#define SPD_INT_R 2//7 4 // encoder kanan 1
#define SPD_PUL_R 10 // encoder kanan 2
#define MPU_INT 2//0 // mpu int

// Balance PID controller Definitions
#define BALANCE_KP  32 //32 //135.8 //12 //15                   
#define BALANCE_KI  90 //90 //812.7 //100 //90
#define BALANCE_KD  0.5 //0.5 //5.7 //1 //0.8
#define BALANCE_PID_MIN -255             
#define BALANCE_PID_MAX 255
#define ACC_MIN -20
#define ACC_MAX 20

// Motor Misc
#define PWM_MIN 0
#define PWM_MAX 255
float MOTORSLACK_A=62;                     
float MOTORSLACK_B=65;
float angle;
#define MOTOR_A_PWM_MAX 255            
#define MOTOR_B_PWM_MAX 255               
bool blinkState = false;
int rx_count=0;
byte buf_tmp=0;
uint8_t i2cData[14]; // Buffer for I2C data

long speed_real_l,speed_real_r;

int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input

int MotorAspeed, MotorBspeed, MotorSlack,moveState=0,d_speed,d_dir;

double yaw,input,out,setpoint,originalSetpoint,Buffer[3];

//uint32_t timer,timer1;

double bal_kp,bal_ki,bal_kd;
//int addressFloat=0;

PID pid(&input,&out,&setpoint,BALANCE_KP,BALANCE_KI,BALANCE_KD,DIRECT);

String content = "";
  char character;


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup()
{
  Serial.begin(115200);

  initmot();
  
pid.SetMode(AUTOMATIC);                  
pid.SetOutputLimits(-255, 255);
pid.SetSampleTime(10);

setpoint = 0.4;
originalSetpoint = setpoint;

bal_kp=BALANCE_KP;
bal_ki=BALANCE_KI;
bal_kd=BALANCE_KD;

pid.SetTunings(bal_kp,bal_ki,bal_kd);                      //change PID values

 Wire.begin();// join I2C bus (I2Cdev library doesn't do this automatically)
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
Serial.begin(115200);
  
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  delay(2);
  
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
  // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(47);
    mpu.setYGyroOffset(77);
    mpu.setZGyroOffset(-19);
    mpu.setXAccelOffset(-105);
    mpu.setYAccelOffset(-116);
    mpu.setZAccelOffset(1589); // 1688 factory default for my test chip
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    
    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

}

void loop() 
{
  
   if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {  
  }
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    //Get sensor data
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGyro(gyro, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    input = -ypr[1] * 180/M_PI;
    angle = ypr[1];
  
  new_pid();       
  printval();
  
   blinkState = !blinkState;
   digitalWrite(LED, blinkState);

}
}
    
void printval()
{
  Serial.print(angle * RAD_TO_DEG);
  Serial.print(", ");
 // Serial.print(speed_real_l);Serial.print("\t"); 
 // Serial.println(speed_real_r); Serial.print("\t");
 // Serial.println(yout); //Serial.print("\t");Serial.print("\t"); 
 
  
//  Serial.print(input);Serial.print("\t");
//  Serial.print(originalSetpoint); Serial.print("\t");
//  Serial.print(setpoint); Serial.print("\t");
  Serial.println(out); 
 // Serial.print(", ");
 // Serial.println("0");
  //Serial.print("\t");Serial.print("\t");
  
  
//  Serial.print(MotorAspeed); Serial.print("\t");
//  Serial.print(MotorBspeed); Serial.println("\t");

}



double compensate_slack(double yOutput,double Output,bool A)
  {
   // Compensate for DC motor non-linear "dead" zone around 0 where small values don't result in movement
   //yOutput is for left,right control
  if(A)
  {
   if (Output >= 0) 
   Output = Output + MOTORSLACK_A + yOutput;
   if (Output < 0) 
   Output = Output - MOTORSLACK_A - yOutput;
  }
  else
  {
    if (Output >= 0) 
   Output = Output + MOTORSLACK_B + yOutput;
   if (Output < 0) 
   Output = Output - MOTORSLACK_B - yOutput;
  }
   Output = constrain(Output, BALANCE_PID_MIN, BALANCE_PID_MAX); 
  return Output;
}



void new_pid()
{
  //Compute error
  pid.Compute();

double acc = constrain(speed_real_l, ACC_MIN, ACC_MIN);
  
   // Convert PID output to motor control
     MotorAspeed = compensate_slack(speed_real_l,out,1);
     MotorBspeed = compensate_slack(speed_real_l,out,0);
     motorspeed(MotorAspeed, MotorBspeed);           
}
//Fast digitalWrite is implemented

void initmot()
{
  //Pin definitions
    pinMode(SPD_PUL_L, INPUT);
    pinMode(SPD_PUL_R, INPUT);
    digitalWrite(SPD_PUL_L, HIGH);
    digitalWrite(SPD_PUL_R, HIGH);
    motorGo(1, CW, 0);
    motorGo(0, CW, 0);

    pinMode(LED, OUTPUT);
    
      for (int i=0; i<2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // configure external interruption
  attachInterrupt(1, speed_int_l, RISING);
  attachInterrupt(0, speed_int_r, RISING);
}


void motorspeed(int MotorAspeed, int MotorBspeed) {
  // Motor A control
  if (MotorAspeed >= 0) 
  {
    motorGo(0, CCW, MotorAspeed);
  }
  else 
{
    MotorAspeed=-MotorAspeed;
    motorGo(0, CW, MotorAspeed);
}
  
  // Motor B control
  if (MotorBspeed >= 0) 
  {
    motorGo(1, CW, MotorBspeed);
  }
  else 
  {
    MotorBspeed=-MotorBspeed;
    motorGo(1, CCW, MotorBspeed);
  }
}

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{

      // Set inA[motor]
      if (direct ==1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if (direct==2)
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);
      analogWrite(pwmpin[motor], pwm);
  
}

void speed_int_l()
{
  if (digitalRead(SPD_PUL_L)){
    speed_real_l = -15;
  }
  else{
    speed_real_l = 15;
}
}

void speed_int_r()
{
  if (digitalRead(SPD_PUL_R)){
    speed_real_r = -10;
  }
  else{
    speed_real_r = 10;
}
}
