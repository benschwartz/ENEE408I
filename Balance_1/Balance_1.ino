//####################################################
//LIBRARY INCLUDES
//####################################################
//Motor Controller
#include "DualMC33926MotorShield.h"
//IMU
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
//KALMAN FILTERING
#include "MatrixMath.h"
//##################################################
//CONSTANT DEFINITIONS
//###################################################
//CALIBRATION
#define CAL_ITERS 1000
#define WAIT_TIME 20000
//PINS 
#define CALIB_LED_PIN 11
//KALMAN FILTERING
#define N 6
#define M 3
#define P 2
//##################################################
//GLOBAL VARIABLES
//###################################################
//Motor Controller
DualMC33926MotorShield md; // Motorshield Object
////////////////////////////////////////////////
//IMU
MPU6050 mpu; // IMU Object
//MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
//orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
///////////////////////////////////////////////////
//KALMAN FILTERING
float Innovation[N][M] = {
  {.0774,.0876,0},
  {0,0,.9967},
  {.9938,-.0068,0},
  {0,0,7.9460},
  {-.0068,.9919,0},
  {0,0,4.195}
}; // Innovation gain (Kalman filter)
float K[P][N] = {
  {.7071,1.0995,3.1623,2.2409,-6.6562,-1.3915},
  {.7071,1.0995,-3.1623,-2.2409,-6.6562,-1.3915}
}; // LQR gain u = -Kx
float x[N];
float x_dot[N];
float u[P];
float y[M];
float A[N][N] = {
  {0,1,0,0,0,0},
  {0,0,0,0,5.4365,0},
  {0,0,0,1,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,1},
  {0,0,0,0,61.1608,0}
};
float B[N][P] = {
  {0,0},
  {-16.6455,-16.6455},
  {0,0},
  {146.7373,-146.7373},
  {0,0},
  {-57.6542,-57.6542}
};
///////////////////////////////////////////////////
//CALIBRATION
int calibration_iter;
double offset_Ax;
double offset_Ay;
double offset_Az;
double offset_yaw;
double offset_pitch;
double offset_roll;
/////////////////////////////////////////////////////
//TIMING
unsigned long last_millis;
/////////////////////////////////////////////////////
// Control Params
double k = 15;
double kD = 2;
double kI = 7;
/////////////////////////////////////////////////////
// State Vars
double yaw;
double pitch;
double roll;
double Ax;
double Ay;
double Az;
double lastPitch;
double intPitch;
double dPitch;
double intAx;

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
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    //###########################################
    // PINS
    //###########################################
    pinMode(CALIB_LED_PIN, OUTPUT);
    //###########################################
    // END PINS
    //###########################################
    //###########################################
    // IMU
    //###########################################
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    //###########################################
    // END IMU
    //###########################################
    //###########################################
    // MOTOR SHIELD
    //###########################################
    md.init();
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
    // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);
      // enable Arduino interrupt detection
      // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      dmpReady = true;
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    //###########################################
    // END MOTOR SHIELD
    //###########################################
    //###########################################
    // KALMAN FILTER INITIALIZATION
    //###########################################
    for(int i=0;i<N;++i) {
      x[i]=0;
    }
    //###########################################
    // END KALMAN FILTER INITIALIZATION
    //###########################################
    //###########################################
    // CALIBRATION INITIALIZATION
    //###########################################
    offset_Ax = 0;
    offset_Ay = 0;
    offset_Az = 0;
    offset_yaw = 0;
    offset_pitch = 0;
    offset_roll = 0;
    calibration_iter = 0;
    digitalWrite(CALIB_LED_PIN, HIGH);
    //###########################################
    // END CALIBRATION INITIALIZATION
    //###########################################
    //###########################################
    // STATE INITIALIZATION
    //###########################################
    Ax = 0;
    Ay = 0;
    Az = 0;
    yaw = 0;
    pitch = 0;
    roll = 0;
    intAx = 0;
    lastPitch = 0;
    intPitch = 0;
    dPitch = 0;
    //###########################################
    // END STATE INITIALIZATION
    //###########################################
    
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  if (!dmpReady) return; // if programming failed, don't try to do anything
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) { 
    if(calibration_iter > CAL_ITERS){ // check if calibration has finished
      //######################################################
      // BEGIN MAIN CONTROL PROGRAM
      //######################################################        
      double v_e = k*pitch*abs(pitch) + kD*dPitch + kI*intPitch;
      md.setM1Speed(v_e);
      md.setM2Speed(-v_e);
      //######################################################
      // END MAIN CONTROL PROGRAM
      //######################################################    
    }
  }
  //########################################################
  // BEGIN MPU INTERRUPT ROUTINE
  //########################################################
  mpuInterrupt = false; // reset interrupt flag
  mpuIntStatus = mpu.getIntStatus(); // get INT_STATUS byte
  fifoCount = mpu.getFIFOCount(); // get current FIFO count
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO(); // reset so we can continue cleanly
    Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize); // read a packet from FIFO
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    //######################################################
    // BEGIN UPDATE MPU STATE
    //######################################################
    unsigned long this_millis = millis();
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    double yaw_raw = ypr[0] * 180/M_PI;
    double pitch_raw = ypr[1] * 180/M_PI;
    double roll_raw = ypr[2] * 180/M_PI;
    double Ax_raw = aaWorld.x;
    double Ay_raw = aaWorld.y;
    double Az_raw = aaWorld.z;
    // Only begin calibration once the WAIT_TIME has been reached
    if(this_millis > WAIT_TIME){
      if(calibration_iter < CAL_ITERS){ // Calibration in progress
        offset_Ax += Ax_raw;
        offset_Ay += Ay_raw;
        offset_Az += Az_raw;
        offset_yaw += yaw_raw;
        offset_pitch += pitch_raw;
        offset_roll += roll_raw;
        calibration_iter += 1;  
      }
      else if(calibration_iter == CAL_ITERS){ // Calibration finished
        offset_Ax /= CAL_ITERS;
        offset_Ay /= CAL_ITERS;
        offset_Az /= CAL_ITERS;
        offset_yaw /= CAL_ITERS;
        offset_pitch /= CAL_ITERS;
        offset_roll /= CAL_ITERS;
        calibration_iter += 1; // never do this again
        digitalWrite(CALIB_LED_PIN,LOW);
      }
      else {  // Post calibration - update state to account for offsets and integrate x
        Ax = Ax_raw-offset_Ax;
        Ay = Ay_raw-offset_Ay;
        Az = Az_raw-offset_Az;
        yaw = yaw_raw-offset_yaw;
        pitch = pitch_raw-offset_pitch;
        roll = roll_raw-offset_roll;
        double delta_t = (this_millis-last_millis)*.001;
        dPitch = pitch - lastPitch;
        intPitch += dPitch*delta_t;
        intAx += Ax*delta_t;
        lastPitch = pitch;
      }
      last_millis=this_millis;
    }
    //######################################################
    // END UPDATE MPU STATE
    //######################################################
  }
  //########################################################
  // END MPU INTERRUPT ROUTINE
  //########################################################  
}
