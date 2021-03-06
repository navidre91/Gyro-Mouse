//Note that sampling rate is 20Hz, It can be set in the MPU6050_6Axis_MotionApps20.h

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <SoftwareSerial.h>
SoftwareSerial BT = SoftwareSerial(2, 0);
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL


#define TOUCH_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define Interrupt_PIN 4

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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//Touch state
bool touched;
bool interrupted;
int filterBufferLength=10;

float tempGyroY[2];
float tempGyroX[2];
int movementY;
int movementX;
int velocityZ;
int velocityX;

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
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    mpu.initialize();

    // wait for ready
  //  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //  while (Serial.available() && Serial.read()); // empty buffer
  //  while (!Serial.available());                 // wait for data
  //  while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP

    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready

        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection

        //attachInterrupt(0, dmpDataReady, RISING);
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

    }

    // configure LED for output
    BT.begin(9600);
    pinMode(TOUCH_PIN, INPUT);
    pinMode(Interrupt_PIN, INPUT);
    tempGyroY[0]=0;
    tempGyroY[1]=0;
    tempGyroX[0]=0;
    tempGyroX[1]=0;
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    while (1){
      interrupted=digitalRead(Interrupt_PIN);
      if(interrupted){
        dmpDataReady();
        break;
        }
      }
    
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
  
            // display Euler angles in degrees
           // mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            //setVelocity(aaReal.x,aaReal.z);
            
            tempGyroY[0]=tempGyroY[1];
            tempGyroY[1]=ypr[2];
            tempGyroX[0]=tempGyroX[1];
            tempGyroX[1]=ypr[0];
            movementY=15*(tempGyroY[1]-tempGyroY[0])* 180/M_PI;
            movementX=15*(tempGyroX[1]-tempGyroX[0])* 180/M_PI;
            touched=digitalRead(TOUCH_PIN);
            if (touched){
           // Serial.print("ypr\t");
          //  Serial.print(ypr[0] * 180/M_PI);
          //  Serial.print("\t");
          //  Serial.print(ypr[1] * 180/M_PI);
          //  Serial.print("\t");
          //  Serial.println(ypr[2] * 180/M_PI);
         
             mouseCommand(0,movementX,movementY);
            }
        
    }
}


void mouseCommand(uint8_t buttons, uint8_t x, uint8_t y) {
  BT.write(0xFD);
  BT.write((byte)0x00);
  BT.write((byte)0x03);
  BT.write(buttons);
  BT.write(x);
  BT.write(y);
  BT.write((byte)0x00);
  BT.write((byte)0x00);
  BT.write((byte)0x00);
}

//void setVelocity(float accX, float accZ){
 // velocityX=velocityX+accX;
 // velocityZ=velocityX+accZ;
 // }

