/*
    File:       Gyromos.ino
    Contains:   Motion gimbal controller
    Website:    http://www.analogx.com/

    Change History (most recent first):

        <001>   02/13/15    Origination:    Initial release

    Copyright 2015 AnalogX, LLC.  All rights reserved.
    Free for non-commercial use, may not be redistributed without prior approval.

*/

#include "I2Cdev.h"
#include <SoftwareSerial.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

/*********************************************************************/

#define TRUE                                    (1)
#define FALSE                                   (0)

#define GIMBAL_DEGREES2ANGLE(Degree)            ((int16_t)((float)(Degree) * (1.0f / 0.02197265625f)))
#define GIMBAL_CALCULATESPEED(Speed)            ((int16_t)((float)(Speed) * (1.0f / 0.1220740379f)))

#define GIMBAL_MAXRESPONSE                      (40)
#define GIMBAL_COMMANDTIMEOUT                   (1 * 1000)
#define GIMBAL_PINGTIMEOUT                      (1 * 1000)
#define GIMBAL_PINGFAILURES                     (3)
#define GIMBAL_MODE1                            (1)
#define GIMBAL_MODE2                            (2)
#define GIMBAL_MODE3                            (3)
#define GIMBAL_MODEMANUAL                       (4)

#define MPU_DISPLAYRATE                         (100)
#define MPU_SAMPLESMOOTHING                     (3)

#define GIMBAL_RANGEROLL                        (90)
#define GIMBAL_RANGEPITCH                       (180)
#define GIMBAL_RANGEYAW                         (90)

#define HC05_POWER                              (8)
#define HC05_ATMODE                             (9)

#define LED_PIN                                 (13)

#define ZERO_PIN                                (6)

/*********************************************************************/

MPU6050 mpu;

uint16_t MPUPacketSize                          = 0;
int MPUCounter                                  = 0;
bool MPUReady                                   = FALSE;
float MPUPitch                                  = 0.0;
float MPURoll                                   = 0.0;
float MPUYaw                                    = 0.0;
float MPUStartPitch                             = 0.0;
float MPUStartRoll                              = 0.0;
float MPUStartYaw                               = 0.0;


SoftwareSerial BluetoothSerial(10, 11);                     /* RX, TX */

byte GimbalDetected                             = FALSE;
byte GimbalJustConnected                        = FALSE;
byte GimbalPingAttempts                         = 0;
int GimbalSpeed                                 = 80;
int GimbalLastMessage                           = 0;
char GimbalResponseBuffer[GIMBAL_MAXRESPONSE]   = {0};
int GimbalResponseFilled                        = 0;

/*********************************************************************/

void System_FlashLED(int Count)
{
    byte i;

    for(i=0; i < Count; i++)
        {
        digitalWrite(LED_PIN, HIGH);
        delay(250);
        digitalWrite(LED_PIN, LOW);
        delay(250);
        }

    delay(500);
}

int System_MemoryAvailable(void)
{
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

/*********************************************************************/

void Gimbal_SendCommand(byte Command, byte *Data, int Size)
{
    byte i, Checksum = 0;


    BluetoothSerial.write('>');
    BluetoothSerial.write(Command);
    BluetoothSerial.write(Size);
    BluetoothSerial.write(Command + Size);
    for(i=0; i < Size; i++)
        {
        Checksum += Data[i];
        BluetoothSerial.write(Data[i]);
        }
    BluetoothSerial.write(Checksum);

//    Serial.print(Command);
//    Serial.println(" Message sent.");

    delay(20);                                                  /* 20ms delay as suggested in SimpleBGC firmware */
}


int Gimbal_ProcessCommand(int Timeout, byte WaitForResponse, byte Command, byte *Data, int DataSize)
{
    int i;
    int StartTime;
    int CurrentTime;
    int ResponseSize = 4;
    int InMessage = FALSE;
    char NewChar;


    if (WaitForResponse == TRUE)                                /* Make sure anything that might be waiting is cleared */
        {
        delay(20);
        while(BluetoothSerial.available())
            BluetoothSerial.read();
        }

    if (Command != 0)
        Gimbal_SendCommand(Command, Data, DataSize);

    BluetoothSerial.println(Command);

    GimbalResponseFilled = 0;
    if (WaitForResponse == FALSE)
        return TRUE;

    StartTime = millis();
    while(GimbalResponseFilled < ResponseSize)
        {
        CurrentTime = millis();
        if ((CurrentTime - StartTime) > Timeout)
            {
            Serial.println("Timeout");
            return FALSE;
            }

        if (BluetoothSerial.available())
            {
            GimbalLastMessage = millis();
            GimbalPingAttempts = 0;
            NewChar = BluetoothSerial.read();
            if ((NewChar == '>') && (InMessage == FALSE))
                {
                GimbalResponseFilled = 0;
                InMessage = TRUE;
                }
            if (InMessage == TRUE)
                {
                GimbalResponseBuffer[GimbalResponseFilled++] = NewChar;
                if (GimbalResponseFilled > (GIMBAL_MAXRESPONSE - 2))
                    {
                    Serial.print("Overrun: ");
                    Serial.println(ResponseSize);
                    return FALSE;
                    }
                if (GimbalResponseFilled == 3)
                    {
                    ResponseSize = (GimbalResponseBuffer[2] + 4 + 1);
                    }
                }
            }
        }

    GimbalResponseBuffer[GimbalResponseFilled] = '\0';
    if ((GimbalResponseFilled == 0) && (ResponseSize > 0))
        {
        Serial.print("No response: ");
        Serial.println(ResponseSize);
        return FALSE;
        }

    Serial.print("Response ");
    Serial.print(GimbalResponseFilled, DEC);
    Serial.print(": ");
    for(i=0; i < GimbalResponseFilled; i++)
        {
        Serial.print(GimbalResponseBuffer[i], HEX);
        Serial.print(" ");
        }
    Serial.println("");

    if ((Command != 0) &&
        (GimbalResponseBuffer[1] != Command))
        {
        Serial.print("Mismatched: ");
        Serial.println(Command, HEX);
        return FALSE;
        }
    return TRUE;
}


void Gimbal_SetDetected(int IsDetected)
{
    if (GimbalDetected == IsDetected)
        return;

    GimbalDetected = IsDetected;
                                                                    /* Give some physical feedback that something changed */
    if (GimbalDetected == TRUE)
        {
        GimbalJustConnected = TRUE;
        Serial.print("Connected: ");
        Serial.println(System_MemoryAvailable());
        digitalWrite(LED_PIN, HIGH);
        }
    else
        {
        Serial.println("Disconnected");
        digitalWrite(LED_PIN, LOW);
        }
}

int Gimbal_CommandSetAngle(float Pitch, float Roll, float Yaw)
{
    int TempSpeed;
    int TempValue;
    byte DataBuffer[20];


    DataBuffer[0] = 2;                              /* MODE_ANGLE */
    TempSpeed = GIMBAL_CALCULATESPEED(GimbalSpeed);
                                                    /* Roll */
    memcpy(DataBuffer + 1, &TempSpeed, 2);
    TempValue = GIMBAL_DEGREES2ANGLE(Roll);
    memcpy(DataBuffer + 3, &TempValue, 2);

                                                    /* Pitch */
    memcpy(DataBuffer + 5, &TempSpeed, 2);
    TempValue = GIMBAL_DEGREES2ANGLE(Pitch);
    memcpy(DataBuffer + 7, &TempValue, 2);
                                                    /* Yaw */
    memcpy(DataBuffer + 9, &TempSpeed, 2);
    TempValue = GIMBAL_DEGREES2ANGLE(Yaw);
    memcpy(DataBuffer + 11, &TempValue, 2);

    return Gimbal_ProcessCommand(GIMBAL_COMMANDTIMEOUT, FALSE, 67, DataBuffer, 13);         /* CMD_CONTROL */
}

int Gimbal_CommandSetAngleSpeed(float Pitch, float Roll, float Yaw)
{
    int TempSpeed;
    int TempValue;
    byte DataBuffer[20];


    DataBuffer[0] = 3;                              /* MODE_SPEED_ANGLE */
    TempSpeed = GIMBAL_CALCULATESPEED(GimbalSpeed);
                                                    /* Roll */
    memcpy(DataBuffer + 1, &TempSpeed, 2);
    TempValue = GIMBAL_DEGREES2ANGLE(Roll);
    memcpy(DataBuffer + 3, &TempValue, 2);

                                                    /* Pitch */
    memcpy(DataBuffer + 5, &TempSpeed, 2);
    TempValue = GIMBAL_DEGREES2ANGLE(Pitch);
    memcpy(DataBuffer + 7, &TempValue, 2);
                                                    /* Yaw */
    memcpy(DataBuffer + 9, &TempSpeed, 2);
    TempValue = GIMBAL_DEGREES2ANGLE(Yaw);
    memcpy(DataBuffer + 11, &TempValue, 2);

    return Gimbal_ProcessCommand(GIMBAL_COMMANDTIMEOUT, FALSE, 67, DataBuffer, 13);         /* CMD_CONTROL */
}

int Gimbal_CommandSetAngleRC(float Pitch, float Roll, float Yaw)
{
    int TempSpeed;
    int TempValue;
    byte DataBuffer[20];


    DataBuffer[0] = 4;                              /* MODE_ANGLE */
    TempSpeed = GIMBAL_CALCULATESPEED(GimbalSpeed);
                                                    /* Roll */
    memcpy(DataBuffer + 1, &TempSpeed, 2);
    TempValue = (int)((Roll / ((float)GIMBAL_RANGEROLL / 2.0)) * 500.0);
    memcpy(DataBuffer + 3, &TempValue, 2);
                                                    /* Pitch */
    memcpy(DataBuffer + 5, &TempSpeed, 2);
    TempValue = (int)((Pitch / ((float)GIMBAL_RANGEPITCH / 2.0)) * 500.0);
    memcpy(DataBuffer + 7, &TempValue, 2);
                                                    /* Yaw */
    memcpy(DataBuffer + 9, &TempSpeed, 2);
    TempValue = (int)((Yaw / ((float)GIMBAL_RANGEYAW / 2.0)) * 500.0);
    memcpy(DataBuffer + 11, &TempValue, 2);

    return Gimbal_ProcessCommand(GIMBAL_COMMANDTIMEOUT, FALSE, 67, DataBuffer, 13);         /* CMD_CONTROL */
}


int Gimbal_CommandGetAngle(void)
{
    int TempValue;


    if (Gimbal_ProcessCommand(GIMBAL_COMMANDTIMEOUT, TRUE, 73, NULL, 0) == FALSE)           /* CMD_GET_ANGLES */
        return FALSE;
    memcpy(&TempValue, GimbalResponseBuffer + 4, 2);
    Serial.print("Actual Roll: ");
    Serial.print(TempValue);

    memcpy(&TempValue, GimbalResponseBuffer + 10, 2);
    Serial.print("  Pitch: ");
    Serial.print(TempValue);

    memcpy(&TempValue, GimbalResponseBuffer + 16, 2);
    Serial.print("  Yaw: ");
    Serial.println(TempValue);

    memcpy(&TempValue, GimbalResponseBuffer + 6, 2);
    Serial.print("Target Roll: ");
    Serial.print(TempValue);

    memcpy(&TempValue, GimbalResponseBuffer + 12, 2);
    Serial.print("  Pitch: ");
    Serial.print(TempValue);

    memcpy(&TempValue, GimbalResponseBuffer + 18, 2);
    Serial.print("  Yaw: ");
    Serial.println(TempValue);

    memcpy(&TempValue, GimbalResponseBuffer + 8, 2);
    Serial.print("Speed Roll: ");
    Serial.print(TempValue);

    memcpy(&TempValue, GimbalResponseBuffer + 14, 2);
    Serial.print("  Pitch: ");
    Serial.print(TempValue);

    memcpy(&TempValue, GimbalResponseBuffer + 20, 2);
    Serial.print("  Yaw: ");
    Serial.println(TempValue);
    return TRUE;
}


int Gimbal_CommandPing(byte WaitForResponse)
{
    return Gimbal_ProcessCommand(GIMBAL_COMMANDTIMEOUT, WaitForResponse, 86, NULL, 0);      /* CMD_BOARD_INFO */
}

int Gimbal_CommandChangeMode(byte WaitForResponse, byte Mode)
{
    byte DataBuffer[2];

    DataBuffer[0] = Mode;
    return Gimbal_ProcessCommand(GIMBAL_COMMANDTIMEOUT, WaitForResponse, 69, DataBuffer, 1);      /* CMD_EXECUTE_MENU */
}

/*********************************************************************/

void MPU_UpdatePosition(void)
{
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];
    uint8_t mpuIntStatus;
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];


    while(1)
        {
        fifoCount = mpu.getFIFOCount();
        if (fifoCount >= MPUPacketSize)
            break;
        delay(1);
        }

    mpuIntStatus = mpu.getIntStatus();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || (fifoCount >= 1024))
        {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        MPU_UpdatePosition();
        return;
        }

    if (mpuIntStatus & 0x02)
        {
        mpu.getFIFOBytes(fifoBuffer, MPUPacketSize);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        MPUYaw   = ypr[0] * (180 / M_PI);
        MPUPitch = ypr[1] * (180 / M_PI);
        MPURoll  = ypr[2] * (180 / M_PI);
        return;
        }

    /* We didn't get anything, try again */

    MPU_UpdatePosition();
}

float MPU_FindSmallestDifference(float LastValue, float *ValueList, byte ValueCount)
{
    byte i;
    byte SmallestOffset;
    float Difference;
    float SmallestDifference;


    SmallestOffset = 0;
    SmallestDifference = 360.0;
    for(i=0; i < ValueCount; i++)
        {
        if (LastValue > ValueList[i])
            Difference = LastValue - ValueList[i];
        else
            Difference = ValueList[i] - LastValue;
        if (SmallestDifference > Difference)
            {
            SmallestDifference = Difference;
            SmallestOffset = i;
            }
        }
    return ValueList[SmallestOffset];
}


void MPU_Process(int CurrentTime)
{
    byte i;
    float LastPitch, LastRoll, LastYaw;
    float PitchPool[MPU_SAMPLESMOOTHING];
    float RollPool[MPU_SAMPLESMOOTHING];
    float YawPool[MPU_SAMPLESMOOTHING];
    uint8_t mpuIntStatus;


    if (MPUReady == FALSE)
        return;

    LastPitch = MPUPitch;
    LastRoll = MPURoll;
    LastYaw = MPUYaw;

    for(i=0; i < MPU_SAMPLESMOOTHING; i++)
        {
        mpu.resetFIFO();
        delay(1);

        MPU_UpdatePosition();
        PitchPool[i] = MPUPitch;
        RollPool[i] = MPURoll;
        YawPool[i] = MPUYaw;
        }

    MPUPitch = MPU_FindSmallestDifference(LastPitch, PitchPool, MPU_SAMPLESMOOTHING);
    MPURoll = MPU_FindSmallestDifference(LastRoll, RollPool, MPU_SAMPLESMOOTHING);
    MPUYaw = MPU_FindSmallestDifference(LastYaw, YawPool, MPU_SAMPLESMOOTHING);

    if ((GimbalJustConnected == TRUE) ||
        (digitalRead(ZERO_PIN) == 0))
        {
        MPUStartYaw   = MPUYaw;
        MPUStartPitch = MPUPitch;
        MPUStartRoll  = MPURoll;

        Serial.print(MPUStartYaw);
        Serial.print("\t");
        Serial.print(MPUStartPitch);
        Serial.print("\t");
        Serial.println(MPUStartRoll);

        GimbalJustConnected = FALSE;
        }

    Serial.print("cnt\t");
    Serial.print(MPUCounter);

    Serial.print("\t ypr\t");
    Serial.print(MPUYaw);
    Serial.print("\t");
    Serial.print(MPUPitch);
    Serial.print("\t");
    Serial.println(MPURoll);

    MPUCounter += 1;
}


/*********************************************************************/

void setup()
{
    uint8_t devStatus;


    pinMode(ZERO_PIN, INPUT);
    digitalWrite(ZERO_PIN, HIGH);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);


    pinMode(HC05_POWER, OUTPUT);
    pinMode(HC05_ATMODE, OUTPUT);
    digitalWrite(HC05_POWER, LOW);
    digitalWrite(HC05_ATMODE, LOW);
    delay(100);
//    digitalWrite(HC05_ATMODE, HIGH);      /* Not running in AT mode */
    delay(100);
    digitalWrite(HC05_POWER, HIGH);

    System_FlashLED(1);

    Serial.begin(19200);
    Serial.println("Start\n");

    Wire.begin();
    TWBR = 152; // 400kHz I2C clock (200kHz if CPU is 8MHz)

    BluetoothSerial.begin(9600);
    Serial.println("Bluetooth connected\n");

    System_FlashLED(2);

    Serial.println(F("Initializing MPU"));
    mpu.initialize();

    Serial.println(mpu.testConnection() ? F("MPU6050 connected") : F("MPU6050 not found"));


    Serial.println(F("Initializing DMP"));
    devStatus = mpu.dmpInitialize();

                /* You must first get these values from MPU6050_calibration.ino */

    mpu.setXAccelOffset(-2382);
    mpu.setYAccelOffset(308);
    mpu.setZAccelOffset(1268);
    mpu.setXGyroOffset(19);
    mpu.setYGyroOffset(-28);
    mpu.setZGyroOffset(31);

    System_FlashLED(3);

    if (devStatus == 0)
        {
        Serial.println(F("Enabled DMP"));
        mpu.setDMPEnabled(true);

        MPUReady = TRUE;
        MPUPacketSize = mpu.dmpGetFIFOPacketSize();

        Serial.print(F("DMP active ("));
        Serial.print(MPUPacketSize);
        Serial.println(F(")"));

        System_FlashLED(4);
        }
    else
        {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));

        while(1)                /* We're dead, flash 5 times */
            {
            System_FlashLED(5);
            }
        }

}


void loop()
{
    int CurrentTime;
    int TempValue;
    float TempPitch;
    float TempYaw;



    CurrentTime = millis();

    MPU_Process(CurrentTime);

                                        /* We ignore everything else if we're not connected */

    if (GimbalDetected == FALSE)
        {
        if (Gimbal_CommandPing(TRUE) == TRUE)
            {
            Gimbal_SetDetected(TRUE);
            }
        else
            Gimbal_SetDetected(FALSE);
        return;
        }

                                        /* The first thing we want to do is ensure we are at least heart-beating the gimbal to verify the connection */

    if ((CurrentTime - GimbalLastMessage) > GIMBAL_PINGTIMEOUT)
        {
        if (GimbalPingAttempts >= GIMBAL_PINGFAILURES)
            {
            Gimbal_SetDetected(FALSE);
            return;
            }

        Gimbal_CommandPing(FALSE);
        GimbalPingAttempts += 1;
        GimbalLastMessage = CurrentTime;
        }

    if (BluetoothSerial.available() > 0)
        {
        GimbalLastMessage = CurrentTime;
        GimbalPingAttempts = 0;
        while(BluetoothSerial.available())
            {
            BluetoothSerial.read();
    //        Serial.println(Serial1.read());
            }
        }


    Gimbal_CommandSetAngleRC(-MPUPitch, MPURoll, MPUYaw - MPUStartYaw);
}
