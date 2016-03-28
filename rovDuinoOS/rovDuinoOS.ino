
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <HMC5883L.h>


// Variables for Temperature

#define ONE_WIRE_BUS 2        // Temp sensor Pin no. 2

#define m_temp 1          //calibration co-efficients
#define c_temp 0


OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//Variables for pressure

#define pressure_pin A0       // pressure sensor pin no. A0
#define m_depth 200          //calibration co-efficients
#define c_depth 0

//Variables for compass

  HMC5883L mag;
  
  int16_t mx, my, mz;

//Variables for IMU

  MPU6050 mpu;
  
  // IMU calibration data
  #define X_off 220
  #define Y_off 76
  #define Z_off -85
  #define Accel_off 1788
  
  
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
  
  volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

//Varibles for OS

  char control_para;   //control parameter recieved from PC
  float roll,pitch,heading,depth,temp,ph; //status variables
  float D_roll,D_pitch,D_heading,D_depth; //Desired status

//Supporting functions

void dmpDataReady() 
{
    mpuInterrupt = true;    //Interrupt for IMU
}

void setup() 
{
  
  Wire.begin();
  Serial.begin(9600);
  delay(500);
  Serial.println("######  ROV OS  ######");
  Serial.println(" Send any key to boot");
  Serial.println("######################");
  while(!Serial.available());
  Serial.println("Motherboard is online"); 
  Serial.println("######  ######  ######");

    
  compass_init();
  imu_init();
  temp_init();

      
  Serial.println("System checkup completed");
  Serial.println("#######  ######  #######");
  Serial.println("### SYSTEM IS ONLINE ###");
  Serial.println("#######  ######  #######");
  Serial.flush();
}

void loop() 
{
  while(!Serial.available())
      {
        sensor_update();
        stabilize(); 
      }
  control_para = Serial.read();
  
  switch(control_para)
  {
    case 'c':
      Serial.println(heading);
      break;
      
    case 'r':
      Serial.println(roll);
      break;
      
    case 'p':
      Serial.println(pitch);
      break;
      
    case 't':
      Serial.println(temp);
      break;
      
  }
}


void sensor_update()
{
  
 motion_sens();
 compass_sens();
 pressure_sens(); 
 temp_sens();
 ph_sens();
 
}

void stabilize()
{
  
  
}


void motion_sens()
{
  if (!dmpReady) return;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
      
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            pitch= ypr[1] * 180/M_PI ;
            roll= ypr[2] * 180/M_PI;  
    }
}

void imu_init()
{
    mpu.initialize();
  
  Serial.println(mpu.testConnection() ? F("IMU connection successful") : F("IMU connection failed"));

  devStatus = mpu.dmpInitialize();
  if (devStatus == 0)
  {
    mpu.setXGyroOffset(X_off);                // Offset value
    mpu.setYGyroOffset(Y_off);
    mpu.setZGyroOffset(Z_off);
    mpu.setZAccelOffset(Accel_off);
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP is Turned on");
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

void compass_sens()
{
  
    mag.getHeading(&mx, &my, &mz);
    heading = atan2(my, mx);
    if(heading < 0)
      heading += 2 * M_PI;
    heading = heading * 180/M_PI; 
}

void compass_init()
{
  mag.initialize();  
  delay(500);
  Serial.println(mag.testConnection() ? "Compass connection successful" : "Compass connection failed");
  delay(500);
  
}

void temp_sens()
{
   sensors.requestTemperatures();
   temp = sensors.getTempCByIndex(0);
   temp = temp * m_temp + c_temp;
  
}

void temp_init()
{
    sensors.begin();
    
}

void pressure_sens()
{
  float p = analogRead(pressure_pin); // Pressure sensor ADC @ ADC pin 0
  depth = m_depth*p + c_depth;
  
}

void ph_sens()
{
  
  
}
