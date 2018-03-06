// this code demonstrates the full capabilities of the GY-88
// -MPU6050 -> pitch, roll
// -BMP085 -> elevation (based off air pressure)
// -HMC5883L -> heading 


// includes for i2c and communication stuff
#include "Wire.h"

// includes for sensors
#include "BMP085.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <HMC5883L.h>

// initialize objects for each of our sensors
BMP085 barometer;
MPU6050 mpu;
HMC5883L compass;

// MPU control and status variables
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// MPU orientation and motion   variables
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// MPU interruption detection
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// global variables for the mpu
float currYaw, currPitch, currRoll, prevYaw, prevPitch, prevRoll;   // holds orientation

// global variable for the magnetometer
float heading, prevHeading;          //  holds current and prev heading

// our timer variable
unsigned long currentMillis = 0; 
unsigned long printedMillis = 0;

void setup(){
  Serial.begin(38400);
  Serial.println(F("--------------Welcome to the GY-88 full demonstration tester------------"));

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(1000);

  Serial.println(F("Remember! The sensor should be oriented face-down, with the orientation guide facing upwards"));
  Serial.println(F("Press any key to start device initialization..."));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // initialize our devices!!
  Serial.println(F("---------------------------Initializing devices-------------------------"));
  Serial.print(F("           Initializing MPU: "));
  mpu.dmpInitialize();    // initialize our mpu with the dmp!
  mpu.setXGyroOffset(-157);     // set offsets from test
  mpu.setYGyroOffset(84);
  mpu.setZGyroOffset(29);
  mpu.setZAccelOffset(411);
  mpu.setDMPEnabled(true);    // enable our DMP!! this takes load off our arduino and gives nice results!
  packetSize = mpu.dmpGetFIFOPacketSize();    // get DMP packet size for later comparison
  Serial.println(F("successful"));

  Serial.print(F("           Initializing compass: "));
  compass.begin();    // initialize our compass!
  compass.setRange(HMC5883L_RANGE_1_3GA);  // Set measurement range
  compass.setMeasurementMode(HMC5883L_CONTINOUS);  // Set measurement mode
  compass.setDataRate(HMC5883L_DATARATE_30HZ);  // Set data rate
  compass.setSamples(HMC5883L_SAMPLES_8);  // Set number of samples averaged
  compass.setOffset(0, 0);   // Set calibration offset. See HMC5883L_calibration.ino    
  Serial.println(F("successful"));

  Serial.print(F("           Initializing barometer: "));
  barometer.bmp085Calibration();        // calibrate our pressure sensor
  Serial.println(F("successful"));

  // verify connections
  Serial.println(F("-------------------------Testing device connections---------------------"));
  Serial.println(mpu.testConnection() ? "All connections successful!" : "MPU6050 connection failed");

  Serial.println(F(" "));
  Serial.println(F("Press any key to start testing!!"));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
}


void loop()
{
  // first save our time
  currentMillis = millis();

  // grab barometer information
  float temperature = barometer.bmp085GetTemperature(); //MUST be called first?
  float pressure = barometer.bmp085GetPressure();
  float altitude = barometer.calcAltitude(pressure);

  // grab compass information
  Vector mag = compass.readNormalize();
  heading = processHeading(mag);

  // grab MPU information
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();     // get current FIFO count
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();        // wait for correct available data length
  mpu.getFIFOBytes(fifoBuffer, packetSize);       // read a packet from FIFO 
  mpu.resetFIFO();    // CLEAR THE BUFFER!!
  fifoCount -= packetSize;      // track FIFO count here in case there is > 1 packet available

  // now we can grab yaw, pitch and roll from the dmp buffer
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpu.dmpGetAccel(&aa, fifoBuffer);

  // and turn them into nice degrees
  currYaw = (ypr[0] * 180/M_PI);    // we have this information, but we discard it because the magnetometer is much nicer
  currPitch = (ypr[1] * 180/M_PI);    // flip pitch and roll because we upside down!
  currRoll = -(ypr[2] * 180/M_PI);    


  // now we can print!
  if((currentMillis - printedMillis) > 100)   // wait .1 seconds between displaying
  {
    
   // BAROMETER THINGS
//  Serial.print("Temperature: ");
//   Serial.print(temperature, 2); //display 2 decimal places
//   Serial.println("deg C");
//   Serial.print("Pressure: ");
//   Serial.print(pressure, 0); //whole number only.
//   Serial.println(" Pa"); 
   Serial.print("Altitude: ");
   Serial.print(altitude, 2); //display 2 decimal places
   Serial.print("M");

   // COMPASS THINGS
   Serial.print("    Heading: ");
   Serial.print(heading, 0);    // don't need no decimals here

   // MPU THINGS
   Serial.print("    Pitch: ");
   Serial.print(currPitch);
   Serial.print(" Roll: ");
   Serial.print(currRoll);   
   
   Serial.println();    //line break
  }
}

// tilt compensation, refining, and error scrubbing
float processHeading(Vector mag)
{  
  // convert them to pitch and roll
  float roll;
  float pitch;
/*
  Serial.print(ay);
  Serial.print(" <- ay       ax-> ");
  Serial.println(-ax);
  */
  roll = asin(-ypr[2]);   // we've got this info already from the dmp
  pitch = asin(ypr[1]);

  if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78)
  {
    return prevHeading;
  }
  
    // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(roll);  
  float sinRoll = sin(roll);  
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);
  
  // Tilt compensation
  float Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
  float Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;
 
  float process_heading = atan2(Yh, Xh);

  // adjust for declination angle
  float declinationAngle = (-3.0 + (11.0 / 60.0)) / (180 / M_PI);   // this is declination angle for uiuc!
  process_heading += declinationAngle;

  process_heading = correctAngle(process_heading);    // correct angle for if its above 2pi or less than 0
  process_heading = process_heading * 180/M_PI;     // convert to degrees

  // Fix issue with angles
  float fixedHeadingDegrees;
 
  if (process_heading >= 1 && process_heading < 240)
  {
    fixedHeadingDegrees = map(heading, 0, 239, 0, 179);
  } else
  if (process_heading >= 240)
  {
    fixedHeadingDegrees = map(heading, 240, 360, 180, 360);
  }

  // Smooth angles rotation
  int smoothHeadingDegrees = round(fixedHeadingDegrees);

  if (smoothHeadingDegrees < (prevHeading + 3) && smoothHeadingDegrees > (prevHeading - 3))
  {
    smoothHeadingDegrees = prevHeading;
  }
  prevHeading = smoothHeadingDegrees;

  return process_heading;   // return the smoothed, calibrated heading!
}

// Correct angle
float correctAngle(float process_heading)
{
  if (process_heading < 0) { process_heading += 2 * PI; }
  if (process_heading > 2 * PI) { process_heading -= 2 * PI; }

  return process_heading;
}

