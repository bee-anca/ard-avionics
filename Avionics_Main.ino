//To fix/do:
//- Pitch and roll angle from bno unstable
//- How to filter out gravity to detect motor burnout
//- Code for led and buzzer, corresponding to what action/state? (calibration - blink fast, blink slow in loop)
//- Define the coordinate of the rocket
//- Get data from BNO085

//Import Libraries
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LPS2X.h>

//Define baro: LPS22
uint8_t lpsAddr = 0xB8;
Adafruit_LPS22 LPS;
  sensors_event_t pressure;
  sensors_event_t temp;

//Define high G IMU: LSM6DS
uint8_t lsmAddr = 0xD4;
Adafruit_LSM6DSOX LSM;
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp_lsm;

//Define IMU: BNO085
#define BNO08X_RESET -1

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

//Define pins
typedef struct {
  //Pyro pins
  uint8_t pyro1Fire = 31; 
  //uint8_t pyro1Cont = ;
  uint8_t pyro2Fire = 32;
  //uint8_t pyro2Cont = ;
  //Beeper pin
  uint8_t buzzer = 33;
  //LED pin
  uint8_t ld1 = 5;
  uint8_t ld2 = 6;
} Pins;
Pins pins;

//Define other variables
float referencePressure;

uint8_t event_code = 0;
float lastHeight =0;
float currentHeight=0;
float maxHeight = 0;
int count = 0;
unsigned long startFireTime = 0;
unsigned long currentFireTime = 0;

//Set flight conditions
unsigned long apogeeDelay = 0; //Not sure if needed
unsigned long fireTime = 500; //milliseconds 
float altLockOut = 300; //Altitude lock out for pyro in meter

float errorAccX, errorAccY, errorAccZ, errorGyrX, errorGyrY, errorGyrZ;

//Pyro status
typedef struct {
  bool pyroCont = false;
  bool pyroArmed = false;
  //bool pyroFire = false;
} pyro;
pyro pyro1;
pyro pyro2;

//Define events
typedef struct{
  bool preLiftoff = true; //can be used to lower sampling rate
  bool liftoff = false;
  bool boosterBurnout = false;
  bool apogee = false;
  bool apogeeDeploy = false;
  bool descent = false;
  bool touchdown = false; //can be used to lower sampling rate
} eventList;

eventList events;

//Event codes for RF and data logging
typedef struct {
  char preLiftoff = 0;
  char liftoff = 1;
  char boosterBurnout = 2;
  char apogee = 3;
  char parachute = 4;
  char descent = 5;
  char touchdown = 6;
} eventCode;

eventCode codes;

void setup() {
  //Set serial
  Serial.begin(115200);

  //Set pin mode
  pinMode(pins.pyro1Fire,OUTPUT);
  pinMode(pins.buzzer, OUTPUT);
  pinMode(pins.ld1, OUTPUT);
  pinMode(pins.ld2, OUTPUT);

  digitalWrite(pins.pyro1Fire, LOW);

  //Start SD card 

  //Start LoRa

  //Start sensors (IMU, High G, Baro)
  beginLPS(lpsAddr);
  beginLSM(lsmAddr);
  //beginBNO();

  //Calibration (IMU, High G, Baro) 
  //-> Determine direction/orientation of the rocket -> set g to the accel of the longitudinal axis
  //-> Reset height to 0
  //Avionics must be stabilized at launch position for the duration of calibration 
  calibrateLPS(referencePressure);
  calibrateLSM(errorAccX, errorAccY, errorAccZ, errorGyrX, errorGyrY, errorGyrZ);
  //BNO is already calibrated but after the calibration of the other two sensors, should move and rotate the avionics to improve readings
  
}

void loop() {
  float g = errorAccZ //longtiudinal accel (CHANGE THIS TO THE RIGHT ORIENTATION)
  float gTrigger = 2 * g; //Set a g trigger to detect launch

  //Get baro data (Height, Pressure, Temp)
  LPS.getEvent(&pressure,&temp);
  float temperatureK = temp.temperature + 273.15;
  float altitudeMeters = (8.314 * temperatureK) / (0.02896 * 9.80665) * log(1013.25 / pressure.pressure);
  float Height = (44330.0 * (1.0 - pow(pressure.pressure/referencePressure, 0.1903)));

  //Get IMU data and high G IMU
  //BNO is rated to 8g

  if (!events.liftoff) {
    if (/*longitudinal accel: lsm accel || bno accel */ accel.acceleration.z > gTrigger) /*(CHANGE THIS TO THE RIGHT ORIENTATION)*/{
      events.preLiftoff = false;
      events.liftoff = true;
    }
    event_code = events.preLiftoff;
    //Sampling rate?
  }

  if (events.liftoff) {
    event_code = events.liftoff;
    //Sampling rate?
    
    //Calculate height difference, set max height
    //Set a counter, if 10 continuous decreasing meaurement in height than fire charge. Otherwise, reset counter to 0
    lastHeight=currentHeight;
    currentHeight=Height;
    if(currentHeight > maxHeight){
      maxHeight=currentHeight;
      if (count>0) {
        count = 0;
      }
    }
    else {
      count += 1;
    }

    checkEvents()

    //Check apogee detection if yes then fire pyro charge
    if (events.apogee && pyro1.pyroArmed && digitalRead(pins.pyro1Fire) == LOW) {
      //log the time at pyro charge firing
      startFireTime = millis();
      digitalWrite(pins.pyro1Fire, HIGH);
    }

    //When the pyro is fired, keep checking if firing time is reached
    if (events.apogee && pyro1.pyroArmed && digitalRead(pins.pyro1Fire) == HIGH) {
      currentFireTime = millis();
      if (currentFireTime - startFireTime >= fireTime) {
        digitalWrite(pins.pyro1Fire, LOW);
        
        pyro1.pyroArmed = false;
        event.apogeeDeploy = true;
        event_code = codes.parachute;
      }
    }
  }

  //Log to SD card
  //Log: time, temperature, altitude, height, 3 acc, 3 rotation, 3 mag, baro speed, acc speed, ypr, heading, event code
  
  //Send to ground station

}

void checkEvents() {
  //Arm Pyro
  if (events.liftoff && !pyro1.pyroArmed && !events.apogeeDeploy && currentHeight >= altLockOut) {
    pyro1.pyroArmed = true;
  }
  //Motor burnout
  if (events.liftoff && !events.boosterBurnout && /*longitudinal accel: lsm accel || bno accel*/ accel.acceleration.z <= 0) {
    events.boosterBurnout = true;
    event_code = codes.boosterBurnout;
  }
  //Apogee when 10 continuous decreasing measurments have been recorded
  if (events.liftoff && !events.apogee && count >= 10) {
    events.apogee = true;
    event_code = codes.apogee;
  }
  //Parachute Deployment

  if (events.liftoff && !events.descent && currentHeight < maxHeight - 50) {
    events.descent = true;
    event_code = codes.descent;
  }
  //Touchdown 
  if (events.descent && !events.touchdown && currentHeight < 10) {
    events.touchdown = true;
    event_code = codes.touchdown;
  }
}

void beginLPS(uint8_t address) {
  if (!LPS.begin_I2C(address)) { // LPS22 conection.
    Serial.println("Failed to find LPS22 chip");
    while (1) { delay(10); }}
  Serial.println("LPS22 Found!");
  LPS.setDataRate(LPS22_RATE_10_HZ); // Options: 1,10,25,50,75, ONE_SHOT
}

void beginLSM(uint8_t address) {
  if (!LSM.begin_I2C(address)) {
    Serial.println("Failed to find LSM6DSOX chip");
    while (1) {delay(10); }}
  Serial.println("LPS22 Found!");
  LSM.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  LSM.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);  
  LSM.setAccelDataRate(LSM6DS_RATE_104_HZ);
  LSM.setGyroDataRate(LSM6DS_RATE_104_HZ);
}

/*
void beginBNO() {
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }}
  Serial.println("BNO08x Found!");

  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    Serial.println("Could not enable magnetic field calibrated");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    Serial.println("Could not enable linear acceleration");
  }
  if (!bno08x.enableReport(SH2_GRAVITY)) {
    Serial.println("Could not enable gravity");
  }
}
*/

void calibrateLPS(float& referencePressure) {

  for (int cal_int = 0; cal_int < 1000 ; cal_int ++)  // Calculando la presión ambiente "pref"
  {
    LPS.getEvent(&pressure,&temp);
    float currentPressure = pressure.pressure;
    referencePressure += currentPressure;
    Serial.println(currentPressure);
  }
  referencePressure = referencePressure / 1000;
}

void calibrateLSM(float& errorAccX, float& errorAccY, float& errorAccZ, float& errorGyrX, float& errorGyrY, float& errorGyrZ) {
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++)  // Calculando la presión ambiente "pref"
  {
    LSM.getEvent(&accel, &gyro, &temp);

    float currentAccX = accel.acceleration.x;
    float currentAccY = accel.acceleration.y;
    float currentAccZ = accel.acceleration.z;
    
    float currentGyrX = gyro.gyro.x;
    float currentGyrY = gyro.gyro.y; 
    float currentGyrZ = gyro.gyro.z;
    
    errorAccX += currentAccX;
    errorAccY += currentAccY;
    errorAccZ += currentAccZ;
    errorGyrX += currentGyrX;
    errorGyrX += currentGyrX;
    errorGyrX += currentGyrX;
  }
  errorAccX = errorAccX / 1000;
  errorAccY = errorAccY / 1000;
  errorAccZ = errorAccZ / 1000;
  errorGyrX = errorGyrX / 1000;
  errorGyrY = errorGyrY / 1000;
  errorGyrZ = errorGyrZ / 1000;
}

//Not finished
/*
void getBNO() {
  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  switch (sensorValue.sensorId) {

  case SH2_ACCELEROMETER:
    Serial.print("Accelerometer - x: ");
    Serial.print(sensorValue.un.accelerometer.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.accelerometer.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.accelerometer.z);
    break;
  case SH2_GYROSCOPE_CALIBRATED:
    Serial.print("Gyro - x: ");
    Serial.print(sensorValue.un.gyroscope.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.gyroscope.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.gyroscope.z);
    break;
  case SH2_MAGNETIC_FIELD_CALIBRATED:
    Serial.print("Magnetic Field - x: ");
    Serial.print(sensorValue.un.magneticField.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.magneticField.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.magneticField.z);
    break;
  case SH2_LINEAR_ACCELERATION:
    Serial.print("Linear Acceration - x: ");
    Serial.print(sensorValue.un.linearAcceleration.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.linearAcceleration.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.linearAcceleration.z);
    break;
  case SH2_GRAVITY:
    Serial.print("Gravity - x: ");
    Serial.print(sensorValue.un.gravity.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.gravity.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.gravity.z);
    break;
}
*/
