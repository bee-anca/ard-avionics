//Import Libraries
#include <Wire.h>

#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
Adafruit_LPS22 lps;
    sensors_event_t pressure;
    sensors_event_t temp;

//Define pins
typedef struct {
  //Pyro pins
  uint8_t pyro1Fire = ; 
  uint8_t pyro1Cont = ;
  uint8_t pyro2Fire = ;
  uint8_t pyro2Cont = ;
  //Beeper pin
  uint8_t beeper = ;
  //LED pin
  uint8_t ld1 = ;
  uint8_t ld2 = ;
  uint8_t ld3 = ;
  uint8_t ld4 = ;  
} pins;

//Define variables
uint8_t eventCode = 0;
float lastHeight =0;
float currentHeight=0;
float maxHeight = 0;
int count = 0;

//Set flight conditions
unsigned long apogeeDelay = 0; //Not sure if needed if apogee detection is based on moving average
unsigned long fireTime = 500; //milliseconds 
float altLockOut = 300; //Altitude lock out for pyro in meter
//float maxAngleOffVertical = ;

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

eventCode eventcode;

void setup() {
  //Set serial
  Serial.begin(115200);

  //Start SD card 

  //Set pin mode
  pinMode(pins.pyro1Fire,OUTPUT);
  pinMode(pins.pyro1Cont, INPUT)
  pinMode(pins.beeper, OUTPUT);
  pinMode(pins.ld1, OUTPUT);
  pinMode(pins.ld2, OUTPUT);
  pinMode(pins.ld3, OUTPUT);
  pinMode(pins.ld4, OUTPUT);

  digitalWrite(pins.pyro1Fire, LOW);

  //Start LoRa

  //Start sensors (IMU, High G, Baro)

  //Detect Continuity

  //Calibration (IMU, High G, Baro) 
  //-> Determine direction/orientation of the rocket -> set g to the accel of the longitudinal axis
  //-> Reset height to 0
  //Avionics must be stabilized at launch position for the duration of calibration 
  float referencePressure = calibrateLPS22();
  
  float g = //longtiudinal accel
  float gTrigger = 2 * g; //Set a g trigger to detect launch

}

void loop() {
  //Get baro data (Height, Pressure, Temp)
  lps.getEvent(&pressure,&temp);
  float temperatureK = temp.temperature + 273.15;
  float altitudeMeters = (8.314 * temperatureK) / (0.02896 * 9.80665) * log(1013.25 / pressure.pressure);
  float Height = (44330.0 * (1.0 - pow(pressure.pressure/referencePressure, 0.1903)));

  //Get IMU data and high G IMU
  //BNO is rated to 8g

  if (!events.liftoff) {
    if (/*longitudinal accel*/ > gTrigger) {
      events.preLiftoff = false;
      events.liftoff = true;
    }
    eventCode = eventcode.preLiftoff;
    //Sampling rate?
  }

  if (events.liftoff) {
    eventCode = eventcode.liftoff;
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
      unsigned long startFireTime = millis();
      digitalWrite(pins.pyro1Fire, HIGH);
    }

    //When the pyro is fired, keep checking if firing time is reached
    if (events.apogee && pyro1.pyroArmed && digitalRead(pins.pyro1Fire) == HIGH) {
      unsigned long currentFireTime = millis();
      if (currentFireTime - startFireTime >= fireTime) {
        digitalWrite(pins.pyro1Fire, LOW);
        
        pyro1.pyroArmed = false;
        event.apogeeDeploy = true;
        eventCode = eventcode.parachute;
      }
    }
  }

  //Log to SD card
  //Log: time, temperature, altitude, height, 3 acc, 3 rotation, 3 mag, baro speed, acc speed, ypr, heading, event code
  
  //Send to ground station

}

void checkEvents() {
  //Arm Pyro
  if (events.liftoff && !pyro1.pyroArmed && !event.apogeeDeploy && currentHeight >= altLockOut) {
    pyro1.pyroArmed = true;
  }
  //Motor burnout
  if (events.liftoff && !events.boosterBurnout && /*longitudinal accel*/ <=0) {
    events.boosterBurnout = true;
    eventCode = eventcode.boosterBurnout;
  }
  //Apogee
  if (events.liftoff && !events.apogee && count >= 10) {
    events.apogee = true;
    eventCode = eventcode.apogee;
  }
  //Parachute Deployment

  if (events.liftoff && !events.descent && /*altitude decreasing and negative acceleration*/) {
    events.descent = true;
    eventCode = eventcode.descent;
  }
  //Touchdown 
  if (events.descent && !events.touchdown && currentHeight < 10) {
    events.touchdown = true;
    eventCode = eventcode.touchdown;
  }
}

void beginLPS22() {
  if (!lps.begin_I2C()) { // LPS22 conection.
    Serial.println("Failed to find LPS22 chip");
    while (1) { delay(10); }}
    Serial.println("LPS22 Found!");
  lps.setDataRate(LPS22_RATE_10_HZ); // Options: 1,10,25,50,75, ONE_SHOT
}

float calibrateLPS22() {
  float referencePressure;

  for (int cal_int = 0; cal_int < 1000 ; cal_int ++)  // Calculando la presión ambiente "pref"
  {
    lps.getEvent(&pressure,&temp);
    float currentPressure = pressure.pressure;
    referencePressure += currentPressure;
    Serial.println(currentPressure);
  }
  referencePressure = referencePressure / 1000;

  return referencePressure
}
