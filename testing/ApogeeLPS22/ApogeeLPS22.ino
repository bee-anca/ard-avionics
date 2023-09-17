//--------------------------------------------------------------
//            LIBRARIES NEEDED
//--------------------------------------------------------------

#include <Wire.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include <SD.h>
#include <SPI.h>

//--------------------------------------------------------------
//            PIN DEFINITION - SCHEMATIC
//--------------------------------------------------------------
#define BUZZER 26            
#define LED 25
#define PYRO_APOGEE 12
#define PYRO_MAIN 13

const int chipSelect = BUILTIN_SDCARD;

//--------------------------------------------------------------
//            DEFINICIÓN DE VARIABLES GLOBALES
//--------------------------------------------------------------

float currentPressure = 0;      // Variables barométrico
float referencePressure = 0;
String dataString = "";
String BEG = "{";
String END = "}";
String SEP = ",";
float lastHeight =0;
float currentHeight=0;
float maxHeight = 0;
const int tresholdHeight = 2;
float apogeeDifference;
bool apogeeBarometer = false;
float H1=0;
float H2=0;

//--------------------------------------------------------------
//            CONSTRUCTORS AND MORE NEEDED
//--------------------------------------------------------------

Adafruit_LPS22 lps;
    sensors_event_t pressure;
    sensors_event_t temp;

void setup()
{
  Serial.begin(115200);
//--------------------------------------------------------------
  if (!lps.begin_I2C()) { // LPS22 conection.
    Serial.println("Failed to find LPS22 chip");
    while (1) { delay(10); }}
    Serial.println("LPS22 Found!");
  lps.setDataRate(LPS22_RATE_10_HZ); // Options: 1,10,25,50,75, ONE_SHOT

//--------------------------------------------------------------

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1) {    }}
  Serial.println("card initialized.");
//--------------------------------------------------------------
    
    for (int cal_int = 0; cal_int < 1000 ; cal_int ++)  // Calculando la presión ambiente "pref"
    {
    lps.getEvent(&pressure,&temp);
    currentPressure = pressure.pressure;
    referencePressure += currentPressure;
    Serial.println(currentPressure);
    }
    referencePressure = referencePressure / 1000;
//    digitalWrite(BUZZER,HIGH);
//    delay(100);
//    digitalWrite(BUZZER,LOW);
  
}

void loop() {

  lps.getEvent(&pressure,&temp);
  float temperatureK = temp.temperature + 273.15;
  float altitudeMeters = (8.314 * temperatureK) / (0.02896 * 9.80665) * log(1013.25 / pressure.pressure);
  float Height = (44330.0 * (1.0 - pow(pressure.pressure/referencePressure, 0.1903)));
  
    dataString += BEG + String(temperatureK) +SEP+ String(altitudeMeters) +SEP+ String(Height);

  lastHeight=currentHeight;
  currentHeight=Height;
  if(currentHeight > maxHeight){
  maxHeight=currentHeight;}
  
    dataString += SEP + String(maxHeight);

  apogeeDifference = ((currentHeight + H1 + H2)/3) + tresholdHeight - maxHeight;
  if((apogeeDifference <= -0.5)&&(apogeeBarometer==false)){
  apogeeBarometer = true;
  //digitalWrite(BUZZER,HIGH);
  //digitalWrite(PYRO_APOGEE, HIGH);
  delay(5000);
  //digitalWrite(PYRO_APOGEE, LOW);
  //digitalWrite(BUZZER,LOW);
  }
  //if((Altura <= 60)&&(apogeo_barometer==true)&&(main_parachute==false)){
  //main_parachute = true;
  //digitalWrite(BUZZER,HIGH);
  //digitalWrite(PYRO_MAIN, HIGH);
  //delay(1000);
  //digitalWrite(PYRO_MAIN, LOW);
  //digitalWrite(BUZZER,LOW);
  //}  
    dataString += SEP+ String(apogeeDifference) +SEP+ String(apogeeBarometer) +SEP;
  
  H2 = H1;
  H1=currentHeight;

//  Serial.print("Temperature Kelvin: ");Serial.print(temperatureK);Serial.println(" K");
//  Serial.print("Altitud: ");Serial.print(altitudeMeters);Serial.println(" m");
//  Serial.print("Alti: ");Serial.print(Height);Serial.println(" m");
//  Serial.print("Reference PRessure: ");Serial.print(referencePressure);Serial.println(" hPa");
//  Serial.print("Current PRessure: ");Serial.print(currentPressure);Serial.println(" hPa");
//  Serial.print("Current PRessure: ");Serial.print(pressure.pressure);Serial.println(" hPa");

    dataString += SEP+String(referencePressure)+SEP+String(currentPressure)+SEP+String(pressure.pressure);
 
  Serial.println(dataString);
  File dataFile = SD.open("datAvionics.txt", FILE_WRITE);
  if (dataFile) {
  dataFile.println(dataString);
  dataFile.close();
  } else {
    // if the file isn't open, pop up an error:
    Serial.println("error opening datalog.txt");
  }
  dataString = "";

}
