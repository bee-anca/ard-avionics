//--------------------------------------------------------------
//                  LIBRERÍAS NECESARIAS  
//--------------------------------------------------------------
#include <SPI.h>
#include <Wire.h>
#include <MS5611.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "mySD.h"
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

//--------------------------------------------------------------
//            DEFINICIÓN PINES SEGÚN ESQUEMÁTICO 
//--------------------------------------------------------------
#define ss 5                  // Definiendo pines para LoRa
#define rst 14
#define dio0 2
#define cssd 4                // Definiendo pines para SD
#define BUZZER 26             // Definiendo pines para dispositivos alternos
#define LED 25
#define PYRO_APOGEE 12
#define PYRO_MAIN 13

//--------------------------------------------------------------
//            DEFINICIÓN DE VARIABLES GLOBALES
//--------------------------------------------------------------
String trama;                 // Variables trama de datos
String trama_to_send;
String BEG = "{";
String END = "}";
String SEP = ",";
float presionActual = 0;      // Variables barométrico
float presion0 = 0;
float Temperatura = 0;
float Presion = 0;
float Altura = 0;
bool apogeo_barometer = false;
bool main_parachute = false;
float alt_anterior =0;
float alt_actual=0;
float altmax = 0;
const int margen_altura = 2;
float apogeo2;
float altura_1=0;
float altura_2=0;
bool apogeo_imu = false;      // Variables IMU
float x_acc = 0;
float y_acc = 0;
float z_acc = 0;
float x_gyr = 0;
float y_gyr = 0;
float z_gyr = 0;
float x_magneto = 0;
float y_magneto = 0;
float z_magneto = 0;
double apogee_detection = 0;
double media_cuadratica = 0;
int counter =0;

volatile unsigned tiempoActual = 0;
volatile unsigned tiempoAnterior = 0;
volatile unsigned intervaloTiempo = 0;

//--------------------------------------------------------------
//                 CONSTRUCTORES Y DEMÁS
//--------------------------------------------------------------
MS5611 MS5611(0x77);                              // LoRa I2C Address
File myFile;                                      // SD file name
#define BNO055_SAMPLERATE_DELAY_MS (100)          // IMU BNO055 - tasa de muestreo
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);  // IMU BNO055 - I2C address



void setup() {
Serial.begin(115200);
SPI.begin();
MS5611.begin();
pinMode(cssd, OUTPUT);
pinMode(BUZZER, OUTPUT);
pinMode(LED, OUTPUT);
pinMode(PYRO_APOGEE, OUTPUT);
pinMode(PYRO_MAIN, OUTPUT);

//--------------------------------------------------------------------------------------------------------------
  if (MS5611.begin() == true)     // Validación status MS5611
  {
    Serial.println("MS5611 DETECTADO - READY.");
    digitalWrite(BUZZER,HIGH);
    delay(500);
    digitalWrite(BUZZER,LOW);
  }
    else{
    Serial.println("MS5611 FAILED.");
    while (1){
  }}
  MS5611.setOversampling(OSR_ULTRA_HIGH);
//--------------------------------------------------------------------------------------------------------------
  if (SD.begin(cssd))  // Validación status SD
    {
    Serial.println("TARJETA SD DETECTADA - READY.");
    digitalWrite(BUZZER,HIGH);
    delay(500);
    digitalWrite(BUZZER,LOW);
    }
    else{
    Serial.println("TARJETA SD FAILED.");
    return;
  }
//--------------------------------------------------------------------------------------------------------------
    for (int cal_int = 0; cal_int < 500 ; cal_int ++)  // Calculando la presión ambiente "pref"
    {
    MS5611.read();          
    delay(10);
    presionActual = MS5611.getPressure();
    presion0 += presionActual;
    Serial.println(presionActual);
    }
    presion0 = presion0 / 500;
    digitalWrite(BUZZER,HIGH);
    delay(100);
    digitalWrite(BUZZER,LOW);
//--------------------------------------------------------------------------------------------------------------
  if(!bno.begin())      // Validación status IMU
  {
    Serial.print("BNO055 FAILED");
    while(1);
  }
     Serial.println("BNO055 DETECTADO - READY");
     digitalWrite(BUZZER,HIGH);
     delay(500);
     digitalWrite(BUZZER,LOW);
     bno.setExtCrystalUse(true);                       // IMU BNO055 - Configuración necesaria
}

void loop() {
  trama += BEG + String(counter) + SEP + String(tiempoAnterior/1000)+SEP+String(intervaloTiempo);
  tiempoActual = millis();
  trama += SEP + String(tiempoActual/1000); 
  intervaloTiempo = (double) tiempoActual - tiempoAnterior;
  tiempoAnterior = tiempoActual;
    
    MS5611.read();
    Temperatura = MS5611.getTemperature();
    Presion = MS5611.getPressure();
    Altura = (44330.0 * (1.0 - pow(Presion/presion0, 0.1903)));
    trama += SEP + String(Temperatura)+SEP+String(Presion)+SEP+String(Altura);
    alt_anterior=alt_actual;
    alt_actual=Altura;
    if(alt_actual>altmax){
    altmax=alt_actual;
    }
    trama += SEP + String(altmax);
    apogeo2 = ((alt_actual+altura_1+altura_2)/3) + margen_altura - altmax;
    if((apogeo2<=-5)&&(apogeo_barometer==false)){
    apogeo_barometer = true;
    digitalWrite(BUZZER,HIGH);
    //digitalWrite(PYRO_APOGEE, HIGH);
    delay(1000);
    //digitalWrite(PYRO_APOGEE, LOW);
    digitalWrite(BUZZER,LOW);
        }
      if((Altura <= 60)&&(apogeo_barometer==true)&&(main_parachute==false)){
        main_parachute = true;
        digitalWrite(BUZZER,HIGH);
        //digitalWrite(PYRO_MAIN, HIGH);
        delay(1000);
        //digitalWrite(PYRO_MAIN, LOW);
        digitalWrite(BUZZER,LOW);
        }
    trama += SEP + String(apogeo2) + SEP +String(apogeo_barometer)+ SEP+ String(main_parachute);
    altura_2 = altura_1;
    altura_1=alt_actual;

      // Sensor IMU
      imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      imu::Vector<3> magneto = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      x_acc = accel.x();
      y_acc = accel.y();
      z_acc = accel.z();
      x_gyr = gyro.x();
      y_gyr = gyro.y();
      z_gyr = gyro.z();
      x_magneto = magneto.x();
      y_magneto = magneto.y();
      z_magneto = magneto.z();
      media_cuadratica = sqrt(pow(x_acc,2)+pow(y_acc,2)+pow(z_acc,2));
      trama += SEP+ String(x_acc) + SEP + String(y_acc) + SEP + String(z_acc) + SEP + String(media_cuadratica);
      trama += SEP+ String(x_gyr) + SEP + String(y_gyr) + SEP + String(z_gyr) + SEP + String(x_magneto) + SEP + String(y_magneto) + SEP + String(z_magneto)+END;
      
        //------- TARJETA SD
        myFile = SD.open("NUCLEO0.txt", FILE_WRITE);
        if (myFile) {
        myFile.println(trama); 
        myFile.close(); // close the file
        }
      counter++;
      trama = "";
      delay(2);
}
