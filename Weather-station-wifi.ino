#include <Adafruit_BME680.h>
#include <bme680.h>
#include <bme680_defs.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <RTClib.h>
#include <driver/adc.h>

#define batt_in 32
#define rx_pin 2


static const char daysOfTheWeek[7][12] = {"Domenica", "Lunedì", "Martedì", "Mercoledì", "Giovedì", "Venerdì", "Sabato"};

float voltage;

/*--------------- Oggetti sensori ---------------*/
RTC_DS3231 rtc;
Adafruit_BME680 bme;



void setup() {

  Serial.begin(115200);
  Wire.begin(); //I2C start
  adcAttachPin(batt_in);
  pinMode(batt_in, INPUT);
  
  /*--------------- Inizializzazione sensori ---------------*/
  if (! rtc.begin() ) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (! bme.begin() ) {
    Serial.println("Couldn't find BME");
    //while(1);
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  /*----------------------------------------------------*/

}


void readBattery() {
  adcStart(batt_in);
  const byte nReadings = 64;
  float voltage_reading = 0;
  float voltage = 0;
 
  for (byte x = 0; x < nReadings; x++) {
    voltage_reading = voltage_reading + analogRead(batt_in);
  }
  /** 5.1 è il voltaggio max da leggere --> resistori tarati su quel livello
      r1 (to device) 1Mohm --- r2 (to gnd) 1Mohm
      Nel caso di resistori diversi moltiplicare per (r1/r2)
      In questo caso il fattore di conversione è 1/2
  */
  voltage = (voltage_reading / 64) * 3.3 / 4095 / 2;
  Serial.print("Voltage: " + (String) voltage);
  if(voltage > 0){
    Serial.println("\t" + (String) map(voltage, 3.2, 4.2, 0, 100)  + "%");
  }
  else Serial.println("\t 0%");
}


void getTime() {
  DateTime now = rtc.now();

  Serial.print((String)now.day() + "/" + (String)now.month() + "/" + (String)now.year() +
               "  " + (String)daysOfTheWeek[now.dayOfTheWeek()] + "  " + (String)now.hour() +
               ":" + (String) now.minute() + ":" + (String)now.second() );
  //Misura circa 1°C in eccesso
  Serial.println("  RTC-Temp: " + (String) (rtc.getTemperature() - 1) );
}


void ambientMeasurement() {
  Serial.println("----------------------------------------");
  //bme.getPressureSensor()->printSensorDetails();
  Serial.println("Temperature: " + (String) bme.readTemperature() + " °C");
  Serial.println("Pressure: " + (String) bme.readPressure() + " Pa");
  Serial.println("Humidity: " + (String) bme.readHumidity() + " %RH");
}


void loop() {
  Serial.println("----------------------------------------");
  readBattery();
  getTime();
  //ambientMeasurement();

  Serial.println();
  delay(2000);
}
