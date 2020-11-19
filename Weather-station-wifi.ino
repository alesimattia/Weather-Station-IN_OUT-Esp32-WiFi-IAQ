#include <Adafruit_BMP280.h>

#include <Adafruit_BME680.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <RTClib.h>
#include <driver/adc.h>

#define batt_in 32
#define rx_pin 2

/*-------------------- Sensori ------------------*/
RTC_DS3231 rtc;
Adafruit_BME680 bme;
Adafruit_BMP280 bmp;


/*------------------ Variabili globali -------------------*/

static const char daysOfTheWeek[7][12] = {"Domenica", "Lunedì", "Martedì", "Mercoledì", "Giovedì", "Venerdì", "Sabato"};
DateTime currentTime;
float voltage;
int vPercent;
float voltage_ext;
int vPercent_ext;
float temp;
float temp_ext;
float humidity;
float humidity_ext;
float pressure;
float pressure_ext;
float airIndex;


void setup() {

  Serial.begin(115200);
  Wire.begin(); //I2C start
  
  /*------------------- Inizializzazione sensori ---------------------*/
  if (! rtc.begin() )   Serial.println("Couldn't find RTC");
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  if (! bmp.begin(0x76) )   Serial.println("Couldn't find BMP");
  
  adcAttachPin(batt_in);
  adcStart(batt_in);
  pinMode(batt_in, INPUT);
  /*------------------------------------------------------------------*/
}


void readBattery() {
  const byte nReadings = 64;
  float voltage_reading = 0;
  float voltage = 0;
 
  for (byte x = 0; x < nReadings; x++) {
    voltage_reading = voltage_reading + analogRead(batt_in);
  }
  /** 5.1 è il voltaggio max da leggere --> resistori tarati su quel livello
      r1 (to device) 1Mohm --- r2 (to gnd) 1Mohm
      Nel caso di resistori diversi moltiplicare per  r1/(r1+r2)
      In questo caso il fattore di conversione è 1M/(1M+1M)
  */
  voltage = (voltage_reading / 64) * 3.3 / 4095 / 2;
  vPercent = map(voltage, 3.2, 4.21, 0, 100);
}


void getTime() {
  currentTime = rtc.now();
  
  Serial.print((String)currentTime.day() + "/" + (String)currentTime.month() + "/" + (String)currentTime.year() +
               "  " + (String)daysOfTheWeek[currentTime.dayOfTheWeek()] + "  " + (String)currentTime.hour() +
               ":" + (String) currentTime.minute() + ":" + (String)currentTime.second() );
  //Misura circa 1°C in eccesso
  Serial.println("  RTC-Temp: " + (String) (rtc.getTemperature() - 1) );
}


void ambientMeasurement() {
  temp = bmp.readTemperature();
  pressure = bmp.readPressure()/100;
}


void printToSerial(){
  
  /*--------------------------------------------------------------------------------*/
  Serial.print("Voltage: " + (String)voltage +"V");
  if(voltage > 0){
    Serial.println("\t" + (String)vPercent + "%");
  }
  else Serial.println("\t 0% \n");
  /*--------------------------------------------------------------------------------*/
  Serial.println("Temperature: " + (String) temp + " °C");
  Serial.println("Pressure: " + (String) pressure + " hPa");
  Serial.println();
  /*--------------------------------------------------------------------------------*/
  Serial.println("----------------------------------------");
  
}


void loop() {
  
  readBattery();
  getTime();
  ambientMeasurement();
  
  printToSerial(); 
  delay(4000);
}
