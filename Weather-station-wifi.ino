#include <Adafruit_BMP280.h>
#include <Adafruit_BME680.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <RTClib.h>
#include <WiFi.h>
#include <HTTPClient.h>


#define batt_in 34
#define rx_pin 2

/*-------------------- Sensori ------------------*/
RTC_DS3231 rtc;
Adafruit_BME680 bme;
Adafruit_BMP280 bmp;

/*------------------ Variabili globali -------------------*/

const char* ssid = "ESP-sensor";
const char* password = "myesp8266";
const char* extSensor_volt = "http://192.168.4.1/volt";
const char* extSensor_temp = "http://192.168.4.1/temp";
const char* extSensor_pres = "http://192.168.4.1/pres";
const char* extSensor_hum = "http://192.168.4.1/hum";
const char* extSensor_quality = "http://192.168.4.1/quality";

static const char daysOfTheWeek[7][12] = {
    "Domenica",
    "Lunedì",
    "Martedì",
    "Mercoledì",
    "Giovedì",
    "Venerdì",
    "Sabato"
};
DateTime currentTime;

float voltage;
int vPercent;
float voltage_ext = 0;
int vPercent_ext = 0;
float temp;
float temp_ext = 0;
float humidity;
float humidity_ext = 0;
float pressure;
float pressure_ext = 0;
float airIndex = 0;
/*-------------------------------------------------------*/


void setup() {
    Serial.begin(115200);
    Wire.begin(); //I2C start

    adcAttachPin(batt_in);

    /*-------------------------- External sensor WIFI ----------------------*/
    WiFi.begin(ssid, password);
    Serial.print("Connecting: ");
    while(WiFi.status() != WL_CONNECTED) { 
        delay(500);
        Serial.print(".");
    }
    Serial.print("\n External sensor WIFI connected: ");
    Serial.println(WiFi.localIP());

    /*--------------------- Sensor initializing ----------------------*/
    if (!rtc.begin()) Serial.println("Couldn't find RTC");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    if (!bmp.begin(0x76)) Serial.println("Couldn't find BMP");
    /*------------------------------------------------------------------*/
}


int battPercentage(float v){
    /** Map seems to not work   vPercent = map(voltage, 3.1 , 4.20, 0, 100);
     * Minimum voltage 3.1V (as 0%)
     * Maximum voltage 4.2V (100%)
     * Percent value can rise up to 100% while charging
     * That's the desired behaviour to detect the "charging rate"
     */
    if (v >= 3.1)		        //4.2-3.1
        return (v - 3.1) * 100 / 1.1;
    else return 0;
}
void readBattery() {

    const byte nReadings = 64;
    float voltage_reading = 0;

    for (byte x = 0; x < nReadings; x++)
        voltage_reading += analogRead(batt_in);

    /** 5.1 maximum voltage to measure (USB)
     * r1 (to device) 1Mohm --- r2 (to gnd) 1Mohm
     * In case of not equal resistors the read value 
     * is scaled down by a factor of  r2/(r1+r2)
     * so multiply per 1/attenuation (now 2)
     * Sperimental offset in reading of 0.525V
     */
    voltage = ((voltage_reading / nReadings) * 3.3 / 4095 * 2) + 0.52;
    vPercent = battPercentage(voltage);
}


void getTime() {
    currentTime = rtc.now();

    Serial.print((String) currentTime.day() + "/" + (String) currentTime.month() + "/" + (String) currentTime.year() +
        "  " + (String) daysOfTheWeek[currentTime.dayOfTheWeek()] + "  " + (String) currentTime.hour() +
        ":" + (String) currentTime.minute() + ":" + (String) currentTime.second());
    //Misura circa 1°C in eccesso
    Serial.println("  RTC-Temp: " + (String)(rtc.getTemperature() - 1));
}


void ambientMeasurement() {
    temp = bmp.readTemperature() - 1.389;   //BME self heating;
    pressure = bmp.readPressure() / 100;
}


String httpGETRequest(const char* serverName) {
  HTTPClient http;
  // Your IP address with path or Domain name with URL path 
  http.begin(serverName);
  // Send HTTP POST request
  int httpResponseCode = http.GET();
  
  String payload = "--"; 
  
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  
  http.end();   // Free resources
  return payload;
}


void printToSerial() {
    /*--------------------------------------------------------------------------------*/
    Serial.print("Voltage: ");
    Serial.print(voltage, 3);
    Serial.print("V");
    Serial.println("\t" + (String) vPercent + "%");
    /*--------------------------------------------------------------------------------*/
    Serial.println("Temperature: " + (String) temp + " °C");
    Serial.println("Pressure: " + (String) pressure + " hPa");
    //Serial.println("Humidity: " + (String) humidity + " %RH");
    Serial.println();
    /*--------------------------------------------------------------------------------*/
    Serial.println("EXT_Temperature: " + (String) temp_ext + " °C");
    Serial.println("EXT_Pressure: " + (String) pressure_ext + " hPa");
    Serial.println("EXT_Humidity: " + (String) humidity_ext + " %RH");
    Serial.println("Air Quality: " + (String) airIndex + " %RH");
    Serial.println();
    Serial.println("----------------------------------------");
}


void loop() {

    readBattery();
    getTime();
    ambientMeasurement();
    getExternalSensor();

    printToSerial();
    delay(2500);
}