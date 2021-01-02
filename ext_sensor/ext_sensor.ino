#include <bsec.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>

#define esp8266
#define ARDUINO_ARCH_ESP32

/** Weather-Station (receiver) must be aware 
 * of these for synchronization */
static const byte TIME_TO_SLEEP = 4200;
static const byte TIME_TO_LISTEN_HTTP = 1;


const char* ssid = "ESP-sensor";
const char* password = "esp8266sensor";
AsyncWebServer server(80);

Adafruit_BME680 bme;
Bsec util;

float voltage_ext;
int vPercent_ext;
float temp_ext;
float humidity_ext;
float pressure_ext;
float airIndex = 0;
String airQuality = "NULL";


void setup() {

    Serial.begin(115200);   //Will be removed for 'production' to save battery
    Wire.begin(); //I2C start
    system_deep_sleep_set_option(0);
    pinMode('A0', INPUT);


    /*----------------------------- BME680 Sensor ------------------------------*/

    if (! bme.begin(0x77))  Serial.println("Couldn't find BME sensor, but keep working");
    
    /** Weather/Climate-monitor  Calibration */
    bme.setTemperatureOversampling(BME680_OS_1X);
    bme.setHumidityOversampling(BME680_OS_1X);
    bme.setPressureOversampling(BME680_OS_1X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_0);
    bme.setGasHeater(320, 150); 

    util.begin(BME680_I2C_ADDR_SECONDARY, Wire);
    checkutilStatus();
    
    /*-------------------------- WIFI async web server ----------------------*/
    WiFi.softAP(ssid, password);
    //Serial.println( WiFi.softAPIP() );  /** 192.168.4.1 */

    /*---------------------------- HTTP requests ---------------------------*/
    server.on("/voltage", HTTP_GET, [](AsyncWebServerRequest *request){
        char temp[6];
        dtostrf(readBattery(), 2, 4, temp);
        request->send_P(200, "text/plain",  temp );
    } );
    server.on("/temp", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/plain", String( bme.readTemperature() ).c_str() );
    } );
    server.on("/press", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/plain", String( bme.readPressure() / 100.0 ).c_str() );
    } );
    server.on("/hum", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/plain", String( bme.readHumidity() ).c_str() );
    } );
    server.on("/air", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/plain", String( bme.readGas() / 1000.0 ).c_str() );
    } );

    server.begin();
}


//Only for testing (serial output) --> will be removed in 'production'
void ambientMeasurement() {
    bme.performReading();
    temp_ext = bme.temperature;
    pressure_ext = bme.pressure / 100.0;
    humidity_ext = bme.humidity;
    airIndex = bme.gas_resistance / 1000.0;

    while (! util.run()) { // If no data is available
        Serial.println("BSEC calculations not ready");
        //return;
    }
    Serial.println("\nIAQ: " + String(util.iaq) + "\nAccuracy: " + String(util.iaqAccuracy) 
                 + "\nStatic-IAQ: " + String(util.staticIaq) +"\nCO2 " + String(util.co2Equivalent)
                 + "\nBreath-VOC" + String(util.breathVocEquivalent) );
}


/** 5.1 maximum voltage to measure (USB)
 * Voltage divider resistors are tuned for this.
 * Calculations are based on applying: 
 * res_to_bat-->(1M Ohm) -- res_to_gnd-->(1M Ohm)
 * Thus the readings are attenuated down by a factor of  
 * r2/(r1+r2) so multiply what red per 1/attenuation 
 **/
float readBattery() {
    //My charging circuit: (Max-4.2V)(Min-3.2)
    const byte nReadings = 64;
    float voltage_reading = 0;

    for (byte x = 0; x < nReadings; x++)
        voltage_reading += analogRead('A0');

    //res_to_batt = 1.05 MOhm
    //res_in_A0 = 315.75 kOhm
    return voltage_ext = (voltage_reading / nReadings) * 3.3 / 1024 ;;
}


void printToSerial() {
    /*--------------------------------------------------------------------------------*/
    Serial.print("Voltage: ");
    Serial.print(voltage_ext, 4);
    Serial.print("V");
    Serial.println("\t" + (String) vPercent_ext + "%");
    /*--------------------------------------------------------------------------------*/
    Serial.println("Temperature: " + (String) temp_ext + " °C");
    Serial.println("Pressure: " + (String) pressure_ext + " hPa");
    Serial.println("Humidity: " + (String) humidity_ext + " %RH");
    Serial.println("Air Index: " + (String) airIndex + " KOhms");
    Serial.println("");
    Serial.println("----------------------------------------");
    /*--------------------------------------------------------------------------------*/
}

void checkutilStatus(void)
{
  if (util.status != BSEC_OK) {
    if (util.status < BSEC_OK) {
      Serial.println( "BSEC error code : " + String(util.status));
    } else {
      Serial.println( "BSEC warning code : " + String(util.status));
    }
  }
 
  if (util.bme680Status != BME680_OK) {
    if (util.bme680Status < BME680_OK) {
      Serial.println( "BME680 error code : " + String(util.bme680Status));
    } else {
      Serial.println( "BME680 warning code : " + String(util.bme680Status));
    }
  }
}

/** A delay will let the ESP to stay on for the time 
     * to being reacheable to the weather-station via WIFI 
     * and get the data from ext_sensor device --> then sleep
**/ 
void loop() {
    readBattery();
    ambientMeasurement();
    printToSerial();
    delay(3000);
    //delay(TIME_TO_LISTEN_HTTP * 1000);
    //ESP.deepSleep(TIME_TO_SLEEP * 1E6, WAKE_RF_DEFAULT);
}
