#include <bsec.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

#define ESP8266

static unsigned long TIME_TO_NEXT_SENDING = 1000;   /** Override from HTTP response payload */

static const char * SSID = "ESP-WeatherStation";
static const char * PASS = "esp32station";
static const uint8_t bssid[] = {0x30, 0xAE ,0xA4,0x98,0x83,0xB8}; /** In AP call WiFi.macAddress() -- "30:AE:A4:98:83:B8" */
static const IPAddress staticIP(192,168,4,2);
static const IPAddress gateway(192,168,4,1);
static const IPAddress subnet(255,255,255,0);

Adafruit_BME680 bme;
//Bsec util;

float voltage_ext;
float temp_ext;
float humidity_ext;
float pressure_ext;
float airIndex = 0;


void setup() {
    /** Disabling WiFi when waking up */
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin(); delay(1);

    system_deep_sleep_set_option(0);

    pinMode(2, OUTPUT);
    digitalWrite(2,HIGH); /*Turn off built-in led */
    
    Serial.begin(115200);   //Will be removed for 'production' to save battery
    Wire.begin();

    pinMode('A0', INPUT); /** Battery voltage divider */

    /*----------------------------- BME680 Sensor ------------------------------*/

    if (! bme.begin(0x77))  Serial.println("Couldn't find BME sensor, but keep working");
    
    /** Weather/Climate-monitor  Calibration */
    bme.setTemperatureOversampling(BME680_OS_1X);
    bme.setHumidityOversampling(BME680_OS_1X);
    bme.setPressureOversampling(BME680_OS_1X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_0);
    bme.setGasHeater(320, 150); 

    //util.begin(BME680_I2C_ADDR_SECONDARY, Wire);
    //checkutilStatus();
}


//Only for testing (serial output) --> will be removed in 'production'
void ambientMeasurement() {
    bme.performReading();
    temp_ext = bme.temperature;
    pressure_ext = bme.pressure / 100.0;
    humidity_ext = bme.humidity;
    airIndex = bme.gas_resistance / 1000.0;

    /*while (! util.run()) { // If no data is available
        Serial.println("BSEC calculations not ready");
        //return;
    }
    Serial.println("\nIAQ: " + String(util.iaq) + "\nAccuracy: " + String(util.iaqAccuracy) 
                 + "\nStatic-IAQ: " + String(util.staticIaq) +"\nCO2 " + String(util.co2Equivalent)
                 + "\nBreath-VOC" + String(util.breathVocEquivalent) );*/
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

    //res_to_batt = 1.00 MOhm
    //res_in_gnd = 240.50 kOhm
    return voltage_ext = (voltage_reading / nReadings) * 3.3 / 1024 ;
}


void sendData(){

  WiFi.forceSleepWake();   delay(1);
  /** Avoid saving network connection information to flash, 
   * and then reading back when it next starts the WIFI */
  WiFi.persistent(false);   
  WiFi.mode(WIFI_STA);  
  //WiFi.config(staticIP, gateway, subnet);
  //WiFi.begin(SSID, PASS, 1, bssid, true);  /** Faster with BSSID + channel*/
  WiFi.begin(SSID, PASS);
  
  byte retries = 0;
  while( WiFi.status() != WL_CONNECTED && retries<200 ){ 
    retries++;
    delay(10);
  }
  if(retries = 200 && WiFi.status() == WL_DISCONNECTED){
    WiFi.disconnect(true); delay( 1 );
    WiFi.mode(WIFI_OFF);
    Serial.println("No access point.");
    return;
  }
    
  HTTPClient http;
  /**GET requests with data in the query string
   * Hardcoded domain url for less power consumption 
   **/
  http.begin((String)"192.168.4.1/update" + "?temp=" + (String)temp_ext + "&hum=" + (String)humidity_ext + "&press=" + (String)pressure_ext
                                          + "&airIndex=" + (String)airIndex + "&volt=" + (String)voltage_ext);  
  if( http.GET() == 200){
    TIME_TO_NEXT_SENDING = (unsigned long) http.getString().toFloat();  /**Need 32bit variable */
    Serial.println("Data sent correctly");
    Serial.println("Server told me to sleep for: " + (String)(TIME_TO_NEXT_SENDING/1000U) + "s");
  }
  else  Serial.println("Error sending data");
  
  http.end();
  
  WiFi.disconnect(true); delay(1);
  WiFi.mode(WIFI_OFF);
}


void printToSerial() {
    /*--------------------------------------------------------------------------------*/
    Serial.print("Voltage: ");
    Serial.print(voltage_ext, 4);
    Serial.println("V");
    /*--------------------------------------------------------------------------------*/
    Serial.println("Temperature: " + (String) temp_ext + " °C");
    Serial.println("Pressure: " + (String) pressure_ext + " hPa");
    Serial.println("Humidity: " + (String) humidity_ext + " %RH");
    Serial.println("Air Index: " + (String) airIndex + " KOhms");
    Serial.println("");
    Serial.println("----------------------------------------");
    /*--------------------------------------------------------------------------------*/
}


/*void checkutilStatus(void)
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
*/


void loop() {
    ambientMeasurement();
    readBattery();
    printToSerial();
    while(TIME_TO_NEXT_SENDING == 1000){  //Syncs first time to weather-station
      sendData();
      delay(1000);
    }
    sendData();

    ESP.deepSleep(TIME_TO_NEXT_SENDING *1000U, WAKE_RF_DISABLED);  //Minutes expressed in milliseconds converted to microseconds
    delay(1); /*Needed for proper sleeping*/
}
