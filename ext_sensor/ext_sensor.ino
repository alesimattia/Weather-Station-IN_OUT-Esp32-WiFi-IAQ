#include <bsec.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

#include <ESP8266WiFi.h>
#include <espnow.h>

#define ESP8266

static unsigned long TIME_TO_NEXT_SENDING = 3000;   /** Override from HTTP response payload */

//static const char * SSID = "ESP-WeatherStation";
//static const char * PASS = "esp32station";
u8 receiverAddress[] = {0x30, 0xAE, 0xA4, 0x98, 0x83, 0xB8}; /** In AP call WiFi.macAddress() -- "30:AE:A4:98:83:B8" */

Adafruit_BME680 bme;
//Bsec util;

typedef struct data_struct {
  float voltage_ext;
  float temp_ext;
  float humidity_ext;
  float pressure_ext;
  float airTVOC = 0;
} data_struct;
data_struct sensorData;


void setup() {
    /** Disabling WiFi when waking up */
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin(); delay(1);

    system_deep_sleep_set_option(0);

    /*pinMode(2, OUTPUT);
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
    sensorData.temp_ext = bme.temperature;
    sensorData.pressure_ext= bme.pressure / 100.0;
    sensorData.humidity_ext = bme.humidity;
    sensorData.airTVOC = bme.gas_resistance / 1000.0;

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
    return sensorData.voltage_ext = (voltage_reading / nReadings) * 3.3 / 1024 ;
}


void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus == 0)  
    Serial.println("Data sent correctly");
  else  Serial.println("Delivery fail");

  esp_now_unregister_recv_cb();
  esp_now_unregister_send_cb();
  esp_now_deinit();
  //WiFi.disconnect(true); delay(1); /**Includes wifi radio shut-down */
}


void sendData(){

  WiFi.forceSleepWake();  delay(1);
  /** Avoid saving network connection information to flash, 
   * and then reading back when it next starts the WIFI */
  //WiFi.persistent(false); 
  if(!WiFi.mode(WIFI_STA))
    Serial.println("Error during Wifi switch-on");
  //WiFi.setOutputPower(20);

  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(receiverAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_register_send_cb(OnDataSent);   /** Callback function */
  esp_now_send(NULL, (uint8_t *) &sensorData, sizeof(sensorData));

  /*http.begin((String)"192.168.4.1/update" + "?temp=" + (String)temp_ext + "&hum=" + (String)humidity_ext + "&press=" + (String)pressure_ext
                                          + "&airTVOC=" + (String)airTVOC + "&volt=" + (String)voltage_ext);  */
}


void printToSerial() {
    /*--------------------------------------------------------------------------------*/
    Serial.print("Voltage: ");
    Serial.print(sensorData.voltage_ext, 4);
    Serial.println("V");
    /*--------------------------------------------------------------------------------*/
    Serial.println("Temperature: " + (String) sensorData.temp_ext + " °C");
    Serial.println("Pressure: " + (String) sensorData.pressure_ext + " hPa");
    Serial.println("Humidity: " + (String) sensorData.humidity_ext + " %RH");
    Serial.println("Air Index: " + (String) sensorData.airTVOC + " KOhms");
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
    sendData();

    delay(TIME_TO_NEXT_SENDING);
    //ESP.deepSleep(TIME_TO_NEXT_SENDING *1000U, WAKE_RF_DISABLED);  //Minutes expressed in milliseconds converted to microseconds
    delay(1); /*Needed for proper sleeping*/
}
