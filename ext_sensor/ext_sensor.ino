#include "bsec.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

#include <ESP8266WiFi.h>
#include <espnow.h>

#define ESP8266

uint64_t TIME_TO_NEXT_SENDING = 4;
u8 receiverAddress[] = {0x30, 0xAE, 0xA4, 0x98, 0x83, 0xB8};  /** In AP call WiFi.macAddress() -- "30:AE:A4:98:83:B8" */

Adafruit_BME680 bme;

typedef struct data_struct {
  float voltage_ext;
  float temp_ext;
  float humidity_ext;
  float pressure_ext;
  float airTVOC = 0;
} data_struct;
data_struct sensorData;

bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };
//Bsec util;
unsigned long start;
void setup() {
    start = millis();
    /** Disabling WiFi radio as soon as waking up */
    WiFi.mode(WIFI_OFF);  
    WiFi.forceSleepBegin();   //Causes error in espNOW sending

    Serial.begin(115200);   //Will be removed for 'production' to save battery
    Wire.begin();
    pinMode(A0, INPUT);  /** Battery voltage divider input*/


    /*-------------------------------- BME680 Sensor ---------------------------------*/
    if (! bme.begin(0x77, false))  Serial.println("Couldn't find BME sensor, but keep working");
    
    /** Weather/Climate-monitor  Calibration */
    bme.setTemperatureOversampling(BME680_OS_1X);
    bme.setHumidityOversampling(BME680_OS_1X);
    bme.setPressureOversampling(BME680_OS_1X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_0);
    bme.setGasHeater(320, 150); 

    /*------------------------------ Air quality API config. --------------------------*/
    //util.begin(BME680_I2C_ADDR_SECONDARY, Wire);
    /** Sets the desired sensors and the sample rates */
    //util.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_ULP);
    //checkutilStatus();
}


void ambientMeasurement() {
    
    if(!bme.performReading() ) 
      Serial.println("Error reading BME");
    sensorData.temp_ext = bme.temperature;
    sensorData.pressure_ext= bme.pressure / 100.0;
    sensorData.humidity_ext = bme.humidity;
    sensorData.airTVOC = bme.gas_resistance / 1000.0;

    sensorData.voltage_ext = analogRead(A0) * 3.3F / 1024.0F * 4.711F;
    
    /*Serial.println("\nIAQ: " + String(util.iaq) + "\nAccuracy: " + String(util.iaqAccuracy) 
                 + "\nStatic-IAQ: " + String(util.staticIaq) +"\nCO2 " + String(util.co2Equivalent)
                 + "\nBreath-VOC" + String(util.breathVocEquivalent) );*/
}


/*void computeAirQuality(){
    if(! util.run()) {
        Serial.println("BSEC calculations not ready");
        return;
    }
    Serial.println("\nRaw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%], Static IAQ, CO2 equivalent, breath VOC equivalent");
    String output;
      output += ", " + String(util.rawTemperature);
      output += ", " + String(util.pressure);
      output += ", " + String(util.rawHumidity);
      output += ", " + String(util.gasResistance);
      output += ", " + String(util.iaq);
      output += ", " + String(util.iaqAccuracy);
      output += ", " + String(util.temperature);
      output += ", " + String(util.humidity);
      output += ", " + String(util.staticIaq);
      output += ", " + String(util.co2Equivalent);
      output += ", " + String(util.breathVocEquivalent);
      Serial.println(output);
}*/


/*void checkutilStatus(void)
{
  if (util.status != BSEC_OK)
    if (util.status < BSEC_OK)
      Serial.println( "BSEC error code : " + String(util.status));
    else
      Serial.println( "BSEC warning code : " + String(util.status));

  if (util.bme680Status != BME680_OK) 
    if (util.bme680Status < BME680_OK)
      Serial.println( "BME680 error code : " + String(util.bme680Status));
    else
      Serial.println( "BME680 warning code : " + String(util.bme680Status));
}*/


void onDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus == 0)  
    Serial.println("Data sent correctly");
  else  Serial.println("Delivery fail");

  esp_now_deinit();
}


void sendData(){

  while(!WiFi.forceSleepWake()){
    Serial.println("Can't wake wifi");
    delay(1);
  }
  while(!WiFi.mode(WIFI_STA) ){
    Serial.println("Wifi STA not ready");
    delay(1);
  }
  WiFi.setOutputPower(20.5);

  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(receiverAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_register_send_cb(onDataSent);   // Callback function
  esp_now_send(NULL, (uint8_t *) &sensorData, sizeof(sensorData));
}


void printToSerial() {
    Serial.print("\nVoltage: ");
    Serial.print(sensorData.voltage_ext, 4);
    Serial.println("V");
    /*--------------------------------------------------------------------------------*/
    Serial.println("Temperature: " + (String) sensorData.temp_ext + " °C");
    Serial.println("Pressure: " + (String) sensorData.pressure_ext + " hPa");
    Serial.println("Humidity: " + (String) sensorData.humidity_ext + " %RH");
    Serial.println("Air Index: " + (String) sensorData.airTVOC + " KOhms");
    Serial.println("\n-----------------------------------------");
}


void loop() {
    ambientMeasurement();
    //computeAirQuality();
    printToSerial();
    sendData();

    Serial.println("I took: "+ (String)(start-millis()) + " millis. to complete a cycle");
    ESP.deepSleep(TIME_TO_NEXT_SENDING *1E6, WAKE_RF_DEFAULT);
    delay(1); /*Needed for proper sleeping*/
}
