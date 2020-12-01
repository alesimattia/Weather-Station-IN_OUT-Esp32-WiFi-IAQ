#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>

/** Weather-Station (receiver) must be aware 
 * of these for synchronization */
#define TIME_TO_SLEEP  20
#define TIME_TO_LISTEN_HTTP 40      
#define batt_in A0


const char* ssid = "ESP-sensor";
const char* password = "esp8266sensor";
AsyncWebServer server(80);

Adafruit_BMP280 bmp;

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

    pinMode(batt_in, INPUT);


    /*----------------------------- bmp280 Sensor ------------------------------*/

    if (! bmp.begin(0x76))  Serial.println("Couldn't find bmp, but keep working");
    
    /** Weather/Climate-monitor  Calibration */
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED, 
                    Adafruit_BMP280::SAMPLING_X1, // temperature 
                    Adafruit_BMP280::SAMPLING_X1, // pressure
                    Adafruit_BMP280::FILTER_OFF ); 


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
        String ris = String( bmp.readTemperature() - 1.5 );  //ESP self heathing
        request->send_P(200, "text/plain", ris.c_str() );
    } );
    server.on("/pres", HTTP_GET, [](AsyncWebServerRequest *request){
        String ris = String( bmp.readPressure() / 100 );  //hPA
        request->send_P(200, "text/plain", ris.c_str() );
    } );
    
    /*server.on("/hum", HTTP_GET, [](AsyncWebServerRequest *request){
        char ris = (char)  bmp.readHumidity();
        request->send_P(200, "text/plain", &ris );
    } );
    server.on("/air", HTTP_GET, [](AsyncWebServerRequest *request){
        char ris = (char)  bmp.readGas();
        request->send_P(200, "text/plain", &ris );
    } );*/

    server.begin();
}


//Only for testing (serial output) --> will be removed in 'production'
void ambientMeasurement() {
    temp_ext = ( bmp.readTemperature() - 1.5);  //BME self heating
    pressure_ext = bmp.readPressure() / 100;
    //humidity_ext = bme.readHumidity();
    //airIndex = bme.readGas();
}


/** 5.1 maximum voltage to measure (USB)
 * Voltage divider resistors are tuned for this.
 * Calculations are based on applying: 
 * res_to_bat-->(1M Ohm) -- res_to_gnd-->(1M Ohm)
 * Thus the readings are attenuated down by a factor of  
 * r2/(r1+r2) so multiply what red per 1/attenuation 
 **/
float readBattery() {

    const byte nReadings = 64;
    float voltage_reading = 0;

    for (byte x = 0; x < nReadings; x++)
        voltage_reading += analogRead(batt_in);

    /** Wemos D1 Mini has an internal voltage divider to measure up 
     * to 3.3V;  Bare ESP board can measure up to 1V
     * My board is actually 'damaged' so I have to applly a sperimental offset
    **/
    const float offset = 3.42;
    voltage_ext = (voltage_reading / nReadings) * 3.3 / 1024 * offset;
    return voltage_ext;
}


void printToSerial() {
    /*--------------------------------------------------------------------------------*/
    Serial.print("\nVoltage: ");
    Serial.print(voltage_ext, 6);
    Serial.print("V");
    Serial.println("\t" + (String) vPercent_ext + "%");
    /*--------------------------------------------------------------------------------*/
    Serial.println("Temperature: " + (String) temp_ext + " °C");
    Serial.println("Pressure: " + (String) pressure_ext + " hPa");
    //Serial.println("Humidity: " + (String) humidity_ext + " %RH");
    //Serial.println("Air Index: " + (String) airIndex + " tVOC");
    Serial.println();
    Serial.println("----------------------------------------");
    /*--------------------------------------------------------------------------------*/
}


/** A delay will let the ESP to stay on for the time 
     * to being reacheable to the weather-station via WIFI 
     * and get the data from ext_sensor device --> then sleep
    **/ 
void loop() {
    readBattery();
    ambientMeasurement();
    printToSerial();

    delay(TIME_TO_LISTEN_HTTP * 1000);
    ESP.deepSleep(TIME_TO_SLEEP * 1000000ULL, WAKE_RF_DEFAULT);
}
