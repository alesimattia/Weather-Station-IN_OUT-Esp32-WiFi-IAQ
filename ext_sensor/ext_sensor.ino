#include <Adafruit_BMP280.h>

#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>

#define TIME_TO_SLEEP  4
#define TIME_TO_LISTEN_HTTP 30
#define batt_in A0


Adafruit_BMP280 bmp;

const char* ssid = "ESP-sensor";
const char* password = "myesp8266";
AsyncWebServer server(80);

float voltage_ext;
int vPercent_ext;
float temp_ext;
float humidity_ext;
float pressure_ext;
float airIndex;


void setup() {
    Serial.begin(115200);
    Wire.begin(12,13); //I2C start
    system_deep_sleep_set_option(0);

    pinMode(batt_in, INPUT);

    /*---------------------- bmp280 Sensor -------------------------*/
    pinMode(15,OUTPUT);     //GND
    digitalWrite(15,LOW);
    
    if (! bmp.begin(0x76))  Serial.println("Couldn't find bmp");
    //Weather - Climate Monitor calibration
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED, 
                    Adafruit_BMP280::SAMPLING_X1, // temperature 
                    Adafruit_BMP280::SAMPLING_X1, // pressure
                    Adafruit_BMP280::FILTER_OFF ); 

    /*-------------------  WIFI async web server -------------------*/
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    //IPAddress local_IP(192, 168, 4, 1);
    /*-------------------- HTTP requests ---------------------------*/
    server.on("/voltage", HTTP_GET, [](AsyncWebServerRequest *request){
        char temp[6];
        dtostrf(readBattery(), 2, 4, temp);
        request->send_P(200, "text/plain",  temp );
    } );
    server.on("/temp", HTTP_GET, [](AsyncWebServerRequest *request){
        String ris = String( bmp.readTemperature() );
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
    server.on("/quality", HTTP_GET, [](AsyncWebServerRequest *request){
        char ris = (char)  bmp.readGas();
        request->send_P(200, "text/plain", &ris );
    } );*/

    server.begin();
    /*--------------------------------------------------------*/
}


void ambientMeasurement() {
    temp_ext = bmp.readTemperature();
    pressure_ext = bmp.readPressure() / 100;
    //humidity_ext = bmp.readHumidity();
}


float readBattery() {
    const byte nReadings = 64;
    float voltage_reading = 0;

    for (byte x = 0; x < nReadings; x++)
        voltage_reading += analogRead(batt_in);

    /** 5.1 maximum voltage to measure (USB)
     * Wemos D1 mini sees a res_gnd different to what applied (1M Ohm)
     * because of probabily an parallel internal resistor (A0 to GND) 
     * Thus the read is attenuated down (attenuation) by a factor of  
     * r2/(r1+r2) so multiply per 1/attenuation 
     * SEEMS NOT TO FOLLOW A LOGIC IN THE VOLTAGE DIVIDER -->
     * SPERIMENTAL VOLTAGE OFFSET of 4.662...
     */
    const float offset = 3.44;
    voltage_ext = (voltage_reading / nReadings) * 3.3 / 1024 * offset;
    return voltage_ext;
}


void printToSerial() {
    /*--------------------------------------------------------------------------------*/
    Serial.print("Voltage: ");
    Serial.print(voltage_ext, 6);
    Serial.print("V");
    Serial.println("\t" + (String) vPercent_ext + "%");
    /*--------------------------------------------------------------------------------*/
    Serial.println("Temperature: " + (String) temp_ext + " °C");
    Serial.println("Pressure: " + (String) pressure_ext + " hPa");
    Serial.println("Humidity: " + (String) humidity_ext + " %RH");
    Serial.println("Air Quality: " + (String) airIndex + " tVOC");
    Serial.println();
    Serial.println("----------------------------------------");
    /*--------------------------------------------------------------------------------*/
}


void loop() {
    /** A delay will let the ESP to stay on for the time 
     * to being reacheable to the weather-station via WIFI 
     * and get the data from ext_sensor device --> then sleep
    **/ 
    delay(TIME_TO_LISTEN_HTTP * 1000);
    ESP.deepSleep(TIME_TO_SLEEP * 1000000ULL, WAKE_RF_DEFAULT);
}
