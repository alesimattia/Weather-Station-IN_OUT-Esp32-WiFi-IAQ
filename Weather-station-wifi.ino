#include <TFT_eSPI.h>
#include <User_Setup.h>
#include "Custom_font.h"

#include <Wire.h>

#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_HDC1000.h>
#include <DHT.h>    //heat-index

#include <WiFi.h>
#include <HTTPClient.h>
#include <ESPAsyncWebServer.h>

#include <ArduinoJson.h>
#include "Connect.h"    //Weather api parameters
#include "Weather_icons.h"

#define ESP32


/*-------------------- Sensori ------------------*/
RTC_DS3231 rtc;
Adafruit_BMP280 bmp = Adafruit_BMP280();
Adafruit_HDC1000 hdc = Adafruit_HDC1000();
DHT util = DHT(NULL,DHT22); //just for heat-index

const byte batt_in = 34U;


/*-------------------------------------------- Variabili globali ----------------------------------------*/
const unsigned long TIME_TO_SLEEP = 30;
unsigned short TIME_TO_NEXT_HTTP = 60;  //Seconds - sent and overriden by ext_sensor
unsigned short call_miss = 0;    /** After 2 ext_sensor CYCLE data missing, writes 0 on the struct*/

/*---------------- Access point -------------------*/
const char* ap_ssid = "ESP-WeatherStation";
const char* ap_password = "esp32station";
const IPAddress localIP(192, 168, 4, 1);
const IPAddress gateway(192, 168, 4, 1);
const IPAddress subnetM(255, 255, 255, 252);
AsyncWebServer server(80);

/*---------------- Internet connection ----------------*/
/** Stored in a private  Connect.h file 
const byte lan_bssid[] = {0xFF, , , };
const char* lan_password = "";
const String API_KEY;
*/
const char* lan_ssid = "d-Link";
const IPAddress lan_ip(192, 168, 1, 200);
const IPAddress lan_gateway(192, 168, 1, 1);
const IPAddress lan_subnetM(255, 255, 255, 0);

const String lat = "42.826", lon = "13.691";
const String exlude = "current,daily,minutely,alerts";
const String queryString ="api.openweathermap.org/data/2.5/onecall?lat=" + 
                        lat + "&lon=" + lon + "&exclude="+ exlude +"&units=metric&lang=it&appid="+ API_KEY;


const char daysOfTheWeek[7][12] = {
    "Domenica",
    "Lunedi'",
    "Martedi'",
    "Mercoledi'",
    "Giovedi'",
    "Venerdi'",
    "Sabato"
};
DateTime currentTime;

const char heatCondition[6][15] = {"Good", "Caution", "High-Caution", "Danger"};
const char airCondition[6][11] = {"Healthy", "Acceptable", "Not-Good", "Bad", "Danger", "Extreme"}; 
String forecast = "Sun";


/*---------------- Display pinout ----------------*/
const uint8_t screen_pwm_channel = 0;
const uint8_t screen_led = 16;
const uint8_t screen_reset = 17;
const uint8_t screen_DC = 4;    //Data Command pin
TFT_eSPI display = TFT_eSPI();


/*----------------- Environment data  ----------------*/
float voltage, temp, humidity, pressure, heatIndex;;
int vPercent, vPercent_ext = NULL;

String heatIndexLevel ="";
String airQualityIndex = airCondition[0];

/**  External sensor data may not be 
 *   available --> initialised to NULL
 * **/
typedef struct data_struct {
  float voltage_ext = NULL;
  float temp_ext = NULL;
  float humidity_ext = NULL;
  float pressure_ext = NULL;
  float TVOC = NULL;    //BSEC_OUTPUT_BREATH_VOC_EQUIVALENT in ppm
  float IAQ = NULL;
  int accuracy = NULL;
  float CO = NULL;
  int rssi = NULL;
  int conn_time = NULL;
} data_struct;
data_struct sensorData;




void setup() {
    /** Disabling Radio when waking up */
    WiFi.disconnect(true, true);
    btStop(); delay(1);

    Serial.begin(115200);
    Wire.begin();

    adcAttachPin(batt_in);
    
    /*--------------------- Sensor initializing ----------------------*/
    if ( !rtc.begin() ) Serial.println("Couldn't find RTC");
    rtc.disable32K();   
    if( rtc.lostPower() )
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));


    bmp.reset();
    if (!bmp.begin(0x76))   Serial.println("Couldn't find BMP");
    delay(2);   /** Start-up time*/
    
    /** Weather/Climate-monitor  Calibration */
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED, 
                    Adafruit_BMP280::SAMPLING_X1, // temperature 
                    Adafruit_BMP280::SAMPLING_X1, // pressure
                    Adafruit_BMP280::FILTER_OFF); 
    

    if(! hdc.begin(0x40))     Serial.println("Couldn't find HDC");
    //hdc.drySensor();		/** Blocking => wasting time, disabled while not in production */


    /*-----------------------  Display ---------------------------------*/
    ledcSetup(screen_pwm_channel, 5000.0, 8);
    ledcAttachPin(screen_led, screen_pwm_channel);

	display.begin();
    display.setSwapBytes(true); /* Endianess */
	display.setRotation('3');
	display.setTextSize(1);
    

    /*---------------------------- Web server - Access Point -------------------------*/
    while( !WiFi.mode(WIFI_AP) )
        Serial.println("Wifi radio not ready");

    if(! WiFi.config(localIP, gateway, subnetM) )
      Serial.println("AP Failed to configure IP");

    WiFi.persistent(false);
    if(! WiFi.setTxPower(WIFI_POWER_19_5dBm) )
        Serial.println("Can't set Wifi Power mode");

    while(! WiFi.softAP(ap_ssid, ap_password, 1, 0, 2) )    //Channel 1 - 2412MHz
        Serial.println("Acccess Point not ready");

    //Serial.println("\nAccess Point IP: " + WiFi.softAPIP().toString() + " Bap_ssid: "+WiFi.macAddress());

    server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) {
        if (request->hasParam("volt") && request->hasParam("rssi") )    //no need to check all querystring parameters
        {
            sensorData.temp_ext = request->getParam("temp")->value().toFloat();
            sensorData.humidity_ext = request->getParam("hum")->value().toFloat();
            sensorData.pressure_ext = request->getParam("pres")->value().toFloat() / 100;
            sensorData.voltage_ext = request->getParam("volt")->value().toFloat();
            sensorData.TVOC = request->getParam("tvoc")->value().toFloat();
            sensorData.IAQ = request->getParam("iaq")->value().toFloat();
            sensorData.accuracy = request->getParam("accuracy")->value().toInt();
            sensorData.CO = request->getParam("co")->value().toFloat();
            sensorData.conn_time = request->getParam("time")->value().toInt();
            sensorData.rssi = request->getParam("rssi")->value().toInt();
            TIME_TO_NEXT_HTTP = request->getParam("next")->value().toInt();

            vPercent_ext = battPercentage(sensorData.voltage_ext);
            heatIndex = util.computeHeatIndex(sensorData.temp_ext, sensorData.humidity_ext, false);
            heatIndexLevel = getHeatCondition(heatIndex);
            call_miss = 0;

            request->send(200, "text/plain", (String)TIME_TO_NEXT_HTTP );
            Serial.println("Answered to a client\n");
        }
        else{ 
            Serial.println("Bad URL request");
            request->send(400, "text/plain", "Bad URL request");
        }
    });
    server.begin();
}


int battPercentage(float v) 
{
    const float battMin = 3.2F;
    const float battMax = 4.229F;  //My charging circuit
    if (v >= battMin)
        return (v - battMin) * 100 / (battMax - battMin);
    else return 0;
}


void readBattery() 
{
    const byte nReadings = 40;
    float voltage_reading = 0;

    /** Multiple readings to get a stabilised value */
    for (byte x = 0; x < nReadings; x++){
        voltage_reading += analogRead(batt_in);
        delay(5);
    }
    voltage = (voltage_reading / nReadings) * 3.3F / 4095 * 2.17F;   //Sperimental attenuation
    vPercent = battPercentage(voltage);
}


void ambientMeasurement() 
{
    bmp.takeForcedMeasurement();    /** Mandatory while in forced mode */
    temp = bmp.readTemperature() - 5.573F;     /** See Calibration.xls */
    bmp.takeForcedMeasurement();
    pressure = bmp.readPressure() / 100;  //hPa

    humidity = hdc.readHumidity();
}


String getHeatCondition(float heatIndex)
{
    if(heatIndex < 26)   heatIndexLevel = heatCondition[0];
    else if (heatIndex >= 26 && heatIndex <= 32)
        heatIndexLevel = heatCondition[1];
    else if(heatIndex > 32 && heatIndex <= 41)
        heatIndexLevel = heatCondition[2];
    else if(heatIndex > 41 && heatIndex <= 54)
        heatIndexLevel = heatCondition[3];
    else    heatIndexLevel = "";

    return heatIndexLevel;
}


void getForecast()
{
    WiFi.disconnect(true, true);
    while( !WiFi.mode(WIFI_STA) )
		Serial.println("Wifi STA not ready");

    if ( !WiFi.config(lan_ip, lan_gateway, lan_subnetM) )
		Serial.println("STA Failed to configure IP");

    WiFi.begin( lan_ssid, lan_password);

    unsigned long temp = millis();
    short retries = 0;
    while(WiFi.status() != WL_CONNECTED && retries < 3000){
        retries++;
        delay(1);
    }
    if(retries == 3000){
        Serial.println("No internet connection");
        return;
    }

	Serial.println("\n" + (String)(millis() - temp) + " millis for connection to router \t Retries: " + (String) retries );

    Serial.println("Router BSSID: "+ WiFi.BSSIDstr() + "  RSSI: "+ (String) WiFi.RSSI() +
                   "\nMy IP: " + WiFi.localIP().toString() );

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("I'm going to call: " + queryString);

        HTTPClient http;
        http.useHTTP10(true);
        http.begin( queryString.c_str() );

        int responseCode = http.GET();
        if (responseCode > 0 ){
            String payload = http.getString();
            parseJsonAnswer(payload);
        }
        else
            Serial.println("Error code: " + (String)responseCode + "\n");

        http.end();
    }
    else Serial.println("Not connected to a network");

    
    /** Back to HTTP listener */
    WiFi.mode(WIFI_AP);
    while( WiFi.disconnect(false, true) );
    WiFi.mode(WIFI_AP);
    while(! WiFi.softAP(ap_ssid, ap_password, 1, 0, 2) )    //Channel 1 - 2412MHz
        Serial.println("Acccess Point not ready");
}


void parseJsonAnswer(String payload){
    Serial.println("\nn I'm going to parse: \n" + payload + "\n\n");
    StaticJsonDocument<600> doc;

    DeserializationError error = deserializeJson(doc, payload.c_str() );
    // Test if parsing succeeds.
    if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
    }
    Serial.println("I parsed");
    //forecast = doc["hourly"][2]["weather"][0]["main"];
    //Serial.println("Forecast: "+ (String)forecast);
}


void printToSerial() {

    Serial.println((String) currentTime.day() + "/" + (String) currentTime.month() + "/" + (String) currentTime.year() +
        "  " + (String) daysOfTheWeek[currentTime.dayOfTheWeek()] + "  " + (String) currentTime.hour() +
        ":" + (String) currentTime.minute() + ":" + (String) currentTime.second());
    /*--------------------------------------------------------------------------------*/
    Serial.print("Voltage: ");
    Serial.print(voltage, 3);
    Serial.println("V\t\t" + (String) vPercent + "%\n");
    /*--------------------------------------------------------------------------------*/
    Serial.println("Temperature BMP: " + (String) temp + " °C");
    Serial.println("Temperature HDC: " + (String) hdc.readTemperature() + " °C");
    Serial.println("Temperature RTC: " + (String) rtc.getTemperature() + " °C");
    Serial.println("Temperature CPU: "+ (String)temperatureRead() +"\n");
    Serial.println("Pressure: " + (String) pressure + " hPa");
    Serial.println("Humidity: " + (String) humidity + " %RH");
    Serial.println();
    /*--------------------------------------------------------------------------------*/
    Serial.println("EXT_Voltage: " + String( sensorData.voltage_ext,3 ) + " V\t" + (String) vPercent_ext + "%");
    Serial.println("EXT_Temperature: " + (String) sensorData.temp_ext + " °C");
    Serial.println("EXT_Pressure: " + (String) sensorData.pressure_ext + " hPa");
    Serial.println("EXT_Humidity: " + (String) sensorData.humidity_ext + " %RH");
    Serial.println("Heat Index: " + (String) heatIndex + "\t" + heatIndexLevel);
    Serial.println("\nAir tVOC: " + (String) sensorData.TVOC + " ppm\n");
    Serial.println("Connection Time: " + (String) sensorData.conn_time + " millis - RSSI: " + (String)sensorData.rssi);
    Serial.println("Next sending in: " + (String) TIME_TO_NEXT_HTTP + " s.");
    Serial.println("------------------------------------\n");
}


void displayToScreen(){ 
    /*---------------------------------------- BRIGHTNESS -------------------------------*/
    if(currentTime.hour() >= 21 && currentTime.hour() < 23)     
         ledcWrite(screen_pwm_channel, 20);
    else if(currentTime.hour() >= 23 || (currentTime.hour() >= 0 && currentTime.hour() <= 7) )
         ledcWrite(screen_pwm_channel, 2);
    else ledcWrite(screen_pwm_channel, 220);    /* 8:00-20:00 */

    display.fillScreen(TFT_BLACK);
	display.setTextColor(TFT_WHITE);


	//DATE
	display.setFreeFont(&URW_Gothic_L_Book_41);
	display.setCursor(0, 32+3);		/* 32 is number's font height */
	(currentTime.day()<10)   ? display.print( "0" + (String)currentTime.day() + "/" )   :  display.print( (String)currentTime.day() + "/" );
    (currentTime.month()<10) ? display.print( "0" + (String)currentTime.month() ) :  display.print( (String)currentTime.month() );
    
	//DoWEEK
    display.setFreeFont(&URW_Gothic_L_Book_28);
    display.print("  "+ (String) daysOfTheWeek[currentTime.dayOfTheWeek()]);

	//Clock position
	display.setFreeFont(&URW_Gothic_L_Book_77);
	int text_w = display.textWidth("00:00");
	display.setCursor( display.width() - text_w, 57+2 );	/* 57 is number max height for this font and size */
	//CLOCK
	(currentTime.hour()<10)  ?  display.print( "0" + (String)currentTime.hour() )  :  display.print( (String)currentTime.hour() );
    display.print(":");
    (currentTime.minute()<10)  ?  display.print( "0" + (String)currentTime.minute() )  :  display.print( (String)currentTime.minute() );


	//WEATHER FORECAST
	const byte icon_w = 80, icon_h = 80;
    display.pushImage(10, display.getCursorY()+4, icon_w, icon_h, Sun);


	/*--------------------------------- SENSOR DATA ------------------------------------------*/
    //Right alignment
	display.setFreeFont(&URW_Gothic_L_Book_41);
    const short right_offset = display.textWidth("00.0%rH");    /* The widest string */
    
    //out - Pressure
    display.setFreeFont(&URW_Gothic_L_Book_28);
    display.setCursor( icon_w + 38, display.getCursorY() + 41);
	display.setTextColor(0xAE3F);
    display.print( (String)(int)sensorData.pressure_ext + " hPa");

    //SIGNAL STRENGTH (ext-sensor)
    display.setCursor( display.width() - display.textWidth( (String)sensorData.rssi +"dBi"), display.getCursorY() );
    display.setTextColor(0x94B2);
    display.println((String)sensorData.rssi + "dBi");

    //HEAT INDEX
    display.setCursor(icon_w + 38, display.getCursorY() + 2);
    display.setTextColor(0xE6B1);
    display.print("Feels like: ");
    display.print(heatIndex, 1);
    display.print(" 'C" );
    display.setCursor(display.width() - display.textWidth(heatIndexLevel), display.getCursorY() );
    display.println(heatIndexLevel);

    //AIR QUALITY
    display.setTextColor(0x94B2);
    display.setCursor(1, display.getCursorY() + 8);
    display.print( (String)sensorData.TVOC + " ppM " /*+ (String)(int)sensorData.IAQ */+ " CO2: " + (String)(int)sensorData.CO );
    //display.setCursor(display.width() - display.textWidth(airQualityIndex), display.getCursorY() );
    display.setCursor(display.width() - display.textWidth("Acc. 0"), display.getCursorY() );
    display.println("Acc. " + (String) sensorData.accuracy);

    //out - Temp
	display.setCursor(1, display.getCursorY() + 22);    //light Y offset
	display.setTextColor(0xF9E7);
	display.setFreeFont(&URW_Gothic_L_Book_41);
	display.print(sensorData.temp_ext, 1);
	display.setFreeFont(&URW_Gothic_L_Book_28);
	display.print(" 'C");
    //in
    display.setFreeFont(&URW_Gothic_L_Book_41);
    display.setCursor(display.width()-right_offset, display.getCursorY());
    display.print(temp,1);
    display.setFreeFont(&URW_Gothic_L_Book_28);
	display.println(" 'C");

    //out - Humidity
	display.setCursor(1, display.getCursorY() + 5);
	display.setTextColor(0x3B7F);
	display.setFreeFont(&URW_Gothic_L_Book_41);
	display.print(sensorData.humidity_ext, 1);
	display.setFreeFont(&URW_Gothic_L_Book_28);
	display.print(" %rH");
    //in
    display.setFreeFont(&URW_Gothic_L_Book_41);
    display.setCursor(display.width()-right_offset, display.getCursorY());
    display.print(humidity,1);
    display.setFreeFont(&URW_Gothic_L_Book_28);
	display.println(" %rH");

    //out - Voltage
	display.setCursor(1, display.getCursorY() + 5);
	display.setTextColor(0x4CA8);
	display.setFreeFont(&URW_Gothic_L_Book_41);
	display.print((String)vPercent_ext + "%   ");
    display.setFreeFont(&URW_Gothic_L_Book_28);
    display.print(sensorData.voltage_ext, 3);
	display.print(" V");
    //in
    display.setCursor(display.width()-display.textWidth("8.888 V") - 3, display.getCursorY());
    display.print(voltage, 3);
	display.println(" V");

    display.drawFastVLine( display.width() / 2 +20, (display.height() / 1.55F), (display.height() / 1.55F), 0x94B2);
    
    display.endWrite();
}


void clearData(){
    memset(&sensorData, 0, sizeof(sensorData) ); /** Clear old ext_sensor data */
    vPercent_ext = 0;   
    heatIndex = 0;
    call_miss = 0;    /** No need to always clear the struct */
    Serial.println("No ext_sensor found");
}



void loop() {
    call_miss++;

    readBattery();
    ambientMeasurement();
    currentTime = rtc.now();
    
    //if(currentTime.minute() == 0 || currentTime.minute() == 30 )
        //getForecast();

    /*if(currentTime.minute() % 3 == 0 )
        getForecast();
    */

    /** Ext_sensor not available or not sending data */
    if( call_miss >= 2*(TIME_TO_NEXT_HTTP/TIME_TO_SLEEP) )   
        clearData();
    //else data are overwritten
    
    
    //printToSerial();
    displayToScreen();

    delay(TIME_TO_SLEEP*1000UL);
}