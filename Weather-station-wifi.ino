#include <TFT_eSPI.h>
#include <User_Setup_Select.h>
#include <User_Setup.h>
#include <Custom_font.h>

#include <Wire.h>

#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_HDC1000.h>
#include <DHT.h> //heat-index

#include <WiFi.h>
#include <HTTPClient.h>

/*-------------------- Sensori ------------------*/
RTC_DS3231 rtc;
Adafruit_BMP280 bmp;
Adafruit_HDC1000 hdc;
DHT util = DHT(NULL,DHT22); //just for heat-index

static const byte batt_in = 34;

/*---------------------------- Variabili globali ------------------------*/
static const char * ssid = "ESP-sensor";
static const char * password = "esp8266sensor";

static const byte screen_pwm_channel = 0;
static const byte screen_led = 16;
static const byte screen_reset = 17;
static const byte screen_DC = 4;    //Data Command pin
TFT_eSPI display = TFT_eSPI();


static const char daysOfTheWeek[7][12] = {
    "Domenica",
    "Lunedi'",
    "Martedi'",
    "Mercoledi'",
    "Giovedi'",
    "Venerdi'",
    "Sabato"
};
DateTime currentTime;

static const char heatCondition[6][15] = {"Good", "Caution", "High-Caution", "Danger", "Extreme-Danger"};
static const char airQualityLevels[6][11] = {"Healthy", "Acceptable", "Not-Good", "Bad", "Danger", "Extreme"}; 


/*----------------------- Environment data  -------------------------------*/
/**  External sensor data may not be available --> initialised to NULL**/
float voltage;
int vPercent;
float voltage_ext = NULL;
int vPercent_ext = NULL;

float temp;
float temp_ext = NULL;
float humidity;
float humidity_ext = NULL;
float pressure;
float pressure_ext = NULL;

float heatIndex;
String heatIndexLevel ="NULL";
float airTVOC = NULL;
String airQualityIndex = airQualityLevels[0];

/*-----------------------------------------------------------------------*/

void setup() {
	WiFi.mode(WIFI_OFF); delay(1);
	btStop(); delay(1);

    Serial.begin(115200);
    Wire.begin();

    adcAttachPin(batt_in);
    
    /*--------------------- Sensor initializing ----------------------*/
    if ( !rtc.begin() ) Serial.println("Couldn't find RTC");
    rtc.disable32K();   
    if( rtc.lostPower() )
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));


    if (!bmp.begin(0x76))   Serial.println("Couldn't find BMP");
    /** Weather/Climate-monitor  Calibration */
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED, 
                    Adafruit_BMP280::SAMPLING_X1, // temperature 
                    Adafruit_BMP280::SAMPLING_X1, // pressure
                    Adafruit_BMP280::FILTER_OFF ); 
    

    if(! hdc.begin(0x40))     Serial.println("Couldn't find HDC");
    //hdc.drySensor();		/** Blocking => wasting time, disabled while not in production */

    /*-----------------------  Display ---------------------------------*/
    
    ledcSetup(screen_pwm_channel, 5000, 8);
    ledcAttachPin(screen_led, screen_pwm_channel);
   
	display.begin();
    display.setSwapBytes(true); /* Endianess */
	display.setRotation(3);
	display.setTextSize(1);
}


int battPercentage(float v) {
    /** Map seems to not work   vPercent = map(voltage, 3.20 , 4.20, 0, 100);
     * Minimum voltage 3.2V (as 0%)
     * Maximum voltage 4.2V (100%)
     */
    const float battMin = 3.2;
    const float battMax = 4.2;  
    if (v >= battMin)
        return (v - battMin) * 100 / (battMax - battMin);
    else return 0;
}


void readBattery() {

    const byte nReadings = 64;
    float voltage_reading = 0;

    /** Multiple readings to get a stabilised value */
    for (byte x = 0; x < nReadings; x++)
        voltage_reading += analogRead(batt_in);

    /** 5.1 maximum voltage to measure (USB)
     * r1 (to device) 1Mohm --- r2 (to gnd) 1Mohm
     * In case of not equal resistors the read value 
     * is scaled down by a factor of  r2/(r1+r2)
     * so multiply per 1/attenuation (now 2)
     */
    voltage = (voltage_reading / nReadings) * 3.3 / 4095 * 2.275;
    vPercent = battPercentage(voltage);
}


void getTime() {
    currentTime = rtc.now();

    Serial.println((String) currentTime.day() + "/" + (String) currentTime.month() + "/" + (String) currentTime.year() +
        "  " + (String) daysOfTheWeek[currentTime.dayOfTheWeek()] + "  " + (String) currentTime.hour() +
        ":" + (String) currentTime.minute() + ":" + (String) currentTime.second());
}


void ambientMeasurement() {
    bmp.MODE_FORCED;
    temp = bmp.readTemperature();
    pressure = bmp.readPressure() / 100;  //hPa
    bmp.MODE_SLEEP;

    humidity = hdc.readHumidity();
    heatIndex = util.computeHeatIndex(temp, humidity, false);

    if(heatIndex < 26)      heatIndexLevel = heatCondition[0];
    else if (heatIndex >= 26 && heatIndex <= 32)
        heatIndexLevel = heatCondition[1];
    else if(heatIndex > 32 && heatIndex <= 41)
        heatIndexLevel = heatCondition[2];
    else if(heatIndex > 41 && heatIndex <= 54)
        heatIndexLevel = heatCondition[3];
    else    heatIndexLevel = heatCondition[3];
    
}


String httpGETrequest(const char * serverName) {
    HTTPClient http;
    http.begin(serverName);

    if (http.GET() > 0) //contains the ResponseCode
        return http.getString();    //contains the Payload
    else {
        Serial.println("Bad request; code: " + http.GET());
        return "";
    }
    http.end(); // Free resources
}


void getExtSensor() {
    /*--------------------- Initialization -----------------*/
    WiFi.begin(ssid, password);
    Serial.print("Connecting: ");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.print("\n External sensor WIFI connected: ");
    Serial.println(WiFi.localIP());
    /*------------------------------------------------------*/

    /** Can be fixed to 192.168.4.1 */
    static const String gateway = WiFi.gatewayIP().toString();
    Serial.println("Gateway: " + gateway);

    voltage_ext = httpGETrequest(("http://" + gateway + "/volt").c_str()).toFloat();
    vPercent_ext = battPercentage(voltage_ext);
    temp_ext = httpGETrequest(("http://" + gateway + "/temp").c_str()).toFloat();
    humidity_ext = httpGETrequest(("http://" + gateway + "/hum").c_str()).toFloat();
    pressure_ext = httpGETrequest(("http://" + gateway + "/press").c_str()).toFloat();
    airTVOC = httpGETrequest(("http://" + gateway + "/air").c_str()).toFloat();

    /**Doesn't repeat the connection attempt until the ext_sensor
     * will be ready to answer the requests. 
     * It goes in deepSleep mode after a cycle of requests (to implement)
     * This is a syncronization constant => TIME_TO_SLEEP in the "ext_sensor.ino"
     **/
}


void printToSerial() {
    /*--------------------------------------------------------------------------------*/
    Serial.print("Voltage: ");
    Serial.print(voltage, 3);
    Serial.println("V\t\t" + (String) vPercent + "%");
    /*--------------------------------------------------------------------------------*/
    Serial.println("Temperature BMP: " + (String) temp + " °C");
    Serial.println("Temperature HDC: " + (String) hdc.readTemperature() + " °C");
    Serial.println("Temperature RTC: " + (String) rtc.getTemperature() + " °C");
    Serial.println("Pressure: " + (String) pressure + " hPa");
    Serial.println("Humidity: " + (String) humidity + " %RH");
    Serial.println("Heat Index: " + (String) heatIndex + "\t" + heatIndexLevel);
    Serial.println();
    /*--------------------------------------------------------------------------------*/
    Serial.println("EXT_Voltage: " + (String) voltage_ext + " V\t" + (String) vPercent_ext + "%");
    Serial.println("EXT_Temperature: " + (String) temp_ext + " °C");
    Serial.println("EXT_Pressure: " + (String) pressure_ext + " hPa");
    Serial.println("EXT_Humidity: " + (String) humidity_ext + " %RH");
    Serial.println("Air Quality: " + (String) airTVOC + " ppm");
    Serial.println("-----------------------------------------");
}


void displayToScreen(){

    /*---------------------------------------- BRIGHTNESS -------------------------------*/
    if(currentTime.hour() >= 21 && currentTime.hour() < 23)     
        ledcWrite(screen_pwm_channel, 11);
    else if(currentTime.hour() >= 23 || (currentTime.hour() >= 0 && currentTime.hour() <= 7) )
        ledcWrite(screen_pwm_channel, 3);
    else if(currentTime.hour() >= 8 && currentTime.hour() <= 20)
        ledcWrite(screen_pwm_channel, 250);

    display.fillScreen(TFT_BLACK);
	display.setTextColor(TFT_WHITE);

	//DATE
	display.setFreeFont(&URW_Gothic_L_Book_41);
	display.setCursor(1, 32+3);		/* 32 is number's font height */
	(currentTime.day()<10)  ?  display.print( "0" + (String)currentTime.day() + " / " )  :  display.print( (String)currentTime.day() + " / " );
    (currentTime.month()<10)  ?  display.print( "0"+(String)currentTime.month() )       :  display.print( (String)currentTime.month() );
    
	//WEEK
    display.setFreeFont(&URW_Gothic_L_Book_28);
    display.print("  "+ (String) daysOfTheWeek[currentTime.dayOfTheWeek()]);

	//Clock position
	display.setFreeFont(&URW_Gothic_L_Book_75);
	int text_w = display.textWidth("00:00");
	display.setCursor( display.width() - text_w -1, 57+2 );	/* 57 is number max height for this font and size */
	//CLOCK
	(currentTime.hour()<10)  ?  display.print( "0" + (String)currentTime.hour() )  :  display.print( (String)currentTime.hour() );
    display.print(":");
    (currentTime.minute()<10)  ?  display.print( "0" + (String)currentTime.minute() )  :  display.print( (String)currentTime.minute() );


	//WEATHER FORECAST
	const byte forecast_w = 120, forecast_h = 75;	/* PLACEHOLDER -> removed when inserting icons */
	display.setCursor(1, display.getCursorY()+4);	/* Upper font size + 4 */ 
	display.drawRoundRect(1, display.getCursorY(), forecast_w, forecast_h, 5, TFT_LIGHTGREY);

	/*--------------------------------- SENSOR DATA ------------------------------------------*/
    //Right alignment
	display.setFreeFont(&URW_Gothic_L_Book_41);
    const short right_offset = display.textWidth("00.0%rH");
    
    //OUT - pressure
    display.setFreeFont(&URW_Gothic_L_Book_28);
    display.setCursor(display.getCursorX() + forecast_w + 10, display.getCursorY() + 41);
	display.setTextColor(0x0451);
    display.println((int) pressure_ext + " hPa");

    //AIR QUALITY
    display.setCursor(forecast_w + 10, display.getCursorY() + 2);
    display.setTextColor(TFT_LIGHTGREY);
    display.println( (String)(int)airTVOC + " ppm tVOC - "+ airQualityIndex);

    //HEAT INDEX
    display.setTextColor(TFT_YELLOW);
    display.print("H.I:  " + (String)(int)heatIndex + "'C -> " + (String) heatIndexLevel);

    //IN tag
    display.setFreeFont(&URW_Gothic_L_Book_28);
    display.setTextColor(TFT_WHITE);
    display.setCursor(display.width()-right_offset, display.getCursorY());
    display.println("- IN -");

    //OUT - temp
	display.setCursor(1, display.getCursorY() + 25);    //light Y offset
	display.setTextColor(0xF9E7);
	display.setFreeFont(&URW_Gothic_L_Book_41);
	display.print(temp_ext, 1);
	display.setFreeFont(&URW_Gothic_L_Book_28);
	display.print(" 'C");
    //IN
    display.setFreeFont(&URW_Gothic_L_Book_41);
    display.setCursor(display.width()-right_offset, display.getCursorY());
    display.print(temp,1);
    display.setFreeFont(&URW_Gothic_L_Book_28);
	display.println(" 'C");

    //OUT - humidity
	display.setCursor(1, display.getCursorY() + 5);
	display.setTextColor(0x43B9);
	display.setFreeFont(&URW_Gothic_L_Book_41);
	display.print(humidity_ext, 1);
	display.setFreeFont(&URW_Gothic_L_Book_28);
	display.print(" %rH");
    //IN
    display.setFreeFont(&URW_Gothic_L_Book_41);
    display.setCursor(display.width()-right_offset, display.getCursorY());
    display.print(humidity,1);
    display.setFreeFont(&URW_Gothic_L_Book_28);
	display.println(" %rH");

    //OUT - voltage
	display.setCursor(1, display.getCursorY() + 5);
	display.setTextColor(0x4CA8);
	display.setFreeFont(&URW_Gothic_L_Book_41);
	display.print((String)vPercent_ext + "%   " + (String)voltage_ext);
	display.setFreeFont(&URW_Gothic_L_Book_28);
	display.print("V");
    //IN
    display.setCursor(display.width()-display.textWidth("0.00V"), display.getCursorY());
    display.print((String)voltage);
	display.println("V");
}


void loop() {

    readBattery();
    getTime();
    ambientMeasurement();
    //getExtSensor();
    //printToSerial();
    displayToScreen();

    delay(3000);
}