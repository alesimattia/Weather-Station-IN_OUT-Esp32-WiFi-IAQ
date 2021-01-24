//#include "bsec.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

#define ESP8266

unsigned int TIME_TO_NEXT_SENDING = 30;

const char* ssid = "ESP-WeatherStation";  
const byte bssid[] = {0x30,0xAE,0xA4,0x98,0x83,0xB9}; /** In AP call WiFi.macAddress() -- "30:AE:A4:98:83:B9" */
const char* password = "esp32station"; 

IPAddress localIP(192, 168, 4, 2);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 252);

Adafruit_BME680 bme;

float voltage_ext;
float temp_ext;
float humidity_ext;
float pressure_ext;
float airTVOC = 0;

/*bsec_virtual_sensor_t sensorList[10] = {
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
Bsec util;*/


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
	temp_ext = bme.temperature;
	pressure_ext= bme.pressure / 100.0;
	humidity_ext = bme.humidity;
	airTVOC = bme.gas_resistance / 1000.0;

	voltage_ext = analogRead(A0) * 3.3F / 1024.0F * 4.711F;
	
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


void sendData() {

	if (!WiFi.forceSleepWake())
		Serial.println("Can't wake wifi");

	WiFi.persistent(false); /** Improves connection of about 400 millis */

	if (!WiFi.mode(WIFI_STA))
		Serial.println("Wifi STA not ready");

	WiFi.setPhyMode(WIFI_PHY_MODE_11N);
	WiFi.setOutputPower(20.5);

	if (!WiFi.config(localIP, gateway, subnet))
		Serial.println("STA Failed to configure IP");

	unsigned long temp = millis();
	short retry = 0;
	WiFi.begin(ssid, password, 12, bssid, true);
	while (WiFi.status() != WL_CONNECTED && retry < 2000) {
		delay(1);
		retry++;
	}
	Serial.println("Retries: " + (String) retry);
	Serial.println((String)(millis() - temp) + " millis for WIFI connection");

	if (retry == 2000)
		Serial.println("Can't connect to AP. Too much retries");
	else Serial.println("Connected to an AP");

	if (WiFi.status() == WL_CONNECTED) {
		//Serial.println("My Ip: "+ WiFi.localIP().toString());

		HTTPClient http;
		http.begin(String("http://192.168.4.1/update?temp=" + (String) temp_ext + "&hum=" + (String) humidity_ext +
			"&pres=" + (String) pressure_ext + "&volt=" + (String) voltage_ext));
		/*if ( http.GET() == 200) {
		  Serial.println("Server told me to sleep for " + http.getString() + " seconds");
		  TIME_TO_NEXT_SENDING = http.getString().toInt();
		}
		else   Serial.println("Server error");*/

		http.GET();
		http.end();
	}
	WiFi.disconnect(true);
}


void printToSerial() {
	Serial.print("\nVoltage: ");
	Serial.print(voltage_ext, 4);
	Serial.println("V");
	/*--------------------------------------------------------------------------------*/
	Serial.println("Temperature: " + (String) temp_ext + " °C");
	Serial.println("Pressure: " + (String) pressure_ext + " hPa");
	Serial.println("Humidity: " + (String) humidity_ext + " %RH");
	Serial.println("Air Index: " + (String) airTVOC + " KOhms");
}


void loop() {
	ambientMeasurement();
	//computeAirQuality();
	printToSerial();
	sendData();

	Serial.println("I took: "+ (String)(millis()-start) + " millis. to complete a cycle");
	Serial.println("\n-----------------------------------------\n");

	ESP.deepSleepInstant(TIME_TO_NEXT_SENDING *1E6, WAKE_RF_DISABLED);
	delay(1); /*Needed for proper sleeping*/
}
