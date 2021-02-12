#include <bsec.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

#define ESP8266

unsigned short TIME_TO_NEXT_SENDING = 60;

const char* ssid = "ESP-WeatherStation";  
const byte bssid[] = {0x30,0xAE,0xA4,0x98,0x83,0xB9}; /** In AP call WiFi.macAddress() -- "30:AE:A4:98:83:B9" */
const char* password = "esp32station"; 
unsigned long conn_time = NULL;

const IPAddress localIP(192, 168, 4, 2);
const IPAddress gateway(192, 168, 4, 1);
const IPAddress subnet(255, 255, 255, 252);

float voltage_ext;

bsec_virtual_sensor_t sensorList[10] = {
	BSEC_OUTPUT_IAQ,
	BSEC_OUTPUT_STATIC_IAQ,
	BSEC_OUTPUT_CO2_EQUIVALENT,
	BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,	//tVOC
	BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
	BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
};
Bsec bme=Bsec();


unsigned long start;
void setup() {

	start = millis();
	/** Disabling WiFi radio as soon as waking up */
	WiFi.mode(WIFI_OFF);  
	WiFi.forceSleepBegin();

	Serial.begin(115200);   //Will be removed for 'production' to save battery
	Wire.begin();
	pinMode(A0, INPUT);  /** Battery voltage divider input*/


	/*------------------------------ BME680 Sensor + Air quality API --------------------------*/
	bme.begin(BME680_I2C_ADDR_SECONDARY, Wire);
	//bme.setTemperatureOffset(-0.541F);
	/** Sets the desired sensors and the sample rates */
	bme.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
}


void checkSensorStatus(void)
{
	if (bme.status != BSEC_OK)
		if (bme.status < BSEC_OK) 
			Serial.println("BSEC error code : " + String(bme.status));
		else 
			Serial.println("BSEC warning code : " + String(bme.status));

	if (bme.bme680Status != BME680_OK) 
		if (bme.bme680Status < BME680_OK) 
			Serial.println("BME680 error code : " + String(bme.bme680Status));
		else
			Serial.println("BME680 warning code : " + String(bme.bme680Status));
}


void forceWifiOn(){
	wifi_fpm_do_wakeup();
   	wifi_fpm_close();
   	wifi_set_opmode(STATION_MODE);
   	wifi_station_connect();
}


void sendData() 
{
	forceWifiOn();

	if (!WiFi.forceSleepWake())
		Serial.println("Can't wake wifi");

	WiFi.persistent(false); /** Improves connection of about 400 millis */

	if ( !WiFi.mode(WIFI_STA) )
		Serial.println("Wifi STA not ready");

	WiFi.setPhyMode(WIFI_PHY_MODE_11N);
	WiFi.setOutputPower(20.5);

	if ( !WiFi.config(localIP, gateway, subnet) )
		Serial.println("STA Failed to configure IP");

	unsigned long temp = millis();
	unsigned short retry = 0;
	WiFi.begin(ssid, password, 1, bssid, true);		//Channel 1 - 2412MHz
	while (WiFi.status() != WL_CONNECTED && retry < 2000) {
		delay(1);
		retry++;
	}
	conn_time = millis() - temp;

	Serial.println("\nConn.retries in while loop: " + (String) retry);
	Serial.println((String)conn_time + " millis for WIFI connection");

	if (retry == 2000)
		Serial.println("Can't connect to AP. Too much retries");

	if (WiFi.status() == WL_CONNECTED) {
		//Serial.println("My Ip: "+ WiFi.localIP().toString());
		Serial.println("RSSI: "+ (String) WiFi.RSSI() );

		HTTPClient http;
		http.begin("http://192.168.4.1/update?temp=" + 
				(String)bme.temperature + "&hum=" + (String)bme.humidity + "&pres=" + (String)bme.pressure + 
				"&tvoc=" + (String)bme.breathVocEquivalent + "&iaq=" + (String)bme.iaq + "&co=" + (String)bme.co2Equivalent + 
				"&volt=" + (String)voltage_ext + "&time=" + (String)conn_time + "&rssi=" + (String)WiFi.RSSI() 
		);
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

	Serial.println("\nIAQ: " + (String)bme.iaq + "\nAccuracy: " + (String)bme.iaqAccuracy 
				+ "\nStatic-IAQ: " + (String)bme.staticIaq +"\nCO2: " + (String)bme.co2Equivalent
				+ "\ntVOC: " + (String)bme.breathVocEquivalent );

	Serial.println("\nTemp: " + (String)bme.temperature + "\nHumidity: " + (String)bme.humidity
				 + "\nPressure: " + (String)(bme.pressure/100) );
}


void loop() 
{
	voltage_ext = analogRead(A0) * 3.3F / 1023.0F * 4.6843F;

	if (! bme.run()) {
		Serial.println("BME measurements not available");
		checkSensorStatus();
	}

	printToSerial();
	sendData();

	Serial.println("I took: "+ (String)(millis()-start) + " millis. to complete a cycle");
	Serial.println("\n-----------------------------------------\n");

	ESP.deepSleep(TIME_TO_NEXT_SENDING *1E6, WAKE_RF_DISABLED);
	delay(1); /*Needed for proper sleeping*/
}
