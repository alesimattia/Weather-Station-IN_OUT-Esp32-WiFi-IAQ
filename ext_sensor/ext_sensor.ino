#include <bsec.h>
#include <EEPROM.h> //to store BSEC calibration state
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

#define ESP8266

const uint32_t TIME_TO_NEXT_SENDING = 60U * 1000000U;

const char* ssid = "ESP-WeatherStation";
const char* password = "esp32station";
const byte bssid[] = {0x30,0xAE,0xA4,0x98,0x83,0xB9};   /** In AP call WiFi.macAddress() -- "30:AE:A4:98:83:B9" */
unsigned int conn_time = NULL;

const IPAddress localIP(192, 168, 4, 2), 
				gateway(192, 168, 4, 1), 
				subnet(255, 255, 255, 252);

float voltage_ext;

Bsec bme=Bsec();

bsec_virtual_sensor_t sensor_list[14] = {
	BSEC_OUTPUT_IAQ,
	BSEC_OUTPUT_STATIC_IAQ,
	BSEC_OUTPUT_CO2_EQUIVALENT,
	BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,	//tVOC
	BSEC_OUTPUT_RAW_TEMPERATURE,
	BSEC_OUTPUT_RAW_PRESSURE,
	BSEC_OUTPUT_RAW_HUMIDITY,
	BSEC_OUTPUT_RAW_GAS,
	BSEC_OUTPUT_RUN_IN_STATUS,
	BSEC_OUTPUT_STABILIZATION_STATUS,
	BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
	BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
	BSEC_OUTPUT_COMPENSATED_GAS,
	BSEC_OUTPUT_GAS_PERCENTAGE
};

uint8_t sensor_state[BSEC_MAX_STATE_BLOB_SIZE] = {0};

const uint8_t bsec_config_iaq[] = {
	#include "config/generic_33v_300s_4d/bsec_iaq.txt"
};



unsigned long start;
void setup() 
{
	start = millis();
	WiFi.forceSleepBegin();	 /** Disabling WiFi radio as soon as waking up */

	Serial.begin(115200);
	Wire.begin();
	EEPROM.begin( BSEC_MAX_STATE_BLOB_SIZE + 1 );

	pinMode(A0, INPUT);  /** Battery voltage divider input*/


	/*------------------- BME680 Sensor + Air quality API ---------------------*/
	bme.begin(BME680_I2C_ADDR_SECONDARY, Wire); 
	bme.setConfig(bsec_config_iaq);
	
	if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE) {	//There's a saved state
		for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
			sensor_state[i] = EEPROM.read(i + 1);	/** Skip first control-sector */
		bme.setState(sensor_state);

		Serial.print("\nI read this state: ");
			printState(sensor_state);
	}
	else {	// Erase the EEPROM with zeroes
		Serial.println("Not a valid state -> Erasing EEPROM");
		for (uint16_t i = 0; i < (BSEC_MAX_STATE_BLOB_SIZE + 1); i++)
			EEPROM.write(i, 0);
		EEPROM.commit();
	}

	bme.updateSubscription(sensor_list, sizeof(sensor_list)/sizeof(sensor_list[0]), BSEC_SAMPLE_RATE_LP);

	if (!checkSensor()){ 
		Serial.println("\nFailed to init BME680 !!");   
		return; 
	}
}

bool checkSensor() {
	if (bme.status < BSEC_OK) {
		Serial.println("BSEC error, status "+ (String) bme.status);
		return false;
	} else if (bme.status > BSEC_OK)
		Serial.println("BSEC warning, status "+ (String) bme.status);

	if (bme.bme680Status < BME680_OK) {
		Serial.println("Sensor error, bme680_status "+ (String) bme.bme680Status);
		return false;
	} else if (bme.bme680Status > BME680_OK)
		Serial.println("Sensor warning, status "+ (String) bme.bme680Status);

	return true;
}


void sendData() 
{
	if ( !WiFi.forceSleepWake() )
		Serial.println("Can't wake wifi");

	WiFi.persistent(false);   /** Improves connection of about 400 millis */

	if ( !WiFi.mode(WIFI_STA) )
		Serial.println("Wifi STA not ready");

	WiFi.setPhyMode(WIFI_PHY_MODE_11N);
	WiFi.setOutputPower(19);

	if ( !WiFi.config(localIP, gateway, subnet) )
		Serial.println("STA Failed to configure IP");

	unsigned short retry = 0;
	unsigned long partial = millis();
	WiFi.begin(ssid, password, 1, bssid, true);		//Channel 1 - 2412MHz
	while (WiFi.status() != WL_CONNECTED && retry < 2000) {
		retry++;
		delay(1);
	}
	conn_time = millis() - partial;
	Serial.println( "\n"+(String)(millis() - partial) + " ms. for WIFI connection. " + (String)retry + " retries");

	if (retry == 2000)
		Serial.println("Can't connect to AP. Too much retries");

	if (WiFi.status() == WL_CONNECTED) {
		//Serial.println("My Ip: "+ WiFi.localIP().toString() + "RSSI: "+ (String) WiFi.RSSI());

		HTTPClient http;
		http.begin("http://192.168.4.1/update?temp=" + 
				(String)bme.temperature + "&hum=" + (String)bme.humidity + "&pres=" + (String)bme.pressure + 
				"&tvoc=" + (String)bme.breathVocEquivalent + "&iaq=" + (String)bme.staticIaq + "&accuracy=" + (String) bme.iaqAccuracy + "&co=" + (String)bme.co2Equivalent + 
				"&volt=" + String(voltage_ext, 3) + "&time=" + (String)conn_time + "&rssi=" + (String)WiFi.RSSI() + "&next=" + (String)(TIME_TO_NEXT_SENDING/1000000U) 
		);

		http.GET();	 //Not aware of response, just send.
		http.end();
	}

	WiFi.forceSleepBegin();
}


void printToSerial() {
	Serial.print("\nVoltage: ");
	Serial.print(voltage_ext, 4);
	Serial.println("V");

	Serial.println("\nStab status: " + (String) bme.stabStatus + "\nResistance: " + bme.gasResistance +
				 "\nRun in status: " + bme.runInStatus + "\nGas accuracy: " + bme.compGasAccuracy +
				 "\nComp gas value: " + bme.compGasValue + "\nGas percentage: " + (String) bme.gasPercentage );
	Serial.println("\nACCURACY: " + (String)bme.iaqAccuracy + "\nStatic-IAQ: " + (String)bme.staticIaq 
		+ "\nCO2: " + (String)bme.co2Equivalent + " ppM" + "\ntVOC: " + (String)bme.breathVocEquivalent + " ppM"
		+ "\n\nTemp: " + (String)bme.temperature + " 'C" + "\nHumidity: " + (String)bme.humidity + "%RH"
		+ "\nPressure: " + (String)(bme.pressure/100) + "hPa" );
	
}


void printState(uint8_t sensor_state[] ){
	for (short i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
		Serial.printf("%02x ", sensor_state[i]);
		if (i % 16 == 15)	Serial.print("\n");
	}
	Serial.println();
}


void updateState(void)
{
	bme.getState(sensor_state);

	for (uint16_t i = 1; i <= BSEC_MAX_STATE_BLOB_SIZE ; i++)
		EEPROM.write(i , sensor_state[i]);
	EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
	EEPROM.commit();
   	EEPROM.end();

	Serial.println("\nI saved into EMMC memory: ");	
		printState(sensor_state);
}



void loop() 
{
	voltage_ext = analogRead(A0) * 3.3F / 1023.0F * 4.37F;   //4.37 offset at full charge

	bme.run();
	updateState();

	Serial.print("\n" + (String)(millis()-start) + " ms. for ambient measurement");
	
	sendData();
	printToSerial();

	Serial.println("\n----- I took "+ (String)(millis()-start) + " ms. to complete a cycle ----\n");
	
	ESP.deepSleep(TIME_TO_NEXT_SENDING, WAKE_RF_DISABLED);
	delay(1); /*Needed for proper sleeping*/
}
