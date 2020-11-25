#include <Adafruit_BMP280.h>

#include <Adafruit_BME680.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define batt_in A0

Adafruit_BMP280 bmp;
Adafruit_BME680 bme;

float voltage_ext;
int vPercent_ext;
float temp_ext;
float humidity_ext;
float pressure_ext;
float airIndex;


void setup() {
    Serial.begin(115200);
    Wire.begin(12,13); //I2C start

    pinMode(15,OUTPUT); //sensor GND
    digitalWrite(15,LOW);

    if (! bmp.begin(0x76)) Serial.println("Couldn't find BMP");

    pinMode(batt_in, INPUT);
}


void ambientMeasurement() {
    temp_ext = bmp.readTemperature();
    pressure_ext = bmp.readPressure() / 100;
    //humidity_ext = bme.readHumidity();
}


void readBattery() {
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
    const float offset = 4.662177;
    voltage_ext = (voltage_reading / nReadings) * 3.3 / 1024 * offset;

    /** Map seems to not work   vPercent = map(voltage, 3.1 , 4.20, 0, 100);
     * Minimum voltage 3.1V (as 0%)
     * Maximum voltage 4.2V (100%)
     * Percent value can rise up to 100% while charging
     * That's the desired behaviour to detect the "charging rate"
     */
    if (voltage_ext >= 3.1)					  //4.2-3.1
         vPercent_ext = (voltage_ext - 3.1) * 100 / 1.1;
    else vPercent_ext = 0;
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
    Serial.println();
    Serial.println("----------------------------------------");
    /*--------------------------------------------------------------------------------*/
}


void loop() {
    readBattery();
    ambientMeasurement();

    printToSerial();
    delay(2500);
}
