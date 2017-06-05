/*****************************************************************************
  Arduino sketch to report the temperature using an ESP8266 with connected
  Dallas DS18B20 sensor, for the Blynk system.
  http://www.blynk.cc
*****************************************************************************/

#define BLYNK_DEBUG
#define BLYNK_PRINT Serial // Comment this out to disable prints and save space

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "myssid";
char pass[] = "mypassword";

// Temperature sensor full ID, including family code and CRC
DeviceAddress tempSensor = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };

// Pin used for the OneWire interface
#define ONEWIRE_PIN 4

OneWire oneWire(ONEWIRE_PIN);
DallasTemperature sensors(&oneWire);
SimpleTimer timer;

// Virtual pins to send temperature on
#define V_PIN_A V0
#define V_PIN_B V1

#undef TEMP_IN_FAHRENHEIT
// Uncomment to use Fahrenheit else use Celsius
#define TEMP_IN_FAHRENHEIT

// Temperature reading interval, in SECONDS
#define READ_INTERVAL 30

// Check interval for network connection, in milliseconds
// This should be a bit shorter than the Blynk reconnect poll time, which
// at the time of writing was 7 seconds
#define CONNECTION_CHECK_TIME 6500

// LED flash time for link down, in milliseconds
#define LED_DISC_FLASH_TIME 2000

// LED blink rate for "sensor not found" indication, in milliseconds
#define NO_SENSOR_BLINK_SPEED 500

// LED blink time for "sensor not found" indication, in milliseconds
#define NO_SENSOR_BLINK_TIME 200

// LED flash time for successful sensor read, in milliseconds
#define LED_READ_OK_FLASH_TIME 30

// LED flash time for sensor read error, in milliseconds
#define LED_READ_ERR_FLASH_TIME 5000

// Pin for LED indicator
#define LED_PIN 2

// The number of bits of temperature resolution
// 9 = 0.5, 10 = 0.25, 11 = 0.125, 12 = 0.0625 degrees Celsius
#define TEMPERATURE_RES_BITS 10

// Temperature conversion wait time, in milliseconds
#define READ_CONV_WAIT 800

// Pin values for indicator LED on and off
#define LED_ON LOW
#define LED_OFF HIGH

//--------------------- SETUP -------------------
void setup()
{
  Serial.begin(9600);

  pinMode(LED_PIN, OUTPUT);
  indLEDoff();

  sensors.begin();
  if (!sensors.isConnected(tempSensor)) {
    while (true) {
      indLEDon();
      delay(NO_SENSOR_BLINK_TIME);
      indLEDoff();
      delay(NO_SENSOR_BLINK_SPEED - NO_SENSOR_BLINK_TIME);
    }
  }

  sensors.setResolution(tempSensor, TEMPERATURE_RES_BITS);
  sensors.setWaitForConversion(false);

  Blynk.begin(auth, ssid, pass);

  // Connection timer
  timer.setInterval(CONNECTION_CHECK_TIME, connectionCheck);

  // Sensor read timer
  timer.setInterval(READ_INTERVAL * 1000, startSensorRead);
}
//-----------------------------------------------

// Blink the LED if the connection is down
void connectionCheck() {
  if (Blynk.connected()) {
    return;
  }

  indLEDon();
  delay(LED_DISC_FLASH_TIME);
  indLEDoff();
}

// Start temperature conversion and initiate indicator LED blink
void startSensorRead() {
  if (!Blynk.connected()) {
    return;
  }

  sensors.requestTemperatures();
  timer.setTimeout(READ_CONV_WAIT, sensorRead);
}

// Read the temperature from the sensor
void sensorRead() {
  int16_t tempRaw;
  float temp;

  if (!Blynk.connected()) {
    return;
  }

  tempRaw = sensors.getTemp(tempSensor);
  if (tempRaw != DEVICE_DISCONNECTED_RAW) {
#ifdef TEMP_IN_FAHRENHEIT
    temp = sensors.rawToFahrenheit(tempRaw);
#else
    temp = sensors.rawToCelsius(tempRaw);
#endif
    Blynk.virtualWrite(V_PIN_A, temp);
    Blynk.virtualWrite(V_PIN_B, temp);

    flashLED(LED_READ_OK_FLASH_TIME);
  }
  else {
    flashLED(LED_READ_ERR_FLASH_TIME);
  }
}

//--------------------- LOOP --------------------
void loop()
{
  Blynk.run();
  timer.run();
}
//-----------------------------------------------

// Flash the indicator LED for the specified duration
void flashLED(long dur) {
  timer.setTimeout(dur, indLEDoff);
  indLEDon();
}

// Turn on the indicator LED
void indLEDon() {
  digitalWrite(LED_PIN, LED_ON);
}

// Turn off the indicator LED
void indLEDoff() {
  digitalWrite(LED_PIN, LED_OFF);
}

