/*****************************************************************************
  Arduino sketch to report the temperature using an ESP8266 with connected
  Dallas DS18B20 sensor, for the Blynk system.
  http://www.blynk.cc
*****************************************************************************/

/*
Copyright (c) 2017 Scott Allen

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

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

// Alert messages
char alertMessageLow[] = "House temperature is LOW!";
char alertMessageHigh[] = "House temperature is HIGH!";

// Pin used for the OneWire interface
#define ONEWIRE_PIN 4

OneWire oneWire(ONEWIRE_PIN);
DallasTemperature sensors(&oneWire);
SimpleTimer timer;

int alertTempLow, alertTempHigh;
boolean lowAlertOn = false;
boolean highAlertOn = false;

// Virtual pins to send temperature on
#define V_PIN_TEMP_A V0 // Value - Widget must allow color change
#define V_PIN_TEMP_B V1
// Virtual pins for alert temperature set points
#define V_PIN_ALERT_LOW V2
#define V_PIN_ALERT_HIGH V3

#undef TEMP_IN_FAHRENHEIT
// Uncomment to use Fahrenheit or comment out for Celsius
#define TEMP_IN_FAHRENHEIT

// Hysteresis in degrees for alerts
#define ALERT_HYSTERESIS 3

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

// Widget colors for alerts
#define ALERT_COLOR_OK   "#23C48E" // Green
#define ALERT_COLOR_LOW  "#5F7CD8" // Dark Blue
#define ALERT_COLOR_HIGH "#D3435C" // Red

//--------------------- SETUP -------------------
void setup() {
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

  // Set the resolution of the temperature readings
  sensors.setResolution(tempSensor, TEMPERATURE_RES_BITS);

  // We'll do the "wait for conversion" ourselves using a timer,
  // to avoid the call to delay() that the library would use
  sensors.setWaitForConversion(false);

  // Start up Blynk
  Blynk.begin(auth, ssid, pass);

  // Start the interval timer that checks if the connection is up
  timer.setInterval(CONNECTION_CHECK_TIME, connectionCheck);

  // Start the timer that handles the temperature read interval
  timer.setInterval(READ_INTERVAL * 1000, startSensorRead);
}
//-----------------------------------------------

// Synchronize pins when connection comes up
BLYNK_CONNECTED() {
  Blynk.syncAll();
}

// Set low alert temperature
BLYNK_WRITE(V_PIN_ALERT_LOW) {
  alertTempLow = param.asInt();
}

// Set high alert temperature
BLYNK_WRITE(V_PIN_ALERT_HIGH) {
  alertTempHigh = param.asInt();
}

// Blink the LED if the connection is down
void connectionCheck() {
  if (Blynk.connected()) {
    return;
  }

  indLEDon();
  delay(LED_DISC_FLASH_TIME);
  indLEDoff();
}

// Start a temperature read and the conversion wait timer
void startSensorRead() {
  if (!Blynk.connected()) {
    return;
  }

  sensors.requestTemperatures();
  timer.setTimeout(READ_CONV_WAIT, sensorRead);
}

// Read the temperature from the sensor and perform the appropriate actions
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

    // Send temperature value
    Blynk.virtualWrite(V_PIN_TEMP_A, temp);
    Blynk.virtualWrite(V_PIN_TEMP_B, temp);

    // Low temperature alerts
    if ((temp <= alertTempLow) && !lowAlertOn) {
      Blynk.setProperty(V_PIN_TEMP_A, "color", ALERT_COLOR_LOW);
      Blynk.notify(alertMessageLow);
      lowAlertOn = true;
    }
    else if (lowAlertOn && (temp > alertTempLow + ALERT_HYSTERESIS)) {
      Blynk.setProperty(V_PIN_TEMP_A, "color", ALERT_COLOR_OK);
      lowAlertOn = false;
    }

    // High temperature alerts
    if ((temp >= alertTempHigh) && !highAlertOn) {
      Blynk.setProperty(V_PIN_TEMP_A, "color", ALERT_COLOR_HIGH);
      Blynk.notify(alertMessageHigh);
      highAlertOn = true;
    }
    else if (highAlertOn && (temp < alertTempHigh - ALERT_HYSTERESIS)) {
      Blynk.setProperty(V_PIN_TEMP_A, "color", ALERT_COLOR_OK);
      highAlertOn = false;
    }

    flashLED(LED_READ_OK_FLASH_TIME);
  }
  else {
    flashLED(LED_READ_ERR_FLASH_TIME);
  }
}

//--------------------- LOOP --------------------
void loop() {
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

