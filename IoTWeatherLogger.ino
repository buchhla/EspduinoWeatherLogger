
// Depends on the following Arduino libraries:
// - Adafruit Unified Sensor Library:
// https://github.com/adafruit/Adafruit_Sensor
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <MS5611.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <ESP8266WiFi.h>

#define DHTPIN            2  // Pin which is connected to the DHT
// sensor.

// Uncomment the type of sensor in use:
#define DHTTYPE           DHT11 // DHT 11

// See guide for details on sensor wiring and usage:
// https://learn.adafruit.com/dht/overview

DHT_Unified     dht(DHTPIN, DHTTYPE);
MS5611          ms5611;
double          referencePressure;
#define delayMS         300000

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "ENTER ACCESS POINT NAME HERE"
#define WLAN_PASS       "ENTER PASSWORD HERE"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "ENTER USERNAME HERE"
#define AIO_KEY         "ENTER AIO KEY HERE"

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient      client;

// Store the MQTT server, username, and password in flash memory.
// This is required for using the Adafruit MQTT library.
const char      MQTT_SERVER[] PROGMEM = AIO_SERVER;
const char      MQTT_USERNAME[] PROGMEM = AIO_USERNAME;
const char      MQTT_PASSWORD[] PROGMEM = AIO_KEY;

// Setup the MQTT client class by passing in the WiFi client and MQTT
// server and login details.
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT,
                          MQTT_USERNAME, MQTT_PASSWORD);

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char      IoTWeatherLogger_TempDHT11_FEED[] PROGMEM =
  AIO_USERNAME "/feeds/IoTWeatherLogger_TempDHT11";
Adafruit_MQTT_Publish IoTWeatherData_TempDHT11 =
  Adafruit_MQTT_Publish(&mqtt, IoTWeatherLogger_TempDHT11_FEED);

const char      IoTWeatherLogger_TempMS5611_FEED[] PROGMEM =
  AIO_USERNAME "/feeds/IoTWeatherLogger_TempMS5611";
Adafruit_MQTT_Publish IoTWeatherData_TempMS5611 =
  Adafruit_MQTT_Publish(&mqtt, IoTWeatherLogger_TempMS5611_FEED);

const char      IoTWeatherLogger_Humidity_FEED[] PROGMEM =
  AIO_USERNAME "/feeds/IoTWeatherLogger_Humidity";
Adafruit_MQTT_Publish IoTWeatherData_Humidity =
  Adafruit_MQTT_Publish(&mqtt, IoTWeatherLogger_Humidity_FEED);

const char      IoTWeatherLogger_Pressure_FEED[] PROGMEM =
  AIO_USERNAME "/feeds/IoTWeatherLogger_Pressure";
Adafruit_MQTT_Publish IoTWeatherData_Pressure =
  Adafruit_MQTT_Publish(&mqtt, IoTWeatherLogger_Pressure_FEED);

const char      IoTWeatherLogger_Altitude_FEED[] PROGMEM =
  AIO_USERNAME "/feeds/IoTWeatherLogger_Altitude";
Adafruit_MQTT_Publish IoTWeatherData_Altitude =
  Adafruit_MQTT_Publish(&mqtt, IoTWeatherLogger_Altitude_FEED);

// Setup a feed called 'onoff' for subscribing to changes.
const char      ONOFF_FEED[] PROGMEM =
  AIO_USERNAME "/feeds/IoTWeatherLoggerOnOff";
Adafruit_MQTT_Subscribe IoTWeatherLoggerOnOff =
  Adafruit_MQTT_Subscribe(&mqtt, ONOFF_FEED);

/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function
// declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void            MQTT_connect();
double          Celcius2Fahrenheit(double celsius);
void            checkSettings();

void
setup()
{
  pinMode(BUILTIN_LED, OUTPUT); // Initialize the BUILTIN_LED pin
  // as an output
  Serial.begin(115200);
  delay(5000);

  // Connect to WiFi access point.
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  // Set WiFi mode to station (as opposed to AP or AP_STA)
  WiFi.mode(WIFI_STA);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&IoTWeatherLoggerOnOff);

  // Initialize device.
  dht.begin();
  Serial.println("DHTxx Unified Sensor Example");
  // Print temperature sensor details.
  sensor_t        sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" *C");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" *C");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" *C");
  Serial.println("------------------------------------");
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println("%");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println("%");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println("%");
  Serial.println("------------------------------------");
  // Set delay between sensor readings based on sensor details.
  // delayMS = sensor.min_delay / 1000;


  while (!ms5611.begin()) {
    Serial.println
    ("Could not find a valid MS5611 sensor, check wiring!");
    delay(500);
  }

  // Get reference pressure for relative altitude
  referencePressure = ms5611.readPressure();

  // Check settings
  checkSettings();

}

void
loop()
{
  // Ensure the connection to the MQTT server is alive (this will make
  // the first
  // connection and automatically reconnect when disconnected).  See the
  //
  //
  // MQTT_connect
  // function definition further below.
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &IoTWeatherLoggerOnOff) {
      Serial.print(F("Got: "));
      Serial.println((char *) IoTWeatherLoggerOnOff.lastread);
    }
  }

  digitalWrite(BUILTIN_LED, LOW); // Turn the LED on (Note that LOW
  // is the voltage level
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  Serial.println("DHT11 Sensor Readings--");
  if (isnan(event.temperature)) {
    Serial.println(" Error reading temperature!");
  } else {
    Serial.print(" Temperature: ");
    Serial.print(Celcius2Fahrenheit(event.temperature));
    Serial.println(" *F");

    // Now we can publish stuff!
    Serial.print(F("\nSending DHT11 Temperature Data "));
    Serial.print("...");
    if (!IoTWeatherData_TempDHT11.publish
        (Celcius2Fahrenheit(event.temperature))) {
      Serial.println(F("Failed"));
    } else {
      Serial.println(F("OK!"));
    }
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
  } else {
    Serial.print(" Humidity: ");
    Serial.print(event.relative_humidity);
    Serial.println("%");

    // Now we can publish stuff!
    Serial.print(F("\nSending DHT11 Humidity Data "));
    Serial.print("...");
    if (!IoTWeatherData_Humidity.publish(event.relative_humidity)) {
      Serial.println(F("Failed"));
    } else {
      Serial.println(F("OK!"));
    }
  }

  // Read raw values
  uint32_t        rawTemp = ms5611.readRawTemperature();
  uint32_t        rawPressure = ms5611.readRawPressure();

  // Read true temperature & Pressure
  double          realTemperature = ms5611.readTemperature();
  realTemperature = Celcius2Fahrenheit(realTemperature);
  long            realPressure = ms5611.readPressure();

  // Calculate altitude
  float           absoluteAltitude = ms5611.getAltitude(realPressure);
  float           relativeAltitude =
    ms5611.getAltitude(realPressure, referencePressure);

  Serial.println("MS5611 Sensor Readings--");

  Serial.print(" rawTemp = ");
  Serial.print(rawTemp);
  Serial.print(", realTemp = ");
  Serial.print(realTemperature);
  Serial.println(" *F");
  // Now we can publish stuff!
  Serial.print(F("\nSending MS5611 Temp Data "));
  Serial.print("...");
  if (!IoTWeatherData_TempMS5611.publish(realTemperature)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

  Serial.print(" rawPressure = ");
  Serial.print(rawPressure);
  Serial.print(", realPressure = ");
  Serial.print((float) (realPressure / 100));
  Serial.println(" kPa");

  Serial.print(F("\nSending Barometric Pressure Data "));
  Serial.print("...");
  if (!IoTWeatherData_Pressure.publish((float) (realPressure / 100))) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

  Serial.print(" absoluteAltitude = ");
  Serial.print(absoluteAltitude);
  Serial.print(" m, relativeAltitude = ");
  Serial.print(relativeAltitude);
  Serial.println(" m");

  Serial.print(F("\nSending Altitude Data "));
  Serial.print("...");
  if (!IoTWeatherData_Altitude.publish((float) relativeAltitude)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the
  // voltage HIGH

  // ping the server to keep the mqtt connection alive
  if (!mqtt.ping()) {
    mqtt.disconnect();
  }
  // Delay between measurements.
  delay(delayMS);
}

void
checkSettings()
{
  Serial.print("Oversampling: ");
  Serial.println(ms5611.getOversampling());
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if
// connecting.
void
MQTT_connect()
{
  int8_t          ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0
    // for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);    // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}

double
Celcius2Fahrenheit(double celsius)
{
  return 1.8 * celsius + 32;
}
