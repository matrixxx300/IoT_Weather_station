extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <Wire.h>
#include <AsyncMqttClient.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <WiFi.h>
#include <SPI.h>
#include <PMserial.h>

#include "prot.pb.h"
#include "pms.pb.h"
#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"

// MQTT CONFIGURATIONS
#define WIFI_SSID "OPPO A53"
#define WIFI_PASSWORD "12345678"
#define MQTT_HOST IPAddress(192, 168, 43, 29)
#define MQTT_PORT 1883

// TOPICS
#define MQTT_PUB_PROTO  "esp/bme680/proto"
#define MQTT_PUB_PMS  "esp/bme680/pms"

// I2C Pinout
// SDA -> GPIO 21
// SCL -> GPIO 22

// SPI Pinout
//#define BME_SCK 18
//#define BME_MISO 19
//#define BME_MOSI 23
//#define BME_CS 5

#define PMS_RX 16
#define PMS_TX 17

Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS);
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // SPI

SerialPM pms(PMSx003, PMS_RX, PMS_TX);

// Variables to hold sensor readings
float temperature;
float humidity;
float pressure;
float gasResistance;
float pm01;
float pm25;
float pm10;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

AtmosphericConditions atmosphericConditions;
PM pm;

uint8_t buffer[128];
pb_ostream_t stream;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings

// Prototypes
void getBME680Readings();
void connectToWifi();
void connectToMqtt();
void WiFiEvent(WiFiEvent_t event);
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
//void onMqttSubscribe(uint16_t packetId, uint8_t qos);
//void onMqttUnsubscribe(uint16_t packetId);
void onMqttPublish(uint16_t packetId);
void encodeProto();
void encodePMS();

void setup() {
  Serial.begin(115200);
  Serial.println();

  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1);
  }

  pms.init();

  // WiFi and MQTT timers
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToWifi();

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  atmosphericConditions = AtmosphericConditions_init_zero;
  stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
}

void loop() {
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds)
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    getBME680Readings();
    Serial.println();
    Serial.printf("Temperature = %.2f ºC \n", temperature);
    Serial.printf("Humidity = %.2f % \n", humidity);
    Serial.printf("Pressure = %.2f hPa \n", pressure);
    Serial.printf("Gas Resistance = %.2f KOhm \n", gasResistance);

    encodeProto();
    
    // Publish an MQTT message on topic esp/bme680/proto
    uint16_t packetIdPub5 = mqttClient.publish(MQTT_PUB_PROTO, 1, true, (const char*)buffer, stream.bytes_written);
    Serial.printf("\nPublishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_PROTO, packetIdPub5);

    pms.read();// read the PM sensor
    pm01 = pms.pm01;
    pm25 = pms.pm25;
    pm10 = pms.pm10;
    Serial.print(F("\nPM1.0 "));Serial.print(pm01);Serial.print(F(" [ug/m3]"));
    Serial.print(F("\nPM2.5 "));Serial.print(pm25);Serial.print(F(" [ug/m3]"));
    Serial.print(F("\nPM10 ")) ;Serial.print(pm10);Serial.println(F(" [ug/m3]"));

    encodePMS();

    // Publish an MQTT message on topic esp/bme680/pms
    uint16_t packetIdPub6 = mqttClient.publish(MQTT_PUB_PMS, 1, true, (const char*)buffer, stream.bytes_written);
    Serial.printf("\nPublishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_PMS, packetIdPub6);
  }
}

void encodeProto() {
  pm.PM1 = pm01;
  pm.PM2 = pm25;
  pm.PM10 = pm10;

  memset(buffer, 0, 128);
  stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  
  bool status = pb_encode(&stream, AtmosphericConditions_fields, &atmosphericConditions);
  if (!status)
  {
    Serial.println("Failed to encode");
    return;
  }

  Serial.print("Message Length: ");
  Serial.println(stream.bytes_written);
  Serial.print("Message: ");

  for (int i = 0; i < stream.bytes_written; i++) {
    Serial.printf("%02X", buffer[i]);
  }
}

void encodePMS() {
  atmosphericConditions.temperature = temperature;
  atmosphericConditions.humidity = humidity;
  atmosphericConditions.pressure = pressure;
  atmosphericConditions.gas = gasResistance;

  memset(buffer, 0, 128);
  stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  
  bool status = pb_encode(&stream, PM_fields, &pm);
  if (!status)
  {
    Serial.println("Failed to encode");
    return;
  }

  Serial.print("Message Length: ");
  Serial.println(stream.bytes_written);
  Serial.print("Message: ");

  for (int i = 0; i < stream.bytes_written; i++) {
    Serial.printf("%02X", buffer[i]);
  }
}

void getBME680Readings() {
  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }
  temperature = bme.temperature;
  pressure = bme.pressure / 100.0;
  humidity = bme.humidity;
  gasResistance = bme.gas_resistance / 1000.0;
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}
