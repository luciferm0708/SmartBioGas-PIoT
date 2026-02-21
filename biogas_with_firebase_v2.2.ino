#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_AHTX0.h>
#include "ScioSense_ENS160.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <FirebaseESP32.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include "DHT.h"
#include "biogas_rf_modelv0.2.1.h"

//----------------------------Pressure Sensor Definitions----------------------------//
#define trans_pin 35
const float alpha = 0.95;
const float aRef =  4.6;
const int adcResolution = 4095;
float filteredVal = 0, voltage = 0, psiVal = 0, kpaVal = 0;
const float zeroOffset = 0.450;
const float scaleFactor = 0.0401;

//---------------------------MQ5/MQ136-DS18B20-DHT11-------------------------------------//
#define mq5_pin 33
#define mq136_pin 32
#define DHTPIN 23
#define DHTTYPE DHT11
#define DSB 14

Adafruit_AHTX0 AHT;
ScioSense_ENS160 ens160(ENS160_I2CADDR_1);
DHT dht(DHTPIN, DHTTYPE);
DeviceAddress thermometerAddress;
OneWire oneWire(DSB);
DallasTemperature tempSensor(&oneWire);

//-------------------------WiFi---------------------------------------------------------//
#define WIFI_SSID "DIU_Daffodil Smart City"
#define WIFI_PASSWORD "diu123456789"

//------------------------Firebase----------------------------------------------------//
#define API_KEY ""
#define DATABASE_URL  ""
#define USER_EMAIL    "tester000@gmail.com"
#define USER_PASSWORD "123456"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;

float tempReading = 0.0;
int tempC = 0;
int humidity = 0;
float extTempC = 0.0;
float extHum = 0.0;
int mq5_val = 0;
int mq136_val = 0;
int co2_val = 0;
bool aht_ok = true;
TaskHandle_t SensorTaskHandle = NULL;
TaskHandle_t FirebaseTaskHandle = NULL;
SemaphoreHandle_t dataMutex = NULL;

//--------------------WDT Timeout----------------------------------------------//
const int WDT_TIMEOUT_S = 10;

bool connectWiFi(uint32_t timeout_ms = 15000);
void ensureWiFi();
void sendDataToFB();
void SensorTask(void *pvParameters);
void FirebaseTask(void *pvParameters);

//----------------------WiFi Helpers-----------------------------------------//
void onWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_START:  Serial.println("[WiFi] STA Start"); break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:  Serial.println("[WiFi] Connected to AP"); break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("[WiFi] Got IP: %s\n", WiFi.localIP().toString().c_str());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("[WiFi] Disconnected → will try to reconnect");
      break;
    default: break;
  }
}

bool connectWiFi(uint32_t timeout_ms) {
  WiFi.onEvent(onWiFiEvent);
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.setAutoReconnect(true);

  WiFi.disconnect(true, true);
  delay(200);

  Serial.printf("Connecting to \"%s\" ...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeout_ms) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Connected. IP: %s  RSSI: %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return true;
  } else {
    Serial.println("WiFi connect timeout.");
    return false;
  }
}

void ensureWiFi() {
  static uint32_t lastTry = 0;
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastTry < 5000) return;
  lastTry = millis();
  Serial.println("[WiFi] ensureWiFi(): reconnecting...");
  connectWiFi(8000);
}

esp_task_wdt_config_t wdt_config = {
  .timeout_ms = WDT_TIMEOUT_S * 1000,
  .idle_core_mask = (1 << 0) | (1 << 1),   // Enable WDT on both cores
  .trigger_panic = true
};

//-------------------------------------------------------TinyML DEFINITION----------------------------------------------------------------------//
using Eloquent::ML::Port::RandomForestRegressor;
RandomForestRegressor rfModel;
float ch4ProdRate = 0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  delay(200);

  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) {
    Serial.println("Failed to create data Mutex");
  }
  esp_task_wdt_init(&wdt_config);
  Serial.printf("WDT initialized with %d s timeout\n", WDT_TIMEOUT_S);

  connectWiFi();

  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;
  Firebase.reconnectWiFi(true);
  Firebase.begin(&config, &auth);

  int samples = 10, sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(trans_pin);
    delay(10);
  }
  filteredVal = sum / (float)samples;
  Serial.println("Pressure Sensor is Ready!");

  //-------------ENS160----------------------//
  Serial.println("Initializing ENS160...");
  ens160.begin();
  if (ens160.available()) {
    ens160.setMode(ENS160_OPMODE_STD);
  }

  //----------------------AHT20------------------------//


  if (!AHT.begin()) {
    Serial.println("AHT20 not found! Bypassing...");
    aht_ok = false;   // mark sensor as unavailable
  }


  //--------------------DHT11--------------------------//
  dht.begin();

  //---------------------DS18B20-------------------------//
  Serial.println("Initializing DS18B20 for Internat Temperature...");
  tempSensor.begin();
  if (!tempSensor.getAddress(thermometerAddress, 0)) {
    Serial.println("DS18B20 Not Found!");
  } else {
    Serial.println("DS18B20 Address: ");
    for (uint8_t i = 0; i < 8; i++) {
      if (thermometerAddress[i] < 16) {
        Serial.println("0");
      }
      Serial.println(thermometerAddress[i], HEX);
    }
    Serial.println();
  }

  tempSensor.setResolution(thermometerAddress, 11);

  //------------Sensor Task => Core 1-------------------------//
  BaseType_t ok;
  ok = xTaskCreatePinnedToCore (
    SensorTask,
    "SensorTask",
    8192,
    NULL,
    1,
    &SensorTaskHandle,
    1
  );

  if (ok != pdPASS)
    Serial.println("Failed to Create Sensor Task");
  
  //-------------Firebase Task => Core 0----------------------//
  ok = xTaskCreatePinnedToCore (
    FirebaseTask,
    "FirebaseTask",
    16384,
    NULL,
    1,
    &FirebaseTaskHandle,
    0
  );

  if (ok != pdPASS) 
    Serial.print("Failed to Create Firebase Task");
}

void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelay(pdMS_TO_TICKS(1000));
}

void updateInternalTemperatureFromDS18B20() {
  tempSensor.requestTemperatures();
  float t = tempSensor.getTempC(thermometerAddress);
  if (t == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: DS18B20 Disconnected!");
    return;
  }
  tempReading = t;
  tempC = (int)round(t);

  Serial.print("Internal Temperature (DS18B20): ");
  Serial.print(tempReading);
  Serial.println(" °C");
}

void updateHumidityFromAHT20() {
  if (!aht_ok) {
    humidity = 0;   
    return;
  }

  sensors_event_t tempEvt;
  sensors_event_t humEvtLocal;

  AHT.getEvent(NULL, &tempEvt);
  AHT.getEvent(&humEvtLocal, &tempEvt);

  humidity = (int)round(humEvtLocal.relative_humidity);
  Serial.print("Internal Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
}


void updateENS160() {
  if (ens160.available()) {
    ens160.set_envdata(tempC, humidity);
    ens160.measure(true);
    co2_val = ens160.geteCO2();
    Serial.print("CO₂: ");
    Serial.print(co2_val);
    Serial.println(" ppm");
  } else {
    co2_val = 0;
  }
  
}

void updateMQSensors() {
  mq5_val = analogRead(mq5_pin);
  mq136_val = analogRead(mq136_pin);
  Serial.print("H₂S: ");
  Serial.println(mq5_val);
  Serial.print("CH₄: ");
  Serial.println(mq136_val);
}

void updatePressure() {
  int sensorVal = analogRead(trans_pin);
  filteredVal = (alpha * filteredVal) + ((1.0 - alpha) *  sensorVal);
  voltage = (filteredVal / adcResolution) * aRef;
  psiVal = (voltage - zeroOffset) / scaleFactor;
  kpaVal = psiVal * 6.89476;
  kpaVal = kpaVal * 0.14;
  Serial.print("Voltage (V): ");
  Serial.print(voltage, 3);
  Serial.print(" | Pressure (PSI): ");
  Serial.print(psiVal, 2);
  Serial.print(" | Pressure (kPa): ");
  Serial.println(kpaVal, 2);
}

void updateExternalDHT() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if(!isnan(t))
    extTempC = t;

  Serial.print("External Temperature: ");
  Serial.println(extTempC);

  if(!isnan(h))
    extHum = h;

  Serial.print("External Humidity: ");
  Serial.println(extHum);
}

void SensorTask(void *pvParamaters) {
  esp_task_wdt_add(NULL);
  Serial.println("Sensor Task Started and Added to WDT (CORE 1)");

  for (;;) {
    updateInternalTemperatureFromDS18B20();
    updateHumidityFromAHT20();
    updateENS160();
    updateMQSensors();
    updatePressure();
    updateExternalDHT();

    float features[5];
    features[0] = tempReading;
    features[1] = humidity;
    features[2] = extTempC;
    features[3] =  extHum;
    features[4] = kpaVal;

    ch4ProdRate = rfModel.predict(features);

    Serial.println("Predicted CH4 Production Rate: ");
    Serial.print(ch4ProdRate);
    Serial.println(" L/h");


    if(dataMutex != NULL) {
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        xSemaphoreGive(dataMutex);
      } else {
        Serial.println("SensorTask: Failed to take dataMutex");
      }
    }

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(800));
  }
}

void FirebaseTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  Serial.println("Firebase Task Started and Added to WDT (CORE 0)");

  unsigned long lastLocalSend = 0;

  for(;;) {
    ensureWiFi();

    float local_tempReading = 0;
    int local_tempC = 0;
    int local_humidity = 0;
    int local_co2 = 0;
    int local_mq5 = 0;
    int local_mq136 = 0;
    float local_kpa = 0;
    float local_extTemp = 0;
    float local_extHum = 0;
    float local_ch4ProdRate = 0;
  

    if (dataMutex != NULL) {
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        local_tempReading = tempReading;
        local_tempC = tempC;
        local_humidity = humidity;
        local_co2 = co2_val;
        local_mq5 = mq5_val;
        local_mq136 = mq136_val;
        local_kpa = kpaVal;
        local_extTemp = extTempC;
        local_extHum = extHum;
        local_ch4ProdRate = ch4ProdRate;
        xSemaphoreGive(dataMutex);
      } else {
          Serial.println("FirebaseTask: failed to take dataMutex, will try again next loop");
      }
    }
    if (millis() - lastLocalSend >= 10000) {
      lastLocalSend = millis();

        if (WiFi.status() == WL_CONNECTED && Firebase.ready()) {

      // Create a unique child ID using timestamp
      String entryID = String(millis());
      String basePath = "/biogas_data/" + entryID;

      bool ok;

      // Save timestamp FIRST
      ok = Firebase.setTimestamp(fbdo, basePath + "/timestamp");
      if (!ok) Serial.println("Timestamp upload failed: " + fbdo.errorReason());

      ok = Firebase.setFloat(fbdo, basePath + "/temperature", local_tempReading);
      ok = Firebase.setInt(fbdo, basePath + "/humidity", local_humidity);
      ok = Firebase.setInt(fbdo, basePath + "/co2", local_co2);
      ok = Firebase.setInt(fbdo, basePath + "/ch4", local_mq5);
      ok = Firebase.setInt(fbdo, basePath + "/h2s", local_mq136);
      ok = Firebase.setFloat(fbdo, basePath + "/pressure", local_kpa);
      ok = Firebase.setFloat(fbdo, basePath + "/extTemp", local_extTemp);
      ok = Firebase.setFloat(fbdo, basePath + "/extHum", local_extHum);
      ok = Firebase.setFloat(fbdo, basePath + "/ch4ProdRate", local_ch4ProdRate);

      Serial.println("Uploaded history entry → " + basePath);

      esp_task_wdt_reset();
  } else {
        Serial.println("FirebaseTask: WiFi or Firebase Not Ready!");
        if (WiFi.status() == WL_CONNECTED && !Firebase.ready()) {
          Serial.println("Re-initializing Firebase...");
          Firebase.begin(&config, &auth);
        } 
      }
    }
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void sendDataToFB() {
  return;
}
