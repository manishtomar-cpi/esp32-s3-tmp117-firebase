#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>
#include <Wire.h>
#include <Adafruit_TMP117.h>
#include <Adafruit_Sensor.h>
#include "time.h"

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "TP-Link_2A33"
#define WIFI_PASSWORD "27552279"

// Insert Firebase project API Key
#define API_KEY "AIzaSyAPgyDugu3b1N1eUK2w36nmrRE49EEINMA"

// Insert RTDB URL without https://
#define DATABASE_URL "https://esp32-temp-e3226-default-rtdb.europe-west1.firebasedatabase.app/"

//Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

Adafruit_TMP117 tmp117;

unsigned long sendDataPrevMillis = 0;
bool signupOK = false;

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;  // Adjust to your timezone offset in seconds (e.g., for GMT+1, it's 3600 seconds)
const int daylightOffset_sec = 3600;  // Adjust if daylight savings is applicable

String getFormattedTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return "TimeError";
  }
  char timeStringBuff[50]; // Create a buffer for time string
  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo); // Format: 2024-09-17 13:45:00
  return String(timeStringBuff);
}


void setup() {
  // Start Serial Monitor
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // Initialize NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("NTP Time synchronization setup complete.");

  // Firebase configuration
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  // Anonymous sign-in (you can also use email/password if needed)
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase sign-up successful");
    signupOK = true;
  } else {
    Serial.printf("Firebase sign-up failed: %s\n", config.signer.signupError.message.c_str());
  }

  // Set up Firebase
  config.token_status_callback = tokenStatusCallback; // For token generation
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // Initialize I2C communication (GPIO21 for SDA, GPIO20 for SCL)
  Wire.begin(21, 20);  // SDA = GPIO21, SCL = GPIO20

  // Initialize TMP117 temperature sensor (I2C)
  if (!tmp117.begin(0x48)) {  // 0x48 is the default I2C address for TMP117
    Serial.println("Failed to find TMP117 sensor!");
    while (1);  // Stop if the sensor isn't found
  }

  Serial.println("TMP117 sensor found! Sending temperature data to Firebase...");
}


void loop() {
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 5000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();

    // Read temperature from TMP117 sensor
    sensors_event_t temp_event;
    tmp117.getEvent(&temp_event);  // Get temperature data from the sensor

    float temperatureC = temp_event.temperature;  // Temperature in Celsius

    // Get the current timestamp in a human-readable format
    String timestamp = getFormattedTime();

    // Log the timestamp and temperature to the Serial Monitor
    Serial.println("Timestamp: " + timestamp);
    Serial.println("Temperature: " + String(temperatureC) + " °C");

    // Create a unique path for each entry using the timestamp
    String path = "/sensors/temperature/formated/" + String(millis());

    // Create a JSON object to hold both timestamp and temperature
    FirebaseJson json;
    json.set("timestamp", timestamp);
    json.set("temperature", temperatureC);

    // Send temperature data and timestamp to Firebase
    if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
      Serial.println("Data sent successfully to Firebase");
      Serial.println("PATH: " + fbdo.dataPath());
    } else {
      Serial.println("Failed to send data to Firebase");
      Serial.println("REASON: " + fbdo.errorReason());
    }
  }
}
































