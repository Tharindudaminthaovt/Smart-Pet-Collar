#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <DHT.h>
#include <Firebase_ESP_Client.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define DHTPIN D4  // Pin connected to the DHT sensor
#define DHTTYPE DHT11  // DHT 11

const char* ssid = "chillinnn";
const char* password = "12345678";

DHT dht(DHTPIN, DHTTYPE);

const int pulsePin = A0; // Pulse sensor connected to analog pin A0
const int numReadings = 10; // Number of readings to average
int readings[numReadings]; // Array to store readings
int idx = 0; // Index for array
int total = 0; // Total of readings

int BPM;
unsigned long previousMillis = 0;
const int samplingInterval = 20; // Sampling interval in milliseconds

#define gpsRxPin D1
#define gpsTxPin D2
SoftwareSerial neo6m(gpsTxPin, gpsRxPin);

TinyGPSPlus gps;

#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
bool signupOK = false;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  dht.begin();
  pinMode(pulsePin, INPUT);

  /* Assign the api key (required) */
  config.api_key = "AIzaSyDovvRiMrVNTuIJVt54e-duROvWK82mpPk";

  /* Assign the RTDB URL (required) */
  config.database_url = "https://iotfinal-f0658-default-rtdb.firebaseio.com/";

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("ok");
    signupOK = true;
  }
  else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  neo6m.begin(9600); // Initialize GPS
}

void loop() {
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();

    // Read temperature and humidity
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    // Check if any reads failed and exit early (to try again).
    if (!isnan(temperature) && !isnan(humidity)) {
      // Send temperature and humidity to Firebase
      if (Firebase.RTDB.setFloat(&fbdo, "environment/temperature", temperature) && Firebase.RTDB.setFloat(&fbdo, "environment/humidity", humidity)) {
        Serial.println("Temperature and Humidity Data Sent to Firebase");
      }
      else {
        Serial.println("Failed to Send Temperature and Humidity Data to Firebase");
      }
    }

    // Read the pulse sensor and calculate average
    total -= readings[idx];
    readings[idx] = analogRead(pulsePin);
    total += readings[idx];
    idx = (idx + 1) % numReadings;
    int average = total / numReadings;

    // Calculate Heart Rate
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= samplingInterval) {
      BPM = 60000 / (average * 2); // Convert to beats per minute
      previousMillis = currentMillis;
      // Send Heart Rate to Firebase
      if (Firebase.RTDB.setInt(&fbdo, "health/heart_rate", BPM)) {
        Serial.println("Heart Rate Data Sent to Firebase");
      }
      else {
        Serial.println("Failed to Send Heart Rate Data to Firebase");
      }
    }

    smartdelay_gps(1000); // Update GPS data

    if (gps.location.isValid()) {
      // Storing the Latitude and Longitude
      float latitude = gps.location.lat();
      float longitude = gps.location.lng();

      // Send Latitude and Longitude to Firebase
      if (Firebase.RTDB.setFloat(&fbdo, "gps/latitude", latitude) && Firebase.RTDB.setFloat(&fbdo, "gps/longitude", longitude)) {
        Serial.println("GPS Data Sent to Firebase");
      }
      else {
        Serial.println("Failed to Send GPS Data to Firebase");
      }
    }
  }
}

static void smartdelay_gps(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (neo6m.available())
      gps.encode(neo6m.read());
  } while (millis() - start < ms);
}
