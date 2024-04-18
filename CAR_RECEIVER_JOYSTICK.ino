#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <FirebaseESP8266.h>
#include <ArduinoJson.h>
#include <FastLED.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Preferences.h>
#include "SPIFFS.h"
#include <FS.h>
#include <TinyGPS++.h>

#define STATUS_LED_DATA 13
#define NUM_STATUS_LED 1
#define BRIGHTNESS 50
#define COLOR_ORDER GRB
#define CHIPSET WS2812
#define STATUS_LED_POSI 0
#define BOOT_BUTTON_PIN 0
#define HORN_PIN 12
#define RX_PIN 16 //RX2 PIN OF ESP32
#define TX_PIN 17
#define BATTERY_PIN 34
#define LEFT_FORWARD 2 //IN1
#define LEFT_BACKWARD 4 //IN2
#define RIGHT_FORWARD 18 //IN3
#define RIGHT_BACKWARD 19 //IN4
#define SPEED_CONTROLL 32

CRGB status_leds[NUM_STATUS_LED];
HardwareSerial neogps(1);
TinyGPSPlus gps;

unsigned long updatePrevMillis = 0;
const unsigned long updateInterval = 25000;
const int portalOpenTime = 300000;
const int freq = 10000;
const int pwmChanel = 0;
const int pwmResolution = 8;
const int numBatReadings = 30;
int indexNo = 0;
bool onDemand;
String firebaseStatus = "";
float total = 0.0;
float latt = 00.000000;
float lngi = 00.000000;
float gpsSpeed = 0;
float speedNow = 0;
float batteryValue = 11.99;
float blcValue = 11.99;
float bhcValue = 12.5;
uint8_t throttleValue;
uint8_t forwardValue;
uint8_t backwardValue;
uint8_t leftValue;
uint8_t rightValue;
uint8_t hornValue;
uint8_t headlightValue;
uint8_t satNo;
uint8_t gpsValue;


FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;
AsyncWebServer server(80);

Preferences preferences;
TaskHandle_t Task1;
SemaphoreHandle_t variableMutex;

void setup() {
  Serial.begin(115200);
  neogps.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(500);

  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
    return;
  }

  pinMode(BOOT_BUTTON_PIN, INPUT);
  pinMode(BATTERY_PIN, INPUT);
  pinMode(HORN_PIN, OUTPUT);
  pinMode(LEFT_FORWARD, OUTPUT);
  pinMode(LEFT_BACKWARD, OUTPUT);
  pinMode(RIGHT_FORWARD, OUTPUT);
  pinMode(RIGHT_BACKWARD, OUTPUT);
  delay(500);

  ledcSetup(pwmChanel, freq, pwmResolution);
  ledcAttachPin(SPEED_CONTROLL, pwmChanel);
  delay(500);

  FastLED.addLeds<CHIPSET, STATUS_LED_DATA, COLOR_ORDER>(status_leds, NUM_STATUS_LED);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
  delay(500);

  connectWiFi();
  delay(500);
  connectFirebase();
  delay(500);

  variableMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(
    loop1,
    "Task1",
    10000,
    NULL,
    1,
    &Task1,
    1);
  delay(500);
}


void connectFirebase() {
  preferences.begin("my-app", false);

  if (preferences.getString("firebaseUrl", "") != "" && preferences.getString("firebaseToken", "") != "") {
    Serial.println("Firebase settings already exist. Checking Firebase connection...");

    String firebaseUrl = preferences.getString("firebaseUrl", "");
    String firebaseToken = preferences.getString("firebaseToken", "");
    config.database_url = firebaseUrl;
    config.api_key = firebaseToken;
    Firebase.signUp(&config, &auth, "", "");  //for anonymous user
    delay(100);
    Firebase.begin(&config, &auth);
    delay(100);

    Firebase.reconnectWiFi(true);
    delay(100);

    if (isFirebaseConnected() == true) {
      Serial.println("Connected to Firebase. Skipping server setup.");
      firebaseStatus = "ok";
    } else {
      Serial.println("Failed to connect to Firebase. Starting server setup.");
      setupServer();
    }
  } else {
    Serial.println("Firebase settings not found. Starting server setup.");
    setupServer();
  }

}


void setupServer() {
  preferences.begin("my-app", false);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html", String(), false);
  });

  server.on("/Submit", HTTP_POST, [](AsyncWebServerRequest * request) {

    String firebaseUrl = request->arg("url");
    String firebaseToken = request->arg("token");

    preferences.putString("firebaseUrl", firebaseUrl);
    preferences.putString("firebaseToken", firebaseToken);

    config.database_url = firebaseUrl;
    config.api_key = firebaseToken;
    Firebase.signUp(&config, &auth, "", "");  //for anonymous user
    delay(100);
    Firebase.begin(&config, &auth);
    delay(100);
    
    Firebase.reconnectWiFi(true);
    delay(100);

    if (isFirebaseConnected() == true) {
      firebaseStatus = "ok";
      Serial.println("Firebase settings saved");
      delay(300);
      Serial.println("Success");
      delay(300);
      Serial.println("Restarting your device...");
      delay(500);
      ESP.restart();
    } else {
      firebaseStatus = "";
      Serial.println("Firebase settings saved");
      delay(300);
      Serial.println("Error! Check your credentials.");
      delay(300);
      Serial.println("Restarting your device...");
      delay(500);
      ESP.restart();
    }
  });

  server.serveStatic("/", SPIFFS, "/");
  server.begin();

  Serial.println("server begin");
  Serial.println(WiFi.localIP());

  showLedStatus(0, 0, 255);


  delay(portalOpenTime);
  Serial.println("Restarting your device...");
  delay(1000);
  ESP.restart();
}

void connectWiFi() {

  WiFiManager wm;
  WiFi.disconnect();
  delay(50);
  bool success = false;
  while (!success) {
    wm.setConfigPortalTimeout(60);
    success = wm.autoConnect("ESP.CAR");
    if (!success) {
      Serial.println("ESP.CAR");
      Serial.println("Setup IP - 192.168.4.1");
      Serial.println("Conection Failed!");
    }
  }

  Serial.print("Connected SSID - ");
  Serial.println(WiFi.SSID());
  Serial.print("IP Address is : ");
  Serial.println(WiFi.localIP());
  delay(3000);
}

void onDemandFirebaseConfig() {
  if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
    onDemand = true;
    firebaseStatus = "";
    setupServer();
  }
  delay(100);
}

void decodeData(String data) {

  Serial.println(data); //For Example=> {"value1":"\"on\"","value2":"\"on\"","value3":"\"off\"","value4":"\"off\""}
  /*
      goto website https://arduinojson.org/v6/assistant/#/step1
      select board
      choose input datatype
      and paste your JSON data
      it automatically generate your code
  */
  StaticJsonDocument<385> doc;
  DeserializationError error = deserializeJson(doc, data);

  if (error) {
    Serial.println(error.f_str());
    return;
  }

  backwardValue = doc["BACKWARD"];
  bhcValue = doc["BHC"];
  blcValue = doc["BLC"];
  forwardValue = doc["FORWARD"];
  gpsValue = doc["GPS"];
  headlightValue = doc["HL"];
  hornValue = doc["HORN"];
  leftValue = doc["LEFT"];
  rightValue = doc["RIGHT"];
  throttleValue = doc["THROTTLE"];
}
//////////////////////////////////////////////////////////////
boolean isFirebaseConnected() {
  Firebase.getString(firebaseData, "/ESP-CAR");
  if (firebaseData.stringData() != "") {
    return true;
  }
  else {
    return false;
  }
}
//////////////////////////////////////////////////////////////
void showLedStatus(uint8_t r, uint8_t g, uint8_t b ) {
  status_leds[STATUS_LED_POSI] = CRGB(r, g, b);;
  FastLED.show();
}
///////////////////////////////////////////////////////////////
void loading()
{
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;

  uint8_t sat8 = beatsin88( 87, 220, 250);
  uint8_t brightdepth = beatsin88( 341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 1, 3000);

  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis ;
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88( 400, 5, 9);
  uint16_t brightnesstheta16 = sPseudotime;

  for ( uint16_t i = 0 ; i < NUM_STATUS_LED; i++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;

    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);

    CRGB newcolor = CHSV( hue8, sat8, bri8);

    uint16_t pixelnumber = i;
    pixelnumber = (NUM_STATUS_LED - 1) - pixelnumber;

    nblend( status_leds[pixelnumber], newcolor, 64);
  }
}
///////////////////////////////////////////////////////////////
void gpsReadings() {
  if (gpsValue == 1) {

    if (neogps.available() > 0) {
      if (gps.encode(neogps.read())) {
        if (gps.location.isUpdated()) {
          latt = gps.location.lat();
          lngi = gps.location.lng();
        }
        if (gps.satellites.isUpdated()) {
          satNo = gps.satellites.value();
        }
        if (gps.speed.isUpdated()) {
          speedNow = gps.speed.mps();
        }
      }
    }
  }
}
///////////////////////////////////////////////////////////////
void speedDataUpload() {
  if (abs(speedNow - gpsSpeed) >= 1) {
    gpsSpeed = speedNow;
    Firebase.setFloat(firebaseData, "/ESP-CAR/SPEED", gpsSpeed);
  }
}
///////////////////////////////////////////////////////////////
void gpsAndBatDataUpload() {

  unsigned long currentMillis = millis();
  if (currentMillis - updatePrevMillis >= updateInterval) {
    updatePrevMillis = currentMillis;
    float averageVolt = total / indexNo;
    if (averageVolt >= blcValue && averageVolt <= bhcValue) {
      batteryValue = averageVolt;
    }
    if (averageVolt <= blcValue) {
      batteryValue = blcValue;
    }
    if (averageVolt >= bhcValue) {
      batteryValue = bhcValue - 0.1;
    }
    uploadAllData();
    Serial.print("Average Volt: ");
    Serial.println(averageVolt);
    total = 0;
    indexNo = 0;
  }
  int pwmValue = analogRead(BATTERY_PIN);
  float volt = pwmValue * 0.0041831;
  total += volt;
  indexNo++;
}
///////////////////////////////////////////////////////////////
void uploadAllData() {
  if (gpsValue == 1) {
    Serial.print("Lat:");
    Serial.print(latt, 6);
    Serial.print(" Lng:");
    Serial.print(lngi, 6);
    Serial.print(" Sat:");
    Serial.print(satNo);
    Serial.print(" m/s:");
    Serial.println(gpsSpeed, 2);
    Firebase.setFloat(firebaseData, "/ESP-CAR/LAT", latt);
    Firebase.setFloat(firebaseData, "/ESP-CAR/LNG", lngi);
    Firebase.setInt(firebaseData, "/ESP-CAR/SAT", satNo);
    Firebase.setFloat(firebaseData, "/ESP-CAR/SPEED", gpsSpeed);
  }
  Firebase.setFloat(firebaseData, "/ESP-CAR/BATTERY", batteryValue);
}
///////////////////////////////////////////////////////////////
void blowHorn() {
  if (hornValue == 1) {
    digitalWrite(HORN_PIN, HIGH);
  } else {
    digitalWrite(HORN_PIN, LOW);
  }
}
///////////////////////////////////////////////////////////////
void movedForward(uint8_t isActive, uint8_t rpm) {
  if (isActive == 1) {
    allOff();
    ledcWrite(pwmChanel, rpm);
    digitalWrite(RIGHT_FORWARD, HIGH);
    digitalWrite(LEFT_FORWARD, HIGH);
  }
}
void movedBackward(uint8_t isActive, uint8_t rpm) {
  if (isActive == 1) {
    allOff();
    ledcWrite(pwmChanel, rpm);
    digitalWrite(RIGHT_BACKWARD, HIGH);
    digitalWrite(LEFT_BACKWARD, HIGH);
  }
}
void turnLeft(uint8_t isActive, uint8_t rpm) {
  if (isActive == 1) {
    allOff();
    ledcWrite(pwmChanel, rpm);
    digitalWrite(RIGHT_FORWARD, HIGH);
    digitalWrite(LEFT_BACKWARD, HIGH);
  }
}
void turnRight(uint8_t isActive, uint8_t rpm) {
  if (isActive == 1) {
    allOff();
    ledcWrite(pwmChanel, rpm);
    digitalWrite(RIGHT_BACKWARD, HIGH);
    digitalWrite(LEFT_FORWARD, HIGH);
  }
}
void allOff() {
  digitalWrite(RIGHT_FORWARD, LOW);
  digitalWrite(RIGHT_BACKWARD, LOW);
  digitalWrite(LEFT_FORWARD, LOW);
  digitalWrite(LEFT_BACKWARD, LOW);
}
///////////////////////////////////////////////////////////////
void controllNavigation() {
  allOff();
  movedForward(forwardValue, throttleValue);
  movedBackward(backwardValue, throttleValue);
  turnLeft(leftValue, throttleValue);
  turnRight(rightValue, throttleValue);
}
///////////////////////////////////////////////////////////////
void loop1(void * parameter) {

  for (;;) {

    if (WiFi.status() == WL_CONNECTED && firebaseStatus == "ok") {
      showLedStatus(0, 255, 0);
      gpsReadings();
      controllNavigation();
      blowHorn();
    }
    if (onDemand == true) {
      loading();
      FastLED.show();
    }
    if (WiFi.status() != WL_CONNECTED) {
      showLedStatus(255, 0, 0);
      connectWiFi();
    }
  }
}
//////////////////////////////////////////////////////////////

void loop() {
  onDemand = false;
  onDemandFirebaseConfig();

  if (firebaseStatus == "ok") {
    if (backwardValue != 1 && forwardValue != 1 && leftValue != 1 && rightValue != 1) {
      gpsAndBatDataUpload();
      
    }
//    gpsAndBatDataUpload();
    speedDataUpload();
    Firebase.getString(firebaseData, "/ESP-CAR");
    decodeData(firebaseData.stringData());
  }
  else {
    Serial.println("firebase failed");
  }

  if (firebaseStatus != "ok") {
    if (WiFi.status() == WL_CONNECTED) {
      Firebase.getString(firebaseData, "/ESP-CAR");
      decodeData(firebaseData.stringData());
    }
  }
}
