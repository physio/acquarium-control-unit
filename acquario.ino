#define MY_DEBUG
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include "secrets.h"

// Data wire is plugged into port D2 on the ESP8266
#define ONE_WIRE_BUS D3

ESP8266WiFiMulti WiFiMulti;

// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;

const int buttonPin = 13;
const int fanPin = 12;
const int heaterPin = 10;

bool fanActive = false;
bool heaterActive = false;

int buttonState = 0;

const float fanShold = 28;
float mediumTemp = fanShold - 1;
const float heaterShold = 26;

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
float tempSensor1, tempSensor2, tempSensor3;

uint8_t sensor1[8] = { 0x28, 0x61, 0x4D, 0x44, 0xD4, 0xE1, 0x3C, 0x2A };
uint8_t sensor2[8] = { 0x28, 0xB9, 0x1D, 0x57, 0x04, 0xE1, 0x3D, 0x2D };
uint8_t sensor3[8] = { 0x28, 0xF3, 0x2F, 0x57, 0x04, 0xE1, 0x3D, 0x90 };

// variable to hold device addresses
DeviceAddress Thermometer;
int deviceCount = 0;


time_t now;
time_t nowish = 1510592825;
unsigned long startTime = 0;         // Variabile per memorizzare l'istante iniziale
unsigned long elapsedTime = 0;       // Variabile per memorizzare il tempo trascorso
unsigned long duration = 1000 * 30;  // Durata del temporizzatore in millisecondi (30 secondi)

WiFiClientSecure net;

BearSSL::X509List cert(cacert);
BearSSL::X509List client_crt(client_cert);
BearSSL::PrivateKey key(privkey);

PubSubClient client(net);

#define AWS_IOT_PUBLISH_TOPIC "device/1/data"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp8266/sub"
unsigned long lastMillis = 0;
unsigned long previousMillis = 0;
const long interval = 30000;

#define TdsSensorPin A0
#define VREF 3.3   // analog reference voltage(Volt) of the ADC
#define SCOUNT 30  // sum of sample point

int analogBuffer[SCOUNT];  // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

void NTPConnect(void) {
  Serial.print("Setting time using SNTP");
  configTime(TIME_ZONE * 3600, 0 * 3600, "pool.ntp.org", "time.nist.gov");
  now = time(nullptr);
  while (now < nowish) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("done!");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: ");
  Serial.print(asctime(&timeinfo));
}

void messageReceived(char *topic, byte *payload, unsigned int length) {
  Serial.print("Received [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void connectWifi() {
  //WiFi.mode(WIFI_STA);
  Serial.println(String("Attempting to connect to SSID: ") + String(WIFI_SSID));
  WiFiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
  while (WiFiMulti.run() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
}

void connectAWS() {
  net.setTrustAnchors(&cert);
  net.setClientRSACert(&client_crt, &key);
  client.setServer(MQTT_HOST, 8883);
  client.setCallback(messageReceived);

  Serial.println("Connecting to AWS IOT");

  while (!client.connect(THINGNAME)) {
    Serial.print(".");
    delay(1000);
  }

  if (!client.connected()) {
    Serial.println("AWS IoT Timeout!");
    return;
  }
  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
  Serial.println("AWS IoT Connected!");
  lcd.clear();
}

void publishMessage() {
  StaticJsonDocument<200> doc;
  StaticJsonDocument<200> settings;

  settings["fanTemp"] = fanShold;
  settings["heaterTemp"] = heaterShold;


  doc["uptime"] = millis();
  doc["mediumTemp"] = mediumTemp; // printf("%.2f", mediumTemp);
  doc["temp1"] = tempSensor1;  // printf("%.2f", tempSensor1);
  doc["temp2"] = tempSensor2;
  doc["temp3"] = tempSensor3;
  doc["tds"] = tdsValue;
  doc["fanActive"] = fanActive;
  doc["heaterActive"] = heaterActive;
  doc["settings"] = settings;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);  // print to client

  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(heaterPin, OUTPUT);
  pinMode(TdsSensorPin, INPUT);

  Serial.begin(115200);
  delay(10);

  wifi_set_sleep_type(NONE_SLEEP_T);
  delay(2000);
  // Serial.setDebugOutput(1);

  // initialize LCD
  lcd.init();
  lcd.clear();

  // turn on LCD backlight
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Connecting wifi");
  lcd.setCursor(0, 1);
  lcd.print(WIFI_SSID);

  connectWifi();
  NTPConnect();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Conn Server...");

  connectAWS();

  lcd.clear();

  startTime = millis();  // Registra l'istante iniziale

  // Start up the library
  sensors.begin();

  // Uncomment to get the addresses on bus
  // // locate devices on the bus
  // Serial.println("Locating devices...");
  // Serial.print("Found ");
  // deviceCount = sensors.getDeviceCount();
  // Serial.print(deviceCount, DEC);
  // Serial.println(" devices.");
  // Serial.println("");

  // Serial.println("Printing addresses...");
  // for (int i = 0;  i < deviceCount;  i++)
  // {
  //   Serial.print("Sensor ");
  //   Serial.print(i+1);
  //   Serial.print(" : ");
  //   sensors.getAddress(Thermometer, i);
  //   printAddress(Thermometer);
  // }
}

void checkButton() {
  buttonState = digitalRead(buttonPin);
  if (buttonState) {
    displayOn();
  }
}

void manageFan(float value) {
  if (value > fanShold) {
    digitalWrite(fanPin, HIGH);
    fanActive = true;
  } else {
    fanActive = false;
    digitalWrite(fanPin, LOW);
  }
}

void manageHeater(float value) {
  if (value < heaterShold) {
    digitalWrite(heaterPin, HIGH);
    heaterActive = true;
  } else {
    heaterActive = false;
    digitalWrite(heaterPin, LOW);
  }
}

void displayOn() {
  startTime = millis();
  lcd.backlight();
}



void loop() {
  unsigned long currentTime = millis();
  elapsedTime = currentTime - startTime;
  checkButton();

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;

      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationCoefficient = 1.0 + 0.02 * (mediumTemp - 25.0);
      //temperature compensation
      float compensationVoltage = averageVoltage / compensationCoefficient;

      //convert voltage value to tds value
      tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;

      Serial.print("voltage:");
      Serial.print(averageVoltage,2);
      Serial.print("V   ");
      //Serial.print("TDS Value:");
      //Serial.print(tdsValue, 0);
      //Serial.println("ppm");
    }
  }

  sensors.requestTemperatures();

  tempSensor1 = sensors.getTempC(sensor1);
  tempSensor2 = sensors.getTempC(sensor2);
  tempSensor3 = sensors.getTempC(sensor3);

  mediumTemp = (tempSensor2 + tempSensor3) / 2;


  now = time(nullptr);


  if (!client.connected()) {
    connectAWS();
  } else {
    client.loop();
    if (millis() - lastMillis > interval) {
      lastMillis = millis();
      Serial.print("Medium Temp: ");
      Serial.println(mediumTemp);
      publishMessage();
    }
  }

  manageFan(mediumTemp);
  manageHeater(mediumTemp);

  checkButton();

  if (elapsedTime >= duration) {
    lcd.setBacklight(0);

    // Reinizializza il temporizzatore
    startTime = millis();
  }

  // set cursor to first column, first row
  lcd.setCursor(0, 0);
  // print message
  lcd.print(tempSensor1);
  lcd.print("   ");
  lcd.print(tdsValue);

  delay(100);
  lcd.clear();

  // set cursor to first column, second row
  lcd.setCursor(0, 1);
  lcd.print("");
  lcd.print(tempSensor2);
  lcd.print("    ");
  lcd.print(tempSensor3);

  checkButton();
}

// Uncomment to get the addresses
// void printAddress(DeviceAddress deviceAddress) {
//   for (uint8_t i = 0; i < 8; i++) {
//     Serial.print("0x");
//     if (deviceAddress[i] < 0x10) Serial.print("0");
//     Serial.print(deviceAddress[i], HEX);
//     if (i < 7) Serial.print(", ");
//   }
//   Serial.println("");
// }
