// Blynk Configuration
#define BLYNK_TEMPLATE_ID  "TMPL6N6ZN0S9E"
#define BLYNK_TEMPLATE_NAME "CAR AIR SENTINEL"
#define BLYNK_AUTH_TOKEN    "XibW_d_zhJunuX90XhGidobavWs27mIP"

// Include the libraries
#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          // https://github.com/bblanchon/ArduinoJson
#include <MQUnifiedsensor.h>       // Library for sensor MQ Series 
#include <BlynkSimpleEsp32.h>      // Library for blynk IoT

// MQ-135 Configuration
#define CHIP                  "ESP-32" // Wemos ESP-32 or other board, whatever have ESP32 
#define TYPE                  "MQ-135" // MQ135
#define Voltage_Resolution    3.3 // 3V3 <- IMPORTANT. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define ADC_Bit_Resolution    12 // For arduino ESP32
#define RatioMQ135CleanAir    3.6 // RS / R0 = 3.6 ppm  

// Pinout Configuration
#define SENSOR_PIN 34 // Analog input ESP32
#define BUZZER_PIN 23 // Pin for buzzer
#define RED_PIN 17    // Pin LED RED RGB Module
#define GREEN_PIN 18  // Pin LED GREEN RGB Module
#define BLUE_PIN 19   // Pin LED BLUE RGB Module
#define RELAY_PIN 5   // Pin For Relay Module

// Declare Sensor
MQUnifiedsensor MQ135(CHIP, Voltage_Resolution, ADC_Bit_Resolution, SENSOR_PIN, TYPE);

char blynkTemplateId[40];
char blynkTemplateName[40];
char blynkAuthToken[34];

// Configuration for WiFi-Manager
WiFiManager wm;
WiFiManagerParameter custom_blynk_template_id("template_id", "Blynk Template ID", BLYNK_TEMPLATE_ID, 40);
WiFiManagerParameter custom_blynk_template_name("template_name", "Blynk Template Name", BLYNK_TEMPLATE_NAME, 40);
WiFiManagerParameter custom_blynk_auth_token("auth_token", "Blynk Auth Token", BLYNK_AUTH_TOKEN, 34);

// Blynk setup
BlynkTimer timer;

void sendToBlynk(float CO, float CO2, String status, int relayStatus) {
  Blynk.virtualWrite(V0, CO2);          // Send CO2 data to Blynk
  Blynk.virtualWrite(V1, CO);           // Send CO data to Blynk
  Blynk.virtualWrite(V2, status);       // Send status to Blynk
  Blynk.virtualWrite(V3, relayStatus);  // Send relay status to Blynk
}

// Timer variables
unsigned long previousMillis = 0;
unsigned long previousMillisRelay = 0;
bool relayOn = false;   // Relay Status

void setup() {
  // Init the serial port communication - to debug the library
  Serial.begin(115200); // Init serial port

  // Setting pinMode ESP32
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);

  wm.addParameter(&custom_blynk_template_id);
  wm.addParameter(&custom_blynk_template_name);
  wm.addParameter(&custom_blynk_auth_token);

  // Set custom header for the WiFiManager portal
  wm.setTitle("Car Air Sentinel");

  // AutoConnect and timeout in seconds
  wm.setConfigPortalTimeout(180);

  if (!wm.autoConnect("CAS config", "1234567890")) {
    Serial.println("Failed to connect and hit timeout");
    ESP.restart();
  }

  Serial.println("Connected to WiFi!");

  strcpy(blynkTemplateId, custom_blynk_template_id.getValue());
  strcpy(blynkTemplateName, custom_blynk_template_name.getValue());
  strcpy(blynkAuthToken, custom_blynk_auth_token.getValue());

  Serial.println("Blynk Template ID: " + String(blynkTemplateId));
  Serial.println("Blynk Template Name: " + String(blynkTemplateName));
  Serial.println("Blynk Auth Token: " + String(blynkAuthToken));

  Blynk.config(blynkAuthToken);
  Blynk.connect();

  // Set math model to calculate the PPM concentration and the value of constants
  MQ135.setRegressionMethod(1); // _PPM =  a*ratio^b
  
  /*****************************  MQ Init ********************************************/ 
  MQ135.init(); 

  /*****************************  MQ Calibration ********************************************/ 
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0 / 10);
  Serial.println("  done!");
  
  if (isinf(calcR0)) {
    Serial.println("Warning: Connection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    while (1);
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Connection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    while (1);
  }

  Serial.println("** Values from MQ-135 **");
  Serial.println("|    CO    |    CO2    |");  

  // Setup a function to be called every second
  timer.setInterval(1000L, []() {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin

    MQ135.setA(605.18); MQ135.setB(-3.937); // Configure the equation to calculate CO concentration value
    float CO = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

    MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration value
    float CO2 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

    CO2 += 400; // Add 400 to the CO2 value to account for baseline atmospheric CO2

    Serial.print("|   "); Serial.print(CO);
    Serial.print("   |   "); Serial.print(CO2);
    Serial.println("   |");

    String status = "";
    int relayStatus = 0;

    unsigned long currentMillis = millis();

    if ((CO < 50) || (CO2 < 1000)) {
      // Kadar CO < 50 atau CO2 < 1000
      digitalWrite(GREEN_PIN, LOW);
      digitalWrite(RED_PIN, HIGH);
      digitalWrite(BLUE_PIN, HIGH);
      digitalWrite(BUZZER_PIN, LOW);
      status = "Aman";
      Serial.println("Status: Aman - LED Hijau Menyala");

      Blynk.logEvent("gas_safe_alarm", "AMAN! Kadar CO atau CO2 RENDAH");
    }
    else if ((CO >= 51 && CO <= 200) || (CO2 >= 1001 && CO2 <= 2000)) {
      // Kadar CO > 51 < 200 atau CO2 > 1001 < 2000
      if (currentMillis - previousMillis >= 5000) {
        previousMillis = currentMillis;
        digitalWrite(RED_PIN, LOW);
        digitalWrite(BLUE_PIN, HIGH);
        digitalWrite(GREEN_PIN, LOW);
        digitalWrite(BUZZER_PIN, HIGH); // Buzzer on 
        status = "Waspada";
        Serial.println("Status: Waspada - LED Kuning Menyala, Buzzer Aktif");

        Blynk.logEvent("gas_alarm", "WASPADA! Kadar CO atau CO2 cukup tinggi");
      }
    }
    else if ((CO >= 201 && CO <= 400) || (CO2 >= 2001 && CO2 <= 2500)) {
      // Kadar CO > 201 < 400 atau CO2 > 2001 < 2500
      if (currentMillis - previousMillis >= 2500) {
        previousMillis = currentMillis;
        digitalWrite(GREEN_PIN, HIGH);
        digitalWrite(BLUE_PIN, HIGH);
        digitalWrite(RED_PIN, LOW);
        digitalWrite(BUZZER_PIN, HIGH); // Buzzer on 
        status = "Bahaya";
        Serial.println("Status: Bahaya - LED Merah Menyala, Buzzer Aktif");

        Blynk.logEvent("gas_alarm", "BAHAYA! Kadar CO atau CO2 tinggi");
      }
    }
    else if (CO > 400 || CO2 > 2500) {
      // Kadar CO > 400 atau CO2 > 2500
      if (currentMillis - previousMillis >= 1000) {
        previousMillis = currentMillis;
        digitalWrite(GREEN_PIN, HIGH);
        digitalWrite(RED_PIN, LOW);
        digitalWrite(BLUE_PIN, LOW);
        digitalWrite(BUZZER_PIN, HIGH); // Buzzer on 
        status = "Sangat Berbahaya";
        Serial.println("Status: Sangat Berbahaya - LED Ungu Menyala, Buzzer Aktif, Relay Aktif");
        digitalWrite(RELAY_PIN, HIGH); // Relay on
        relayOn = true;
        relayStatus = 1; // data for blynk
        previousMillisRelay = currentMillis;

        Blynk.logEvent("gas_alarm", "SANGAT BERBAHAYA! Kadar CO atau CO2 sangat tinggi");
      }
    }

    // Turn off relay after 10 seconds
    if (relayOn && (currentMillis - previousMillisRelay >= 10000)) {
      digitalWrite(RELAY_PIN, LOW); // Relay off
      relayOn = false;
      relayStatus = 0; // data for blynk
    }

    // Send data to Blynk
    sendToBlynk(CO, CO2, status, relayStatus);
  });
}

void loop() {
  Blynk.run();
  timer.run();
}
