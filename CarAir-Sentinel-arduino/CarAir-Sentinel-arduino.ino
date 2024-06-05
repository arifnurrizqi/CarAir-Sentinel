// Include the library
#include <WiFi.h>
#include <MQUnifiedsensor.h>

// MQ-135 Configuration
#define CHIP "ESP-32" // Wemos ESP-32 or other board, whatever have ESP32 
#define Voltage_Resolution 3.3 // 3V3 <- IMPORTANT. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define TYPE "MQ-135" //MQ135
#define ADC_Bit_Resolution 12 // For arduino ESP32
#define RatioMQ135CleanAir 3.6 //RS / R0 = 3.6 ppm  
//#define calibration_button 13 //Pin to calibrate your sensor

// Pinout Configuration
#define SENSOR_PIN 34 // Analog input ESP32
#define BUZZER_PIN 23 // Pin for buzzer
#define RED_PIN 17    // Pin LED RED RGB Module
#define GREEN_PIN 18  // Pin LED GREEN RGB Module
#define BLUE_PIN 19   // Pin LED BLUE RGB Module
#define RELAY_PIN 5   // Pin For Relay Module

//Declare Sensor
MQUnifiedsensor MQ135(CHIP, Voltage_Resolution, ADC_Bit_Resolution, SENSOR_PIN, TYPE);

const char* ssid = "ARNUR";
const char* password = "takonmama";

void setup() {
  // Init the serial port communication - to debug the library
  Serial.begin(115200); // Init serial port
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Set math model to calculate the PPM concentration and the value of constants
  MQ135.setRegressionMethod(1); // _PPM =  a*ratio^b
  
  /*****************************  MQ Init ********************************************/ 
  MQ135.init(); 

  /*****************************  MQ Calibration ********************************************/ 
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i++) {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  Serial.println("  done!");
  
  if(isinf(calcR0)) {
    Serial.println("Warning: Connection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    while(1);
  }
  if(calcR0 == 0) {
    Serial.println("Warning: Connection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    while(1);
  }

  Serial.println("** Values from MQ-135 **");
  Serial.println("|    CO    |    CO2    |");  
}

void loop() {
  MQ135.update(); // Update data, the arduino will read the voltage from the analog pin

  MQ135.setA(605.18); MQ135.setB(-3.937); // Configure the equation to calculate CO concentration value
  float CO = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration value
  float CO2 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  CO2 += 400; // Add 400 to the CO2 value to account for baseline atmospheric CO2

  Serial.print("|   "); Serial.print(CO); 
  Serial.print("   |   "); Serial.print(CO2); 
  Serial.println("   |"); 

  if ((CO < 50) || (CO2 < 1000)) {
    // Kadar CO < 50 atau CO2 < 1000
    digitalWrite(GREEN_PIN, LOW);
    digitalWrite(RED_PIN, HIGH);
    digitalWrite(BLUE_PIN, HIGH);
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("Status: Aman - LED Hijau Menyala");
  } 
  else if ((CO >= 51 && CO <= 200) || (CO2 >= 1001 && CO2 <= 2000)) {
    // Kadar CO > 51 < 200 atau CO2 > 1001 < 2000
    digitalWrite(RED_PIN, LOW);
    digitalWrite(BLUE_PIN, HIGH);
    digitalWrite(GREEN_PIN, LOW);
    tone(BUZZER_PIN, 1000); // Buzzer on
    Serial.println("Status: Waspada - LED Kuning Menyala, Buzzer Aktif");
    delay(5000);
    noTone(BUZZER_PIN); // Buzzer off
  } 
  else if ((CO >= 201 && CO <= 400) || (CO2 >= 2001 && CO2 <= 2500)) {
    // Kadar CO > 201 < 400 atau CO2 > 2001 < 2500
    digitalWrite(GREEN_PIN, HIGH);
    digitalWrite(BLUE_PIN, HIGH);
    digitalWrite(RED_PIN, LOW);
    tone(BUZZER_PIN, 1000); // Buzzer on
    Serial.println("Status: Bahaya - LED Merah Menyala, Buzzer Aktif");
    delay(2500);
    noTone(BUZZER_PIN); // Buzzer off
  } 
  else if (CO > 400 || CO2 > 2500) {
    // Kadar CO > 400 atau CO2 > 2500
    digitalWrite(GREEN_PIN, HIGH);
    digitalWrite(RED_PIN, LOW);
    digitalWrite(BLUE_PIN, LOW);
    tone(BUZZER_PIN, 1000); // Buzzer on 
    Serial.println("Status: Sangat Berbahaya - LED Ungu Menyala, Buzzer Aktif, Relay Aktif");
    delay(1000);
    noTone(BUZZER_PIN); // Buzzer off
    digitalWrite(RELAY_PIN, HIGH); // Relay on
    delay(10000); // Relay on for 10 seconds
    digitalWrite(RELAY_PIN, LOW); // Relay off
  }

  delay(500); 
}
