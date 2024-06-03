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
#define SENSOR_PIN 35 // Analog input ESP32
#define BUZZER_PIN 16 // Pin for buzzer
#define RED_PIN 17    // Pin LED RED RGB Module
#define GREEN_PIN 18  // Pin LED GREEN RGB Module
#define BLUE_PIN 19   // Pin LED BLUE RGB Module
#define RELAY_PIN 5   // Pin For Relay Module

//Declare Sensor
MQUnifiedsensor MQ135(CHIP, Voltage_Resolution, ADC_Bit_Resolution, SENSOR_PIN, TYPE);
BUZZER_PIN 
const char* ssid     = "ARNUR";
const char* password = "tanyamama";

void setup() {
  //Init the serial port communication - to debug the library
  Serial.begin(115200); //Init serial port
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN_PIN, OUTPUT);
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

  //Set math model to calculate the PPM concentration and the value of constants
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  
  /*****************************  MQ Init ********************************************/ 
  //Remarks: Configure the pin of arduino as input.
  /************************************************************************************/ 
  MQ135.init(); 
  /* 
    //If the RL value is different from 10K please assign your RL value with the following method:
    MQ135.setRL(10);
  */
  /*****************************  MQ CAlibration ********************************************/ 
  // Explanation: 
  // In this routine the sensor will measure the resistance of the sensor supposedly before being pre-heated
  // and on clean air (Calibration conditions), setting up R0 value.
  // We recomend executing this routine only on setup in laboratory conditions.
  // This routine does not need to be executed on each restart, you can load your R0 value from eeprom.
  // Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
  /*****************************  MQ CAlibration ********************************************/ 
  Serial.println("** Values from MQ-135 ****");
  Serial.println("|    CO    |    CO2    |");  
}

void loop() {
  MQ135.update(); // Update data, the arduino will read the voltage from the analog pin

  MQ135.setA(605.18); MQ135.setB(-3.937); // Configure the equation to calculate CO concentration value
  float CO = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration value
  float CO2 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  Serial.print("|   "); Serial.print(CO); 
  // Note: 400 Offset for CO2 source: https://github.com/miguel5612/MQSensorsLib/issues/29
  /*
  Motivation:
  We have added 400 PPM because when the library is calibrated it assumes the current state of the
  air as 0 PPM, and it is considered today that the CO2 present in the atmosphere is around 400 PPM.
  https://www.lavanguardia.com/natural/20190514/462242832581/concentracion-dioxido-cabono-co2-atmosfera-bate-record-historia-humanidad.html
  */
  Serial.print("   |   "); Serial.print(CO2 + 400); 
  Serial.println("   |"); 
  /*
    Exponential regression:
  GAS      | a      | b
  CO       | 605.18 | -3.937  
  CO2      | 110.47 | -2.862
  */

  delay(500); //Sampling frequency

  // Decission
  // Kadar CO < 50 atau CO2 < 1000, LED RGB menyala warna hijau;
  // Kadar CO > 51 < 200 atau CO2 > 1001 < 2000, LED RGB menyala Berwarna Kuning, Buzzer menyala dgn delay 5000;
  // Kadar CO > 201 < 400 atau CO2 > 2001 < 2500, LED RGB menyala Berwarna Merah, Buzzer menyala dgn delay 2500;
  // Kadar CO > 400 atau CO2 > 2500, LED RGB menyala Berwarna Ungu, Buzzer menyala dgn delay 1000, Notif Telegram dan menyalakan relay dengan durasi tertentu;
}
