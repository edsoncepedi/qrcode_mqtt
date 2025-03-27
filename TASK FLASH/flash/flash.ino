#include "Arduino.h"
#include <Wire.h>
// #include "esp_camera.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_VL53L0X.h"

// -----------------I2C-----------------
#define SDA_PIN 15 // SDA Connected to GPIO 15
#define SCL_PIN 13 // SCL Connected to GPIO 13
#define LED_Board 4 //LED Flash

// VL53L0X (Using I2C)
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup(){
  pinMode(LED_Board, OUTPUT);
  digitalWrite(LED_Board, LOW);
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1)
      ;
  }
  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
}

void loop(){
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  // lox.getRangingMeasurement(&measure, false);
  if (measure.RangeStatus != 4) {
    Serial.print("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
    digitalWrite(LED_Board, LOW);
  }
  delay(100);
}