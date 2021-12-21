#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int oemBrake = 5;
const int brakeLED = 4;

Adafruit_MPU6050 mpu;

void setup() {
  //initialization
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(brakeLED, OUTPUT);
  pinMode(oemBrake, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(brakeLED, HIGH);
  delay(1000);
  digitalWrite(brakeLED, LOW);
  delay(500);
}