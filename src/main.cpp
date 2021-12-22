#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int oemBrake = 5; //stepped down signal of original brake light connected to pin D5
const int brakeLED = 4; //MOSFET gate connected to pin D4

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_accel;

//variables for flashing intervals to avoid using delay()
int ledState = LOW;
unsigned long previousMillis = 0; //stores previous update time
const long interval = 100;        //stores flashing interval; convert to Hz with 1000/(2*interval)
                                  //100ms interval for 5 Hz is used; according to ECE regulations flashing must be at 4+-1Hz

void setup()
{
  //initialization of accelerometer
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  mpu_accel = mpu.getAccelerometerSensor();
  mpu_accel->printSensorDetails();

  //initializing pin modes
  pinMode(brakeLED, OUTPUT);
  pinMode(oemBrake, INPUT);
}

void loop()
{

  sensors_event_t accel;

  /*
The following handles acceleration detection using MPU 6050.
To avoid complex gravity adjusted calculations, the MPU must be mounted with an axis pointing
in travel direction. In this case z is assumed to point forwards / backwards.
Code must be adjusted for different mounting orientations!
*/

  while (digitalRead(oemBrake))
  {
    mpu_accel->getEvent(&accel); //continuously read current acceleleration
    if (accel.acceleration.z >= 5 || accel.acceleration.z <= -5)
    { //flashing begins if acceleration exceeds 5m/s^2 and cuntinues until it fall below 2 m/s^2
      while (accel.acceleration.z >= 2 || accel.acceleration.z <= -2)
      {
        mpu_accel->getEvent(&accel); //continuously read current acceleleration
        unsigned long currentMillis = millis();

        if (currentMillis - previousMillis >= interval) //could break from rollover after 50 days
        {
          // save the last time you blinked the LED
          previousMillis = currentMillis;

          // if the LED is off turn it on and vice-versa:
          if (ledState == LOW)
          {
            ledState = HIGH;
          }
          else
          {
            ledState = LOW;
          }
          digitalWrite(brakeLED, ledState); //turn LED on or off depending on state
        }
      }
    }
    digitalWrite(brakeLED, HIGH); //ensures brake light is always on if not flashing and brakes applied
    ledState = HIGH;
  }
  digitalWrite(brakeLED, LOW); //ensures brake light turns off with main light
  ledState = LOW;              //set state counter to low as well
}