#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int brakeLED = 4; //MOSFET gate connected to pin D4

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_accel;
int mpu_rdy = 0;

//variables for flashing intervals to avoid using delay()
int ledState = LOW;
unsigned long previousMillis = 0; //stores previous update time
const int interval = 100;        //stores flashing interval; convert to Hz with 1000/(2*interval)
                                  //100ms interval for 5 Hz is used; according to ECE regulations flashing must be at 4+-1Hz

void setup()
{
  pinMode(brakeLED, OUTPUT);
  //initialization of accelerometer
  if (mpu.begin())
  {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    mpu_accel = mpu.getAccelerometerSensor();
    mpu_accel->printSensorDetails();
    mpu_rdy = 1;

    digitalWrite(brakeLED, HIGH);
    delay(100);
    digitalWrite(brakeLED, LOW);
    delay(100);
    digitalWrite(brakeLED, HIGH);
    delay(100);
    digitalWrite(brakeLED, LOW);
    delay(100);
    digitalWrite(brakeLED, HIGH);
    delay(100);
    digitalWrite(brakeLED, LOW);
  }

  //initializing pin modes

  if (!mpu_rdy)
  {
    digitalWrite(brakeLED, HIGH);
    delay(500);
    digitalWrite(brakeLED, LOW);
    delay(500);
    digitalWrite(brakeLED, HIGH);
    delay(500);
    digitalWrite(brakeLED, LOW);
    delay(500);
    digitalWrite(brakeLED, HIGH);
    delay(500);
    digitalWrite(brakeLED, LOW);
  }
}

void loop()
{ /*
The following handles acceleration detection using MPU 6050.
To avoid complex gravity adjusted calculations, the MPU must be mounted with an axis pointing
in travel direction. In this case z is assumed to point forwards / backwards.
Code must be adjusted for different mounting orientations!
*/

  digitalWrite(brakeLED, HIGH); //ensures brake light is always on if not flashing and brakes applied
  ledState = HIGH;
  sensors_event_t accel;
  if (mpu_rdy)
  { //flashing only executed if mpu works
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
  }
}