#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int brakeLED = 4; //MOSFET gate connected to pin D4

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_accel;
int mpu_rdy = 0; //emergency stop signal disabled by default

//offsets in m/s^2 determined thru sensor observation using adafruit MPU6050 library.
float offsetx = -0.26;
float offsety = -0.27;
float offsetz = 0.81;

//variables for flashing intervals to avoid using delay()
int ledState = LOW;
unsigned long previousMillis = 0; //stores previous update time
const int interval = 100;         //stores flashing interval; convert to Hz with 1000/(2*interval)
                                  //100ms interval for 5 Hz is used; according to ECE regulations flashing must be at 4+-1Hz

void setup()
{
  pinMode(brakeLED, OUTPUT);
  //initialization of accelerometer

  if (mpu.begin()) //will flash rapidly on success
  {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    mpu_accel = mpu.getAccelerometerSensor();
    mpu_rdy = 1; //enables emergency stop signal

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

  if (!mpu_rdy) // will flash slowly on failure. Brake light still functions, but emergency stop signal disabled.
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
float getaccel() //creates average of last 4 accelerations to even out noise
{
  sensors_event_t accel;
  float sum = 0;
  for (int i = 0; i < 4; i++)
  {
    mpu_accel->getEvent(&accel);
    sum += accel.acceleration.z + offsetz;
  }
  return sum / 4;
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

  if (mpu_rdy)
  { //flashing only executed if mpu works

    if (getaccel() >= 7 || getaccel() <= -7)
    { //flashing begins if acceleration exceeds 7m/s^2 and cuntinues until it falls below 4 m/s^2
      while (getaccel() >= 4 || getaccel() <= -4)
      {
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