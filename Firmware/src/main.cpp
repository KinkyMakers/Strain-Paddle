#include "def.h"      // pin definitions and helpers
#include <Arduino.h>
#include <FastLED.h>  // LED library
#include <HX711.h>    // load cell library
#include <Wire.h>     // I2C library for accelerometer



CRGB leds[1]; //LED strip object
HX711 scale;  //load cell object

int calibration_factor = 2500; //calibration factor for load cell 
bool run = false;

void led_from_strain(float strain)
  {

    //the LED will change color based on the load cell reading
    //if the load cell is greater than 1, the LED will be red
    //if the load cell is less than 1, the LED will be green fading to yellow to red
    if(strain > 1){
        leds[0] = CRGB::Red;
        FastLED.show();
        }
      else
      {
        leds[0] = CHSV(85 - strain * 85, 255, 255);
        FastLED.show();
        };

  }

int read_accel_z()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);
  int16_t AcX = Wire.read() << 8 | Wire.read();
  int16_t AcY = Wire.read() << 8 | Wire.read();
  int16_t AcZ = Wire.read() << 8 | Wire.read();
  return AcZ;
}

#define LIS2DW12TR_ADDR 0x18 // Replace with the actual address for LIS2DW12TR
#define CTRL_REG1 0x20       // Example register address, replace with actual
#define POWER_ACTIVE_MODE 0x01 // Replace with the actual value to set active mode

void wakeUpAccelerometer() {
  Wire.beginTransmission(LIS2DW12TR_ADDR);
  Wire.write(CTRL_REG1);
  Wire.write(POWER_ACTIVE_MODE);
  Wire.endTransmission(true);
}


void i2c_scanner() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning for I2C devices...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.println("Scanning complete.");
  }
}

void setup() {

// so the hx711 doesn't like the ESP32 at full speed, so we need to slow it down
setCpuFrequencyMhz(80); //set the CPU frequency to 80MHz

//initialize serial monitor
Serial.begin(115200);

//initialize wire library
Wire.begin();

//wake up the accelerometer
wakeUpAccelerometer();

delay(150);

//initialize I2C scanner
i2c_scanner();

//initialize the accelerometer
Wire.beginTransmission(0x68);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);


//initialize LED strip
FastLED.addLeds<WS2812B, led_pixel, GRB>(leds, 1);
FastLED.setBrightness(15);
FastLED.clear();
leds[0] = CRGB::Green;
FastLED.show();


//initialize the hx711

scale.begin(s_dout, s_clock);
scale.set_gain(64);

delay(1000);

for(int i = 0; i < 10; i++)
{
  Serial.println(scale.read_average(10));
  delay(500);
}

scale.tare();

LogDebug("Scale tared, test reading: " + String(scale.get_units()));

delay(2000);

LogDebug("HX711 initialized");

long zero_factor = scale.read_average(10); //get the zero factor
Serial.println("Zero factor: " + String(zero_factor));




}

void loop() {

  if(run){

  //read the load cell and accelerometer
  float current_reading = (abs(scale.get_units(5))/1000);


  Serial.print("Strain Guage: ");
  Serial.println(current_reading);
  Serial.print("Accelerometer: ");
  Serial.println(read_accel_z());

  //change the LED color based on the load cell reading
  led_from_strain(current_reading);

  }

  if(Serial.available())
  {
    char temp = Serial.read();
    switch(temp)
    {
      case 't':
        LogStatus("Tare");
        scale.tare();
        break;
      case 'c':
        LogStatus("Calibrate");
        calibration_factor = Serial.parseInt();
        break;
      case 'r':
        LogDebug("Run: " + String(run));
        run = !run;
        break;
      case 'a':
        LogStatus("Increase Calibration Factor");
        calibration_factor += 1000;
        break;
      case 'z':
        LogStatus("Decrease Calibration Factor");
        calibration_factor -= 1000;
        break;
      case 's':
        i2c_scanner();   
        break;  

    }
  }
}




