#include "def.h"      // pin definitions and helpers
#include <Arduino.h>
#include <FastLED.h>  // LED library
#include <HX711.h>    // load cell library
#include <Wire.h>     // I2C library



CRGB leds[1]; //LED strip object
HX711 scale;  //load cell object

int calibration_factor = 2500; //calibration factor for load cell 
bool run = false;


void setup() {

//initialize serial monitor
Serial.begin(115200);

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
  Serial.println(scale.read_average(100));
  delay(500);
}

scale.tare();

LogDebug("Scale tared, test reading: " + String(scale.get_units()));

delay(2000);

LogDebug("HX711 initialized");

long zero_factor = scale.read_average(100); //get the zero factor
Serial.println("Zero factor: " + String(zero_factor));




}

void loop() {
  if(run){
  Serial.print("Reading: ");
  Serial.print(scale.get_units(),1);
  Serial.print(" lbs"); //Change this to kg and re-adjust the calibration factor if you follow SI units like a sane person
  Serial.print(" calibration_factor: ");
  Serial.print(calibration_factor);
  Serial.println();
  };

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

    }
  }
}


