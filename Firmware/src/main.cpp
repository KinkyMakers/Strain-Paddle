#include "def.h"      // pin definitions and helpers
#include <Arduino.h>
#include <FastLED.h>  // LED library
#include <HX711.h>    // load cell library
#include <Wire.h>     // I2C library
#include <EEPROM.h> // include for reading/write to eeprom
#include <esp_now.h> // include for esp now
#include <WiFi.h> // include for wifi

// EEProm address for calibration factor
#define CALIBRATION_FACTOR_ADDRESS 0

CRGB leds[1]; //LED strip object
HX711 scale;  //load cell object

int calibration_factor = 250; //calibration factor for load cell 
bool run = false;


/////////////////////////////////////////////
// ESP NOW
/////////////////////////////////////////////
// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  float peakValue;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


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

  /////////////////////////////////////////////////
  // PEAK DETECTION
  /////////////////////////////////////////////////

  float lastPEAK = 0.0; // Global variable to store the last peak
  const int maxReadings = 20; // Maximum number of readings for noise floor calculation
  float readings[maxReadings]; // Array to store recent readings
  int readingsIndex = 0; // Current index in the readings array
  bool readingsFilled = false; // Flag to check if the readings array has been filled

// Function to calculate the average of the readings array
float calculateAverage() {
    float sum = 0.0;
    int count = readingsFilled ? maxReadings : readingsIndex;
    for (int i = 0; i < count; i++) {
        sum += readings[i];
    }
    return count > 0 ? sum / count : 0.0;
}

  // Function to update the last peak value
  void updateLastPeak(float currentReading) {
      // Insert current reading into the circular buffer
      readings[readingsIndex++] = currentReading;
      if (readingsIndex >= maxReadings) {
          readingsIndex = 0;
          readingsFilled = true;
      }

      // Calculate noise floor as the average of recent readings
      float noiseFloor = calculateAverage();

      // Set a dynamic threshold, adjust the multiplier as needed
      float dynamicThreshold = noiseFloor * 1.5; // Example: 50% above the noise floor

      // Check if current reading is a peak
      if (currentReading > dynamicThreshold) {
          lastPEAK = currentReading; // Update last peak value
      }
  }



void setup() {

//setup the eeprom
EEPROM.begin(512);

//read the calibration factor from eeprom
calibration_factor = EEPROM.read(CALIBRATION_FACTOR_ADDRESS);

//if calibration factor is 0, set it to 25
if(calibration_factor == 0)
{
  calibration_factor = 25;
  EEPROM.write(CALIBRATION_FACTOR_ADDRESS, calibration_factor);
}

// so the hx711 doesn't like the ESP32 at full speed, so we need to slow it down
setCpuFrequencyMhz(80); //set the CPU frequency to 80MHz

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
  Serial.println(scale.read_average(10));
  delay(100);
}

scale.tare();

LogDebug("Scale tared, test reading: " + String(scale.get_units()));

delay(500);

LogDebug("HX711 initialized");

long zero_factor = scale.read_average(10); //get the zero factor
Serial.println("Zero factor: " + String(zero_factor));

run = true; //start the loop




// ESPnow setup
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  Serial.println("ESP-NOW setup complete");

}

void loop() {

  if(run){

  //read the load cell and accelerometer
  float current_reading = (abs(scale.get_units(2))/calibration_factor);
  updateLastPeak(current_reading);
  

  Serial.print("Strain Guage: ");
  Serial.print(current_reading);
  Serial.print(" Peak: ");
  Serial.println(lastPEAK);

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
        LogStatus("Calibration Factor: " + String(calibration_factor));
        break;
      case 'r':
        LogDebug("Run: " + String(run));
        run = !run;
        break;
      case 'a':
        LogStatus("Increase Calibration Factor");
        calibration_factor += 1;
        //write the calibration factor to eeprom
        EEPROM.write(CALIBRATION_FACTOR_ADDRESS, calibration_factor);
        EEPROM.commit();
        break;
      case 'z':
        LogStatus("Decrease Calibration Factor");
        calibration_factor -= 1;
        //write the calibration factor to eeprom
        EEPROM.write(CALIBRATION_FACTOR_ADDRESS, calibration_factor);
        EEPROM.commit();
        break;


    }
  }
}




