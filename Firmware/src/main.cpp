
//   _________ __                .__      __________             .___  .___.__          
//  /   _____//  |_____________  |__| ____\______   \_____     __| _/__| _/|  |   ____  
//  \_____  \\   __\_  __ \__  \ |  |/    \|     ___/\__  \   / __ |/ __ | |  | _/ __ \ 
//  /        \|  |  |  | \// __ \|  |   |  \    |     / __ \_/ /_/ / /_/ | |  |_\  ___/ 
// /_______  /|__|  |__|  (____  /__|___|  /____|    (____  /\____ \____ | |____/\___  >
//         \/                  \/        \/               \/      \/    \/      2024 \/ 
//
//  By: KinkMakers.core()
//  Version 1.0
//
//  See Licenses.txt file for details of included items

#include "def.h"      // pin definitions and helpers
#include <Arduino.h>  // Arduino library, some functions just make it easier to have this included
#include <FastLED.h>  // LED library
#include <HX711.h>    // load cell library
#include <Wire.h>     // I2C library
#include <EEPROM.h>   // include for reading/write to eeprom
#include <esp_now.h>  // include for esp now
#include <WiFi.h>     // include for wifi

/////////////////////////////////////////////
// Setup Your Device here
/////////////////////////////////////////////

#define MyDeviceName ""                 // Name of the device
#define autoStart true                 // Start the serial dump automatically
#define CALIBRATION_FACTOR_ADDRESS 0    // Address in EEPROM to store the calibration factor

CRGB leds[1]; //LED strip object
HX711 scale;  //load cell object

int calibration_factor = 250; //calibration factor for load cell 
bool run = false;             // run will be flagged in Setup() after the first tare if autoStart is true
long espNowSendTime = 0;      // time to send the next ESP_NOW message
long peakTime = 0;            // time to reset the peak value

/////////////////////////////////////////////
// After Now Function
/////////////////////////////////////////////

// if the current time in millis() is greater than the input return true
bool afterNow(unsigned long time){
  return millis() > time;
}

/////////////////////////////////////////////
// ESP NOW
/////////////////////////////////////////////

  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // using broadcast address to send the data to all peers

  // Data Structure to send data
  typedef struct struct_message {
    float peakValue;
    uint8_t macAddress[6];
    char deviceName[20];
  } struct_message;

  // Create a struct_message called myData
  struct_message myData;
  esp_now_peer_info_t peerInfo;

  // callback when data is sent
  void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // only log the send status if it failed
    if (status != ESP_NOW_SEND_SUCCESS) {
      LogError("Last Packet Send Status:\t" + String(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail"));
    }
  }

/////////////////////////////////////////////////
// Load Cell Functions
/////////////////////////////////////////////////

float readLoadCell() {
  float load = scale.get_units(5); // Get average of 5 readings
  return load;
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

      // peak should reset after 5 seconds
      if (afterNow(peakTime)) {
          lastPEAK = 0.0;
          peakTime = millis() + 5000;
      }
  }


  /////////////////////////////////////////////////
  // i2c Scanner
  /////////////////////////////////////////////////


  void i2c_scanner() {
    byte error, address;
    int nDevices;

    LogDebug("Scanning for I2C devices...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
      Wire.beginTransmission(address);
      error = Wire.endTransmission();

      if (error == 0) {
        LogDebug("I2C device found at address 0x");
        if (address < 16) {
          Serial.print("0");
        }
        Serial.print(address, HEX);
        Serial.println(" !");
        nDevices++;
      }
      else if (error == 4) {
        LogError("Unknown error at address 0x");
        if (address < 16) {
          Serial.print("0");
        }
        Serial.println(address, HEX);
      }
    }
    if (nDevices == 0) {
      LogError("No I2C devices found.");
    } else {
      LogDebug("Scanning complete.");
    }
  }


  /////////////////////////////////////////////////
  // read from accelerometer
  /////////////////////////////////////////////////

  void readAccelData(int16_t *x, int16_t *y, int16_t *z) {
    // Read STATUS register to check for data ready
    Wire.beginTransmission(g_ADDR);
    Wire.write(0x27); // STATUS register
    if (Wire.endTransmission() != 0 || Wire.requestFrom(g_ADDR, 1) != 1) {
      Serial.println("Error reading STATUS register.");
      return;
    }
  
    byte status = Wire.read();
    
    if (!(status & 0x01)) { // Check if DRDY (data-ready) bit is set
      Serial.println("Data not ready.");
      return;
    }
    
    // If DRDY bit is set, read accelerometer data
    Wire.beginTransmission(g_ADDR);
    Wire.write(0x28 | 0x80); // Set auto-increment and start at OUT_X_L
    if (Wire.endTransmission(false) != 0) {
      Serial.println("Error setting auto-increment address.");
      return;
    }

    Wire.requestFrom(g_ADDR, (uint8_t)6); 
    while (Wire.available() < 6);
    
    *x = Wire.read() | ((int16_t)Wire.read() << 8);
    *y = Wire.read() | ((int16_t)Wire.read() << 8);
    *z = Wire.read() | ((int16_t)Wire.read() << 8);
  
}

void setupAccelerometer() {
  Wire.beginTransmission(g_ADDR);
  Wire.write(0x20); // CTRL1 register address
  // Assuming you want high-performance mode at 100 Hz:
  Wire.write(0b01010100); // This is 0x4A in hexadecimal
  Wire.endTransmission();
}

void readStatusRegister() {
  Wire.beginTransmission(g_ADDR);
  Wire.write(0x27);                               // STATUS register address
  if (Wire.endTransmission() != 0) {
    Serial.println("Error transmitting to device.");
    return;
  }
  
  Wire.requestFrom(g_ADDR, 1);
  if (Wire.available()) {
    byte status = Wire.read();
    Serial.println("STATUS Register:");
    Serial.print("FIFO threshold status: ");
    Serial.println((status & 0x80) ? "FIFO above threshold" : "FIFO below threshold");
    Serial.print("Wakeup event detected: ");
    Serial.println((status & 0x40) ? "Yes" : "No");
    Serial.print("Sleep event detected: ");
    Serial.println((status & 0x20) ? "Yes" : "No");
    Serial.print("Double-tap event detected: ");
    Serial.println((status & 0x10) ? "Yes" : "No");
    Serial.print("Single-tap event detected: ");
    Serial.println((status & 0x08) ? "Yes" : "No");
    Serial.print("6D orientation change detected: ");
    Serial.println((status & 0x04) ? "Yes" : "No");
    Serial.print("Free-fall event detected: ");
    Serial.println((status & 0x02) ? "Yes" : "No");
    Serial.print("Data-ready for X, Y, and Z: ");
    Serial.println((status & 0x01) ? "Ready" : "Not ready");
  } else {
    Serial.println("No data received from STATUS register.");
  }
}

/////////////////////////////////////////////
// Loading Serial Logo for Project
/////////////////////////////////////////////

void serialLogo(){
  Serial.println(" ");
  Serial.println(" ");
  Serial.println("  _________ __                .__      __________             .___  .___.__          ");
  Serial.println(" /   _____//  |_____________  |__| ____\\______   \\_____     __| _/__| _/|  |   ____  ");
  Serial.println(" \\_____  \\\\   __\\_  __ \\__  \\ |  |/    \\|     ___/\\__  \\   / __ |/ __ | |  | _/ __ \\ ");
  Serial.println(" /        \\|  |  |  | \\// __ \\|  |   |  \\    |     / __ \\_/ /_/ / /_/ | |  |_\\  ___/ ");
  Serial.println("/_______  /|__|  |__|  (____  /__|___|  /____|    (____  /\\____ \\____ | |____/\\___  >");
  Serial.println("        \\/                  \\/        \\/               \\/      \\/    \\/      2024 \\/ ");
  Serial.println(" ");
  Serial.println(" By: KinkMakers.core()");
  Serial.println(" Version 1.0");
  Serial.println(" ");
  Serial.println(" See Licenses.txt file for details of included items");
  Serial.println(" ");
  Serial.println(" ");
}




/////////////////////////////////////////////
// Serial Response
/////////////////////////////////////////////
void serialResponse(){
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
      case 's':
        LogStatus("Read Status Register");
        readStatusRegister();
        break;


    } // switch(temp) 
}


/////////////////////////////////////////////
// Setup
/////////////////////////////////////////////


void setup() {

  // so the hx711 doesn't like the ESP32 at full speed, we need to slow it down
    setCpuFrequencyMhz(80); //set the CPU frequency to 80MHz

  //initialize serial monitor
    Serial.begin(115200);
    serialLogo();

  //setup the eeprom
    EEPROM.begin(512);
  
  // i2c stuff for accelerometer

    Wire.begin(g_SDA, g_SCL); // Initialize I2C with custom SDA and SCL pins
    i2c_scanner(); // Call the I2C scanner function
    setupAccelerometer();


  //read the calibration factor from eeprom
    calibration_factor = EEPROM.read(CALIBRATION_FACTOR_ADDRESS);

  //if calibration factor is 0, set it to 25, 0 =  empty EEPROM
    if(calibration_factor == 0)
    {
      calibration_factor = 25;
      EEPROM.write(CALIBRATION_FACTOR_ADDRESS, calibration_factor);
    }

  //initialize LED strip
    FastLED.addLeds<WS2812B, led_pixel, GRB>(leds, 1);
    FastLED.setBrightness(255);
    FastLED.clear();
    leds[0] = CRGB::Red;
    FastLED.show();


  //initialize the hx711

    scale.begin(s_dout, s_clock);
    scale.set_gain(64);

    delay(500);

    scale.tare();
    LogDebug("Scale tared, test reading: " + String(scale.get_units()));

    delay(500);

    LogDebug("HX711 initialized");

    long zero_factor = scale.read_average(10); //get the zero factor
      LogDebug("Zero factor: " + String(zero_factor));

  // run flag
    run = autoStart;  // Look in the defines at the top of the file to set this


  // get that LED showing something 
    FastLED.clear();
    leds[0] = CRGB::Yellow;
    FastLED.show();


  // ESPnow setup
    WiFi.mode(WIFI_STA);
    LogDebug("ESP-Now address: " + WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
      LogError("Error initializing ESP-NOW");
      return;
    } else {
      LogDebug("ESP-NOW initialized");
    }

    esp_now_register_send_cb(OnDataSent);
      LogDebug("Registering ESP-NOW send callback");


  // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
  
  // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      LogError("Failed to add peer");
      return;
    }
    LogDebug("ESP-NOW setup complete");

    FastLED.clear();
    leds[0] = CRGB::Green;
    FastLED.show();

} //Void Setup



void loop() {

  if(run){

    //read the load cell and accelerometer
    float current_reading = (abs(scale.get_units(2))/calibration_factor);
    updateLastPeak(current_reading);
    
    Serial.print(">Strain Guage:");
    Serial.println(current_reading);
    Serial.print(">Peak:");
    Serial.println(lastPEAK);

    // read from the accelerometer  
    int16_t x, y, z;
    readAccelData(&x, &y, &z);
    Serial.print(">X: ");
    Serial.println(x);
    Serial.print(">Y: ");
    Serial.println(y);
    Serial.print(">Z: ");
    Serial.println(z);


    //change the LED color based on the load cell reading
    //led_from_strain(current_reading);

    // advance colour rainbow on led strip to demonstrait program is stillrunning
    static uint8_t hue = 0;
    hue++;
    leds[0] = CHSV(hue, 255, 255);
    FastLED.show();

  } // if(run)


  // Before we go check for serial input and then deal with it
  if(Serial.available()){ serialResponse();}





  if(afterNow(espNowSendTime)){
    // Send every 250ms + a random amount of time to avoid collisions
      espNowSendTime = millis() + 250 + random(0,50); 

    // build the ESP_NOW message 
      myData.peakValue = lastPEAK;
      memcpy(myData.macAddress, WiFi.macAddress().c_str(), 6);
      memcpy(myData.deviceName, MyDeviceName, sizeof(MyDeviceName));

    // Send message via ESP-NOW
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
      if (result == ESP_OK) {
        // LogDebug("Sent data via ESP-NOW");
      } else {
        LogError("Error sending data via ESP-NOW");
      }
  }// if(afterNow(espNowSendTime))




} //Void Loop




