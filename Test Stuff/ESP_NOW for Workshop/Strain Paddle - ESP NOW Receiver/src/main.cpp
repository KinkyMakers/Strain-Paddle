/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    float peakValue;
    uint8_t macAddress[6];
    char deviceName[20];
  } struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  // if the device name is not null, print the device name
  if (myData.deviceName[0] != '\0') {
    Serial.print(">");
    Serial.print(myData.deviceName);
    Serial.print(" : ");
  }else {
    Serial.print("> ");
    for (int i = 0; i < 6; i++) {
      Serial.print(myData.macAddress[i], HEX);
      if (i < 5){
        Serial.print("_");
      }
    }
    Serial.print(" : ");
  }


  Serial.println(myData.peakValue);

}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {

}