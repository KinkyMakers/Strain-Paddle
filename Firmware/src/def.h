
//Pin Definitions

#define led_pixel 12        //ws2812b data pin
#define led_red 4           //red led pin
#define led_green 16        //green led pin

#define s_dout 19             //HX711 data pin
#define s_clock 18              //HX711 clock pin

#define g_SCL 25            //gyro scl pin
#define g_SDA 26            //gyro sda pin
#define g_INT 27            //gyro interrupt pin
#define g_INT2 17           //gyro interrupt pin 2


//Kyle's Macros from OSSM + some updates
#define LogDebug(message) Serial.println("[DEBUG] " + String(message))
#define LogError(message) Serial.println("[ERROR] " + String(message))
#define LogStatus(message) Serial.println("[STATUS] " + String(message))
#define LogSystem(message) Serial.println("[SYSTEM] " + String(message))