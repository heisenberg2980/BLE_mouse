#include <Arduino.h>
/**
 * This example turns the ESP32 into a Bluetooth LE mouse that continuously moves the mouse.
 */
#include <BleMouse.h>

//BleMouse bleMouse;
BleMouse bleMouse("Unicorn mouse", "Unicorn enterprise", 100);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  bleMouse.begin();
	//pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {

  static long mouseTick = millis();
  static long resetTick = millis();

  if(bleMouse.isConnected()) {

    Serial.println("Move mouse pointer up");
    bleMouse.move(0,-1);
    //digitalWrite(LED_BUILTIN, LOW); 
    delay(1000);

    Serial.println("Move mouse pointer down");
    bleMouse.move(0,1);
    //digitalWrite(LED_BUILTIN, HIGH);  

    delay(60000);
    mouseTick = millis();
    resetTick = millis();

  }
  else {
    //digitalWrite(LED_BUILTIN, LOW); 
    //delay(60000);
    if (millis() - mouseTick > 10000) {
      mouseTick = millis();
      Serial.println("BLE not connected");
      if (millis() - resetTick > 60000) {
        resetTick = millis();
     		Serial.println("Restarting");
    		ESP.restart();
      }
    }
    //digitalWrite(LED_BUILTIN, HIGH);  
    //delay(120000);
  }
}