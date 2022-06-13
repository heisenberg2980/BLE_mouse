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
}

void loop() {
  if(bleMouse.isConnected()) {

    Serial.println("Move mouse pointer up");
      bleMouse.move(0,-1);
    //digitalWrite(LED_BUILTIN, HIGH); 
    delay(1000);

    Serial.println("Move mouse pointer down");
      bleMouse.move(0,1);
    //digitalWrite(LED_BUILTIN, LOW);  

    delay(60000);

  }
  else {
    Serial.println("BLE not connected");
    delay(1000);
  }
}