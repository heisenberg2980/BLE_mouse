//Replace with the name of the mouse
#define deviceName "My BT mouse"
#define deviceNameNoSpaces "my_BT_mouse"

//Replace with the room name where the node will be placed; example: #define room "living-room"
#define room "living-room"



//Replace with your Wifi SSID; example: #define ssid "MyWifi"
#define ssid "MyWifi"


//Replace with your Wifi password; example: #define password "12345678"
#define password "12345678"

//Replace with a human-friendly host name. Must not contain spaces or special characters and be unique on your network
#define hostname "esp32_" deviceNameNoSpaces "_" room

//Replace with your MQTT Broker address; example: #define mqttHost IPAddress(192, 168, 1, 195)
#define mqttHost IPAddress(192, 168, 1, 195)

//Replace with your MQTT Broker port; example: #define mqttPort 1883
#define mqttPort 1883

//Replace with your MQTT Broker user; example: #define mqttUser "homeassistant"
#define mqttUser "mqtt_user"

//Replace with your MQTT Broker password; example: #define mqttPassword "12345678"
#define mqttPassword "mqtt_password"

// Logic level for turning the led on. Most boards use active low, meaning LED_ON should be set to 0
#define LED_ON 0

//Define the topic for publishing availability
#define availabilityTopic "mouse_nodes/" room

//Define the topic for publishing JSON attributes
#define telemetryTopic "mouse_nodes/tele/" room

//Define the base topic for room detection. Usually "room_presence"
#define channel availabilityTopic deviceNameNoSpaces