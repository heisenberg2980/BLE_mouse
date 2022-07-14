#include <Arduino.h>
/**
 * This example turns the ESP32 into a Bluetooth LE mouse that continuously moves the mouse.
 */
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#include <BleMouse.h>
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include "Settings.h"

#define LED_GPIO LED_BUILTIN  

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
bool updateInProgress = false;
String localIp;
byte retryAttemptsWifi = 0;
byte retryAttemptsMqtt = 0;

//BleMouse bleMouse;
BleMouse bleMouse(deviceName " " room, deviceName " enterprise", 100);

bool sendTelemetry(int ble_connected = -1) {
	StaticJsonDocument<256> tele;
	tele["room"] = room;
	tele["ssid"] = ssid;
	tele["ip"] = localIp;
	tele["hostname"] = WiFi.getHostname();

	if (ble_connected > -1) {
		Serial.printf("connected: %d\n\r",ble_connected);
    tele["BLE_connected"] = ble_connected;
	}

	char teleMessageBuffer[258];
	serializeJson(tele, teleMessageBuffer);

	if (mqttClient.publish(telemetryTopic, 0, 0, teleMessageBuffer) == true) {
		Serial.println("Telemetry sent");
		return true;
	} else {
		Serial.println("Error sending telemetry");
		return false;
	}
}

void connectToWifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
	WiFi.setHostname(hostname);
	WiFi.begin(ssid, password);
}

bool handleWifiDisconnect() {
	Serial.println("WiFi has been disconnected.");
	if (WiFi.isConnected()) {
		Serial.println("WiFi appears to be connected. Not retrying.");
		return true;
	}
//	if (retryAttemptsWifi > 10) {
//		Serial.println("Too many retries. Restarting");
//		ESP.restart();
//	} else {
		retryAttemptsWifi++;
//	}
	if (mqttClient.connected()) {
		mqttClient.disconnect();
	}
	if (xTimerIsTimerActive(mqttReconnectTimer) != pdFALSE) {
		xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
	}

	if (xTimerReset(wifiReconnectTimer, 0) == pdFAIL) {
		Serial.println("failed to restart wifi");
		xTimerStart(wifiReconnectTimer, 0);
		return false;
	} else {
		Serial.println("wifi restarted");
		return true;
	}
}

void connectToMqtt() {
  Serial.print("Connecting to MQTT with ClientId ");
  Serial.println(hostname);
	if (WiFi.isConnected() && !updateInProgress) {
		mqttClient.setServer(mqttHost, mqttPort);
		mqttClient.setWill(availabilityTopic, 0, 1, "DISCONNECTED");
		mqttClient.setKeepAlive(60);
		mqttClient.setCredentials(mqttUser, mqttPassword);
		mqttClient.setClientId(hostname);
	  	mqttClient.connect();
	} else {
		Serial.println("Cannot reconnect MQTT - WiFi error");
		handleWifiDisconnect();
	}
}

bool handleMqttDisconnect() {
	Serial.println("MQTT has been disconnected.");
	if (updateInProgress) {
		Serial.println("Not retrying MQTT connection - OTA update in progress");
		return true;
	}
		retryAttemptsMqtt++;
	if (WiFi.isConnected() && !updateInProgress) {
		Serial.println("Starting MQTT reconnect timer");
    if (xTimerReset(mqttReconnectTimer, 0) == pdFAIL) {
			Serial.println("failed to restart mqtt");
			xTimerStart(mqttReconnectTimer, 0);
		} else {
			Serial.println("mqtt restarted");
		}
    } else {
		Serial.print("Disconnected from WiFi; starting WiFi reconnect timiler\t");
		handleWifiDisconnect();
	}
    return true;
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %x\n\r", event);
		switch(event) {
	    case SYSTEM_EVENT_STA_GOT_IP:
					//digitalWrite(LED_GPIO, !LED_ON);
	        Serial.print("IP address: \t");
	        Serial.println(WiFi.localIP());
					localIp = WiFi.localIP().toString().c_str();
					Serial.print("Hostname: \t");
					Serial.println(WiFi.getHostname());
	        connectToMqtt();
					if (xTimerIsTimerActive(wifiReconnectTimer) != pdFALSE) {
						Serial.println("Stopping wifi reconnect timer");
						xTimerStop(wifiReconnectTimer, 0);
					}
					retryAttemptsWifi = 0;
	        break;
	    case SYSTEM_EVENT_STA_DISCONNECTED:
					//digitalWrite(LED_GPIO, LED_ON);
	        Serial.println("WiFi lost connection, resetting timer\t");
					handleWifiDisconnect();
					break;
			case SYSTEM_EVENT_WIFI_READY:
					Serial.println("Wifi Ready");
					handleWifiDisconnect();
					break;
			case SYSTEM_EVENT_STA_START:
					Serial.println("STA Start");
					tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, hostname);
					if (xTimerIsTimerActive(wifiReconnectTimer) != pdFALSE) {
						TickType_t xRemainingTime = xTimerGetExpiryTime( wifiReconnectTimer ) - xTaskGetTickCount();
						Serial.print("WiFi Time remaining: ");
						Serial.println(xRemainingTime);
					} else {
						Serial.println("WiFi Timer is inactive; resetting\t");
						handleWifiDisconnect();
					}
					break;
			case SYSTEM_EVENT_STA_STOP:
					Serial.println("STA Stop");
					handleWifiDisconnect();
					break;
			default:
					Serial.println("Event not considered");
					handleWifiDisconnect();
					break;
    }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
	retryAttemptsMqtt = 0;

	if (mqttClient.publish(availabilityTopic, 0, 1, "CONNECTED") == true) {
		//Serial.print("Success sending message to topic:\t");
		//Serial.println(availabilityTopic);
	} else {
		Serial.println("Error sending message");
	}

	//sendTelemetry();

}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.print("Disconnected from MQTT. Reason: ");
  Serial.println(static_cast<uint8_t>(reason));
  handleMqttDisconnect();
}

void configureOTA() {
	ArduinoOTA
    .onStart([]() {
			Serial.println("OTA Start");
			updateInProgress = true;
			mqttClient.disconnect(true);
			xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    })
    .onEnd([]() {
			updateInProgress = false;
			//digitalWrite(LED_GPIO, !LED_ON);
      Serial.println("\n\rEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
			byte percent = (progress / (total / 100));
      Serial.printf("Progress: %u", percent);
      Serial.println("");
			//digitalWrite(LED_GPIO, percent % 2);
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
			//ESP.restart();
    });
	ArduinoOTA.setHostname(hostname);
	ArduinoOTA.setPort(8266);
  ArduinoOTA.begin();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  bleMouse.begin();
	//pinMode(LED_BUILTIN, OUTPUT);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);

  connectToWifi();

	configureOTA();
}

void loop() {

	TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
	TIMERG0.wdt_feed=1;
	TIMERG0.wdt_wprotect=0;
	ArduinoOTA.handle();

	unsigned long mouseTick = millis();
	unsigned long resetTick = millis();
	static int ble_connected;
	static bool firstIterationConnected = true;
	static bool firstIterationNotConnected = true;

	if(bleMouse.isConnected()) {

		if ((millis() - mouseTick > 60000) or firstIterationConnected) {
			firstIterationConnected = false;
			firstIterationNotConnected = true;
			ble_connected = 1;
			sendTelemetry(ble_connected);

			Serial.println("Move mouse pointer up");
			bleMouse.move(0,-1);
			//digitalWrite(LED_BUILTIN, LOW); 
			delay(1000);

			Serial.println("Move mouse pointer down");
			bleMouse.move(0,1);
			//digitalWrite(LED_BUILTIN, HIGH);  

			mouseTick = millis();
			resetTick = millis();
		}
	}
	else {
		//digitalWrite(LED_BUILTIN, LOW); 
		if ((millis() - mouseTick > 10000) or firstIterationNotConnected) {
			firstIterationNotConnected = false;
			firstIterationConnected = true;
			mouseTick = millis();
			Serial.println("BLE not connected");
			ble_connected = 0;
			sendTelemetry(ble_connected);
			if (millis() - resetTick > 60000) {
				resetTick = millis();
				Serial.println("Restarting ESP");
				ESP.restart();
			}
		}
		//digitalWrite(LED_BUILTIN, HIGH);  
	}
}