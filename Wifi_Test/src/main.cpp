#include <Arduino.h>
#include <WiFi.h>

#define SSID "ESP-TEST-TTGO-S3"
#define PASSWORD "123456789"


void setup(void) {
  Serial.begin(115200);
  delay(500);

  // Setup ESP32 as access point
  Serial.println("Setting Access Point...");
	WiFi.softAP(SSID, PASSWORD);

  // Print ESP Local IP Address
	IPAddress IP = WiFi.softAPIP();
	Serial.print("AP IP Address: ");
	Serial.println(IP);

}

void loop() {
  // put your main code here, to run repeatedly:
}
