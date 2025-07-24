#include <Arduino.h>
#include <AsyncTCP.h>
#include <WiFi.h>

#include "config.h"


static void replyToServer(void* arg) {
	AsyncClient* client = reinterpret_cast<AsyncClient*>(arg);

	// send reply
	if (client->space() > 32 && client->canSend()) {
		char message[32];
		client->add(message, strlen(message));
		client->send();
	}
}

/* event callbacks */
static void handleData(void* arg, AsyncClient* client, void *data, size_t len) {
	Serial.printf("\n data received from %s \n", client->remoteIP().toString().c_str());
	Serial.write((uint8_t*)data, len);

}

void onConnect(void* arg, AsyncClient* client) {
	Serial.printf("\n client has been connected to %s on port %d \n", SERVER_HOST_NAME, TCP_PORT);
	replyToServer(client);
}

void setup() {
	Serial.begin(115200);
	delay(500); // Delay required so that print statements will show up

  // Setup ESP32 as the access point 
  // https://randomnerdtutorials.com/esp32-access-point-ap-web-server/
  Serial.println("Setting Access Point...");
  WiFi.softAP(SSID, PASSWORD);

  // Print ESP Local IP Address
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP Address: ");
  Serial.println(IP);

  // Setup ESP32 as AsyncTCP Client 
	AsyncClient* client = new AsyncClient;
	client->onData(&handleData, client);
	client->onConnect(&onConnect, client);
	client->connect(SERVER_HOST_NAME, TCP_PORT);

  // TODO: wait for PC connection to AP if possible

}

void loop() {
  // TODO

}
