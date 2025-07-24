#include <Arduino.h>
#include <AsyncTCP.h>
#include <WiFi.h>
#include <stdio.h>


#include "config.h"

#define TRIG_PIN 11
#define ECHO_PIN 10


AsyncClient* client = new AsyncClient;



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

	pinMode(TRIG_PIN, OUTPUT);
  	pinMode(ECHO_PIN, INPUT);
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
	client->onData(&handleData, client); // when data received
	client->onConnect(&onConnect, client); // on successful connect

	// TODO: either make this a blocking call or ensure PC is connected first
	// TOOD: check if I dont connect 
	client->connect(SERVER_HOST_NAME, TCP_PORT);

	Serial.println("Connecting to TCP server");

	while (!client->connecting())
	{
		Serial.print(".");
		delay(500);
	}

	// TODO: wait for PC connection to AP if possible

}

void loop() {

	float duration, distance;

	// read ultrasonic sensor data
	digitalWrite(TRIG_PIN, LOW);
	delayMicroseconds(2);
	digitalWrite(TRIG_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(TRIG_PIN, LOW);

	duration = pulseIn(ECHO_PIN, HIGH);
	distance = (duration*.0343)/2; // in cm
	Serial.print("Distance: ");
	Serial.println(distance);

	// Send data
	if (client->space() > 32 && client->canSend()) {
		char buffer[32];
		sprintf(buffer, "%.2f", distance); 
		client->add(buffer, strlen(buffer));
		client->send();
	}

	// TODO: What happens if the space is not large enough?


	delay(100);

}

