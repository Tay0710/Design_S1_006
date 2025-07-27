#include <Arduino.h>
#include <AsyncTCP.h>
#include <WiFi.h>
#include <stdio.h>
#include <Ticker.h>

#include "config.h"

#define TRIG_PIN 11
#define ECHO_PIN 10


AsyncClient* client = new AsyncClient; // Initialise TCP client

Ticker sendTimer;

int counter = 0;

volatile float duration, distance = 0.0; // Initialise variables for ultrasonic sensor 

volatile float x, y, z = 0.0; // Initialise position variables
volatile float yaw = 0.0; // Initalise orientation variables


void sendData() {
	char buffer[100]; // arbitrarily large size
    sprintf(buffer, "(%d, %d, %.2f, %.2f, %.2f, %.2f)", counter, counter, y, z, yaw, distance); // automatically trims buffer[]
	// Send data
	if (client->canSend()) {
		Serial.println(buffer);
		client->add(buffer, strlen(buffer));
		client->send();
	}
	counter++; // increment counter
}

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
	sendTimer.attach(0.1, sendData);  // interval in seconds

}

void onDisconnect(void* arg, AsyncClient* client) {

	Serial.printf("\n client has been disconnected from %s on port %d \n", SERVER_HOST_NAME, TCP_PORT);
	sendTimer.detach(); // Pause timer

	// TODO: land drone
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
	client->onConnect(&onConnect, client); // on successful connect
	
	client->connect(SERVER_HOST_NAME, TCP_PORT); // attempt to connect
	client->onData(&handleData, client); // when data is received

	Serial.println("Connecting to TCP server");

	// Wait until ESP32 is connected to the TCP Server on PC
	while (!client->connected())
	{
		Serial.print(".");
		delay(1000);
	}

	client->onDisconnect(&onDisconnect, client); // when disconnected
}

void loop() {

	// Read ultrasonic sensor data
	digitalWrite(TRIG_PIN, LOW);
	delayMicroseconds(2);
	digitalWrite(TRIG_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(TRIG_PIN, LOW);

	duration = pulseIn(ECHO_PIN, HIGH);
	distance = (duration*.0343)/2; // in cm

	// Note: this seems a bit slow
	Serial.print("Distance: ");
	Serial.println(distance);

}

