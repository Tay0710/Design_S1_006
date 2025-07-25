#include <Arduino.h>
#include <AsyncTCP.h>
#include <WiFi.h>
#include <stdio.h>


#include "config.h"

#define TRIG_PIN 11
#define ECHO_PIN 10


AsyncClient* client = new AsyncClient; // Initialise TCP client

hw_timer_t* Timer0_Cfg = NULL; // Initialise hardware timer

int counter = 0;


void IRAM_ATTR Timer0_ISR()
{
    Serial.println("STUFF TO DO IN TIMER...");
	delay(1000); // ok 2 seconds is too long, best to add data?? but 1 second is fine

	// bruh cus 2 seconds is longer than the actual timer, so this will obviously break if the next alarm occurs before first alarm is done, but cant guarantee how long it takes the send stuff unless you add timeout on sending??
	// hmm senc continuously and 

	counter++;
	Serial.println(counter);

	// So ISR has a time limit if not watchdog timer will start 


	// char message[32];

	// sprintf(message, "%.2f", counter);

	// if (client->space() > 32 && client->canSend()) {
		
	// 	client->add(message, strlen(message));
	// 	client->send();
	// }
	// TODO: read distance from sensor

	// update variables to be sent, i.e. time, position, distance 
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
	timerAlarmEnable(Timer0_Cfg); // Enable timer
	// TODO: bug with timer messing with AP and causing reboot

}

void onDisconnect(void* arg, AsyncClient* client) {

	// TODO: check this never occurs if it never 

	// TODO: fix... appear to have a timeout where this occurs? but only occurs once? might not be an issue??
	Serial.printf("\n client has been disconnected from %s on port %d \n", SERVER_HOST_NAME, TCP_PORT);
	// TODO: Stop sending data??
	timerAlarmDisable(Timer0_Cfg); // Disable timer
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

	// Setup timer
	Timer0_Cfg = timerBegin(0, 80, true); // configures timer 0 to have a prescaler of 80 and count up (true=count_up, false=count_down)
	// Note: timer 2 stops TCP from working, 0 1 and 3 causes reboot loop
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, 1000000, true); // Prescaler of 80 + clock of 80 Mhz = TickTime of 1us, alarm register = 1000000, therefore every 1 s

	// Setup ESP32 as AsyncTCP Client 
	client->onConnect(&onConnect, client); // on successful connect
	
	client->connect(SERVER_HOST_NAME, TCP_PORT); // attempt to connect
	client->onData(&handleData, client); // when data received

	Serial.println("Connecting to TCP server");

	// Wait until ESP32 is connect to the TCP Server on PC
	while (!client->connected())
	{
		Serial.print(".");
		delay(1000);
	}

	client->onDisconnect(&onDisconnect, client); // when disconnected

	// TODO: issue with esp32 rebooting?? when pc connects to AP
	// issue was to do with how long ISR takes to run

}

void loop() {

	// float duration, distance;

	// // read ultrasonic sensor data
	// // TODO: ideally this would be in a separate thread, running asynchronously
	// digitalWrite(TRIG_PIN, LOW);
	// delayMicroseconds(2);
	// digitalWrite(TRIG_PIN, HIGH);
	// delayMicroseconds(10);
	// digitalWrite(TRIG_PIN, LOW);

	// duration = pulseIn(ECHO_PIN, HIGH);
	// distance = (duration*.0343)/2; // in cm
	// Serial.print("Distance: ");
	// Serial.println(distance);


	// TODO: instead of reading ultrasonic send dummy data
	// MESSAGE FORMAT: "(TIME, x, y, z, yaw, front_distance)"



	// Send data
	// if (client->space() > 32 && client->canSend()) {
	// 	char buffer[32];
	// 	sprintf(buffer, "%.2f", distance); // TODO: format message properly
	// 	client->add(buffer, strlen(buffer));
	// 	client->send();
	// }

	// TODO: What happens if the space is not large enough?
	// TODO: 


	// delay(100);

}

