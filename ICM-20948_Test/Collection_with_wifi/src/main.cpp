// Code for extracting raw data from the ICM-20948 and sending data via TCP

#include <ICM_20948.h> // SparkFun ICM-20948 Library
#include <Wire.h>
#include <Arduino.h>

#include <AsyncTCP.h>
#include <WiFi.h>
#include <stdio.h>
#include <Ticker.h>

#include <config.h>

AsyncClient* client = new AsyncClient; // Initialise TCP client

Ticker sendTimer;

int counter = 0;

#define SDA_PIN   21
#define SCL_PIN   44

float timeStamp, accelXOffset, accelYOffset, accelZOffset, rollOffset, pitchOffset, yawOffset; // Calibration floats

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define AD0_VAL 1

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

void sendData() {

  myICM.getAGMT();
  timeStamp = millis()/1000.0;

	char buffer[100]; // arbitrarily large size
  //sprintf(buffer, "(%d, %d, %.2f, %.2f, %.2f, %.2f)", counter, counter, y, z, yaw, distance); // automatically trims buffer[]
  sprintf(buffer, "(%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f)",
          timeStamp,
          myICM.gyrX() - rollOffset,
          myICM.gyrZ() - yawOffset,
          myICM.gyrY() - pitchOffset,
          (myICM.accX() - accelXOffset)/1000,
          (myICM.accY() - accelYOffset)/1000,
          (myICM.accZ() - accelZOffset)/1000);
  
	// Send data
	if (client->canSend()) {
		Serial.println(buffer);
		client->add(buffer, strlen(buffer));
		client->send();
	}
	// counter++; // increment counter
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
	sendTimer.attach(0.1, sendData);  // interval in seconds
}

void onDisconnect(void* arg, AsyncClient* client) {

	Serial.printf("\n client has been disconnected from %s on port %d \n", SERVER_HOST_NAME, TCP_PORT);
	sendTimer.detach(); // Pause timer

	// TODO: land drone safely 
}

void setup(void) {
  Serial.begin(115200);

  while (!Serial) {};
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);  // changing the SDA and SCL pins due to T-Display

  Wire.setClock(100000);
  delay(500);

  bool initialized = false;
  while (!initialized)
  {

#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(Wire, AD0_VAL);
#endif

    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  Serial.println("");

  Serial.println("Calibrating...");
  int num_avg;
  
  for (num_avg = 0; num_avg < 1000; num_avg++) {
    myICM.getAGMT(); // read all data
    accelXOffset += myICM.accX();
    accelYOffset += myICM.accY();
    accelZOffset += myICM.accZ();
    rollOffset += myICM.gyrX();
    pitchOffset += myICM.gyrY();
    yawOffset += myICM.gyrZ();
    delay(1);
  }

  accelXOffset /= num_avg;
  accelYOffset /= num_avg;
  accelZOffset /= num_avg;
  accelZOffset -= 1;
  rollOffset /= num_avg;
  pitchOffset /= num_avg;
  yawOffset /= num_avg;

  Serial.println("Calibration done!");

  Serial.println("");
  delay(100);

  // Setup ESP32 as access point
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

void loop() {}