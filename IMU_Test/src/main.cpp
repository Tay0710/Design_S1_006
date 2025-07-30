// Code for extracting raw data from the MPU-6050 and sending data via TCP

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <AsyncTCP.h>
#include <WiFi.h>
#include <stdio.h>
#include <Ticker.h>

#include "config.h"

AsyncClient* client = new AsyncClient; // Initialise TCP client

Ticker sendTimer;

int counter = 0;

#define SDA_PIN   21
#define SCL_PIN   44

float timeStamp, accelXOffset, accelYOffset, accelZOffset, rollOffset, pitchOffset, yawOffset; // Calibration floats

Adafruit_MPU6050 mpu;

void sendData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  timeStamp = millis()/1000.0;

	char buffer[100]; // arbitrarily large size
  //sprintf(buffer, "(%d, %d, %.2f, %.2f, %.2f, %.2f)", counter, counter, y, z, yaw, distance); // automatically trims buffer[]
  sprintf(buffer, "(%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f)", timeStamp, g.gyro.x - rollOffset, g.gyro.y - pitchOffset, 
    g.gyro.z - yawOffset, a.acceleration.x - accelXOffset, a.acceleration.y - accelYOffset, a.acceleration.z - accelZOffset);
  
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
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);  // changing the SDA and SCL pins due to T-Display

  while (!Serial)
    delay(10); // pausing until the serial console is open

  Serial.println("Path tracking test");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");

  Serial.println("Calibrating...");
  int num_avg;
  
  for (num_avg = 0; num_avg < 1000; num_avg++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accelXOffset += a.acceleration.x;
    accelYOffset += a.acceleration.y;
    accelZOffset += a.acceleration.z;
    rollOffset += g.gyro.x;
    pitchOffset += g.gyro.y;
    yawOffset += g.gyro.z;
    delay(1);
  }

  accelXOffset = accelXOffset/num_avg;
  accelYOffset = accelYOffset/num_avg;
  accelZOffset = accelZOffset/num_avg - 1;
  rollOffset = rollOffset/num_avg;
  pitchOffset = pitchOffset/num_avg;
  yawOffset = yawOffset/num_avg;

  Serial.println("Calibration done!");

  Serial.println("");

  // Serial.println("Time (s),Gyroscope X (deg/s),Gyroscope Y (deg/s),Gyroscope Z (deg/s),Accelerometer X (g),Accelerometer Y (g),Accelerometer Z (g)");
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

void loop() {

  /* Get new sensor events with the readings */
  // sensors_event_t a, g, temp;
  // mpu.getEvent(&a, &g, &temp);
  // timeStamp = millis()/1000.0;
  
  /* Print out the values */
  // Serial.print(timeStamp, 4);
  // Serial.print(",");
  // Serial.print(g.gyro.x - rollOffset, 4);
  // Serial.print(",");
  // Serial.print(g.gyro.y - pitchOffset, 4);
  // Serial.print(",");
  // Serial.print(g.gyro.z - yawOffset, 4);
  // Serial.print(",");
  // Serial.print(a.acceleration.x - accelXOffset, 4);
  // Serial.print(",");
  // Serial.print(a.acceleration.y - accelYOffset, 4);
  // Serial.print(",");
  // Serial.println(a.acceleration.z - accelZOffset, 4);
  // delay(5);
  
}