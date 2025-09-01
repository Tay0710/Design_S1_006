#include <WiFi.h>
#include <WebServer.h>
#include <SD.h>
#include <SPI.h>
#include "ICM45686.h"

// Use VSPI for both IMU and SD card
#define IMU_CS   5   // IMU CS pin
#define SD_CS    32  // SD card CS pin
#define HSPI_MOSI 23 // (19)
#define HSPI_CLK 18
#define HSPI_MISO 19 // (23)

SPIClass hspi(HSPI);
ICM456xx IMU(hspi, IMU_CS);

#define TRIGGER_PIN 25

#define AP_SSID "ESP32_Frames"
#define AP_PASSWORD "12345678"

bool recording = false;

// std::array<float, 6> imu_values = {0, 0, 0, 0, 0, 0};

float  G_rating = 2;      // 2/4/8/16/32 g
float  dps_rating = 125; // 15.625/31.25/62.5/125/250/500/1000/2000/4000 dps

// Calibration values
//  -0.015272462	0.009307082	1.006992415	0.754845671	-0.746207889	-0.116757765
//  AccelX(g)	AccelY(g)	AccelZ(g)	GyroX(dps)	GyroY(dps)	GyroZ(dps)



// Web server
WebServer server(80);
const char* filename = "/imu.csv";


// Calibration offsets
float calibAccelX = 0, calibAccelY = 0, calibAccelZ = 0;
float calibGyroX  = 0, calibGyroY  = 0, calibGyroZ  = 0;

void calibrateIMU(int samples) {
  long sumAx = 0, sumAy = 0, sumAz = 0;
  long sumGx = 0, sumGy = 0, sumGz = 0;

  Serial.println("Starting IMU calibration...");

  unsigned long startTime = millis();

  for (int i = 0; i < samples; i++) {
    inv_imu_sensor_data_t imu_data;
    IMU.getDataFromRegisters(imu_data);

    sumAx += imu_data.accel_data[0];
    sumAy += imu_data.accel_data[1];
    sumAz += imu_data.accel_data[2];
    sumGx += imu_data.gyro_data[0];
    sumGy += imu_data.gyro_data[1];
    sumGz += imu_data.gyro_data[2];

    // delay(2); // ~500 Hz
  }

  unsigned long endTime = millis();
  unsigned long duration = endTime - startTime;

  float avgAx = (float)sumAx / samples;
  float avgAy = (float)sumAy / samples;
  float avgAz = (float)sumAz / samples;
  float avgGx = (float)sumGx / samples;
  float avgGy = (float)sumGy / samples;
  float avgGz = (float)sumGz / samples;



  calibAccelX = avgAx * G_rating / 32768.0;
  calibAccelY = avgAy * G_rating / 32768.0;
  calibAccelZ = avgAz * G_rating / 32768.0;
  calibGyroX  = avgGx * dps_rating / 32768.0;
  calibGyroY  = avgGy * dps_rating / 32768.0;
  calibGyroZ  = avgGz * dps_rating / 32768.0;

  Serial.println("Calibration complete:");
  Serial.printf("Accel offsets: %.9f, %.9f, %.9f\n", calibAccelX, calibAccelY, calibAccelZ);
  Serial.printf("Gyro  offsets: %.9f, %.9f, %.9f\n", calibGyroX, calibGyroY, calibGyroZ);
  Serial.printf("Calibration took %lu ms (%lu samples at ~%lu Hz)\n",
                duration, samples, (samples * 1000UL) / duration);
}



void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(TRIGGER_PIN, INPUT_PULLUP);

  // Start Wi-Fi AP
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("AP started. Connect to: %s\nIP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  // Initialize VSPI
  // SPI.begin(18, 19, 23); // SCK=18, MISO=19, MOSI=23
  hspi.begin(HSPI_CLK, HSPI_MISO, HSPI_MOSI);


  // Initialize IMU
  if (IMU.begin() != 0) {
    Serial.println("ICM456xx initialization failed");
    while (1);
  }
  IMU.startAccel(1600, G_rating);   // 100 Hz, ±16 g
  IMU.startGyro(1600, dps_rating);  // 100 Hz, ±2000 dps

  
  // Initialize SD card on same SPI bus but different CS
  if (!SD.begin(SD_CS, hspi)) {
    Serial.println("SD init failed!");
    while (1);
  }

  // CSV header
  SD.remove(filename);
  File file = SD.open(filename, FILE_WRITE);
  if (file) {
    file.println("time,gyro x,gyro y,gyro z,accel x,accel y,accel z");
    file.close();
  }



  // Web server endpoints
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", "<html><body><h1>ESP32 IMU Download</h1><a href='/download'><button>Download CSV</button></a></body></html>");
  });

  server.on("/download", HTTP_GET, []() {
    File f = SD.open(filename);
    if(f) {
      server.streamFile(f, "text/csv");
      f.close();
    } else {
      server.send(404, "text/plain", "File not found");
    }

  });

  server.begin();

  Serial.println("Do Not move Drone while Calibrating the ICM.");
  calibrateIMU(1000);

  Serial.println("Entering Loop");
}

static File file;  // persistent across loop() calls

void loop() {

    recording = (digitalRead(TRIGGER_PIN) == LOW);

    if (recording) {
        inv_imu_sensor_data_t imu_data;
        IMU.getDataFromRegisters(imu_data);

        // static unsigned long lastMicros = 0;
        unsigned long now = micros();
        float now_s = now / 1000000.0;
        // float dt = (now - lastMicros) / 1000000.0; // delta time in seconds
        // lastMicros = now;


        // Open, write, and close file each time
        // Open file if not already open
        if (!file) {
            file = SD.open(filename, FILE_APPEND);
            Serial.println("Opening File! You have to pull trigger pin high to close file and be able to download the data.");
            if (!file) {
                Serial.println("Failed to open file for writing!");
            }
          }

        if (file) {
            file.printf("%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
                        now_s, // dt,
                        imu_data.gyro_data[0]*dps_rating/32768.0 - calibGyroX,
                        imu_data.gyro_data[1]*dps_rating/32768.0 - calibGyroY,
                        imu_data.gyro_data[2]*dps_rating/32768.0 - calibGyroZ,
                        imu_data.accel_data[0]*G_rating/32768.0 - calibAccelX,
                        imu_data.accel_data[1]*G_rating/32768.0 - calibAccelY,
                        imu_data.accel_data[2]*G_rating/32768.0 - calibAccelZ);
        }
    }
    else { // recording stopped
      server.handleClient();

        if (file) {
            file.close();   // close the file
            file = File();  // reset to null so !file is true next time
            Serial.println("Closing File");
    }
  }
}



/////////////////////////////////////////////////////////////////////////////////
// Micros() of ESP32 ABsolute time
// void loop() {
//     server.handleClient();

//     recording = (digitalRead(TRIGGER_PIN) == LOW);

//     static File file;

//     // Open file once at start of recording
//     if (recording && !file) {
//         file = SD.open(filename, FILE_APPEND);
//         if (!file) {
//             Serial.println("Failed to open file for writing!");
//             return;
//         }
//     }

//     // Write data if recording
//     if (recording && file) {
//         inv_imu_sensor_data_t imu_data;
//         IMU.getDataFromRegisters(imu_data);

//         unsigned long timestamp = micros(); // absolute time in microseconds

//         file.printf("%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
//                     timestamp,
//                     imu_data.gyro_data[0]*dps_rating/32768.0 - calibGyroX,
//                     imu_data.gyro_data[1]*dps_rating/32768.0 - calibGyroY,
//                     imu_data.gyro_data[2]*dps_rating/32768.0 - calibGyroZ,
//                     imu_data.accel_data[0]*G_rating/32768.0 - calibAccelX,
//                     imu_data.accel_data[1]*G_rating/32768.0 - calibAccelY,
//                     imu_data.accel_data[2]*G_rating/32768.0 - calibAccelZ);
//     }

//     // Close file if recording stopped
//     if (!recording && file) {
//         file.close();
//         file = File(); // reset handle
//     }

//     delay(1); // adjust delay to control sample rate
// }
/////////////////////////////////////////////////////////////////////////////////