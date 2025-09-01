// Changes that can be made:
// 1. Add timestamps to Optical Flow data (currently commented out)???
// 2. Add Flush to force code out of buffer to SD Card, to prevent data loss. 
// 3. Move OF onto VSPI, instead of HSPI
// 4. Add Code for additional ToF L7 sensor on the same I2C bus (different address) - make one L7 4x4 (roof or floor) and one L7 8x8 (side wall).


// Side Notes:
// I think 2gs is enough for the accelerometer range (image the speed of you falling is only 1 g, drone while mapping wouldn't be accelerating that quick) - Owen. 

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SD.h>
#include <SPI.h>
#include <ICM45686.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
#include <Bitcraze_PMW3901.h>

// Optical flow SPI pins (pins for Owen's ESP32)
#define OF_CS 15 // Optical Flow CS pin
// #define OF_MOSI 13
// #define OF_CLK 14
// #define OF_MISO 33
#define IMU_MOSI 23 // (19)
#define IMU_CLK 18
#define IMU_MISO 19 // (23)
#define SD_CS 32  // (36) Example CS pin for SD card
#define IMU_CS 5  // Example CS pin for SD card
#define TRIGGER_PIN 25 // (4) use GPIO4 as SWITCH to turn on/off when the esp32 is recording data mode. When pulled LOW, RECORDING Starts. 

#define AP_SSID "ESP32_Frames"
#define AP_PASSWORD "12345678"

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM
ICM456xx IMU(SPI, IMU_CS); 

// SPIClass SPI2(HSPI);
Bitcraze_PMW3901 flow(OF_CS);

char frame[35*35]; //array to hold the framebuffer

int imageResolution = 0; // Used to pretty print output

// Web server
WebServer server(80);
const char* imuFileName = "/imu_ICM45686.csv";
const char* tofFileName = "/tof_L7.csv";
const char* ofFileName = "/of_PMW3901.csv";

// Global file handles
File imuFile;
File tofFile;
File ofFile;

// Calibration offsets
float calibAccelX = 0, calibAccelY = 0, calibAccelZ = 0;
float calibGyroX  = 0, calibGyroY  = 0, calibGyroZ  = 0;

float  G_rating = 4;      // 2/4/8/16/32 g
float  dps_rating = 125; // 15.625/31.25/62.5/125/250/500/1000/2000/4000 dps

// Timing control
unsigned long lastIMUtime = 0;
unsigned long lastTOFtime = 0;
unsigned long lastOFtime = 0;
const unsigned long imuInterval = 625;    // microseconds → ~1600 Hz
const unsigned long tofInterval = 5000000;   // microseconds → ~15 Hz
const unsigned long ofInterval = 10000000;   // microseconds → ~120 Hz

char imuBuf[128];
char tofBuf[512];
char ofBuf[4096];

// ---- Calibration function ----
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

    // delay(2); // ~500 Hz - Delay is not required (ignore CHATGPT suggestion)
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
  Serial.printf("Accel offsets: %.6f, %.6f, %.6f\n", calibAccelX, calibAccelY, calibAccelZ);
  Serial.printf("Gyro  offsets: %.6f, %.6f, %.6f\n", calibGyroX, calibGyroY, calibGyroZ);
  Serial.printf("Calibration took %lu ms (%lu samples at ~%lu Hz)\n",
                duration, samples, (samples * 1000UL) / duration);
}


void setup()
{
  int ret;

  Serial.begin(115200);
  delay(1000);

  pinMode(TRIGGER_PIN, INPUT_PULLUP); // pull HIGH internally

  // Start Wi-Fi AP
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("AP started. Connect to: ");
  Serial.println(AP_SSID);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  // VSPI Setup
  SPI.begin(IMU_CLK, IMU_MISO, IMU_MOSI);

  // ICM45686 Begin
  if (IMU.begin() != 0) {
    Serial.println("ICM456xx initialization failed");
    while (1);
  }
  

  // Configure accelerometer and gyro
  IMU.startAccel(1600, G_rating);     // Max 6400Hz; 100 Hz, ±2/4/8/16/32 g
  IMU.startGyro(1600, dps_rating);    // 100 Hz, ±15.625/31.25/62.5/125/250/500/1000/2000/4000 dps
  // Data comes out of the IMU as steps from -32768 to +32768 representing the full scale range

  Serial.println("Do not move drone while calibrating the ICM.");
  calibrateIMU(1000);

  // PMW3901 begin
  if (!flow.begin()) {
      Serial.println("PMW3901 initialization failed. Check wiring!");
      while (1);  // stop if not found
  }
  Serial.println("b");
  delay(100);

  flow.enableFrameBuffer(); 
  Serial.println("c");

  // --- Initialize SD card (on same VSPI but with different CS) ---
  if (!SD.begin(SD_CS, SPI)) {
    Serial.println("SD init failed!");
    while (1);
  }

  // Init IMU CSV
  SD.remove(imuFileName);
  File imu = SD.open(imuFileName, FILE_WRITE);
  imu.println("time,gyro x,gyro y,gyro z,accel x,accel y,accel z");
  imu.close();

  // Init ToF CSV
  SD.remove(tofFileName);
  File tof = SD.open(tofFileName, FILE_WRITE);
  tof.print("time");
  for(int i=0; i<16; i++) tof.print(",D"+String(i));
  tof.println();
  tof.close();

  // Init OF CSV
  SD.remove(ofFileName);
  File of = SD.open(ofFileName, FILE_WRITE);
  of.print("time");
  for(int i=0; i<1225; i++) of.print(",P"+String(i));
  of.println();
  of.close();

  // Serve web page with button
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html",       "<h1>ESP32 Data Logger</h1>"
      "<a href='/download_imu'><button>Download IMU CSV</button></a><br>"
      "<a href='/download_tof'><button>Download ToF CSV</button></a><br>"
    "<a href='/download_of'><button>Download OF CSV</button></a>");
  });
  Serial.println("d");

  server.on("/download_imu", HTTP_GET, []() {
    File f = SD.open(imuFileName);
    if(f) { server.streamFile(f, "text/csv"); f.close(); }
    else server.send(404, "text/plain", "IMU file not found");
  });

  server.on("/download_tof", HTTP_GET, []() {
    File f = SD.open(tofFileName);
    if(f) { server.streamFile(f, "text/csv"); f.close(); }
    else server.send(404, "text/plain", "ToF file not found");
  });

  server.on("/download_of", HTTP_GET, []() {
    File f = SD.open(ofFileName);
    if(f) { server.streamFile(f, "text/csv"); f.close(); }
    else server.send(404, "text/plain", "OF file not found");
  });

  server.begin();
  Serial.println("e");

  Wire.begin(); // This resets I2C bus to 100kHz
  Wire.setClock(1000000); //Sensor has max I2C freq of 1MHz

  Serial.println("Clock Has been Set for I2C!");

 // myImager.setWireMaxPacketSize(128); // Increase default from 32 bytes to 128 - not supported on all platforms. Default is 32 bytes. 

  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  if (myImager.begin() == false)
  {
    Serial.println(F("ToF Sensor not found - check your wiring. Freezing"));
    while (1)
      ;
  }
  Serial.println("f");

  myImager.setResolution(8*8); // Enable all 64 pads or 16 pads for 4x4 resolution

  imageResolution = myImager.getResolution(); // Query sensor for current resolution - either 4x4 or 8x8

  // Using 4x4, min frequency is 1Hz and max is 60Hz
  // Using 8x8, min frequency is 1Hz and max is 15Hz
  myImager.setRangingFrequency(15);
  myImager.startRanging();

  Serial.println("Trigger LOW to Record data. Trigger HIGH to Stop and Download files.");


}

// Helper: append unsigned long to buffer, returns number of chars
int appendULong(char* buf, unsigned long val) {
    char temp[11]; // max 10 digits + null
    int i = 0;
    if(val == 0) {
        buf[0] = '0';
        return 1;
    }
    while(val > 0) {
        temp[i++] = '0' + (val % 10);
        val /= 10;
    }
    for(int j = 0; j < i; j++) buf[j] = temp[i - j - 1];
    return i;
}

// Write timestamp in seconds.microseconds
int appendTimestamp(char* buf, unsigned long micros_val) {
    unsigned long seconds = micros_val / 1000000;
    unsigned long us      = micros_val % 1000000;
    int idx = 0;
    idx += appendULong(buf + idx, seconds);
    buf[idx++] = '.';
    // Pad microseconds with leading zeros
    int digits = 6;
    char temp[6];
    for(int i = 5; i >= 0; i--) {
        temp[i] = '0' + (us % 10);
        us /= 10;
    }
    for(int i = 0; i < 6; i++) buf[idx++] = temp[i];
    return idx;
}


void logIMU() {
  unsigned long now = micros();
  if (now - lastIMUtime < imuInterval) return;  
  lastIMUtime = now;
  Serial.print("entering_imu:");
  Serial.printf("%.9f",now/1000000.0);
  if (!imuFile) return; // if file is not open; skip!

  inv_imu_sensor_data_t imu_data;
  IMU.getDataFromRegisters(imu_data);

  float ax = imu_data.accel_data[0]*G_rating/32768.0 - calibAccelX;
  float ay = imu_data.accel_data[1]*G_rating/32768.0 - calibAccelY;
  float az = imu_data.accel_data[2]*G_rating/32768.0 - calibAccelZ;
  float gx = imu_data.gyro_data[0]*dps_rating/32768.0 - calibGyroX;
  float gy = imu_data.gyro_data[1]*dps_rating/32768.0 - calibGyroY;
  float gz = imu_data.gyro_data[2]*dps_rating/32768.0 - calibGyroZ;

  int idx = appendTimestamp(imuBuf, now);
  idx += snprintf(imuBuf + idx, sizeof(imuBuf) - idx, ",%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n", ax, ay, az, gx, gy, gz);
  imuFile.write((uint8_t*)imuBuf, idx);
  Serial.print("leaving_imu:");
  now = micros();
  Serial.printf("%.9f",now/1000000.0);
  Serial.println("");
  // imuFile.flush(); // optional for safety -- check this out?
}

// Fast integer to string conversion, returns number of chars written
int intToStr(int val, char* buf) {
  char temp[6]; // max 3500 -> 4 digits + null
  int i = 0;
  if(val == 0) {
      buf[0] = '0';
      return 1;
  }
  while(val > 0) {
      temp[i++] = '0' + (val % 10);
      val /= 10;
  }
  // reverse
  for(int j = 0; j < i; j++) {
      buf[j] = temp[i - j - 1];
  }
  return i;
}

void logToF() {
  unsigned long now = micros();
  if (now - lastTOFtime < tofInterval) return;  
  lastTOFtime = now;
  Serial.print("entering_tof:");
  Serial.printf("%.9f",now/1000000.0);
  if (!tofFile) return;

  if(myImager.isDataReady() && myImager.getRangingData(&measurementData)) {
        Serial.print("      read:");
        now = micros();
        Serial.printf("%.9f",now/1000000.0);
        Serial.println("");
      int idx = appendTimestamp(tofBuf, now); // write timestamp
      for(int i = 0; i < 64; i++) { //8x8 = 64; 4x4 = 16
          tofBuf[idx++] = ',';                  
          idx += intToStr(measurementData.distance_mm[i], tofBuf + idx); // fast int -> string
      }
      tofBuf[idx++] = '\n';
      tofFile.write((uint8_t*)tofBuf, idx);  // write raw bytes
  }
  Serial.print("       leaving_tof:");
  now = micros();
  Serial.printf("%.9f",now/1000000.0);
  Serial.println("");
}

void logOF() {
  unsigned long now = micros();
  if (now - lastOFtime < ofInterval) return;  
  lastOFtime = now;

  Serial.print("entering_of:");
  Serial.printf("%.9f", now / 1000000.0);

  if (!ofFile) return;

  flow.readFrameBuffer(frame);

  Serial.print("      read:");
  now = micros();
  Serial.printf("%.9f", now / 1000000.0);
  Serial.println("");

  char line[4096];  // buffer for one frame
  int idx = 0;

  // Add timestamp
  idx = snprintf(line, sizeof(line), "%.9f", now / 1000000.0);

  // Add each pixel value
  for (int i = 0; i < 1225; i++) {
      idx += sprintf(line + idx, ",%d", frame[i]);
      if (idx >= sizeof(line) - 10) break; // safeguard
  }

  // Add newline
  line[idx++] = '\n';
  line[idx] = 0;

  // Write raw bytes to SD
  ofFile.write((uint8_t*)line, idx);

  Serial.print("      leaving_of:");
  now = micros();
  Serial.printf("%.9f", now / 1000000.0);
  Serial.println("");
}




// This method opens all the files when trigger is set LOW. The files stay open until trigger is HIGH (or Not Low - it is pulled high internally)
// This is to eliminate the time it takes to open and close the files each time for each sensor, which can make up few hundreds of us to low ms. 


void loop() {

    bool recording = (digitalRead(TRIGGER_PIN) == LOW);

    if (recording) {
        // --- Open files once when recording starts ---
        if (!imuFile) {
          imuFile = SD.open(imuFileName, FILE_APPEND);
          tofFile = SD.open(tofFileName, FILE_APPEND);
          ofFile = SD.open(ofFileName, FILE_APPEND);


            Serial.println("Opening files for logging...");
            if (!imuFile || !tofFile || !ofFile) {
                Serial.println("Failed to open one or more files!");
            }
        }

        // --- Write data ---
        logIMU();   // writes to imuFile
        logToF();   // writes to tofFile
        logOF();    // writes to ofFile
    } 
    else {
         server.handleClient(); // Handle web server clients only when not recording, to save time

        // --- Close files once when recording stops ---
        if (imuFile) {
            imuFile.close();
            imuFile = File(); // reset handle
        }
        if (tofFile) {
            tofFile.close();
            tofFile = File();
        }
        if (ofFile) {
            ofFile.close();
            ofFile = File();
            Serial.println("Files closed, safe to download.");
        }
    }

// Data is stored in RAM buffer temporarily before being written to SD card. Flushing forces the data being stored in the RAM buffer to be written to the SD card immediately.
// Frequent flushing can reduce data logging rate as it takes time to write to SD card, but flushing prevents data loss in case of power failure or unexpected reset.
// Investigate this. 

}
