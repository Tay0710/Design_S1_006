


Sensor pin assignment:

SDA : SDA pin (21)
SCL : SCL pin (22)
LPn : 3.3V (High) (This is the reset pin, need to use it with a GPIO pin when using mutlpile sensors on the I2C bus to chnage addresses.)
PWR_EN : 3.3V (High)
AVDD : 3.3V or 5V (used in voltage regulator, this and PWR_EN are required to make sensor work)
IOVDD : 3.3V
GND : GND



Notes from code: 
  delay(100); // Ensure full reset - this does not ensure a full reset of address back to 0x29. To do a full reset of the I2C, the I2C_RST pin must be pulled high then pull low again. This just tells the sensor to reset its address however the address at this point has still not been changed from XxXX to 0x29. TO change it to 0x29, the sensor must be powered-off then on again. (Which pin needs to be powered-on and off I do not know (testing is done in comment below), so far i have been powering off and on the whole board.)
  // Pin to disconnect, then reconnect is: (any one of the following) IOVDO, AVDD, PWREN, GND. 
  // So disconnecting and reconnecting (NOT CHANGING THE STATE!) of any of the power related pins is enough. 


      if (!sensor2.begin()) { // IF both sensors are active then it will identify both sensors and begin both of them. This means when the address is changed, both addresses for each sensor will be changed to 0x30 from 0x29. Therefore, have to ensure that only one sensor is active when the address change is made. 


  // --- Step 3: Start ranging on both sensors ---
  // Have to make sure both LPn pins are set high before starting ranging, otherwise you will get an Error message. 
  // If the ranging has already started, then does it need to be reset or can it be skipped?
    //    ANS: Both Sensors have to be reset, everytime ESP32 is restarted.  



Example of Data produced 8x8 array:

        34      33      37      39      47      51      55      1295
        35      38      44      46      53      57      1348    57
        42      45      49      54      57      66      64      68
        49      52      57      61      71      79      64      76
        56      64      67      73      83      90      64      70
        67      75      79      85      91      3871    59      68
        81      92      94      88      77      65      59      66
        85      95      123     84      62      58      69      67

        32      28      36      37      42      47      51      47
        37      36      42      43      50      56      61      54
        39      43      47      51      56      63      61      61
        45      51      54      60      67      73      66      64
        56      60      66      71      81      86      59      67
        68      72      78      84      87      80      60      63
        77      90      91      87      76      58      57      62
        81      93      118     92      58      58      58      64

        31      29      36      37      44      50      53      46
        33      36      42      42      49      54      59      56
        39      43      46      51      57      62      59      64
        45      50      56      58      68      76      59      64
        55      61      62      69      79      87      56      69
        67      72      78      85      89      76      61      69
        1657    88      93      85      70      66      63      70
        79      94      114     86      61      59      59      65

        33      28      35      38      41      51      52      50
        33      35      41      43      50      55      61      58
        38      43      47      54      57      65      60      63
        45      51      54      59      70      76      65      67
        56      60      719     71      83      88      63      72
        67      72      76      85      89      79      60      65
        2652    90      92      89      75      65      61      68
        82      88      122     93      67      61      69      68

        29      25      25      24      25      29      27      25
        29      21      23      23      27      28      26      28
        23      23      21      24      25      26      28      30
        27      23      23      24      26      28      29      33
        26      24      24      25      26      29      30      34
        29      24      24      26      29      32      33      35
        30      24      25      28      30      34      35      36
        33      28      26      29      33      35      34      34







Old Code: 


/*
  Read an 8x8 array of distances from the VL53L5CX
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 26, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to read all 64 distance readings at once.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642

*/

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

// LPn pins to control sensor power/reset
#define LPN_PIN_1 12
#define LPN_PIN_2 14

// VL53L5CX default and new I2C addresses
#define SENSOR1_ADDR 0x29
#define SENSOR2_ADDR 0x30

// Sensor objects and measurement data
SparkFun_VL53L5CX sensor1;
VL53L5CX_ResultsData measurementData1; // Result data class structure, 1356 byes of RAM

SparkFun_VL53L5CX sensor2;
VL53L5CX_ResultsData measurementData2;

int imageResolution1 = 0;
int imageWidth1 = 0;

int imageResolution2 = 0;
int imageWidth2 = 0;

// SparkFun_VL53L5CX myImager;
// VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

// int imageResolution = 0; //Used to pretty print output
// int imageWidth = 0; //Used to pretty print output

// #define LPN_PIN   39  // I don't think this is even needed. LPn just needs to be high for I2C to be active. 

void powerDownBoth() {
  digitalWrite(LPN_PIN_1, LOW);
  digitalWrite(LPN_PIN_2, LOW);
  delay(10);
}

void scanI2C() {
  Serial.println("Scanning I2C bus...");
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      count++;
    }
  }
  if (count == 0) Serial.println("No I2C devices found.");
  Serial.println();
}


void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("SparkFun VL53L5CX Dual Sensor Example");




  // // Configure LPn pins
  // pinMode(LPN_PIN_1, OUTPUT);
  // pinMode(LPN_PIN_2, OUTPUT);

  // // Power down both sensors
  // digitalWrite(LPN_PIN_1, LOW);
  // digitalWrite(LPN_PIN_2, LOW);
  delay(100); 

  Wire.begin(); // Initialize I2C bus
  Wire.setClock(400000); // Optional: 400 kHz I2C

  // // --- Step 1: Initialize Sensor 2 first (address 0x30) ---
  // digitalWrite(LPN_PIN_2, HIGH);
  // delay(50);

  Serial.println("SCAN #1");
  scanI2C(); // Scan I2C bus to find devices

  // Set sensor 2 to 0x30 before calling begin()
  Serial.println("Initializing Sensor 2 at 0x29...");

  Serial.println("SCAN #2");
  scanI2C(); // Scan I2C bus to find devices



  while (!Serial) ; // Wait for Serial Monitor to connect (USB boards)
  
  Serial.println("Make sensor2 LPn pin HIGH. Press Y to change I2C addresses, or N to skip:");
  
  // Wait until some input arrives
  while (!Serial.available()) {
    delay(10);
  }

  String input1 = Serial.readStringUntil('\n'); // Read until newline
  input1.trim(); // Remove spaces/newlines

  if (input1.equalsIgnoreCase("Y")) {
    Serial.println("Changing addresses of Sensor 2");
    if (!sensor2.begin()) { 
    Serial.println("Sensor 2 not found at 0x29!");
    while (1);
  }

  sensor2.setAddress(SENSOR2_ADDR);
  Serial.println("SCAN for Sensor 2 Address change, should be 0x30!");
  scanI2C(); // Scan I2C bus to find devices


  sensor2.setResolution(8 * 8);
  imageResolution2 = sensor2.getResolution();
  imageWidth2 = sqrt(imageResolution2);
  Serial.println("Sensor 2 initialized successfully at 0x30");


  } else {
    Serial.println("Skipping address change.");
  }





  // // --- Step 2: Initialize Sensor 1 (default 0x29) ---
  // digitalWrite(LPN_PIN_1, HIGH);
      Serial.println("You have 10 seconds to set LPN high!! Go GO GO!!");
  // delay(10000); // Wait for user to set LPN high for Sensor 1

    Serial.println("SCAN #3");
    scanI2C(); // Scan I2C bus to find devices
  delay(50);




  
  Serial.println("Make sensor 1 LPn pin HIGH (keep sensor 2 LPn Pin HIGH). Press Y to change I2C addresses, or N to skip:");
  
  // Wait until some input arrives
  while (!Serial.available()) {
    delay(10);
  }

  String inpu2 = Serial.readStringUntil('\n'); // Read until newline
  inpu2.trim(); // Remove spaces/newlines

  if (inpu2.equalsIgnoreCase("Y")) {
    Serial.println("Changing addresses...");
    Serial.println("Initializing Sensor 1 at 0x29...");
  if (!sensor1.begin()) {
    Serial.println("Sensor 1 not found at 0x29!");
    while (1);
  }

  sensor1.setResolution(8 * 8);
  imageResolution1 = sensor1.getResolution();
  imageWidth1 = sqrt(imageResolution1);
  Serial.println("Sensor 1 initialized successfully at 0x29");


  } else {
    Serial.println("Skipping address change.");
  }

    Serial.println("SCAN #4"); // Expect two device found, one at 0x29 and one at 0x30. 
    scanI2C(); // Scan I2C bus to find devices
  delay(50);



  Serial.println("Do both sensors need to start their ranging?. Press Y to change I2C addresses, or N to skip:");
  
  // Wait until some input arrives
  while (!Serial.available()) {
    delay(10);
  }

  String RangeInput = Serial.readStringUntil('\n'); // Read until newline
  RangeInput.trim(); // Remove spaces/newlines

  if (RangeInput.equalsIgnoreCase("Y")) {
    Serial.println("Starting ranging on both sensors...");
  sensor1.startRanging();
  sensor2.startRanging();

  } else {
    Serial.println("Skipping address change.");
  }

  Serial.println("Both sensors are now ranging.");
}



  // Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  // if (myImager.begin() == false)
  // {
  //   Serial.println(F("Sensor not found - check your wiring. Freezing"));
  //   while (1) ;
  // }

  // myImager.setResolution(8*8); //Enable all 64 pads

  // imageResolution = myImager.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  // imageWidth = sqrt(imageResolution); //Calculate printing width

  // myImager.startRanging();
//}

// void loop()
// {
//   //Poll sensor for new data
//   if (myImager.isDataReady() == true)
//   {
//     if (myImager.getRangingData(&measurementData)) //Read distance data into array
//     {
//       //The ST library returns the data transposed from zone mapping shown in datasheet
//       //Pretty-print data with increasing y, decreasing x to reflect reality
//       for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
//       {
//         for (int x = imageWidth - 1 ; x >= 0 ; x--)
//         {
//           Serial.print("\t");
//           Serial.print(measurementData.distance_mm[x + y]);
//         }
//         Serial.println();
//       }
//       Serial.println();
//     }
//   }

//   delay(5); //Small delay between polling
// }



void loop()
{
  // Sensor 1 data ready check and read
  if (sensor1.isDataReady()) {
    if (sensor1.getRangingData(&measurementData1)) {
      Serial.println("Sensor 1 data:");
      for (int y = 0; y <= imageWidth1 * (imageWidth1 - 1); y += imageWidth1) {
        for (int x = imageWidth1 - 1; x >= 0; x--) {
          Serial.print(measurementData1.distance_mm[x + y]);
          Serial.print("\t");
        }
        Serial.println();
      }
      Serial.println();
    }
  }

  // Sensor 2 data ready check and read
  if (sensor2.isDataReady()) {
    if (sensor2.getRangingData(&measurementData2)) {
      Serial.println("Sensor 2 data:");
      for (int y = 0; y <= imageWidth2 * (imageWidth2 - 1); y += imageWidth2) {
        for (int x = imageWidth2 - 1; x >= 0; x--) {
          Serial.print(measurementData2.distance_mm[x + y]);
          Serial.print("\t");
        }
        Serial.println();
      }
      Serial.println();
    }
  }

  delay(5); // Small delay between polling
}