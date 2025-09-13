/**
 * @file      boards.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  ShenZhen XinYuan Electronic Technology Co., Ltd
 * @date      2024-04-24
 * @last-update 2025-05-26
 *
 */

#include "LoRaBoards.h"

#if defined(HAS_SDCARD) && !defined(SD_SHARE_SPI_BUS)
SPIClass SDCardSPI(HSPI);
#endif

#if defined(ARDUINO_ARCH_ESP32)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
#include "hal/gpio_hal.h"
#endif
#include "driver/gpio.h"
#endif //ARDUINO_ARCH_ESP32


DISPLAY_MODEL *u8g2 = NULL;
static DevInfo_t  devInfo;


uint32_t deviceOnline = 0x00;


static void enable_slow_clock();

bool beginDisplay()
{
    Wire.beginTransmission(DISPLAY_ADDR);
    if (Wire.endTransmission() == 0) {
        Serial.printf("Find Display model at 0x%X address\n", DISPLAY_ADDR);
        u8g2 = new DISPLAY_MODEL(U8G2_R0, U8X8_PIN_NONE);
        u8g2->begin();
        u8g2->clearBuffer();
        u8g2->setFont(u8g2_font_inb19_mr);
        u8g2->drawStr(0, 30, "LilyGo");
        u8g2->drawHLine(2, 35, 47);
        u8g2->drawHLine(3, 36, 47);
        u8g2->drawVLine(45, 32, 12);
        u8g2->drawVLine(46, 33, 12);
        u8g2->setFont(u8g2_font_inb19_mf);
        u8g2->drawStr(58, 60, "LoRa");
        u8g2->sendBuffer();
        u8g2->setFont(u8g2_font_fur11_tf);
        delay(3000);
        return true;
    }

    Serial.printf("Warning: Failed to find Display at 0x%0X address\n", DISPLAY_ADDR);
    return false;
}

#ifdef HAS_SDCARD
bool writeFile(const char *path, const char *buffer)
{
    bool rlst = false;
    File file = SD.open(path, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return false;
    }
    if (file.print(buffer)) {
        Serial.println("File written");
        rlst = true;
    } else {
        Serial.println("Write failed");
        rlst = false;
    }
    file.close();
    return  rlst;
}


bool readFile(const char *path, uint8_t *buffer, size_t size)
{
    File file = SD.open(path, FILE_READ);
    if (!file) {
        Serial.println("Failed to open file for reading");
        return false;
    }
    file.read(buffer, size);
    file.close();
    return false;
}

bool testSDWriteAndRead()
{
    const char *path = "/test_sd.txt";
    const char *message = "This is a string for reading and writing SD card.";
    uint8_t buffer[128] = {0};

    if (!writeFile(path, message)) {
        Serial.println("SD Text write failed");
        return false;
    }
    delay(100);

    readFile(path, buffer, 128);

    if (memcmp(buffer, message, strlen(message)) != 0) {
        Serial.println("SD verification failed");
        return false;
    }
    Serial.println("SD verification successful");
    return true;
}
#endif /*HAS_SDCARD*/

bool beginSDCard()
{
#ifdef HAS_SDCARD
    bool rlst = SD.begin(SDCARD_CS, SDCardSPI);


    if (rlst) {
        uint32_t cardSize = SD.cardSize() / (1024 * 1024);
        Serial.print("Sd Card init succeeded, The current available capacity is ");
        Serial.print(cardSize / 1024.0);
        Serial.println(" GB");
        deviceOnline |= SDCARD_ONLINE;
        return testSDWriteAndRead();
    } else {
        Serial.println("Warning: Failed to init Sd Card");
    }
#endif /*HAS_SDCARD*/
    return false;
}



void printWakeupReason()
{
#ifdef ARDUINO_ARCH_ESP32
    Serial.print("Reset reason:");
    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        Serial.println(" In case of deep sleep, reset was not caused by exit from deep sleep");
        break;
    case ESP_SLEEP_WAKEUP_ALL :
        break;
    case ESP_SLEEP_WAKEUP_EXT0 :
        Serial.println("Wakeup caused by external signal using RTC_IO");
        break;
    case ESP_SLEEP_WAKEUP_EXT1 :
        Serial.println("Wakeup caused by external signal using RTC_CNTL");
        break;
    case ESP_SLEEP_WAKEUP_TIMER :
        Serial.println("Wakeup caused by timer");
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD :
        Serial.println("Wakeup caused by touchpad");
        break;
    case ESP_SLEEP_WAKEUP_ULP :
        Serial.println("Wakeup caused by ULP program");
        break;
    default :
        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
        break;
    }
#endif
}


void getChipInfo()
{
#if defined(ARDUINO_ARCH_ESP32)

    Serial.println("-----------------------------------");

    printWakeupReason();


    if (psramFound()) {
        uint32_t psram = ESP.getPsramSize();
        devInfo.psramSize = psram / 1024.0 / 1024.0;
        Serial.printf("PSRAM is enable! PSRAM: %.2fMB\n", devInfo.psramSize);
        deviceOnline |= PSRAM_ONLINE;
    } else {
        Serial.println("PSRAM is disable!");
        devInfo.psramSize = 0;
    }


    Serial.print("Flash:");
    devInfo.flashSize       = ESP.getFlashChipSize() / 1024.0 / 1024.0;
    devInfo.flashSpeed      = ESP.getFlashChipSpeed() / 1000 / 1000;
    devInfo.chipModel       = ESP.getChipModel();
    devInfo.chipModelRev    = ESP.getChipRevision();
    devInfo.chipFreq        = ESP.getCpuFreqMHz();

    Serial.print(devInfo.flashSize);
    Serial.println(" MB");
    Serial.print("Flash speed:");
    Serial.print(devInfo.flashSpeed);
    Serial.println(" M");
    Serial.print("Model:");

    Serial.println(devInfo.chipModel);
    Serial.print("Chip Revision:");
    Serial.println(devInfo.chipModelRev);
    Serial.print("Freq:");
    Serial.print(devInfo.chipFreq);
    Serial.println(" MHZ");
    Serial.print("SDK Ver:");
    Serial.println(ESP.getSdkVersion());
    Serial.print("DATE:");
    Serial.println(__DATE__);
    Serial.print("TIME:");
    Serial.println(__TIME__);

    uint8_t mac[6];
    char macStr[18] = { 0 };
    esp_efuse_mac_get_default(mac);
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.print("EFUSE MAC: ");
    Serial.print(macStr);
    Serial.println();

    Serial.println("-----------------------------------");

#endif
}



void setupBoards(bool disable_u8g2 )
{
    Serial.begin(115200);

    Serial.println("setupBoards");

    getChipInfo();

    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);

    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, LED_ON);

    // Perform an I2C scan after power-on operation
#ifdef I2C_SDA
    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.println("Scan Wire...");
    scanDevices(&Wire);
#endif

    if (!disable_u8g2) {
        beginDisplay();
    }

    Serial.println("init done . ");
}


void printResult(bool radio_online)
{
    Serial.print("Radio        : ");
    Serial.println((radio_online) ? "+" : "-");

#if defined(CONFIG_IDF_TARGET_ESP32)  ||  defined(CONFIG_IDF_TARGET_ESP32S3)

    Serial.print("PSRAM        : ");
    Serial.println((psramFound()) ? "+" : "-");

    Serial.print("Display      : ");
    Serial.println(( u8g2) ? "+" : "-");

#ifdef HAS_SDCARD
    Serial.print("Sd Card      : ");
    Serial.println((SD.cardSize() != 0) ? "+" : "-");
#endif


    if (u8g2) {

        u8g2->clearBuffer();
        u8g2->setFont(u8g2_font_NokiaLargeBold_tf );
        uint16_t str_w =  u8g2->getStrWidth(BOARD_VARIANT_NAME);
        u8g2->drawStr((u8g2->getWidth() - str_w) / 2, 16, BOARD_VARIANT_NAME);
        u8g2->drawHLine(5, 21, u8g2->getWidth() - 5);

        u8g2->drawStr( 0, 38, "Disp:");     u8g2->drawStr( 45, 38, ( u8g2) ? "+" : "-");

#ifdef HAS_SDCARD
        u8g2->drawStr( 0, 54, "SD :");      u8g2->drawStr( 45, 54, (SD.cardSize() != 0) ? "+" : "-");
#endif

        u8g2->drawStr( 62, 38, "Radio:");    u8g2->drawStr( 120, 38, ( radio_online ) ? "+" : "-");

        u8g2->sendBuffer();

        delay(2000);
    }
#endif
}


#ifdef BOARD_LED
static uint8_t ledState = LOW;
static const uint32_t debounceDelay = 50;
static uint32_t lastDebounceTime = 0;
#endif


void flashLed()
{
#ifdef BOARD_LED
    if ((millis() - lastDebounceTime) > debounceDelay) {
        ledState = !ledState;
        if (ledState) {
            digitalWrite(BOARD_LED, LED_ON);
        } else {
            digitalWrite(BOARD_LED, !LED_ON);
        }
        lastDebounceTime = millis();
    }
#endif
}


void scanDevices(TwoWire *w)
{
    uint8_t err, addr;
    int nDevices = 0;
    uint32_t start = 0;

    Serial.println("I2C Devices scanning");
    for (addr = 1; addr < 127; addr++) {
        start = millis();
        w->beginTransmission(addr); delay(2);
        err = w->endTransmission();
        if (err == 0) {
            nDevices++;
            switch (addr) {
            case 0x77:
            case 0x76:
                Serial.println("\tFind BMX280 Sensor!");
                deviceOnline |= BME280_ONLINE;
                break;
            case 0x34:
                Serial.println("\tFind AXP192/AXP2101 PMU!");
                deviceOnline |= POWERMANAGE_ONLINE;
                break;
            case 0x3C:
                Serial.println("\tFind SSD1306/SH1106 display!"); // Note: only this is used
                deviceOnline |= DISPLAY_ONLINE;
                break;
            case 0x51:
                Serial.println("\tFind PCF8563 RTC!");
                deviceOnline |= PCF8563_ONLINE;
                break;
            case 0x1C:
                Serial.println("\tFind QMC6310 MAG Sensor!");
                deviceOnline |= QMC6310_ONLINE;
                break;
            default:
                Serial.print("\tI2C device found at address 0x");
                if (addr < 16) {
                    Serial.print("0");
                }
                Serial.print(addr, HEX);
                Serial.println(" !");
                break;
            }

        } else if (err == 4) {
            Serial.print("Unknow error at address 0x");
            if (addr < 16) {
                Serial.print("0");
            }
            Serial.println(addr, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");

    Serial.println("Scan devices done.");
    Serial.println("\n");
}