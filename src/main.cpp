#include <Arduino.h>
#include <FreeRTOS.h>
#include <SPI.h>
#include <Wire.h>

// --- 라이브러리 헤더 ---
#include <Adafruit_NeoPixel.h>
#include "LedController.hpp" // 7-Segment 라이브러리
#include <SdFat.h>           // SD 카드 라이브러리
#include "RTClib.h"          // RTC 라이브러리
#include <TinyGPSPlus.h>     // GPS 라이브러리
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- 핀 및 설정 정의 ---
// 공유 SPI 버스 장치 CS 핀
#define SD_CS   37
#define LORA_CS 5
#define MAXCS   38 // 7-Segment 및 74HCT245 버퍼 제어

// 커스텀 SPI 버스 핀
#define SCK_PIN  14
#define MISO_PIN 12
#define MOSI_PIN 13

// I2C 핀
#define SCL_PIN 35
#define SDA_PIN 36

// RPM 센서
#define RPM_PIN 9
#define MAX_RPM 11000
#define MIN_RPM 4500
const int pulsesPerRevolution = 1;

// 기어 입력 핀
#define G0 41
#define G1 48
#define G2 47

// 유온(EOT) 센서
#define EOT_PIN 19
#define B_COEFFICIENT 4250
#define NOMINAL_RESISTANCE 50000
#define FIXED_RESISTOR 50000
#define NOMINAL_TEMPERATURE 25

// NeoPixel RPM 게이지
#define WS2812_PIN 40
#define NUM_LEDS 19

// GPS
#define RXPin 18
#define TXPin 17

#define LED_PIN 10 // 상태 표시 LED

// --- 하드웨어 객체 생성 ---
Adafruit_NeoPixel leds(NUM_LEDS, WS2812_PIN, NEO_GRB + NEO_KHZ800);
LedController display = LedController(43, 39, MAXCS);
SdFat sd;
FsFile dataFile;
RTC_DS3231 rtc;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
Adafruit_MPU6050 mpu;

// --- 데이터 구조체 및 FreeRTOS 객체 ---
struct SensorData {
    char timestamp[25];
    float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
    uint16_t rpm, oilTemp;
    uint8_t gear;
    // --- 아직 테스트하지 않은 센서 데이터 ---
    // uint16_t susFL, susFR, susRL, susRR;
    // uint16_t coolantTemp, tps, steeringAngle;
    double latitude, longitude;
    uint16_t speed;
};
QueueHandle_t sensorQueue;
SemaphoreHandle_t spiMutex; // ★ SPI 버스 보호를 위한 Mutex 추가

// --- 전역 변수 ---
volatile unsigned int pulseCounter = 0;
bool rtcUpdatedByGps = false;
char logFilePath[40];

// --- Task 함수 프로토타입 ---
void SensorTask(void *pvParameters);
void LoggerTask(void *pvParameters);
void DisplayTask(void *pvParameters);

// --- 인터럽트 서비스 루틴 ---
void IRAM_ATTR countPulse() {
    pulseCounter++;
}


void setup() {
    Serial.begin(115200);
    delay(1000); // 시스템 안정화 및 시리얼 모니터 연결 시간
    Serial.println("\n--- Student Formula DAQ System Booting ---");

    // --- ★ 진단을 위해 GPS 초기화를 맨 앞으로 이동 ---
    // 만약 이 코드 직후에 리셋된다면, 원인은 100% UART1 핀(17, 18)의 하드웨어 충돌입니다.
    Serial.println("Attempting to initialize GPS Serial Port...");
    gpsSerial.begin(9600, SERIAL_8N1, RXPin, TXPin);
    Serial.println("GPS Serial Port Initialized.");

    // --- 핀 모드 및 인터럽트 설정 ---
    pinMode(RPM_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(RPM_PIN), countPulse, RISING);
    pinMode(G0, INPUT); pinMode(G1, INPUT); pinMode(G2, INPUT);
    pinMode(EOT_PIN, INPUT);

    sensorQueue = xQueueCreate(1, sizeof(SensorData)); // ★ 큐 크기를 1로 변경
    spiMutex = xSemaphoreCreateMutex();

    // --- ★★★ 공유 SPI 버스 초기화 (가장 중요) ★★★ ---
    Serial.println("Initializing Shared SPI Bus...");
    delay(100); // SD카드 전원 안정화 시간
    // 1. 다른 SPI 장치들 먼저 비활성화
    pinMode(LORA_CS, OUTPUT); digitalWrite(LORA_CS, HIGH);
    // pinMode(MAXCS, OUTPUT);   digitalWrite(MAXCS, HIGH);
    // 2. 커스텀 핀으로 SPI 버스 초기화
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, -1);
    // 3. SD 카드 초기화
    if (!sd.begin(SD_CS, SPI_QUARTER_SPEED)) {
        digitalWrite(LED_PIN, HIGH); // 오류 표시
        Serial.println("SD Card initialization failed! Halting.");
        // sd.initErrorHalt(&Serial);
    }else{
        Serial.println("SD Card Initialized.");
        xTaskCreatePinnedToCore(LoggerTask, "LoggerTask", 8192, NULL, 3, NULL, 1);
        Serial.println("LoggerTask Created.");
    }
    Serial.println("Shared SPI Bus Initialized Successfully.");

    // --- I2C 버스 및 센서 초기화 ---
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!rtc.begin()) { Serial.println("RTC not found! Halting."); while(1); }
    if (!mpu.begin(0x69, &Wire)) { Serial.println("MPU6050 not found! Halting."); while(1); }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG); // 각속도 범위 설정
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // 저역통과 필터 대역폭 설정
    Serial.println("I2C Bus: RTC & MPU6050 Initialized.");
    
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    

    // --- 디스플레이 초기화 ---
    leds.begin(); leds.clear(); leds.show();
    display.activateAllSegments(); display.setIntensity(10); display.clearMatrix();
    Serial.println("Displays Initialized.");

    // --- 로그 파일 생성 ---
    DateTime now = rtc.now();
    char dirPath[12]; sprintf(dirPath, "/%04d%02d%02d", now.year(), now.month(), now.day());
    if (!sd.exists(dirPath)) { sd.mkdir(dirPath); }
    sprintf(logFilePath, "%s/%02d%02d%02d.csv", dirPath, now.hour(), now.minute(), now.second());
    dataFile.open(logFilePath, FILE_WRITE);
    if (dataFile) {
        // CSV 헤더 작성
        dataFile.println("Timestamp,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,RPM,OilTemp,Gear,Latitude,Longitude,Speed");
        dataFile.close();
        Serial.printf("Logging to: %s\n", logFilePath);
    } else {
        Serial.println("Failed to create log file! Halting.");
    }
    
    // --- FreeRTOS 설정 ---
    xTaskCreatePinnedToCore(SensorTask, "SensorTask", 4096, NULL, 2, NULL, 0);
    Serial.println("SensorTask Created.");
    xTaskCreatePinnedToCore(DisplayTask, "DisplayTask", 4096, NULL, 1, NULL, 1);
    Serial.println("DisplayTask Created.");

    Serial.println("--- System Ready ---");
}

void loop() {
    // 메인 루프는 비워둡니다. 모든 작업은 FreeRTOS 태스크에서 처리됩니다.
}

/**
 * @brief 50Hz (20ms) 주기로 모든 센서 데이터를 읽어 큐로 전송하는 태스크
 */
void SensorTask(void *pvParameters) {
    SensorData data;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int rpmLoopCounter = 0;
    int tempLoopCounter = 0; // ★ 유온 계산 주기를 위한 카운터 추가

    for (;;) {
        // --- 시간 및 GPS 데이터 ---
        while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());
        if (!rtcUpdatedByGps && gps.date.isValid() && gps.time.isValid() && gps.date.year() > 2020) {
            rtc.adjust(DateTime(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second()));
            rtcUpdatedByGps = true;
        }
        DateTime now = rtc.now();
        sprintf(data.timestamp, "%02d:%02d:%02d.%03d", now.hour(), now.minute(), now.second(), (millis() % 1000));
        // --- GPS 좌표/속도 저장 ---
        if (gps.location.isValid()) {
            data.latitude = gps.location.lat();
            data.longitude = gps.location.lng();
        } else {
            data.latitude = 0;
            data.longitude = 0;
        }
        if (gps.speed.isValid()) {
            data.speed = gps.speed.kmph();
        } else {
            data.speed = 0;
        }
        // --- MPU6050 ---
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        data.accelX = a.acceleration.x; data.accelY = a.acceleration.y; data.accelZ = a.acceleration.z;
        data.gyroX = g.gyro.x; data.gyroY = g.gyro.y; data.gyroZ = g.gyro.z;

        
        if (++rpmLoopCounter >= 5) { // 20ms * 5 = 100ms
            rpmLoopCounter = 0;
            noInterrupts();
            unsigned int currentPulseCount = pulseCounter;
            pulseCounter = 0;
            interrupts();
            // 100ms 동안 측정된 펄스 수를 rpm으로 변환
            // 1분 = 60,000ms, 100ms 동안의 펄스 수 × (60,000 / 100) = ×600
            float rpmCalc = (currentPulseCount * 60000.0f) / (100.0f * pulsesPerRevolution);
            data.rpm = static_cast<uint16_t>(rpmCalc + 0.5f);
        }
        
        // --- 기어 단수 ---
        /*
        011	0
        110	1
        001	2
        111	3 또는 6
        101	4
        010	5
        */
        int g0 = digitalRead(G0);
        int g1 = digitalRead(G1);
        int g2 = digitalRead(G2);
        
        // 3비트 조합을 계산 (g2g1g0)
        int gearBits = (g2 << 2) | (g1 << 1) | g0;
        
        // 이전 기어 값을 임시 저장
        uint8_t prevGear = data.gear;
        
        switch (gearBits) {
            case 0b011:  // 011 -> 0단
                data.gear = 0;
                break;
            case 0b110:  // 110 -> 1단
                data.gear = 1;
                break;
            case 0b001:  // 001 -> 2단
                data.gear = 2;
                break;
            case 0b111:  // 111 -> 3단 또는 6단 (이전 기어에 따라 결정)
                if (prevGear >= 5) {
                    data.gear = 6;
                } else {
                    data.gear = 3;
                }
                break;
            case 0b101:  // 101 -> 4단
                data.gear = 4;
                break;
            case 0b010:  // 010 -> 5단
                data.gear = 5;
                break;
            default:
                // 유효하지 않은 조합의 경우 이전 기어 유지
                data.gear = prevGear;
                break;
        }


        // --- 유온 (NTC) - 2Hz로 계산 (50ms * 10 = 500ms) ---
        if (++tempLoopCounter >= 10) {
            tempLoopCounter = 0;
            int adcValue = analogRead(EOT_PIN);
            if (adcValue > 10) {
                float resistance = FIXED_RESISTOR * ((4095.0 / adcValue) - 1.0);
                float steinhart = log(resistance / NOMINAL_RESISTANCE) / B_COEFFICIENT + 1.0 / (NOMINAL_TEMPERATURE + 273.15);
                data.oilTemp =(uint16_t)((float)data.oilTemp*0.8)+(((1.0 / steinhart) - 273.15)*0.2);
            } else { data.oilTemp = 0; }
        }
        
        xQueueOverwrite(sensorQueue, &data); // ★ 큐에 최신 데이터를 덮어씁니다.
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20)); // 50Hz 주기 유지
    }
}

/**
 * @brief 큐에서 최신 데이터를 주기적으로 받아와 SD 카드에 기록하는 태스크 (저속 로깅)
 */
void LoggerTask(void *pvParameters) {
    // ★★★ 로거 로직 변경 ★★★
    // 큐가 덮어쓰기 방식으로 변경되었으므로, 고속(20Hz)의 모든 데이터를 버퍼링할 수 없습니다.
    // 대신, 약 10Hz(100ms) 주기로 최신 데이터를 하나씩 기록합니다.
    SensorData dataToLog;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // 큐에서 최신 데이터를 하나 받아옵니다.
        if (xQueueReceive(sensorQueue, &dataToLog, portMAX_DELAY)) {
                // digitalWrite(MAXCS, HIGH); 
                // digitalWrite(LORA_CS, HIGH);

                // ★ Mutex로 SPI 버스 접근 보호
            if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
                if (dataFile.open(logFilePath, FILE_WRITE)) {
                    dataFile.printf("%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%.6f,%.6f,%.2f\n",
                        dataToLog.timestamp,
                        dataToLog.accelX, dataToLog.accelY, dataToLog.accelZ,
                        dataToLog.gyroX, dataToLog.gyroY, dataToLog.gyroZ,
                        dataToLog.rpm, dataToLog.oilTemp, dataToLog.gear,
                        dataToLog.latitude, dataToLog.longitude, dataToLog.speed);
                    dataFile.close();
                }
                xSemaphoreGive(spiMutex); // ★ 작업 후 Mutex 반환
            }
        }
        // 약 10Hz 주기로 로깅을 시도합니다.
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

/**
 * @brief 큐에서 데이터를 받아와 디스플레이를 업데이트하는 태스크 (약 30Hz)
 */
void DisplayTask(void *pvParameters) {
    SensorData latestData;

    for (;;) {
        if (xQueueReceive(sensorQueue, &latestData, portMAX_DELAY) == pdTRUE) {
            // --- LED RPM 게이지 업데이트 ---
            int ledsToShow = (latestData.rpm < MIN_RPM) ? 0 : map(latestData.rpm, MIN_RPM, MAX_RPM, 0, NUM_LEDS);
            for (int i = 0; i < NUM_LEDS; i++) {
                if (i < ledsToShow) {
                    int red = 255;
                    int green = (i >= NUM_LEDS - 4) ? 0 : map(i, 0, NUM_LEDS - 5, 165, 0);
                    leds.setPixelColor(i, leds.Color(red, green, 0));
                } else {
                    leds.setPixelColor(i, leds.Color(0, 0, 0));
                }
            }
            // 경고: 유온이 95도 이상이면 마지막 LED를 항상 빨간색으로 설정
            if (latestData.oilTemp >= 95) {
                leds.setPixelColor(NUM_LEDS - 1, leds.Color(255, 0, 0));
            }
            leds.show();

            // ★ Mutex로 SPI 버스 접근 보호
            if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
                // 7-Segment 업데이트를 위해 CS핀 제어
                digitalWrite(MAXCS, LOW);

                if (latestData.gear == 0) display.setChar(0, 6, '-', false);
                else display.setDigit(0, 6, latestData.gear, false);

                uint16_t temp = latestData.oilTemp;
                if (temp > 999) temp = 999;
                
                // 온도가 100 미만일 때 앞자리 공백 처리
                display.setChar(0, 3, (temp >= 100) ? (temp / 100) + '0' : ' ', false);
                display.setChar(0, 2, (temp >= 10) ? ((temp / 10) % 10) + '0' : ' ', false);
                display.setDigit(0, 1, temp % 10, false);
                
                digitalWrite(MAXCS, HIGH);
                xSemaphoreGive(spiMutex); // ★ 작업 후 Mutex 반환
            }
            Serial.printf("%s | Accel: [%.2f, %.2f, %.2f] | Gyro: [%.2f, %.2f, %.2f] | RPM: %u | Oil: %u | Gear: %u | GPS: %.6f, %.6f | %.2f km/h\n",
            latestData.timestamp,
            latestData.accelX, latestData.accelY, latestData.accelZ,
            latestData.gyroX, latestData.gyroY, latestData.gyroZ,
            latestData.rpm, latestData.oilTemp, latestData.gear,
            latestData.latitude, latestData.longitude, latestData.speed);
        }
    }
}