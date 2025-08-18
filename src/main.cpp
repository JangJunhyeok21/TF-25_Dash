#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <stdint.h>
#include "LedController.hpp"
#include <Adafruit_NeoPixel.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <FreeRTOS.h>
#include "RTClib.h"


// SPI pins for LoRa
#define SCK_PIN 14     // GPIO14 (MTMS)
#define MISO_PIN 12    // GPIO12 (MTDI) 
#define MOSI_PIN 13    // GPIO13 (MTCK)
#define LORA_CS 5      // GPIO5 - NSS
#define LORA_RST 4     // GPIO4 - RESET
#define LORA_IRQ 42     // GPIO42 - DIO0

// Display pins (max7219) 
#define MAXCS 38        // GPIO38
LedController display = LedController(MOSI_PIN, SCK_PIN, MAXCS);

// NeoPixel LED strip
#define WS2812_PIN 40  // GPIO40
#define NUM_LEDS 19

// Suspension sensors (analog inputs)
#define CH1 6           // GPIO6 - Front Left
#define CH2 7           // GPIO7 - Front Right  
#define CH3 16          // GPIO16 - Rear Left
#define CH4 1           // GPIO1 - Rear Right

// Steering angle sensor
#define steering 2      // GPIO2 - Steering

// Power monitoring
#define powerCheckPin 15  // GPIO15

// I2C pins for MPU6050 and RTC
#define SCL_PIN 35     // GPIO35 - I2C Clock
#define SDA_PIN 36     // GPIO36 - I2C Data
#define MPU6050_ADDRESS 0x69 // MPU6050 I2C address
#define RTC_ADDRESS 0x68 // RTC I2C address
#define SQW_PIN 43 // GPIO43 - SQW pin for RTC

// GPS UART pins
#define RXPin 18       // GPIO18 - GPS TX
#define TXPin 17       // GPIO17 - GPS RX
static const uint32_t GPSBaud = 9600;

//Engine Signal
#define RPM_PIN 9      // GPIO9 - RPM Signal
#define ECT_PIN 20      // GPIO20 - Engine Temperature Signal
#define EOT_PIN 19      // GPIO19 - Engine Oil Temperature Signal
#define TPS 8           // GPIO8 - Throttle Position Sensor
#define MAX_RPM 11000 
#define MIN_RPM 4500  // RPM 게이지 시작 값

//SD Card pins
#define SD_CS 37       // GPIO37 - SD Card Chip Select
#define SD_HWCD 39     // GPIO39 - SD Card Hardware CDC

//gear input
#define G0 41        // GPIO41 - Gear 0
#define G1 48        // GPIO48 - Gear 1
#define G2 47        // GPIO47 - Gear 2

//etc pins
#define Fan_pin 21 // GPIO21 - Fan Control
#define mode_pin 44 // GPIO44 - Mode Switch
#define LED_pin 10 // GPIO10 - LED Control

//RTC 세팅
int microsCounter = 0; // 마이크로초 카운터

void IRAM_ATTR onSQWInterrupt() {
    microsCounter = micros(); // SQW 핀에서 인터럽트 발생 시 현재 마이크로초 시간 저장
}

//RPM 측정용 변수
volatile unsigned int pulseCounter = 0; // 인터럽트에서 사용할 펄스 카운터 (volatile 키워드 필수)
const int pulsesPerRevolution = 1;      // 엔진 1회전 당 발생하는 펄스 수

void IRAM_ATTR countPulse() {
    pulseCounter++;
}

/// 센서 데이터 구조체
struct SensorData
{
    int16_t accelX, accelY, accelZ, yawRate, coolantTemp, oilTemp;
    uint16_t rpm, susFL, susFR, susRL, susRR, speed;
    int32_t latitude, longitude;
    uint8_t tps, gear;
    DateTime timestamp;
};

// FreeRTOS 객체
QueueHandle_t sensorQueue;
SemaphoreHandle_t dataMutex;

// Adafruit MPU6050 객체
Adafruit_MPU6050 mpu;
// RTC모듈
RTC_DS1307 rtc;

Adafruit_NeoPixel leds(NUM_LEDS, WS2812_PIN, NEO_GRB + NEO_KHZ800);
TinyGPSPlus gps;             // GPS 객체 생성
// HardwareSerial gpsSerial(2); // Serial1 사용
// KWP2000 kwp(&kwpSerial);

// Task 함수 프로토타입
void SensorTask(void *pvParameters);
void LoggerTask(void *pvParameters);
void DisplayTask(void *pvParameters);
void TelemetryTask(void *pvParameters);
/*
소수점 시간 재는 함수-인터럽트 사용

*/

// // 전원종료 시 종료시퀀스
// void powerOffSequence()
// {
//     lora.end();      // LoRa 종료
//     kwpSerial.end(); // KWP 종료
//     Serial1.end();   // GPS 종료
// }

void setup()
{
    Serial.begin(115200);
    pinMode(RPM_PIN, INPUT_PULLUP); // 센서 신호 특성에 따라 INPUT 또는 INPUT_PULLUP 사용
    attachInterrupt(digitalPinToInterrupt(RPM_PIN), countPulse, RISING); // 펄스의 상승 엣지(Rising Edge)에서 인터럽트 발생

    // SQW 핀 인터럽트 등록
    pinMode(SQW_PIN, INPUT_PULLUP); // SQW 핀을 입력으로 설정
    attachInterrupt(digitalPinToInterrupt(SQW_PIN), onSQWInterrupt, RISING);
    // I2C 및 MPU6050 초기화
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!mpu.begin(MPU6050_ADDRESS, &Wire))
    {
        Serial.println("MPU6050 초기화 실패!");
        while (1)
        {
            Serial.println("MPU6050 초기화 실패!");
            vTaskDelay(pdMS_TO_TICKS(10)); // 10ms 대기
        }
    }
    Serial.println("MPU6050 연결 성공!");
    // RTC 초기화
    if (!rtc.begin())
    {
        while (1)
        {
            Serial.println("Couldn't find RTC");
            Serial.flush();
            vTaskDelay(pdMS_TO_TICKS(100)); // 100ms 대기
        }
    }
    Serial.println("RTC 연결 성공!");

    // --- 기어 입력 핀 설정 ---
    pinMode(G0, INPUT_PULLUP);
    pinMode(G1, INPUT_PULLUP);
    pinMode(G2, INPUT_PULLUP);
    // --- MAX7219 디스플레이 초기화 ---
    display.activateAllSegments();
    display.setIntensity(10); // 밝기 설정 (0-15)
    display.clearMatrix();


    // gps초기화
    // Ensure Serial1 is properly initialized and pins are configured
    // pinMode(RXPin, INPUT_PULLDOWN);
    // Serial2.begin(9600, SERIAL_8N1, RXPin, TXPin);
    // Serial.println("GPS 연결 성공!");
    
    //RPM게이지 세리모니
    leds.begin();
    leds.clear();
    // 1초 동안 차오르는 애니메이션
    for (int i = 0; i < NUM_LEDS; i++)
    {
        int red, green;
        if (i >= NUM_LEDS - 4) // 끝 4개는 빨강 고정
        {
            red = 255;
            green = 0;
        }
        else // 나머지는 그라데이션
        {
            red = map(i, 0, NUM_LEDS - 5, 255, 255); // 빨간색은 고정
            green = map(i, 0, NUM_LEDS - 5, 165, 0); // 초록색은 점점 감소
        }
        int blue = 0; // 파란색은 없음
        leds.setPixelColor(i, leds.Color(red, green, blue));
        leds.show();
        vTaskDelay(pdMS_TO_TICKS(1000 / NUM_LEDS));
    }
    // 1초 동안 가라앉는 애니메이션
    for (int i = NUM_LEDS - 1; i >= 0; i--)
    {
        leds.setPixelColor(i, leds.Color(0, 0, 0)); // 꺼짐
        leds.show();
        vTaskDelay(pdMS_TO_TICKS(1000 / NUM_LEDS));
    }

    // Queue 및 Mutex 생성
    sensorQueue = xQueueCreate(1, sizeof(SensorData));
    dataMutex = xSemaphoreCreateMutex();

    //SD 카드 초기화
    //CSV파일 생성 후 단위를 포함한 헤더 작성


    // FreeRTOS Task 생성
    xTaskCreatePinnedToCore(SensorTask, "SensorTask", 4096, NULL, 3, NULL, 1);
    // xTaskCreatePinnedToCore(LoggerTask, "LoggerTask", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(DisplayTask, "DisplayTask", 4096, NULL, 2, NULL, 0);
    // xTaskCreatePinnedToCore(TelemetryTask, "TelemetryTask", 4096, NULL, 2, NULL, 0);
}

void loop()
{
}
void SensorTask(void *pvParameters)
{
    SensorData data;
    sensors_event_t accelEvent, gyroEvent, tempEvent;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms 주기 (100Hz)
    int loopCounter = 0;

    for (;;)
    {
        // 10번 루프마다 (즉, 10 * 10ms = 100ms 마다) RPM 계산
        if (++loopCounter >= 10) {
            loopCounter = 0; // 카운터 리셋

            noInterrupts();
            unsigned int currentPulseCount = pulseCounter;
            pulseCounter = 0;
            interrupts();
            
            data.rpm = (currentPulseCount * 600) / pulsesPerRevolution;
        }
        // 센서 데이터 읽기
        mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
        data.accelX = (int16_t)(accelEvent.acceleration.x * 1000);
        data.accelY = (int16_t)(accelEvent.acceleration.y * 1000);
        data.accelZ = (int16_t)(accelEvent.acceleration.z * 1000);
        data.yawRate = (int16_t)(gyroEvent.gyro.z * 1000);
        data.timestamp = rtc.now();
        // while (Serial2.available() > 0)
        // {
        //     gps.encode(Serial2.read());
        // }
        // if (gps.location.isUpdated())
        // {
        //     data.longitude = gps.location.lng() * 1000000;
        //     data.latitude = gps.location.lat() * 1000000;
        //     data.speed = gps.speed.kmph();
        // }

        data.susFL = map(analogRead(CH1), 0, 4095, 0, 5000);
        data.susFR = map(analogRead(CH2), 0, 4095, 0, 5000);
        data.susRL = map(analogRead(CH3), 0, 4095, 0, 5000);
        data.susRR = map(analogRead(CH4), 0, 4095, 0, 5000);
        
        // ## 기어 단수 계산 로직 ##----------------------------------
        int gear_binary = (digitalRead(G2) << 2) | (digitalRead(G1) << 1) | digitalRead(G0);

        switch (gear_binary) {
            case 0b110: data.gear = 1; break;
            case 0b101: data.gear = 2; break;
            case 0b100: data.gear = 3; break;
            case 0b011: data.gear = 4; break;
            case 0b010: data.gear = 5; break;
            case 0b111: data.gear = 0; break;
            default:    data.gear = 6; break; // 0은 중립(Neutral)
        }
        //---------------------------------------------------------
        xQueueOverwrite(sensorQueue, &data);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void LoggerTask(void *pvParameters)
{
    SensorData latest;
    for (;;)
    {
        if (xQueueReceive(sensorQueue, &latest, portMAX_DELAY) == pdTRUE)
        {
            // SD 카드 기록 로직
        }
    }
}

void DisplayTask(void *pvParameters)
{
    SensorData latest;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz → 50ms로 확장

    for (;;)
    {
        if (xQueueReceive(sensorQueue, &latest, portMAX_DELAY) == pdTRUE)
        {
            int ledsToShow;

            // 1. 현재 RPM이 최소 RPM보다 낮은지 확인
            if (latest.rpm < MIN_RPM) {
                ledsToShow = 0; // 최소 RPM 미만이면 모든 LED를 끕니다.
            } else {
                // 2. 4500 ~ 11000 RPM 범위를 0 ~ 19개 LED로 매핑합니다.
                ledsToShow = map(latest.rpm, MIN_RPM, MAX_RPM, 0, NUM_LEDS);
            }
            // 3. (이하 로직 동일) 각 LED의 색상을 설정합니다.
            for (int i = 0; i < NUM_LEDS; i++)
            {
                if (i < ledsToShow) {
                    // [켜져야 할 LED]
                    int red, green;
                    if (i >= NUM_LEDS - 4) {
                        red = 255; green = 0;
                    } else {
                        red = 255; green = map(i, 0, NUM_LEDS - 5, 165, 0);
                    }
                    leds.setPixelColor(i, leds.Color(red, green, 0));
                } else {
                    // [꺼져야 할 LED]
                    leds.setPixelColor(i, leds.Color(0, 0, 0));
                }
            }
            leds.show();
            
            //7세그먼트 디스플레이 업데이트------------------------
            display.setDigit(0, 0, latest.gear, false);
            display.setDigit(0, 1, latest.coolantTemp/100, false);
            display.setDigit(0, 2, (latest.coolantTemp/10)%10, false);
            display.setDigit(0, 3, latest.coolantTemp%10, false);
            display.setDigit(0, 4, latest.oilTemp/100, false);
            display.setDigit(0, 5, (latest.oilTemp/10)%10, false);
            display.setDigit(0, 6, latest.oilTemp%10, false);
            ///------------------------------------------------

            // 데이터 출력
            char buffer[256];
            snprintf(buffer, sizeof(buffer),
                     "Timestamp: %04d/%02d/%02d %02d:%02d:%02d\n"
                     "Accel X: %.3f m/s^2, Y: %.3f m/s^2, Z: %.3f m/s^2\n"
                     "YawRate: %.3f rad/s\n",
                     latest.timestamp.year(), latest.timestamp.month(), latest.timestamp.day(),
                     latest.timestamp.hour(), latest.timestamp.minute(), latest.timestamp.second(),
                     latest.accelX / 1000.0, latest.accelY / 1000.0, latest.accelZ / 1000.0,
                     latest.yawRate / 1000.0);
            Serial.print(buffer);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TelemetryTask(void *pvParameters)
{
    SensorData latest;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100ms 주기 (10Hz)

    for (;;)
    {
        //     if (xQueueReceive(sensorQueue, &latest, portMAX_DELAY) == pdTRUE)
        //     {
        //         // LoRa 전송 로직
        //         LoRa.beginPacket();
        //         LoRa.print(latest.accelX);      LoRa.print(",");
        //         LoRa.print(latest.accelY);      LoRa.print(",");
        //         LoRa.print(latest.accelZ);      LoRa.print(",");
        //         LoRa.print(latest.speed);       LoRa.print(",");
        //         LoRa.print(latest.latitude);    LoRa.print(",");
        //         LoRa.print(latest.longitude);   LoRa.print(",");
        //         LoRa.print(latest.rpm);         LoRa.print(",");
        //         LoRa.print(latest.susFL);       LoRa.print(",");
        //         LoRa.print(latest.susFR);       LoRa.print(",");
        //         LoRa.print(latest.susRL);       LoRa.print(",");
        //         LoRa.print(latest.susRR);       LoRa.print(",");
        //         LoRa.print(latest.tps);         LoRa.print(",");
        //         LoRa.print(latest.gear);        LoRa.print(",");
        //         LoRa.print(latest.coolantTemp); LoRa.endPacket();
        //     }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
