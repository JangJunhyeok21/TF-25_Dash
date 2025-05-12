#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <stdint.h>
#include <TM1637Display.h>
#include <Adafruit_NeoPixel.h>
#include <TinyGPSPlus.h>
#include <FreeRTOS.h>
#include "RTClib.h"
#include "KWP2000.h"

#define SCK_PIN 14
#define MISO_PIN 12
#define MOSI_PIN 13
#define LORA_CS 5  // NSS
#define LORA_RST 4 // RESET
#define LORA_IRQ 2 // DIO0
#define CLK 10
#define DIO 11
#define WS2812_PIN 18
#define NUM_LEDS 16
#define KWP_RX 8
#define KWP_TX 9
#define FL 6
#define FR 7
#define RL 15
#define RR 16
#define stearing 17
#define gpsRX 19
#define gpsTX 20
#define powerCheckPin A0

// attachInterrupt(digitalPinToInterrupt(powerCheckPin), []() {
//     if (analogRead(powerCheckPin) < 930) // 930 corresponds to ~2.8V on a 3.3V ADC scale
//     {
//         powerOffSequence();
//     }
// }, FALLING);

/// 센서 데이터 구조체
struct SensorData
{
    int16_t accelX, accelY, accelZ, yawRate, coolantTemp;
    uint16_t rpm, susFL, susFR, susRL, susRR, speed;
    int32_t latitude, longitude;
    uint8_t tps, gear;
    DateTime timestamp;
    bool DTC;
};

// FreeRTOS 객체
QueueHandle_t sensorQueue;
SemaphoreHandle_t dataMutex;

// Adafruit MPU6050 객체
Adafruit_MPU6050 mpu;
// RTC모듈
RTC_DS1307 rtc;

// TM1637Display display(CLK, DIO);
// Adafruit_NeoPixel leds(NUM_LEDS, WS2812_PIN, NEO_GRB + NEO_KHZ800);
// TinyGPSPlus gps; // GPS 객체 생성
// HardwareSerial kwpSerial(1); // UART1 사용
// KWP2000 kwp(&kwpSerial);

// Task 함수 프로토타입
void SensorTask(void *pvParameters);
void LoggerTask(void *pvParameters);
void DisplayTask(void *pvParameters);
void TelemetryTask(void *pvParameters);

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
    // I2C 및 MPU6050 초기화
    Wire.begin(36, 35);
    if (!mpu.begin(0x69, &Wire))
    {
        Serial.println("MPU6050 초기화 실패!");
        while (1)
        {
            Serial.println("MPU6050 초기화 실패!");
            delay(10);
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
            delay(10);
        }
    }

    // Queue 및 Mutex 생성
    sensorQueue = xQueueCreate(1, sizeof(SensorData));
    dataMutex = xSemaphoreCreateMutex();

    // FreeRTOS Task 생성
    xTaskCreatePinnedToCore(SensorTask, "SensorTask", 2048, NULL, 3, NULL, 1);
    // xTaskCreatePinnedToCore(LoggerTask, "LoggerTask", 2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(DisplayTask, "DisplayTask", 2048, NULL, 2, NULL, 0);
    // xTaskCreatePinnedToCore(TelemetryTask, "TelemetryTask", 2048, NULL, 2, NULL, 0);
}

void loop()
{
}

void SensorTask(void *pvParameters)
{
    SensorData data;
    sensors_event_t accelEvent, gyroEvent, tempEvent;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms 주기 (50Hz)

    for (;;)
    {
        // 센서 데이터 읽기
        mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
        data.accelX = (int16_t)(accelEvent.acceleration.x * 1000);
        data.accelY = (int16_t)(accelEvent.acceleration.y * 1000);
        data.accelZ = (int16_t)(accelEvent.acceleration.z * 1000);
        data.yawRate = (int16_t)(gyroEvent.gyro.z * 1000);
        data.timestamp = rtc.now();
        data.susFL = map(analogRead(FL), 0, 4095, 0, 5000);
        data.susFR = map(analogRead(FR), 0, 4095, 0, 5000);
        data.susRL = map(analogRead(RL), 0, 4095, 0, 5000);
        data.susRR = map(analogRead(RR), 0, 4095, 0, 5000);

        xQueueOverwrite(sensorQueue, &data);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void LoggerTask(void *pvParameters)
{
    SensorData latest;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100ms 주기 (10Hz)

    for (;;)
    {
        if (xQueueReceive(sensorQueue, &latest, portMAX_DELAY) == pdTRUE)
        {
            // SD 카드 기록 로직
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
