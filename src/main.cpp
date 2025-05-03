#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <stdint.h>
#include <TM1637Display.h>
#include <Adafruit_NeoPixel.h>
#include <tinygps.h>
#include <FreeRTOS.h>
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

attachInterrupt(digitalPinToInterrupt(powerCheckPin), []() {
    if (analogRead(powerCheckPin) < 930) // 930 corresponds to ~2.8V on a 3.3V ADC scale
    {
        powerOffSequence();
    }
}, FALLING);

/// 센서 데이터 구조체
struct SensorData
{
    int16_t accelX, accelY, accelZ, yawRate, coolantTemp;
    uint16_t rpm, susFL, susFR, susRL, susRR, speed;
    int32_t latitude, longitude;
    uint8_t tps, gear;
    bool DTC;
};

// FreeRTOS 객체
QueueHandle_t sensorQueue;
SemaphoreHandle_t dataMutex;

// Adafruit MPU6050 객체
Adafruit_MPU6050 mpu;
TM1637Display display(CLK, DIO);

Adafruit_NeoPixel leds(NUM_LEDS, WS2812_PIN, NEO_GRB + NEO_KHZ800);

TinyGPSPlus gps; // GPS 객체 생성

HardwareSerial kwpSerial(1); // UART1 사용
KWP2000 kwp(&kwpSerial);

// Task 함수 프로토타입
void SensorTask(void *pvParameters);
void LoggerTask(void *pvParameters);
void DisplayTask(void *pvParameters);
void TelemetryTask(void *pvParameters);

//전원종료 시 종료시퀀스
void powerOffSequence() {
    lora.end(); // LoRa 종료
    kwpSerial.end(); // KWP 종료
    Serial1.end(); // GPS 종료
}



void setup()
{
    Serial.begin(115200);
    display.setBrightness(0x0f);
    leds.begin();
    leds.show();

    // WS2812 시동 세리모니 (1초 동안 LED 채우기 → 줄이기)
    for (int i = 0; i <= NUM_LEDS; i++)
    {
        leds.clear();
        for (int j = 0; j < i; j++)
        {
            leds.setPixelColor(j, leds.Color(0, 150, 255));
        }
        leds.show();
        delay(30);
    }
    for (int i = NUM_LEDS; i >= 0; i--)
    {
        leds.clear();
        for (int j = 0; j < i; j++)
        {
            leds.setPixelColor(j, leds.Color(0, 150, 255));
        }
        leds.show();
        delay(30);
    }

    kwpSerial.begin(10400, SERIAL_8N1, KWP_RX, KWP_TX);
    kwp.begin();

    // LoRa 설정
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, LORA_CS);
    LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
    if (!LoRa.begin(433E6))
    {
        Serial.println("LoRa 시작 실패!");
        while (true)
            ;
    }
    Serial.println("LoRa 시작 완료!");

    // I2C 및 MPU6050 초기화
    Wire.begin();
    if (!mpu.begin())
    {
        Serial.println("MPU6050 초기화 실패!");
        while (1)
            delay(10);
    }
    Serial.println("MPU6050 연결 성공!");

    // NEO-n8m 초기화
    Serial1.begin(9600, SERIAL_8N1, gpsRX, gpsTX);
    delay(1000);                     // GPS 모듈 초기화 대기
    Serial1.println("AT+CGNSPWR=1"); // GPS 모듈 전원 켜기

    // Queue 및 Mutex 생성
    sensorQueue = xQueueCreate(1, sizeof(SensorData));
    dataMutex = xSemaphoreCreateMutex();

    // FreeRTOS Task 생성
    xTaskCreatePinnedToCore(SensorTask, "SensorTask", 2048, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(LoggerTask, "LoggerTask", 2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(DisplayTask, "DisplayTask", 2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(TelemetryTask, "TelemetryTask", 2048, NULL, 2, NULL, 0);
}

void loop()
{
    // FreeRTOS 사용 시 loop()는 비워두면된다.
}

void SensorTask(void *pvParameters)
{
    SensorData data;
    sensors_event_t accelEvent, gyroEvent, tempEvent;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms 주기 (50Hz)
    for (;;)
    {
        while (Serial1.available())
        {
            gps.encode(Serial1.read());
        }
        // GPS 데이터 읽기
        if (gps.location.isUpdated())
        {
            data.latitude = gps.location.lat() * 1000000;
            data.longitude = gps.location.lng() * 1000000;
            data.speed = gps.speed.kmph() * 1000;
        }
        mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

        data.accelX = (int16_t)((accelEvent.acceleration.x / 9.80665f) * 1000);
        data.accelY = (int16_t)((accelEvent.acceleration.y / 9.80665f) * 1000);
        data.accelZ = (int16_t)((accelEvent.acceleration.z / 9.80665f) * 1000);

        float degPerSec = gyroEvent.gyro.z * 57.2958f; // 센서 방향에 따라 축 변경필요
        data.yawRate = (int16_t)(degPerSec * 10);

        data.susFL = map(analogRead(FL), 0, 4095, 0, 5000);
        data.susFR = map(analogRead(FR), 0, 4095, 0, 5000);
        data.susRL = map(analogRead(RL), 0, 4095, 0, 5000);
        data.susRR = map(analogRead(RR), 0, 4095, 0, 5000);

        // TODO: 나머지 센서(coolantTemp, rpm, gps, speed, tps, gear) 채워기

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
    bool blinkState = false;

    display.clear();
    leds.clear();
    leds.show();

    for (;;)
    {
        if (xQueueReceive(sensorQueue, &latest, portMAX_DELAY) == pdTRUE)
        {
            // TM1637 기어 + 냉각수 표시
            display.showNumberDec(latest.gear * 1000 + latest.coolantTemp, true, 4, 0);

            // WS2812 LED RPM 표시
            uint16_t rpm = latest.rpm;
            leds.clear();

            if (rpm >= 8500)
            {
                blinkState = !blinkState;
                for (int i = 0; i < NUM_LEDS; i++)
                {
                    leds.setPixelColor(i, blinkState ? leds.Color(255, 0, 0) : 0);
                }
            }
            else if (rpm >= 4000)
            {
                int active = map(rpm, 4000, 10000, 0, NUM_LEDS);
                for (int i = 0; i < active; i++)
                {
                    if (i < 5)
                        leds.setPixelColor(i, leds.Color(0, 255, 0));
                    else if (i < 10)
                        leds.setPixelColor(i, leds.Color(255, 200, 0));
                    else
                        leds.setPixelColor(i, leds.Color(255, 0, 0));
                }
            }
            leds.show();
            if (latest.DTC)
            {
                // DTC 발생 시 LED 점멸
            }
            if (latest.coolantTemp > 110)
            {
                // 냉각수 과열열 경고 LED 점멸
            }
            else if (latest.coolantTemp < 100 && latest.coolantTemp > 80)
            {
                // 냉각수 온도 정상일 때 LED 점멸 해제
            }
            else if (latest.coolantTemp < 70)
            {
                // 냉각수 온도 저온일 때 LED 점멸
            }
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
        if (xQueueReceive(sensorQueue, &latest, portMAX_DELAY) == pdTRUE)
        {
            // LoRa 전송 로직
            LoRa.beginPacket();
            LoRa.print(latest.accelX);      LoRa.print(",");
            LoRa.print(latest.accelY);      LoRa.print(",");
            LoRa.print(latest.accelZ);      LoRa.print(",");
            LoRa.print(latest.speed);       LoRa.print(",");
            LoRa.print(latest.latitude);    LoRa.print(",");
            LoRa.print(latest.longitude);   LoRa.print(",");
            LoRa.print(latest.rpm);         LoRa.print(",");
            LoRa.print(latest.susFL);       LoRa.print(",");
            LoRa.print(latest.susFR);       LoRa.print(",");
            LoRa.print(latest.susRL);       LoRa.print(",");
            LoRa.print(latest.susRR);       LoRa.print(",");
            LoRa.print(latest.tps);         LoRa.print(",");
            LoRa.print(latest.gear);        LoRa.print(",");
            LoRa.print(latest.coolantTemp); LoRa.endPacket();
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
