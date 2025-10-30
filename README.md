# TF-25 Dashboard & Data Logger

ESP32-S3 기반 학생 포뮬러카 데이터 수집 및 대시보드 시스템

## 📋 프로젝트 개요

TACHYON 2025 Formula 차량용 실시간 데이터 로깅 및 운전자 대시보드 시스템입니다. ESP32-S3-DevKitC-1-N16R8V를 사용하여 다양한 센서 데이터를 수집하고, SD 카드에 기록하며, 운전자에게 실시간 정보를 표시합니다.

### 주요 기능

- **실시간 센서 데이터 수집 (50Hz)**
  - 6축 가속도/자이로 센서 (MPU6050)
  - RPM 센서 (홀 센서 기반)
  - 유온 센서 (NTC 서미스터)
  - GPS 위치 및 속도

- **데이터 로깅 (10Hz)**
  - SD 카드에 CSV 형식 저장
  - 날짜별 폴더 자동 생성
  - RTC 기반 타임스탬프

- **운전자 디스플레이**
  - 7-Segment 디스플레이: 속도, 유온
  - WS2812B LED 스트립: RPM 게이지 (19 LEDs)

## 🔧 하드웨어 구성

### ESP32-S3-DevKitC-1-N16R8V

#### 센서 및 주변장치 핀 맵

| 기능 | GPIO | 설명 |
|------|------|------|
| **SPI (공유 버스)** |
| SCK | GPIO14 (MTMS) | SPI Clock |
| MISO | GPIO12 (MTDI) | SPI MISO |
| MOSI | GPIO13 (MTCK) | SPI MOSI |
| SD_CS | GPIO37 | SD 카드 칩 셀렉트 |
| LORA_CS | GPIO5 | LoRa 모듈 칩 셀렉트 |
| MAXCS | GPIO38 | 7-Segment 디스플레이 CS |
| **I2C** |
| SCL | GPIO35 | I2C Clock (MPU6050, RTC) |
| SDA | GPIO36 | I2C Data (MPU6050, RTC) |
| **UART (GPS)** |
| GPS_RX | GPIO18 | GPS 모듈 RX |
| GPS_TX | GPIO17 | GPS 모듈 TX |
| **센서 입력** |
| RPM_PIN | GPIO9 | 홀 센서 펄스 입력 |
| EOT_PIN | GPIO19 | 유온 센서 (NTC, ADC) |
| **디스플레이** |
| WS2812_PIN | GPIO40 | NeoPixel LED 스트립 |
| **기타** |
| LED_PIN | GPIO10 | 상태 표시 LED |

### 센서 사양

#### MPU6050 (6축 IMU)
- I2C 주소: 0x69
- 가속도 범위: ±4G
- 자이로 범위: ±500°/s

#### 유온 센서 (NTC)
- 회로: 3.3V → 3.3kΩ 풀업 → ADC → 50kΩ NTC → GND
- B-계수: 3950
- 공칭 저항: 50kΩ @ 25°C
- 필터: 지수 이동 평균 (α=0.2)

#### RPM 센서
- 홀 센서 펄스 카운트
- 범위: 0-11000 RPM
- LED 게이지 활성화: 4500 RPM 이상

#### GPS
- 모듈: NEO-6M 호환
- 보레이트: 9600
- 기능: 위치(위도/경도), 속도, RTC 자동 동기화 (UTC+9)

## 📦 소프트웨어 아키텍처

### FreeRTOS 멀티태스킹

| Task | Core | Priority | 주기 | 설명 |
|------|------|----------|------|------|
| **SensorTask** | 0 | 2 | 20ms (50Hz) | 모든 센서 데이터 수집 및 큐 전송 |
| **LoggerTask** | 1 | 3 | 100ms (10Hz) | SD 카드에 CSV 기록 |
| **DisplayTask** | 1 | 1 | 대기 | 디스플레이 업데이트 (LED, 7-Seg) |

### 데이터 구조체

```cpp
struct SensorData {
    char timestamp[25];              // HH:MM:SS.mmm
    float accelX, accelY, accelZ;   // m/s²
    float gyroX, gyroY, gyroZ;      // rad/s
    uint16_t rpm;                    // RPM
    double oilTemp;                  // °C
    double latitude, longitude;      // GPS 좌표
    uint16_t speed;                  // km/h
};
```

### CSV 로그 포맷

```
Timestamp,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,RPM,OilTemp,Latitude,Longitude,Speed
12:34:56.789,0.12,0.34,-9.81,0.01,-0.02,0.00,5500,85.3,37.123456,127.123456,45.2
```

## 🚀 빌드 및 업로드

### 필요 환경
- [PlatformIO](https://platformio.org/)
- VS Code + PlatformIO Extension

### 라이브러리 의존성
```ini
lib_deps = 
    sandeepmistry/LoRa@^0.8.0
    adafruit/Adafruit MPU6050@^2.2.6
    adafruit/Adafruit NeoPixel@^1.12.5
    mikalhart/TinyGPSPlus@^1.1.0
    adafruit/RTClib@^2.1.4
    majicdesigns/MD_MAX72XX@^3.5.1
```

### 빌드 및 업로드

```bash
# 빌드
pio run

# 업로드
pio run --target upload

# 시리얼 모니터
pio device monitor
```

## 📊 운영 방법

### 시동 시퀀스
1. SD 카드 초기화
2. RTC 시간 확인 (컴파일 타임으로 초기 설정)
3. MPU6050 센서 초기화
4. GPS 모듈 시작
5. FreeRTOS 태스크 생성 및 시작

### GPS 기반 RTC 동기화
- GPS 신호 수신 시 자동으로 RTC를 UTC+9(KST) 시간으로 설정
- 한 번만 동기화되며, 이후 RTC가 독립적으로 시간 유지

### LED RPM 게이지
- 4500 RPM 이하: 전부 꺼짐
- 4500-11000 RPM: 점진적으로 점등
- 상위 4개 LED: 레드 존 (빨간색)
- 하위 LED: 오렌지색 그라데이션

### 7-Segment 디스플레이
- **위치 4-6**: 차량 속도 (km/h, 3자리)
- **위치 1-3**: 유온 (°C, 3자리, 앞자리 공백)

## 🛠️ 트러블슈팅

### SD 카드 초기화 실패
- SD 카드가 제대로 삽입되었는지 확인
- SD 카드 포맷: FAT32
- SPI 핀 연결 확인 (SCK, MISO, MOSI, CS)

### MPU6050 감지 안 됨
- I2C 주소 확인 (0x68 또는 0x69)
- SDA/SCL 풀업 저항 확인 (일반적으로 4.7kΩ)

### GPS 데이터 수신 안 됨
- 안테나 연결 확인
- 야외에서 테스트 (실내는 신호 약함)
- TX/RX 핀 연결 확인

### 유온 센서 이상값
- NTC 연결 확인
- 풀업 저항 값 확인 (3.3kΩ)
- ADC 핀 할당 확인

## 📁 디렉토리 구조

```
TF-25_Dash/
├── platformio.ini          # PlatformIO 설정
├── README.md               # 이 파일
├── src/
│   └── main.cpp           # 메인 소스 코드
├── include/               # 헤더 파일
├── lib/                   # 프로젝트 전용 라이브러리
└── test/                  # 테스트 코드
```

## 📝 로그 파일 구조

SD 카드에 다음과 같은 구조로 저장됩니다:

```
/20251030/              # YYYYMMDD 폴더
  ├── 143022.csv       # HHMMSS.csv (시작 시간)
  └── 150045.csv
/20251031/
  └── 091530.csv
```

## 🔐 안전 기능

- **SPI Mutex**: SD 카드 쓰기와 디스플레이 업데이트 간 충돌 방지
- **RPM 타임아웃**: 2초간 펄스 없으면 RPM=0
- **온도 범위 제한**: -20°C ~ 200°C
- **큐 오버라이트**: 최신 데이터 우선 (오래된 데이터 폐기)

## ⚠️ 면책 조항 (DISCLAIMER)

**이 시스템(하드웨어 및 소프트웨어)은 "있는 그대로" 제공되며, 명시적이거나 묵시적인 어떠한 보증도 하지 않습니다.**

본 시스템은 교육 및 연구 목적의 학생 프로젝트로 개발되었으며, 상용 제품 수준의 안정성이나 신뢰성을 보장하지 않습니다. 사용자는 다음 사항에 동의한 것으로 간주됩니다:

### 책임 제한
- 본 시스템(회로 설계, PCB, 배선, 소프트웨어 포함)의 사용으로 인해 발생하는 **직접적, 간접적, 우발적, 특수한, 징벌적 또는 결과적 손해**에 대해 개발팀은 어떠한 책임도 지지 않습니다.
- 여기에는 데이터 손실, 차량 손상, 화재, 전기적 고장, 인명 피해, 경기 실격, 장비 고장, 재산 피해 등이 포함되나 이에 국한되지 않습니다.
- **사용자의 책임 하에** 모든 기능을 사전 테스트하고 검증해야 합니다.

### 보증 부인
- **하드웨어**: 회로 설계 오류, 배선 실수, 부품 불량, 전원 공급 문제, 단락, 과전압, EMI/EMC 문제 등에 대한 책임을 지지 않습니다.
- **소프트웨어**: 시스템의 **오류 없음, 중단 없는 작동, 정확성, 적합성**을 보증하지 않습니다.
- 센서 오류, SD 카드 손상, 전원 문제, EMI 간섭, 납땜 불량, 커넥터 접촉 불량 등으로 인한 데이터 손실이나 오작동 가능성이 있습니다.
- 실시간 임베디드 시스템 및 자작 하드웨어의 특성상 예상치 못한 동작이 발생할 수 있습니다.
- PCB 설계, 회로도, 부품 선정에 오류가 있을 수 있으며, 이로 인한 손해에 대해 책임지지 않습니다.

### 사용자 의무
- 본 시스템을 차량에 설치하거나 사용하기 전 **충분한 테스트**를 수행해야 합니다.
- 경기 규정 및 안전 규정을 준수할 책임은 **전적으로 사용자**에게 있습니다.
- 중요한 의사결정(차량 세팅, 안전 판단 등)에 본 시스템의 데이터만 의존하지 마십시오.

**본 시스템을 사용함으로써 위 조항에 동의한 것으로 간주됩니다.**

---

## 📄 라이선스

이 프로젝트는 TACHYON Formula Racing Team의 소유입니다.

## 👥 작성자

TACHYON Formula Racing Team - 2025 Season

---

**참고**: 이 시스템은 학생 포뮬러카 경기용으로 개발되었습니다. 실제 차량 운행 중 데이터를 수집하므로 안전에 유의하여 사용하십시오.
