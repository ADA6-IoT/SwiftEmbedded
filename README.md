# SwiftEmbedded

ESP32 기반 실내 측위 시스템 - Beacon & Gateway 프로젝트

## 📋 프로젝트 소개

SwiftEmbedded는 ESP32 디바이스를 활용한 실내 측위 시스템입니다. FTM (Fine Timing Measurement)과 ESP-NOW 프로토콜을 사용하여 비콘과 게이트웨이 간의 거리를 측정하고, 실시간으로 위치 데이터를 수집합니다.

### 주요 기능

- **Beacon Device**: FTM 기반 거리 측정 및 층 정보 수집
- **Gateway Device**: 비콘 데이터 수신 및 서버 전송
- **ESP-NOW 통신**: 저전력 P2P 통신으로 데이터 전송
- **칼만 필터**: 측정값 노이즈 감소 및 정확도 향상

## 🔗 실행 방법

### 1. 환경 설정

#### ESP-IDF 설치
```bash
# macOS
brew install cmake ninja dfu-util

# ESP-IDF 설치
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32

# 환경변수 설정
. $HOME/esp/esp-idf/export.sh
```

#### 프로젝트 클론
```bash
git clone https://github.com/ADA6-IoT/SwiftEmbedded.git
cd SwiftEmbedded
```

### 2. Beacon 디바이스 빌드 및 플래시

```bash
# beacon 디렉토리로 이동
cd beacon

# 설정 메뉴 열기 (선택사항)
idf.py menuconfig

# 빌드
idf.py build

# 플래시 (ESP32 연결 필요)
idf.py -p /dev/ttyUSB0 flash

# 모니터링
idf.py -p /dev/ttyUSB0 monitor

# 빌드 + 플래시 + 모니터링 (한 번에)
idf.py -p /dev/ttyUSB0 flash monitor
```

### 3. Gateway 디바이스 빌드 및 플래시

```bash
# gateway 디렉토리로 이동
cd gateway

# 빌드
idf.py build

# 플래시 (ESP32 연결 필요)
idf.py -p /dev/ttyUSB0 flash

# 모니터링
idf.py -p /dev/ttyUSB0 monitor

# 빌드 + 플래시 + 모니터링 (한 번에)
idf.py -p /dev/ttyUSB0 flash monitor
```

### 4. 포트 확인 방법

#### macOS
```bash
ls /dev/cu.*
# 결과 예: /dev/cu.usbserial-0001
```

#### Linux
```bash
ls /dev/ttyUSB*
# 결과 예: /dev/ttyUSB0
```

#### Windows
```bash
# 장치 관리자에서 COM 포트 확인
# 예: COM3, COM4
```

### 5. 설정 변경

각 프로젝트의 `sdkconfig` 파일을 수정하거나, `idf.py menuconfig` 명령으로 설정을 변경할 수 있습니다.

#### 주요 설정 항목
- Wi-Fi SSID/Password
- FTM 파라미터
- 채널 설정
- 전송 주기

## 📂 프로젝트 구조

```
SwiftEmbedded/
├── beacon/                 # Beacon 디바이스 프로젝트
│   ├── main/
│   │   ├── main.c         # Beacon 메인 코드
│   │   └── CMakeLists.txt
│   ├── CMakeLists.txt
│   └── sdkconfig          # Beacon 설정 파일
│
├── gateway/               # Gateway 디바이스 프로젝트
│   ├── main/
│   │   ├── main.c         # Gateway 메인 코드
│   │   └── CMakeLists.txt
│   ├── CMakeLists.txt
│   ├── sdkconfig          # Gateway 설정 파일
│   └── partitions.csv     # 파티션 테이블
│
├── .github/
│   └── pull_request_template.md  # PR 템플릿
│
├── .gitignore
├── README.md              # 프로젝트 소개 및 실행 방법
└── CONVENTION.md          # 개발 컨벤션
```

## 🛠️ 기술 스택

- **Hardware**: ESP32
- **Framework**: ESP-IDF
- **Language**: C
- **Protocol**: Wi-Fi, ESP-NOW, FTM
- **Filter**: Kalman Filter

## 🔧 트러블슈팅

### 플래시 실패 시
```bash
# 포트 권한 설정 (Linux)
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0

# ESP32 리셋 후 재시도
# BOOT 버튼을 누른 상태로 플래시 시도
```

### 빌드 오류 시
```bash
# 빌드 파일 정리
idf.py fullclean

# 재빌드
idf.py build
```

### ESP-IDF 환경변수 설정
```bash
# 매번 터미널을 열 때마다 실행
. $HOME/esp/esp-idf/export.sh

# 또는 ~/.bashrc 또는 ~/.zshrc에 추가
echo '. $HOME/esp/esp-idf/export.sh' >> ~/.bashrc
```

## 📖 참고 자료

- [ESP-IDF 프로그래밍 가이드](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [ESP-NOW 문서](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html)
- [FTM 문서](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-ftm)
- [개발 컨벤션](./CONVENTION.md)

## 👥 팀

ADA6-IoT Team

## 📄 라이선스

This project is licensed under the MIT License.
