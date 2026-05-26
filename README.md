# WeAct Studio STM32H743VIT6 BLDC Line Tracer

이 프로젝트는 WeAct Studio의 STM32H743VIT6 보드를 활용하여 제작된 BLDC 모터 기반 라인트레이서(Line Tracer)입니다. 센서 어레이를 통한 정밀한 라인 인식과 상태 머신(State Machine)을 이용한 교차로 및 마크 판별 알고리즘이 적용되어 있습니다.

## 🛒 하드웨어 참조 링크
* [WeAct Studio AliExpress Store](https://weactstudio.ko.aliexpress.com/store/910567080?spm=a2g0o.detail.0.0.55c9v8gBv8gB7I)

## 🛠 주요 하드웨어 구성
* **MCU**: STM32H743VIT6 (고성능 ARM Cortex-M7)
* **모터 드라이버**: MCF8316C (BLDC 모터 제어)
* **IMU 센서**: LSM6DS3TR-C (6축 가속도 및 자이로 센서)
* **디스플레이**: ST7735 TFT LCD (디버깅 및 상태 출력용)
* **저장 장치**: MicroSD 카드 (FatFS 적용)
* **라인 센서**: 적외선 센서 어레이

## 주요 기능 및 특징

### 1. 정밀한 주행 및 조향 제어
* **비례 제어 알고리즘**: 라인 센서에서 읽어들인 오차 값(`sensor.position`)과 `kp` 상수를 활용하여 좌우 모터(ML, MR)의 속도를 실시간으로 보정합니다.
* **곡선 구간 감속**: 직선 구간의 목표 속도(`target_mps`)를 바탕으로, 곡선의 곡률에 따라 자동으로 속도를 줄여 안정적인 코너링을 수행합니다.
* **하드웨어 타이머 기반 제어**: `LPTIM5` 인터럽트를 사용하여 일정한 주기마다 모터의 조향 속도를 정밀하게 업데이트합니다.

### 2. 마크 인식 상태 머신 (Mark State Machine)
센서의 입력값을 누적(`sensor_accum`)하여 주변 환경을 파악하고, 다음과 같은 주행 트랙의 특징점들을 인식합니다.
* **IDLE / READ / CROSS / DECISION** 4단계의 상태 천이를 통해 노이즈에 의한 오작동을 방지합니다.
* 판별 가능한 마크 종류:
  * `MARK_LEFT` (좌측 직각/예각 마크)
  * `MARK_RIGHT` (우측 직각/예각 마크)
  * `MARK_CROSS` (십자 교차로)
  * `MARK_END` (종료 지점)

### 3. 고성능 처리 최적화
* Cortex-M7 프로세서의 성능을 극대화하기 위해 **I-Cache 및 D-Cache를 활성화**하였습니다.
* **MPU(Memory Protection Unit)** 설정을 통해 QSPI Flash 메모리 영역의 접근 권한과 캐시 정책을 최적화하여 안정성을 높였습니다.

### 4. 실시간 디버깅 인터페이스
* 주행 중 LCD 화면을 통해 좌우 모터의 현재 속도, 센서 위치 값, 인식된 마크의 상태(`mark_state`) 등을 실시간으로 모니터링할 수 있습니다.

## 📂 핵심 코드 구조
* `Src/main.c`: 시스템 클럭 구성, MPU 및 캐시 초기화, 필수 주변장치(I2C, SPI, TIM, ADC 등) 및 각종 외부 하드웨어(디스플레이, 모터 드라이버, 센서) 초기화 진입점.
* `Main/Src/drive.c`: 라인트레이서의 핵심 자율 주행 로직. 타이머 인터럽트 기반의 조향 제어, 마크 인식 상태 머신, 그리고 단계별 주행 루틴(`Drive_First` 등)을 포함.
* `Main/Src/motor.c` & `mcf8316c.c`: BLDC 모터 구동 및 제어 로직.
* `Main/Src/sensor.c` & `lsm6ds3tr_c.c`: 라인 센서 어레이 데이터 취득 및 IMU 센서 인터페이스.
