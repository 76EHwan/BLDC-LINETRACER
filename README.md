# WeAct Studio STM32H743VIT6 BLDC Line Tracer

이 프로젝트는 WeAct Studio의 STM32H743VIT6 보드를 활용하여 제작된 BLDC 모터 기반 라인트레이서(Line Tracer)입니다. 고성능 MCU와 전용 BLDC 모터 드라이버를 결합하여 정밀하고 빠른 주행을 목표로 하며, 센서 어레이를 통한 라인 인식과 상태 머신(State Machine)을 이용한 교차로 및 마크 판별 알고리즘이 적용되어 있습니다.

## 🛒 하드웨어 참조 링크
* [WeAct Studio AliExpress Store](https://weactstudio.ko.aliexpress.com/store/910567080?spm=a2g0o.detail.0.0.55c9v8gBv8gB7I)

## 🛠 주요 하드웨어 구성
* **MCU**: STM32H743VIT6 (고성능 ARM Cortex-M7)
* **모터 드라이버**: MCF8316C (BLDC Sensorless FOC 전용 IC)
* **IMU 센서**: LSM6DS3TR-C (6축 가속도 및 자이로 센서)
* **디스플레이**: ST7735 TFT LCD (디버깅 및 상태 출력용)
* **라인 센서**: 적외선 센서 어레이

---

## 🚀 주행 알고리즘 및 제어 (Driving Algorithm)

주행의 핵심 로직은 `drive.c`에 구현되어 있으며, 하드웨어 타이머와 비례 제어를 통해 실시간으로 조향을 수행합니다.

### 1. 비례 제어 (Proportional Control) 기반 조향
* **LPTIM5 인터럽트 (`Drive_LPTIM5_IRQ`)**: 일정한 타이머 주기마다 센서 값을 읽어와 모터의 속도를 업데이트합니다.
* **조향 계산식**: 
  * 목표 속도(`target_mps`)와 곡선 감속 비율(`curve_decel`)을 바탕으로 현재 주행해야 할 기준 속도(`center_mps`)를 계산합니다.
  * 센서 오차 값(`sensor.position`)과 비례 상수(`kp = 0.6f`)를 곱하여 좌/우 모터(ML, MR)에 인가할 최종 속도(m/s)를 차등 분배합니다.
  * *좌측 모터*: `center_mps * (1 + kp * sensor.position)`
  * *우측 모터*: `center_mps * (1 - kp * sensor.position)`

### 2. 마크 인식 상태 머신 (Mark State Machine)
센서의 입력값을 누적(`sensor_accum`)하여 주행 트랙의 특징점들을 인식하고 오작동을 방지합니다.
* **4단계 상태 천이**: `MARK_STATE_IDLE` ➔ `MARK_STATE_READ` ➔ `MARK_STATE_CROSS` ➔ `MARK_STATE_DECISION`
* **판별 마크**:
  * `MARK_LEFT` / `MARK_RIGHT`: 좌우측 턴 마크
  * `MARK_CROSS`: 십자 교차로 (`WINDOW_ALL` 조건 충족 시)
  * `MARK_END`: 주행 종료 지점

### 3. 단계별 주행 시퀀스 (`Drive_First`)
1. **초기화**: `Sensor_Start()`, `Motor_Start()`, `Drive_Start()`를 순차적으로 호출하여 주변 장치를 활성화합니다.
2. **주행 루프**: 센서가 트랙을 인식(`sensor.state`)하는 동안 루프를 돌며 LCD에 좌우 모터 속도, 센서 위치, 마크 인식 상태를 실시간으로 출력합니다.
3. **정지 조건**: 턴 마크 인식 상태(`sensor.state >> 14 == 0x3`)에 도달하거나 트랙을 벗어나면 주행을 종료하고 모터를 안전하게 정지(`Drive_Stop()`, `Motor_Stop()`)시킵니다.

---

## ⚙️ 모터 드라이버 (MCF8316C) 설정

이 라인트레이서는 일반적인 DC 모터가 아닌 BLDC 모터를 채택하여 응답성과 효율을 극대화했습니다. FOC 제어의 초기 개발 난이도를 줄이기 위해 텍사스 인스트루먼트(TI)사의 **MCF8316C** Sensorless FOC 모터 드라이버를 사용하여 제어합니다.

* **초기화 및 구동 (`Motor_Init`, `MCF8316C_Init`, `MCF8316C_MPET`)**: 
  * MCU 초기화 단계(`main.c`)에서 `Motor_Init()`과 `MCF8316C_Init()`을 호출하여 I2C 통신을 통해 모터 드라이버의 파라미터(극수, 가감속 프로파일 등)를 설정합니다.
  * **MCF8316C**의 내장 기능인 Motor Parameter Extraction Tool (MPET)을 활용하여 정확한 모터의 파라미터 및 Sensorless FOC에 필요한 모터 파라미터 및 Speed KP/KI, Torque KP/KI를 자동으로 구합니다.
* **속도 제어 인터페이스**:
  * 단순한 PWM 제어가 아닌, 물리적 속도 단위인 **m/s (meters per second)** 를 직접 사용하여 제어합니다 (`motor[ML].mps`, `motor[MR].mps`).
  * 상위 주행 알고리즘(`Drive_LPTIM5_IRQ`)에서 목표 속도(m/s)를 업데이트하면, 드라이버 제어 로직이 이를 내부 FOC 제어 목표값으로 변환하여 모터에 인가합니다.

---

## 💡 기타 고성능 처리 최적화
### 1. DWT (Data Watchpoint and Trace) 기반 마이크로초 지연
* 고속 주행 시 센서 데이터 샘플링 및 통신 타이밍의 정밀도를 보장하기 위해 Cortex-M7 코어의 **DWT 사이클 카운터(`DWT->CYCCNT`)**를 활성화하였습니다 (`custom_delay.c`).
* 표준 내장 함수인 `HAL_Delay()`가 제공하는 밀리초($ms$) 단위를 넘어, 하드웨어 클럭 사이클을 직접 카운트함으로써 **마이크로초($\mu s$) 단위의 극정밀 블로킹 지연**을 구현해 제어 타이밍 오차를 최소화합니다.

### 2. 캐시(Cache) 및 MPU 설정
* 프로세서 성능을 극대화하기 위해 **I-Cache 및 D-Cache를 활성화**(`CPU_CACHE_Enable`) 하였습니다.
* **MPU(Memory Protection Unit)** 설정을 통해 QSPI Flash 영역의 메모리 접근 권한을 제한하고 캐시 정책(Cacheable WT)을 최적화하여 시스템 안정성을 높였습니다.

## 📂 핵심 코드 구조
* `Src/main.c`: 시스템 클럭 구성, MPU/캐시 활성화, 필수 주변장치 및 하드웨어 초기화 진입점.
* `Main/Src/drive.c`: 타이머 기반의 조향 제어 로직, 마크 인식 상태 머신, 단계별 주행 루틴(`Drive_First`) 포함.
* * `Main/Src/custom_delay.c`: DWT를 이용한 마이크로초($\mu s$) 단위 정밀 지연 함수 구현.
* `Main/Src/mcf8316c.c` & `motor.c`: MCF8316C BLDC 모터 드라이버 설정 및 m/s 단위의 모터 제어 로직.
* `Main/Src/lsm6ds3tr_c.c` & `sensor.c`: 라인 센서 데이터 취득 및 IMU 센서 인터페이스.
