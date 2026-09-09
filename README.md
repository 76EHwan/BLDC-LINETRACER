# BLDC LINE TRACER

STM32H743VIT6 기반 **BLDC(FOC 제어) 라인트레이서** 펌웨어입니다.
좌/우 2개의 BLDC 모터를 센서리스가 아닌 **엔코더 기반 Field Oriented Control** 로 구동하고,
16채널 라인 센서 + 2채널 마커 센서로 라인을 추종하면서,
1회차 주행에서 기록한 **마커/거리/요각(yaw) 로그를 기반으로 2~4회차에서 구간별 가속·감속 및 인코스 주행**을 수행합니다.

---

## 1. 하드웨어 구성

| 구분 | 사양 |
|---|---|
| MCU | STM32H743VIT6 (Cortex-M7, SYSCLK 480 MHz, I/D-Cache + MPU 사용) |
| 모터 | maxon ECX SPEED 16 M (coreless, 극쌍수 1, 36 V 권선 파라미터 기준) × 2 |
| 게이트 드라이버 | TI **DRV8316C** × 2 (SPI2 제어, nFAULT / nSLEEP / DRVOFF GPIO) |
| 엔코더 | 쿼드러처 엔코더 2048 CPR — LPTIM2(좌) / LPTIM1(우) 엔코더 모드 |
| 보조 엔코더 | MT6701 자기 엔코더 (SPI2 SSI, 테스트 메뉴에서만 사용) |
| 라인 센서 | IR 18채널 (라인 16 + 좌/우 마커 2), 4bit MUX 스캔, ADC3 + BDMA |
| IMU | LSM6DS3TR-C (I2C1 DMA, Gyro Z 적분으로 yaw 산출) |
| 디스플레이 | ST7789 LCD (SPI, `st7735` 드라이버도 함께 포함) |
| 저장장치 | microSD (SDMMC1 + FatFs), W25Qxx QSPI 플래시 |
| 입력 | 5-way 버튼 (U / D / L / R / K), 싱글·더블·홀드 인식 |
| 부저 | DAC + DMA 파형 출력 (FUET-8540), LPTIM3로 발음 시간 관리 |
| 기타 | 흡입팬 PWM(TIM15), 배터리 전압 분압 측정, USB DFU 부트로더 진입 |

기구 상수 (`motor.h`, `drive.h`)

```
TIRE_DIAMETER      0.023 m      // 바퀴 지름 23 mm
GEAR_RATIO         39 / 11
THREAD             0.186 m      // 좌우 바퀴 중심 간격
SENSOR_HALF_WIDTH  0.08 m       // 센서바 중앙 ~ 끝단 15번 센서
```

---

## 2. 디렉터리 구조

```
Main/                     ← 애플리케이션 로직 (직접 작성한 코드)
 ├── Src/foc.c            FOC 전류/속도 제어, 캘리브레이션, 오도메트리
 ├── Src/motor.c          모터 기동/정지, 조향(Steer), 드라이버 설정, 튜닝 UI
 ├── Src/sensor.c         라인 센서 스캔·정규화·위치 추정, 마커 검출/로깅
 ├── Src/drive.c          1~4회차 주행 시퀀스, 가감속 램프, 구간 계획
 ├── Src/menu.c           계층형 LCD 메뉴, 파라미터 편집, Last-Used 기록
 ├── Src/sd_ui.c          SD 카드 설정 저장/로드, 주행 로그 저장, 슬롯 선택 UI
 ├── Src/bootloader.c     시스템 메모리(DFU) 부트로더 점프
 └── Src/user_init.c      전원 인가 후 초기화 시퀀스

Drivers/BSP/              보드 주변장치 드라이버 (DRV8316C, MCT8316Z, LSM6DS3TR-C,
                          MT6701, ST7735/ST7789, SDcard, w25qxx, Button, Buzzer)
Core/                     CubeMX 생성 코드 (main.c, 주변장치 init, IRQ 디스패처)
Drivers/CMSIS/DSP         CMSIS-DSP (Clarke/Park/PID/삼각함수)
FATFS/, Middlewares/      FatFs, USB Device
MDK-ARM/                  Keil 프로젝트 (보조)
BLDC_Template_v2.0.0.ioc  STM32CubeMX 프로젝트 파일
```

---

## 3. 제어 루프 타이밍

| 루프 | 트리거 | 주기 | 하는 일 |
|---|---|---|---|
| FOC 전류 루프 | ADC1/ADC2 **injected** 변환 완료 IRQ (TIM4/TIM3 TRGO) | **25 kHz** | 전기각 갱신 → Clarke/Park → Id/Iq PI → 역Park/Clarke → PWM duty |
| 속도 루프 | TIM13 | **2 kHz** | 엔코더 차분 속도 측정(이동평균 4), 속도 PID → Iq 지령, Vbus LPF |
| 램프 & 조향 | TIM14 | **2 kHz** | 오도메트리 적산, 목표속도 가감속 램프, `Steer_Motor()` |
| 센서 스캔 | TIM7 → TIM2 → ADC3 | 슬롯 25 µs / 전체 **2 kHz** | MUX 채널 전환 → 5 µs 후 ADC 트리거 → 정규화 + 임계 판정 |
| IMU 샘플링 | SysTick 10 tick | 100 Hz | Gyro Z DMA 읽기 → DWT 사이클 기반 dt로 yaw 적분 |

PWM은 TIM3(좌)/TIM4(우) **center-aligned, ARR 4800 @ 240 MHz → 25 kHz** 입니다.
`MX_DRV8316C_Init()` 에서 CubeMX가 Hall 모드로 만들어 둔 TIM3/TIM4를 PWM 모드로 재구성하고,
TIM3는 카운터 0, TIM4는 4800에서 출발시켜 **좌우 모터의 전류 샘플링 시점을 반주기 어긋나게** 합니다.
채널4(OC4REF)가 ADC injected 변환 트리거로 쓰이며, 샘플 시점은 `ADC_READ_TIMING`(170 tick)으로 조정합니다.

---

## 4. FOC 제어 (`Main/Src/foc.c`, `Main/Inc/foc.h`)

- **전류 센싱**: DRV8316C의 CSA 출력을 ADC injected 2채널(rank1 = A상, rank2 = C상)로 동시 취득,
  B상은 `-(Ia + Ic)`로 계산. 16bit + 4배 오버샘플링. `CURRENT_SCALE = 3.3 / 65536 / (300 mA/V)`.
- **오프셋 캘리브레이션**: 무부하 상태에서 400회 평균 (`FOC_Calibrate_Offset`).
- **엔코더 영점 정렬**: 정격의 10 % 전압을 D축에 인가해 회전자를 강제 정렬한 뒤
  LPTIM CNT를 읽어 `theta_offset` 산출 (`FOC_Calibrate_Encoder_Offset_Both` 는 좌우 동시 수행).
- **게인 자동 산출**: 모터 파라미터로부터 매크로에서 계산합니다.
  - 전류 루프: 대역폭 = 제어주파수 / 20 × 2π, `Kp = BW·L`, `Ki = BW·R·dt`
  - 속도 루프: 대역폭 = 2 kHz / 20 × 2π, `Kp = J·ω_s / Kt × 부하비율`, `Ki = Kp / τ_m`
- **피드포워드**: 역기전력 항 `we·λ` 를 Vq에 가산 (`ff_bemf_gain = 1.0`).
  상호간섭(cross-coupling) 디커플링 항은 게인 0으로 두어 기본 비활성.
- **보호**: Vd/Vq를 Vbus로 클램프, duty 상한 90 %, 속도 PID 적분 anti-windup(Iq limit 3 A),
  D항은 measurement 미분 + 전용 LPF.
- **오도메트리**: 좌우 측정 속도의 평균을 2 kHz로 적산하여 `g_odom_distance_m` 유지.
  마커를 기록할 때마다 리셋되므로 값은 **직전 마커로부터의 거리**를 의미합니다.

구동 모드 (`FOC_DriveMode_t`)

| 모드 | 설명 |
|---|---|
| `FOC_MODE_NO_SVPWM_SPIN` | PWM만 수동 인가 (6-step 테스트) |
| `FOC_MODE_SVPWM_NO_SPIN` | 연산만 수행, 출력 차단 (전류/각도 디버깅) |
| `FOC_MODE_SVPWM_SPIN` | 전류 루프까지 정상 FOC |
| `FOC_MODE_SPEED_LOOP` | 속도 폐루프까지 동작 (실제 주행) |

---

## 5. 라인 센싱 (`Main/Src/sensor.c`)

- 18채널을 **10슬롯 × 2그룹 = 20슬롯**으로 나눠 인접 채널이 연속으로 켜지지 않도록 순서를 섞어 스캔합니다
  (`scan_group1/2` — 크로스토크 저감). 마커 센서 16/17번은 두 그룹 모두에 포함되어 2배 빠르게 갱신됩니다.
- 캘리브레이션에서 채널별 `whitemax` / `blackmax` 를 수집하고, 정규화 계수를
  `(255 << 8) / (white - black)` 로 미리 계산해 런타임에는 정수 곱/시프트만 수행합니다.
- **위치 추정** `Sensor_Get_Position()`
  - 직전 피크 인덱스 주변 ±3 창에서만 피크를 탐색해 노이즈/분기에 강인하게 처리
  - 창 안에서 7개 센서의 정규화 값으로 **가중 평균(무게중심)** → −1.0 ~ +1.0 의 위치 반환
  - 창 밖에 켜진 센서가 있으면 좌/우 **마커 후보**(`mark_left` / `mark_right`)로 분리
  - 창 안이 과도하게 켜지면 교차선(cross) 상태로 판단해 직전 위치를 유지
  - 총 밝기 합이 `line_lost_sum_min` 이하인 상태가 50회 연속되면 **라인 로스트** 판정 → 주행 종료
- **마커 검출** `Cross_Detect_Update()` — 마커 진입 시 상태를 누적하고 이탈하는 순간 판정합니다.

  | 누적 결과 | 이벤트 |
  |---|---|
  | 좌 + 우 & 중앙 12개 이상 점등 | `CROSS_CROSS` (교차로) |
  | 좌 + 우 & 그 외 | `CROSS_STOP` (출발/정지선) |
  | 좌만 | `CROSS_LEFT` |
  | 우만 | `CROSS_RIGHT` |

  이벤트마다 부저를 울리고, 곡선 구간 상태(L/R 토글)에 따라 LCD를 반전 표시합니다.
  로그에는 **직전 마커로부터의 주행 거리**와 그 구간에서의 **최대 yaw 변화량**이 함께 저장됩니다
  (`CrossMarkerLog_t`, 최대 256개).

---

## 6. 조향 및 속도 배분 (`Steer_Motor`, `motor.c`)

```
error      = line_pos - target_pos
steer      = PD(error)                       // CMSIS arm_pid_f32, Ki = 0
atten      = clamp(1 - |error| * pos_atten_gain, 0.4, 1.0)   // 비대칭 필터 (감속 즉시 / 복귀 완만)
active_mps = g_current_base_mps * atten
mps_L      = active_mps * (1 + steer * THREAD/2)
mps_R      = active_mps * (1 - steer * THREAD/2)
target_omega = mps * (2/TIRE_DIA) * POLE_PAIRS * GEAR_RATIO
```

가감속은 TIM14 램프에서 `accel` / `decel` [m/s²] 로 선형 처리하며,
`Drive_Stop_At_Distance(d)` 는 `decel = v² / 2d` 를 역산해 **지정한 거리 안에서 정지**시킵니다(피트인).

---

## 7. 주행 모드 (`Main/Src/drive.c`)

| 메뉴 | 함수 | 동작 |
|---|---|---|
| **1st Drive** | `Drive_First` | 기본 속도로 완주하며 마커·거리·yaw 로그 수집. 두 번째 STOP 마커에서 종료 후 SD 슬롯(1~10)에 저장 |
| **2nd Drive** | `Drive_Second` | 1회차 로그로 구간 계획을 세워 **직선 구간에서만 `max_mps` 로 가속**, 제동거리 `(v1²−v2²)/2a + 마진` 을 남기고 감속 |
| **3rd Drive** | `Drive_Third` | 가속은 끄고, 다가올 코너 방향에 따라 **목표 위치(`target_pos`)를 좌우로 선형 보간 이동**시켜 인코스 주행 |
| **4th Drive** | `Drive_Fourth` | 2회차의 가속 + 3회차의 타겟 시프트를 결합. 기록된 yaw 크기로 **15°~65° 얕은 연속 곡선(슬라롬)** 을 판별해 `max_mps × 0.85` 로 가속 |
| **Vibe Test** | `Drive_Vibration_Test` | 기본 속도 주행 중 Gyro Z를 1 ms 주기로 최대 10000 샘플 기록 → `/Drive_Data/vib_log_*.csv` |

공통 사항

- 매 주행 시작 시 센서 캘리브레이션 로드, 팬 기동, 마커 로그·오도메트리 초기화, 조향 PID 재초기화,
  FOC 속도 루프 기동, yaw 리셋을 수행합니다 (`Drive_*_Init_Sequence`).
- 2~4회차는 실제 마커 순서를 기준 로그와 대조하다가 어긋나면 `mismatch` 로 전환해 가속을 중단하고,
  이후 `CROSS_CROSS` 를 만나면 인덱스를 재동기화합니다.
- **라인 로스트 또는 두 번째 STOP 마커**에서 루프를 빠져나와 `pit_in_distance_m` 만큼 이동 후 정지합니다.
- 종료 후 LCD에 좌/우/교차 마커 개수와 랩타임을 표시합니다.

---

## 8. 메뉴 구조 (`Main/Src/menu.c`)

버튼: `U`/`D` 커서 이동, `R` 진입/실행, `L` 상위 메뉴, `K` 홀드로 각 기능 종료.

```
Main Menu
├── Sensor Menu   Calibration / Raw / Normalized / State / Position / IMU Test
├── Motor Menu    Driver Setup(레지스터 뷰) / Update Setup / 6-Step PWM / Simple FOC /
│                 Tune Cur PI / Encoder Test / Tune Spd PID / Fan Test /
│                 Mag Enc Test / Battery Volt / Load From SD / Save To SD
├── Drive Menu    1st ~ 4th Drive / Vibe Test
│   └── Update Param  Threshold, Lost Pos Min, Base m/s, Base Accel, Base Decel,
│                     Max m/s, Steer KP, Steer KD, Pos Abs Gain, Pit In Dis M,
│                     Target Shift, Fan Enable
├── Boot Load     K 홀드 4초 → DFU 부트로더 진입
└── Last Used     마지막 실행 기능 바로 실행 (내부 플래시에 기록)
```

`Last Used` 는 내부 플래시 Bank2 마지막 섹터(`0x081E0000`)에 32바이트 레코드를 append 방식으로 저장하고,
섹터가 차면 지우고 처음부터 다시 기록합니다. 저장된 콜백 주소가 현재 펌웨어에 존재할 때만 실행됩니다.

---

## 9. SD 카드 파일

| 경로 | 내용 |
|---|---|
| `/Sensor_Data/calibration_result.txt` | 채널별 whitemax / blackmax / 정규화 계수 |
| `/Foc_Data/foc_param.txt` | 좌우 모터 전류 오프셋, theta 오프셋, Id/Iq·속도 PID 게인, Iq limit, 엔코더 방향 |
| `/Drive_Data/save_slot_1..10.txt` | 주행 파라미터 + 마커 로그 (`IDX / TYPE / DIST / YAW`) |
| `/Drive_Data/vib_log_*.csv` | 진동 테스트 Gyro Z 로그 |

`save_slot_*.txt` 는 부팅할 때 `Delete_All_Marker_Logs()` 가 전부 삭제하므로,
보존이 필요한 로그는 전원을 내리기 전에 PC로 옮겨야 합니다.

---

## 10. 주요 파라미터 기본값

`DriveParam_t driveData` (`drive.c`) — 메뉴 `Update Param` 에서 실시간 편집

| 항목 | 기본값 | 설명 |
|---|---|---|
| `base_mps` | 1.8 | 기본 주행 속도 [m/s] |
| `max_mps` | 8.0 | 가속 구간 최고 속도 [m/s] |
| `accel` / `decel` | 5.5 / 5.5 | 가감속도 [m/s²] |
| `steer_gain_p` | 22.4 | 조향 P 게인 |
| `steer_gain_d` | 600.0 | 조향 D 게인 |
| `pos_atten_gain` | 0.0 | 오차 비례 감속 게인 (0이면 비활성) |
| `pit_in_distance_m` | 0.15 | 정지선 통과 후 정지까지 거리 [m] |
| `target_shift_val` | 0.3 | 3·4회차 인코스 타겟 이동량 (센서 정규화 좌표) |
| `fan_en` | 0 | 흡입팬 사용 여부 |

센서 임계값 `threshold = 95`, 라인 로스트 판정 합 `line_lost_sum_min = 20` (`sensor.c`).

---

## 11. 빌드 및 플래시

**필요 환경**

- STM32CubeIDE (프로젝트 파일 `.project` / `.cproject` 포함, 툴체인 GCC ARM)
- STM32CubeMX — `BLDC_Template_v2.0.0.ioc` 로 주변장치 재생성 시 사용
- 링커 스크립트: `STM32H743VITX_FLASH.ld` (RAM 실행용 `..._RAM.ld` 도 포함)

**빌드**

1. STM32CubeIDE에서 `File > Open Projects from File System...` 으로 저장소 루트를 임포트
2. `Debug` 또는 `Release` 구성으로 빌드
3. ST-LINK로 다운로드

> CubeMX로 코드를 재생성한 뒤에는 `Core/Src/stm32h7xx_it.c` 의 USER CODE 블록(타이머/ADC 콜백 디스패처)과
> `main.c` 의 `Check_Bootloader_Request()` 호출이 유지되었는지 확인하세요.
> TIM3/TIM4는 CubeMX에서 Hall 모드로 생성되며 `MX_DRV8316C_Init()` 이 런타임에 PWM으로 바꿉니다 — 정상 동작입니다.

**USB DFU로 플래시하기**

`Main Menu > Boot Load` 진입 후 K 버튼을 4초간 홀드하면 RTC 백업 레지스터에 플래그를 쓰고 리셋하며,
재부팅 직후 시스템 메모리(`0x1FF09800`)로 점프해 USB DFU 모드로 진입합니다.
이후 STM32CubeProgrammer 등으로 펌웨어를 기록할 수 있습니다.

---

## 12. 사용 순서 (첫 세팅)

1. **센서 캘리브레이션** — `Sensor Menu > Calibration`
   흰 바닥 위에서 센서바를 흔들어 whitemax 수집 → K 홀드 → 검은 라인 위에서 blackmax 수집 → K 홀드.
   결과는 자동으로 SD에 저장됩니다.
2. **모터 방향/엔코더 확인** — `Motor Menu > 6-Step PWM` 에서 DirL / DirR 이 `OK` 인지 확인,
   `Encoder Test` 로 CNT 증가 방향 확인.
3. **전류 PI 튜닝** — `Motor Menu > Tune Cur PI` (Id 스텝 응답 관찰), 필요 시 `Tune Spd PID`.
   튜닝값은 `Save To SD` 로 저장, 다음 전원 인가 후 `Load From SD` 로 복원합니다.
4. **주행 파라미터 설정** — `Drive Menu > Update Param`.
5. **1st Drive** 로 코스를 한 바퀴 학습시킨 뒤, 같은 전원 세션에서 **2nd / 3rd / 4th Drive** 실행.
   (기준 로그는 RAM에 유지되므로 전원을 내리면 다시 1회차부터 수행해야 합니다.)

---

## 13. 안전 유의사항

- `MTR_Safe_Stop()` 은 FOC 상태 초기화 → 속도 루프 정지 → PWM 정지 → ADC 정지 → 엔코더 정지 순으로
  수행합니다. 주행 함수를 임의로 중단시킬 때는 반드시 이 경로를 타도록 하세요.
- 배터리 전압은 `Motor Menu > Battery Volt` 에서 확인합니다. 이 모드는 드라이버 출력을 차단한 채
  ADC/타이머만 돌리므로 모터가 떨리지 않습니다.
- `MOTOR_RATED_VOLTAGE`(16.8 V)는 사용하는 배터리에 맞춰 `foc.h` 에서 수정해야 합니다.
