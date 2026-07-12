/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "sd_diskio_patch.h"
/* USER CODE END Header */
#include "fatfs.h"

uint8_t retSD;    /* Return value for SD */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS;    /* File system object for SD logical drive */
FIL SDFile;       /* File object for SD */

/* USER CODE BEGIN Variables */
__attribute__((section(".ram_d2_nocache"), aligned(32))) FILINFO fno;
__attribute__((section(".ram_d2_nocache"), aligned(32))) DIR dir;
__attribute__((section(".ram_d2_nocache"), aligned(32))) FATFS SDFatFS_NC;
__attribute__((section(".ram_d2_nocache"), aligned(32))) FIL SDFile_NC;
<<<<<<< HEAD

static long long days_from_civil(int y, int m, int d);
static void civil_from_days(long long z, int *y, int *m, int *d);

=======
>>>>>>> refs/heads/Bug/SDcard
/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */
  FATFS_UnLinkDriver(SDPath);
  retSD = FATFS_LinkDriver(&SD_Driver_Fixed, SDPath);
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  /* VBAT이 없어 RTC가 전원 차단마다 리셋되므로, 부팅 시 "빌드(코드 업로드)
   * 시각"을 기준점으로 잡아두고, 이후에는 HAL_GetTick()으로 부팅 후 경과한
   * 시간을 더해서 시간이 실제로 흘러가는 것처럼 계산한다.
   * (단, 전원을 껐다 켜면 다시 빌드 시각부터 시작함 - 절대시간은 아님) */
  static long long base_seconds = 0;

  if (base_seconds == 0) {
    /* __DATE__ 형식: "Mmm dd yyyy" (예: "Jul  9 2026", 1~9일은 공백 패딩) */
    static const char build_date[] = __DATE__;
    static const char build_time[] = __TIME__; /* "hh:mm:ss" */
    static const char *months = "JanFebMarAprMayJunJulAugSepOctNovDec";

    int month = 0;
    for (int i = 0; i < 12; i++) {
      if (build_date[0] == months[i * 3]
       && build_date[1] == months[i * 3 + 1]
       && build_date[2] == months[i * 3 + 2]) {
        month = i + 1;
        break;
      }
    }

    int day  = (build_date[4] == ' ') ? (build_date[5] - '0')
                                       : ((build_date[4] - '0') * 10 + (build_date[5] - '0'));
    int year = (build_date[7] - '0') * 1000 + (build_date[8] - '0') * 100
             + (build_date[9] - '0') * 10   + (build_date[10] - '0');

    int hour = (build_time[0] - '0') * 10 + (build_time[1] - '0');
    int min  = (build_time[3] - '0') * 10 + (build_time[4] - '0');
    int sec  = (build_time[6] - '0') * 10 + (build_time[7] - '0');

    base_seconds = days_from_civil(year, month, day) * 86400LL
                 + hour * 3600 + min * 60 + sec;
  }

  long long now_seconds = base_seconds + (long long)(HAL_GetTick() / 1000);

  int y, mo, d;
  civil_from_days(now_seconds / 86400, &y, &mo, &d);
  int rem = (int)(now_seconds % 86400);
  int hh = rem / 3600;
  int mi = (rem % 3600) / 60;
  int ss = rem % 60;

  return ((DWORD)(y - 1980) << 25)
       | ((DWORD)mo << 21)
       | ((DWORD)d << 16)
       | ((DWORD)hh << 11)
       | ((DWORD)mi << 5)
       | ((DWORD)ss >> 1);
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */

/* 날짜(년,월,일) <-> 1970-01-01 기준 경과일수 변환 (윤년 포함 정확한 계산,
 * Howard Hinnant의 공개 알고리즘). get_fattime()에서 "빌드시각 + 경과시간"을
 * 계산할 때 월/년 넘어가는 걸 정확히 처리하기 위해 사용. */
static long long days_from_civil(int y, int m, int d)
{
  y -= (m <= 2);
  long long era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (long long)doe - 719468;
}

static void civil_from_days(long long z, int *y, int *m, int *d)
{
  z += 719468;
  long long era = (z >= 0 ? z : z - 146096) / 146097;
  unsigned doe = (unsigned)(z - era * 146097);
  unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  long long yy = (long long)yoe + era * 400;
  unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  unsigned mp = (5 * doy + 2) / 153;
  unsigned dd = doy - (153 * mp + 2) / 5 + 1;
  unsigned mm = mp + (mp < 10 ? 3 : -9);
  *y = (int)(yy + (mm <= 2));
  *m = (int)mm;
  *d = (int)dd;
}

/* USER CODE END Application */
