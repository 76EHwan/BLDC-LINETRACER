#ifndef ST7789_H
#define ST7789_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
#include <stddef.h>

typedef struct {
  void *handle;
  int32_t (*ReadReg)(void *, uint8_t, uint8_t *);
  int32_t (*WriteReg)(void *, uint8_t, uint8_t *, uint32_t);
  int32_t (*SendData)(void *, uint8_t *, uint32_t);
  int32_t (*RecvData)(void *, uint8_t *, uint32_t);
} st7789_ctx_t;

typedef int32_t (*ST7789_Init_Func)     (void);
typedef int32_t (*ST7789_DeInit_Func)   (void);
typedef int32_t (*ST7789_GetTick_Func)  (void);
typedef int32_t (*ST7789_Delay_Func)    (uint32_t);
typedef int32_t (*ST7789_WriteReg_Func) (uint8_t, uint8_t*, uint32_t);
typedef int32_t (*ST7789_ReadReg_Func)  (uint8_t, uint8_t*);
typedef int32_t (*ST7789_SendData_Func) (uint8_t*, uint32_t);
typedef int32_t (*ST7789_RecvData_Func) (uint8_t*, uint32_t);

typedef struct {
  ST7789_Init_Func          Init;
  ST7789_DeInit_Func        DeInit;
  uint16_t                  Address;
  ST7789_WriteReg_Func      WriteReg;
  ST7789_ReadReg_Func       ReadReg;
  ST7789_SendData_Func      SendData;
  ST7789_RecvData_Func      RecvData;
  ST7789_GetTick_Func       GetTick;
} ST7789_IO_t;

typedef struct {
  ST7789_IO_t         IO;
  st7789_ctx_t        Ctx;
  uint8_t             IsInitialized;
} ST7789_Object_t;

typedef struct {
  uint32_t        Width;
  uint32_t        Height;
  uint32_t        Orientation;
  uint8_t         Panel;
  uint8_t         Type;
} ST7789_Ctx_t;

extern ST7789_Ctx_t ST7789Ctx;

typedef struct {
  int32_t (*Init             )(ST7789_Object_t*, uint32_t, ST7789_Ctx_t*);
  int32_t (*DeInit           )(ST7789_Object_t*);
  int32_t (*ReadID           )(ST7789_Object_t*, uint32_t*);
  int32_t (*DisplayOn        )(ST7789_Object_t*);
  int32_t (*DisplayOff       )(ST7789_Object_t*);
  int32_t (*SetBrightness    )(ST7789_Object_t*, uint32_t);
  int32_t (*GetBrightness    )(ST7789_Object_t*, uint32_t*);
  int32_t (*SetOrientation   )(ST7789_Object_t*, ST7789_Ctx_t*);
  int32_t (*GetOrientation   )(ST7789_Object_t*, uint32_t*);

  int32_t ( *SetCursor       ) (ST7789_Object_t*, uint32_t, uint32_t);
  int32_t ( *DrawBitmap      ) (ST7789_Object_t*, uint32_t, uint32_t, uint8_t *);
  int32_t ( *FillRGBRect     ) (ST7789_Object_t*, uint32_t, uint32_t, uint8_t*, uint32_t, uint32_t);
  int32_t ( *DrawHLine       ) (ST7789_Object_t*, uint32_t, uint32_t, uint32_t, uint32_t);
  int32_t ( *DrawVLine       ) (ST7789_Object_t*, uint32_t, uint32_t, uint32_t, uint32_t);
  int32_t ( *FillRect        ) (ST7789_Object_t*, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);
  int32_t ( *GetPixel        ) (ST7789_Object_t*, uint32_t, uint32_t, uint32_t*);
  int32_t ( *SetPixel        ) (ST7789_Object_t*, uint32_t, uint32_t, uint32_t);
  int32_t ( *GetXSize        ) (ST7789_Object_t*, uint32_t *);
  int32_t ( *GetYSize        ) (ST7789_Object_t*, uint32_t *);
} ST7789_LCD_Drv_t;

#define ST7789_OK                (0)
#define ST7789_ERROR             (-1)

#define ST7789_135x240_screen    0x00U
#define ST7789_240x240_screen    0x01U

#define ST7789_ORIENTATION_PORTRAIT         0x00U
#define ST7789_ORIENTATION_PORTRAIT_ROT180  0x01U
#define ST7789_ORIENTATION_LANDSCAPE        0x02U
#define ST7789_ORIENTATION_LANDSCAPE_ROT180 0x03U

#define ST7789_FORMAT_RBG565                0x05U
#define ST7789_FORMAT_DEFAULT               ST7789_FORMAT_RBG565

extern ST7789_LCD_Drv_t ST7789_LCD_Driver;

/* 외부 호출 가능하도록 전역 선언 */
int32_t ST7789_RegisterBusIO (ST7789_Object_t *pObj, ST7789_IO_t *pIO);
int32_t ST7789_Init(ST7789_Object_t *pObj, uint32_t ColorCoding, ST7789_Ctx_t *pDriver);
int32_t ST7789_DeInit(ST7789_Object_t *pObj);
int32_t ST7789_ReadID(ST7789_Object_t *pObj, uint32_t *Id);
int32_t ST7789_DisplayOn(ST7789_Object_t *pObj);
int32_t ST7789_DisplayOff(ST7789_Object_t *pObj);
int32_t ST7789_SetBrightness(ST7789_Object_t *pObj, uint32_t Brightness);
int32_t ST7789_GetBrightness(ST7789_Object_t *pObj, uint32_t *Brightness);
int32_t ST7789_SetOrientation(ST7789_Object_t *pObj, ST7789_Ctx_t *pDriver);
int32_t ST7789_GetOrientation(ST7789_Object_t *pObj, uint32_t *Orientation);
int32_t ST7789_SetCursor(ST7789_Object_t *pObj, uint32_t Xpos, uint32_t Ypos);
int32_t ST7789_DrawBitmap(ST7789_Object_t *pObj, uint32_t Xpos, uint32_t Ypos, uint8_t *pBmp);
int32_t ST7789_FillRGBRect(ST7789_Object_t *pObj, uint32_t Xpos, uint32_t Ypos, uint8_t *pData, uint32_t Width, uint32_t Height);
int32_t ST7789_DrawHLine(ST7789_Object_t *pObj, uint32_t Xpos, uint32_t Ypos, uint32_t Length, uint32_t Color);
int32_t ST7789_DrawVLine(ST7789_Object_t *pObj, uint32_t Xpos, uint32_t Ypos, uint32_t Length, uint32_t Color);
int32_t ST7789_FillRect(ST7789_Object_t *pObj, uint32_t Xpos, uint32_t Ypos, uint32_t Width, uint32_t Height, uint32_t Color);
int32_t ST7789_GetPixel(ST7789_Object_t *pObj, uint32_t Xpos, uint32_t Ypos, uint32_t *Color);
int32_t ST7789_SetPixel(ST7789_Object_t *pObj, uint32_t Xpos, uint32_t Ypos, uint32_t Color);
int32_t ST7789_GetXSize(ST7789_Object_t *pObj, uint32_t *XSize);
int32_t ST7789_GetYSize(ST7789_Object_t *pObj, uint32_t *YSize);

#ifdef __cplusplus
}
#endif

#endif /* ST7789_H */
