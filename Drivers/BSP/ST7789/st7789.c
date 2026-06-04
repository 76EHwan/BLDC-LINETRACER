#include "st7789.h"

#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_MADCTL  0x36

ST7789_Ctx_t ST7789Ctx;

// Helper Macros
#define st7789_write_reg(ctx, reg, pdata, len) (ctx)->WriteReg((ctx)->handle, (reg), (pdata), (len))
#define st7789_read_reg(ctx, reg, pdata) (ctx)->ReadReg((ctx)->handle, (reg), (pdata))
#define st7789_send_data(ctx, pdata, len) (ctx)->SendData((ctx)->handle, (pdata), (len))
#define st7789_recv_data(ctx, pdata, len) (ctx)->RecvData((ctx)->handle, (pdata), (len))

static int32_t ST7789_SetDisplayWindow(ST7789_Object_t *pObj, uint32_t Xpos, uint32_t Ypos, uint32_t Width, uint32_t Height);
static int32_t ST7789_ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData);
static int32_t ST7789_WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint32_t Length);
static int32_t ST7789_SendDataWrap(void *Handle, uint8_t *pData, uint32_t Length);
static int32_t ST7789_RecvDataWrap(void *Handle, uint8_t *pData, uint32_t Length);
static int32_t ST7789_IO_Delay(ST7789_Object_t *pObj, uint32_t Delay);

ST7789_LCD_Drv_t ST7789_LCD_Driver = {
    ST7789_Init, ST7789_DeInit, ST7789_ReadID, ST7789_DisplayOn, ST7789_DisplayOff,
    ST7789_SetBrightness, ST7789_GetBrightness, ST7789_SetOrientation,
    ST7789_GetOrientation, ST7789_SetCursor, ST7789_DrawBitmap,
    ST7789_FillRGBRect, ST7789_DrawHLine, ST7789_DrawVLine, ST7789_FillRect,
    ST7789_GetPixel, ST7789_SetPixel, ST7789_GetXSize, ST7789_GetYSize
};

int32_t ST7789_RegisterBusIO(ST7789_Object_t *pObj, ST7789_IO_t *pIO) {
    if (pObj == NULL) return ST7789_ERROR;

    pObj->IO.Init = pIO->Init;
    pObj->IO.DeInit = pIO->DeInit;
    pObj->IO.Address = pIO->Address;
    pObj->IO.WriteReg = pIO->WriteReg;
    pObj->IO.ReadReg = pIO->ReadReg;
    pObj->IO.SendData = pIO->SendData;
    pObj->IO.RecvData = pIO->RecvData;
    pObj->IO.GetTick = pIO->GetTick;

    pObj->Ctx.ReadReg = ST7789_ReadRegWrap;
    pObj->Ctx.WriteReg = ST7789_WriteRegWrap;
    pObj->Ctx.SendData = ST7789_SendDataWrap;
    pObj->Ctx.RecvData = ST7789_RecvDataWrap;
    pObj->Ctx.handle = pObj;

    return pObj->IO.Init != NULL ? pObj->IO.Init() : ST7789_ERROR;
}

int32_t ST7789_Init(ST7789_Object_t *pObj, uint32_t ColorCoding, ST7789_Ctx_t *pDriver) {
    uint8_t tmp;
    st7789_write_reg(&pObj->Ctx, 0x01, NULL, 0);
    ST7789_IO_Delay(pObj, 150);

    st7789_write_reg(&pObj->Ctx, 0x11, NULL, 0);
    ST7789_IO_Delay(pObj, 120);

    tmp = ColorCoding;
    st7789_write_reg(&pObj->Ctx, 0x3A, &tmp, 1);

    ST7789_SetOrientation(pObj, pDriver);

    st7789_write_reg(&pObj->Ctx, 0x21, NULL, 0); // Inversion ON
    ST7789_IO_Delay(pObj, 10);

    st7789_write_reg(&pObj->Ctx, 0x13, NULL, 0); // Normal Display ON
    ST7789_IO_Delay(pObj, 10);

    st7789_write_reg(&pObj->Ctx, 0x29, NULL, 0); // Display ON
    ST7789_IO_Delay(pObj, 120);

    return ST7789_OK;
}

int32_t ST7789_SetOrientation(ST7789_Object_t *pObj, ST7789_Ctx_t *pDriver) {
    uint8_t madctl = 0x00;
    ST7789Ctx.Orientation = pDriver->Orientation;
    ST7789Ctx.Type = pDriver->Type;

    if (ST7789Ctx.Type == ST7789_135x240_screen) {
        if (pDriver->Orientation == ST7789_ORIENTATION_PORTRAIT || pDriver->Orientation == ST7789_ORIENTATION_PORTRAIT_ROT180) {
            ST7789Ctx.Width = 135;
            ST7789Ctx.Height = 240;
        } else {
            ST7789Ctx.Width = 240;
            ST7789Ctx.Height = 135;
        }
    } else {
        ST7789Ctx.Width = 240;
        ST7789Ctx.Height = 240;
    }

    switch(pDriver->Orientation) {
        case ST7789_ORIENTATION_PORTRAIT: madctl = 0x00; break;
        case ST7789_ORIENTATION_PORTRAIT_ROT180: madctl = 0xC0; break;
        case ST7789_ORIENTATION_LANDSCAPE: madctl = 0x70; break;
        case ST7789_ORIENTATION_LANDSCAPE_ROT180: madctl = 0xA0; break;
    }

    st7789_write_reg(&pObj->Ctx, ST7789_MADCTL, &madctl, 1);
    return ST7789_SetDisplayWindow(pObj, 0, 0, ST7789Ctx.Width, ST7789Ctx.Height);
}

static int32_t ST7789_SetDisplayWindow(ST7789_Object_t *pObj, uint32_t Xpos, uint32_t Ypos, uint32_t Width, uint32_t Height) {
    uint8_t tmp[4];
    uint32_t x_offset = 0, y_offset = 0;

    if (ST7789Ctx.Type == ST7789_135x240_screen) {
        if (ST7789Ctx.Orientation == ST7789_ORIENTATION_PORTRAIT) { x_offset = 52; y_offset = 40; }
        else if (ST7789Ctx.Orientation == ST7789_ORIENTATION_PORTRAIT_ROT180) { x_offset = 53; y_offset = 40; }
        else if (ST7789Ctx.Orientation == ST7789_ORIENTATION_LANDSCAPE) { x_offset = 40; y_offset = 53; }
        else if (ST7789Ctx.Orientation == ST7789_ORIENTATION_LANDSCAPE_ROT180) { x_offset = 40; y_offset = 52; }
    }

    Xpos += x_offset;
    Ypos += y_offset;

    st7789_write_reg(&pObj->Ctx, ST7789_CASET, NULL, 0);
    tmp[0] = (Xpos >> 8) & 0xFF; tmp[1] = Xpos & 0xFF;
    tmp[2] = ((Xpos + Width - 1) >> 8) & 0xFF; tmp[3] = (Xpos + Width - 1) & 0xFF;
    st7789_send_data(&pObj->Ctx, tmp, 4);

    st7789_write_reg(&pObj->Ctx, ST7789_RASET, NULL, 0);
    tmp[0] = (Ypos >> 8) & 0xFF; tmp[1] = Ypos & 0xFF;
    tmp[2] = ((Ypos + Height - 1) >> 8) & 0xFF; tmp[3] = (Ypos + Height - 1) & 0xFF;
    st7789_send_data(&pObj->Ctx, tmp, 4);

    st7789_write_reg(&pObj->Ctx, ST7789_RAMWR, NULL, 0);
    return ST7789_OK;
}

int32_t ST7789_SetCursor(ST7789_Object_t *pObj, uint32_t Xpos, uint32_t Ypos) {
    return ST7789_SetDisplayWindow(pObj, Xpos, Ypos, 1, 1);
}

int32_t ST7789_FillRGBRect(ST7789_Object_t *pObj, uint32_t Xpos, uint32_t Ypos, uint8_t *pData, uint32_t Width, uint32_t Height) {
    if (((Xpos + Width) > ST7789Ctx.Width) || ((Ypos + Height) > ST7789Ctx.Height)) return ST7789_ERROR;

    ST7789_SetDisplayWindow(pObj, Xpos, Ypos, Width, Height);
    st7789_send_data(&pObj->Ctx, pData, Width * Height * 2);
    return ST7789_OK;
}

int32_t ST7789_FillRect(ST7789_Object_t *pObj, uint32_t Xpos, uint32_t Ypos, uint32_t Width, uint32_t Height, uint32_t Color) {
    uint16_t swapped_color = (Color >> 8) | (Color << 8);
        uint32_t color_32 = (swapped_color << 16) | swapped_color;
        uint16_t line_buffer[Width];
    uint32_t *p_buffer = (uint32_t *)line_buffer;

    for (uint32_t i = 0; i < Width / 2; i++) {
        p_buffer[i] = color_32;
    }

    if (Width % 2 != 0) {
        line_buffer[Width - 1] = swapped_color;
    }

    ST7789_SetDisplayWindow(pObj, Xpos, Ypos, Width, Height);

    for (uint32_t i = 0; i < Height; i++) {
        st7789_send_data(&pObj->Ctx, (uint8_t *)line_buffer, Width * 2);
    }

    return ST7789_OK;
}

int32_t ST7789_DrawHLine(ST7789_Object_t *pObj, uint32_t Xpos, uint32_t Ypos, uint32_t Length, uint32_t Color) {
    return ST7789_FillRect(pObj, Xpos, Ypos, Length, 1, Color);
}

int32_t ST7789_DrawVLine(ST7789_Object_t *pObj, uint32_t Xpos, uint32_t Ypos, uint32_t Length, uint32_t Color) {
    return ST7789_FillRect(pObj, Xpos, Ypos, 1, Length, Color);
}

int32_t ST7789_SetPixel(ST7789_Object_t *pObj, uint32_t Xpos, uint32_t Ypos, uint32_t Color) {
    return ST7789_FillRect(pObj, Xpos, Ypos, 1, 1, Color);
}

int32_t ST7789_DeInit(ST7789_Object_t *pObj) { return ST7789_OK; }
int32_t ST7789_ReadID(ST7789_Object_t *pObj, uint32_t *Id) { *Id = 0; return ST7789_OK; }
int32_t ST7789_DisplayOn(ST7789_Object_t *pObj) { return ST7789_OK; }
int32_t ST7789_DisplayOff(ST7789_Object_t *pObj) { return ST7789_OK; }
int32_t ST7789_SetBrightness(ST7789_Object_t *pObj, uint32_t Brightness) { return ST7789_ERROR; }
int32_t ST7789_GetBrightness(ST7789_Object_t *pObj, uint32_t *Brightness) { return ST7789_ERROR; }
int32_t ST7789_GetOrientation(ST7789_Object_t *pObj, uint32_t *Orientation) { *Orientation = ST7789Ctx.Orientation; return ST7789_OK; }
int32_t ST7789_DrawBitmap(ST7789_Object_t *pObj, uint32_t Xpos, uint32_t Ypos, uint8_t *pBmp) { return ST7789_OK; }
int32_t ST7789_GetPixel(ST7789_Object_t *pObj, uint32_t Xpos, uint32_t Ypos, uint32_t *Color) { return ST7789_OK; }
int32_t ST7789_GetXSize(ST7789_Object_t *pObj, uint32_t *XSize) { *XSize = ST7789Ctx.Width; return ST7789_OK; }
int32_t ST7789_GetYSize(ST7789_Object_t *pObj, uint32_t *YSize) { *YSize = ST7789Ctx.Height; return ST7789_OK; }

static int32_t ST7789_ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData) { return ((ST7789_Object_t*) Handle)->IO.ReadReg(Reg, pData); }
static int32_t ST7789_WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint32_t Length) { return ((ST7789_Object_t*) Handle)->IO.WriteReg(Reg, pData, Length); }
static int32_t ST7789_SendDataWrap(void *Handle, uint8_t *pData, uint32_t Length) { return ((ST7789_Object_t*) Handle)->IO.SendData(pData, Length); }
static int32_t ST7789_RecvDataWrap(void *Handle, uint8_t *pData, uint32_t Length) { return ((ST7789_Object_t*) Handle)->IO.RecvData(pData, Length); }
static int32_t ST7789_IO_Delay(ST7789_Object_t *pObj, uint32_t Delay) {
    uint32_t tickstart = pObj->IO.GetTick();
    while ((pObj->IO.GetTick() - tickstart) < Delay) {}
    return ST7789_OK;
}
