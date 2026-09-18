#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>
#include "oled_data.h"
#include "main.h"

/*
 * OLED 驱动方式选择（二选一）：
 *   OLED_USE_HW_I2C — 硬件 I2C3
 *   OLED_USE_SW_I2C — 软件模拟 I2C（PB6=SCL, PB7=SDA）
 *
 * 注意：软件 I2C 与软件串口 (sw_uart) 共用 PB6/PB7，
 *       两者互斥，只能启用其一。
 */
//#define OLED_USE_HW_I2C
//#define OLED_USE_SW_I2C  /* 与 SW_UART 互斥，串口模式下禁用 */

/*参数宏定义*********************/

/*FontSize参数取值*/
/*此参数值不仅用于判断，而且用于计算横向字符偏移，默认值为字体像素宽度*/
#define OLED_8X16				8
#define OLED_6X8				6

/*IsFilled参数数值*/
#define OLED_UNFILLED			0
#define OLED_FILLED				1

/*********************参数宏定义*/


/*函数声明*********************/

/*初始化函数*/
void OLED_Init(void);

/*更新函数*/
void OLED_Update(void);
void OLED_UpdateArea(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height);

/*显存控制函数*/
void OLED_Clear(void);
void OLED_ClearArea(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height);
void OLED_Reverse(void);
void OLED_ReverseArea(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height);

/*显示函数*/
void OLED_ShowChar(uint8_t X, uint8_t Y, char Char, uint8_t FontSize);
void OLED_ShowString(uint8_t X, uint8_t Y, char *String, uint8_t FontSize);
void OLED_ShowNum(uint8_t X, uint8_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowSignedNum(uint8_t X, uint8_t Y, int32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowHexNum(uint8_t X, uint8_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowBinNum(uint8_t X, uint8_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowFloatNum(uint8_t X, uint8_t Y, float Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize);
void OLED_ShowChinese(uint8_t X, uint8_t Y, char *Chinese);
void OLED_ShowImage(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image);
void OLED_Printf(uint8_t X, uint8_t Y, uint8_t FontSize, char *format, ...);

/*绘图函数*/
void OLED_SetCursor(uint8_t Page, uint8_t X);
void OLED_DrawPoint(uint8_t X, uint8_t Y);
uint8_t OLED_GetPoint(uint8_t X, uint8_t Y);
void OLED_DrawLine(uint8_t X0, uint8_t Y0, uint8_t X1, uint8_t Y1);
void OLED_DrawRectangle(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height, uint8_t IsFilled);
void OLED_DrawTriangle(uint8_t X0, uint8_t Y0, uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, uint8_t IsFilled);
void OLED_DrawCircle(uint8_t X, uint8_t Y, uint8_t Radius, uint8_t IsFilled);
void OLED_DrawEllipse(uint8_t X, uint8_t Y, uint8_t A, uint8_t B, uint8_t IsFilled);
void OLED_DrawArc(uint8_t X, uint8_t Y, uint8_t Radius, int16_t StartAngle, int16_t EndAngle, uint8_t IsFilled);

/* 软件 I2C 通用读写接口（供同总线上的其他设备使用，如 GY-33） */
HAL_StatusTypeDef OLED_SW_I2C_WriteReg(uint8_t DevAddr, uint8_t Reg, uint8_t Value);
HAL_StatusTypeDef OLED_SW_I2C_ReadRegs(uint8_t DevAddr, uint8_t Reg, uint8_t *Buf, uint16_t Len);
HAL_StatusTypeDef OLED_SW_I2C_IsDeviceReady(uint8_t DevAddr, uint8_t Trials);

/*********************函数声明*/

#endif


/*****************江协科技|版权所有****************/
/*****************jiangxiekeji.com*****************/
