/**
 * @file    wit_protocol.c
 * @brief   WitMotion 传感器协议栈实现。
 * @note    运行时选择 Normal / Modbus / CAN / I2C 协议。
 *          本文件仅依赖 wit_protocol.h，不直接依赖任何 HAL 或 OS。
 */

#include "Common_used.h"
#include "wit_protocol.h"

/* ========================================================================
   内部状态
   ======================================================================== */

static SerialWrite   p_WitSerialWriteFunc  = NULL;
static WitI2cWrite   p_WitI2cWriteFunc     = NULL;
static WitI2cRead    p_WitI2cReadFunc      = NULL;
static CanWrite      p_WitCanWriteFunc     = NULL;
static RegUpdateCb   p_WitRegUpdateCbFunc  = NULL;
static DelaymsCb     p_WitDelaymsFunc      = NULL;

static uint8_t  s_ucAddr          = 0xFFU;
static uint8_t  s_ucWitDataBuff[WIT_DATA_BUFF_SIZE];
static uint32_t s_uiWitDataCnt    = 0U;
static uint32_t s_uiProtoclo      = 0U;
static uint32_t s_uiReadRegIndex  = 0U;

int16_t sReg[REGSIZE];

#define FUNC_W  0x06U
#define FUNC_R  0x03U

/* ========================================================================
   CRC16 Modbus 查找表
   ======================================================================== */

static const uint8_t __auchCRCHi[256] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
};

static const uint8_t __auchCRCLo[256] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
    0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40,
};

/* ========================================================================
   内部辅助函数
   ======================================================================== */

static uint16_t CRC16(uint8_t *puchMsg, uint16_t usDataLen)
{
    uint8_t uchCRCHi = 0xFFU;
    uint8_t uchCRCLo = 0xFFU;
    uint8_t uIndex;
    for (uint16_t i = 0U; i < usDataLen; i++)
    {
        uIndex  = uchCRCHi ^ puchMsg[i];
        uchCRCHi = uchCRCLo ^ __auchCRCHi[uIndex];
        uchCRCLo = __auchCRCLo[uIndex];
    }
    return (uint16_t)(((uint16_t)uchCRCHi << 8) | (uint16_t)uchCRCLo);
}

static uint8_t CaliSum(uint8_t *data, uint32_t len)
{
    uint8_t ucCheck = 0U;
    for (uint32_t i = 0U; i < len; i++) ucCheck += data[i];
    return ucCheck;
}

static char CheckRange(short sTemp, short sMin, short sMax)
{
    return (sTemp >= sMin && sTemp <= sMax) ? 1 : 0;
}

/* ========================================================================
   数据解析分发
   ======================================================================== */

static void CopeWitData(uint8_t ucIndex, uint16_t *p_data, uint32_t uiLen)
{
    uint32_t uiReg1 = 0U, uiReg2 = 0U, uiReg1Len = 0U, uiReg2Len = 0U;
    uint16_t *p_usReg1Val = p_data;
    uint16_t *p_usReg2Val = p_data + 3;

    uiReg1Len = 4U;
    switch (ucIndex)
    {
    case WIT_ACC:       uiReg1 = AX;    uiReg1Len = 3U;  uiReg2 = TEMP;     uiReg2Len = 1U;  break;
    case WIT_ANGLE:     uiReg1 = Roll;  uiReg1Len = 3U;  uiReg2 = VERSION;  uiReg2Len = 1U;  break;
    case WIT_TIME:      uiReg1 = YYMM;   break;
    case WIT_GYRO:      uiReg1 = GX;     uiLen = 3U;     break;
    case WIT_MAGNETIC:  uiReg1 = HX;     uiLen = 3U;     break;
    case WIT_DPORT:     uiReg1 = D0Status;               break;
    case WIT_PRESS:     uiReg1 = PressureL;              break;
    case WIT_GPS:       uiReg1 = LonL;                   break;
    case WIT_VELOCITY:  uiReg1 = GPSHeight;              break;
    case WIT_QUATER:    uiReg1 = q0;                     break;
    case WIT_GSA:       uiReg1 = SVNUM;                  break;
    case WIT_REGVALUE:  uiReg1 = s_uiReadRegIndex;       break;
    default:
        return;
    }

    if (uiLen == 3U)
    {
        uiReg1Len = 3U;
        uiReg2Len = 0U;
    }
    if (uiReg1Len)
    {
        memcpy(&sReg[uiReg1], p_usReg1Val, uiReg1Len << 1);
        if (p_WitRegUpdateCbFunc) p_WitRegUpdateCbFunc(uiReg1, uiReg1Len);
    }
    if (uiReg2Len)
    {
        memcpy(&sReg[uiReg2], p_usReg2Val, uiReg2Len << 1);
        if (p_WitRegUpdateCbFunc) p_WitRegUpdateCbFunc(uiReg2, uiReg2Len);
    }
}

/* ========================================================================
   串口 / CAN 数据输入（由 ISR 调用）
   ======================================================================== */

void WitSerialDataIn(uint8_t ucData)
{
    uint16_t usCRC16, usTemp, i, usData[4];
    uint8_t  ucSum;

    if (p_WitRegUpdateCbFunc == NULL) return;

    s_ucWitDataBuff[s_uiWitDataCnt++] = ucData;

    switch (s_uiProtoclo)
    {
    case WIT_PROTOCOL_NORMAL:
        if (s_ucWitDataBuff[0] != 0x55U)
        {
            s_uiWitDataCnt--;
            memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
            return;
        }
        if (s_uiWitDataCnt >= 11U)
        {
            ucSum = CaliSum(s_ucWitDataBuff, 10U);
            if (ucSum != s_ucWitDataBuff[10])
            {
                s_uiWitDataCnt--;
                memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                return;
            }
            usData[0] = ((uint16_t)s_ucWitDataBuff[3] << 8) | s_ucWitDataBuff[2];
            usData[1] = ((uint16_t)s_ucWitDataBuff[5] << 8) | s_ucWitDataBuff[4];
            usData[2] = ((uint16_t)s_ucWitDataBuff[7] << 8) | s_ucWitDataBuff[6];
            usData[3] = ((uint16_t)s_ucWitDataBuff[9] << 8) | s_ucWitDataBuff[8];
            CopeWitData(s_ucWitDataBuff[1], usData, 4U);
            s_uiWitDataCnt = 0U;
        }
        break;

    case WIT_PROTOCOL_MODBUS:
        if (s_uiWitDataCnt > 2U)
        {
            if (s_ucWitDataBuff[1] != FUNC_R)
            {
                s_uiWitDataCnt--;
                memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                return;
            }
            if (s_uiWitDataCnt < (uint32_t)(s_ucWitDataBuff[2] + 5U)) return;
            usTemp  = ((uint16_t)s_ucWitDataBuff[s_uiWitDataCnt - 2U] << 8)
                    |  s_ucWitDataBuff[s_uiWitDataCnt - 1U];
            usCRC16 = CRC16(s_ucWitDataBuff, s_uiWitDataCnt - 2U);
            if (usTemp != usCRC16)
            {
                s_uiWitDataCnt--;
                memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                return;
            }
            usTemp = (uint16_t)(s_ucWitDataBuff[2] >> 1);
            for (i = 0U; i < usTemp; i++)
            {
                sReg[i + s_uiReadRegIndex] =
                    ((uint16_t)s_ucWitDataBuff[(i << 1) + 3U] << 8)
                    | s_ucWitDataBuff[(i << 1) + 4U];
            }
            if (p_WitRegUpdateCbFunc) p_WitRegUpdateCbFunc(s_uiReadRegIndex, usTemp);
            s_uiWitDataCnt = 0U;
        }
        break;

    case WIT_PROTOCOL_CAN:
    case WIT_PROTOCOL_I2C:
        s_uiWitDataCnt = 0U;
        break;
    }
    if (s_uiWitDataCnt == WIT_DATA_BUFF_SIZE) s_uiWitDataCnt = 0U;
}

void WitCanDataIn(uint8_t ucData[8], uint8_t ucLen)
{
    uint16_t usData[3];
    if (p_WitRegUpdateCbFunc == NULL) return;
    if (ucLen < 8U) return;

    if (s_uiProtoclo == WIT_PROTOCOL_CAN)
    {
        if (ucData[0] != 0x55U) return;
        usData[0] = ((uint16_t)ucData[3] << 8) | ucData[2];
        usData[1] = ((uint16_t)ucData[5] << 8) | ucData[4];
        usData[2] = ((uint16_t)ucData[7] << 8) | ucData[6];
        CopeWitData(ucData[1], usData, 3U);
    }
}

/* ========================================================================
   回调注册
   ======================================================================== */

int32_t WitSerialWriteRegister(SerialWrite Write_func)
{
    if (!Write_func) return WIT_HAL_INVAL;
    p_WitSerialWriteFunc = Write_func;
    return WIT_HAL_OK;
}

int32_t WitI2cFuncRegister(WitI2cWrite write_func, WitI2cRead read_func)
{
    if (!write_func) return WIT_HAL_INVAL;
    if (!read_func)  return WIT_HAL_INVAL;
    p_WitI2cWriteFunc = write_func;
    p_WitI2cReadFunc  = read_func;
    return WIT_HAL_OK;
}

int32_t WitCanWriteRegister(CanWrite Write_func)
{
    if (!Write_func) return WIT_HAL_INVAL;
    p_WitCanWriteFunc = Write_func;
    return WIT_HAL_OK;
}

int32_t WitRegisterCallBack(RegUpdateCb update_func)
{
    if (!update_func) return WIT_HAL_INVAL;
    p_WitRegUpdateCbFunc = update_func;
    return WIT_HAL_OK;
}

int32_t WitDelayMsRegister(DelaymsCb delayms_func)
{
    if (!delayms_func) return WIT_HAL_INVAL;
    p_WitDelaymsFunc = delayms_func;
    return WIT_HAL_OK;
}

/* ========================================================================
   协议栈初始化 / 反初始化
   ======================================================================== */

int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr)
{
    if (uiProtocol > WIT_PROTOCOL_I2C) return WIT_HAL_INVAL;
    s_uiProtoclo   = uiProtocol;
    s_ucAddr       = ucAddr;
    s_uiWitDataCnt = 0U;
    return WIT_HAL_OK;
}

void WitDeInit(void)
{
    p_WitSerialWriteFunc = NULL;
    p_WitI2cWriteFunc    = NULL;
    p_WitI2cReadFunc     = NULL;
    p_WitCanWriteFunc    = NULL;
    p_WitRegUpdateCbFunc = NULL;
    s_ucAddr             = 0xFFU;
    s_uiWitDataCnt       = 0U;
    s_uiProtoclo         = 0U;
}

/* ========================================================================
   寄存器读写
   ======================================================================== */

int32_t WitWriteReg(uint32_t uiReg, uint16_t usData)
{
    uint16_t usCRC;
    uint8_t  ucBuff[8];

    if (uiReg >= REGSIZE) return WIT_HAL_INVAL;

    switch (s_uiProtoclo)
    {
    case WIT_PROTOCOL_NORMAL:
        if (p_WitSerialWriteFunc == NULL) return WIT_HAL_EMPTY;
        ucBuff[0] = 0xFFU;
        ucBuff[1] = 0xAAU;
        ucBuff[2] = (uint8_t)(uiReg & 0xFFU);
        ucBuff[3] = (uint8_t)(usData & 0xFFU);
        ucBuff[4] = (uint8_t)(usData >> 8);
        p_WitSerialWriteFunc(ucBuff, 5U);
        break;

    case WIT_PROTOCOL_MODBUS:
        if (p_WitSerialWriteFunc == NULL) return WIT_HAL_EMPTY;
        ucBuff[0] = s_ucAddr;
        ucBuff[1] = FUNC_W;
        ucBuff[2] = (uint8_t)(uiReg >> 8);
        ucBuff[3] = (uint8_t)(uiReg & 0xFFU);
        ucBuff[4] = (uint8_t)(usData >> 8);
        ucBuff[5] = (uint8_t)(usData & 0xFFU);
        usCRC = CRC16(ucBuff, 6U);
        ucBuff[6] = (uint8_t)(usCRC >> 8);
        ucBuff[7] = (uint8_t)(usCRC & 0xFFU);
        p_WitSerialWriteFunc(ucBuff, 8U);
        break;

    case WIT_PROTOCOL_CAN:
        if (p_WitCanWriteFunc == NULL) return WIT_HAL_EMPTY;
        ucBuff[0] = 0xFFU;
        ucBuff[1] = 0xAAU;
        ucBuff[2] = (uint8_t)(uiReg & 0xFFU);
        ucBuff[3] = (uint8_t)(usData & 0xFFU);
        ucBuff[4] = (uint8_t)(usData >> 8);
        p_WitCanWriteFunc(s_ucAddr, ucBuff, 5U);
        break;

    case WIT_PROTOCOL_I2C:
        if (p_WitI2cWriteFunc == NULL) return WIT_HAL_EMPTY;
        ucBuff[0] = (uint8_t)(usData & 0xFFU);
        ucBuff[1] = (uint8_t)(usData >> 8);
        if (p_WitI2cWriteFunc(s_ucAddr << 1, (uint8_t)uiReg, ucBuff, 2U) != 1)
        {
            /* I2C write fail — caller can check return code */
        }
        break;

    default:
        return WIT_HAL_INVAL;
    }
    return WIT_HAL_OK;
}

int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum)
{
    uint16_t usTemp, i;
    uint8_t  ucBuff[8];

    if ((uiReg + uiReadNum) >= REGSIZE) return WIT_HAL_INVAL;

    switch (s_uiProtoclo)
    {
    case WIT_PROTOCOL_NORMAL:
        if (uiReadNum > 4U) return WIT_HAL_INVAL;
        if (p_WitSerialWriteFunc == NULL) return WIT_HAL_EMPTY;
        ucBuff[0] = 0xFFU;
        ucBuff[1] = 0xAAU;
        ucBuff[2] = 0x27U;
        ucBuff[3] = (uint8_t)(uiReg & 0xFFU);
        ucBuff[4] = (uint8_t)(uiReg >> 8);
        p_WitSerialWriteFunc(ucBuff, 5U);
        break;

    case WIT_PROTOCOL_MODBUS:
        if (p_WitSerialWriteFunc == NULL) return WIT_HAL_EMPTY;
        usTemp = (uint16_t)(uiReadNum << 1);
        if ((usTemp + 5U) > WIT_DATA_BUFF_SIZE) return WIT_HAL_NOMEM;
        ucBuff[0] = s_ucAddr;
        ucBuff[1] = FUNC_R;
        ucBuff[2] = (uint8_t)(uiReg >> 8);
        ucBuff[3] = (uint8_t)(uiReg & 0xFFU);
        ucBuff[4] = (uint8_t)(uiReadNum >> 8);
        ucBuff[5] = (uint8_t)(uiReadNum & 0xFFU);
        usTemp = CRC16(ucBuff, 6U);
        ucBuff[6] = (uint8_t)(usTemp >> 8);
        ucBuff[7] = (uint8_t)(usTemp & 0xFFU);
        p_WitSerialWriteFunc(ucBuff, 8U);
        break;

    case WIT_PROTOCOL_CAN:
        if (uiReadNum > 3U) return WIT_HAL_INVAL;
        if (p_WitCanWriteFunc == NULL) return WIT_HAL_EMPTY;
        ucBuff[0] = 0xFFU;
        ucBuff[1] = 0xAAU;
        ucBuff[2] = 0x27U;
        ucBuff[3] = (uint8_t)(uiReg & 0xFFU);
        ucBuff[4] = (uint8_t)(uiReg >> 8);
        p_WitCanWriteFunc(s_ucAddr, ucBuff, 5U);
        break;

    case WIT_PROTOCOL_I2C:
        if (p_WitI2cReadFunc == NULL) return WIT_HAL_EMPTY;
        usTemp = (uint16_t)(uiReadNum << 1);
        if (WIT_DATA_BUFF_SIZE < usTemp) return WIT_HAL_NOMEM;
        if (p_WitI2cReadFunc(s_ucAddr << 1, (uint8_t)uiReg,
                             s_ucWitDataBuff, usTemp) == 1)
        {
            if (p_WitRegUpdateCbFunc == NULL) return WIT_HAL_EMPTY;
            for (i = 0U; i < uiReadNum; i++)
            {
                sReg[i + uiReg] =
                    ((uint16_t)s_ucWitDataBuff[(i << 1) + 1U] << 8)
                    | s_ucWitDataBuff[i << 1];
            }
            p_WitRegUpdateCbFunc(uiReg, uiReadNum);
        }
        break;

    default:
        return WIT_HAL_INVAL;
    }
    s_uiReadRegIndex = uiReg;
    return WIT_HAL_OK;
}

/* ========================================================================
   传感器配置函数
   ======================================================================== */

int32_t WitSetUartBaud(int32_t uiBaudIndex)
{
    if (!CheckRange((short)uiBaudIndex, WIT_BAUD_4800, WIT_BAUD_921600))
        return WIT_HAL_INVAL;
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    if (s_uiProtoclo == WIT_PROTOCOL_MODBUS)
        p_WitDelaymsFunc(20U);
    else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL)
        p_WitDelaymsFunc(1U);
    return WitWriteReg(BAUD, (uint16_t)uiBaudIndex);
}

int32_t WitSetCanBaud(int32_t uiBaudIndex)
{
    if (!CheckRange((short)uiBaudIndex, CAN_BAUD_3000, CAN_BAUD_1000000))
        return WIT_HAL_INVAL;
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    if (s_uiProtoclo == WIT_PROTOCOL_MODBUS)
        p_WitDelaymsFunc(20U);
    else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL)
        p_WitDelaymsFunc(1U);
    return WitWriteReg(BAUD, (uint16_t)uiBaudIndex);
}

int32_t WitSetBandwidth(int32_t uiBandwidth)
{
    if (!CheckRange((short)uiBandwidth, BANDWIDTH_256HZ, BANDWIDTH_5HZ))
        return WIT_HAL_INVAL;
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    if (s_uiProtoclo == WIT_PROTOCOL_MODBUS)
        p_WitDelaymsFunc(20U);
    else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL)
        p_WitDelaymsFunc(1U);
    return WitWriteReg(BANDWIDTH, (uint16_t)uiBandwidth);
}

int32_t WitSetOutputRate(int32_t uiRate)
{
    if (!CheckRange((short)uiRate, RRATE_02HZ, RRATE_NONE))
        return WIT_HAL_INVAL;
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    if (s_uiProtoclo == WIT_PROTOCOL_MODBUS)
        p_WitDelaymsFunc(20U);
    else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL)
        p_WitDelaymsFunc(1U);
    return WitWriteReg(RRATE, (uint16_t)uiRate);
}

int32_t WitSetContent(int32_t uiRsw)
{
    if (!CheckRange((short)uiRsw, RSW_TIME, RSW_MASK))
        return WIT_HAL_INVAL;
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    if (s_uiProtoclo == WIT_PROTOCOL_MODBUS)
        p_WitDelaymsFunc(20U);
    else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL)
        p_WitDelaymsFunc(1U);
    return WitWriteReg(RSW, (uint16_t)uiRsw);
}

/* ========================================================================
   校准函数
   ======================================================================== */

static void CaliDelay(void)
{
    if (s_uiProtoclo == WIT_PROTOCOL_MODBUS)
        p_WitDelaymsFunc(20U);
    else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL)
        p_WitDelaymsFunc(1U);
}

int32_t WitStartAccCali(void)
{
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    CaliDelay();
    return WitWriteReg(CALSW, CALGYROACC);
}

int32_t WitStopAccCali(void)
{
    if (WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK) return WIT_HAL_ERROR;
    CaliDelay();
    return WitWriteReg(SAVE, SAVE_PARAM);
}

int32_t WitStartMagCali(void)
{
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    CaliDelay();
    return WitWriteReg(CALSW, CALMAGMM);
}

int32_t WitStopMagCali(void)
{
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    CaliDelay();
    return WitWriteReg(CALSW, NORMAL);
}

int32_t WitStartANGLEZCali(void)
{
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    CaliDelay();
    return WitWriteReg(CALSW, CALANGLEZ);
}

int32_t WitStopANGLEZCali(void)
{
    if (WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK) return WIT_HAL_ERROR;
    CaliDelay();
    return WitWriteReg(SAVE, SAVE_PARAM);
}

int32_t WitStartREFANGLECali(void)
{
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    CaliDelay();
    return WitWriteReg(CALSW, CALREFANGLE);
}

int32_t WitStopREFANGLECali(void)
{
    if (WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK) return WIT_HAL_ERROR;
    CaliDelay();
    return WitWriteReg(SAVE, SAVE_PARAM);
}

int32_t WitStartIYAWCali(void)
{
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    CaliDelay();
    return WitWriteReg(0x76U, 0x00U);
}

int32_t WitStartRKMODECali(void)
{
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    CaliDelay();
    return WitWriteReg(0x48U, 0x01U);
}

int32_t WitStopRKMODECali(void)
{
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    CaliDelay();
    return WitWriteReg(0x48U, 0x00U);
}

int32_t WitStartALGRITHM6Cali(void)
{
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    CaliDelay();
    return WitWriteReg(AXIS6, ALGRITHM6);
}

int32_t WitStopALGRITHM6Cali(void)
{
    if (WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK) return WIT_HAL_ERROR;
    CaliDelay();
    return WitWriteReg(SAVE, SAVE_PARAM);
}