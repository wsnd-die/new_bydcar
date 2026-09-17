/**
 * @file    wit_protocol.h
 * @brief   WitMotion (维特智能) 传感器协议层 —— 类型、常量、寄存器定义与接口。
 * @note    支持 Normal / Modbus / CAN / I2C 四种协议，运行时通过 WitInit() 选择。
 * @version 0.1
 * @date    2026-08-02
 */

#ifndef __WIT_PROTOCOL_H
#define __WIT_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========================================================================
   基础常量
   ======================================================================== */

#define WIT_DATA_BUFF_SIZE  256U    /* 内部数据缓冲区大小              */
#define REGSIZE             128U    /* 寄存器表大小                    */

/* ========================================================================
   协议类型枚举（运行时选择）
   ======================================================================== */

#define WIT_PROTOCOL_NORMAL  0x00U  /* 主动上报 0x55 开头             */
#define WIT_PROTOCOL_MODBUS  0x01U  /* Modbus-RTU                     */
#define WIT_PROTOCOL_CAN     0x02U  /* CAN 总线                       */
#define WIT_PROTOCOL_I2C     0x03U  /* I2C 总线（本工程默认）          */

/* ========================================================================
   回调函数类型
   ======================================================================== */

typedef void     (*SerialWrite)(uint8_t *p_ucData, uint32_t uiLen);
typedef int32_t  (*WitI2cWrite)(uint8_t ucAddr, uint8_t ucReg,
                                uint8_t *p_ucData, uint16_t usLen);
typedef int32_t  (*WitI2cRead)(uint8_t ucAddr, uint8_t ucReg,
                                uint8_t *p_ucData, uint16_t usLen);
typedef void     (*CanWrite)(uint8_t ucAddr, uint8_t *p_ucData, uint8_t ucLen);
typedef void     (*RegUpdateCb)(uint32_t uiReg, uint32_t uiRegNum);
typedef void     (*DelaymsCb)(uint16_t ucMs);

/* ========================================================================
   返回码
   ======================================================================== */

#define WIT_HAL_OK       0
#define WIT_HAL_ERROR   -1
#define WIT_HAL_INVAL   -2
#define WIT_HAL_EMPTY   -3
#define WIT_HAL_NOMEM   -4

/* ========================================================================
   寄存器索引（sReg[] 下标）
   ======================================================================== */

#define AX        0x34U   /* 加速度 X                        */
#define AY        0x35U   /* 加速度 Y                        */
#define AZ        0x36U   /* 加速度 Z                        */
#define GX        0x37U   /* 角速度 X                        */
#define GY        0x38U   /* 角速度 Y                        */
#define GZ        0x39U   /* 角速度 Z                        */
#define HX        0x3AU   /* 磁场 X                          */
#define HY        0x3BU   /* 磁场 Y                          */
#define HZ        0x3CU   /* 磁场 Z                          */
#define Roll      0x3DU   /* 横滚角 (x10^-2 °)               */
#define Pitch     0x3EU   /* 俯仰角 (x10^-2 °)               */
#define Yaw       0x3FU   /* 偏航角 (x10^-2 °)               */
#define TEMP      0x40U   /* 温度                            */
#define D0Status  0x41U   /* D0 状态                         */
#define D1Status  0x42U   /* D1 状态                         */
#define D2Status  0x43U   /* D2 状态                         */
#define D3Status  0x44U   /* D3 状态                         */
#define PressureL 0x45U   /* 气压低字节                      */
#define PressureH 0x46U   /* 气压高字节                      */
#define HeightL   0x47U   /* 高度低字节                      */
#define HeightH   0x48U   /* 高度高字节                      */
#define LonL      0x49U   /* 经度低字节                      */
#define LonH      0x4AU   /* 经度高字节                      */
#define LatL      0x4BU   /* 纬度低字节                      */
#define LatH      0x4CU   /* 纬度高字节                      */
#define GPSHeight 0x4DU   /* GPS 高度                        */
#define GPSYaw    0x4EU   /* GPS 航向                        */
#define GPSV      0x4FU   /* GPS 速度                        */
#define q0        0x50U   /* 四元数 q0                       */
#define q1        0x51U   /* 四元数 q1                       */
#define q2        0x52U   /* 四元数 q2                       */
#define q3        0x53U   /* 四元数 q3                       */
#define SVNUM     0x54U   /* 卫星数量                        */
#define VERSION   0x55U   /* 版本号                          */
#define YYMM      0x56U   /* 年月 (BCD)                      */
#define DDHH      0x57U   /* 日时 (BCD)                      */
#define MMSS      0x58U   /* 分秒 (BCD)                      */
#define MS        0x59U   /* 毫秒                            */

/* 控制寄存器（写） */
#define KEY       0x69U   /* 解锁寄存器                      */
#define SAVE      0x00U   /* 保存参数                        */
#define CALSW     0x01U   /* 校准开关                        */
#define BAUD      0x04U   /* 串口/CAN 波特率                  */
#define BANDWIDTH 0x1FU   /* 回传带宽                        */
#define RRATE     0x03U   /* 回传速率                        */
#define RSW       0x02U   /* 回传内容                        */
#define AXIS6     0x06U   /* 6 轴算法                        */

/* ========================================================================
   寄存器参数枚举
   ======================================================================== */

/* -- SAVE 保存 -- */
#define SAVE_PARAM  0x00U

/* -- CALSW 校准 -- */
#define NORMAL      0x00U
#define CALGYROACC  0x01U
#define CALMAGMM    0x02U
#define CALANGLEZ   0x05U
#define CALREFANGLE 0x03U

/* -- KEY 解锁 -- */
#define KEY_UNLOCK  0xB588U

/* -- BAUD 波特率（串口） -- */
#define WIT_BAUD_4800     0x01U
#define WIT_BAUD_9600     0x02U
#define WIT_BAUD_19200    0x03U
#define WIT_BAUD_38400    0x04U
#define WIT_BAUD_57600    0x05U
#define WIT_BAUD_115200   0x06U
#define WIT_BAUD_230400   0x07U
#define WIT_BAUD_460800   0x08U
#define WIT_BAUD_921600   0x09U

/* -- BAUD 波特率（CAN） -- */
#define CAN_BAUD_1000000  0x01U
#define CAN_BAUD_800000   0x02U
#define CAN_BAUD_500000   0x03U
#define CAN_BAUD_400000   0x04U
#define CAN_BAUD_250000   0x05U
#define CAN_BAUD_125000   0x06U
#define CAN_BAUD_100000   0x07U
#define CAN_BAUD_50000    0x08U
#define CAN_BAUD_20000    0x09U
#define CAN_BAUD_10000    0x0AU
#define CAN_BAUD_5000     0x0BU
#define CAN_BAUD_3000     0x0CU

/* -- BANDWIDTH 回传带宽 -- */
#define BANDWIDTH_256HZ   0x00U
#define BANDWIDTH_200HZ   0x01U
#define BANDWIDTH_100HZ   0x02U
#define BANDWIDTH_50HZ    0x03U
#define BANDWIDTH_20HZ    0x04U
#define BANDWIDTH_10HZ    0x05U
#define BANDWIDTH_5HZ     0x06U

/* -- RRATE 回传速率 -- */
#define RRATE_02HZ    0x01U
#define RRATE_05HZ    0x02U
#define RRATE_1HZ     0x03U
#define RRATE_2HZ     0x04U
#define RRATE_5HZ     0x05U
#define RRATE_10HZ    0x06U
#define RRATE_20HZ    0x07U
#define RRATE_50HZ    0x08U
#define RRATE_100HZ   0x09U
#define RRATE_200HZ   0x0AU
#define RRATE_NONE    0x0BU

/* -- RSW 回传内容 -- */
#define RSW_TIME    0x01U
#define RSW_ACC     0x02U
#define RSW_GYRO    0x04U
#define RSW_ANGLE   0x08U
#define RSW_MAG     0x10U
#define RSW_PORT    0x20U
#define RSW_PRESS   0x40U
#define RSW_GPS     0x80U
#define RSW_MASK    0xFFFFU

/* -- AXIS6 算法 -- */
#define ALGRITHM6  0x01U

/* ---- 数据包标识 ---- */
#define WIT_ACC        0x51U
#define WIT_GYRO       0x52U
#define WIT_ANGLE      0x53U
#define WIT_MAGNETIC   0x54U
#define WIT_TIME       0x50U
#define WIT_PRESS      0x56U
#define WIT_GPS        0x57U
#define WIT_VELOCITY   0x58U
#define WIT_QUATER     0x59U
#define WIT_GSA        0x5AU
#define WIT_DPORT      0x41U
#define WIT_REGVALUE   0x5FU

/* ========================================================================
   全局数据
   ======================================================================== */

extern int16_t sReg[REGSIZE];

/* ========================================================================
   协议 API
   ======================================================================== */

/* 初始化协议栈，选择协议类型和设备地址 */
int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr);
void    WitDeInit(void);

/* 注册底层通信回调 */
int32_t WitSerialWriteRegister(SerialWrite Write_func);
int32_t WitI2cFuncRegister(WitI2cWrite write_func, WitI2cRead read_func);
int32_t WitCanWriteRegister(CanWrite Write_func);
int32_t WitDelayMsRegister(DelaymsCb delayms_func);
int32_t WitRegisterCallBack(RegUpdateCb update_func);

/* 读写寄存器 */
int32_t WitWriteReg(uint32_t uiReg, uint16_t usData);
int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum);

/* 数据输入（由底层 ISR / 接收线程调用） */
void WitSerialDataIn(uint8_t ucData);
void WitCanDataIn(uint8_t ucData[8], uint8_t ucLen);

/* 传感器配置 */
int32_t WitSetUartBaud(int32_t uiBaudIndex);
int32_t WitSetCanBaud(int32_t uiBaudIndex);
int32_t WitSetBandwidth(int32_t uiBandwidth);
int32_t WitSetOutputRate(int32_t uiRate);
int32_t WitSetContent(int32_t uiRsw);

/* 校准 */
int32_t WitStartAccCali(void);
int32_t WitStopAccCali(void);
int32_t WitStartMagCali(void);
int32_t WitStopMagCali(void);
int32_t WitStartANGLEZCali(void);
int32_t WitStopANGLEZCali(void);
int32_t WitStartREFANGLECali(void);
int32_t WitStopREFANGLECali(void);
int32_t WitStartIYAWCali(void);
int32_t WitStartRKMODECali(void);
int32_t WitStopRKMODECali(void);
int32_t WitStartALGRITHM6Cali(void);
int32_t WitStopALGRITHM6Cali(void);

#ifdef __cplusplus
}
#endif

#endif /* __WIT_PROTOCOL_H */