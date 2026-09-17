/**
 * @file    locator_dev.h
 * @brief   定位设备抽象接口 --- 抽象设备层 (Device)
 *
 * 规范来源：《机器人嵌入式代码架构规范与修改日志》V1.0 第 3 节。
 *
 * @note    本文件为【目标架构】定义的接口骨架，目前**没有任何实现**。
 *
 * @warning 禁止修改已有函数指针的定义与参数格式；
 *          只能扩展新字段，不得删除/修改既有成员。
 */

#ifndef _LOCATOR_DEV_H_
#define _LOCATOR_DEV_H_

#include <stdint.h>
#include "pose_data.h"

/**
 * @brief 定位设备抽象接口
 * @note 新增定位源必须完整实现以下四个函数指针
 */
typedef struct {
    /* 设备初始化：硬件外设初始化、协议初始化 */
    void (*init)(void);

    /* 周期更新：解析硬件数据、更新内部缓存、校验数据有效性 */
    void (*update)(void);

    /* 获取最新位姿：将内部缓存的位姿数据输出到 pose_out */
    void (*get_pose)(PoseData_t *pose_out);

    /* 设备健康检查：返回设备是否在线、数据是否可信 */
    uint8_t (*is_healthy)(void);
} LocatorDev_t;

/* ============================================================
 * 已注册设备实例（规范 3.2）
 *
 * 已实现：
 *     locator_wheel         轮式里程计        device/drv_wheel_odom.c
 *
 * 尚未实现（声明留待各自驱动文件建成后补充；此处不预先 extern 声明，
 * 否则一旦被引用就会产生 undefined reference 链接错误）：
 *     locator_ops9          OPS9 光学定位     device/drv_ops9.c
 *     locator_optical_flow  光流传感器        device/drv_optical_flow.c
 *
 * 业务层标准调用方式（切换传感器只改 active_locator 这一行）：
 *
 *     const LocatorDev_t *active_locator = &locator_wheel;
 *
 *     active_locator->init();
 *     // 周期循环（固定周期，全局只调一处）：
 *     active_locator->update();
 *     // 任意位置按需纯读取：
 *     active_locator->get_pose(&robot_pose);
 * ============================================================ */

/* 轮式里程计实例，定义于 device/drv_wheel_odom.c */
extern const LocatorDev_t locator_wheel;

#endif /* _LOCATOR_DEV_H_ */