/**
 * @file    key.c
 * @brief   按键消抖实现 —— 见 key.h
 */

#include "Common_used.h"
#include "key.h"

#define LIMIT_PORT   GPIOB
#define LIMIT_PIN    GPIO_PIN_0    /* 限位开关 */
#define START_PORT   GPIOA
#define START_PIN    GPIO_PIN_0    /* 启动开关, 即 main.h 的 START_Pin */

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       stable;    /* 消抖后状态: 1 = 按下 */
    uint8_t       cnt;       /* 与 stable 相反的连续采样数 */
    uint8_t       pressed;   /* 有按下边沿, 读后清 */
    uint8_t       released;  /* 有抬起边沿, 读后清 */
} KeyCtx_t;

static KeyCtx_t s_key[KEY_COUNT] = {
    [KEY_LIMIT] = { .port = LIMIT_PORT, .pin = LIMIT_PIN },
    [KEY_START] = { .port = START_PORT, .pin = START_PIN },
};

static uint8_t raw_pressed(const KeyCtx_t *k)
{
    return ((uint8_t)HAL_GPIO_ReadPin(k->port, k->pin) == KEY_ACTIVE_LEVEL) ? 1u : 0u;
}

void Key_Init(void)
{
    /* 按当前电平取初值, 免得开关本来按着时上电就报一次边沿 */
    for (uint8_t i = 0; i < (uint8_t)KEY_COUNT; i++) {
        s_key[i].stable   = raw_pressed(&s_key[i]);
        s_key[i].cnt      = 0u;
        s_key[i].pressed  = 0u;
        s_key[i].released = 0u;
    }
}

void Key_Update(void)
{
    for (uint8_t i = 0; i < (uint8_t)KEY_COUNT; i++) {
        KeyCtx_t *k = &s_key[i];
        uint8_t   raw = raw_pressed(k);

        if (raw == k->stable) {
            k->cnt = 0u;            /* 有一拍回到原电平, 抖动计数作废 */
        } else if (++k->cnt >= (uint8_t)KEY_DEBOUNCE_SAMPLES) {
            k->cnt    = 0u;
            k->stable = raw;
            if (raw != 0u) {
                k->pressed = 1u;
            } else {
                k->released = 1u;
            }
        }
    }
}

bool Key_IsPressed(KeyId_t k)
{
    return ((uint8_t)k < (uint8_t)KEY_COUNT) && (s_key[(uint8_t)k].stable != 0u);
}

bool Key_WasPressed(KeyId_t k)
{
    if ((uint8_t)k >= (uint8_t)KEY_COUNT || s_key[(uint8_t)k].pressed == 0u) {
        return false;
    }
    s_key[(uint8_t)k].pressed = 0u;
    return true;
}

bool Key_WasReleased(KeyId_t k)
{
    if ((uint8_t)k >= (uint8_t)KEY_COUNT || s_key[(uint8_t)k].released == 0u) {
        return false;
    }
    s_key[(uint8_t)k].released = 0u;
    return true;
}
