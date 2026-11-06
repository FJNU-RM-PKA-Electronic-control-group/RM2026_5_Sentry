/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  Wang Hongxi
 * @version V1.1.0
 * @date    2022/3/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "bsp_dwt.h"

DWT_Time_t SysTime;
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RountCount;
static uint32_t CYCCNT_LAST;
uint64_t CYCCNT64;

#define DWT_CYCCNT_CYCLE 4294967296ULL // 32位CYCCNT回绕一圈的周期数(2^32)

static uint64_t DWT_CNT_Update(void);

void DWT_Init(uint32_t CPU_Freq_mHz) {
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz       = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms    = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us    = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;
}

float DWT_GetDeltaT(uint32_t *cnt_last) {
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt                  = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last                 = cnt_now;

    return dt;
}

double DWT_GetDeltaT64(uint32_t *cnt_last) {
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt                 = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last                 = cnt_now;

    return dt;
}

void DWT_SysTimeUpdate(void) {
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    CYCCNT64 = DWT_CNT_Update(); // 只在DWT_CNT_Update内部读一次CYCCNT

    CNT_TEMP1  = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2  = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s  = CNT_TEMP1;
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3  = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

float DWT_GetTimeline_s(void) {
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}

float DWT_GetTimeline_ms(void) {
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

    return DWT_Timelinef32;
}

uint64_t DWT_GetTimeline_us(void) {
    DWT_SysTimeUpdate();

    uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

    return DWT_Timelinef32;
}

/**
 * @brief 读取CYCCNT并累加回绕圈数,返回自DWT_Init以来的绝对周期数
 * @note  圈数与计数值必须在同一临界区内拼合:多个任务都会调用本函数,
 *        若分成两次出临界区,可能取到"新圈数+旧计数",时间会前跳2^32个周期(约25.56s)
 */
static uint64_t DWT_CNT_Update(void) {
    uint32_t irq = __get_PRIMASK();
    __disable_irq();

    volatile uint32_t cnt_now = DWT->CYCCNT; // 整个函数只读这一次
    if (cnt_now < CYCCNT_LAST)
        CYCCNT_RountCount++;
    CYCCNT_LAST       = cnt_now;
    uint64_t cyccnt64 = (uint64_t)CYCCNT_RountCount * DWT_CYCCNT_CYCLE + (uint64_t)cnt_now;

    __set_PRIMASK(irq);
    return cyccnt64;
}

void DWT_Delay(float Delay) {
    uint32_t tickstart = DWT->CYCCNT;
    float wait         = Delay;

    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz) {
    }
}
