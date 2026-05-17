#ifndef __FEED_FORWARD_H
#define __FEED_FORWARD_H

#include "bsp_dwt.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"

#define FF_CNT 3

typedef struct {
    float
        format_value; // ��ѭ���޷��ķ�Χ��0��ʾ�����޷�������=180��ʾ���޷���-180��180
    float K1;
    float K2;
    float max_out;
    float RC;
    uint32_t DWT_CNT;
    float dt;

    float *set_ptr;
    float set;
    float previous_set;
    float pre_previous_set;

    float temp_out;  // δ�˲���out
    float last_out;
    float final_out; // �����˲���out
    float output;
} FeedForwardInstance;

typedef struct {
    float
        format_value; // ��ѭ���޷��ķ�Χ��0��ʾ�����޷�������=180��ʾ���޷���-180��180
    float K1;
    float K2;
    float max_out;
    float RC;
    float *set_ptr;
} FeedForward_Init_Config_s;

/**
 * @brief ��ʼ��ǰ��ʵ��
 * @param config  	ǰ������ָ��
 * @return float  	ǰ��ʵ��ָ��
 */
FeedForwardInstance *FeedForwardInit(FeedForward_Init_Config_s *config);

void FeedForwardSetRef(FeedForwardInstance *FF, float *ref);

void FeedForwardControl(void);

#endif
