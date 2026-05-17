#include "feed_forward.h"
#include "math.h"
#include "user_lib.h"

uint8_t ff_idx;
static FeedForwardInstance *feed_forward_instance[FF_CNT];

FeedForwardInstance *FeedForwardInit(FeedForward_Init_Config_s *config) {
    FeedForwardInstance *instance = (FeedForwardInstance *)malloc(sizeof(FeedForwardInstance));
    memset(instance, 0, sizeof(FeedForwardInstance));
    memcpy(instance, config, sizeof(FeedForward_Init_Config_s));
    instance->set_ptr = config->set_ptr;
    DWT_GetDeltaT(&instance->DWT_CNT);
    feed_forward_instance[ff_idx++] = instance;
    return instance;
}

static void FeedForwardCacl(FeedForwardInstance *FF) {
    if (FF->dt <= 0.0001f) {
        FF->dt = DWT_GetDeltaT(&FF->DWT_CNT);
        return; // 防止除以0
    }

    // 1. 计算真实的物理微分（目标角速度），必须除以 dt
    float current_target = *(FF->set_ptr);
    float delta_value    = current_target - FF->previous_set;
    if (FF->format_value != 0) {
        delta_value = loop_fp32_constrain(delta_value, -FF->format_value, FF->format_value);
    }
    float raw_velocity = delta_value / FF->dt;

    // 2. 对微分后的"速度"进行轻度低通滤波，而不是对"位置"滤波
    // 这样位置信号不会滞后，只是平滑了前馈的爆发力
    FF->temp_out =
        raw_velocity * FF->RC / (FF->RC + FF->dt) + FF->last_out * FF->dt / (FF->RC + FF->dt);

    // 3. 乘以速度前馈系数 K1 (注意：修改代码后你的K1值需要重新调小，因为它现在是真实的物理增益了)
    FF->final_out = FF->K1 * FF->temp_out;

    // 注：去掉了 K2 (加速度项)

    // 限幅...
    if (FF->final_out > FF->max_out)
        FF->final_out = FF->max_out;
    else if (FF->final_out < -(FF->max_out))
        FF->final_out = -(FF->max_out);

    // 更新历史值
    FF->previous_set = current_target; // 注意这里存的是未滤波的原值
    FF->last_out     = FF->temp_out;
    FF->dt           = DWT_GetDeltaT(&FF->DWT_CNT);
}

void FeedForwardSetRef(FeedForwardInstance *FF, float *ref) { FF->set_ptr = ref; }

void FeedForwardControl(void) {
    for (size_t i = 0; i < ff_idx; ++i) {
        FeedForwardCacl(feed_forward_instance[i]);
    }
}
