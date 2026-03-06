#include "bsp_dwt.h"
#include "bsp_pwm.h"
#include "buzzer.h"
#include "stdlib.h"
#include "string.h"


PWMInstance *buzzer = NULL;
// static uint8_t idx;
static BuzzzerInstance *buzzer_list[BUZZER_DEVICE_CNT] = {0};

/**
 * @brief ·äÃùÆ÷³õÊ¼»¯
 * @note  A°å·äÃùÆ÷£¬ÐèÒªpwmÇý¶¯£¬¶î¶¨ÆµÂÊÎª2700Hz,TIM12_CH1(PH6)
 *
 */
void BuzzerInit() {
    PWM_Init_Config_s buzzer_config = {
        .htim      = &htim12,
        .channel   = TIM_CHANNEL_1,
        .dutyratio = 0.5,
        .period    = 1,
    };

    buzzer = PWMRegister(&buzzer_config);
    PWMStart(buzzer);
}

/**
 * @brief ×¢²áÒ»¸ö·äÃùÆ÷ÊµÀý
 *
 * @param config ³õÊ¼»¯ÅäÖÃ
 * @return BuzzzerInstance*
 */
BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config) {

    if (config->alarm_level >
        BUZZER_DEVICE_CNT) // è¶…è¿‡æœ€å¤§å®žä¾‹æ•°,è€ƒè™‘å¢žåŠ æˆ–æŸ¥çœ‹æ˜¯å¦æœ‰å†…å­˜æ³„æ¼
        while (1)
            ;
    BuzzzerInstance *buzzer_temp = (BuzzzerInstance *)malloc(sizeof(BuzzzerInstance));
    memset(buzzer_temp, 0, sizeof(BuzzzerInstance));

    buzzer_temp->alarm_level = config->alarm_level;
    buzzer_temp->loudness    = config->loudness;
    buzzer_temp->octave      = config->octave;
    buzzer_temp->alarm_state = ALARM_OFF;

    buzzer_list[config->alarm_level] = buzzer_temp;
    return buzzer_temp;
}
void AlarmSetOctave(BuzzzerInstance *buzzer, octave_e octave) { buzzer->octave = octave; }

void AlarmSetStatus(BuzzzerInstance *buzzer, AlarmState_e state) { buzzer->alarm_state = state; }

/**
 * @brief ·äÃùÆ÷ÈÎÎñ
 *
 */
void BuzzerTask() {
    if (buzzer == NULL)
    {
        return;
    }
    
    BuzzzerInstance *buzz;
    for (size_t i = 0; i < BUZZER_DEVICE_CNT; ++i) {
        // PWMStop(buzzer);
        buzz = buzzer_list[i];
        if (buzz->alarm_level > ALARM_LEVEL_LOW) {
            continue;
        }
        if (buzz->alarm_state == ALARM_OFF) {
            PWMSetDutyRatio(buzzer, 0);
        }
        // else if (buzz->alarm_state == ALARM_ON)
        // {
        //     PWMSetDutyRatio(buzzer, 0.5);
        // }
        
        else {
            PWMSetDutyRatio(buzzer, buzz->loudness);
            switch (buzz->octave) {
            case OCTAVE_1:
                PWMSetPeriod(buzzer, (float)1000 / DoFreq);
                break;
            case OCTAVE_2:
                PWMSetPeriod(buzzer, (float)1000 / ReFreq);
                break;
            case OCTAVE_3:
                PWMSetPeriod(buzzer, (float)1000 / MiFreq);
                break;
            case OCTAVE_4:
                PWMSetPeriod(buzzer, (float)1000 / FaFreq);
                break;
            case OCTAVE_5:
                PWMSetPeriod(buzzer, (float)1000 / SoFreq);
                break;
            case OCTAVE_6:
                PWMSetPeriod(buzzer, (float)1000 / LaFreq);
                break;
            case OCTAVE_7:
                PWMSetPeriod(buzzer, (float)1000 / SiFreq);
                break;
            }
            break;
        }
        // PWMStart(buzzer);
    }
}
