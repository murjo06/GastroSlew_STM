#include "step_dir.h"
#include "config.h"
#include "tim.h"
#include "servo.h"

static uint32_t last_step_count = 0;

void StepDir_Init(void)
{
    //HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);

    /*
    ?MODIFY_REG(TIM2->SMCR,
    ?           TIM_SMCR_SMS_Msk | TIM_SMCR_TS_Msk,
    ?           (TIM_SLAVEMODE_EXTERNAL1 << TIM_SMCR_SMS_Pos) |
    ?           (TIM_TS_TI2FP2 << TIM_SMCR_TS_Pos));
    */

    //TIM2->CNT = 0;
    last_step_count = 0;
}

void StepDir_Loop(void)
{
    uint32_t current_count = 0;
    int32_t delta;

    //current_count = TIM2->CNT;

    delta = (int32_t)(current_count - last_step_count);

    last_step_count = current_count;

    if(delta == 0) {
        return;
    }
    if(HAL_GPIO_ReadPin(DIR_PORT, DIR_PIN)) {
        position_target += delta;
    } else {
        position_target -= delta;
    }
}