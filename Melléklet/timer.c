#include "timer.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_rcc.h"

void initTimer17()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);
        
    TIM_TimeBaseInitTypeDef TIM_InitStruct;
    TIM_TimeBaseStructInit(&TIM_InitStruct);
    TIM_InitStruct.TIM_Prescaler = 72;
    TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStruct.TIM_Period = 65535;
    TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
    TIM_InitStruct.TIM_RepetitionCounter = 0;
    
    TIM_TimeBaseInit(TIM17, &TIM_InitStruct);
    
    TIM_Cmd(TIM17, ENABLE);
}

void delayus(uint32_t delay)
{
    uint32_t delay_buffer = delay;
    uint32_t previous_time;
    uint32_t current_time;
    uint32_t start_time = previous_time = TIM_GetCounter(TIM17);
    while(1)
    {
        current_time = TIM_GetCounter(TIM17);
        if(previous_time > current_time)
        {
            delay_buffer -= (65535 - start_time);
            start_time = 0;
        }
        if(current_time - start_time > delay_buffer)
        {
            break;
        }
        previous_time = current_time;
    }
    return;
}
