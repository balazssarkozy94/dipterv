#include "nvic.h"
#include "stm32f30x_misc.h"
#include "stm32f30x_exti.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_syscfg.h"
#include "gpio.h"
#include "cr_base.h"
#include "init.h"
#include "stm32f30x.h"
#include "meas.h"
#include "spi.h"

volatile uint8_t metering_flt_cnt;
extern struct regs_of_interest_type regs_of_interest_L1;
extern struct regs_of_interest_type regs_of_interest_L2;
extern struct regs_of_interest_type regs_of_interest_L3;
volatile uint64_t l1_line_cyc_cntr = 0, l2_line_cyc_cntr = 0, l3_line_cyc_cntr = 0;

volatile uint8_t meas_update_timeout_cnt_L1 = 0;
volatile uint8_t meas_update_timeout_cnt_L2 = 0;
volatile uint8_t meas_update_timeout_cnt_L3 = 0;

void enableIRQn1()
{
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = 40; //EXTI15_10_IRQn
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
    NVIC_Init(&NVIC_InitStruct);
}

void disableIRQn1()
{
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = 40; //EXTI15_10_IRQn
    NVIC_InitStruct.NVIC_IRQChannelCmd = DISABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
    NVIC_Init(&NVIC_InitStruct);
}

void initNVIC()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //2 bits pre-emption, 2 bits subpriority
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    EXTI_InitTypeDef   EXTI_InitStructure;
    
    SYSCFG_EXTILineConfig(nIRQ1_EXTIPORT, nIRQ1_EXTI_PinSource);
    
    EXTI_InitStructure.EXTI_Line = nIRQ1_EXTI_Line;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    enableIRQn1();
}

void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(nIRQ1_EXTI_Line) != RESET)
    {
        uint32_t status1_buffer;
        status1_buffer = readSPI1(STATUS1, 4);
        
        if(status1_buffer&(1<<9))
        {
            ledToggle(LED_L1);
            l1_line_cyc_cntr++;
            resetTimeout(&regs_of_interest_L1);
            updateRegsOfInterest(regs_of_interest_L1);
//            writeSPI1(STATUS1,0x00000000|(1<<9), 4);
        }
        
        if(status1_buffer&(1<<10))
        {
            ledToggle(LED_L2);
            l2_line_cyc_cntr++;
            resetTimeout(&regs_of_interest_L2);
            updateRegsOfInterest(regs_of_interest_L2);
//            writeSPI1(STATUS1,(1<<10), 4);
        }
        if(status1_buffer&(1<<11))
        {
            ledToggle(LED_L3);
            l3_line_cyc_cntr++;
            resetTimeout(&regs_of_interest_L3);
            updateRegsOfInterest(regs_of_interest_L3);
//            writeSPI1(STATUS1,(1<<11), 4);
        }
        
        if(status1_buffer&(1<<15))
        {
            ledOnSys(LED_SYS_FLT);
            metering_flt_cnt = 3;
            initMeteringIC();
        }
    }
    writeSPI1(STATUS1,0xFFFFFFFF, 4); //temp
    EXTI_ClearITPendingBit(nIRQ1_EXTI_Line);
    return;
}

