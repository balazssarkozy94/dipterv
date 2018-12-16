#include "gpio.h"
#include "init.h"
#include "stm32f30x.h"
#include "stm32f30x_conf.h"
#include "time.h"
#include "story.txt"
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_usart.h"

void initUART()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    USART_InitTypeDef USART_InitStruct;
    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate = 460800;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = (USART_Mode_Tx | USART_Mode_Rx);
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;  
    USART_Init(USART3, &USART_InitStruct);
    
    USART_Cmd(USART3, ENABLE);
}

void sendDBG(char* msg, uint8_t size)
{
    int i;
    for(i = 0; i<size; i++)
    {
        USART_SendData(USART3, msg[i]);
        while(USART_GetFlagStatus(USART3, USART_FLAG_TC) != SET);
        USART_ClearFlag(USART3, USART_FLAG_TC);
    }
}
