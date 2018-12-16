#include "dma.h"
#include "adc.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_dma.h"
#include "stm32f30x_adc.h"
#include "stm32f30x.h"

volatile uint16_t ADC_Values[4];

void initDMA()
{
    DMA_InitTypeDef  DMA_InitStruct;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStruct.DMA_BufferSize = 4;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_ADDRESS;
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)ADC_Values;
    
    DMA_Init(DMA1_Channel1, &DMA_InitStruct);
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

