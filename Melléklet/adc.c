#include "adc.h"
#include "stm32f30x_adc.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x.h"
#include "timer.h"

void initADC()
{
    ADC_InitTypeDef ADC_InitStruct;
    ADC_CommonInitTypeDef ADC_CommonInitStruct;
    uint32_t calibration_value;
    
    RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div128);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12,ENABLE);
    
    ADC_StructInit(&ADC_InitStruct);
    ADC_VoltageRegulatorCmd(ADC1, ENABLE);
    delayus(10);
    ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1) != RESET );
    calibration_value = ADC_GetCalibrationValue(ADC1);
    
    ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;                                                                    
    ADC_CommonInitStruct.ADC_Clock = ADC_Clock_AsynClkMode;                    
    ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_1;             
    ADC_CommonInitStruct.ADC_DMAMode = ADC_DMAMode_Circular;                  
    ADC_CommonInitStruct.ADC_TwoSamplingDelay = 0xF;          
  
  ADC_CommonInit(ADC1, &ADC_CommonInitStruct);
  
    ADC_InitStruct.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b; 
    ADC_InitStruct.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
    ADC_InitStruct.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_OverrunMode = ADC_OverrunMode_Disable;   
    ADC_InitStruct.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
    ADC_InitStruct.ADC_NbrOfRegChannel = 4;
    ADC_Init(ADC1, &ADC_InitStruct);
    
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_601Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_Vbat, 2, ADC_SampleTime_601Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 3, ADC_SampleTime_601Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 4, ADC_SampleTime_601Cycles5);
    ADC_TempSensorCmd(ADC1, ENABLE);
    ADC_VbatCmd(ADC1, ENABLE);
    ADC_VrefintCmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_DMAConfig(ADC1, ADC_DMAMode_Circular);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
    ADC_StartConversion(ADC1);
}
