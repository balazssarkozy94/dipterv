#include "gpio.h"
#include "init.h"
#include "stm32f30x.h"
#include "stm32f30x_conf.h"
#include "time.h"
#include "story.txt"
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"

void initGpios()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit ( &GPIO_InitStruct );
    
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Pin  = nIRQ0_PIN;
    GPIO_Init ( nIRQ0_PORT, &GPIO_InitStruct );
    
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Pin  = nIRQ1_PIN;
    GPIO_Init ( nIRQ1_PORT, &GPIO_InitStruct );
    
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Pin  = nRESET_METER_PIN;
    GPIO_Init ( nRESET_METER_PORT, &GPIO_InitStruct );
    
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Pin  = nDREADY_PIN;
    GPIO_Init ( nDREADY_PORT, &GPIO_InitStruct );
        
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Pin  = PPS_PIN;
    GPIO_Init ( PPS_PORT, &GPIO_InitStruct );
    
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Pin  = SPI1_NSS_PIN;
    GPIO_Init ( SPI1_NSS_PORT, &GPIO_InitStruct );
    GPIO_WriteBit(SPI1_NSS_PORT, SPI1_NSS_PIN, Bit_SET); //Setting SPI1_NSS High at init
    
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Pin  = SPI1_MISO_PIN;
    GPIO_Init ( SPI1_MISO_PORT, &GPIO_InitStruct );
    GPIO_PinAFConfig(SPI1_MISO_PORT, SPI1_MISO_PIN_SOURCE, GPIO_AF_5);
    
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Pin  = SPI1_MOSI_PIN;
    GPIO_Init ( SPI1_MOSI_PORT, &GPIO_InitStruct );
    GPIO_PinAFConfig(SPI1_MOSI_PORT, SPI1_MOSI_PIN_SOURCE, GPIO_AF_5);
    
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Pin  = SPI1_SCK_PIN;
    GPIO_Init ( SPI1_SCK_PORT, &GPIO_InitStruct );
    GPIO_PinAFConfig(SPI1_SCK_PORT, SPI1_SCK_PIN_SOURCE, GPIO_AF_5);
    
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Pin  = DBG_UART_TX_PIN;
    GPIO_Init ( DBG_UART_TX_PORT, &GPIO_InitStruct );
    GPIO_PinAFConfig(DBG_UART_TX_PORT, DBG_UART_TX_PIN_SOURCE, GPIO_AF_7);
    
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Pin  = DBG_UART_RX_PIN;
    GPIO_Init ( DBG_UART_RX_PORT, &GPIO_InitStruct );
    GPIO_PinAFConfig(DBG_UART_RX_PORT, DBG_UART_RX_PIN_SOURCE, GPIO_AF_7);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Pin  = _12V_READOUT_PIN;
    GPIO_Init ( _12V_READOUT_PORT, &GPIO_InitStruct );
}

 void initCalibSelectPin()
 {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit ( &GPIO_InitStruct );
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Pin  = A3_PIN;
    GPIO_Init ( A3_PORT, &GPIO_InitStruct );
 }

//l__attribute__(( __section__( ".app_data" ))) const uint16_t a16Version[2] = { __VERSION_HIGH__, __VERSION_LOW__ };

