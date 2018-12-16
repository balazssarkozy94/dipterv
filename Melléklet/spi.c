#include "spi.h"
#include "gpio.h"
#include "stm32f30x.h"
#include "stm32f30x_conf.h"
#include "stm32f30x_spi.h"
#include "stm32f30x_gpio.h"
#include "time.h"
#include "timer.h"
#include "cr_base.h"
#include "init.h"
#include "meas.h"

uint8_t spi_data_buffer[7] = {0,0,0,0,0,0,0};
uint8_t data_to_send = 0;
uint8_t data_to_read = 0;

enum spi_mode
{
    read = 0,
    write = 1,
} spi_mode;

void initSPI()
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    
  SPI_InitTypeDef SPI_InitStruct;
  SPI_StructInit(&SPI_InitStruct);
  
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  SPI_Init(SPI1, &SPI_InitStruct);
  
  SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE | SPI_I2S_IT_RXNE, ENABLE);
  
  SPI_Cmd(SPI1, ENABLE);
}

void slaveSelectSPI1(BitAction state)
{
    GPIO_WriteBit(SPI1_NSS_PORT, SPI1_NSS_PIN, state);
    return;
}

    
uint32_t readSPI1(uint16_t reg_address,  uint8_t no_of_bytes)
{
    int i;
    uint8_t addr_sliced[2];
    uint32_t input_buffer;
    input_buffer = 0;
    addr_sliced[1] = (reg_address&0xFF00)>>8;
    addr_sliced[0] = reg_address&0x00FF;
    
    slaveSelectSPI1(Bit_RESET);
    delayus(1);
    SPI_SendData8(SPI1, 1);
//    while(SPI_GetTransmissionFIFOStatus(SPI1));
    SPI_SendData8(SPI1, addr_sliced[1] );
//    while(SPI_GetTransmissionFIFOStatus(SPI1));
    SPI_SendData8(SPI1, addr_sliced[0] );
    while(SPI_GetTransmissionFIFOStatus(SPI1));
    
    while(SPI_GetReceptionFIFOStatus(SPI1))
    {
        SPI_ReceiveData8(SPI1);
    }
    
    for(i = 0; i<no_of_bytes; i++)
    {
        input_buffer = input_buffer<<8;
        SPI_SendData8(SPI1, 0);
        while(SPI_GetTransmissionFIFOStatus(SPI1));
        while(!SPI_GetReceptionFIFOStatus(SPI1));
        input_buffer |= SPI_ReceiveData8(SPI1);
    }

    delayus(1);
    slaveSelectSPI1(Bit_SET);
    return input_buffer;
}

void writeSPI1(uint16_t reg_address, uint32_t data, uint8_t no_of_bytes)
{
    int i;
    uint8_t addr_sliced[2];
    uint8_t data_sliced[4];
    addr_sliced[1] = (reg_address&0xFF00)>>8;
    addr_sliced[0] = reg_address&0x00FF;
    
    data_sliced[3] = (data>>24)&0xFF;
    data_sliced[2] = (data>>16)&0xFF;
    data_sliced[1] = (data>>8)&0xFF;
    data_sliced[0] = data&0xFF;
    
    slaveSelectSPI1(Bit_RESET);
    delayus(1);
    SPI_SendData8(SPI1, 0);
    while(SPI_GetTransmissionFIFOStatus(SPI1));
    SPI_SendData8(SPI1, addr_sliced[1] );
    while(SPI_GetTransmissionFIFOStatus(SPI1));
    SPI_SendData8(SPI1, addr_sliced[0] );
    while(SPI_GetTransmissionFIFOStatus(SPI1));
    for(i = 0; i<no_of_bytes; i++)
    {
        SPI_SendData8(SPI1, data_sliced[3-i]);
        while(SPI_GetTransmissionFIFOStatus(SPI1));
    }
    delayus(1);
    slaveSelectSPI1(Bit_SET);
    return;
}
