#ifndef SPI_H
#define SPI_H

void initSPI();
uint32_t readSPI1(uint16_t reg_address,  uint8_t no_of_bytes);
void writeSPI1(uint16_t reg_address, uint32_t data, uint8_t no_of_bytes);

#endif
