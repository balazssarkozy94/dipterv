#ifndef NVIC_H
#define NVIC_H

void initNVIC(void);
void enableIRQn1(void);
void disableIRQn1(void);

volatile uint8_t metering_flt_cnt;

#endif
