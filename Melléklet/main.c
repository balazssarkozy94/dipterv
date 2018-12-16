#include "stm32f30x.h"
#include "stm32f30x_conf.h"
#include <stdio.h>
#include "gpio.h"
#include "init.h"
#include "cr_base.h"
#include "meas.h"
#include "spi.h"
#include "timer.h"
#include "uart.h"
#include "dma.h"
#include <string.h>
#include "nvic.h"
#include "led.h"


#define MAX_NO_OF_REGS_TO_READ 50

volatile struct regs_of_interest_type regs_of_interest_L1;
volatile struct regs_of_interest_type regs_of_interest_L2;
volatile struct regs_of_interest_type regs_of_interest_L3;
volatile struct regs_of_interest_type regs_of_interest_N;

extern volatile uint16_t ADC_Values[4];

extern volatile uint8_t metering_flt_cnt;

extern volatile run_or_calib run_or_calib_state;

void tickHandler1000()
{
    if(metering_flt_cnt > 0)
    {
        metering_flt_cnt--;
    }else
    {
        ledOffSys(LED_SYS_FLT);
    }
}

void tickHandler500()
{
//       float float_buffer = 0;
//       float ADC_ref_cal = 0;
//       ADC_ref_cal = 1.23/ADC_Values[3];
//       float_buffer = ADC_Values[0]*V12_MEAS_CONST*ADC_ref_cal;
//       float_buffer = ADC_Values[1]*V3V3_MEAS_CONST*ADC_ref_cal;
//       float_buffer = (1690-ADC_Values[2])*ADC_ref_cal/4.3+25.0;
    
    float output_buffer[MAX_NO_OF_REGS_TO_READ];
    
    output_buffer[0] = readRegsOfInterest(regs_of_interest_L1, "L1 TEMP");
    dataSet(400, DATA_TYPE_MEASURE, (uint8_t *)&output_buffer[0], 0, timeGetTS());

    output_buffer[1] = readRegsOfInterest(regs_of_interest_L1, "L1 VRMS"); 
    dataSet(401, DATA_TYPE_MEASURE, (uint8_t *)&output_buffer[1], 0, timeGetTS());

    output_buffer[2] = readRegsOfInterest(regs_of_interest_L1, "L1 IRMS");
    dataSet(402, DATA_TYPE_MEASURE, (uint8_t *)&output_buffer[2], 0, timeGetTS());

    output_buffer[3] = readRegsOfInterest(regs_of_interest_L1, "L1 POWER");
    dataSet(403, DATA_TYPE_MEASURE, (uint8_t *)&output_buffer[3], 0, timeGetTS());

    output_buffer[4] = readRegsOfInterest(regs_of_interest_L1, "L1 ENERGY");
    dataSet(404, DATA_TYPE_MEASURE, (uint8_t *)&output_buffer[4], 0, timeGetTS());
    
    output_buffer[5] = readRegsOfInterest(regs_of_interest_L2, "L2 TEMP");
    dataSet(405, DATA_TYPE_MEASURE, (uint8_t *)&output_buffer[5], 0, timeGetTS());

    output_buffer[6] = readRegsOfInterest(regs_of_interest_L2, "L2 VRMS"); 
    dataSet(406, DATA_TYPE_MEASURE, (uint8_t *)&output_buffer[6], 0, timeGetTS());

    output_buffer[7] = readRegsOfInterest(regs_of_interest_L2, "L2 IRMS");
    dataSet(407, DATA_TYPE_MEASURE, (uint8_t *)&output_buffer[7], 0, timeGetTS());

    output_buffer[8] = readRegsOfInterest(regs_of_interest_L2, "L2 POWER");
    dataSet(408, DATA_TYPE_MEASURE, (uint8_t *)&output_buffer[8], 0, timeGetTS());

    output_buffer[9] = readRegsOfInterest(regs_of_interest_L2, "L2 ENERGY");
    dataSet(409, DATA_TYPE_MEASURE, (uint8_t *)&output_buffer[9], 0, timeGetTS());
    
    output_buffer[10] = readRegsOfInterest(regs_of_interest_L3, "L3 TEMP");
    dataSet(410, DATA_TYPE_MEASURE, (uint8_t *)&output_buffer[10], 0, timeGetTS());

    output_buffer[11] = readRegsOfInterest(regs_of_interest_L3, "L3 VRMS"); 
    dataSet(411, DATA_TYPE_MEASURE, (uint8_t *)&output_buffer[11], 0, timeGetTS());

    output_buffer[12] = readRegsOfInterest(regs_of_interest_L3, "L3 IRMS");
    dataSet(412, DATA_TYPE_MEASURE, (uint8_t *)&output_buffer[12], 0, timeGetTS());

    output_buffer[13] = readRegsOfInterest(regs_of_interest_L3, "L3 POWER");
    dataSet(413, DATA_TYPE_MEASURE, (uint8_t *)&output_buffer[13], 0, timeGetTS());

    output_buffer[14] = readRegsOfInterest(regs_of_interest_L3, "L3 ENERGY");
    dataSet(414, DATA_TYPE_MEASURE, (uint8_t *)&output_buffer[14], 0, timeGetTS());
    
    output_buffer[15] = readRegsOfInterest(regs_of_interest_N, "N TEMP");
    dataSet(415, DATA_TYPE_MEASURE, (uint8_t *)&output_buffer[15], 0, timeGetTS());

    output_buffer[16] = readRegsOfInterest(regs_of_interest_N, "N VRMS"); 
    dataSet(416, DATA_TYPE_MEASURE, (uint8_t *)&output_buffer[16], 0, timeGetTS());

    output_buffer[17] = readRegsOfInterest(regs_of_interest_N, "N IRMS");
    dataSet(417, DATA_TYPE_MEASURE, (uint8_t *)&output_buffer[17], 0, timeGetTS());
}

void tickHandler100()
{
    updateRegsOfInterest(regs_of_interest_N);
    checkAndUpdateOnTimeout(&regs_of_interest_L1);
    checkAndUpdateOnTimeout(&regs_of_interest_L2);
    checkAndUpdateOnTimeout(&regs_of_interest_L3);
}

int main ( void )
{
    
    uint32_t linecyc_accumulation_time = 50;
    
    struct meas_register reg_list_L1[MAX_NO_OF_REGS_TO_READ/3];
  struct calib_data calibration_data_l1;
  regs_of_interest_L1.calibration_data = &calibration_data_l1;
  regs_of_interest_L1.no_of_regs = 0;
  regs_of_interest_L1.reg_array = reg_list_L1;
  regs_of_interest_L1.meas_update_timeout_cnt = 0;

  struct meas_register reg_list_L2[MAX_NO_OF_REGS_TO_READ/3];
  struct calib_data calibration_data_l2;
  regs_of_interest_L2.calibration_data = &calibration_data_l2;
  regs_of_interest_L2.no_of_regs = 0;
  regs_of_interest_L2.reg_array = reg_list_L2;
  regs_of_interest_L2.meas_update_timeout_cnt = 0;
  
  struct meas_register reg_list_L3[MAX_NO_OF_REGS_TO_READ/3];
  struct calib_data calibration_data_l3;
  regs_of_interest_L3.calibration_data = &calibration_data_l3;
  regs_of_interest_L3.no_of_regs = 0;
  regs_of_interest_L3.reg_array = reg_list_L3;
  regs_of_interest_L3.meas_update_timeout_cnt = 0;
  
  struct meas_register reg_list_N[MAX_NO_OF_REGS_TO_READ/3];
  struct calib_data calibration_data_n;
  regs_of_interest_N.calibration_data = &calibration_data_n;
  regs_of_interest_N.no_of_regs = 0;
  regs_of_interest_N.reg_array = reg_list_N;
  regs_of_interest_N.meas_update_timeout_cnt = 0;
  
    loadCalibData();

    //calib select init
    initCalibSelectPin();
    
    if(GPIO_ReadInputDataBit(A3_PORT, A3_PIN)==Bit_RESET)
    {
        run_or_calib_state = CALIB;
        linecyc_accumulation_time = CALIBRATION_AVERAGING_CYCLES;
    }
    else
    {
        run_or_calib_state = RUN;
        
    }

    addToRegsOfInterest(&regs_of_interest_L1, "L1 VRMS", AVRMS, 4, _signed, voltage, linecyc_accumulation_time);
    addToRegsOfInterest(&regs_of_interest_L1, "L1 IRMS", AIRMS, 4, _signed, current, linecyc_accumulation_time);
    addToRegsOfInterest(&regs_of_interest_L1, "L1 FVRMS", AFVRMS, 4, _signed, voltage, linecyc_accumulation_time);
    addToRegsOfInterest(&regs_of_interest_L1, "L1 FIRMS", AFIRMS, 4, _signed, current, linecyc_accumulation_time);
    addToRegsOfInterest(&regs_of_interest_L1, "L1 POWER", AWATT, 4, _signed, power, linecyc_accumulation_time);
    addToRegsOfInterest(&regs_of_interest_L1, "L1 ENERGY", AWATTHR, 4, _signed, energy, 1);
    addToRegsOfInterest(&regs_of_interest_L1, "L1 REACTIVE ENERGY", AVARHR, 4, _signed, energy, 1);
    addToRegsOfInterest(&regs_of_interest_L1, "L1 TEMP", ATEMP, 4, _signed, temperature, 10);

    addToRegsOfInterest(&regs_of_interest_L2, "L2 VRMS", BVRMS, 4, _signed, voltage, linecyc_accumulation_time);
    addToRegsOfInterest(&regs_of_interest_L2, "L2 IRMS", BIRMS, 4, _signed, current, linecyc_accumulation_time);
    addToRegsOfInterest(&regs_of_interest_L2, "L2 FVRMS", BFVRMS, 4, _signed, voltage, linecyc_accumulation_time);
    addToRegsOfInterest(&regs_of_interest_L2, "L2 FIRMS", BFIRMS, 4, _signed, current, linecyc_accumulation_time);
    addToRegsOfInterest(&regs_of_interest_L2, "L2 POWER", BWATT, 4, _signed, power, linecyc_accumulation_time);
    addToRegsOfInterest(&regs_of_interest_L2, "L2 ENERGY", BWATTHR, 4, _signed, energy, 1);
    addToRegsOfInterest(&regs_of_interest_L2, "L2 REACTIVE ENERGY", BVARHR, 4, _signed, energy, 1);
    addToRegsOfInterest(&regs_of_interest_L2, "L2 TEMP", BTEMP, 4, _signed, temperature, 10);

    addToRegsOfInterest(&regs_of_interest_L3, "L3 VRMS", CVRMS, 4, _signed, voltage, linecyc_accumulation_time);
    addToRegsOfInterest(&regs_of_interest_L3, "L3 IRMS", CIRMS, 4, _signed, current, linecyc_accumulation_time);
    addToRegsOfInterest(&regs_of_interest_L3, "L3 FVRMS", CFVRMS, 4, _signed, voltage, linecyc_accumulation_time);
    addToRegsOfInterest(&regs_of_interest_L3, "L3 FIRMS", CFIRMS, 4, _signed, current, linecyc_accumulation_time);
    addToRegsOfInterest(&regs_of_interest_L3, "L3 POWER", CWATT, 4, _signed, power, linecyc_accumulation_time);
    addToRegsOfInterest(&regs_of_interest_L3, "L3 ENERGY", CWATTHR, 4, _signed, energy, 1);
    addToRegsOfInterest(&regs_of_interest_L3, "L3 REACTIVE ENERGY",CVARHR, 4, _signed, energy, 1);
    addToRegsOfInterest(&regs_of_interest_L3, "L3 TEMP", CTEMP, 4, _signed, temperature, 10);

    addToRegsOfInterest(&regs_of_interest_N, "N VRMS", NVRMS, 4, _signed, voltage, 10);
    addToRegsOfInterest(&regs_of_interest_N, "N IRMS", NIRMS, 4, _signed, current, 10);
    addToRegsOfInterest(&regs_of_interest_N, "N TEMP", NTEMP, 4, _signed, temperature, 10);


    init();
  
  tickDsInit(1000, tickHandler1000);
  tickDsInit(500, tickHandler500);
  tickDsInit(100, tickHandler100);
  
  while ( 1 )
  {
    crBaseHandler();
    }

  return 0;
}
