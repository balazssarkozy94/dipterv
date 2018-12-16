#include "meas.h"
#include "gpio.h"
#include "init.h"
#include "stm32f30x.h"
#include "stm32f30x_conf.h"
#include "time.h"
#include "spi.h"
#include "timer.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>
#include "channel.h"
#include "led.h"
#include "nvic.h"
#include "cr_base.h"
#include "math.h"
#include "eeprom.h"
#include "meas.h"

#define pi 3.14159265359

#define NOMINAL_TEST_CURRENT 8
#define MINIMAL_TEST_CURRENT 3.2
#define NOMINAL_TEST_VOLTAGE 230
#define MINIMAL_TEST_VOLTAGE 23

extern volatile run_or_calib run_or_calib_state;

extern volatile struct regs_of_interest_type regs_of_interest_L1;
extern volatile struct regs_of_interest_type regs_of_interest_L2;
extern volatile struct regs_of_interest_type regs_of_interest_L3;
extern volatile struct regs_of_interest_type regs_of_interest_N;

extern volatile struct regs_of_interest_type regs_of_interest;
extern volatile struct regs_of_interest_type regs_of_interest_L1;
extern volatile struct regs_of_interest_type regs_of_interest_L2;
extern volatile struct regs_of_interest_type regs_of_interest_L3;
extern volatile struct regs_of_interest_type regs_of_interest_N;

extern volatile uint64_t l1_line_cyc_cntr;
extern volatile uint64_t l2_line_cyc_cntr;
extern volatile uint64_t l3_line_cyc_cntr;

volatile uint16_t VirtAddVarTab[NB_OF_VAR];

void eeStoreFloat(float data, uint16_t ee_address)
{
    uint16_t ee_reg_buffer;
    uint32_t uint32buffer;
    float floatbuffer = data;
    memcpy(&uint32buffer, &floatbuffer, sizeof(float));
    
    ee_reg_buffer = (uint16_t)(((uint32buffer)>>16)&0x0000FFFF);
    EE_WriteVariable(VirtAddVarTab[ee_address], ee_reg_buffer);
    ee_reg_buffer = (uint16_t)((uint32buffer)&0x0000FFFF);
    EE_WriteVariable(VirtAddVarTab[ee_address+1], ee_reg_buffer);
}

void eeReadFloat(volatile float* calib_address, uint16_t ee_address)
{
    uint16_t buffer1, buffer2;
    uint32_t float_buffer;
    
    EE_ReadVariable(VirtAddVarTab[ee_address], &buffer1);
    EE_ReadVariable(VirtAddVarTab[ee_address+1], &buffer2);
    float_buffer = (uint32_t)(buffer1 << 16);
    float_buffer |= (uint32_t)(buffer2&0x0000FFFF);
    memcpy((void*)calib_address, &float_buffer, sizeof(float));
}

void eeStoreUint32_t(uint32_t data, uint16_t ee_address)
{
    uint16_t ee_reg_buffer;

    ee_reg_buffer = (uint16_t)(((data)>>16)&0x0000FFFF);
    EE_WriteVariable(VirtAddVarTab[ee_address], ee_reg_buffer);
    ee_reg_buffer = (uint16_t)((data)&0x0000FFFF);
    EE_WriteVariable(VirtAddVarTab[ee_address+1], ee_reg_buffer);
}

void eeReadUint32_t(volatile uint32_t* calib_address, uint16_t ee_address)
{
    uint16_t buffer1, buffer2;
    uint32_t uint32_t_buffer;
    
    EE_ReadVariable(VirtAddVarTab[ee_address], &buffer1);
    EE_ReadVariable(VirtAddVarTab[ee_address+1], &buffer2);
    uint32_t_buffer = (uint32_t)(buffer1 << 16);
    uint32_t_buffer |= (uint32_t)(buffer2&0x0000FFFF);
    memcpy((void*)calib_address, &uint32_t_buffer, sizeof(uint32_t));
}

void eeStoreInt32_t(int32_t data, uint16_t ee_address)
{
    uint16_t ee_reg_buffer;

    ee_reg_buffer = (uint16_t)(((data)>>16)&0x0000FFFF);
    EE_WriteVariable(VirtAddVarTab[ee_address], ee_reg_buffer);
    ee_reg_buffer = (uint16_t)((data)&0x0000FFFF);
    EE_WriteVariable(VirtAddVarTab[ee_address+1], ee_reg_buffer);
}

void eeReadInt32_t(volatile int32_t* calib_address, uint16_t ee_address)
{
    uint16_t buffer1, buffer2;
    int32_t int32_t_buffer;
    
    EE_ReadVariable(VirtAddVarTab[ee_address], &buffer1);
    EE_ReadVariable(VirtAddVarTab[ee_address+1], &buffer2);
    int32_t_buffer = (uint32_t)(buffer1 << 16);
    int32_t_buffer |= (uint32_t)(buffer2&0x0000FFFF);
    memcpy((void*)calib_address, &int32_t_buffer, sizeof(int32_t));
}

void checkAndUpdateOnTimeout(volatile struct regs_of_interest_type* regs_of_interest)
{
        if(regs_of_interest->meas_update_timeout_cnt >= NO_UPDATE_TIMEOUT_VALUE)
    {
        updateRegsOfInterest(*regs_of_interest);
    }
    else
    {
        regs_of_interest->meas_update_timeout_cnt++;
    }
}

void resetTimeout(volatile struct regs_of_interest_type* regs_of_interest)
{
    regs_of_interest->meas_update_timeout_cnt = 0;
}

void loadCalibData()
{
    uint32_t i;    
    for(i = 0; i<NB_OF_VAR; i++)
    {
        VirtAddVarTab[i] = i;
    }
    
    FLASH_Unlock();
    EE_Init();
    
    enum calib_reg_addr addr2read;

    //Loading V/I/LSB values
    
    addr2read = V_LSB_H;
    eeReadFloat(&regs_of_interest_L1.calibration_data->v_lsb, addr2read);
    
    addr2read = I_LSB_H;
    eeReadFloat(&regs_of_interest_L1.calibration_data->i_lsb, addr2read);
    
    addr2read = V_LSB_H;
    eeReadFloat(&regs_of_interest_L2.calibration_data->v_lsb, addr2read+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2read = I_LSB_H;
    eeReadFloat(&regs_of_interest_L2.calibration_data->i_lsb, addr2read+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2read = V_LSB_H;
    eeReadFloat(&regs_of_interest_L3.calibration_data->v_lsb, addr2read+2*EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2read = I_LSB_H;
    eeReadFloat(&regs_of_interest_L3.calibration_data->i_lsb, addr2read+2*EEPROM_CALIB_SIZE_PER_PHASE);

    //Loading VGAIN/IGAIN values
    
    addr2read = VGAIN_H;
    eeReadUint32_t(&regs_of_interest_L1.calibration_data->vgain, addr2read);
    
    addr2read = IGAIN_H;
    eeReadUint32_t(&regs_of_interest_L1.calibration_data->igain, addr2read);
    
    addr2read = VGAIN_H;
    eeReadUint32_t(&regs_of_interest_L2.calibration_data->vgain, addr2read+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2read = IGAIN_H;
    eeReadUint32_t(&regs_of_interest_L2.calibration_data->igain, addr2read+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2read = VGAIN_H;
    eeReadUint32_t(&regs_of_interest_L3.calibration_data->vgain, addr2read+2*EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2read = IGAIN_H;
    eeReadUint32_t(&regs_of_interest_L3.calibration_data->igain, addr2read+2*EEPROM_CALIB_SIZE_PER_PHASE);
    
    //Loading offsets for fundamental, total voltages and currents
    
    addr2read = VOS_H;
    eeReadInt32_t(&regs_of_interest_L1.calibration_data->vos, addr2read);
    
    addr2read = IOS_H;
    eeReadInt32_t(&regs_of_interest_L1.calibration_data->ios, addr2read);
    
    addr2read = FVOS_H;
    eeReadInt32_t(&regs_of_interest_L1.calibration_data->fvos, addr2read);
    
    addr2read = FIOS_H;
    eeReadInt32_t(&regs_of_interest_L1.calibration_data->fios, addr2read);
    
    addr2read = VOS_H;
    eeReadInt32_t(&regs_of_interest_L2.calibration_data->vos, addr2read+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2read = IOS_H;
    eeReadInt32_t(&regs_of_interest_L2.calibration_data->ios, addr2read+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2read = FVOS_H;
    eeReadInt32_t(&regs_of_interest_L2.calibration_data->fvos, addr2read+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2read = FIOS_H;
    eeReadInt32_t(&regs_of_interest_L2.calibration_data->fios, addr2read+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2read = VOS_H;
    eeReadInt32_t(&regs_of_interest_L3.calibration_data->vos, addr2read+EEPROM_CALIB_SIZE_PER_PHASE*2);
    
    addr2read = IOS_H;
    eeReadInt32_t(&regs_of_interest_L3.calibration_data->ios, addr2read+EEPROM_CALIB_SIZE_PER_PHASE*2);
    
    addr2read = FVOS_H;
    eeReadInt32_t(&regs_of_interest_L3.calibration_data->fvos, addr2read+EEPROM_CALIB_SIZE_PER_PHASE*2);
    
    addr2read = FIOS_H;
    eeReadInt32_t(&regs_of_interest_L3.calibration_data->fios, addr2read+EEPROM_CALIB_SIZE_PER_PHASE*2);

    //Loading phase calibration data
    
    addr2read = PHCAL_H;
    eeReadInt32_t(&regs_of_interest_L1.calibration_data->phcal, addr2read);
    
    addr2read = PHCAL_H;
    eeReadInt32_t(&regs_of_interest_L2.calibration_data->phcal, addr2read+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2read = PHCAL_H;
    eeReadInt32_t(&regs_of_interest_L3.calibration_data->phcal, addr2read+EEPROM_CALIB_SIZE_PER_PHASE*2);
    
    //Loading W, WH lsb
    
    addr2read = WH_LSB_H;
    eeReadFloat(&regs_of_interest_L1.calibration_data->wh_lsb, addr2read);
    
    addr2read = W_LSB_H;
    eeReadFloat(&regs_of_interest_L1.calibration_data->w_lsb, addr2read);
    
    addr2read = WH_LSB_H;
    eeReadFloat(&regs_of_interest_L2.calibration_data->wh_lsb, addr2read+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2read = W_LSB_H;
    eeReadFloat(&regs_of_interest_L2.calibration_data->w_lsb, addr2read+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2read = WH_LSB_H;
    eeReadFloat(&regs_of_interest_L3.calibration_data->wh_lsb, addr2read+EEPROM_CALIB_SIZE_PER_PHASE*2);
    
    addr2read = W_LSB_H;
    eeReadFloat(&regs_of_interest_L3.calibration_data->w_lsb, addr2read+EEPROM_CALIB_SIZE_PER_PHASE*2);

    regs_of_interest_N.calibration_data->v_lsb = voltage_constant;
    regs_of_interest_N.calibration_data->i_lsb = current_constant;
}

void resetMeter()
{
    GPIO_WriteBit(nRESET_METER_PORT, nRESET_METER_PIN, Bit_RESET);
    delayus(10);
    GPIO_WriteBit(nRESET_METER_PORT, nRESET_METER_PIN, Bit_SET);
}

void initMeteringIC()
{
   disableIRQn1();
   
    int i = 0;
    resetMeter();
    //Waiting for the nIRQ1 to go low
    while(GPIO_ReadInputDataBit(nIRQ1_PORT, nIRQ1_PIN)!=Bit_RESET);
    //Pulsing the ~SS/HSA pin three times from high to low to select SPI interface
    for(i = 0; i<3; i++)
    {
    delayus(1);
    GPIO_WriteBit(SPI1_NSS_PORT, SPI1_NSS_PIN, Bit_RESET);
    delayus(1);
    GPIO_WriteBit(SPI1_NSS_PORT, SPI1_NSS_PIN, Bit_SET);
    }
    //Read STATUS1 register to verify that RSTDONE is set to 1, an then clearing all STATUS1 and STATUS0 registers by writing 1 to all bits
    while(!(readSPI1(STATUS1, 4)&(1<<15)));
    writeSPI1(STATUS0,0xFFFFFFFF, 4);
    writeSPI1(STATUS1,0xFFFFFFFF, 4);
    //Write any data to CONFIG2 to lock the port
    writeSPI1(CONFIG2,0x0BC5ABA0, 4);
    //Temperature sensors enabled, second voltage channels disabled
    writeSPI1(CONFIG3,0x00,1);
    //Setting VLEVEL Vfs = 354Vrms, Vnom = 230Vrms
    writeSPI1(VLEVEL,0x5DEA97,4);
    //Initialize AIGAIN, BIGAIN, CIGAIN, NIGAIN based on calibration levels

    writeSPI1(AIGAIN,regs_of_interest_L1.calibration_data->igain, 4);
    writeSPI1(BIGAIN,regs_of_interest_L2.calibration_data->igain, 4);
    writeSPI1(CIGAIN,regs_of_interest_L3.calibration_data->igain, 4);
//    writeSPI1(NIGAIN,0x00000000, 4);
    writeSPI1(AVGAIN,regs_of_interest_L1.calibration_data->vgain, 4);
    writeSPI1(BVGAIN,regs_of_interest_L2.calibration_data->vgain, 4);
    writeSPI1(CVGAIN,regs_of_interest_L3.calibration_data->vgain, 4);
    
    //Init offset registers
    
    writeSPI1(AIRMSOS, regs_of_interest_L1.calibration_data->ios, 4);
    writeSPI1(BIRMSOS, regs_of_interest_L2.calibration_data->ios, 4);
    writeSPI1(CIRMSOS, regs_of_interest_L3.calibration_data->ios, 4);
    
    writeSPI1(AFIRMSOS, regs_of_interest_L1.calibration_data->fios, 4);
    writeSPI1(BFIRMSOS, regs_of_interest_L2.calibration_data->fios, 4);
    writeSPI1(CFIRMSOS, regs_of_interest_L3.calibration_data->fios, 4);
    
    writeSPI1(AVRMSOS, regs_of_interest_L1.calibration_data->vos, 4);
    writeSPI1(BVRMSOS, regs_of_interest_L2.calibration_data->vos, 4);
    writeSPI1(CVRMSOS, regs_of_interest_L3.calibration_data->vos, 4);
    
    writeSPI1(AFVRMSOS, regs_of_interest_L1.calibration_data->fvos, 4);
    writeSPI1(BFVRMSOS, regs_of_interest_L2.calibration_data->fvos, 4);
    writeSPI1(CFVRMSOS, regs_of_interest_L3.calibration_data->fvos, 4);
    
    
//    writeSPI1(NVGAIN,0x00000000, 4);
    
    //Start the DSP by writing 0x0001 to Run register
    writeSPI1(Run,0x00010000, 2);
    //Initialize DSP RAM-based reigsters from 0x4380 to 0x43BF. Write the last register in the queue three times
    //TODO
    //Initialize the hardware-based configuration registers located from 0xE507 to 0xEA04
    //TODO
    //Enable the DSP RAM write protection by wrinting 0xAD to 0xE7FE, then write 0x80 to 0xE7E3
    //TODO
    //Read energy registers to erase their contents and start the accumulation from a known state.
    //TODO
    //Clear Bit 9, 10, 11 in CFMODE register (0xE610) to enable pulses
    uint16_t tmp16;
    tmp16 = readSPI1(CFMODE,2);
    tmp16 &= (0xFFFF^(0b111<<9));   
    writeSPI1(CFMODE,tmp16, 2);
    
    writeSPI1(MASK1,0x00000E00,4); //enables interrupts on zero crossing on L1, L2, L3 voltage channels
    
    //Enable read with reset on xWATTHR registers, set linecyc mode
//     uint8_t tmp8;
 //   writeSPI1(LCYCMODE,0b01001111, 1);
    //Set line cycle mode count
 //   writeSPI1(LINECYC, 0x000A, 2);

    enableIRQn1();
}

void updateRegsOfInterest(struct regs_of_interest_type regs_of_interest)
{
    uint8_t i;
    for(i = 0; i<regs_of_interest.no_of_regs; i++)
    {
        if(regs_of_interest.reg_array[i].dimension == energy)
        {
            regs_of_interest.reg_array[i].value += (int32_t)readSPI1(regs_of_interest.reg_array[i].reg_addr, regs_of_interest.reg_array[i].reg_size);
        }
        else
        {
            regs_of_interest.reg_array[i].avg_acc += (int32_t)readSPI1(regs_of_interest.reg_array[i].reg_addr, regs_of_interest.reg_array[i].reg_size);
            regs_of_interest.reg_array[i].avg_cnt++;
            if(regs_of_interest.reg_array[i].avg_cnt >= regs_of_interest.reg_array[i].avg_cycles)
            {
                regs_of_interest.reg_array[i].value = regs_of_interest.reg_array[i].avg_acc/regs_of_interest.reg_array[i].avg_cycles;
                regs_of_interest.reg_array[i].avg_acc = 0;
                regs_of_interest.reg_array[i].avg_cnt = 0;
            }
        }
    }
}


float readRegsOfInterest(struct regs_of_interest_type regs_of_interest, const char* tag)
{
    struct meas_register ret;
    float buffer = 0;
    memset(&ret, 0, sizeof(struct meas_register));
    
    for(uint32_t i = 0; i<regs_of_interest.no_of_regs; i++)
    {
        if( strcmp(tag,  regs_of_interest.reg_array[i].tag ) == 0 )
        {
            ret = regs_of_interest.reg_array[i];
            break;
        }
    }
    switch(ret.dimension)
    {
    case voltage:
        buffer = ret.value*regs_of_interest.calibration_data->v_lsb;
        break;
    case current:
        buffer = ret.value*regs_of_interest.calibration_data->i_lsb;
        break;
    case power:
        buffer = (int32_t)ret.value*regs_of_interest.calibration_data->w_lsb;
        break;
    case energy:
        buffer = ret.value*regs_of_interest.calibration_data->wh_lsb;
        break;
    case temperature:
        buffer = ret.value*temperature_constant_1;
        buffer -=temperature_constant_2;
        break;
    case scalar:
    default:
        break;
    }
    return buffer;
}

void addToRegsOfInterest(struct regs_of_interest_type* regs_of_interest, const char* tag, const uint16_t reg_addr, const uint8_t reg_size, const enum issigned type, const enum dimension dimension, const uint32_t avg_cycles)
{
    regs_of_interest->reg_array[regs_of_interest->no_of_regs].tag = tag;
    regs_of_interest->reg_array[regs_of_interest->no_of_regs].value = 0;
    regs_of_interest->reg_array[regs_of_interest->no_of_regs].reg_addr = reg_addr;
    regs_of_interest->reg_array[regs_of_interest->no_of_regs].reg_size = reg_size;
    regs_of_interest->reg_array[regs_of_interest->no_of_regs].type = type;
    regs_of_interest->reg_array[regs_of_interest->no_of_regs].dimension = dimension;
    regs_of_interest->reg_array[regs_of_interest->no_of_regs].avg_acc = 0;
    regs_of_interest->reg_array[regs_of_interest->no_of_regs].avg_cnt = 0;
    regs_of_interest->reg_array[regs_of_interest->no_of_regs].avg_cycles = avg_cycles;

    regs_of_interest->no_of_regs++;
}

void calibrateMeteringIC_GainCalib()
{
    
}

void calibrateMeteringIC_WattOSCalib()
{
    
}

void calibrateMeteringIC_VAROSCalib()
{
    
}

void calibrateMeteringIC_VIGainMatching()
{
    enum calib_reg_addr addr2write;
    
    uint32_t AIRMS_AVG, BIRMS_AVG, CIRMS_AVG, AVRMS_AVG, BVRMS_AVG, CVRMS_AVG;
    
    regs_of_interest_L1.calibration_data->v_lsb = 1;
   regs_of_interest_L1.calibration_data->i_lsb = 1;
   regs_of_interest_L2.calibration_data->v_lsb = 1;
   regs_of_interest_L2.calibration_data->i_lsb = 1;
   regs_of_interest_L3.calibration_data->v_lsb = 1;
   regs_of_interest_L3.calibration_data->i_lsb = 1;
    
    writeSPI1(AIGAIN,0x00000000, 4);
    writeSPI1(AVGAIN,0x00000000, 4);
    
    delayms(CALIBRATION_AVERAGING_CYCLES*25);
    
    AIRMS_AVG = readRegsOfInterest(regs_of_interest_L1, "L1 IRMS");
    BIRMS_AVG = readRegsOfInterest(regs_of_interest_L2, "L2 IRMS");
    CIRMS_AVG = readRegsOfInterest(regs_of_interest_L3, "L3 IRMS");
    
    regs_of_interest_L1.calibration_data->igain = 0;
    regs_of_interest_L2.calibration_data->igain = AIRMS_AVG/BIRMS_AVG-1;
    regs_of_interest_L3.calibration_data->igain = AIRMS_AVG/CIRMS_AVG-1;
    
    writeSPI1(BIGAIN, regs_of_interest_L2.calibration_data->igain, 4);
    writeSPI1(CIGAIN, regs_of_interest_L3.calibration_data->igain, 4);
    
    AVRMS_AVG = readRegsOfInterest(regs_of_interest_L1, "L1 VRMS");
    BVRMS_AVG = readRegsOfInterest(regs_of_interest_L2, "L2 VRMS");
    CVRMS_AVG = readRegsOfInterest(regs_of_interest_L3, "L3 VRMS");
    
    regs_of_interest_L1.calibration_data->vgain = 0;
    regs_of_interest_L2.calibration_data->vgain = AVRMS_AVG/BVRMS_AVG-1;
    regs_of_interest_L3.calibration_data->vgain = AVRMS_AVG/CVRMS_AVG-1;
    
    writeSPI1(BVGAIN, regs_of_interest_L2.calibration_data->vgain, 4);
    writeSPI1(CVGAIN, regs_of_interest_L3.calibration_data->vgain, 4);
    
    addr2write = VGAIN_H;
    eeStoreUint32_t(regs_of_interest_L1.calibration_data->vgain, addr2write);
    
    addr2write = IGAIN_H;
    eeStoreUint32_t(regs_of_interest_L1.calibration_data->igain, addr2write);
    
    addr2write = VGAIN_H;
    eeStoreUint32_t(regs_of_interest_L2.calibration_data->vgain, addr2write+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2write = IGAIN_H;
    eeStoreUint32_t(regs_of_interest_L2.calibration_data->igain, addr2write+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2write = VGAIN_H;
    eeStoreUint32_t(regs_of_interest_L3.calibration_data->vgain, addr2write+EEPROM_CALIB_SIZE_PER_PHASE*2);
    
    addr2write = IGAIN_H;
    eeStoreUint32_t(regs_of_interest_L3.calibration_data->igain, addr2write+EEPROM_CALIB_SIZE_PER_PHASE*2);
}

void calibrateMeteringIC_VILSBCalib()
{
    enum calib_reg_addr addr2write;
   
    delayms(CALIBRATION_AVERAGING_CYCLES*25);
   
   regs_of_interest_L1.calibration_data->v_lsb = NOMINAL_TEST_VOLTAGE/readRegsOfInterest(regs_of_interest_L1, "L1 VRMS");
   regs_of_interest_L1.calibration_data->i_lsb = NOMINAL_TEST_CURRENT/readRegsOfInterest(regs_of_interest_L1, "L1 IRMS");
   
   regs_of_interest_L2.calibration_data->v_lsb = NOMINAL_TEST_VOLTAGE/readRegsOfInterest(regs_of_interest_L2, "L2 VRMS");
   regs_of_interest_L2.calibration_data->i_lsb = NOMINAL_TEST_CURRENT/readRegsOfInterest(regs_of_interest_L2, "L2 IRMS");
   
   regs_of_interest_L3.calibration_data->v_lsb = NOMINAL_TEST_VOLTAGE/readRegsOfInterest(regs_of_interest_L3, "L3 VRMS");
   regs_of_interest_L3.calibration_data->i_lsb = NOMINAL_TEST_CURRENT/readRegsOfInterest(regs_of_interest_L3, "L3 IRMS");
   
    addr2write = V_LSB_H;
    eeStoreFloat(regs_of_interest_L1.calibration_data->v_lsb, addr2write);
    
    addr2write = I_LSB_H;
    eeStoreFloat(regs_of_interest_L1.calibration_data->i_lsb, addr2write);
    
    addr2write = V_LSB_H;
    eeStoreFloat(regs_of_interest_L2.calibration_data->v_lsb, addr2write+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2write = I_LSB_H;
    eeStoreFloat(regs_of_interest_L2.calibration_data->i_lsb, addr2write+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2write = V_LSB_H;
    eeStoreFloat(regs_of_interest_L3.calibration_data->v_lsb, addr2write+EEPROM_CALIB_SIZE_PER_PHASE*2);
    
    addr2write = I_LSB_H;
    eeStoreFloat(regs_of_interest_L3.calibration_data->i_lsb, addr2write+EEPROM_CALIB_SIZE_PER_PHASE*2);
}

void calibrateMeteringIC_VIOSMatching()
{
    enum calib_reg_addr addr2write;
     
    int64_t i_expected_low_squared_L1, v_expected_low_squared_L1;
    int64_t i_expected_low_squared_L2, v_expected_low_squared_L2;
    int64_t i_expected_low_squared_L3, v_expected_low_squared_L3;
    
    int64_t v_l1_os_low, i_l1_os_low;
    int64_t v_l2_os_low, i_l2_os_low;
    int64_t v_l3_os_low, i_l3_os_low;
    
    int64_t fv_l1_os_low, fi_l1_os_low;
    int64_t fv_l2_os_low, fi_l2_os_low;
    int64_t fv_l3_os_low, fi_l3_os_low;
    
    delayms(CALIBRATION_AVERAGING_CYCLES*25);
    
    v_l1_os_low = readRegsOfInterest(regs_of_interest_L1, "L1 VRMS")/regs_of_interest_L1.calibration_data->v_lsb;
    i_l1_os_low = readRegsOfInterest(regs_of_interest_L1, "L1 IRMS")/regs_of_interest_L1.calibration_data->i_lsb;
    v_l2_os_low = readRegsOfInterest(regs_of_interest_L2, "L2 VRMS")/regs_of_interest_L2.calibration_data->v_lsb;
    i_l2_os_low = readRegsOfInterest(regs_of_interest_L2, "L2 IRMS")/regs_of_interest_L2.calibration_data->i_lsb;
    v_l3_os_low = readRegsOfInterest(regs_of_interest_L3, "L3 VRMS")/regs_of_interest_L3.calibration_data->v_lsb;
    i_l3_os_low = readRegsOfInterest(regs_of_interest_L3, "L3 IRMS")/regs_of_interest_L3.calibration_data->i_lsb;
    
    fv_l1_os_low = readRegsOfInterest(regs_of_interest_L1, "L1 FVRMS")/regs_of_interest_L1.calibration_data->v_lsb;
    fi_l1_os_low = readRegsOfInterest(regs_of_interest_L1, "L1 FIRMS")/regs_of_interest_L1.calibration_data->i_lsb;
    fv_l2_os_low = readRegsOfInterest(regs_of_interest_L2, "L2 FVRMS")/regs_of_interest_L2.calibration_data->v_lsb;
    fi_l2_os_low = readRegsOfInterest(regs_of_interest_L2, "L2 FIRMS")/regs_of_interest_L2.calibration_data->i_lsb;
    fv_l3_os_low = readRegsOfInterest(regs_of_interest_L3, "L3 FVRMS")/regs_of_interest_L3.calibration_data->v_lsb;
    fi_l3_os_low = readRegsOfInterest(regs_of_interest_L3, "L3 FIRMS")/regs_of_interest_L3.calibration_data->i_lsb;
    
    i_expected_low_squared_L1 = (MINIMAL_TEST_CURRENT/regs_of_interest_L1.calibration_data->i_lsb)*(MINIMAL_TEST_CURRENT/regs_of_interest_L1.calibration_data->i_lsb);
    v_expected_low_squared_L1 = (MINIMAL_TEST_VOLTAGE/regs_of_interest_L1.calibration_data->v_lsb)*(MINIMAL_TEST_VOLTAGE/regs_of_interest_L1.calibration_data->v_lsb);
    i_expected_low_squared_L2 = (MINIMAL_TEST_CURRENT/regs_of_interest_L2.calibration_data->i_lsb)*(MINIMAL_TEST_CURRENT/regs_of_interest_L2.calibration_data->i_lsb);
    v_expected_low_squared_L2 = (MINIMAL_TEST_VOLTAGE/regs_of_interest_L2.calibration_data->v_lsb)*(MINIMAL_TEST_VOLTAGE/regs_of_interest_L2.calibration_data->v_lsb);
    i_expected_low_squared_L3 = (MINIMAL_TEST_CURRENT/regs_of_interest_L3.calibration_data->i_lsb)*(MINIMAL_TEST_CURRENT/regs_of_interest_L3.calibration_data->i_lsb);
    v_expected_low_squared_L3 = (MINIMAL_TEST_VOLTAGE/regs_of_interest_L3.calibration_data->v_lsb)*(MINIMAL_TEST_VOLTAGE/regs_of_interest_L3.calibration_data->v_lsb);
    
    
    regs_of_interest_L1.calibration_data->ios = (i_expected_low_squared_L1-(i_l1_os_low)*(i_l1_os_low))/128;
    regs_of_interest_L1.calibration_data->fios = (i_expected_low_squared_L1-(fi_l1_os_low)*(fi_l1_os_low))/128;
    regs_of_interest_L1.calibration_data->vos = (v_expected_low_squared_L1-(v_l1_os_low)*(v_l1_os_low))/128;
    regs_of_interest_L1.calibration_data->fvos = (v_expected_low_squared_L1-(fv_l1_os_low)*(fv_l1_os_low))/128;
    
    regs_of_interest_L2.calibration_data->ios = (i_expected_low_squared_L2-(i_l2_os_low)*(i_l2_os_low))/128;
    regs_of_interest_L2.calibration_data->fios = (i_expected_low_squared_L2-(fi_l2_os_low)*(fi_l2_os_low))/128;
    regs_of_interest_L2.calibration_data->vos = (v_expected_low_squared_L2-(v_l2_os_low)*(v_l2_os_low))/128;
    regs_of_interest_L2.calibration_data->fvos = (v_expected_low_squared_L2-(fv_l2_os_low)*(fv_l2_os_low))/128;
    
    regs_of_interest_L3.calibration_data->ios = (i_expected_low_squared_L3-(i_l3_os_low)*(i_l3_os_low))/128;
    regs_of_interest_L3.calibration_data->fios = (i_expected_low_squared_L3-(fi_l3_os_low)*(fi_l3_os_low))/128;
    regs_of_interest_L3.calibration_data->vos = (v_expected_low_squared_L3-(v_l3_os_low)*(v_l3_os_low))/128;
    regs_of_interest_L3.calibration_data->fvos = (v_expected_low_squared_L3-(fv_l3_os_low)*(fv_l3_os_low))/128;
    
    writeSPI1(AIRMSOS, regs_of_interest_L1.calibration_data->ios, 4);
    writeSPI1(BIRMSOS, regs_of_interest_L2.calibration_data->ios, 4);
    writeSPI1(CIRMSOS, regs_of_interest_L3.calibration_data->ios, 4);
    
    writeSPI1(AFIRMSOS, regs_of_interest_L1.calibration_data->fios, 4);
    writeSPI1(BFIRMSOS, regs_of_interest_L2.calibration_data->fios, 4);
    writeSPI1(CFIRMSOS, regs_of_interest_L3.calibration_data->fios, 4);
    
    writeSPI1(AVRMSOS, regs_of_interest_L1.calibration_data->vos, 4);
    writeSPI1(BVRMSOS, regs_of_interest_L2.calibration_data->vos, 4);
    writeSPI1(CVRMSOS, regs_of_interest_L3.calibration_data->vos, 4);
    
    writeSPI1(AFVRMSOS, regs_of_interest_L1.calibration_data->fvos, 4);
    writeSPI1(BFVRMSOS, regs_of_interest_L2.calibration_data->fvos, 4);
    writeSPI1(CFVRMSOS, regs_of_interest_L3.calibration_data->fvos, 4);
    
    addr2write = VOS_H;
    eeStoreInt32_t(regs_of_interest_L1.calibration_data->vos, addr2write);
    
    addr2write = IOS_H;
    eeStoreInt32_t(regs_of_interest_L1.calibration_data->ios, addr2write);
    
    addr2write = FVOS_H;
    eeStoreInt32_t(regs_of_interest_L1.calibration_data->fvos, addr2write);
    
    addr2write = FIOS_H;
    eeStoreInt32_t(regs_of_interest_L1.calibration_data->fios, addr2write);
    
    addr2write = VOS_H;
    eeStoreInt32_t(regs_of_interest_L2.calibration_data->vos, addr2write+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2write = IOS_H;
    eeStoreInt32_t(regs_of_interest_L2.calibration_data->ios, addr2write+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2write = FVOS_H;
    eeStoreInt32_t(regs_of_interest_L2.calibration_data->fvos, addr2write+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2write = FIOS_H;
    eeStoreInt32_t(regs_of_interest_L2.calibration_data->fios, addr2write+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2write = VOS_H;
    eeStoreInt32_t(regs_of_interest_L3.calibration_data->vos, addr2write+EEPROM_CALIB_SIZE_PER_PHASE*2);
    
    addr2write = IOS_H;
    eeStoreInt32_t(regs_of_interest_L3.calibration_data->ios, addr2write+EEPROM_CALIB_SIZE_PER_PHASE*2);
    
    addr2write = FVOS_H;
    eeStoreInt32_t(regs_of_interest_L3.calibration_data->fvos, addr2write+EEPROM_CALIB_SIZE_PER_PHASE*2);
    
    addr2write = FIOS_H;
    eeStoreInt32_t(regs_of_interest_L3.calibration_data->fios, addr2write+EEPROM_CALIB_SIZE_PER_PHASE*2);
}


void calibrateMeteringIC_PhaseCalib()
{
    enum calib_reg_addr addr2write;
    
    float phaseResolution = 360*50/(1.024*10e6);
    float sin_60 = 0.86602540378;
    float l1_watthr_buf, l1_varhr_buf, l2_watthr_buf, l2_varhr_buf, l3_watthr_buf, l3_varhr_buf;
    float l1_error_deg, l2_error_deg, l3_error_deg;
    float l1_phase_comp, l2_phase_comp, l3_phase_comp;

    regs_of_interest_L1.calibration_data->wh_lsb = 1;
    regs_of_interest_L2.calibration_data->wh_lsb = 1;
    regs_of_interest_L3.calibration_data->wh_lsb = 1;
    regs_of_interest_L1.calibration_data->phcal = 0;
    regs_of_interest_L2.calibration_data->phcal = 0;
    regs_of_interest_L3.calibration_data->phcal = 0;
    /*
    delayms(CALIBRATION_AVERAGING_CYCLES*25);
    
    l1_watthr_buf = readRegsOfInterest(regs_of_interest_L1, "L1 ENERGY");
    l1_varhr_buf = readRegsOfInterest(regs_of_interest_L1, "L1 REACTIVE ENERGY");
    l2_watthr_buf = readRegsOfInterest(regs_of_interest_L2, "L2 ENERGY");
    l2_varhr_buf = readRegsOfInterest(regs_of_interest_L2, "L2 REACTIVE ENERGY");
    l3_watthr_buf = readRegsOfInterest(regs_of_interest_L3, "L3 ENERGY");
    l3_varhr_buf = readRegsOfInterest(regs_of_interest_L3, "L3 REACTIVE ENERGY");

    l1_error_deg = ((l1_watthr_buf*sin_60-l1_varhr_buf*sin_60)/(l1_watthr_buf*sin_60+l1_varhr_buf*sin_60))*180/pi;
    l2_error_deg = ((l2_watthr_buf*sin_60-l2_varhr_buf*sin_60)/(l2_watthr_buf*sin_60+l2_varhr_buf*sin_60))*180/pi;
    l3_error_deg = ((l3_watthr_buf*sin_60-l3_varhr_buf*sin_60)/(l3_watthr_buf*sin_60+l3_varhr_buf*sin_60))*180/pi;
    
    l1_phase_comp = fabs(l1_error_deg/phaseResolution);
    l2_phase_comp = fabs(l2_error_deg/phaseResolution);
    l3_phase_comp = fabs(l3_error_deg/phaseResolution);
    
    if(l1_error_deg>0)
    {
        regs_of_interest_L1.calibration_data->phcal = (int32_t)(l1_phase_comp+512);
    }
    else
    {
        regs_of_interest_L1.calibration_data->phcal = (int32_t)l1_phase_comp;
    }
    
    if(l2_error_deg>0)
    {
        regs_of_interest_L2.calibration_data->phcal = (int32_t)(l2_phase_comp+512);
    }
    else
    {
        regs_of_interest_L2.calibration_data->phcal = (int32_t)l2_phase_comp;
    }
    
    if(l3_error_deg>0)
    {
        regs_of_interest_L3.calibration_data->phcal = (int32_t)(l3_phase_comp+512);
    }
    else
    {
         regs_of_interest_L3.calibration_data->phcal = (int32_t)l3_phase_comp;
    }
    */
    writeSPI1(APHCAL, regs_of_interest_L1.calibration_data->phcal, 4);
    writeSPI1(BPHCAL, regs_of_interest_L2.calibration_data->phcal, 4);
    writeSPI1(CPHCAL, regs_of_interest_L3.calibration_data->phcal, 4);
    
    addr2write = PHCAL_H;
    eeStoreFloat(regs_of_interest_L1.calibration_data->phcal, addr2write);
    
    addr2write = PHCAL_H;
    eeStoreFloat(regs_of_interest_L2.calibration_data->phcal, addr2write+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2write = PHCAL_H;
    eeStoreFloat(regs_of_interest_L3.calibration_data->phcal, addr2write+EEPROM_CALIB_SIZE_PER_PHASE*2);
    
}

void calibrateMeteringIC_WhLSBCalc()
{
    enum calib_reg_addr addr2write;
    
    uint64_t l1_start_cyc, l2_start_cyc, l3_start_cyc;
    int64_t l1_wh_buffer, l2_wh_buffer, l3_wh_buffer;
    
    regs_of_interest_L1.calibration_data->w_lsb = 1;
    regs_of_interest_L2.calibration_data->w_lsb = 1;
    regs_of_interest_L3.calibration_data->w_lsb = 1;
    
    float nominal_active_load = NOMINAL_TEST_VOLTAGE*NOMINAL_TEST_CURRENT*0.5; //cosphi= 0.5
    
    l1_wh_buffer = readRegsOfInterest(regs_of_interest_L1, "L1 ENERGY");
    l1_start_cyc = l1_line_cyc_cntr;
    while((l1_line_cyc_cntr-l1_start_cyc)<CALIBRATION_AVERAGING_CYCLES);
    l1_wh_buffer = readRegsOfInterest(regs_of_interest_L1, "L1 ENERGY") - l1_wh_buffer;
    
    l2_wh_buffer = readRegsOfInterest(regs_of_interest_L2, "L2 ENERGY");
    l2_start_cyc = l2_line_cyc_cntr;
    while((l2_line_cyc_cntr-l2_start_cyc)<CALIBRATION_AVERAGING_CYCLES);
    l2_wh_buffer = readRegsOfInterest(regs_of_interest_L2, "L2 ENERGY") - l2_wh_buffer;
    
    l3_wh_buffer = readRegsOfInterest(regs_of_interest_L3, "L3 ENERGY");
    l3_start_cyc = l3_line_cyc_cntr;
    while((l3_line_cyc_cntr-l3_start_cyc)<CALIBRATION_AVERAGING_CYCLES);
    l3_wh_buffer = readRegsOfInterest(regs_of_interest_L3, "L3 ENERGY") - l3_wh_buffer;
    
    regs_of_interest_L1.calibration_data->w_lsb = nominal_active_load/readRegsOfInterest(regs_of_interest_L1, "L1 POWER");
    regs_of_interest_L2.calibration_data->w_lsb = nominal_active_load/readRegsOfInterest(regs_of_interest_L2, "L2 POWER");
    regs_of_interest_L3.calibration_data->w_lsb = nominal_active_load/readRegsOfInterest(regs_of_interest_L3, "L3 POWER");
    
    regs_of_interest_L1.calibration_data->wh_lsb = (nominal_active_load*CALIBRATION_AVERAGING_CYCLES*10/100)/(l1_wh_buffer*3600);
    regs_of_interest_L2.calibration_data->wh_lsb = (nominal_active_load*CALIBRATION_AVERAGING_CYCLES*10/100)/(l2_wh_buffer*3600);
    regs_of_interest_L3.calibration_data->wh_lsb = (nominal_active_load*CALIBRATION_AVERAGING_CYCLES*10/100)/(l3_wh_buffer*3600);
    
    addr2write = WH_LSB_H;
    eeStoreFloat(regs_of_interest_L1.calibration_data->wh_lsb, addr2write);
    
    addr2write = W_LSB_H;
    eeStoreFloat(regs_of_interest_L1.calibration_data->w_lsb, addr2write);

    addr2write = WH_LSB_H;
    eeStoreFloat(regs_of_interest_L2.calibration_data->wh_lsb, addr2write+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2write = W_LSB_H;
    eeStoreFloat(regs_of_interest_L2.calibration_data->w_lsb, addr2write+EEPROM_CALIB_SIZE_PER_PHASE);
    
    addr2write = WH_LSB_H;
    eeStoreFloat(regs_of_interest_L3.calibration_data->wh_lsb, addr2write+EEPROM_CALIB_SIZE_PER_PHASE*2);
    
    addr2write = W_LSB_H;
    eeStoreFloat(regs_of_interest_L3.calibration_data->w_lsb, addr2write+EEPROM_CALIB_SIZE_PER_PHASE*2);
    
}

CHANNEL_ERR_e calibrateHandler(uint16_t channel, uint8_t* data, uint16_t length)
{
    switch(channel)
    {
        case 300: //312C
        {
            // BEFORE ENTERING THIS SUPPLY 230Vrms, 8Arms, cosPhi= 1
            calibrateMeteringIC_VIGainMatching();           
            calibrateMeteringIC_VILSBCalib();
            break;
        }
        case 301: //312D
        {
            // BEFORE ENTERING THIS SUPPLY 23Vrms, 0.4Arms, cosPhi= 1
            calibrateMeteringIC_VIOSMatching();
            break;
        }
        case 302: //312E
        {
            // BEFORE ENTERING THIS SUPPLY 230Vrms, 8Arms, cosPhi= 0.5 (inductive)
            calibrateMeteringIC_PhaseCalib();
            calibrateMeteringIC_WhLSBCalc();
            run_or_calib_state = RUN; //TEST   
            initCcan();                         //TEST

 //           calibrateMeteringIC_GainCalib(); //WhLSB calc-ot lehetne általánosabbra tenni, és azzal beégetni egy fix értéket, utána itt állítani +-100%-ban.
            break;
        }
        case 303: //312F
        {
            run_or_calib_state = RUN; //TEST   
            initCcan();                         //TEST
            // BEFORE ENTERING THIS SUPPLY 230Vrms, 0.4Arms, cosPhi= 1
  //          calibrateMeteringIC_WattOSCalib(); //todo latelr
  //          calibrateMeteringIC_VAROSCalib(); //todo later
            break;
        }
        case 304: //3130
        {
            NVIC_SystemReset();
            break;
        }
        default:
            break;
    }
    return CHANNEL_ERROR_NONE;
}
