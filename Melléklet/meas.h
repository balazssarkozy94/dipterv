#ifndef MEAS_H
#define MEAS_H

#include <stdint.h>
#include "channel.h"

#define voltage_constant 0.788/8388607*1000
#define current_constant 0.04927/(8388607*0.005)
#define power_constant 1000/0.005/16/26991271*2
#define energy_constant 1000/0.005/16/26991271*2/150
#define temperature_constant_1 8.72101*0.00001
#define temperature_constant_2 306.47

#define CALIBRATION_AVERAGING_CYCLES 200
#define NO_UPDATE_TIMEOUT_VALUE 5

#define V12_MEAS_CONST 12
#define V3V3_MEAS_CONST 2

enum issigned{
    _signed,
    _unsigned
};

enum dimension{
    voltage,
    current,
    power,
    energy,
    temperature,
    scalar
};

struct meas_register{
    const char* tag;
    int64_t value;
    int64_t avg_acc;
    uint32_t avg_cnt;
    uint32_t avg_cycles;
    uint16_t reg_addr;
    uint8_t reg_size;
    enum issigned type;
    enum dimension dimension;
};

struct calib_data{
    float v_lsb;
    float i_lsb;
    float w_lsb;
    float wh_lsb;
    uint32_t vgain;
    uint32_t igain;
    int32_t phcal;
    int32_t vos;
    int32_t fvos;
    int32_t ios;
    int32_t fios;
};

enum calib_reg_addr{
    V_LSB_H = 0,
    V_LSB_L,
    I_LSB_H,
    I_LSB_L,
    W_LSB_H,
    W_LSB_L,
    WH_LSB_H,
    WH_LSB_L,
    VGAIN_H,
    VGAIN_L,
    IGAIN_H,
    IGAIN_L,
    PHCAL_H,
    PHCAL_L,
    VOS_H,
    VOS_L,
    FVOS_H,
    FVOS_L,
    IOS_H,
    IOS_L,
    FIOS_H,
    FIOS_L,
    EEPROM_CALIB_SIZE_PER_PHASE,
    MAX_VALUE = 0xFF
};


struct regs_of_interest_type{
   struct meas_register* reg_array;
   struct calib_data* calibration_data;
   uint8_t meas_update_timeout_cnt;
   uint8_t no_of_regs;
};

void eeStoreInt32_t(int32_t data, uint16_t ee_address);
void eeReadInt32_t(volatile int32_t* calib_address, uint16_t ee_address);
void eeStoreUint32_t(uint32_t data, uint16_t ee_address);
void eeReadUint32_t(volatile uint32_t* calib_address, uint16_t ee_address);
void eeStoreFloat(float data, uint16_t ee_address);
void eeReadFloat(volatile float* calib_address, uint16_t ee_address);
void loadCalibData();
void resetMeter();
void initMeteringIC();
CHANNEL_ERR_e calibrateHandler(uint16_t channel, uint8_t* data, uint16_t length);
void calibrateMeteringIC_VIGainMatching(void);
void calibrateMeteringIC_PhaseCalib(void);
void calibrateMeteringIC_WhLSBCalc(void);
void calibrateMeteringIC_GainCalib(void);
void calibrateMeteringIC_WattOSCalib(void);
void calibrateMeteringIC_VAROSCalib(void);
void calibrateMeteringIC_VILSBCalib(void);
void calibrateMeteringIC_VIOSMatching(void);
void checkAndUpdateOnTimeout(volatile struct regs_of_interest_type* regs_of_interest);
void resetTimeout(volatile struct regs_of_interest_type* regs_of_interest);

void updateRegsOfInterest(struct regs_of_interest_type regs_of_interest);
void displayRegsOfInterest(struct regs_of_interest_type regs_of_interest);
float readRegsOfInterest(struct regs_of_interest_type regs_of_interest, const char* tag);
void addToRegsOfInterest(struct regs_of_interest_type* regs_of_interest, const char* tag, const uint16_t reg_addr, const uint8_t reg_size, const enum issigned type, const enum dimension dimension, uint32_t avg_cycles);

struct instantaneous_values{
    int32_t IAWV;
    int32_t IBWV;
    int32_t ICWV;
    int32_t INWV;
    int32_t VAWV;
    int32_t VBWV;
    int32_t VCWV;
    int32_t VA2WV;
    int32_t VB2WV;
    int32_t VC2WV;
    int32_t VNWV;
    int32_t VN2WV;
    int32_t AWATT;
    int32_t BWATT;
    int32_t CWATT;
    int32_t AVAR;
    int32_t BVAR;
    int32_t CVAR;
    int32_t AVA;
    int32_t BVA;
    int32_t CVA;
    int32_t AVTHD;
    int32_t AITHD;
    int32_t BVTHD;
    int32_t BITHD;
    int32_t CVTHD;
    int32_t CITHD;
};

enum  DSP_register{
    AIGAIN = 0x4380,
    AVGAIN,
    AV2GAIN,
    BIGAIN,
    BVGAIN,
    BV2GAIN,
    CIGAIN,
    CVGAIN,
    CV2GAIN,
    NIGAIN,
    NVGAIN,
    NV2GAIN,
    AIRMSOS,
    AVRMSOS,
    AV2RMSOS,
    BIRMSOS,
    BVRMSOS,
    BV2RMSOS,
    CIRMSOS,
    CVRMSOS,
    CV2RMSOS,
    NIRMSOS,
    NVRMSOS,
    NV2RMSOS,
    ISUMLVL,
    APGAIN,
    BPGAIN,
    CPGAIN,
    AWATTOS,
    BWATTOS,
    CWATTOS,
    AVAROS,
    BVAROS,
    CVAROS,
    VLEVEL,
    AFWATTOS,
    BFWATTOS,
    CFWATTOS,
    AFVAROS,
    BFVAROS,
    CFVAROS,
    AFIRMSOS,
    BFIRMSOS,
    CFIRMSOS,
    AFVRMSOS,
    BFVRMSOS,
    CFVRMSOS,
    TEMPCO,
    ATEMP0,
    BTEMP0,
    CTEMP0,
    NTEMP0,
    ATGAIN,
    BTGAIN,
    CTGAIN,
    NTGAIN,
    //Reserved registers from 0x43B8 to 0x43BF
    AIRMS = 0x43C0,
    AVRMS,
    AV2RMS,
    BIRMS,
    BVRMS,
    BV2RMS,
    CIRMS,
    CVRMS,
    CV2RMS,
    NIRMS,
    ISUM,
    ATEMP,
    BTEMP,
    CTEMP,
    NTEMP,
    //Reserved registers from 0x43CF to 0x43FF
    // 0xE203 Reserved
    Run = 0xE228, //16 Bit register!
    
    //Billable registers
    AWATTHR = 0xE400,
    BWATTHR,
    CWATTHR,
    AFWATTHR,
    BFWATTHR,
    CFWATTHR,
    AVARHR,
    BVARHR,
    CVARHR,
    AFVARHR,
    BFVARHR,
    CFVARHR,
    AVAHR,
    BVAHR,
    CVAHR,
    
    //Configuration and Power Quality registers
    IPEAK = 0xE500,
    VPEAK,
    STATUS0,
    STATUS1,
    //Reserved registers from 0xE504 to 0xE506
    OILVL = 0xE507,
    OVLVL,
    SAGLVL,
    MASK0,
    MASK1,
    IAWV,
    IBWV,
    ICWV,
    INWV,
    VAWV,
    VBWV,
    VCWV,
    VA2WV,
    VB2WV,
    VC2WV,
    VNWV,
    VN2WV,
    AWATT,
    BWATT,
    CWATT,
    AVAR,
    BVAR,
    CVAR,
    AVA,
    BVA,
    CVA,
    AVTHD,
    AITHD,
    BVTHD,
    BITHD,
    CVTHD,
    CITHD,
    //Reserved registers from 0xE527 to 0xE52F
    NVRMS = 0xE530,
    NV2RMS,
    CHECKSUM,
    VNOM,
    //Reserved registers from 0xE534 to 0xE536
    AFIRMS = 0xE537,
    AFVRMS,
    BFIRMS,
    BFVRMS,
    CFIRMS,
    CFVRMS,
    //Reserved registers from 0xE53D to 0xE5FE
    LAST_RWDATA32 = 0xE5FF,
    PHSTATUS,
    ANGLE0,
    ANGLE1,
    ANGLE2,
    //Reserved registers from 0xE604 to 0x607
    PHNOLOAD = 0xE608,
    //Reserved registers from 0xE609 to 0xE60B
    LINECYC = 0xE60C,
    ZXTOUT,
    COMPMODE,
    //Reserved register 0xE60F
    CFMODE = 0xE610,
    CF1DEN,
    CF2DEN,
    CF3DEN,
    APHCAL,
    BPHCAL,
    CPHCAL,
    PHSIGN,
    CONFIG,
    //Reserved registers from 0xE619 to 0xE6FF
    MMODE = 0xE700,
    ACCMODE,
    LCYCMODE,
    PEAKCYC,
    SAGCYC,
    CFCYC,
    HSDC_CFG,
    Version,
    CONFIG3,
    ATEMPOS,
    BTEMPOS,
    CTEMPOS,
    NTEMPOS,
    //Reserved registers from 0xE70D to 0xE7FC
    LAST_RWDATA8 = 0xE7FD,
    //Reserved registers from 0xE7FE to 0xE901
    APF = 0xE902,
    BPF,
    CPF,
    APERIOD,
    BPERIOD,
    CPERIOD,
    APNOLOAD,
    VARNOLOAD,
    VANOLOAD,
    //Reserved registers from 0xE90B to 0xE9FD
    LAST_ADD = 0xE9FE,
    LAST_RWDATA16,
    CONFIG2,
    LAST_OP,
    WTHR,
    VARTHR,
    VATHR,
    //Reserved registers from 0xEA05 to 0xEBFF
};

#endif
