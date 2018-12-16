#include <string.h>
#include "init.h"
#include "cr_base.h"
#include "story.txt"
#include "meas.h"
#include "gpio.h"
#include "spi.h"
#include "adc.h"
#include "timer.h"
#include "uart.h"
#include "adc.h"
#include "dma.h"
#include "nvic.h"
#include <stdio.h>

__attribute__(( __section__( ".app_data" ))) const uint16_t a16Version[2] = { __VERSION_HIGH__, __VERSION_LOW__ };

volatile run_or_calib run_or_calib_state;

void initLeds()
{
  ledConf_s baseLeds[4] = {
    { GPIO_Pin_2,  GPIOA, RCC_AHBPeriph_GPIOA, LED_LOGIC_INVERT, 0, LED_SYS_BUS},
    { GPIO_Pin_3, GPIOA, RCC_AHBPeriph_GPIOA, LED_LOGIC_INVERT, 0, LED_SYS_FLT},
    { GPIO_Pin_1,  GPIOA, RCC_AHBPeriph_GPIOA, LED_LOGIC_INVERT, 0, LED_SYS_STAT},
    { 0, 0, 0, 0, 0, 0}
  };
  ledInitSys(baseLeds);

  ledConf_s leds[LED_NUM+1]= {
    { GPIO_Pin_0, GPIOB, RCC_AHBPeriph_GPIOB, LED_LOGIC_INVERT, 0, LED_L1},
    { GPIO_Pin_1, GPIOB, RCC_AHBPeriph_GPIOB, LED_LOGIC_INVERT, 0, LED_L2},
    { GPIO_Pin_2, GPIOB, RCC_AHBPeriph_GPIOB, LED_LOGIC_INVERT, 0, LED_L3},
    { 0, 0, 0, 0, 0, 0}
  };
  ledInit(leds);
}

void initCcan()
{
  CCAN_GPIOConf_s     gpio;
  CCAN_ModConf_s      module;
  CCAN_Addrs_s        addr;
  CCAN_ProtocolConf_s prot;

  gpio.gpioPort = GPIOA;
  gpio.clock = RCC_AHBPeriph_GPIOA;
  gpio.gpioAf = GPIO_AF_9;
  gpio.txPin = GPIO_Pin_12;
  gpio.txPinSource = GPIO_PinSource12;
  gpio.rxPin = GPIO_Pin_11;
  gpio.rxPinSource = GPIO_PinSource11;

  module.module = CAN1;
  module.mode = CAN_Mode_Normal;
  module.fifo = CAN_FIFO0;
  module.rcc_apb = RCC_APB1Periph_CAN1;
  module.prescaler = 3;
  module.bs1 = CAN_BS1_8tq;
  module.bs2 = CAN_BS2_3tq;
  module.sjw = CAN_SJW_1tq;

  addr.type = CCAN_ATYPE_FIX;
  addr.gpioPort = GPIOC;
  addr.clock = RCC_AHBPeriph_GPIOC;
  addr.pin0 = GPIO_Pin_13;
  addr.pin1 = GPIO_Pin_14;
  addr.pin2 = GPIO_Pin_15;
  addr.fixAddr = 0;

  memset( &prot, 0, sizeof(prot));
  if(run_or_calib_state == CALIB)
  {
      prot.sendHB = 0;
  }
  else
  {
      prot.sendHB = 1;  
  }
 
  prot.checkHB = 0;
  prot.checkRxState = 0;
  prot.maxHBmisses = 2;
  prot.checkBlkIndex = 0;

  cCanInit(gpio, module, addr, prot);
}

void initVers()
{
  versions_s v;
  memset(&v, 0, sizeof(v));
  uint8_t hwtype[8] = __HW_TYPE__;
  memcpy(v.hwType, hwtype, 8 );
  uint8_t appType[8] = __APPL_TYPE__;
  memcpy(v.appType, appType, 8 );
  v.appVer[0] = __VERSION_HIGH__;
  v.appVer[1] = __VERSION_LOW__;
  v.appParamVer[0] = 1;
  v.appParamVer[1] = 5;
  versionInit(v);
}

#define PARAM_ARR_SIZE 50
paramDesc_s  paramDescArr[PARAM_ARR_SIZE];

#define DATA_ARR_SIZE 50
dataDesc_s dataDescArr[DATA_ARR_SIZE];

measurement_s l1_temp;
measurement_s l1_vrms;
measurement_s l1_irms;
measurement_s l1_pow;
measurement_s l1_ener;

measurement_s l2_temp;
measurement_s l2_vrms;
measurement_s l2_irms;
measurement_s l2_pow;
measurement_s l2_ener;

measurement_s l3_temp;
measurement_s l3_vrms;
measurement_s l3_irms;
measurement_s l3_pow;
measurement_s l3_ener;

measurement_s n_temp;
measurement_s n_vrms;
measurement_s n_irms;

void initParams()
{
  paramInitArr(paramDescArr, PARAM_ARR_SIZE);
  
  paramAddMacro( 299,  1 , l1_temp.cycle);
  paramAddMacro( 299,  2 , l1_temp.sig);
  paramAddMacro( 299,  3 , l1_vrms.cycle);
  paramAddMacro( 299,  4 , l1_vrms.sig);
  paramAddMacro( 299,  5 , l1_irms.cycle);
  paramAddMacro( 299,  6 , l1_irms.sig);
  paramAddMacro( 299,  7 , l1_pow.cycle);
  paramAddMacro( 299,  8 , l1_pow.sig);
  paramAddMacro( 299,  9 , l1_ener.cycle);
  paramAddMacro( 299,  10 , l1_ener.sig);
  
  paramAddMacro( 299,  11 , l2_temp.cycle);
  paramAddMacro( 299,  12 , l2_temp.sig);
  paramAddMacro( 299,  13 , l2_vrms.cycle);
  paramAddMacro( 299,  14 , l2_vrms.sig);
  paramAddMacro( 299,  15 , l2_irms.cycle);
  paramAddMacro( 299,  16 , l2_irms.sig);
  paramAddMacro( 299,  17 , l2_pow.cycle);
  paramAddMacro( 299,  18 , l2_pow.sig);
  paramAddMacro( 299,  19 , l2_ener.cycle);
  paramAddMacro( 299,  20 , l2_ener.sig);
  
  paramAddMacro( 299,  21 , l3_temp.cycle);
  paramAddMacro( 299,  22 , l3_temp.sig);
  paramAddMacro( 299,  23 , l3_vrms.cycle);
  paramAddMacro( 299,  24 , l3_vrms.sig);
  paramAddMacro( 299,  25 , l3_irms.cycle);
  paramAddMacro( 299,  26 , l3_irms.sig);
  paramAddMacro( 299,  27 , l3_pow.cycle);
  paramAddMacro( 299,  28 , l3_pow.sig);
  paramAddMacro( 299,  29 , l3_ener.cycle);
  paramAddMacro( 299,  30 , l3_ener.sig);
  
  paramAddMacro( 299,  31 , n_temp.cycle);
  paramAddMacro( 299,  32 , n_temp.sig);
  paramAddMacro( 299,  33 , n_vrms.cycle);
  paramAddMacro( 299,  34 , n_vrms.sig);
  paramAddMacro( 299,  35 , n_irms.cycle);
  paramAddMacro( 299,  36 , n_irms.sig);
}

void initData()
{    
  dataInitArr( dataDescArr, DATA_ARR_SIZE);
  
  dataAddPubMacro(400, DATA_TYPE_MEASURE, l1_temp.val, l1_temp.sent_val, l1_temp.sent_ts, 299, 1, 299, 2);
  dataAddPubMacro(401, DATA_TYPE_MEASURE, l1_vrms.val, l1_vrms.sent_val, l1_vrms.sent_ts, 299, 3, 299, 4);
  dataAddPubMacro(402, DATA_TYPE_MEASURE, l1_irms.val, l1_irms.sent_val, l1_irms.sent_ts, 299, 5, 299, 6);
  dataAddPubMacro(403, DATA_TYPE_MEASURE, l1_pow.val, l1_pow.sent_val, l1_pow.sent_ts, 299, 7, 299, 8);
  dataAddPubMacro(404, DATA_TYPE_MEASURE, l1_ener.val, l1_ener.sent_val, l1_ener.sent_ts, 299, 9, 299, 10);
  
  dataAddPubMacro(405, DATA_TYPE_MEASURE, l1_temp.val, l1_temp.sent_val, l1_temp.sent_ts, 299, 11, 299, 12);
  dataAddPubMacro(406, DATA_TYPE_MEASURE, l1_vrms.val, l1_vrms.sent_val, l1_vrms.sent_ts, 299, 13, 299, 14);
  dataAddPubMacro(407, DATA_TYPE_MEASURE, l1_irms.val, l1_irms.sent_val, l1_irms.sent_ts, 299, 15, 299, 16);
  dataAddPubMacro(408, DATA_TYPE_MEASURE, l1_pow.val, l1_pow.sent_val, l1_pow.sent_ts, 299, 17, 299, 18);
  dataAddPubMacro(409, DATA_TYPE_MEASURE, l1_ener.val, l1_ener.sent_val, l1_ener.sent_ts, 299, 19, 299, 20);
  
  dataAddPubMacro(410, DATA_TYPE_MEASURE, l1_temp.val, l1_temp.sent_val, l1_temp.sent_ts, 299, 21, 299, 22);
  dataAddPubMacro(411, DATA_TYPE_MEASURE, l1_vrms.val, l1_vrms.sent_val, l1_vrms.sent_ts, 299, 23, 299, 24);
  dataAddPubMacro(412, DATA_TYPE_MEASURE, l1_irms.val, l1_irms.sent_val, l1_irms.sent_ts, 299, 25, 299, 26);
  dataAddPubMacro(413, DATA_TYPE_MEASURE, l1_pow.val, l1_pow.sent_val, l1_pow.sent_ts, 299, 27, 299, 28);
  dataAddPubMacro(414, DATA_TYPE_MEASURE, l1_ener.val, l1_ener.sent_val, l1_ener.sent_ts, 299, 29, 299, 30);
  
  dataAddPubMacro(415, DATA_TYPE_MEASURE, l1_temp.val, l1_temp.sent_val, l1_temp.sent_ts, 299, 31, 299, 32);
  dataAddPubMacro(416, DATA_TYPE_MEASURE, l1_vrms.val, l1_vrms.sent_val, l1_vrms.sent_ts, 299, 33, 299, 34);
  dataAddPubMacro(417, DATA_TYPE_MEASURE, l1_irms.val, l1_irms.sent_val, l1_irms.sent_ts, 299, 35, 299, 36);

}

#define CHANNEL_ARR_SIZE 30
channelDesc_s channelsArr[CHANNEL_ARR_SIZE];

void initChannels()
{
  channelInitArr( channelsArr, CHANNEL_ARR_SIZE);
  channelAdd(299, 299, &paramCanHandler);
  channelAdd(300, 310, &calibrateHandler);
}

void myCallback( CCAN_CBREASON_e reason )
{
  switch(reason)
  {
    case CCAN_CBREASON_PARAM2RUN:
      dataSendSubs();
//      ledOn(LED_L1);
      break;
    case CCAN_CBREASON_RUN2PARAM:
    case CCAN_CBREASON_RUN2DISC:
 //     ledOff(LED_L1);
      break;
    default:
      break;
  }
}

void init()
{
    //lib init
    crBaseInit();
    cCanSysCmdInit( myCallback );
    
    //calib select init end
    
    //user inits of lib modules
    initLeds();
    initCcan();
    initParams();
    initData();
    initChannels();
    initVers();

    //user init of own module
    ledOnSys(LED_SYS_FLT);
    initGpios();

    initTimer17();
    initSPI();
    initDMA();
    initADC();
    initMeteringIC();
    ledOffSys(LED_SYS_FLT);
    initNVIC();
}


