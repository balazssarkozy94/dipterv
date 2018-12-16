#ifndef INIT_H
#define INIT_H

typedef enum
{
  LED_L1 = 3,
  LED_L2,
  LED_L3,
  LED_NUM
} LED_e;

typedef enum
{
  APP_CH_C1_VAL = 461,
  APP_CH_C2_VAL,
  APP_CH_C1_CNT = 471,
  APP_CH_C2_CNT,
} APP_CH_e;

typedef struct
{
  float    val;
  float    sent_val;
  uint64_t sent_ts;
  uint32_t cycle;
  float    sig;
} measurement_s;

typedef struct
{
  char    val;
  char    sent_val;
  uint64_t sent_ts;
  uint32_t cycle;
  float    sig;
} string1_s;

typedef struct
{
  uint8_t    val;
  uint8_t    sent_val;
  uint64_t   sent_ts;
  uint32_t   cycle;
  float      sig;
} signal_s;

typedef struct
{
  uint32_t    val;
  uint32_t    sent_val;
  uint64_t   sent_ts;
  uint32_t   cycle;
  float      sig;
} counter_s;

typedef enum
{
  RUN = 0,
  CALIB
} run_or_calib;

void init();

#endif
