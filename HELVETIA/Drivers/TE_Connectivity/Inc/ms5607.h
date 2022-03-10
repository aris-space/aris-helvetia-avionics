#ifndef MS5607_H_
#define MS5607_H_

#ifdef __cplusplus
  extern "C" {
#endif

#include "../../Core/Inc/i2c.h"
#include <stdint.h>

// Commands
#define MS5607_COMMAND_RESET           0x1E
#define MS5607_COMMAND_CONVERT_D1_BASE 0x40
#define MS5607_COMMAND_CONVERT_D2_BASE 0x50
#define MS5607_COMMAND_ADC_READ        0x00
#define MS5607_COMMAND_PROM_READ_BASE  0xA0

uint16_t ms5607_addr = 0x76; //TO DO!!!

// Conversion time
#define MS5607_BARO_CONVERSION_TIME_OSR_BASE 0.6f

/* MS5607 Oversampling Ratio Enumeration*/
typedef enum OSRFactors{
  OSR_256,
  OSR_512=0x02,
  OSR_1024=0x04,
  OSR_2048=0x06,
  OSR_4096=0x08
}MS5607OSRFactors;

typedef enum MS5607States{
  MS5607_STATE_FAILED,
  MS5607_STATE_READY
}MS5607StateTypeDef;

struct promData
{
  uint16_t reserved;
  uint16_t sens;
  uint16_t off;
  uint16_t tcs;
  uint16_t tco;
  uint16_t tref;
  uint16_t tempsens;
  uint16_t crc;
};

struct MS5607UncompensatedValues{
  uint32_t pressure;
  uint32_t temperature;
};

struct MS5607Readings{
  int32_t pressure;
  int32_t temperature;
};

void ms5607_assignI2C(I2C_HandleTypeDef *hi2c_device);

void ms5607_setup();

void ms5607_PromRead(struct promData* prom);

void ms5607_UncompensatedRead(struct MS5607UncompensatedValues *uncompensated);

void ms5607_Convert(struct MS5607UncompensatedValues *uncompensated, struct MS5607Readings *readings);

void ms5607_Update();

double ms5607_getTemperature();

double ms5607_getPreassure();

//void ms5607_enableCSB();

//void ms5607_disableCSB();

void ms5607_setTempeatureOSR(MS5607OSRFactors *OSRFactors);

void ms5607_setPressureOSR(MS5607OSRFactors *OSRFactors);

#ifdef __cplusplus
  }
#endif
#endif
