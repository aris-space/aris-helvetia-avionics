#ifndef MS5607_H_
#define MS5607_H_



// Commands
#define COMMAND_RESET           0x1E
#define COMMAND_CONVERT_D1_BASE 0x40
#define COMMAND_CONVERT_D2_BASE 0x50
#define COMMAND_ADC_READ        0x00
#define COMMAND_PROM_READ_BASE  0xA0

// Conversion time
#define BARO_CONVERSION_TIME_OSR_BASE 0.6f

/* MS5607 Oversampling Ratio Enumeration*/
typedef enum OSRFactors{
  OSR_256,
  OSR_512=0x02,
  OSR_1024=0x04,
  OSR_2048=0x06,
  OSR_4096=0x08
}MS5607OSRFactors;


void ms5607_setup();
void ms5607_reset();



float ms5607_getPressure();
float ms5607_getTemperature();

#endif