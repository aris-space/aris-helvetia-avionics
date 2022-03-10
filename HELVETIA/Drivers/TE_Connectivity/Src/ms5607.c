#include "ms5607.h"


I2C_HandleTypeDef *_ms5607_i2c_port;

// Private OSR Instantiations
static uint8_t Pressure_OSR = OSR_256;
static uint8_t Temperature_OSR = OSR_256;

static struct promData promData;
static struct MS5607UncompensatedValues uncompValues;
static struct MS5607Readings readings;



void ms5607_assignI2C(I2C_HandleTypeDef *hi2c_device){
    _ms5607_i2c_port = hi2c_device;
}

void ms5607_setup(){
    //ms5607_enableCSB();
    uint8_t msg = MS5607_COMMAND_RESET;
    HAL_I2C_Master_Transmit(_ms5607_i2c_port, ms5607_addr, &msg, 1, 10);
    HAL_Delay(3);
    //ms5607_disableCSB();
    if (promData.reserved == 0x00 || promData.reserved == 0xff)
        return MS5607_STATE_FAILED;
    else
        return MS5607_STATE_READY;
}

void MS5607PromRead(struct promData *prom){
    uint8_t addr;
    uint16_t *structPointer;
    
    structPointer = (uint16_t*) prom;
    for (addr = 0;addr < 8; addr++){
        uint8_t msg = PROM_READ(addr);
        //ms5607_enableCSB();
        HAL_I2C_Master_Transmit(_ms5607_i2c_port,ms5607_addr,&msg,1,10);
        HAL_I2C_Master_Receive(_ms5607_i2c_port,ms5607_addr,structPointer,2,10);
        //ms5607_disableCSB();
        structPointer++;
    }
    structPointer = (uint16_t*) prom;
    for (addr = 0;addr < 8;addr++){
        uint8_t *toSwap = (uint8_t*) structPointer;
        uint8_t secondByte = toSwap[0];
        toSwap[0] = toSwap[1];
        toSwap[1] = secondByte;
        structPointer++;
    }
}

void MS5607UncompensatedRead(struct MS5607UncompensatedValues *uncompValues){
    uint8_t reply[3];
    //ms5607_enableCSB();
    uint8_t msg = MS5607_COMMAND_CONVERT_D1_BASE | Pressure_OSR;
    HAL_I2C_Master_Transmit(_ms5607_i2c_port,ms5607_addr,&msg,1,10);

    if(Pressure_OSR == 0x00)
        HAL_Delay(1);
    else if(Pressure_OSR == 0x02)
        HAL_Delay(2);
    else if(Pressure_OSR == 0x04)
        HAL_Delay(3);
    else if(Pressure_OSR == 0x06)
        HAL_Delay(5);
    else
        HAL_Delay(10);

    //ms5607_disableCSB();
    //ms5607_enableCSB();
    uint8_t msg = MS5607_COMMAND_ADC_READ;
    HAL_I2C_Master_Transmit(_ms5607_i2c_port,ms5607_addr,&msg,1,10);
    HAL_I2C_Master_Receive(_ms5607_i2c_port,ms5607_addr,reply,3,10);
    //ms5607_disableCSB();

    uncompValues->pressure = ((uint32_t)reply[0]<<10) | ((uint32_t) reply[1] << 8) | (uint32_t) reply[2];

    //ms5607_enableCSB();
    uint8_t msg = MS5607_COMMAND_CONVERT_D2_BASE | Temperature_OSR;
    HAL_I2C_Master_Transmit(_ms5607_i2c_port,ms5607_addr,&msg,1,10);

    if(Temperature_OSR == 0x00)
        HAL_Delay(1);
    else if(Temperature_OSR == 0x02)
        HAL_Delay(2);
    else if(Temperature_OSR == 0x04)
        HAL_Delay(3);
    else if(Temperature_OSR == 0x06)
        HAL_Delay(5);
    else
        HAL_Delay(10);

    //ms5607_disableCSB();
    //ms5607_enableCSB();
    uint8_t msg = MS5607_COMMAND_ADC_READ;
    HAL_I2C_Master_Transmit(_ms5607_i2c_port,ms5607_addr,&msg,1,10);
    HAL_I2C_Master_Receive(_ms5607_i2c_port,ms5607_addr,reply,3,10);
    //ms5607_disableCSB();

    /* Assemble the conversion command based on previously set OSR */
    uncompValues->temperature = ((uint32_t) reply[0] << 16) | ((uint32_t) reply[1] << 8) | (uint32_t) reply[2];

}

void MS5607Convert(struct MS5607UncompensatedValues *sample, struct MS5607Readings *value){
    int32_t dT;
    int32_t TEMP;
    int64_t OFF;
    int64_t SENS;

    dT = sample->temperature - ((int32_t) (promData.tref << 8));

    TEMP = 2000 + (((int64_t) dT * promData.tempsens) >> 23);

    OFF = ((int64_t) promData.off << 17) + (((int64_t) promData.tco * dT) >> 6);
    SENS = ((int64_t) promData.sens << 16) + (((int64_t) promData.tcs * dT) >> 7);

    /**/
    if (TEMP < 2000) {
        int32_t T2 = ((int64_t) dT * (int64_t) dT) >> 31;
        int32_t TEMPM = TEMP - 2000;
        int64_t OFF2 = (61 * (int64_t) TEMPM * (int64_t) TEMPM) >> 4;
        int64_t SENS2 = 2 * (int64_t) TEMPM * (int64_t) TEMPM;
        if (TEMP < -1500) {
        int32_t TEMPP = TEMP + 1500;
        int32_t TEMPP2 = TEMPP * TEMPP;
        OFF2 = OFF2 + (int64_t) 15 * TEMPP2;
        SENS2 = SENS2 + (int64_t) 8 * TEMPP2;
        }
        TEMP -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
    }

    value->pressure = ((((int64_t) sample->pressure * SENS) >> 21) - OFF) >> 15;
    value->temperature = TEMP;
}

/* Performs the sensor reading updating the data structures */
void MS5607Update(void){
  MS5607UncompensatedRead(&uncompValues);
  MS5607Convert(&uncompValues, &readings);
}

/* Gets the temperature from the sensor reading */
double MS5607GetTemperatureC(void){
  return (double)readings.temperature/(double)100.0;
}

/* Gets the pressure from the sensor reading */
int32_t MS5607GetPressurePa(void){
  return readings.pressure;
}

/* Sets the OSR for temperature */
void MS5607SetTemperatureOSR(MS5607OSRFactors tOSR){
  Temperature_OSR = tOSR;
}

/* Sets the OSR for pressure */
void MS5607SetPressureOSR(MS5607OSRFactors pOSR){
  Pressure_OSR = pOSR;
}