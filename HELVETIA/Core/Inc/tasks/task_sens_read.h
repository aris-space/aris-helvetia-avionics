#ifndef INC_TASKS_TASK_SENS_READ_H_
#define INC_TASKS_TASK_SENS_READ_H_

/* Includes */

#include "Util/mutex.h"
#include "Util/util.h"
#include "main.h"
#include "Util/logging_util.h"

#define SENSOR_READ_FREQUENCY 100

static void read_data_sb(sb_data_t *sb1, sb_data_t *sb2);
#ifdef
static void read_data_usb();
#endif
static uint32_t calculate_checksum_sb(sb_data_t *sb_data);

//sensor board 1
extern custom_mutex_t sb1_mutex;
extern I2C_HandleTypeDef i2c_sb1;
extern sb_data_t sb1_global;

//sensor board 2
extern custom_mutex_t sb2_mutex;
extern I2C_HandleTypeDef i2c_sb2;
extern sb_data_t sb2_global;




#endif