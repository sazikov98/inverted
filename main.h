#ifndef _MAIN_H_
#define _MAIN_H_


#include "filter.h"
#include "pid_ctrl.h"
#include "imu_sensor.h"
#include "ina219.h"


// RTPmS - Rising Triggers Per Seconds
// RPM - Revolutions Per Minute
#define RTPStoRPM 60.0 / 210.0

#define PI  3.141592
#define RADtoDEG  180.0/PI
#define DEGtoRAD  PI/180.0

#define MOTORS_OFF  GPIOC_ODR.B14 = 0
#define MOTORS_ON   GPIOC_ODR.B14 = 1
#define LED_OFF  GPIOC_ODR.B13 = 1
#define LED_ON   GPIOC_ODR.B13 = 0


typedef struct _encoder encoder;
struct _encoder
{
    unsigned short Gray_code;
    int diff_cntr;
};

typedef struct _wheel wheel;
struct _wheel
{
    filter_data speed_fd;
    filter_data current_fd;
    encoder enc;
    float speed;
    ina219 ina;
    float current;
    pid_obj speed_ctrl;
    pid_obj current_ctrl;
    pid_obj duty_ctrl;
};


#endif