#ifndef _IMU_SENSOR_H_
#define _IMU_SENSOR_H_


#include "matrix.h"


#define SENSOR_ADDR  ( 0b1101000 )
#define SENSOR_ADDR_R  SENSOR_ADDR
#define SENSOR_ADDR_W  (SENSOR_ADDR | 1)

#define WHO_AM_I  ( 0x75 )

#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define ACCEL_CONFIG_2 0x1D

#define ACCEL_XOUT_H  (0x3B)
#define ACCEL_XOUT_L  (0x3C)
#define ACCEL_YOUT_H  0x3D
#define ACCEL_YOUT_L  0x3E
#define ACCEL_ZOUT_H  0x3F
#define ACCEL_ZOUT_L  0x40

#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48

// NOISY_BITS = round(16 - log2(FS/Noise))
// Noise = Nrms * CrestFactor, CrestFactor = 4 (эмпир.)
// Nrms = PSD * sqrt(ENBW), ENBW - equivalent noise bandwidth
// ENBW = 1.57 * BW,  BM = см. init_IMU_sensor()
#define ACCEL_NOISY_BITS  6 // NOISY_BITS = round(6.28) // PSD = 300 ug/sqrt(Hz)
#define GYRO_NOISY_BITS   4 // NOISY_BITS = round(3.88) // PSD = 0.01 dps/sqrt(Hz)

// DPS - Degrees Per Second
#define LSBtoDPS  1.0 / 131.0


typedef struct _vector vector;
struct _vector
{
    float x;
    float y;
    float z;
};


unsigned short who_am_i();

int read_accel_x();
int read_accel_y();
int read_accel_z();

int read_gyro_x();
int read_gyro_y();
int read_gyro_z();

int init_imu_sensor();


void create_accel_act_vals(float* dst);
void create_accel_dsr_vals(float* dst);
int create_accel_conv_matrix();
void convert_accel_vector(vector* accel_cvtr, vector accel_vtr);

void create_gyro_zero_offset_vector();
void convert_gyro_vector(vector* gyro_cvtr, vector gyro_vtr);

void get_accel_vector(vector* dst);
void get_gyro_vector(vector* dst);


#endif