#include "imu_sensor.h"


static void write_reg(unsigned short reg, unsigned short val)
{
      unsigned short buf[2];
      buf[0] = reg;
      buf[1] = val;
      
      I2C1_Start();
      I2C1_Write(SENSOR_ADDR, &buf, 2, END_MODE_STOP);
}

static unsigned short read_reg(unsigned short reg)
{
    unsigned short res;

    I2C1_Start();
    I2C1_Write(SENSOR_ADDR, &reg, 1, END_MODE_RESTART);
    I2C1_Read(SENSOR_ADDR, &res, 1, END_MODE_STOP);

    return res;
}

unsigned short who_am_i()
{
    return read_reg(WHO_AM_I);
}


// Разрядность АЦП - 16 бит

int read_accel_x()
{
    int val = (read_reg(ACCEL_XOUT_H) << 8) | (read_reg(ACCEL_XOUT_L));
    return ((val >> ACCEL_NOISY_BITS) + 1 * (val < 0)) << ACCEL_NOISY_BITS;
}

int read_accel_y()
{
    int val = (read_reg(ACCEL_YOUT_H) << 8) | (read_reg(ACCEL_YOUT_L));
    return ((val >> ACCEL_NOISY_BITS) + 1 * (val < 0)) << ACCEL_NOISY_BITS;
}

int read_accel_z()
{
    int val = (read_reg(ACCEL_ZOUT_H) << 8) | (read_reg(ACCEL_ZOUT_L));
    return ((val >> ACCEL_NOISY_BITS) + 1 * (val < 0)) << ACCEL_NOISY_BITS;
}

int read_gyro_x()
{
    int val = (read_reg(GYRO_XOUT_H) << 8) | (read_reg(GYRO_XOUT_L));
    return ((val >> GYRO_NOISY_BITS) + 1 * (val < 0)) << GYRO_NOISY_BITS;
}

int read_gyro_y()
{
    int val = (read_reg(GYRO_YOUT_H) << 8) | (read_reg(GYRO_YOUT_L));
    return ((val >> GYRO_NOISY_BITS) + 1 * (val < 0)) << GYRO_NOISY_BITS;
}

int read_gyro_z()
{
    int val = (read_reg(GYRO_ZOUT_H) << 8) | (read_reg(GYRO_ZOUT_L));
    return ((val >> GYRO_NOISY_BITS) + 1 * (val < 0)) << GYRO_NOISY_BITS;
}

int init_imu_sensor()
{
    write_reg(SMPLRT_DIV, 0x1F);  // ODR = 31.25 Hz
    write_reg(CONFIG, 0x06);  // GYRO_BW = 5 Hz, Delay = 33.48 ms
    write_reg(ACCEL_CONFIG_2, 0x05);  // ACCEL_BW = 10 Hz, Delay = 35.70 ms

    if (create_accel_conv_matrix())
        return 1;
    create_gyro_zero_offset_vector();

    return 0;
}


float accel_conv_mtx[9];
vector gyro_offset_vtr;

void create_accel_act_vals(float* dst)
{
    vector src[3];

    src[0].x = -16384.0;
    src[0].y = 384.0;
    src[0].z = 192.0;

    src[1].x = 0.0;
    src[1].y = 16576.0;
    src[1].z = 128.0;

    src[2].x = 192.0;
    src[2].y = 64.0;
    src[2].z = 16640.0;

    trans(dst, (float*)src);
}

void create_accel_dsr_vals(float* dst)
{
    vector src[3];

    src[0].x = -16320.0;
    src[0].y = 0.0;
    src[0].z = 0.0;

    src[1].x = 0.0;
    src[1].y = 16320.0;
    src[1].z = 0.0;

    src[2].x = 0.0;
    src[2].y = 0.0;
    src[2].z = 16320.0;

    trans(dst, (float*)src);
}

int create_accel_conv_matrix()
{
    float actual_vals[9], desired_vals[9];
    float temp[9];

    create_accel_act_vals(actual_vals);
    create_accel_dsr_vals(desired_vals);

    if (invert(temp, actual_vals) > 0)
        return 1;
    mul_3x3_3x3(accel_conv_mtx, desired_vals, temp);

    return 0;
}

void convert_accel_vector(vector* accel_cvtr, vector accel_vtr)
{
    mul_3x3_3x1((float*)accel_cvtr, accel_conv_mtx, (float*)&accel_vtr);
}


void create_gyro_zero_offset_vector()
{
    gyro_offset_vtr.x = 176.0;
    gyro_offset_vtr.y = 112.0;
    gyro_offset_vtr.z = 16.0;
}

void convert_gyro_vector(vector* gyro_cvtr, vector gyro_vtr)
{
    gyro_cvtr->x = gyro_vtr.x - gyro_offset_vtr.x;
    gyro_cvtr->y = gyro_vtr.y - gyro_offset_vtr.y;
    gyro_cvtr->z = gyro_vtr.z - gyro_offset_vtr.z;
}

void get_accel_vector(vector* dst)
{
    vector temp;

    temp.x = (float)read_accel_x();
    temp.y = (float)read_accel_y();
    temp.z = (float)read_accel_z();

    convert_accel_vector(dst, temp);
}

void get_gyro_vector(vector* dst)
{
    vector temp;

    temp.x = (float)read_gyro_x();
    temp.y = (float)read_gyro_y();
    temp.z = (float)read_gyro_z();

    convert_gyro_vector(dst, temp);
}