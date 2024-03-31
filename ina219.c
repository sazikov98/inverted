#include "ina219.h"


static void write_reg(unsigned short ina219_addr, unsigned short reg, unsigned int val)
{
      unsigned short buf[3];
      buf[0] = reg;
      buf[1] = (val >> 8) & 0x00FF;
      buf[2] = (val >> 0) & 0x00FF;

      I2C2_Start();
      I2C2_Write(ina219_addr, &buf, 3, END_MODE_STOP);
}

static int read_reg(unsigned short ina219_addr, unsigned short reg)
{
    unsigned short buf[2];

    I2C2_Start();
    I2C2_Write(ina219_addr, &reg, 1, END_MODE_RESTART);
    I2C2_Read(ina219_addr, &buf, 2, END_MODE_STOP);

    return ((buf[0] << 8) | buf[1]);
}


unsigned int read_config_reg(unsigned short ina219_addr)
{
    return read_reg(ina219_addr, CONFIG_REG);
}

int read_shunt_voltage(unsigned short ina219_addr)
{
    return read_reg(ina219_addr, SHUNT_VOLTAGE);
}

int read_bus_voltage(unsigned short ina219_addr)
{
    return read_reg(ina219_addr, BUS_VOLTAGE) >> 3;
}

int read_current(unsigned short ina219_addr)
{
    int val = read_reg(ina219_addr, CURRENT);
    return ((val >> CURR_NOISY_BITS) + 1 * (val < 0)) << CURR_NOISY_BITS;
}

int read_power(unsigned short ina219_addr)
{
    return read_reg(ina219_addr, POWER);
}

unsigned int read_calib_reg(unsigned short ina219_addr)
{
    return read_reg(ina219_addr, CALIB_REG);
}


float conv_to_shunt_mV(int shunt_voltage)
{
    return shunt_voltage * SHUNT_LSB_mV;
}

float conv_to_bus_V(int bus_voltage)
{
    return bus_voltage * BUS_LSB_V;
}

float conv_to_mA(ina219 obj, int current)
{
    return current * obj.current_LSB * 1000.0;
}

float conv_to_mW(ina219 obj, int power)
{
    return power * obj.power_LSB * 1000.0;
}


double round_to_N_sd(double num, unsigned short N)
{
    unsigned short i = 0, cntr = 0;
    while (i < N)
    {
        num *= 10;
        if ((int)num > 0)
            i++;
        cntr++;
    }
    return (double)(((unsigned int)num + 1) / pow(10, cntr));
}

double calc_current_LSB(unsigned short* digit_capacity, float max_exp_current)
{
    double min_current_LSB, current_LSB;
    float cal;

    while (*digit_capacity >= 12)
    {
        min_current_LSB = max_exp_current / ((1 << *digit_capacity) - 1);
        current_LSB = round_to_N_sd(min_current_LSB, 3);
        cal = 0.04096 / (current_LSB * SHUNT_OHMS);
        if (cal < 32767)
            return current_LSB;
        *digit_capacity -= 1;
    }

    return 0;
}

unsigned short calc_PGA_code(float max_exp_shunt_voltage)
{
    unsigned short PGA, PGA_code = 0;

    PGA = (unsigned short) (max_exp_shunt_voltage / 0.04);
    while (PGA > 0)
    {
        PGA >>= 1;
        PGA_code++;
    }

   return PGA_code;
}

unsigned short calc_BRNG_code(float max_exp_bus_voltage)
{
    unsigned short BRNG_code = 0;

    if (max_exp_bus_voltage > 16)
        BRNG_code = 1;

    return BRNG_code;
}

int init_ina219(ina219* obj, float max_exp_current, float max_exp_bus_voltage)
{
    float max_exp_shunt_voltage;
    unsigned short PGA_code, BRNG_code;

    unsigned short digit_capacity = 15;
    float cal;

    if (max_exp_current <= 0 || max_exp_current > (0.32/SHUNT_OHMS))
        return 1;

    if (max_exp_bus_voltage <= 0 || max_exp_bus_voltage > 32)
        return 2;

    max_exp_shunt_voltage = max_exp_current * SHUNT_OHMS;
    PGA_code = calc_PGA_code(max_exp_shunt_voltage);
    BRNG_code = calc_BRNG_code(max_exp_bus_voltage);
    
    // Configuration Register
    // 0b RST — BRNG PG  BADC SADC MODE
    // 0b  0  0  0   00  0000 0000 000
    // ADC: 64 semps, 34,05 ms; MODE: default
    // 0b0000 0111 0111 0111 = 0x0777
    obj->config_reg = 0x0777 | (PGA_code << 11) | (BRNG_code << 13);

    obj->current_LSB = calc_current_LSB(&digit_capacity, max_exp_current);
    if (obj->current_LSB == 0)
        return 3;
    cal = 0.04096 / (obj->current_LSB * SHUNT_OHMS);
    obj->calib_reg = (unsigned int)cal; // bit0 is not used
    obj->power_LSB = 20 * obj->current_LSB;

    obj->max_current = obj->current_LSB * ((1 << digit_capacity) - 1);
    obj->max_shunt_voltage = obj->max_current * SHUNT_OHMS;
    obj->max_bus_voltage = 16 + 16 * BRNG_code;
    obj->max_power = obj->max_current * obj->max_bus_voltage;

    write_reg(obj->addr, CONFIG_REG, obj->config_reg);
    write_reg(obj->addr, CALIB_REG, obj->calib_reg);
    Delay_us(100);

    return 0;
}