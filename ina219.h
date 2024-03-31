#ifndef _INA219_H_
#define _INA219_H_

#define INA219_ADDR  0x40
//#define INA219_ADDR1  0x40
//#define INA219_ADDR2  0x41

#define CONFIG_REG  0x00
#define SHUNT_VOLTAGE  0x01
#define BUS_VOLTAGE  0x02
#define POWER  0x03
#define CURRENT  0x04
#define CALIB_REG  0x05

#define SHUNT_OHMS  0.1
#define SHUNT_LSB_mV  0.01
#define BUS_LSB_V  0.004

#define CURR_NOISY_BITS  4  // (эмпир.)

typedef struct _ina219 ina219;
struct _ina219
{
    unsigned short addr;
    unsigned int config_reg;
    unsigned int calib_reg;
    double current_LSB;
    double power_LSB;
    float max_shunt_voltage;
    float max_current;
    unsigned short max_bus_voltage;
    float max_power;
};


unsigned int read_config_reg(unsigned short ina219_addr);
int read_shunt_voltage(unsigned short ina219_addr);
int read_bus_voltage(unsigned short ina219_addr);
int read_current(unsigned short ina219_addr);
int read_power(unsigned short ina219_addr);
unsigned int read_calib_reg(unsigned short ina219_addr);

float conv_to_shunt_mV(int shunt_voltage);
float conv_to_bus_V(int bus_voltage);
float conv_to_mA(ina219 obj, int current);
float conv_to_mW(ina219 obj, int power);

int init_ina219(ina219* obj, float max_exp_current, float max_exp_bus_voltage);


#endif