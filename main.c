#include "main.h"


// Буферы для обмена данными:
// USB
char USB_rBuf[64], USB_wBuf[64];
// UART
char UART_rBuf[64], UART_tBuf[64];

bool is_next_iteration = false;
wheel wheel1, wheel2;

unsigned short i = 0;
void TIM4_int() iv IVT_INT_TIM4 ics ICS_AUTO
{
    TIM4_SR.UIF = 0; // Сбарсываем флаг прерывания!!!
    is_next_iteration = true;
}

void USB_int() iv IVT_INT_USB_LP_CAN_RX0 ics ICS_AUTO
{
    USB_Interrupt_Proc();
}

void UART1_int() iv IVT_INT_USART1 ics ICS_AUTO
{
//  pass
}

void I2C1_TimeoutCallback(char errorCode) 
{
//  pass
}

void I2C2_TimeoutCallback(char errorCode) 
{
//  pass
}

void calc_enc_cntr(encoder* obj, bool port1_bit, bool port2_bit)
{
    unsigned short Gray_code = 0;

    Gray_code |= (port1_bit << 0) | (port2_bit << 1);
    if (abs(Gray_code - obj->Gray_code) == 1)
        obj->diff_cntr += 1;
    else
        obj->diff_cntr -= 1;
    obj->Gray_code = Gray_code;
}

void GPIOB_int() iv IVT_INT_EXTI15_10 ics ICS_AUTO
{
//    unsigned short Gray_code = 0;
    
    if (EXTI_PR.B12 || EXTI_PR.B13)
    {
        // the bit is cleared by writing a 1 into the bit
        EXTI_PR.B12 = 1;
        EXTI_PR.B13 = 1;
        
        calc_enc_cntr(&(wheel2.enc), GPIOB_IDR.B12, GPIOB_IDR.B13);
    }

    if (EXTI_PR.B14 || EXTI_PR.B15)
    {
        // the bit is cleared by writing a 1 into the bit
        EXTI_PR.B14 = 1;
        EXTI_PR.B15 = 1;
        
        calc_enc_cntr(&(wheel1.enc), GPIOB_IDR.B14, GPIOB_IDR.B15);
    }
}

void init_uart()
{
    UART1_Init_Advanced(9600, _UART_8_BIT_DATA, _UART_NOPARITY,
        _UART_ONE_STOPBIT, &_GPIO_MODULE_USART1_PA9_10);
    NVIC_IntEnable(IVT_INT_USART1);  // Задаём вектор прерывания UART1
    USART1_CR1bits.RXNEIE = 1;  // Разрешаем прерывание по приёму байта
    memset(UART_rBuf, 0, sizeof(UART_rBuf));
    memset(UART_tBuf, 0, sizeof(UART_tBuf));
}

void init_usb_hid()
{
    HID_Enable(&USB_rBuf, &USB_wBuf);
    memset(USB_rBuf, 0, sizeof(USB_rBuf));
    memset(USB_wBuf, 0, sizeof(USB_wBuf));
}

void set_sample_rate(unsigned short T_ms)
{
    // TIM4
    RCC_APB1ENR.TIM4EN = 1;  // Подаём тактирование от шины  APB1 на таймер 4
    TIM4_CR1.CEN = 0;  // Отключаем таймер 4
    TIM4_PSC = 35999;  // Устанавливаем прделитель таймера
    TIM4_ARR = T_ms - 1;  // Устанавливаем значение таймера, при котором
    // произойдёт генерация прерывания и обнуление таймера
    NVIC_IntEnable(IVT_INT_TIM4);  // Задаём вектор прерывания по таймеру
    TIM4_DIER.UIE = 1;  // Разрешаем таймеру генерировать прерывание
    TIM4_CR1.CEN = 1;  // Включаем таймер
}

void init_motor_ctrl()
{
    // GPIO
    GPIO_Config(&GPIOC_BASE, _GPIO_PINMASK_14, _GPIO_CFG_DIGITAL_OUTPUT);
    MOTORS_OFF;
    
    GPIO_Config(&GPIOA_BASE, _GPIO_PINMASK_1, _GPIO_CFG_DIGITAL_OUTPUT);
    GPIO_Config(&GPIOB_BASE, _GPIO_PINMASK_1, _GPIO_CFG_DIGITAL_OUTPUT);
    GPIOA_ODR.B1 = 0;
    GPIOB_ODR.B1 = 0;

    // TIM1, TIM2 (PWM)
    GPIO_Alternate_Function_Enable(&_GPIO_MODULE_TIM2_CH3_PA2);  // вкл. альтер. функцию на PA2
    RCC_APB1ENRbits.TIM2EN = 1;  // подаем тактирование на TIM2
    TIM2_PSC = 0;  // предделитель TIM2
    TIM2_ARR = 999;  // 36 MHz / (999+1) = 36 kHz - PWM frequency
    TIM2_CCR3 = 0;  // скважность (диапазон: от 0 до TIM2_ARR)
    TIM2_CCMR2_Outputbits.OC3M = 0b110;  // режим "прямой ШИМ"
    TIM2_CCERbits.CC3E = 1;  // вкл. CH3
    TIM2_CR1bits.CEN = 1;  //  вкл. TIM2

    GPIO_Alternate_Function_Enable(&_GPIO_MODULE_TIM3_CH3_PB0);
    RCC_APB1ENRbits.TIM3EN = 1;
    TIM3_PSC = 0;
    TIM3_ARR = 999;
    TIM3_CCR3 = 0;
    TIM3_CCMR2_Outputbits.OC3M = 0b110;
    TIM3_CCERbits.CC3E = 1;
    TIM3_CR1bits.CEN = 1;
}

void init_encs_GPIO()
{
    GPIO_Set_Pin_Mode(&GPIOB_BASE,
        _GPIO_PINMASK_12 | _GPIO_PINMASK_13 | _GPIO_PINMASK_14 | _GPIO_PINMASK_15,
        _GPIO_CFG_MODE_INPUT | _GPIO_CFG_PULL_UP);  //Set GPIOB 12 as digital inputs
    AFIO_EXTICR4 |= 0x1111;  // Select PB lines for EXTIx external interrupt
    EXTI_RTSR |= 0xF << 12;  // Select rising edge interrupt
    EXTI_IMR |= 0xF << 12;  // Unmask bits 12-15 for interrupt on this lines
    NVIC_IntEnable(IVT_INT_EXTI15_10);  // Enable NVIC interface
}

void init_led()
{
    GPIO_Config(&GPIOC_BASE, _GPIO_PINMASK_13, _GPIO_CFG_DIGITAL_OUTPUT);
    LED_ON;
}

void init_wheel_obj(wheel* obj)
{
    static unsigned short cnt = 0;

    init_filter_data(&(obj->speed_fd));
    init_filter_data(&(obj->current_fd));

    obj->enc.Gray_code = 0b11;
    obj->enc.diff_cntr = 0;

    obj->ina.addr = INA219_ADDR + cnt;

    init_pid_obj(&(obj->speed_ctrl));
    init_pid_obj(&(obj->current_ctrl));
    init_pid_obj(&(obj->duty_ctrl));

    cnt++;
}

bool get_accel_angle_deg(float* dst, vector src)
{
    if ( abs(src.y) + abs(src.z) > abs(src.x) )
    {
        *dst = atan2(src.z, src.y) * RADtoDEG;
        return true;
    }
    else
        return false;
}

float get_speed_rpm(wheel* obj, float dt)
{
    float diff_cntr;
    diff_cntr = obj->enc.diff_cntr;
    obj->enc.diff_cntr = 0;

    diff_cntr = median_filter(&(obj->speed_fd), diff_cntr);
    diff_cntr = AB_filter(&(obj->speed_fd), diff_cntr, 3);

    return ((float)diff_cntr) / dt * RTPStoRPM;
}

float get_current_mA(wheel* obj)
{
    int current;

    current = read_current(obj->ina.addr);
    current = median_filter(&(obj->current_fd), current);
    current = AB_filter(&(obj->current_fd), current, 3);

    return conv_to_mA(obj->ina, current);
}

int main()
{
    float dt = 0.05;

    vector accel_vtr, gyro_vtr;
    float accel_ang, gyro_dang, angle;

    init_wheel_obj(&wheel1);
    init_wheel_obj(&wheel2);

    set_sample_rate((unsigned short)(dt * 1000));
    init_motor_ctrl();
    init_led();

    I2C1_Init();
    I2C1_SetTimeoutCallback(1000, I2C1_TimeoutCallback);
    I2C2_Init();
    I2C2_SetTimeoutCallback(1000, I2C2_TimeoutCallback);
    init_usb_hid();
    init_uart();
    Delay_ms(100);

    if (init_imu_sensor())
        return 1;
    init_encs_GPIO();
    if (init_ina219(&(wheel1.ina), 0.3, 10.0))
        return 1;
    if (init_ina219(&(wheel2.ina), 0.3, 10.0))
        return 1;

    EnableInterrupts();  // Enable global interrupt
    
    while (1)
    {
        if (is_next_iteration)
        {
            is_next_iteration = false;

            get_accel_vector(&accel_vtr);
            get_gyro_vector(&gyro_vtr);

            if (!get_accel_angle_deg(&accel_ang, accel_vtr))
                continue;

            gyro_dang = (gyro_vtr.x * LSBtoDPS) * dt;

            angle = compl_filter(accel_ang, gyro_dang, 0.1);  // эмпир.

            wheel1.speed = get_speed_rpm(&wheel1, dt);
            wheel2.speed = get_speed_rpm(&wheel2, dt);

            wheel1.current = get_current_mA(&wheel1);
            wheel2.current = get_current_mA(&wheel2);

        }
    }

    return 0;
}