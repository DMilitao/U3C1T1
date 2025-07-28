#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include <math.h>

#define SENSOR1_ADDR 0x23 // BH1750
#define HIGH_RESO2 0x11
#define CONST_SENSOR1 2.4f
#define DESIRED_LUX 500

#define I2C_PORT i2c0
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1

#define LEDB 12
#define PWM_FREQ 1000       
#define PWM_DIV 4.0f

#define TimeTaskAcquisition 150
#define TimeTaskProcessing 150

uint16_t data;
uint ticks_LEDs;

uint16_t sensor1_read() {
    uint8_t buf[2];

    if(i2c_read_blocking(I2C_PORT, SENSOR1_ADDR, buf, 2, false) == 2){
        return  (buf[0] << 8) | buf[1];
    }

    printf("Error in Sensor 1!\n");
    while(true);
    return 0;
}

void init_pwm(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, PWM_DIV); 
    
    pwm_set_wrap(slice_num, ticks_LEDs);             
    pwm_init(slice_num, &config, true);         
    pwm_set_gpio_level(pin, 0);                 
}

void set_pwm_level(uint pin, double duty_cycle) {
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice_num, ticks_LEDs);
    pwm_set_gpio_level(pin, (uint)(duty_cycle * ticks_LEDs));
}


void init_peripherals(){
    printf("Initializing peripherals...\n");

    ticks_LEDs = (clock_get_hz(clk_sys) / (PWM_FREQ * PWM_DIV)) - 1;
    init_pwm(LEDB);
    set_pwm_level(LEDB, 0);
    
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    uint8_t cmd = HIGH_RESO2;
    if (i2c_write_blocking(I2C_PORT, SENSOR1_ADDR, &cmd, 1, false) < 0) {
        printf("Error in Sensor 1!\n");
        while(true);
    }
    printf("Sensor 1 starting...\n");
}

void i2c_acquisition()
{   
    const TickType_t xTimeTask = pdMS_TO_TICKS(TimeTaskAcquisition); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        data = sensor1_read();
        vTaskDelayUntil(&xLastWakeTime, xTimeTask);
    }
}

void i2c_processing()
{   
    const TickType_t xTimeTask = pdMS_TO_TICKS(TimeTaskProcessing); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    double error;
    double pwm_led;

    while (true) {
        double lux = (double)data / CONST_SENSOR1;

        error = (DESIRED_LUX - lux)/DESIRED_LUX;
        pwm_led += error;
        pwm_led = fmin(fmax(pwm_led, 0), 1.0);
        
        set_pwm_level(LEDB, pwm_led);
        
        printf("Lumin: %.2f | LED value: %.2f\n", lux, pwm_led);
        vTaskDelayUntil(&xLastWakeTime, xTimeTask);
    }
}

int main()
{
    stdio_init_all();

    sleep_ms(5000);

    init_peripherals();

    xTaskCreate(i2c_acquisition, "Acquisition", 256, NULL, 10, NULL);
    xTaskCreate(i2c_processing, "Processing", 256, NULL, 1, NULL);
    vTaskStartScheduler();

    while(1){};
}