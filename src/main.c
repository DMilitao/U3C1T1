#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "ssd1306.h"
#include <math.h>

#define SENSOR1_ADDR 0x68 // MPU6050
#define CMD_RESET 0x6B
#define CMD_INIT 0x00
#define CMD_READ 0x3B

#define TH_MAX_INCL 30.0f

#define I2C_PORT i2c0
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1

#define I2C_PORT_OLED i2c1
#define I2C_OLED_SDA_PIN 14
#define I2C_OLED_SCL_PIN 15

#define LEDR 13
#define LEDB 12
#define LEDG 11
#define PWM_FREQ 1000       
#define PWM_DIV 4.0f

#define TimeTaskAcquisition 300.0f
#define TimeTaskProcessing 300.0f

uint8_t data[14];
double pitch, roll;
double AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
const double alpha = 0.9;
uint8_t ssd[ssd1306_buffer_length];
uint ticks_LEDs;

struct render_area frame_area = {
    start_column : 0,
    end_column : ssd1306_width - 1,
    start_page : 0,
    end_page : ssd1306_n_pages - 1
};

void clear_OLED(uint8_t ssd[]){
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, &frame_area);
}

void write_OLED(uint8_t ssd[], char *text[], size_t size_text){
    int y = 0;
    for (uint i = 0; i < size_text; i++)
    {
        ssd1306_draw_string(ssd, 5, y, text[i]);
        y += 8;
    }
    render_on_display(ssd, &frame_area);
}


void sensor1_read(uint8_t *buf) {
    if(i2c_read_blocking(I2C_PORT, SENSOR1_ADDR, buf, 14, false) != 14){
        printf("Error in Sensor 1!\n");
        while(true);
    }
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
    init_pwm(LEDR);
    init_pwm(LEDB);
    init_pwm(LEDG);
    set_pwm_level(LEDR, 0);
    set_pwm_level(LEDB, 0);
    set_pwm_level(LEDG, 0);

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    uint8_t cmd[] = {CMD_RESET, CMD_INIT};
    if (i2c_write_blocking(I2C_PORT, SENSOR1_ADDR, cmd, 2, false) < 0) {
        printf("Error in Sensor 1!\n");
        while(true);
    }

    printf("Sensor 1 starting...\n");
    
    i2c_init(I2C_PORT_OLED, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_OLED_SDA_PIN);
    gpio_pull_up(I2C_OLED_SCL_PIN);

    ssd1306_init();

    calculate_render_area_buffer_length(&frame_area);

    clear_OLED(ssd);
}

void i2c_acquisition()
{   
    const TickType_t xTimeTask = pdMS_TO_TICKS(TimeTaskAcquisition); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        if (i2c_write_blocking(I2C_PORT, SENSOR1_ADDR, (uint8_t[]){CMD_READ}, 1, true) < 0) {
            printf("Error in Sensor 1!\n");
            while(true);
        }
        sensor1_read(data);

        AccX = (double)((int16_t)((data[0] << 8) & 0xFF00) + data[1])/16384.0;
        AccY = (double)((int16_t)((data[2] << 8) & 0xFF00) + data[3])/16384.0;
        AccZ = (double)((int16_t)((data[4] << 8) & 0xFF00) + data[5])/16384.0;
        GyroX = (double)((int16_t)((data[8] << 8) & 0xFF00) + data[9])/131.0;
        GyroY = (double)((int16_t)((data[10] << 8) & 0xFF00) + data[11])/131.0;
        GyroZ = (double)((int16_t)((data[12] << 8) & 0xFF00) + data[13])/131.0;

        float accelRoll = atan2(AccY, sqrt(AccX*AccX + AccZ*AccZ)) * 180.0 / 3.1415;
        float accelPitch = atan2(-AccX, sqrt(AccY*AccY + AccZ*AccZ)) * 180.0 / 3.1415;

        roll = alpha * (roll + GyroX*TimeTaskProcessing/1000) + (1 - alpha)*accelRoll;
        pitch = alpha * (pitch + GyroY*TimeTaskProcessing/1000) + (1 - alpha)*accelPitch;

        vTaskDelayUntil(&xLastWakeTime, xTimeTask);
    }
}

void i2c_processing()
{   
    const TickType_t xTimeTask = pdMS_TO_TICKS(TimeTaskProcessing); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Allocate memory for each line of screen
    char *text[8];
    text[0] = malloc(15);
    text[1] = malloc(15);
    text[2] = malloc(15);
    text[3] = malloc(15);
    text[4] = malloc(15);
    text[5] = malloc(15);
    text[6] = malloc(15);
    text[7] = malloc(15);

    strcpy(text[0], " Monitor");

    while (true) {       
        set_pwm_level(LEDR, fabs(AccX));
        set_pwm_level(LEDB, fabs(AccY));
        set_pwm_level(LEDG, fabs(AccZ));

        if ( fabs(roll) > TH_MAX_INCL || fabs(pitch) > TH_MAX_INCL ) {
            sprintf(text[1], "               ");
            sprintf(text[2], "               ");
            sprintf(text[3], " ALERT  ALERT  ");
            sprintf(text[4], "               ");
            sprintf(text[5], "    REDUCE     ");
            sprintf(text[6], "  INCLINATION  ");
            sprintf(text[7], "               ");
        } else {
            sprintf(text[1], "AccX: %.2f G   ", AccX);
            sprintf(text[2], "AccY: %.2f G   ", AccY);
            sprintf(text[3], "AccZ: %.2f G   ", AccZ);
            sprintf(text[4], "GyrX: %.2f dps ", GyroX);
            sprintf(text[5], "GyrY: %.2f dps ", GyroY);
            sprintf(text[6], "GyrZ: %.2f dps ", GyroZ);
            sprintf(text[7], "R: %.2f P:%.2f ", roll, pitch);

        }

        printf("AccX: %.2f G| AccY: %.2f G| AccZ: %.2f G| GyroX: %.2f dps| GyroY: %.2f dps| GyroZ: %.2f dps| Roll: %.2f d| Pitch: %.2f d|\n", AccX, AccY, AccZ, GyroX, GyroY, GyroZ, roll, pitch);

        write_OLED(ssd, text, sizeof(text)/sizeof(text[0]));
        vTaskDelayUntil(&xLastWakeTime, xTimeTask);
    }
}

int main()
{
    stdio_init_all();

    init_peripherals();

    xTaskCreate(i2c_acquisition, "Acquisition", 256, NULL, 10, NULL);
    xTaskCreate(i2c_processing, "Processing", 256, NULL, 1, NULL);
    vTaskStartScheduler();

    while(1){};
}