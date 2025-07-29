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

#define SENSOR1_ADDR 0x38 // AHT10
#define CMD_RESET 0xBA
#define CMD_INIT 0xE1
#define CMD_START_MEAS1 0xAC
#define CMD_START_MEAS2 0x33
#define CMD_START_MEAS3 0x00

#define TH_MIN_TEMP 20.0f
#define TH_MAX_HUM 70

#define I2C_PORT i2c0
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1

#define I2C_PORT_OLED i2c1
#define I2C_OLED_SDA_PIN 14
#define I2C_OLED_SCL_PIN 15

#define BUZZER_PIN 21
#define LEDR 13
#define PWM_FREQ 1000       
#define PWM_DIV 4.0f

#define TimeTaskAcquisition 150
#define TimeTaskProcessing 150

uint8_t data[6];
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
    if(i2c_read_blocking(I2C_PORT, SENSOR1_ADDR, buf, 6, false) != 6){
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
    set_pwm_level(LEDR, 0);
    init_pwm(BUZZER_PIN);
    set_pwm_level(BUZZER_PIN, 0);
    
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    uint8_t cmd = CMD_RESET;
    if (i2c_write_blocking(I2C_PORT, SENSOR1_ADDR, &cmd, 1, false) < 0) {
        printf("Error in Sensor 1!\n");
        while(true);
    }
    sleep_ms(20);
    cmd = CMD_INIT;
    if (i2c_write_blocking(I2C_PORT, SENSOR1_ADDR, &cmd, 1, false) < 0) {
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
    uint8_t cmd[3] = {CMD_START_MEAS1, CMD_START_MEAS2, CMD_START_MEAS3};

    while (true) {
        sensor1_read(data);

        if (i2c_write_blocking(I2C_PORT, SENSOR1_ADDR, &cmd[0], 3, false) < 0) {
            printf("Error in Sensor 1!\n");
            while(true);
        }
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
    float hum = 0;
    float temp = 0;
    
    while (true) {
        hum = ( (float)(((unsigned long)data[1] << 12) | ((unsigned long)data[2] << 4) | (data[3] >> 4)) / 1048576.0 ) * 100.0;
        temp = ( (float)((((unsigned long)data[3] & 0x0F) << 16) | ((unsigned long)data[4] << 8) | data[5]) / 1048576.0 ) * 200.0 - 50.0;
        
        bool flag_alert = false;
        
        if ( temp < TH_MIN_TEMP ){
            strcpy(text[5], "Alert MIN TEMP");
            flag_alert = true;
        } else {
            strcpy(text[5], "              ");
        }

        if ( hum > TH_MAX_HUM ){
            strcpy(text[7], "Alert MAX HUM");
            flag_alert = true;
        } else {
            strcpy(text[7], "              ");
        }

        printf("Temp: %.2f °C | Hum: %.2f RH | Alert %s\n", temp, hum, (flag_alert ? "ON" : "OFF"));

        if ( flag_alert ) {
            set_pwm_level(LEDR, 0.5);
            set_pwm_level(BUZZER_PIN, 0.5);
        } else {
            set_pwm_level(LEDR, 0);
            set_pwm_level(BUZZER_PIN, 0);
        }

        strcpy(text[0], " Monitor");
        strcpy(text[1], " ");
        sprintf(text[2], "Temp: %.2f C  ", temp);
        sprintf(text[3], " Hum: %.2f RH  ", hum);
        strcpy(text[4], " ");
        strcpy(text[6], " ");
        
        write_OLED(ssd, text, sizeof(text)/sizeof(text[0]));
        vTaskDelayUntil(&xLastWakeTime, xTimeTask);
    }
}

int main()
{
    stdio_init_all();

    sleep_ms(1000);

    init_peripherals();

    xTaskCreate(i2c_acquisition, "Acquisition", 256, NULL, 10, NULL);
    xTaskCreate(i2c_processing, "Processing", 256, NULL, 1, NULL);
    vTaskStartScheduler();

    while(1){};
}