#include "pinout.h"



void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_HOST, &conf);
    i2c_driver_install(I2C_HOST, conf.mode, 0, 0, 0);
    i2c_set_timeout (I2C_HOST, 80000);       // Clock stretching
    i2c_filter_enable (I2C_HOST, 5);
}

void activateSensor(uint8_t xshut_pin) {
    gpio_set_level(xshut_pin, 1);  
    vTaskDelay(10 / portTICK_PERIOD_MS); 
    ESP_LOGI(__func__,"sensor pin %d activated.\n", xshut_pin);
}

void deactivateSensor(uint8_t xshut_pin) {
    gpio_set_level(xshut_pin, 0);  
    ESP_LOGI(__func__,"sensor pin %d deactivated.\n", xshut_pin);
}


// I2C address change
void setSensorAddress(vl53l0x_t *sensor, uint8_t xshut_pin, uint8_t new_address) {
    deactivateSensor(xshut_pin);   
    vTaskDelay(10 / portTICK_PERIOD_MS);  
    activateSensor(xshut_pin);  
    vl53l0x_init(sensor);       
    vl53l0x_setAddress(sensor, new_address); 
    ESP_LOGI(__func__,"sensor pin %d configured for 0x%02X.\n", xshut_pin, new_address);
}



void configure_gpio(uint8_t xshut_pin) {
    gpio_reset_pin(xshut_pin); 
    gpio_set_direction(xshut_pin, GPIO_MODE_OUTPUT); 
    deactivateSensor(xshut_pin); 
}
// Setup sensor function
void setupSensor(vl53l0x_t *sensor, uint8_t xshut_pin, uint8_t new_address) {
    deactivateSensor(xshut_pin);    
    vTaskDelay(10 / portTICK_PERIOD_MS);   
    activateSensor(xshut_pin);   
    if (vl53l0x_init(sensor) == ESP_OK) {
        vl53l0x_setAddress(sensor, new_address); 
        ESP_LOGI(__func__,"Sensor %d configured for 0x%02X.\n", xshut_pin, new_address);
    } else {
        ESP_LOGE(__func__,"Error sensor pin: %d\n", xshut_pin);
    }
}

