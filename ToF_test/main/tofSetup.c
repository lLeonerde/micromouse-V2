#include "pinout.h"

vl53l0x_t *sensor1, *sensor2, *sensor3,*sensor4,*sensor5;


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
    vTaskDelay(500/portTICK_PERIOD_MS);
}
// Setup sensor function
void setupSensor(vl53l0x_t *sensor, uint8_t xshut_pin, uint8_t new_address) {
    deactivateSensor(xshut_pin);    
    vTaskDelay(20 / portTICK_PERIOD_MS);   
    activateSensor(xshut_pin);
    vTaskDelay(20 / portTICK_PERIOD_MS);    
    char* test = vl53l0x_init(sensor); 
    if (test == ESP_OK) {
        vl53l0x_setAddress(sensor, new_address); 
        ESP_LOGI(__func__,"Sensor %d configured for 0x%02X.\n", xshut_pin, new_address);
    } else {
        ESP_LOGE(__func__,"Error sensor pin: %sd\n", test);
    }
}


void setupToF(){
    i2c_master_init();

    configure_gpio(XSHUT_PIN_SENSOR_1);
    configure_gpio(XSHUT_PIN_SENSOR_2);
    configure_gpio(XSHUT_PIN_SENSOR_3);
    configure_gpio(XSHUT_PIN_SENSOR_4);
    configure_gpio(XSHUT_PIN_SENSOR_5);

    uint8_t io_2v8 = 1; // 1 for I/O 2.8V
    uint8_t initial_address = 0x29; // Default address

    // Config/create each sensor
    //sensor1 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_1, initial_address, io_2v8);
    sensor2 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_2, initial_address, io_2v8);
    sensor3 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_3, initial_address, io_2v8);
    sensor4 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_4, initial_address, io_2v8);
    sensor5 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_5, initial_address, io_2v8);

    if (sensor1 == NULL || sensor2 == NULL || sensor3 == NULL || sensor4 == NULL|| sensor5 == NULL) {
        ESP_LOGE(__func__,"Error configuring 1 or more sensors\n");
        //return;
    }
    vTaskDelay(1/portTICK_PERIOD_MS);
    //starting them
    setupSensor(sensor1, XSHUT_PIN_SENSOR_1, 0x30);
    setupSensor(sensor2, XSHUT_PIN_SENSOR_2, 0x31);
    vTaskDelay(1/portTICK_PERIOD_MS);

    setupSensor(sensor3, XSHUT_PIN_SENSOR_3, 0x32);
    vTaskDelay(1/portTICK_PERIOD_MS);
    setupSensor(sensor4, XSHUT_PIN_SENSOR_4, 0x33);
    vTaskDelay(1/portTICK_PERIOD_MS);
    setupSensor(sensor5, XSHUT_PIN_SENSOR_5, 0x34);
    vTaskDelay(1/portTICK_PERIOD_MS);

    vl53l0x_startContinuous(sensor1, 2);
    vl53l0x_startContinuous(sensor2, 2);
    vl53l0x_startContinuous(sensor3, 2);
    vl53l0x_startContinuous(sensor4, 2);
    vl53l0x_startContinuous(sensor5, 2);
}