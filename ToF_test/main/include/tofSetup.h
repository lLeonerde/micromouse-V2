#ifndef TOFSETUP_H_
#define TOPFSETUP_H_

#include "vl53l0x.h"

void i2c_master_init();
void configure_gpio(uint8_t xshut_pin);
void setupSensor(vl53l0x_t *sensor, uint8_t xshut_pin, uint8_t new_address);
#endif