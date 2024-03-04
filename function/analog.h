//
// Created by 86136 on 2024/2/28.
//

#ifndef MECANUM_CART_ANALOG_H
#define MECANUM_CART_ANALOG_H

#define ADC_NUM 4
#define ADC_SCAN (3.3f / 4095.0f)
#define BATTERY_ADC_SCAN (ADC_SCAN * 11.0f)

void analog_init();
float get_battery_voltage();
float get_potentiometer_voltage();
unsigned short get_battery_ADC_value();
unsigned short get_potentiometer_ADC_voltage();

unsigned short get_CCD1_ADC_value();
unsigned short get_CCD2_ADC_value();

float get_CCD1_voltage();
float get_CCD2_voltage();
#endif // MECANUM_CART_ANALOG_H
