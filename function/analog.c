//
// Created by 86136 on 2024/2/28.
//

#include "analog.h"
#include "adc.h"
#include "stm32f4xx_hal_adc_ex.h"

volatile unsigned short adc_dma_result[ADC_NUM];
volatile unsigned short adc_dma_result_sum[ADC_NUM];
volatile unsigned short adc_result[ADC_NUM];

void analog_init() {

  while (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_result, ADC_NUM) !=
         HAL_OK)
    ;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  static unsigned char count = 0;
  count++;
  if (count == 10) {
    for (int i = 0; i < ADC_NUM; i++) {
      adc_result[i] = adc_dma_result_sum[i] / (count - 1);
      adc_dma_result_sum[i] = 0;
    }
    count = 0;
  }
  for (int i = 0; i < ADC_NUM; i++) {
    adc_dma_result_sum[i] += adc_dma_result[i];
  }
}

float get_battery_voltage() { return (float)adc_result[0] * BATTERY_ADC_SCAN; }
float get_potentiometer_voltage() { return (float)adc_result[1] * ADC_SCAN; }

unsigned short get_battery_ADC_value() { return adc_result[0]; }

unsigned short get_potentiometer_ADC_voltage() { return adc_result[1]; }

unsigned short get_CCD1_ADC_value() { return adc_result[2]; }
unsigned short get_CCD2_ADC_value() { return adc_result[3]; }

float get_CCD1_voltage() { return (float)adc_result[2] * ADC_SCAN; }
float get_CCD2_voltage() { return (float)adc_result[3] * ADC_SCAN; }