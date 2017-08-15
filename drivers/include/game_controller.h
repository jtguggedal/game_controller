#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "nrf_drv_gpiote.h"

#define LEFT_BTN_1_PIN              NRF_GPIO_PIN_MAP(0, 22)
#define LEFT_BTN_2_PIN              NRF_GPIO_PIN_MAP(0, 20)
#define LEFT_BTN_3_PIN              NRF_GPIO_PIN_MAP(0, 17)
#define LEFT_BTN_4_PIN              NRF_GPIO_PIN_MAP(0, 25)

#define LEFT_REAR_BTN_1             NRF_GPIO_PIN_MAP(0, 24)
#define LEFT_REAR_BTN_2             NRF_GPIO_PIN_MAP(1, 00)

#define JS_LEFT_X                   NRF_SAADC_INPUT_AIN3
#define JS_LEFT_Y                   NRF_SAADC_INPUT_AIN2
#define JS_LEFT_BTN                 NRF_GPIO_PIN_MAP(1, 1)

#define RIGHT_BTN_1_PIN             NRF_GPIO_PIN_MAP(1, 14)
#define RIGHT_BTN_2_PIN             NRF_GPIO_PIN_MAP(0, 3)
#define RIGHT_BTN_3_PIN             NRF_GPIO_PIN_MAP(0, 12)
#define RIGHT_BTN_4_PIN             NRF_GPIO_PIN_MAP(0, 2)

#define RIGHT_REAR_BTN_1            NRF_GPIO_PIN_MAP(0, 24)
#define RIGHT_REAR_BTN_2            NRF_GPIO_PIN_MAP(1, 00)

#define JS_RIGHT_X                  NRF_SAADC_INPUT_AIN7
#define JS_RIGHT_Y                  NRF_SAADC_INPUT_AIN5
#define JS_RIGHT_BTN                NRF_GPIO_PIN_MAP(0, 19)




#define RGB_R       NRF_GPIO_PIN_MAP(1, 13)
#define RGB_G       NRF_GPIO_PIN_MAP(1, 11)
#define RGB_B       NRF_GPIO_PIN_MAP(1, 10)

void game_controller_init(nrf_drv_gpiote_evt_handler_t evt_handler);

