

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nrf_drv_gpiote.h"

#include "game_controller.h"


void game_controller_init(nrf_drv_gpiote_evt_handler_t evt_handler)
{
    uint32_t err_code;

    if(!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    // Configuring pins
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    config.pull = NRF_GPIO_PIN_PULLUP;
    err_code = nrf_drv_gpiote_in_init(LEFT_BTN_1_PIN, &config, evt_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(LEFT_BTN_2_PIN, &config, evt_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(LEFT_BTN_3_PIN, &config, evt_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(LEFT_BTN_4_PIN, &config, evt_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(RIGHT_BTN_1_PIN, &config, evt_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(RIGHT_BTN_2_PIN, &config, evt_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(RIGHT_BTN_3_PIN, &config, evt_handler);
    err_code = nrf_drv_gpiote_in_init(RIGHT_BTN_4_PIN, &config, evt_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(LEFT_BTN_1_PIN, true);
    nrf_drv_gpiote_in_event_enable(LEFT_BTN_2_PIN, true);
    nrf_drv_gpiote_in_event_enable(LEFT_BTN_3_PIN, true);
    nrf_drv_gpiote_in_event_enable(LEFT_BTN_4_PIN, true);
    nrf_drv_gpiote_in_event_enable(RIGHT_BTN_1_PIN, true);
    nrf_drv_gpiote_in_event_enable(RIGHT_BTN_2_PIN, true);
    nrf_drv_gpiote_in_event_enable(RIGHT_BTN_3_PIN, true);
    nrf_drv_gpiote_in_event_enable(RIGHT_BTN_4_PIN, true);
}

