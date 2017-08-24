#include <stdint.h>
#include <stdbool.h>

#include "nrf_drv_saadc.h"
#include "voltage_monitor.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "nrf_log.h"


static nrf_saadc_value_t       	    m_buffer_pool[2][SAMPLES_IN_MONITOR_BUFFER];
bool * low_voltage_flat_ptr;
static nrf_ppi_channel_t       	    m_ppi_channel;

static void timer_handler(nrf_timer_event_t event_type, void* p_context) {}

static void saadc_sampling_event_init(nrf_drv_timer_t * m_timer)
{
    ret_code_t err_code;
    err_code = nrf_drv_ppi_init();
    //APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    
    err_code = nrf_drv_timer_init(m_timer, &cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 1000ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(m_timer, VOLTAGE_MONITOR_TIMER_INTERVAL);
    nrf_drv_timer_extended_compare(m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
    nrf_drv_timer_enable(m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(m_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_event_addr = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, saadc_sample_event_addr);
    APP_ERROR_CHECK(err_code);
}

static void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}


void voltage_monitor_cb(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t          err_code;
        nrf_saadc_value_t   vdd_result;
        uint16_t            batt_lvl_in_milli_volts;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_MONITOR_BUFFER);
        APP_ERROR_CHECK(err_code);

        
        vdd_result = p_event->data.done.p_buffer[0];
        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(vdd_result) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;
        
        if(batt_lvl_in_milli_volts < VOLTAGE_MONITOR_THRESHOLD) 
        {
            *low_voltage_flat_ptr = true;
        }
        else
        {
            *low_voltage_flat_ptr = false;
        }
        
        #if VOLTAGE_MONITOR_PRINT_VDD_MV
            NRF_LOG_RAW_INFO("Voltage: %d\n", batt_lvl_in_milli_volts);
        #endif
    }
}

void voltage_monitor_init(voltage_monitor_cfg_t * config)
{
    uint32_t err_code;
    nrf_saadc_channel_config_t saadc_config;
    low_voltage_flat_ptr = config->low_voltage_flag;
    
    saadc_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    saadc_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    saadc_config.gain       = NRF_SAADC_GAIN1_6;
    saadc_config.reference  = NRF_SAADC_REFERENCE_INTERNAL;
    saadc_config.acq_time   = NRF_SAADC_ACQTIME_10US;
    saadc_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED; 
    saadc_config.burst      = NRF_SAADC_BURST_DISABLED,  
    saadc_config.pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_VDD);
    saadc_config.pin_n      = NRF_SAADC_INPUT_DISABLED;
    

    err_code = nrf_drv_saadc_channel_init(config->saadc_channel, &saadc_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_MONITOR_BUFFER);
    APP_ERROR_CHECK(err_code);
    
    //err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_MONITOR_BUFFER);
    APP_ERROR_CHECK(err_code);
    
    saadc_sampling_event_init(config->timer_instance);
    saadc_sampling_event_enable();
}



