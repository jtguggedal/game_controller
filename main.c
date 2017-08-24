/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "ble_advdata.h"
#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "app_timer.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_delay.h"

#include "game_controller.h"


#define APP_BLE_CONN_CFG_TAG    1                                       /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   1                                       /**< Application's BLE observer priority. You shoulnd't need to modify this value. */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define SCAN_INTERVAL           0x00A0                                  /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0050                                  /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT            0x0000                                  /**< Timout when scanning. 0x0000 disables timeout. */


#define CONN_INTERVAL_DEFAULT   (uint16_t)(MSEC_TO_UNITS(7.5, UNIT_1_25_MS))    /**< Default connection interval used at connection establishment by central side. */

#define CONN_INTERVAL_MIN       MSEC_TO_UNITS(7.5, UNIT_1_25_MS)         /**< Determines minimum connection interval in millisecond. */
#define CONN_INTERVAL_MAX       MSEC_TO_UNITS(20, UNIT_1_25_MS)         /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY           0                                       /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Determines supervision time-out in units of 10 millisecond. */

#define UUID16_SIZE             2                                       /**< Size of 16 bit UUID */
#define UUID32_SIZE             4                                       /**< Size of 32 bit UUID */
#define UUID128_SIZE            16                                      /**< Size of 128 bit UUID */

#define UUID_TO_CONNECT_TO      0x0001                


#define NAME_RED_CAR            "Nordic_League_Red"
#define NAME_BLUE_CAR           "Nordic_League_Blue"
#define NAME_DEMO_CAR           "Nordic_League_Demo"
#define NAME_BALANCER           "nRF Balancer"

#define DEFAULT_DEVICE_NAME     NAME_RED_CAR


// SAADC defines
#define SAMPLES_IN_BUFFER 		            5
#define SAADC_TIMER_INTERVAL	            15
#define PRINT_SAADC_VALUES                  0

#define SAADC_LEFT_X    		            0
#define SAADC_LEFT_Y        	            1
#define SAADC_RIGHT_X                       2
#define SAADC_RIGHT_Y                       3

#define BATTERY_VOLTAGE_MONITOR_INTERVAL    70
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS       600                                     /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION        6                                       /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS      0                                     /**< Typical forward voltage drop of the diode . */
#define ADC_RES_10BIT                       1024                                    /**< Maximum digital value for 10-bit ADC conversion. */
#define VDD_MIN_THRESHOLD                   3200

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 *
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)


#define CAR_SPEED_IDLE                      128
#define CAR_SPEED_MAX                       100
#define CAR_TURN_IDLE                       128
#define CAR_TURN_MAX                        100

#define CAR_SPEED_INDEX                     SAADC_LEFT_Y
#define CAR_TURN_INDEX                      SAADC_LEFT_X
#define CAR_BLE_SPEED_INDEX                 0
#define CAR_BLE_TURN_INDEX                  1

#define ECHOBACK_BLE_UART_DATA              1                                       /**< Echo the UART data that is received over the Nordic UART Service back to the sender. */


/**@brief Variable length data encapsulation in terms of length and pointer to data. */
typedef struct
{
    uint8_t  * p_data;      /**< Pointer to data. */
    uint16_t   data_len;    /**< Length of data. */
} data_t;


static const nrf_drv_timer_t   	    m_timer = NRF_DRV_TIMER_INSTANCE(3);
static nrf_saadc_value_t       	    m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t       	    m_ppi_channel;
static uint32_t                	    m_adc_evt_counter;

static nrf_saadc_channel_config_t   channel_0_config;
static nrf_saadc_channel_config_t   channel_1_config;
static nrf_saadc_channel_config_t   channel_2_config;
static nrf_saadc_channel_config_t   channel_3_config;
static nrf_saadc_channel_config_t   channel_4_config;
void saadc_sampling_event_enable(void);
void saadc_init(void);
void saadc_sampling_event_init(void);

uint16_t conn_handle;

static bool connected = false;
static bool battery_low_voltage = false;

static uint8_t controller_output[20] = {0};

static bool usb_detected = false;
static bool charge_finished = false;


// Variables for handling a car via the game controller
static volatile uint8_t car_speed = 128;
static volatile uint8_t car_turn = 128;
static volatile bool new_date_available = false;

// Name to use for advertising and connection.
static volatile char m_target_periph_name[] = DEFAULT_DEVICE_NAME;

static volatile uint8_t target_name_length = sizeof(m_target_periph_name) - 1;



void set_led(uint32_t pin, uint8_t value)
{
    if(value)
        nrf_gpio_pin_clear(pin);
    else
        nrf_gpio_pin_set(pin);
}

void rgb_set(uint8_t r, uint8_t g, uint8_t b)
{
    set_led(RGB_R, r);
    set_led(RGB_G, g);
    set_led(RGB_B, b);
}



BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE NUS service client instance. */
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< DB discovery module instance. */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */


// Connection parameters requested for connection.
static ble_gap_conn_params_t m_conn_param =
{
    .min_conn_interval = CONN_INTERVAL_MIN,   // Minimum connection interval.
    .max_conn_interval = CONN_INTERVAL_MAX,   // Maximum connection interval.
    .slave_latency     = SLAVE_LATENCY,       // Slave latency.
    .conn_sup_timeout  = SUPERVISION_TIMEOUT  // Supervisory timeout.
};

/** @brief Parameters used when scanning. */
static ble_gap_scan_params_t const m_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION <= 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION >= 3)
        .use_whitelist = 0,
    #endif
};

/**@brief NUS uuid. */
static ble_uuid_t const m_nus_uuid =
{
    .uuid = UUID_TO_CONNECT_TO,
    .type = NUS_SERVICE_UUID_TYPE
};


/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function to start scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    ret = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(ret);

//    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
//    APP_ERROR_CHECK(ret);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}


/**@brief Function for handling characters received by the Nordic UART Service.
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
    ret_code_t ret_val;

    NRF_LOG_DEBUG("Receiving data.");
    NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);

    for (uint32_t i = 0; i < data_len; i++)
    {
        do
        {
            ret_val = app_uart_put(p_data[i]);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
    if (p_data[data_len-1] == '\r')
    {
        while (app_uart_put('\n') == NRF_ERROR_BUSY);
    }
    if (ECHOBACK_BLE_UART_DATA)
    {
        // Send data back to peripheral.
        do
        {
            ret_val = ble_nus_c_string_send(&m_ble_nus_c, p_data, data_len);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", ret_val);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint16_t index = 0;
    uint32_t ret_val;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
            {
                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                do
                {
                    ret_val = ble_nus_c_string_send(&m_ble_nus_c, data_array, index);
                    if ( (ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_BUSY) )
                    {
                        APP_ERROR_CHECK(ret_val);
                    }
                } while (ret_val == NRF_ERROR_BUSY);

                index = 0;
            }
            break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
            NRF_LOG_ERROR("Communication error occurred while handling UART.");
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


void game_controller_send_data()
{   
    static uint8_t data_array[5];
    
    data_array[CAR_BLE_SPEED_INDEX] = 225;
    data_array[CAR_BLE_TURN_INDEX] = 128;
    
    while (connected && (ble_nus_c_string_send(&m_ble_nus_c, data_array, 5) != NRF_SUCCESS)){}
    NRF_LOG_RAW_INFO("Data sent: %d\t%d\n", data_array[CAR_BLE_SPEED_INDEX], data_array[CAR_BLE_TURN_INDEX]);
//    do
//    {
//        ret_val = ble_nus_c_string_send(&m_ble_nus_c, data_array, 5);
//        if ( (ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_BUSY) )
//        {
//            APP_ERROR_CHECK(ret_val);
//        }
//    } while (ret_val == NRF_ERROR_BUSY);
}


/**@brief Callback handling NUS Client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS Client Handle. This identifies the NUS client
 * @param[in]   p_ble_nus_evt Pointer to the NUS Client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;

    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete.");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Nordic UART Service.");
            rgb_set(0, 0, 1);
            
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            //scan_start();
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */


/**
 * @brief Function for shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);




/**@brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  Type of data to be looked for in advertisement data.
 * @param[in]  Advertisement report length and pointer to report.
 * @param[out] If data type requested is found in the data report, type data length and
 *             pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index + 2];
            p_typedata->data_len = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}



/**@brief Function for searching a given name in the advertisement packets.
 *
 * @details Use this function to parse received advertising data and to find a given
 * name in them either as 'complete_local_name' or as 'short_local_name'.
 *
 * @param[in]   p_adv_report   advertising data to parse.
 * @param[in]   name_to_find   name to search.
 * @return   true if the given name was found, false otherwise.
 */
static bool find_adv_name(ble_gap_evt_adv_report_t const * p_adv_report, char const * name_to_find)
{
    ret_code_t err_code;
    data_t     adv_data;
    data_t     dev_name;
    bool       found = false;

    // Initialize advertisement report for parsing.
    adv_data.p_data   = (uint8_t *)p_adv_report->data;
    adv_data.data_len = p_adv_report->dlen;

    // Search for matching advertising names.
    err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &adv_data, &dev_name);

    if (   (err_code == NRF_SUCCESS)
        && (target_name_length == dev_name.data_len)
        && (memcmp(name_to_find, dev_name.p_data, dev_name.data_len) == 0))
    {
        found = true;
    }
    else
    {
        // Look for the short local name if the complete name was not found.
        err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, &adv_data, &dev_name);

        if (   (err_code == NRF_SUCCESS)
            && (strlen(name_to_find) == dev_name.data_len)
            && (memcmp((const void *)m_target_periph_name, dev_name.p_data, dev_name.data_len) == 0))
        {
            found = true;
        }
    }

    return found;
}



/**@brief Reads an advertising report and checks if a UUID is present in the service list.
 *
 * @details The function is able to search for 16-bit, 32-bit and 128-bit service UUIDs.
 *          To see the format of a advertisement packet, see
 *          https://www.bluetooth.org/Technical/AssignedNumbers/generic_access_profile.htm
 *
 * @param[in]   p_target_uuid The UUID to search for.
 * @param[in]   p_adv_report  Pointer to the advertisement report.
 *
 * @retval      true if the UUID is present in the advertisement report. Otherwise false
 */
static bool is_uuid_present(ble_uuid_t               const * p_target_uuid,
                            ble_gap_evt_adv_report_t const * p_adv_report)
{
    ret_code_t   err_code;
    ble_uuid_t   extracted_uuid;
    uint16_t     index  = 0;
    uint8_t    * p_data = (uint8_t *)p_adv_report->data;

    while (index < p_adv_report->dlen)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (   (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE)
            || (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE))
        {
            for (uint32_t i = 0; i < (field_length / UUID16_SIZE); i++)
            {
                err_code = sd_ble_uuid_decode(UUID16_SIZE,
                                              &p_data[i * UUID16_SIZE + index + 2],
                                              &extracted_uuid);

                if (err_code == NRF_SUCCESS)
                {
                    if (extracted_uuid.uuid == p_target_uuid->uuid)
                    {
                        return true;
                    }
                }
            }
        }
        else if (   (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_MORE_AVAILABLE)
                 || (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_COMPLETE))
        {
            for (uint32_t i = 0; i < (field_length / UUID32_SIZE); i++)
            {
                err_code = sd_ble_uuid_decode(UUID32_SIZE,
                                              &p_data[i * UUID32_SIZE + index + 2],
                                              &extracted_uuid);

                if (err_code == NRF_SUCCESS)
                {
                    if (   (extracted_uuid.uuid == p_target_uuid->uuid)
                        && (extracted_uuid.type == p_target_uuid->type))
                    {
                        return true;
                    }
                }
            }
        }

        else if (   (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE)
                 || (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE))
        {
            err_code = sd_ble_uuid_decode(UUID128_SIZE, &p_data[index + 2], &extracted_uuid);
            if (err_code == NRF_SUCCESS)
            {
                if (   (extracted_uuid.uuid == p_target_uuid->uuid)
                    && (extracted_uuid.type == p_target_uuid->type))
                {
                    return true;
                }
            }
        }
        index += field_length + 1;
    }
    return false;
}




/**@brief Function for handling BLE_GAP_ADV_REPORT events.
 * Search for a peer with matching device name.
 * If found, stop advertising and send a connection request to the peer.
 */
static void on_ble_gap_evt_adv_report(ble_gap_evt_t const * p_gap_evt)
{
    if (!find_adv_name(&p_gap_evt->params.adv_report, (const void *)m_target_periph_name))
    {
        return;
    }

    NRF_LOG_INFO("Device \"%s\" found, sending a connection request.",
                 (uint32_t) m_target_periph_name);

    // Stop advertising.
    (void) sd_ble_gap_adv_stop();

    // Initiate connection.
    m_conn_param.min_conn_interval = CONN_INTERVAL_DEFAULT;
    m_conn_param.max_conn_interval = CONN_INTERVAL_DEFAULT;

    ret_code_t err_code;
    err_code = sd_ble_gap_connect(&p_gap_evt->params.adv_report.peer_addr,
                                  &m_scan_params,
                                  &m_conn_param,
                                  APP_BLE_CONN_CFG_TAG);

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("sd_ble_gap_connect() failed: 0x%x.", err_code);
    }
}




/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            ble_gap_evt_adv_report_t const * p_adv_report = &p_gap_evt->params.adv_report;
            
            on_ble_gap_evt_adv_report(p_gap_evt);

            if (is_uuid_present(&m_nus_uuid, p_adv_report) && 0)
            {

                err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                              &m_scan_params,
                                              &m_conn_param,
                                              APP_BLE_CONN_CFG_TAG);

                if (err_code == NRF_SUCCESS)
                {
                    // scan is automatically stopped by the connect
                    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
                    APP_ERROR_CHECK(err_code);
                    NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                             p_adv_report->peer_addr.addr[0],
                             p_adv_report->peer_addr.addr[1],
                             p_adv_report->peer_addr.addr[2],
                             p_adv_report->peer_addr.addr[3],
                             p_adv_report->peer_addr.addr[4],
                             p_adv_report->peer_addr.addr[5]
                             );
                }
            }
        }break; // BLE_GAP_EVT_ADV_REPORT

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected to target");
            err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            // start discovery of services. The NUS Client waits for a discovery result
            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
            rgb_set(0, 0, 1);
        
            conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        
            
            connected = true;
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                NRF_LOG_INFO("Scan timed out.");
                scan_start();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            rgb_set(0, 1, 0);
            scan_start();
            break;  

#if defined(S132)
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
#endif

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_ble_nus_c.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the UART. */
//static void uart_init(void)
//{
//    ret_code_t err_code;

//    app_uart_comm_params_t const comm_params =
//    {
//        .rx_pin_no    = RX_PIN_NUMBER,
//        .tx_pin_no    = TX_PIN_NUMBER,
//        .rts_pin_no   = RTS_PIN_NUMBER,
//        .cts_pin_no   = CTS_PIN_NUMBER,
//        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
//        .use_parity   = false,
//        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
//    };

//    APP_UART_FIFO_INIT(&comm_params,
//                       UART_RX_BUF_SIZE,
//                       UART_TX_BUF_SIZE,
//                       uart_event_handle,
//                       APP_IRQ_PRIORITY_LOWEST,
//                       err_code);

//    APP_ERROR_CHECK(err_code);
//}

/**@brief Function for initializing the NUS Client. */
static void nus_c_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;

    init.evt_handler = ble_nus_c_evt_handler;

    err_code = ble_nus_c_init(&m_ble_nus_c, &init);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing buttons and leds. */
static void leds_init(void)
{
    nrf_gpio_cfg_output(RGB_R);
    nrf_gpio_cfg_output(RGB_G);
    nrf_gpio_cfg_output(RGB_B);
    nrf_gpio_pin_set(RGB_R);
    nrf_gpio_pin_clear(RGB_G);
    nrf_gpio_pin_set(RGB_B);
}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the Power manager. */
static void power_init(void)
{
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the Database Discovery Module. */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


 static void button_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
 {
     static char name_blue[] = NAME_BLUE_CAR;
     static char name_red[] = NAME_RED_CAR;
     static char name_demo[] = NAME_DEMO_CAR;
     static char name_balancer[] = NAME_BALANCER;
     
    switch(pin)
    {
        case LEFT_BTN_1_PIN:
            connected = false;
            sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            target_name_length = strlen(name_balancer);
            memcpy((void *)&m_target_periph_name, name_balancer, target_name_length);
            if(action == NRF_GPIOTE_POLARITY_HITOLO)
            {
                NRF_LOG_RAW_INFO("Left button 1 pressed");
            } 
            else if(action == NRF_GPIOTE_POLARITY_LOTOHI)
            {
                NRF_LOG_RAW_INFO("Left button 1 released");
            }
            break;
        
        case LEFT_BTN_2_PIN:
            if(action == NRF_GPIOTE_POLARITY_HITOLO)
            {
                NRF_LOG_RAW_INFO("Left button 2 pressed");
            } 
            else if(action == NRF_GPIOTE_POLARITY_LOTOHI)
            {
                NRF_LOG_RAW_INFO("Left button 2 released");
            }
            break;
        
        case LEFT_BTN_3_PIN:
            if(action == NRF_GPIOTE_POLARITY_HITOLO)
            {
                NRF_LOG_RAW_INFO("Left button 3 pressed");
            } 
            else if(action == NRF_GPIOTE_POLARITY_LOTOHI)
            {
                NRF_LOG_RAW_INFO("Left button 3 released");
            }
            break;
        
        case LEFT_BTN_4_PIN:
            if(action == NRF_GPIOTE_POLARITY_HITOLO)
            {
                NRF_LOG_RAW_INFO("Left button 4 pressed");
            } 
            else if(action == NRF_GPIOTE_POLARITY_LOTOHI)
            {
                NRF_LOG_RAW_INFO("Left button 4 released");
            }
            break;
        
        case RIGHT_BTN_1_PIN:
            if(action == NRF_GPIOTE_POLARITY_HITOLO)
            {
                NRF_LOG_RAW_INFO("Right button 1 pressed");
            } 
            else if(action == NRF_GPIOTE_POLARITY_LOTOHI)
            {
                NRF_LOG_RAW_INFO("Right button 1 released");
            }
            break;
        
        case RIGHT_BTN_2_PIN:
            connected = false;
            sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            target_name_length = strlen(name_blue);
            memcpy((void *)&m_target_periph_name, name_blue, target_name_length);
            if(action == NRF_GPIOTE_POLARITY_HITOLO)
            {
                NRF_LOG_RAW_INFO("Right button 2 pressed");
            } 
            else if(action == NRF_GPIOTE_POLARITY_LOTOHI)
            {
                NRF_LOG_RAW_INFO("Right button 2 released");
            }
            break;
        
        case RIGHT_BTN_3_PIN:
            connected = false;
            sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            target_name_length = strlen(name_demo);
            memcpy((void *)&m_target_periph_name, name_demo, target_name_length);
            if(action == NRF_GPIOTE_POLARITY_HITOLO)
            {
                NRF_LOG_RAW_INFO("Right button 3 pressed");
            } 
            else if(action == NRF_GPIOTE_POLARITY_LOTOHI)
            {
                NRF_LOG_RAW_INFO("Right button 3 released");
            }
            break;
        
        case RIGHT_BTN_4_PIN:
            connected = false;
            sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            target_name_length = strlen(name_red);
            memcpy((void *)&m_target_periph_name, name_red, target_name_length);
            if(action == NRF_GPIOTE_POLARITY_HITOLO)
            {
                NRF_LOG_RAW_INFO("Right button 4 pressed");
            } 
            else if(action == NRF_GPIOTE_POLARITY_LOTOHI)
            {
                NRF_LOG_RAW_INFO("Right button 4 released");
            }
            break;
        
    }
    NRF_LOG_FLUSH();
 }

 
 
 
	////// SAADC part ////////



void timer_handler(nrf_timer_event_t event_type, void* p_context) {}

void saadc_sampling_event_init(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    
    nrf_drv_timer_config_t cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    
    err_code = nrf_drv_timer_init(&m_timer, &cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 100ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, SAADC_TIMER_INTERVAL);
    nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_event_addr = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, saadc_sample_event_addr);
    APP_ERROR_CHECK(err_code);
}

void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

// Function to constrain output value to uint8_t and avoid fluctuations around zero-point for the SAADC to be interpreted as user input
uint8_t controller_normalize_output(int16_t input, uint8_t scale, uint8_t lower_border, uint8_t upper_border, uint8_t default_value)
{
    input = input < 0 ? 0 : input;
    uint8_t scaled_value = input / scale;
    return scaled_value < lower_border ? scaled_value : scaled_value > upper_border ? scaled_value : default_value;
}

// Function that outputs values for the joysticks
uint8_t controller_output_calc(nrf_drv_saadc_evt_t const * event) {
    controller_output[CAR_BLE_SPEED_INDEX] = controller_normalize_output(event->data.done.p_buffer[CAR_SPEED_INDEX], 4, 120, 130, 128);
    controller_output[CAR_BLE_TURN_INDEX] = 255 - controller_normalize_output(event->data.done.p_buffer[CAR_TURN_INDEX], 4, 120, 130, 127); 
    
//    NRF_LOG_RAW_INFO("%d\n", controller_output[CAR_BLE_SPEED_INDEX]);
//    NRF_LOG_FLUSH();

    return 0;
}


// SAADC callback
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    static uint16_t connected_counter = 0;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t          err_code;
        
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
        
        if((m_adc_evt_counter % BATTERY_VOLTAGE_MONITOR_INTERVAL) == 0)
        {
            nrf_saadc_value_t   vdd_result;
            uint16_t            batt_lvl_in_milli_volts;
            

            vdd_result = p_event->data.done.p_buffer[4];
            batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(vdd_result) +
                                      DIODE_FWD_VOLT_DROP_MILLIVOLTS;
            
            if(batt_lvl_in_milli_volts < VDD_MIN_THRESHOLD) {
                battery_low_voltage = true;
            }
            
            NRF_LOG_RAW_INFO("Voltage: %d\n", batt_lvl_in_milli_volts);
        }

        // Update characteristic array with new joystick data
        controller_output_calc(p_event);

        #if PRINT_SAADC_VALUES
            for(uint8_t i = 0; i < SAMPLES_IN_BUFFER; i++)
            {
                NRF_LOG_RAW_INFO("%d\t", controller_output[i]);
            }
            NRF_LOG_RAW_INFO("\r\n");
            NRF_LOG_FLUSH();
        #endif
        

        if(connected && (connected_counter < 20))
            connected_counter++;
        
        // Send data
        
        
        if(connected && (m_adc_evt_counter > 40) && (connected_counter >= 20))
        {
            new_date_available = true;
        }
        m_adc_evt_counter++;
    }
}

void saadc_init(void)
{
    ret_code_t err_code;

    //set configuration for saadc channel 0, for the left joystick x axis
    channel_0_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_0_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    channel_0_config.gain       = NRF_SAADC_GAIN1_4;
    channel_0_config.reference  = NRF_SAADC_REFERENCE_VDD4;
    channel_0_config.acq_time   = NRF_SAADC_ACQTIME_3US;
    channel_0_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED;
    channel_0_config.pin_p      = (nrf_saadc_input_t)(JS_LEFT_X);
    channel_0_config.pin_n      = NRF_SAADC_INPUT_DISABLED;


    //set configuration for saadc channel 1, for the left joystick x axis
    channel_1_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_1_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    channel_1_config.gain       = NRF_SAADC_GAIN1_4;
    channel_1_config.reference  = NRF_SAADC_REFERENCE_VDD4;
    channel_1_config.acq_time   = NRF_SAADC_ACQTIME_3US;
    channel_1_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED;
    channel_1_config.pin_p      = (nrf_saadc_input_t)(JS_LEFT_Y);
    channel_1_config.pin_n      = NRF_SAADC_INPUT_DISABLED;


    //set configuration for saadc channel 2, for the left joystick x axis
    channel_2_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_2_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    channel_2_config.gain       = NRF_SAADC_GAIN1_4;
    channel_2_config.reference  = NRF_SAADC_REFERENCE_VDD4;
    channel_2_config.acq_time   = NRF_SAADC_ACQTIME_3US;
    channel_2_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED;
    channel_2_config.pin_p      = (nrf_saadc_input_t)(JS_RIGHT_X);
    channel_2_config.pin_n      = NRF_SAADC_INPUT_DISABLED;


    //set configuration for saadc channel 3, for the left joystick x axis
    channel_3_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_3_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    channel_3_config.gain       = NRF_SAADC_GAIN1_4;
    channel_3_config.reference  = NRF_SAADC_REFERENCE_VDD4;
    channel_3_config.acq_time   = NRF_SAADC_ACQTIME_3US;
    channel_3_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED;
    channel_3_config.pin_p      = (nrf_saadc_input_t)(JS_RIGHT_Y);
    channel_3_config.pin_n      = NRF_SAADC_INPUT_DISABLED;




    //set configuration for saadc channel 3, for the left joystick x axis
    channel_4_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_4_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    channel_4_config.gain       = NRF_SAADC_GAIN1_6;
    channel_4_config.reference  = NRF_SAADC_REFERENCE_INTERNAL;
    channel_4_config.acq_time   = NRF_SAADC_ACQTIME_10US;
    channel_4_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED; 
    channel_4_config.burst      = NRF_SAADC_BURST_DISABLED,  
    channel_4_config.pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_VDD);
    channel_4_config.pin_n      = NRF_SAADC_INPUT_DISABLED;


    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(2, &channel_2_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(3, &channel_3_config);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_saadc_channel_init(4, &channel_4_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}



////// END of SAADC ////////

void chg_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    switch(pin)
    {
        case USB_DETECT:
            if(nrf_gpio_pin_read(USB_DETECT))
                usb_detected = true;
            else
                usb_detected = false;
            break;
        case BAT_CHG_STATUS:
            if(nrf_gpio_pin_read(BAT_CHG_STATUS))
                charge_finished = true;
            else
                charge_finished = false;
            break;
    }
   
    if(usb_detected)
    {
        if(charge_finished)
            rgb_set(0, 1, 1);
        else
            rgb_set(1, 1, 0);
    }
    else
    {
        if(connected)
            rgb_set(0, 0, 1);
        else
            rgb_set(0, 1, 0);
    }
}



void gpiote_init(void)
{
    uint32_t err_code;
    
     // Configuring other GPIOTE events
    nrf_drv_gpiote_in_config_t chg_detect_cfg = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    err_code = nrf_drv_gpiote_in_init(USB_DETECT, &chg_detect_cfg, chg_evt_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(BAT_CHG_STATUS, &chg_detect_cfg, chg_evt_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(USB_DETECT, true);
    nrf_drv_gpiote_in_event_enable(BAT_CHG_STATUS, true);
}
 
 
 

int main(void)
{
  
    log_init();
    timer_init();
    power_init();
    //uart_init();
    leds_init();
    db_discovery_init();
    ble_stack_init();
    gatt_init();
    nus_c_init();
    saadc_sampling_event_init();
    saadc_init();
    saadc_sampling_event_enable();
    
    game_controller_init(button_evt_handler);
    gpiote_init();

    // Start scanning for peripherals and initiate connection
    // with devices that advertise NUS UUID.
    // printf("BLE UART central example started.");
    NRF_LOG_INFO("BLE UART central example started.");
    scan_start();

    for (;;)
    {
        if (new_date_available && connected)
        {
            new_date_available = false;
            
            while (ble_nus_c_string_send(&m_ble_nus_c, controller_output, 2) != NRF_SUCCESS)
            {
                    // repeat until sent.
            }
        }
        
        if(battery_low_voltage)
        {
            rgb_set(1, 0, 0);
            connected = false;
        }
    }
}
