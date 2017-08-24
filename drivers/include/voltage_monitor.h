#include <stdint.h>
#include "nrf_drv_saadc.h"
#include "nrf_drv_timer.h"


#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                                     /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                                       /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  0                                     /**< Typical forward voltage drop of the diode . */
#define ADC_RES_10BIT                   1024                                    /**< Maximum digital value for 10-bit ADC conversion. */
#define SAMPLES_IN_MONITOR_BUFFER       1

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define VOLTAGE_MONITOR_PRINT_VDD_MV    1
#define VOLTAGE_MONITOR_TIMER_INTERVAL	1000
#define VOLTAGE_MONITOR_THRESHOLD       3200




/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 *
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)


typedef struct 
{
    uint8_t saadc_channel;
    nrf_drv_timer_t * timer_instance;
    bool * low_voltage_flag;
} voltage_monitor_cfg_t;


void voltage_monitor_init(voltage_monitor_cfg_t * config);

