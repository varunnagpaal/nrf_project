#ifndef PIN_MAP_H_
#define PIN_MAP_H_

#include <stdint.h>
#include "nrf_twi.h"
#include "app_util_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_LED_COUNT 5
#define BOARD_LED_1 0
#define BOARD_LED_2 1
#define BOARD_LED_3 2
#define BOARD_LED_1_PIN 21
#define BOARD_LED_2_PIN 22
#define BOARD_LED_3_PIN 23

#define BOARD_BUTTON_COUNT 4
#define BOARD_BUTTON_1 0
#define BOARD_BUTTON_1_PIN 17


#define BOARD_LEDS_PIN_LIST {   BOARD_LED_1_PIN, \
                                    BOARD_LED_2_PIN, \
                                    BOARD_LED_3_PIN }

#define BOARD_BUTTON_PIN_LIST { BOARD_BUTTON_1_PIN }

#define GPIO_SENSE_ACCURACY_HIGH    (true)
#define GPIO_SENSE_ACCURACY_LOW     (false)
#define BOARD_BUTTON_SENSE_ACCURACY GPIO_SENSE_ACCURACY_LOW

/* GPIO input pins for entering/exiting sleep modes */
#define DEEP_SLEEP_ENTER_BUTTON_PIN BOARD_BUTTON_1_PIN

/* MAX86150 sensor pin mapping */
/* Note: TWI0 shares same registers with SPI0 as they are both
   located at same base address (the have same ID). So anytime,
   only one of them can be used and other must be disabled.
   Further the shared registers will not be reset on disabling 
   one of the devices and so in order to use other peripheral
   one needs to explicitly re-configure the shared registers explicitly
*/
#define BOARD_NRF_TWI_INSTANCE_ID                       0
#define BOARD_SENSOR_MAX8511_SHDN_N_PIN                 28
#define BOARD_SENSOR_MAX8511_SHDN_N_PIN_INIT_STATE      NRF_GPIOTE_INITIAL_VALUE_LOW
#define BOARD_SENSOR_MAX86150_INTB_PIN                  7
#define BOARD_SENSOR_MAX86150_INTB_PIN_SENSE_ACCURACY   GPIO_SENSE_ACCURACY_LOW
#define BOARD_SENSOR_MAX86150_I2C_TWI0_SCL_PIN          30
#define BOARD_SENSOR_MAX86150_I2C_TWI0_SDA_PIN          29
#define BOARD_SENSOR_MAX86150_I2C_TWI0_FREQ             NRF_TWI_FREQ_100K
#define BOARD_SENSOR_MAX86150_I2C_TWI0_IRQ_PRIORITY     APP_IRQ_PRIORITY_HIGH

#define BOARD_SENSOR_MAX86150_I2C_TWI0_PIN_LIST { BOARD_SENSOR_MAX86150_INTB_PIN, \
                                                      BOARD_SENSOR_MAX86150_I2C_TWI0_SCL_PIN, \
                                                      BOARD_SENSOR_MAX86150_I2C_TWI0_SDA_PIN }

                                                      
#define BOARD_NRF_SPI_INSTANCE_ID                       0
#define BOARD_SENSOR_TDC7200_ENABLE_PIN                 25
#define BOARD_SENSOR_TDC7200_TRIGG_PIN                  
#define BOARD_SENSOR_TDC7200_START_PIN                 
#define BOARD_SENSOR_TDC7200_STOP_PIN
#define BOARD_SENSOR_TDC7200_ENABLE_PIN                 25
#define BOARD_SENSOR_TDC7200_ENABLE_PIN                 25
#define BOARD_SENSOR_TDC7200_ENABLE_PIN                 25                                                      
                                                      
                                                      
                                                      
#define BOARD_SENSOR_MAX8511_SHDN_N_PIN_INIT_STATE      NRF_GPIOTE_INITIAL_VALUE_LOW
                                                      
#define BOARD_SENSOR_MAX86150_INTB_PIN                  7
#define BOARD_SENSOR_MAX86150_INTB_PIN_SENSE_ACCURACY   GPIO_SENSE_ACCURACY_LOW
#define BOARD_SENSOR_MAX86150_I2C_TWI0_SCL_PIN          30
#define BOARD_SENSOR_MAX86150_I2C_TWI0_SDA_PIN          29
#define BOARD_SENSOR_MAX86150_I2C_TWI0_FREQ             NRF_TWI_FREQ_100K
#define BOARD_SENSOR_MAX86150_I2C_TWI0_IRQ_PRIORITY     APP_IRQ_PRIORITY_HIGH

#define BOARD_SENSOR_MAX86150_I2C_TWI0_PIN_LIST { BOARD_SENSOR_MAX86150_INTB_PIN, \
                                                      BOARD_SENSOR_MAX86150_I2C_TWI0_SCL_PIN, \
                                                      BOARD_SENSOR_MAX86150_I2C_TWI0_SDA_PIN }                                                      

#ifdef __cplusplus
}
#endif

#endif  /* PIN_MAP_H_ */
