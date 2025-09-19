#ifndef BOARD_COMPONENTS_H_
#define BOARD_COMPONENTS_H_

#include "pin_map.h"
#include "nrf_sdm.h"
#include "ble.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t board_led_pin_list[BOARD_LED_COUNT];             /* List of Board LED's */
extern uint8_t board_button_pin_list[BOARD_BUTTON_COUNT];       /* List of Board Button's */

/**@brief BLE Button Module error handler type. */
typedef void (*board_buttons_error_handler_t) (uint32_t nrf_error);

/**@brief Function for handling the application's BLE stack events and do something to board buttons accordingly.
 *
 * @details This function handles all events from the BLE stack that are of interest to this module.
 *
 * @param[in] p_ble_evt BLE stack event.
 */
void board_buttons_on_ble_evt(ble_evt_t * p_ble_evt);

#ifdef __cplusplus
}
#endif

#endif  /* BOARD_COMPONENTS_H_ */
