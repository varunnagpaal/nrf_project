#include "board_components.h"
#include <string.h>

/**@brief This macro will call the registered error handler if err_code
 *        is not NRF_SUCCESS and the error handler is not NULL.
 */
#define CALL_HANDLER_ON_ERROR(err_code)                           \
do                                                                \
{                                                                 \
    if (((err_code) != NRF_SUCCESS) && (m_error_handler != NULL)) \
    {                                                             \
        m_error_handler(err_code);                                \
    }                                                             \
}                                                                 \
while (0)

uint8_t board_led_pin_list[BOARD_LED_COUNT] = BOARD_LEDS_PIN_LIST;          /* List of Board LED's */
uint8_t board_button_pin_list[BOARD_BUTTON_COUNT] = BOARD_BUTTON_PIN_LIST;  /* List of Board Button's */
static board_buttons_error_handler_t  m_error_handler = NULL;                       /**< Error handler registered by the user. */
static uint32_t                           m_num_connections = 0;                        /**< Number of connections the device is currently in. */

/**@brief Function for configuring the buttons for connection.
 *
 * @retval NRF_SUCCESS  Configured successfully.
 * @return A propagated error code.
 */
static uint32_t buttons_connection_configure()
{
  uint32_t err_code = NRF_SUCCESS;

  // err_code = bsp_event_to_button_action_assign(BTN_ID_SLEEP,
  //                                              BTN_ACTION_SLEEP,
  //                                              BSP_EVENT_DEFAULT);
  // RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);

  // err_code = bsp_event_to_button_action_assign(BTN_ID_WHITELIST_OFF,
  //                                              BTN_ACTION_WHITELIST_OFF,
  //                                              BSP_EVENT_WHITELIST_OFF);
  // RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);

  // err_code = bsp_event_to_button_action_assign(BTN_ID_DISCONNECT,
  //                                              BTN_ACTION_DISCONNECT,
  //                                              BSP_EVENT_DISCONNECT);
  // RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);

  return err_code;
}

/**@brief Function for configuring the buttons for advertisement.
 *
 * @retval NRF_SUCCESS  Configured successfully.
 * @return A propagated error code.
 */
static uint32_t buttons_advertising_configure()
{
  uint32_t err_code = NRF_SUCCESS;

  // err_code = bsp_event_to_button_action_assign(BTN_ID_DISCONNECT,
  //                                              BTN_ACTION_DISCONNECT,
  //                                              BSP_EVENT_DEFAULT);
  // RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);

  // err_code = bsp_event_to_button_action_assign(BTN_ID_WHITELIST_OFF,
  //                                              BTN_ACTION_WHITELIST_OFF,
  //                                              BSP_EVENT_WHITELIST_OFF);
  // RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);

  // err_code = bsp_event_to_button_action_assign(BTN_ID_SLEEP,
  //                                              BTN_ACTION_SLEEP,
  //                                              BSP_EVENT_SLEEP);
  // RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);

  return err_code;
}

void board_buttons_on_ble_evt(ble_evt_t * p_ble_evt)
{
  uint32_t err_code;

  switch (p_ble_evt->header.evt_id)
  {
    case BLE_GAP_EVT_CONNECTED:
      if (m_num_connections == 0)
      {
        err_code = buttons_connection_configure();
        CALL_HANDLER_ON_ERROR(err_code);
      }

      m_num_connections++;
      break;

    case BLE_GAP_EVT_DISCONNECTED:
      m_num_connections--;

      if (m_num_connections == 0)
      {
        err_code = buttons_advertising_configure();
        CALL_HANDLER_ON_ERROR(err_code);
      }
      break;

    default:
      break;
  }    
}
