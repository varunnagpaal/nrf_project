#include "fsm.h"

int main(void)
{
  STATE_TYPE nextState = STATE_ON_RESET;
  
  do
  {
    switch (nextState)
    {
      case STATE_ON_RESET:
        nextState = state_action_on_reset();
        break;
      
      case STATE_RESOLVE_RESET_REASON:
        nextState = state_action_resolve_reset_reason();
        break;      

      case STATE_DEEP_SLEEP:
        nextState = state_action_deep_sleep();
        break;

      case STATE_HARDWARE_INIT:
        nextState = state_action_hardware_init();
        break;

      case STATE_BLE_CONNECTING:
        nextState = state_action_ble_connecting();
        break;

      case STATE_NAP_SLEEP:
        nextState = state_action_nap_sleep();
        break;

      case STATE_AWAIT_USER_ACTIVITY:
        nextState = state_action_await_user_activity();
        break;

      case STATE_STREAM_SENSOR_DATA:
        nextState = state_action_stream_sensor_data();
        break;

      default:  /* Incorrect state. Goto reset state */
        nextState = STATE_ON_RESET;
        break;
    }
  } while(1);
}
