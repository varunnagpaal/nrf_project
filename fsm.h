#ifndef FSM_H_
#define FSM_H_

#ifdef __cplusplus
extern "C" {
#endif

/* System States */
typedef enum
{
  STATE_ON_RESET = 0,
  STATE_RESOLVE_RESET_REASON,
  STATE_DEEP_SLEEP,
  STATE_HARDWARE_INIT,
  STATE_NAP_SLEEP,
  STATE_BLE_CONNECTING,
  STATE_AWAIT_USER_ACTIVITY,
  STATE_STREAM_SENSOR_DATA,
  STATE_UNKNOWN                 /* All new states should be added before this */
} STATE_TYPE;

/* State Logic Functions */
STATE_TYPE state_action_on_reset( void );
STATE_TYPE state_action_resolve_reset_reason( void );
STATE_TYPE state_action_deep_sleep( void );
STATE_TYPE state_action_hardware_init( void );
STATE_TYPE state_action_nap_sleep( void );
STATE_TYPE state_action_ble_connecting( void );
STATE_TYPE state_action_await_user_activity( void );
STATE_TYPE state_action_stream_sensor_data( void );
STATE_TYPE state_action_unknown( void );

extern const char * const state_to_str[];

#ifdef __cplusplus
}
#endif

#endif  /* FSM_H_ */
