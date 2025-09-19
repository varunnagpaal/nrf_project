#include "fsm.h"
#include "common.h"
#define NRF_LOG_MODULE_NAME "MOD"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "pin_map.h"
#include "helpers.h"
#include "init.h"
#include "nrf_delay.h"
#include "nrf_power.h"
#include "sensor_max86150_nrf_helper.h"

STATE_TYPE lastState = STATE_UNKNOWN;
STATE_TYPE currentState = STATE_UNKNOWN;
STATE_TYPE nextState = STATE_ON_RESET;

nrf_power_resetreas_mask_t reset_reason_mask;

extern volatile bool bNapSleepTimerExpired;
extern volatile bool bAdvTimerExpired;
extern volatile bool bIsBleConnected;
extern volatile bool bIsSensorMax86150IntDet;
extern volatile bool bIsSensorMax86150IntClr;

static volatile uint32_t sensor_max86150_ppg_led1_samples[32];
static volatile uint32_t sensor_max86150_ppg_led2_samples[32];
static volatile uint32_t sensor_max86150_ecg_samples[32];
static volatile uint8_t  sensor_max86150_valid_sample_count;

const char * const state_to_str[] =
{
    [STATE_ON_RESET] = "STATE_ON_RESET",
    [STATE_RESOLVE_RESET_REASON] = "STATE_RESOLVE_RESET_REASON",
    [STATE_DEEP_SLEEP] = "STATE_DEEP_SLEEP",
    [STATE_HARDWARE_INIT]  = "STATE_HARDWARE_INIT",
    [STATE_NAP_SLEEP]  = "STATE_NAP_SLEEP",
    [STATE_BLE_CONNECTING]  = "STATE_BLE_CONNECTING",
    [STATE_AWAIT_USER_ACTIVITY]  = "STATE_AWAIT_USER_ACTIVITY",
    [STATE_STREAM_SENSOR_DATA]  = "STATE_STREAM_SENSOR_DATA",
    [STATE_UNKNOWN]  = "STATE_UNKNOWN"
};

/* True: chip was explictly reset
   False: chip was implicitly reset due to wake up from deep sleep (SYSTEM OFF)
*/
bool bResetOnWakeUpFromDeepSleep = false;

STATE_TYPE state_action_on_reset( void )
{
  currentState = nextState;

  init_logging();

  /* Added wait. This is required because UART terminal needs manual turning ON to see logs.
     Otherwise, when device switched ON through switch, we will miss seeing the initial
     messages (especially upon switch reset, we want to see reset reason)
  */
  //nrf_delay_ms(10000);

  NRF_LOG_INFO( "[%d] %s -> %s\r\n",
                order++,
                (uint32_t) state_to_str[lastState],
                (uint32_t) state_to_str[currentState] );
  NRF_LOG_FLUSH();

  init_clks(true);  // LFCLk must be enabled before as it is required by RTC used as WDT for Power management init
  init_gpio(true);
  init_timers();
  init_power_management();

  nextState =  STATE_RESOLVE_RESET_REASON;
  lastState = currentState;
  return nextState;
}

STATE_TYPE state_action_resolve_reset_reason( void )
{
  currentState = nextState;
  NRF_LOG_INFO( "[%d] %s -> %s\r\n",
                order++,
                (uint32_t) state_to_str[lastState],
                (uint32_t) state_to_str[currentState] );
  NRF_LOG_FLUSH();

  /* Get the reason for reset */
  uint32_t reset_reason_reg = nrf_power_resetreas_get();

  if( reset_reason_reg & NRF_POWER_RESETREAS_RESETPIN_MASK )
    reset_reason_mask = NRF_POWER_RESETREAS_RESETPIN_MASK;
  else if( reset_reason_reg & NRF_POWER_RESETREAS_DOG_MASK )
    reset_reason_mask = NRF_POWER_RESETREAS_DOG_MASK;
  else if( reset_reason_reg & NRF_POWER_RESETREAS_SREQ_MASK )
    reset_reason_mask = NRF_POWER_RESETREAS_SREQ_MASK;
  else if( reset_reason_reg & NRF_POWER_RESETREAS_OFF_MASK )
    reset_reason_mask = NRF_POWER_RESETREAS_OFF_MASK;
  else if( reset_reason_reg & NRF_POWER_RESETREAS_LPCOMP_MASK )
    reset_reason_mask = NRF_POWER_RESETREAS_LPCOMP_MASK;
  else if( reset_reason_reg & NRF_POWER_RESETREAS_DIF_MASK )
    reset_reason_mask = NRF_POWER_RESETREAS_DIF_MASK;
#if defined(POWER_RESETREAS_NFC_Msk) || defined(__SDK_DOXYGEN__)
  else if( reset_reason_reg & NRF_POWER_RESETREAS_NFC_MASK )
    reset_reason_mask = NRF_POWER_RESETREAS_NFC_MASK;
#endif
#if defined(POWER_RESETREAS_VBUS_Msk) || defined(__SDK_DOXYGEN__)
  else if( reset_reason_reg & NRF_POWER_RESETREAS_VBUS_MASK )
    reset_reason_mask = NRF_POWER_RESETREAS_VBUS_MASK;
#endif

  /* Clear the reset reason register */
  nrf_power_resetreas_clear(reset_reason_mask);

  /* Check if reset was due to wake up from system OFF mode when wakeup is
     triggered from DETECT signal from GPIO
  */
  bResetOnWakeUpFromDeepSleep = (NRF_POWER_RESETREAS_OFF_MASK == reset_reason_mask) ? true : false;

  NRF_LOG_INFO( "[%d] Reason for Reset was wakeup from deep sleep ? : %s , reset_reason_mask: %d\r\n",
                order++,
                (uint32_t) (bResetOnWakeUpFromDeepSleep ? "True": "False"),
                (uint32_t) reset_reason_mask);
  NRF_LOG_FLUSH();

  /* Go into deep sleep only if reset is caused by reasons other than
   reset due to waking up from deep sleep */
  if( bResetOnWakeUpFromDeepSleep ) nextState = STATE_HARDWARE_INIT;
  else                              nextState = STATE_DEEP_SLEEP;

  lastState = currentState;
  return nextState;
}

STATE_TYPE state_action_deep_sleep( void )
{
  currentState = nextState;
  NRF_LOG_INFO( "[%d] %s -> %s\r\n",
              order++,
              (uint32_t) state_to_str[lastState],
              (uint32_t) state_to_str[currentState]);
  NRF_LOG_FLUSH();
  
  /* Disable power input into MAX86150 sensor */

  /* Enable interrupt or sense mechanism on buttons */
  init_button_interrupt_sense_control(true);

  /* Turn OFF all LEDs indicating deep sleep */
  board_led_off(BOARD_LED_1);
  board_led_off(BOARD_LED_2);
  board_led_off(BOARD_LED_3);

  /* Enter deep sleep state */
  system_sleep_deep_mode();
  
  nextState =  STATE_HARDWARE_INIT;
  lastState = currentState;
  return nextState;
}

STATE_TYPE state_action_hardware_init( void )
{
  currentState = nextState;
  NRF_LOG_INFO( "[%d] %s -> %s\r\n",
              order++,
              (uint32_t) state_to_str[lastState],
              (uint32_t) state_to_str[currentState]);
  NRF_LOG_FLUSH();

  create_timers();
  init_ble_stack();
  init_gap_params();
  init_advertising();
  init_gatt();
  init_services();
  init_conn_params();
  init_sensor_simulator();
  init_sensors();

  nrf_delay_ms(1000);
  board_led_on(BOARD_LED_1);

  nextState =  STATE_BLE_CONNECTING;
  lastState = currentState;
  return nextState;
}

STATE_TYPE state_action_ble_connecting( void )
{
  currentState = nextState;
  NRF_LOG_INFO( "[%d] %s -> %s\r\n",
              order++,
              (uint32_t) state_to_str[lastState],
              (uint32_t) state_to_str[currentState]);
  NRF_LOG_FLUSH();

  hrs_timer_control(true);

  /* Start advertising for 15 sec. If not Connected
     go back to nap for 5 sec. Wakeup and advertise
     again
  */
  uint32_t napTimeAccum = 0;
  uint32_t advTimeAccum = 0;
  uint32_t advTimerCntFrom = 0;
  uint32_t advTimeTicks = 0;

  do
  {
    /* 1. Start Advertising timer while Advertising */
    adv_timer_control(true, ADV_TIMEOUT_IN_MSEC);
    advTimerCntFrom = app_timer_cnt_get();
    board_nrf_advertisement_control(true);

    /* 2. Enter nap sleep between advertising events */
    bAdvTimerExpired = false;
    system_sleep_nap_mode(&bAdvTimerExpired);

    /* 3. If successfully connected during advertising, stop advertising timer and come out of this loop.
          Else if advertising timer has expired, then stop advertising and nap
          Else flag unexpected error as neither connected nor advertising timer expired
    */
    if(bIsBleConnected){
      /* Stop advertising timer when connected */
      app_timer_cnt_diff_compute(app_timer_cnt_get() - advTimerCntFrom,
                                 advTimerCntFrom,
                                 &advTimeTicks );
      advTimeAccum += APP_TIMER_TICKS_TO_MS(advTimeTicks, APP_TIMER_PRESCALER);
      adv_timer_control(false, NULL);
      break;
    }else if(bAdvTimerExpired){
      board_nrf_advertisement_control(false);
      advTimeAccum += ADV_TIMEOUT_IN_MSEC;
      nap_timer_control(true, NAP_TIMEOUT_IN_MSEC);
      bNapSleepTimerExpired = false;
      system_sleep_nap_mode(&bNapSleepTimerExpired);
      napTimeAccum += NAP_TIMEOUT_IN_MSEC;
    }else{
      app_timer_cnt_diff_compute(app_timer_cnt_get() - advTimerCntFrom,
                           advTimerCntFrom,
                           &advTimeTicks );
      advTimeAccum += APP_TIMER_TICKS_TO_MS(advTimeTicks, APP_TIMER_PRESCALER);
      board_nrf_advertisement_control(false);
      adv_timer_control(false, NULL);
      NRF_LOG_ERROR("[%d] %s: neither BLE connected nor the advertising timer expired. Something wrong!!",
                    order++,
                    (uint32_t) state_to_str[currentState] );
      break;
    }

    /* 4. When Nap timer times out, system will wake up and repeat step 1.
          We make fixed number of attempts before we exhaust total
          20 sec of advertisement (15 sec) + nap interval(5 sec) and go
          back to deep sleep
    */
  }while( (advTimeAccum+napTimeAccum) < ADV_NAP_TIMEOUT_IN_MSEC);

  // When control reaches here for VLED directly connected to VDD, the system
  // - doesn't reset if BLE not connected
  // - resets upon BLE connected (APP_ERROR:ERROR:Fatal). Not sure why: This is due to I2C probably
  nrf_delay_ms(1000);
  board_led_toggle(BOARD_LED_2);

  if(bIsBleConnected) nextState =  STATE_AWAIT_USER_ACTIVITY;
  else                nextState =  STATE_DEEP_SLEEP;

  lastState = currentState;
  return nextState;
}

STATE_TYPE state_action_nap_sleep( void )
{
  currentState = nextState;
  NRF_LOG_INFO( "[%d] %s -> %s\r\n",
              order++,
              (uint32_t) state_to_str[lastState],
              (uint32_t) state_to_str[currentState]);
  NRF_LOG_FLUSH();

  nrf_delay_ms(1000);
  board_led_toggle(BOARD_LED_3);

  nextState =  lastState;
  lastState = currentState;
  return nextState;
}

STATE_TYPE state_action_await_user_activity( void )
{
  currentState = nextState;
  NRF_LOG_INFO( "[%d] %s -> %s\r\n",
              order++,
              (uint32_t) state_to_str[lastState],
              (uint32_t) state_to_str[currentState]);
  NRF_LOG_FLUSH();

  /*
    Each transmit sequence is framed by a START (S) or REPEATED START (Sr) condition and a STOP (P) condition.
    Each word transmitted to the MAX86150 is 8 bits long and is followed by an acknowledge clock pulse.
    A controller reading data from the MAX86150 transmits the proper responder address followed by a series of nine SCL pulses.

    Each read sequence is framed by a START (S) or REPEATED START (Sr) condition, a not acknowledge, and a STOP (P) condition.
    The MAX86150 recognizes a STOP condition at any point during data transmission except if the STOP condition occurs in the same
    high pulse as a START condition. For proper operation, do not send a STOP condition during the same SCL high pulse as the START condition.

    When idle, the MAX86150 waits for a START condition followed by its 8-bit responder ID with LSbit as R/!W bit
    R/!W = 0 inidicates controller writes to the I2C compatible MAx86150
    R/!W = 1 inidicates controller reads from the I2C compatible MAx86150
    The serial interface compares MSignificant 7-bits of responder ID bit by bit, allowing the interface to power down
    and disconnect from SCL immediately if an incorrect responder ID is detected.
    After receiving the proper responder address ID, the MAX86150 issues an ACK by pulling SDA low for one clock cycle.
  */
  nrf_delay_ms(1000);

  sensor_max86150_registers_configure();

  nextState =  STATE_STREAM_SENSOR_DATA;
  lastState = currentState;
  return nextState;
}

STATE_TYPE state_action_stream_sensor_data( void )
{
  currentState = nextState;
  NRF_LOG_INFO( "[%d] %s -> %s\r\n",
              order++,
              (uint32_t) state_to_str[lastState],
              (uint32_t) state_to_str[currentState]);
  NRF_LOG_FLUSH();

  do
  {
    sensor_max86150_get_fifo_data(&sensor_max86150_ppg_led1_samples[0],
                                      &sensor_max86150_ppg_led2_samples[0],
                                      &sensor_max86150_ecg_samples[0],
                                      &sensor_max86150_valid_sample_count );
    NRF_LOG_INFO("[%d] Valid Sample Count: %d \r\n", order++, sensor_max86150_valid_sample_count);

    if( sensor_max86150_valid_sample_count )
    {
      NRF_LOG_INFO("[%d] PPG_LED1_DATA, PPG_LED2_DATA, ECG_DATA \r\n", order++);
      for( uint8_t i = 0; i < sensor_max86150_valid_sample_count; ++i)
      {
        NRF_LOG_INFO("[%d] %d, %d, %d \r\n",
                      order++,
                      sensor_max86150_ppg_led1_samples[i],
                      sensor_max86150_ppg_led2_samples[i],
                      sensor_max86150_ecg_samples[i] );
      }
    }
    NRF_LOG_FLUSH();
    //nrf_delay_ms(2000);
  }while(1);

  nextState =  STATE_AWAIT_USER_ACTIVITY;
  lastState = currentState;
  return nextState;
}

STATE_TYPE state_action_unknown( void )
{
  currentState = nextState;

  NRF_LOG_INFO( "[%d] %s -> %s\r\n",
              order++,
              (uint32_t) state_to_str[lastState],
              (uint32_t) state_to_str[currentState]);
  NRF_LOG_FLUSH();

  nextState =  STATE_ON_RESET;
  lastState = currentState;
  return nextState;
}
