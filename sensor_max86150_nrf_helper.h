#ifndef SENSOR_MAX86150_NRF_HELPER_H_
#define SENSOR_MAX86150_NRF_HELPER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**@brief Set a bit in the word of data type T (uint8_t, uint16_t, uint32_t etc.).
 *
 * @param[in] W  Word whose bit is being set.
 * @param[in] B  Bit number in the word to be set.
 */
#define SET_BIT(W,B,T)      ((W) |= (T)(1U << (B)))

/**@brief Clears a bit in word of data type T (uint8_t, uint16_t, uint32_t etc.).
 *
 * @param[in] W   Word whose bit is to be cleared.
 * @param[in] B   Bit number in the word to be cleared.
 * @param[in] T   Data type of word W
 */
#define CLR_BIT(W, B, T)    ((W) &= (~((T)1 << (B))))

/**@brief Checks if a bit is set.
 *
 * @param[in] W   Word whose bit is to be checked.
 * @param[in] B   Bit number in the word to be checked.
 *
 * @retval 1 if bit is set.
 * @retval 0 if bit is not set.
 */
#define IS_SET(W,B)         (((W) >> (B)) & 1)

/**@brief Checks if a bit is set in a word of data type (uint8_t, uint16_t, uint32_t etc.).
 *
 * @param[in] W   Word of data type T whose bit is to be checked.
 * @param[in] MSK Mask of data type T
 * @param[in] T   Data type T
 *
 * @retval 1 if bit is set.
 * @retval 0 if bit is not set.
 */
#define IS_SET_MSK(W, MSK, T)         ((T)(W) & (T)MSK)

void sensor_max86150_power_on_reset(void);
void sensor_max86150_shutdown(void);
void sensor_max86150_wakeup(void);
void sensor_max86150_fifo_flush(void);
void sensor_max86150_registers_configure(void);
void sensor_max86150_registers_read_all(void);
void sensor_max86150_get_fifo_data(volatile uint32_t * p_ppg_led1_samples,
                                       volatile uint32_t * p_ppg_led2_samples,
                                       volatile uint32_t * p_ecg_samples,
                                       volatile uint8_t  * p_num_valid_samples);

#ifdef __cplusplus
}
#endif

#endif  /* SENSOR_MAX86150_NRF_HELPER_H_ */
