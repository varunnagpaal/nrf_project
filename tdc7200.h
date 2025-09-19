#ifndef SENSOR_TDC7200_REGISTER_MAP_H_
#define SENSOR_TDC7200_REGISTER_MAP_H_

#include <stdint.h>
#include "nrf51.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief TDC7200 sensor register map
  */
typedef struct _sensor_TDC7200_type{            /*!< (@) TDC7200 Register Map Structure                */
  /* Status Registers */
  __IOM uint8_t CONFIG1;                            /*!< (@ 0x00) Configuration Register 1.                */
  __IOM uint8_t CONFIG2;                            /*!< (@ 0x01) Configuration Register 2.                */
  __IOM uint8_t INT_STATUS;                         /*!< (@ 0x02) Interrupt Status Register.               */
  __IOM uint8_t INT_MASK;                           /*!< (@ 0x03) Interrupt Mask Register2.                */
  
  __IOM uint8_t COARSE_CNTR_OVF_H;                  /*!< (@ 0x04) Coarse Counter Overflow Value High.       */
  __IOM uint8_t COARSE_CNTR_OVF_L;                  /*!< (@ 0x05) Coarse Counter Overflow Value Low.        */
  __IOM uint8_t CLOCK_CNTR_OVF_H;                   /*!< (@ 0x06) CLOCK Counter Overflow Value High.        */
  __IOM uint8_t CLOCK_CNTR_OVF_L;                   /*!< (@ 0x07) CLOCK Counter Overflow Value Low.         */
  __IOM uint8_t CLOCK_CNTR_STOP_MASK_H;             /*!< (@ 0x08) CLOCK Counter STOP Mask High.             */
  __IOM uint8_t CLOCK_CNTR_STOP_MASK_L;             /*!< (@ 0x09) CLOCK Counter STOP Mask Low.              */

  /* Reserved */
  __IM uint8_t REG_RESERVED_0[6];                   /*!< (@ 0x0A) to (@ 0x0F) Reserved.                     */
  
  __IM uint8_t TIME1;                               /*!< (@ 0x10) Measured Time 1.                          */
  __IM uint8_t CLOCK_COUNT1;                        /*!< (@ 0x11) CLOCK Counter Value 1.                    */
  __IM uint8_t TIME2;                               /*!< (@ 0x12) Measured Time 2.                          */
  __IM uint8_t CLOCK_COUNT2;                        /*!< (@ 0x13) CLOCK Counter Value 2.                    */
  __IM uint8_t TIME3;                               /*!< (@ 0x14) Measured Time 3.                          */
  __IM uint8_t CLOCK_COUNT3;                        /*!< (@ 0x15) CLOCK Counter Value 3.                    */
  __IM uint8_t TIME4;                               /*!< (@ 0x16) Measured Time 4.                          */
  __IM uint8_t CLOCK_COUNT4;                        /*!< (@ 0x17) CLOCK Counter Value 4.                    */
  __IM uint8_t TIME5;                               /*!< (@ 0x18) Measured Time 5.                          */
  __IM uint8_t CLOCK_COUNT5;                        /*!< (@ 0x19) CLOCK Counter Value 5.                    */  
  __IM uint8_t TIME6;                               /*!< (@ 0x1A) Measured Time 6.                          */
  __IM uint8_t CALIBRATION1;                        /*!< (@ 0x1B) Calibration 1, 1 CLOCK Period.            */
  __IM uint8_t CALIBRATION2;                        /*!< (@ 0x1C) Calibration 2, 2/10/20/40 CLOCK Periods.  */  
} SENSOR_TDC7200_TYPE;                          /*!< Size = 256 (0xFF)                                  */

/* TDC7200 declaration */
#define SENSOR_TDC7200_BASE                              (0x00)
#define SENSOR_TDC7200_INST0                             ((SENSOR_TDC7200_TYPE *) SENSOR_TDC7200_BASE)
#define SENSOR_TDC7200_REG_ADDR(REG)                     (&(SENSOR_TDC7200_INST0->REG))

/* TDC7200 Register: CONFIG1 Configuration Register 1 (0x00) */
typedef union
{
  struct
  {
    uint8_t START_MEAS:1;                       /*!< bit: 0                               */
    uint8_t MEAS_MODE:2;                        /*!< bit: 1..2                            */
    uint8_t START_EDGE:1;                       /*!< bit: 3                               */
    uint8_t STOP_EDGE:1;                        /*!< bit: 4                               */
    uint8_t TRIGG_EDGE:1;                       /*!< bit: 5                               */
    uint8_t PARITY_EN:1;                        /*!< bit: 6                               */
    uint8_t FORCE_CAL:1;                        /*!< bit: 7                               */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} TDC7200_REG_CONFIG1_Type;

/* CONFIG1 Configuration Register 1 (0x00) Mask and Bit Definitions */
#define TDC7200_REG_CONFIG1_FORCE_CAL_Pos               (7)
#define TDC7200_REG_CONFIG1_FORCE_CAL_Msk               (1 << TDC7200_REG_CONFIG1_FORCE_CAL_Pos)
#define TDC7200_REG_CONFIG1_FORCE_CAL_DISABLE           (0)
#define TDC7200_REG_CONFIG1_FORCE_CAL_ENABLE            (1)

#define TDC7200_REG_CONFIG1_PARITY_EN_Pos               (6)
#define TDC7200_REG_CONFIG1_PARITY_EN_Msk               (1 << TDC7200_REG_CONFIG1_PARITY_EN_Pos)
#define TDC7200_REG_CONFIG1_PARITY_EN_DISABLE           (0)
#define TDC7200_REG_CONFIG1_PARITY_EN_ENABLE            (1)

#define TDC7200_REG_CONFIG1_TRIGG_EDGE_Pos              (5)
#define TDC7200_REG_CONFIG1_TRIGG_EDGE_Msk              (1 << TDC7200_REG_CONFIG1_TRIGG_EDGE_Pos)
#define TDC7200_REG_CONFIG1_TRIGG_EDGE_RISING           (0)
#define TDC7200_REG_CONFIG1_TRIGG_EDGE_FALLING          (1)

#define TDC7200_REG_CONFIG1_STOP_EDGE_Pos               (4)
#define TDC7200_REG_CONFIG1_STOP_EDGE_Msk               (1 << TDC7200_REG_CONFIG1_STOP_EDGE_Pos)
#define TDC7200_REG_CONFIG1_STOP_EDGE_RISING            (0)
#define TDC7200_REG_CONFIG1_STOP_EDGE_FALLING           (1)

#define TDC7200_REG_CONFIG1_START_EDGE_Pos              (3)
#define TDC7200_REG_CONFIG1_START_EDGE_Msk              (1 << TDC7200_REG_CONFIG1_START_EDGE_Pos)
#define TDC7200_REG_CONFIG1_START_EDGE_RISING           (0)
#define TDC7200_REG_CONFIG1_START_EDGE_FALLING          (1)

#define TDC7200_REG_CONFIG1_MEAS_MODE_Pos               (1)
#define TDC7200_REG_CONFIG1_MEAS_MODE_Msk               (3 << TDC7200_REG_CONFIG1_MEAS_MODE_Pos)
#define TDC7200_REG_CONFIG1_MEAS_MODE_1                 (0)
#define TDC7200_REG_CONFIG1_MEAS_MODE_2                 (1)
#define TDC7200_REG_CONFIG1_MEAS_MODE_RES_1             (2)
#define TDC7200_REG_CONFIG1_MEAS_MODE_RES_2             (3)

#define TDC7200_REG_CONFIG1_START_MEAS_Pos              (0)
#define TDC7200_REG_CONFIG1_START_MEAS_Msk              (1 << TDC7200_REG_CONFIG1_START_MEAS_Pos)
#define TDC7200_REG_CONFIG1_START_MEAS_NoChange         (0)
#define TDC7200_REG_CONFIG1_START_MEAS_New              (1)

/* TDC7200 Register: CONFIG2 Configuration Register 1 (0x01) */
typedef union
{
  struct
  {
    uint8_t NUM_STOP:3;                         /*!< bit: 0..2                               */
    uint8_t AVG_CYCLES:3;                       /*!< bit: 3..5                            */
    uint8_t CALIBRATION2_PERIODS:2;             /*!< bit: 6..7                               */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} TDC7200_REG_CONFIG2_Type;

/* CONFIG1 Configuration Register 2 (0x01) Mask and Bit Definitions */
#define TDC7200_REG_CONFIG2_CALIBRATION2_PERIODS_Pos              (6)
#define TDC7200_REG_CONFIG2_CALIBRATION2_PERIODS_Msk              (3 << TDC7200_REG_CONFIG2_CALIBRATION2_PERIODS_Pos)
#define TDC7200_REG_CONFIG2_CALIBRATION2_PERIODS_2cc              (0)
#define TDC7200_REG_CONFIG2_CALIBRATION2_PERIODS_10cc             (1)
#define TDC7200_REG_CONFIG2_CALIBRATION2_PERIODS_20cc             (2)
#define TDC7200_REG_CONFIG2_CALIBRATION2_PERIODS_40cc             (3)

#define TDC7200_REG_CONFIG2_AVG_CYCLES_Pos              (3)
#define TDC7200_REG_CONFIG2_AVG_CYCLES_Msk              (7 << TDC7200_REG_CONFIG2_AVG_CYCLES_Pos)
#define TDC7200_REG_CONFIG2_AVG_CYCLES_1c               (0)
#define TDC7200_REG_CONFIG2_AVG_CYCLES_2c               (1)
#define TDC7200_REG_CONFIG2_AVG_CYCLES_4c               (2)
#define TDC7200_REG_CONFIG2_AVG_CYCLES_8c               (3)
#define TDC7200_REG_CONFIG2_AVG_CYCLES_16c              (4)
#define TDC7200_REG_CONFIG2_AVG_CYCLES_32c              (5)
#define TDC7200_REG_CONFIG2_AVG_CYCLES_64c              (6)
#define TDC7200_REG_CONFIG2_AVG_CYCLES_128c             (7)

#define TDC7200_REG_CONFIG2_NUM_STOP_Pos                (0)
#define TDC7200_REG_CONFIG2_NUM_STOP_Msk                (7 << TDC7200_REG_CONFIG2_NUM_STOP_Pos)
#define TDC7200_REG_CONFIG2_NUM_STOP_1                  (0)
#define TDC7200_REG_CONFIG2_NUM_STOP_2                  (1)
#define TDC7200_REG_CONFIG2_NUM_STOP_3                  (2)
#define TDC7200_REG_CONFIG2_NUM_STOP_4                  (3)
#define TDC7200_REG_CONFIG2_NUM_STOP_5                  (4)
#define TDC7200_REG_CONFIG2_NUM_STOP_NoEffect_1         (5)
#define TDC7200_REG_CONFIG2_NUM_STOP_NoEffect_2         (6)
#define TDC7200_REG_CONFIG2_NUM_STOP_NoEffect_3         (7)

/* TDC7200 Register: Interrupt Status Register (0x02) */
typedef union
{
  struct
  {
    uint8_t NEW_MEAS_INT:1;                     /*!< bit: 0                               */
    uint8_t COARSE_CNTR_OVF_INT:1;              /*!< bit: 1                               */
    uint8_t CLOCK_CNTR_OVF_INT:1;               /*!< bit: 2                               */
    uint8_t MEAS_STARTED_FLAG:1;                /*!< bit: 3                               */
    uint8_t MEAS_COMPLETE_FLAG:1;               /*!< bit: 4                               */
    uint8_t _reserved:3;                        /*!< bit: 5..7                            */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} TDC7200_REG_INT_STATUS_Type;

/* Interrupt Status Register (0x02) Mask and Bit Definitions */
#define TDC7200_REG_INT_STATUS_MEAS_COMPLETE_FLAG_Pos     (4)
#define TDC7200_REG_INT_STATUS_MEAS_COMPLETE_FLAG_Msk     (1 << TDC7200_REG_INT_STATUS_MEAS_COMPLETE_FLAG_Pos)
#define TDC7200_REG_INT_STATUS_MEAS_COMPLETE_FLAG_No      (0)
#define TDC7200_REG_INT_STATUS_MEAS_COMPLETE_FLAG_Yes     (1)

#define TDC7200_REG_INT_STATUS_MEAS_STARTED_FLAG_Pos      (3)
#define TDC7200_REG_INT_STATUS_MEAS_STARTED_FLAG_Msk      (1 << TDC7200_REG_INT_STATUS_MEAS_STARTED_FLAG_Pos)
#define TDC7200_REG_INT_STATUS_MEAS_STARTED_FLAG_No       (0)
#define TDC7200_REG_INT_STATUS_MEAS_STARTED_FLAG_Yes      (1)

#define TDC7200_REG_INT_STATUS_CLOCK_CNTR_OVF_INT_Pos     (2)
#define TDC7200_REG_INT_STATUS_CLOCK_CNTR_OVF_INT_Msk     (1 << TDC7200_REG_INT_STATUS_CLOCK_CNTR_OVF_INT_Pos)
#define TDC7200_REG_INT_STATUS_CLOCK_CNTR_OVF_INT_No      (0)
#define TDC7200_REG_INT_STATUS_CLOCK_CNTR_OVF_INT_Yes     (1)

#define TDC7200_REG_INT_STATUS_COARSE_CNTR_OVF_INT_Pos    (1)
#define TDC7200_REG_INT_STATUS_COARSE_CNTR_OVF_INT_Msk    (1 << TDC7200_REG_INT_STATUS_COARSE_CNTR_OVF_INT_Pos)
#define TDC7200_REG_INT_STATUS_COARSE_CNTR_OVF_INT_No     (0)
#define TDC7200_REG_INT_STATUS_COARSE_CNTR_OVF_INT_Yes    (1)

#define TDC7200_REG_INT_STATUS_NEW_MEAS_INT_Pos           (1)
#define TDC7200_REG_INT_STATUS_NEW_MEAS_INT_Msk           (1 << TDC7200_REG_INT_STATUS_NEW_MEAS_INT_Pos)
#define TDC7200_REG_INT_STATUS_NEW_MEAS_INT_No            (0)
#define TDC7200_REG_INT_STATUS_NEW_MEAS_INT_Yes           (1)

/* TDC7200 Register: Interrupt Mask Register (0x03) */
typedef union
{
  struct
  {
    uint8_t NEW_MEAS_MASK:1;                    /*!< bit: 0                               */
    uint8_t COARSE_CNTR_OVF_MASK:1;             /*!< bit: 1                               */
    uint8_t CLOCK_CNTR_OVF_MASK:1;              /*!< bit: 2                               */
    uint8_t _reserved:4;                        /*!< bit: 3..7                            */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} TDC7200_REG_INT_MASK_Type;

/* Interrupt Mask Register (0x03)) Mask and Bit Definitions */
#define TDC7200_REG_INT_MASK_CLOCK_CNTR_OVF_MASK_Pos     (2)
#define TDC7200_REG_INT_MASK_CLOCK_CNTR_OVF_MASK_Msk     (1 << TDC7200_REG_INT_MASK_CLOCK_CNTR_OVF_MASK_Pos)
#define TDC7200_REG_INT_MASK_CLOCK_CNTR_OVF_MASK_Disable (0)
#define TDC7200_REG_INT_MASK_CLOCK_CNTR_OVF_MASK_Enable  (1)

#define TDC7200_REG_INT_MASK_COARSE_CNTR_OVF_MASK_Pos     (1)
#define TDC7200_REG_INT_MASK_COARSE_CNTR_OVF_MASK_Msk     (1 << TDC7200_REG_INT_MASK_COARSE_CNTR_OVF_MASK_Pos)
#define TDC7200_REG_INT_MASK_COARSE_CNTR_OVF_MASK_Disable (0)
#define TDC7200_REG_INT_MASK_COARSE_CNTR_OVF_MASK_Enable  (1)

#define TDC7200_REG_INT_MASK_NEW_MEAS_MASK_Pos            (0)
#define TDC7200_REG_INT_MASK_NEW_MEAS_MASK_Msk            (1 << TDC7200_REG_INT_MASK_NEW_MEAS_MASK_Pos)
#define TDC7200_REG_INT_MASK_NEW_MEAS_MASK_Disable        (0)
#define TDC7200_REG_INT_MASK_NEW_MEAS_MASK_Enable         (1)

/* TDC7200 Register: Coarse Counter Overflow High Value Register (0x04) */
typedef union
{
  struct
  {
    uint8_t COARSE_CNTR_OVF_H:8;                /*!< bit: 0..7                            */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} TDC7200_REG_COARSE_CNTR_OVF_H_Type;

/* Coarse Counter Overflow High Value Register (0x04) Mask and Bit Definitions */
#define TDC7200_REG_COARSE_CNTR_OVF_H_Pos     (0)
#define TDC7200_REG_COARSE_CNTR_OVF_H_Msk     (0xFF << TDC7200_REG_COARSE_CNTR_OVF_H_Pos)

/* TDC7200 Register: Coarse Counter Overflow Low Value Register (0x05) */
typedef union
{
  struct
  {
    uint8_t COARSE_CNTR_OVF_L:8;                /*!< bit: 0..7                            */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} TDC7200_REG_COARSE_CNTR_OVF_L_Type;

/* Coarse Counter Overflow Low Value Register (0x05) Mask and Bit Definitions */
#define TDC7200_REG_COARSE_CNTR_OVF_L_Pos     (0)
#define TDC7200_REG_COARSE_CNTR_OVF_L_Msk     (0xFF << TDC7200_REG_COARSE_CNTR_OVF_L_Pos)

/* TDC7200 Register: Clock Counter Overflow High Register (0x06) */
typedef union
{
  struct
  {
    uint8_t CLOCK_CNTR_OVF_H:8;                /*!< bit: 0..7                            */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} TDC7200_REG_CLOCK_CNTR_OVF_H_Type;

/* Clock Counter Overflow High Register (0x06) Mask and Bit Definitions */
#define TDC7200_REG_CLOCK_CNTR_OVF_H_Pos     (0)
#define TDC7200_REG_CLOCK_CNTR_OVF_H_Msk     (0xFF << TDC7200_REG_CLOCK_CNTR_OVF_H_Pos)

/* TDC7200 Register: Clock Counter Overflow Low Register (0x07) */
typedef union
{
  struct
  {
    uint8_t CLOCK_CNTR_OVF_L:8;                /*!< bit: 0..7                            */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} TDC7200_REG_CLOCK_CNTR_OVF_L_Type;

/* Clock Counter Overflow Low Register (0x07) Mask and Bit Definitions */
#define TDC7200_REG_CLOCK_CNTR_OVF_L_Pos     (0)
#define TDC7200_REG_CLOCK_CNTR_OVF_L_Msk     (0xFF << TDC7200_REG_CLOCK_CNTR_OVF_L_Pos)

/* TDC7200 Register: CLOCK Counter STOP Mask High Value Register (0x08) */
typedef union
{
  struct
  {
    uint8_t CLOCK_CNTR_STOP_MASK_H:8;                /*!< bit: 0..7                            */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} TDC7200_REG_CLOCK_CNTR_STOP_MASK_H_Type;

/* CLOCK Counter STOP Mask High Value Register (0x08) Mask and Bit Definitions */
#define TDC7200_REG_CLOCK_CNTR_STOP_MASK_H_Pos     (0)
#define TDC7200_REG_CLOCK_CNTR_STOP_MASK_H_Msk     (0xFF << TDC7200_REG_CLOCK_CNTR_STOP_MASK_H_Pos)

/* TDC7200 Register: CLOCK Counter STOP Mask Low Value Register (0x09) */
typedef union
{
  struct
  {
    uint8_t CLOCK_CNTR_STOP_MASK_L:8;                /*!< bit: 0..7                            */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} TDC7200_REG_CLOCK_CNTR_STOP_MASK_L_Type;

/* CLOCK Counter STOP Mask Low Value Register (0x09) Mask and Bit Definitions */
#define TDC7200_REG_CLOCK_CNTR_STOP_MASK_L_Pos     (0)
#define TDC7200_REG_CLOCK_CNTR_STOP_MASK_L_Msk     (0xFF << TDC7200_REG_CLOCK_CNTR_STOP_MASK_L_Pos)

/* TDC7200 Register: Time 1 Register (0x10) */
typedef union
{
  struct
  {
    uint32_t MEAS_RESULT:23;                    /*!< bit: 0..22                           */
    uint32_t PARITY_BIT:1;                      /*!< bit: 23                              */
    uint32_t _unused:8;                         /*!< bit: 28..31                          */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint32_t reg_word;                            /*!< Type used for word level access      */
} TDC7200_REG_TIME1_Type;

/* Time 1 Register (0x10) Mask and Bit Definitions */
#define TDC7200_REG_TIME1_MEAS_RESULT_Pos     (0)
#define TDC7200_REG_TIME1_MEAS_RESULT_Msk     (0x3FFFFF << TDC7200_REG_TIME1_MEAS_RESULT_Pos)

#define TDC7200_REG_TIME1_PARITY_BIT_Pos     (23)
#define TDC7200_REG_TIME1_PARITY_BIT_Msk     (1 << TDC7200_REG_TIME1_PARITY_BIT_Pos)

/* TDC7200 Register: Clock Count 1 Register (0x11) */
typedef union
{
  struct
  {
    uint32_t MEAS_RESULT:16;                    /*!< bit: 0..15                           */
    uint32_t _unused0:7;                        /*!< bit: 16..22                          */
    uint32_t PARITY_BIT:1;                      /*!< bit: 23                              */
    uint32_t _unused1:8;                        /*!< bit: 24..31                          */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint32_t reg_word;                            /*!< Type used for word level access      */
} TDC7200_CLOCK_COUNT1_Type;

/* Clock Count 1 Register (0x11) Mask and Bit Definitions */
#define TDC7200_CLOCK_COUNT1_MEAS_RESULT_Pos     (0)
#define TDC7200_CLOCK_COUNT1_MEAS_RESULT_Msk     (0xFFFF << TDC7200_CLOCK_COUNT1_MEAS_RESULT_Pos)

#define TDC7200_CLOCK_COUNT1_PARITY_BIT_Pos     (23)
#define TDC7200_CLOCK_COUNT1_PARITY_BIT_Msk     (1 << TDC7200_CLOCK_COUNT1_PARITY_BIT_Pos)

/* TDC7200 Register: Time 2 Register (0x12) */
typedef union
{
  struct
  {
    uint32_t MEAS_RESULT:23;                    /*!< bit: 0..22                           */
    uint32_t PARITY_BIT:1;                      /*!< bit: 23                              */
    uint32_t _unused:8;                         /*!< bit: 28..31                          */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint32_t reg_word;                            /*!< Type used for word level access      */
} TDC7200_REG_TIME2_Type;

/* Time 2 Register (0x12) Mask and Bit Definitions */
#define TDC7200_REG_TIME2_MEAS_RESULT_Pos     (0)
#define TDC7200_REG_TIME2_MEAS_RESULT_Msk     (0x3FFFFF << TDC7200_REG_TIME2_MEAS_RESULT_Pos)

#define TDC7200_REG_TIME2_PARITY_BIT_Pos     (23)
#define TDC7200_REG_TIME2_PARITY_BIT_Msk     (1 << TDC7200_REG_TIME2_PARITY_BIT_Pos)

/* TDC7200 Register: Clock Count 2 Register (0x13) */
typedef union
{
  struct
  {
    uint32_t MEAS_RESULT:16;                    /*!< bit: 0..15                           */
    uint32_t _unused0:7;                        /*!< bit: 16..22                          */
    uint32_t PARITY_BIT:1;                      /*!< bit: 23                              */
    uint32_t _unused1:8;                        /*!< bit: 24..31                          */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint32_t reg_word;                            /*!< Type used for word level access      */
} TDC7200_CLOCK_COUNT2_Type;

/* Clock Count 2 Register (0x13) Mask and Bit Definitions */
#define TDC7200_CLOCK_COUNT2_MEAS_RESULT_Pos     (0)
#define TDC7200_CLOCK_COUNT2_MEAS_RESULT_Msk     (0xFFFF << TDC7200_CLOCK_COUNT2_MEAS_RESULT_Pos)

#define TDC7200_CLOCK_COUNT2_PARITY_BIT_Pos     (23)
#define TDC7200_CLOCK_COUNT2_PARITY_BIT_Msk     (1 << TDC7200_CLOCK_COUNT2_PARITY_BIT_Pos)





/* TDC7200 Register: Time 3 Register (0x14) */
typedef union
{
  struct
  {
    uint32_t MEAS_RESULT:23;                    /*!< bit: 0..22                           */
    uint32_t PARITY_BIT:1;                      /*!< bit: 23                              */
    uint32_t _unused:8;                         /*!< bit: 28..31                          */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint32_t reg_word;                            /*!< Type used for word level access      */
} TDC7200_REG_TIME3_Type;

/* Time 3 Register (0x14) Mask and Bit Definitions */
#define TDC7200_REG_TIME3_MEAS_RESULT_Pos     (0)
#define TDC7200_REG_TIME3_MEAS_RESULT_Msk     (0x3FFFFF << TDC7200_REG_TIME3_MEAS_RESULT_Pos)

#define TDC7200_REG_TIME3_PARITY_BIT_Pos      (23)
#define TDC7200_REG_TIME3_PARITY_BIT_Msk      (1 << TDC7200_REG_TIME3_PARITY_BIT_Pos)

/* TDC7200 Register: Clock Count 3 Register (0x15) */
typedef union
{
  struct
  {
    uint32_t MEAS_RESULT:16;                    /*!< bit: 0..15                           */
    uint32_t _unused0:7;                        /*!< bit: 16..22                          */
    uint32_t PARITY_BIT:1;                      /*!< bit: 23                              */
    uint32_t _unused1:8;                        /*!< bit: 24..31                          */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint32_t reg_word;                            /*!< Type used for word level access      */
} TDC7200_CLOCK_COUNT3_Type;

/* Clock Count 3 Register (0x15) Mask and Bit Definitions */
#define TDC7200_CLOCK_COUNT3_MEAS_RESULT_Pos     (0)
#define TDC7200_CLOCK_COUNT3_MEAS_RESULT_Msk     (0xFFFF << TDC7200_CLOCK_COUN3_MEAS_RESULT_Pos)

#define TDC7200_CLOCK_COUNT3_PARITY_BIT_Pos     (23)
#define TDC7200_CLOCK_COUNT3_PARITY_BIT_Msk     (1 << TDC7200_CLOCK_COUNT3_PARITY_BIT_Pos)




/* TDC7200 Register: Time 4 Register (0x16) */
typedef union
{
  struct
  {
    uint32_t MEAS_RESULT:23;                    /*!< bit: 0..22                           */
    uint32_t PARITY_BIT:1;                      /*!< bit: 23                              */
    uint32_t _unused:8;                         /*!< bit: 28..31                          */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint32_t reg_word;                            /*!< Type used for word level access      */
} TDC7200_REG_TIME4_Type;

/* Time 4 Register (0x16) Mask and Bit Definitions */
#define TDC7200_REG_TIME4_MEAS_RESULT_Pos     (0)
#define TDC7200_REG_TIME4_MEAS_RESULT_Msk     (0x3FFFFF << TDC7200_REG_TIME4_MEAS_RESULT_Pos)

#define TDC7200_REG_TIME4_PARITY_BIT_Pos      (23)
#define TDC7200_REG_TIME4_PARITY_BIT_Msk      (1 << TDC7200_REG_TIME4_PARITY_BIT_Pos)

/* TDC7200 Register: Clock Count 4 Register (0x17) */
typedef union
{
  struct
  {
    uint32_t MEAS_RESULT:16;                    /*!< bit: 0..15                           */
    uint32_t _unused0:7;                        /*!< bit: 16..22                          */
    uint32_t PARITY_BIT:1;                      /*!< bit: 23                              */
    uint32_t _unused1:8;                        /*!< bit: 24..31                          */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint32_t reg_word;                            /*!< Type used for word level access      */
} TDC7200_CLOCK_COUNT4_Type;

/* Clock Count 4 Register (0x17) Mask and Bit Definitions */
#define TDC7200_CLOCK_COUNT4_MEAS_RESULT_Pos     (0)
#define TDC7200_CLOCK_COUNT4_MEAS_RESULT_Msk     (0xFFFF << TDC7200_CLOCK_COUN4_MEAS_RESULT_Pos)

#define TDC7200_CLOCK_COUNT4_PARITY_BIT_Pos     (23)
#define TDC7200_CLOCK_COUNT4_PARITY_BIT_Msk     (1 << TDC7200_CLOCK_COUNT4_PARITY_BIT_Pos)




/* TDC7200 Register: Time 5 Register (0x18) */
typedef union
{
  struct
  {
    uint32_t MEAS_RESULT:23;                    /*!< bit: 0..22                           */
    uint32_t PARITY_BIT:1;                      /*!< bit: 23                              */
    uint32_t _unused:8;                         /*!< bit: 28..31                          */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint32_t reg_word;                            /*!< Type used for word level access      */
} TDC7200_REG_TIME5_Type;

/* Time 5 Register (0x18) Mask and Bit Definitions */
#define TDC7200_REG_TIME5_MEAS_RESULT_Pos     (0)
#define TDC7200_REG_TIME5_MEAS_RESULT_Msk     (0x3FFFFF << TDC7200_REG_TIME5_MEAS_RESULT_Pos)

#define TDC7200_REG_TIME5_PARITY_BIT_Pos      (23)
#define TDC7200_REG_TIME5_PARITY_BIT_Msk      (1 << TDC7200_REG_TIME5_PARITY_BIT_Pos)

/* TDC7200 Register: Clock Count 5 Register (0x19) */
typedef union
{
  struct
  {
    uint32_t MEAS_RESULT:16;                    /*!< bit: 0..15                           */
    uint32_t _unused0:7;                        /*!< bit: 16..22                          */
    uint32_t PARITY_BIT:1;                      /*!< bit: 23                              */
    uint32_t _unused1:8;                        /*!< bit: 24..31                          */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint32_t reg_word;                            /*!< Type used for word level access      */
} TDC7200_CLOCK_COUNT5_Type;

/* Clock Count 4 Register (0x19) Mask and Bit Definitions */
#define TDC7200_CLOCK_COUNT5_MEAS_RESULT_Pos     (0)
#define TDC7200_CLOCK_COUNT5_MEAS_RESULT_Msk     (0xFFFF << TDC7200_CLOCK_COUNT5_MEAS_RESULT_Pos)

#define TDC7200_CLOCK_COUNT5_PARITY_BIT_Pos     (23)
#define TDC7200_CLOCK_COUNT5_PARITY_BIT_Msk     (1 << TDC7200_CLOCK_COUNT5_PARITY_BIT_Pos)


/* TDC7200 Register: Time 6 Register (0x1A) */
typedef union
{
  struct
  {
    uint32_t MEAS_RESULT:23;                    /*!< bit: 0..22                           */
    uint32_t PARITY_BIT:1;                      /*!< bit: 23                              */
    uint32_t _unused:8;                         /*!< bit: 28..31                          */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint32_t reg_word;                            /*!< Type used for word level access      */
} TDC7200_REG_TIME6_Type;

/* Time 6 Register (0x1A) Mask and Bit Definitions */
#define TDC7200_REG_TIME6_MEAS_RESULT_Pos     (0)
#define TDC7200_REG_TIME6_MEAS_RESULT_Msk     (0x3FFFFF << TDC7200_REG_TIME6_MEAS_RESULT_Pos)

#define TDC7200_REG_TIME6_PARITY_BIT_Pos      (23)
#define TDC7200_REG_TIME6_PARITY_BIT_Msk      (1 << TDC7200_REG_TIME6_PARITY_BIT_Pos)

/* TDC7200 Register: Calibration 1 Register (0x1B) */
typedef union
{
  struct
  {
    uint32_t MEAS_RESULT:23;                    /*!< bit: 0..22                           */
    uint32_t PARITY_BIT:1;                      /*!< bit: 23                              */
    uint32_t _unused:8;                         /*!< bit: 28..31                          */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint32_t reg_word;                            /*!< Type used for word level access      */
} TDC7200_REG_CALIBRATION1_Type;

/* Calibration 1 Register (0x1B) Mask and Bit Definitions */
#define TDC7200_REG_CALIBRATION1_MEAS_RESULT_Pos     (0)
#define TDC7200_REG_CALIBRATION1_MEAS_RESULT_Msk     (0x3FFFFF << TDC7200_REG_CALIBRATION1_MEAS_RESULT_Pos)

#define TDC7200_REG_CALIBRATION1_PARITY_BIT_Pos      (23)
#define TDC7200_REG_CALIBRATION1_PARITY_BIT_Msk      (1 << TDC7200_REG_CALIBRATION1_PARITY_BIT_Pos)

/* TDC7200 Register: Calibration 2 Register (0x1C) */
typedef union
{
  struct
  {
    uint32_t MEAS_RESULT:23;                    /*!< bit: 0..22                           */
    uint32_t PARITY_BIT:1;                      /*!< bit: 23                              */
    uint32_t _unused:8;                         /*!< bit: 28..31                          */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint32_t reg_word;                            /*!< Type used for word level access      */
} TDC7200_REG_CALIBRATION2_Type;

/* Calibration 2 Register (0x1C) Mask and Bit Definitions */
#define TDC7200_REG_CALIBRATION2_MEAS_RESULT_Pos     (0)
#define TDC7200_REG_CALIBRATION2_MEAS_RESULT_Msk     (0x3FFFFF << TDC7200_REG_CALIBRATION2_MEAS_RESULT_Pos)

#define TDC7200_REG_CALIBRATION2_PARITY_BIT_Pos      (23)
#define TDC7200_REG_CALIBRATION2_PARITY_BIT_Msk      (1 << TDC7200_REG_CALIBRATION2_PARITY_BIT_Pos)

#ifdef __cplusplus
}
#endif

#endif  /* SENSOR_TDC7200_REGISTER_MAP_H_ */
