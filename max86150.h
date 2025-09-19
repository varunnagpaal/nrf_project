#ifndef SENSOR_MAX86150_REGISTER_MAP_H_
#define SENSOR_MAX86150_REGISTER_MAP_H_

#include <stdint.h>
#include "nrf51.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief MAX86150 sensor register map
  */
typedef struct _sensor_max86150_type{         /*!< (@) MAX86150 Register Map Structure              */
  /* Status Registers */
  __IM  uint8_t REG_INT_STATUS_1;                 /*!< (@ 0x00) Interrupt status 1.                     */
  __IM  uint8_t REG_INT_STATUS_2;                 /*!< (@ 0x01) Interrupt status 2.                     */
  __IOM uint8_t REG_INT_ENABLE_1;                 /*!< (@ 0x02) Interrupt enable 1.                     */
  __IOM uint8_t REG_INT_ENABLE_2;                 /*!< (@ 0x03) Interrupt enable 2.                     */
  
  /* FIFO Registers */
  __IOM uint8_t REG_FIFO_WR_PTR;                  /*!< (@ 0x04) FIFO write pointer.                     */
  __IM  uint8_t REG_OVF_COUNTER;                  /*!< (@ 0x05) Overflow Counter.                       */
  __IOM uint8_t REG_FIFO_RD_PTR;                  /*!< (@ 0x06) FIFO Read pointer.                      */
  __IM  uint8_t REG_FIFO_DATA;                    /*!< (@ 0x07) FIFO Data Register.                     */
  __IOM uint8_t REG_FIFO_CONFIG;                  /*!< (@ 0x08) FIFO Configuration.                     */

  /* FIFO Data Control */
  __IOM uint8_t REG_FIFO_DATA_CONTROL_1;          /*!< (@ 0x09) FIFO Data Control Register 1.           */
  __IOM uint8_t REG_FIFO_DATA_CONTROL_2;          /*!< (@ 0x0A) FIFO Data Control Register 2.           */

  /* Reserved */
  __IM uint8_t REG_RESERVED_1[2];                 /*!< (@ 0x0B) to (@ 0x0C) Reserved 1.                 */

  /* System Control */
  __IOM uint8_t REG_SYS_CONTROL;                  /*!< (@ 0x0D) System Control.                         */

  /* PPG Configuration (PPG Subsystem) */
  __IOM uint8_t REG_PPG_CONFIG_1;                 /*!< (@ 0x0E) PPG Configuration 1.                    */
  __IOM uint8_t REG_PPG_CONFIG_2;                 /*!< (@ 0x0F) PPG Configuration 2.                    */
  __IOM uint8_t REG_PROX_INT_THRESH;              /*!< (@ 0x10) Proximity Interrupt Threshold.          */

  /* LED Pulse Amplitude (PPG Subsystem) */
  __IOM uint8_t REG_LED1_PA;                      /*!< (@ 0x11) LED1 PA.                                */
  __IOM uint8_t REG_LED2_PA;                      /*!< (@ 0x12) LED2 PA.                                */
  __IM  uint8_t REG_RESERVED_2;                   /*!< (@ 0x13) Reserved 2.                             */
  __IOM uint8_t REG_LED_RANGE;                    /*!< (@ 0x14) LED Range.                              */
  __IOM uint8_t REG_LED_PILOT_PA;                 /*!< (@ 0x15) LED PILOT PA.                           */

  /* Reserved */
  __IM  uint8_t REG_RESERVED_3[38];               /*!< (@ 0x16) to (@ 0x3B) Reserved 3.                 */

  /* ECG Configuration */
  __IOM uint8_t REG_ECG_CONFIG_1;                 /*!< (@ 0x3C) ECG Configuration 1.                    */
  __IM  uint8_t REG_RESERVED_4;                   /*!< (@ 0x3D) Reserved 4.                             */  
  __IOM uint8_t REG_ECG_CONFIG_3;                 /*!< (@ 0x3E) ECG Configuration 3.                    */

  /* Reserved */
  __IM  uint8_t REG_RESERVED_5[192];              /*!< (@ 0x3F) to (@ 0xFE) Reserved 5.                 */

  /* Part ID */
  __IM  uint8_t REG_PART_ID;                      /*!< (@ 0xFF) PART ID.                                */
} SENSOR_MAX86150_TYPE;                       /*!< Size = 256 (0xFF)                                */

/* MAX86150 I2C memory map */
#define SENSOR_MAX86150_I2C_RESPONDER_WR_ADDR              (0xBC)
#define SENSOR_MAX86150_I2C_RESPONDER_RD_ADDR              (SENSOR_MAX86150_I2C_RESPONDER_WR_ADDR | 0x01)
#define SENSOR_MAX86150_I2C_RESPONDER_ADDR                 (uint8_t)(SENSOR_MAX86150_I2C_RESPONDER_WR_ADDR >> 1) /* LSB is R/W bit. The actual 7-bit address is 7-bit MSbits */

/* MAX86150 declaration */
#define SENSOR_MAX86150_BASE                              (0x00)
#define SENSOR_MAX86150_INST0                             ((SENSOR_MAX86150_TYPE *) SENSOR_MAX86150_BASE)
#define SENSOR_MAX86150_REG_ADDR(REG)                     (&(SENSOR_MAX86150_INST0->REG))

/* MAX86150 Register: Interrupt Status 1 (0x00) */
typedef union
{
  struct
  {
    uint8_t PWR_RDY:1;                          /*!< bit: 0                               */
    uint8_t _reserved0:3;                       /*!< bit: 1..3  Reserved                  */
    uint8_t PROX_INT:1;                         /*!< bit: 4                               */
    uint8_t ALC_OVF:1;                          /*!< bit: 5                               */
    uint8_t PPG_DATA_RDY:1;                     /*!< bit: 6                               */
    uint8_t A_FULL:1;                           /*!< bit: 7                               */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_INT_STATUS_1_Type;

/* Interrupt Status 1 (0x00) Register Definitions */
#define MAX86150_REG_INT_STATUS_1_A_FULL_Pos              (7)                                                     /*!< REG_INT_STATUS_1: A_FULL field Position        */
#define MAX86150_REG_INT_STATUS_1_A_FULL_Msk              (1 << MAX86150_REG_INT_STATUS_1_A_FULL_Pos)         /*!< REG_INT_STATUS_1: A_FULL field Mask            */
#define MAX86150_REG_INT_STATUS_1_A_FULL_IsClr            (0)
#define MAX86150_REG_INT_STATUS_1_A_FULL_IsSet            (1)

#define MAX86150_REG_INT_STATUS_1_PPG_DATA_RDY_Pos        (6)                                                     /*!< REG_INT_STATUS_1: PPG_DATA_RDY field Position  */
#define MAX86150_REG_INT_STATUS_1_PPG_DATA_RDY_Msk        (1 << MAX86150_REG_INT_STATUS_1_PPG_DATA_RDY_Pos)   /*!< REG_INT_STATUS_1: PPG_DATA_RDY field Mask      */
#define MAX86150_REG_INT_STATUS_1_PPG_DATA_RDY_IsClr      (0)
#define MAX86150_REG_INT_STATUS_1_PPG_DATA_RDY_IsSet      (1)

#define MAX86150_REG_INT_STATUS_1_ALC_OVF_Pos             (5)                                                     /*!< REG_INT_STATUS_1: ALC_OVF field Position       */
#define MAX86150_REG_INT_STATUS_1_ALC_OVF_Msk             (1 << MAX86150_REG_INT_STATUS_1_ALC_OVF_Pos)        /*!< REG_INT_STATUS_1: ALC_OVF field Mask           */
#define MAX86150_REG_INT_STATUS_1_ALC_OVF_IsClr           (0)
#define MAX86150_REG_INT_STATUS_1_ALC_OVF_IsSet           (1)

#define MAX86150_REG_INT_STATUS_1_PROX_INT_Pos            (4)                                                     /*!< REG_INT_STATUS_1: PROX_INT field Position      */
#define MAX86150_REG_INT_STATUS_1_PROX_INT_Msk            (1 << MAX86150_REG_INT_STATUS_1_PROX_INT_Pos)       /*!< REG_INT_STATUS_1: PROX_INT field Mask          */
#define MAX86150_REG_INT_STATUS_1_PROX_INT_IsClr          (0)
#define MAX86150_REG_INT_STATUS_1_PROX_INT_IsSet          (1)

#define MAX86150_REG_INT_STATUS_1_PWR_RDY_Pos             (0)                                                     /*!< REG_INT_STATUS_1: PWR_RDY field Position    */
#define MAX86150_REG_INT_STATUS_1_PWR_RDY_Msk             (1 << MAX86150_REG_INT_STATUS_1_PWR_RDY_Pos)        /*!< REG_INT_STATUS_1: PWR_RDY field Mask        */
#define MAX86150_REG_INT_STATUS_1_PWR_RDY_VBATT_OK        (0)
#define MAX86150_REG_INT_STATUS_1_PWR_RDY_VBATT_LOW       (1)

/* MAX86150 Register: Interrupt Status 2 (0x01) */
typedef union
{
  struct
  {
    uint8_t _reserved0:2;                       /*!< bit: 0..1  Reserved                  */
    uint8_t ECG_DATA_RDY:1;                     /*!< bit: 2                               */
    uint8_t _reserved1:4;                       /*!< bit: 3..6  Reserved                  */
    uint8_t VDD_OOR:1;                          /*!< bit: 7                               */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_INT_STATUS_2_Type;

/* Interrupt Status 2 (0x01) Register Definitions */
#define MAX86150_REG_INT_STATUS_2_VDD_OOR_Pos             (7)                                                     /*!< REG_INT_STATUS_2: VDD_OOR field Position     */
#define MAX86150_REG_INT_STATUS_2_VDD_OOR_Msk             (1 << MAX86150_REG_INT_STATUS_2_VDD_OOR_Pos)        /*!< REG_INT_STATUS_2: VDD_OOR field Mask         */
#define MAX86150_REG_INT_STATUS_2_VDD_OOR_IsClr           (0)
#define MAX86150_REG_INT_STATUS_2_VDD_OOR_IsSet           (1)

#define MAX86150_REG_INT_STATUS_2_ECG_DATA_RDY_Pos        (2)                                                     /*!< REG_INT_STATUS_1: ECG_RDY field Position    */
#define MAX86150_REG_INT_STATUS_2_ECG_DATA_RDY_Msk        (1 << MAX86150_REG_INT_STATUS_2_ECG_DATA_RDY_Pos)   /*!< REG_INT_STATUS_1: ECG_RDY field Mask        */
#define MAX86150_REG_INT_STATUS_2_ECG_DATA_RDY_IsClr      (0)
#define MAX86150_REG_INT_STATUS_2_ECG_DATA_RDY_IsSet      (1)

/* MAX86150 Register: Interrupt Enable 1 (0x02) */
typedef union
{
  struct
  {
    uint8_t _reserved0:4;                       /*!< bit: 0..3  Reserved                  */    
    uint8_t PROX_INT_EN:1;                      /*!< bit: 4                               */
    uint8_t ALC_OVF_EN:1;                       /*!< bit: 5                               */
    uint8_t PPG_DATA_RDY_EN:1;                  /*!< bit: 6                               */
    uint8_t A_FULL_EN:1;                        /*!< bit: 7                               */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_INT_ENABLE_1_Type;

/* Interrupt Enable 1 (0x02) Register Definitions */
#define MAX86150_REG_INT_ENABLE_1_A_FULL_EN_Pos           (7)                                                         /*!< REG_INT_ENABLE_1: A_FULL_EN field Position           */
#define MAX86150_REG_INT_ENABLE_1_A_FULL_EN_Msk           (1 << MAX86150_REG_INT_ENABLE_1_A_FULL_EN_Pos)          /*!< REG_INT_ENABLE_1: A_FULL_EN field Mask               */
#define MAX86150_REG_INT_ENABLE_1_A_FULL_EN_Disable       (0)
#define MAX86150_REG_INT_ENABLE_1_A_FULL_EN_Enable        (1)

#define MAX86150_REG_INT_ENABLE_1_PPG_DATA_RDY_EN_Pos     (6)                                                         /*!< REG_INT_ENABLE_1: PPG_DATA_RDY_EN field Position     */
#define MAX86150_REG_INT_ENABLE_1_PPG_DATA_RDY_EN_Msk     (1 << MAX86150_REG_INT_ENABLE_1_PPG_DATA_RDY_EN_Pos)    /*!< REG_INT_ENABLE_1: PPG_DATA_RDY_EN field Mask         */
#define MAX86150_REG_INT_ENABLE_1_PPG_DATA_RDY_EN_Disable (0)
#define MAX86150_REG_INT_ENABLE_1_PPG_DATA_RDY_EN_Enable  (1)

#define MAX86150_REG_INT_ENABLE_1_ALC_OVF_EN_Pos          (5)                                                         /*!< REG_INT_ENABLE_1: ALC_OVF_EN field Position          */
#define MAX86150_REG_INT_ENABLE_1_ALC_OVF_EN_Msk          (1 << MAX86150_REG_INT_ENABLE_1_ALC_OVF_EN_Pos)         /*!< REG_INT_ENABLE_1: ALC_OVF_EN field Mask              */
#define MAX86150_REG_INT_ENABLE_1_ALC_OVF_EN_Disable      (0)
#define MAX86150_REG_INT_ENABLE_1_ALC_OVF_EN_Enable       (1)

#define MAX86150_REG_INT_ENABLE_1_PROX_INT_EN_Pos         (4)                                                         /*!< REG_INT_ENABLE_1: PROX_INT_EN field Position         */
#define MAX86150_REG_INT_ENABLE_1_PROX_INT_EN_Msk         (1 << MAX86150_REG_INT_ENABLE_1_PROX_INT_EN_Pos)        /*!< REG_INT_ENABLE_1: PROX_INT_EN field Mask             */
#define MAX86150_REG_INT_ENABLE_1_PROX_INT_EN_Disable     (0)
#define MAX86150_REG_INT_ENABLE_1_PROX_INT_EN_Enable      (1)

/* MAX86150 Register: Interrupt Enable 2 (0x03) */
typedef union
{
  struct
  {
    uint8_t _reserved0:2;                       /*!< bit: 0..1  Reserved                  */    
    uint8_t ECG_DATA_RDY_EN:1;                  /*!< bit: 2                               */
    uint8_t _reserved1:4;                       /*!< bit: 3..6  Reserved                  */
    uint8_t VDD_OOR_EN:1;                       /*!< bit: 7                               */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_INT_ENABLE_2_Type;

/* Interrupt Enable 2 (0x03) Register Definitions */
#define MAX86150_REG_INT_ENABLE_2_VDD_OOR_EN_Pos          (7)                                                         /*!< REG_INT_ENABLE_2: VDD_OOR_EN field Position          */
#define MAX86150_REG_INT_ENABLE_2_VDD_OOR_EN_Msk          (1 << MAX86150_REG_INT_ENABLE_2_VDD_OOR_EN_Pos)         /*!< REG_INT_ENABLE_2: VDD_OOR_EN field Mask              */
#define MAX86150_REG_INT_ENABLE_2_VDD_OOR_EN_Disable      (0)
#define MAX86150_REG_INT_ENABLE_2_VDD_OOR_EN_Enable       (1)

#define MAX86150_REG_INT_ENABLE_2_ECG_DATA_RDY_EN_Pos     (2)                                                         /*!< REG_INT_ENABLE_2: ECG_DATA_RDY_EN field Position     */
#define MAX86150_REG_INT_ENABLE_2_ECG_DATA_RDY_EN_Msk     (1 << MAX86150_REG_INT_ENABLE_2_ECG_DATA_RDY_EN_Pos)    /*!< REG_INT_ENABLE_2: ECG_DATA_RDY_EN field Mask         */
#define MAX86150_REG_INT_ENABLE_2_ECG_DATA_RDY_EN_Disable (0)
#define MAX86150_REG_INT_ENABLE_2_ECG_DATA_RDY_EN_Enable  (1)

/* MAX86150 Register: FIFO Write Pointer (0x04) */
typedef union
{
  struct
  {
    uint8_t FIFO_WR_PTR:5;                      /*!< bit: 0..4                            */    
    uint8_t _reserved0:3;                       /*!< bit: 5..7  Reserved                  */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_FIFO_WR_PTR_Type;

/* FIFO Write Pointer (0x04) Register Definitions */
#define MAX86150_REG_FIFO_WR_PTR_FIFO_WR_PTR_Pos          (0)                                                         /*!< REG_FIFO_WR_PTR: FIFO_WR_PTR field Position          */
#define MAX86150_REG_FIFO_WR_PTR_FIFO_WR_PTR_Msk          (1F << MAX86150_REG_FIFO_WR_PTR_FIFO_WR_PTR_Pos)        /*!< REG_FIFO_WR_PTR: FIFO_WR_PTR field Mask              */

/* MAX86150 Register: Overflow Counter (0x05) */
typedef union
{
  struct
  {
    uint8_t OVF_COUNTER:5;                      /*!< bit: 0..4                            */    
    uint8_t _reserved0:3;                       /*!< bit: 5..7  Reserved                  */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_OVF_COUNTER_Type;

/* Overflow Counter (0x05) Register Definitions */
#define MAX86150_REG_OVF_COUNTER_OVF_COUNTER_Pos          (0)                                                         /*!< REG_OVF_COUNTER: OVF_COUNTER field Position          */
#define MAX86150_REG_OVF_COUNTER_OVF_COUNTER_Msk          (1F << MAX86150_REG_OVF_COUNTER_OVF_COUNTER_Pos)        /*!< REG_OVF_COUNTER: OVF_COUNTER field Mask              */
#define MAX86150_REG_OVF_COUNTER_OVF_COUNTER_MAX          (0x1F)                                                      /*!< REG_OVF_COUNTER: OVF_COUNTER counts number of lost samples due to overflow (FIFO_ROLLS_ON_FULL)          */ 

/* MAX86150 Register: FIFO Read Pointer (0x06) */
typedef union
{
  struct
  {
    uint8_t FIFO_RD_PTR:5;                      /*!< bit: 0..4                            */    
    uint8_t _reserved0:3;                       /*!< bit: 5..7  Reserved                  */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_FIFO_RD_PTR_Type;

/* FIFO Read Pointer (0x06) Register Definitions */
#define MAX86150_REG_FIFO_RD_PTR_FIFO_RD_PTR_Pos          (0)                                                         /*!< REG_FIFO_RD_PTR: FIFO_RD_PTR field Position          */
#define MAX86150_REG_FIFO_RD_PTR_FIFO_RD_PTR_Msk          (1F << MAX86150_REG_FIFO_RD_PTR_FIFO_RD_PTR_Pos)        /*!< REG_FIFO_RD_PTR: FIFO_RD_PTR field Mask              */

/* MAX86150 Register: FIFO Data Register (0x07) */
typedef union
{
  struct
  {
    uint8_t FIFO_DATA:8;                        /*!< bit: 0..7                            */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_FIFO_DATA_Type;

/* FIFO Read Pointer (0x06) Register Definitions */
#define MAX86150_REG_FIFO_DATA_FIFO_DATA_Pos              (0)                                                         /*!< REG_FIFO_DATA: FIFO_DATA field Position            */
#define MAX86150_REG_FIFO_DATA_FIFO_DATA_Msk              (1F << MAX86150_REG_FIFO_DATA_FIFO_DATA_Pos)            /*!< REG_FIFO_DATA: FIFO_DATA field Mask                */

/* MAX86150 Register: FIFO Configuration (0x08) */
typedef union
{
  struct
  {
    uint8_t FIFO_A_FULL:4;                      /*!< bit: 0..3                            */
    uint8_t FIFO_ROLLS_ON_FULL:1;               /*!< bit: 4                               */
    uint8_t A_FULL_TYPE:1;                      /*!< bit: 5                               */
    uint8_t A_FULL_CLR:1;                       /*!< bit: 6                               */
    uint8_t _reserved0:1;                       /*!< bit: 7  Reserved                     */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_FIFO_CONFIG_Type;

/* FIFO Configuration (0x08) Register Definitions */
#define MAX86150_REG_FIFO_CONFIG_A_FULL_CLR_Pos           (6)                                                         /*!< REG_FIFO_CONFIG: A_FULL_CLR field Position           */
#define MAX86150_REG_FIFO_CONFIG_A_FULL_CLR_Msk           (1 << MAX86150_REG_FIFO_CONFIG_A_FULL_CLR_Pos)          /*!< REG_FIFO_CONFIG: A_FULL_CLR field Mask               */
#define MAX86150_REG_FIFO_CONFIG_A_FULL_CLR_RD_DATA_NOCLR (0)
#define MAX86150_REG_FIFO_CONFIG_A_FULL_CLR_RD_DATA_CLR   (1)

#define MAX86150_REG_FIFO_CONFIG_A_FULL_TYPE_Pos          (5)                                                         /*!< REG_FIFO_CONFIG: A_FULL_TYPE field Position          */
#define MAX86150_REG_FIFO_CONFIG_A_FULL_TYPE_Msk          (1 << MAX86150_REG_FIFO_CONFIG_A_FULL_CLR_Pos)          /*!< REG_FIFO_CONFIG: A_FULL_TYPE field Mask              */
#define MAX86150_REG_FIFO_CONFIG_A_FULL_TYPE_RPT          (0)
#define MAX86150_REG_FIFO_CONFIG_A_FULL_TYPE_ONCE         (1)

#define MAX86150_REG_FIFO_CONFIG_FIFO_ROLLS_ON_FULL_Pos   (4)                                                         /*!< REG_FIFO_CONFIG: FIFO_ROLLS_ON_FULL field Position   */
#define MAX86150_REG_FIFO_CONFIG_FIFO_ROLLS_ON_FULL_Msk   (1 << MAX86150_REG_FIFO_CONFIG_FIFO_ROLLS_ON_FULL_Pos)  /*!< REG_FIFO_CONFIG: FIFO_ROLLS_ON_FULL field Mask       */
#define MAX86150_REG_FIFO_CONFIG_FIFO_ROLLS_ON_FULL_OFF   (0)
#define MAX86150_REG_FIFO_CONFIG_FIFO_ROLLS_ON_FULL_ON    (1)

#define MAX86150_REG_FIFO_CONFIG_FIFO_A_FULL_Pos          (0)                                                         /*!< REG_FIFO_CONFIG: FIFO_A_FULL field Position          */
#define MAX86150_REG_FIFO_CONFIG_FIFO_A_FULL_Msk          (0F << MAX86150_REG_FIFO_CONFIG_FIFO_A_FULL_Pos)        /*!< REG_FIFO_CONFIG: FIFO_A_FULL field Mask              */
#define MAX86150_REG_FIFO_CONFIG_FIFO_A_FULL_FreeSpace_0  (0x0)
#define MAX86150_REG_FIFO_CONFIG_FIFO_A_FULL_FreeSpace_15 (0xF)

/* MAX86150 Register: FIFO Data Control Register 1 (0x09) */
typedef union
{
  struct
  {
    uint8_t FD1:4;                              /*!< bit: 0..3                            */
    uint8_t FD2:4;                              /*!< bit: 4..7                            */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_FIFO_DATA_CONTROL_1_Type;

/* FIFO Data Control Register 1 (0x09) Register Definitions */
#define MAX86150_REG_FIFO_DATA_CONTROL_1_FD2_Pos          (4)                                                         /*!< REG_FIFO_DATA_CONTROL_1: FD2 field Position          */
#define MAX86150_REG_FIFO_DATA_CONTROL_1_FD2_Msk          (0xF << MAX86150_REG_FIFO_DATA_CONTROL_1_FD2_Pos)       /*!< REG_FIFO_DATA_CONTROL_1: FD2 field Mask              */
#define MAX86150_REG_FIFO_DATA_CONTROL_1_FD1_Pos          (0)                                                         /*!< REG_FIFO_DATA_CONTROL_1: FD1 field Position          */
#define MAX86150_REG_FIFO_DATA_CONTROL_1_FD1_Msk          (0xF << MAX86150_REG_FIFO_DATA_CONTROL_1_FD1_Pos)       /*!< REG_FIFO_DATA_CONTROL_1: FD1 field Mask              */
#define MAX86150_REG_FIFO_DATA_CONTROL_FD_None            (0x0) /* same for FD1, FD2, FD3 and FD4 */
#define MAX86150_REG_FIFO_DATA_CONTROL_FD_PPG_LED1        (0x1) /* same for FD1, FD2, FD3 and FD4 */
#define MAX86150_REG_FIFO_DATA_CONTROL_FD_PPG_LED2        (0x2) /* same for FD1, FD2, FD3 and FD4 */
#define MAX86150_REG_FIFO_DATA_CONTROL_FD_Pilot_LED1      (0x5) /* same for FD1, FD2, FD3 and FD4 */
#define MAX86150_REG_FIFO_DATA_CONTROL_FD_Pilot_LED2      (0x6) /* same for FD1, FD2, FD3 and FD4 */
#define MAX86150_REG_FIFO_DATA_CONTROL_FD_Pilot_ECG       (0x9) /* same for FD1, FD2, FD3 and FD4 */

/* MAX86150 Register: FIFO Data Control Register 2 (0x0A) */
typedef union
{
  struct
  {
    uint8_t FD3:4;                              /*!< bit: 0..3                            */
    uint8_t FD4:4;                              /*!< bit: 4..7                            */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_FIFO_DATA_CONTROL_2_Type;

/* FIFO Data Control Register 2 (0x0A) Register Definitions */
#define MAX86150_REG_FIFO_DATA_CONTROL_2_FD4_Pos          (4)                                                         /*!< REG_FIFO_DATA_CONTROL_2: FD4 field Position          */
#define MAX86150_REG_FIFO_DATA_CONTROL_2_FD4_Msk          (0xF << MAX86150_REG_FIFO_DATA_CONTROL_2_FD4_Pos)       /*!< REG_FIFO_DATA_CONTROL_2: FD4 field Mask              */
#define MAX86150_REG_FIFO_DATA_CONTROL_2_FD3_Pos          (0)                                                         /*!< REG_FIFO_DATA_CONTROL_2: FD3 field Position          */
#define MAX86150_REG_FIFO_DATA_CONTROL_2_FD3_Msk          (0xF << MAX86150_REG_FIFO_DATA_CONTROL_2_FD3_Pos)       /*!< REG_FIFO_DATA_CONTROL_2: FD3 field Mask              */

/* MAX86150 Register: System Control (0x0D) */
typedef union
{
  struct
  {
    uint8_t RESET:1;                            /*!< bit: 0                               */
    uint8_t SHDN:1;                             /*!< bit: 1                               */
    uint8_t FIFO_EN:1;                          /*!< bit: 2                               */
    uint8_t _reserved0:5;                       /*!< bit: 3..7                            */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_SYS_CONTROL_Type;

/* System Control (0x0D) Register Definitions */
#define MAX86150_REG_SYS_CONTROL_FIFO_EN_Pos          (2)                                                             /*!< REG_FIFO_DATA_CONTROL_2: FIFO_EN field Position        */
#define MAX86150_REG_SYS_CONTROL_FIFO_EN_Msk          (1 << MAX86150_REG_SYS_CONTROL_FIFO_EN_Pos)                 /*!< REG_FIFO_DATA_CONTROL_2: FIFO_EN field Mask            */
#define MAX86150_REG_SYS_CONTROL_FIFO_EN_Disabled     (0)
#define MAX86150_REG_SYS_CONTROL_FIFO_EN_Enabled      (1)

#define MAX86150_REG_SYS_CONTROL_SHDN_Pos             (1)                                                             /*!< REG_FIFO_DATA_CONTROL_2: SHDN field Position           */
#define MAX86150_REG_SYS_CONTROL_SHDN_Msk             (1 << MAX86150_REG_SYS_CONTROL_SHDN_Pos)                    /*!< REG_FIFO_DATA_CONTROL_2: SHDN field Mask               */
#define MAX86150_REG_SYS_CONTROL_SHDN_Disabled        (0)
#define MAX86150_REG_SYS_CONTROL_SHDN_Enabled         (1)

#define MAX86150_REG_SYS_CONTROL_RESET_Pos            (0)                                                             /*!< REG_FIFO_DATA_CONTROL_2: RESET field Position          */
#define MAX86150_REG_SYS_CONTROL_RESET_Msk            (1 << MAX86150_REG_SYS_CONTROL_RESET_Pos)                   /*!< REG_FIFO_DATA_CONTROL_2: RESET field Mask              */
#define MAX86150_REG_SYS_CONTROL_RESET_Disabled       (0)
#define MAX86150_REG_SYS_CONTROL_RESET_Enabled        (1)

/* MAX86150 Register: PPG Configuration 1 (0x0E) */
typedef union
{
  struct
  {
    uint8_t PPG_LED_PW:2;                       /*!< bit: 0..1                            */
    uint8_t PPG_SR:4;                           /*!< bit: 2..5                            */
    uint8_t PPG_ADC_RGE:2;                      /*!< bit: 6..7                            */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_PPG_CONFIG_1_Type;

/* PPG Configuration 1 (0x0E) Register Definitions */
#define MAX86150_REG_PPG_CONFIG_1_PPG_ADC_RGE_Pos         (6)                                                         /*!< REG_PPG_CONFIG_1: PPG_ADC_RGE field Position           */
#define MAX86150_REG_PPG_CONFIG_1_PPG_ADC_RGE_Msk         (0x3 << MAX86150_REG_PPG_CONFIG_1_PPG_ADC_RGE_Pos)      /*!< REG_PPG_CONFIG_1: PPG_ADC_RGE field Mask               */
#define MAX86150_REG_PPG_CONFIG_1_PPG_ADC_RGE_4096        (0)
#define MAX86150_REG_PPG_CONFIG_1_PPG_ADC_RGE_8192        (1)
#define MAX86150_REG_PPG_CONFIG_1_PPG_ADC_RGE_16384       (2)
#define MAX86150_REG_PPG_CONFIG_1_PPG_ADC_RGE_32768       (3)

#define MAX86150_REG_PPG_CONFIG_1_PPG_SR_Pos              (2)                                                         /*!< REG_PPG_CONFIG_1: PPG_SR field Position                */
#define MAX86150_REG_PPG_CONFIG_1_PPG_SR_Msk              (0xF << MAX86150_REG_PPG_CONFIG_1_PPG_SR_Pos)           /*!< REG_PPG_CONFIG_1: PPG_SR field Mask                    */
#define MAX86150_REG_PPG_CONFIG_1_PPG_SR_PPS_1_SPS_10     (0)
#define MAX86150_REG_PPG_CONFIG_1_PPG_SR_PPS_1_SPS_20     (1)
#define MAX86150_REG_PPG_CONFIG_1_PPG_SR_PPS_1_SPS_50     (2)
#define MAX86150_REG_PPG_CONFIG_1_PPG_SR_PPS_1_SPS_84     (3)
#define MAX86150_REG_PPG_CONFIG_1_PPG_SR_PPS_1_SPS_100    (4)
#define MAX86150_REG_PPG_CONFIG_1_PPG_SR_PPS_1_SPS_200    (5)
#define MAX86150_REG_PPG_CONFIG_1_PPG_SR_PPS_1_SPS_400    (6)
#define MAX86150_REG_PPG_CONFIG_1_PPG_SR_PPS_1_SPS_800    (7)
#define MAX86150_REG_PPG_CONFIG_1_PPG_SR_PPS_1_SPS_1000   (8)
#define MAX86150_REG_PPG_CONFIG_1_PPG_SR_PPS_1_SPS_1600   (9)
#define MAX86150_REG_PPG_CONFIG_1_PPG_SR_PPS_1_SPS_3200   (0xA)
#define MAX86150_REG_PPG_CONFIG_1_PPG_SR_PPS_2_SPS_10     (0xB)
#define MAX86150_REG_PPG_CONFIG_1_PPG_SR_PPS_2_SPS_20     (0xC)
#define MAX86150_REG_PPG_CONFIG_1_PPG_SR_PPS_2_SPS_50     (0xD)
#define MAX86150_REG_PPG_CONFIG_1_PPG_SR_PPS_2_SPS_84     (0xE)
#define MAX86150_REG_PPG_CONFIG_1_PPG_SR_PPS_2_SPS_100    (0xF)

#define MAX86150_REG_PPG_CONFIG_1_PPG_LED_PW_Pos          (0)                                                         /*!< REG_PPG_CONFIG_1: PPG_LED_PW field Position           */
#define MAX86150_REG_PPG_CONFIG_1_PPG_LED_PW_Msk          (0x3 << MAX86150_REG_PPG_CONFIG_1_PPG_LED_PW_Pos)       /*!< REG_PPG_CONFIG_1: PPG_LED_PW field Mask               */
#define MAX86150_REG_PPG_CONFIG_1_PPG_LED_PW_US_50        (0)
#define MAX86150_REG_PPG_CONFIG_1_PPG_LED_PW_US_100       (1)
#define MAX86150_REG_PPG_CONFIG_1_PPG_LED_PW_US_200       (2)
#define MAX86150_REG_PPG_CONFIG_1_PPG_LED_PW_US_400       (3)

/* MAX86150 Register: PPG Configuration 2 (0x0F) */
typedef union
{
  struct
  {
    uint8_t SMP_AVE:3;                          /*!< bit: 0..2                            */
    uint8_t _reserved0:5;                       /*!< bit: 3..7                            */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_PPG_CONFIG_2_Type;

/* PPG Configuration 2 (0x0F) Definitions */
#define MAX86150_REG_PPG_CONFIG_2_SMP_AVE_Pos         (0)                                                             /*!< REG_PPG_CONFIG_2: SMP_AVE field Position           */
#define MAX86150_REG_PPG_CONFIG_2_SMP_AVE_Msk         (7 << MAX86150_REG_PPG_CONFIG_2_SMP_AVE_Pos)                /*!< REG_PPG_CONFIG_2: SMP_AVE field Mask               */
#define MAX86150_REG_PPG_CONFIG_2_SMP_AVE_1           (0)
#define MAX86150_REG_PPG_CONFIG_2_SMP_AVE_2           (1)
#define MAX86150_REG_PPG_CONFIG_2_SMP_AVE_4           (2)
#define MAX86150_REG_PPG_CONFIG_2_SMP_AVE_8           (3)
#define MAX86150_REG_PPG_CONFIG_2_SMP_AVE_16          (4)
#define MAX86150_REG_PPG_CONFIG_2_SMP_AVE_32_1        (5)
#define MAX86150_REG_PPG_CONFIG_2_SMP_AVE_32_2        (6)
#define MAX86150_REG_PPG_CONFIG_2_SMP_AVE_32_3        (7)

/* MAX86150 Register: Prox Interrupt Threshold (0x10) */
typedef union
{
  struct
  {
    uint8_t PROX_INT_THRESH:8;                  /*!< bit: 0..7                            */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_PROX_INT_THRESH_Type;

/* Prox Interrupt Threshold (0x10) Register Definitions */
#define MAX86150_REG_PROX_INT_THRESH_PROX_INT_THRESH_Pos  (0)                                                             /*!< REG_PROX_INT_THRESH: PROX_INT_THRESH field Position           */
#define MAX86150_REG_PROX_INT_THRESH_PROX_INT_THRESH_Msk  (0xFF << MAX86150_REG_PROX_INT_THRESH_PROX_INT_THRESH_Pos)  /*!< REG_PROX_INT_THRESH: PROX_INT_THRESH field Mask               */


/* MAX86150 Register: LED1 PA (0x11) */
typedef union
{
  struct
  {
    uint8_t LED1_PA:8;                          /*!< bit: 0..7                            */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_LED1_PA_Type;

/* LED1 PA (0x11) Register Definitions */
#define MAX86150_REG_LED1_PA_LED1_PA_Pos      (0)                                             /*!< REG_LED1_PA: LED1_PA field Position           */
#define MAX86150_REG_LED1_PA_LED1_PA_Msk      (0xFF << MAX86150_REG_LED1_PA_LED1_PA_Pos)  /*!< REG_LED1_PA: LED1_PA field Mask               */
#define MAX86150_REG_LED_PA_0_MA              (0) // 0 mA LED current
#define MAX86150_REG_LED_PA_02_MA             (1) // +0.2 mA LED current (max 51mA) for every accumulation of this value
#define MAX86150_REG_LED_PA_04_MA             (1) // +0.4 mA LED current (max 102mA)for every accumulation of this value 

/* MAX86150 Register: LED2 PA (0x12) */
typedef union
{
  struct
  {
    uint8_t LED2_PA:8;                          /*!< bit: 0..7                            */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_LED2_PA_Type;

/* LED2 PA (0x12) Register Definitions */
#define MAX86150_REG_LED2_PA_LED2_PA_Pos      (0)                                             /*!< REG_LED2_PA: LED2_PA field Position           */
#define MAX86150_REG_LED2_PA_LED2_PA_Msk      (0xFF << MAX86150_REG_LED2_PA_LED2_PA_Pos)  /*!< REG_LED2_PA: LED2_PA field Mask               */

/* MAX86150 Register: LED Range (0x14) */
typedef union
{
  struct
  {
    uint8_t LED1_RGE:2;                         /*!< bit: 0..1                            */
    uint8_t LED2_RGE:2;                         /*!< bit: 2..3                            */
    uint8_t _reserved0:4;                       /*!< bit: 4..7                            */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_LED_RANGE_Type;

/* LED Range (0x14) Register Definitions */
#define MAX86150_REG_LED_RANGE_LED2_RGE_Pos       (2)                                                /*!< REG_LED_RANGE: LED2_RGE field Position           */
#define MAX86150_REG_LED_RANGE_LED2_RGE_Msk       (0x3 << MAX86150_REG_LED_RANGE_LED2_RGE_Pos)   /*!< REG_LED_RANGE: LED2_RGE field Mask               */
#define MAX86150_REG_LED_RANGE_LED_RGE_50_MA      (0)
#define MAX86150_REG_LED_RANGE_LED_RGE_100_MA     (1)
#define MAX86150_REG_LED_RANGE_LED_RGE_NA1        (2)
#define MAX86150_REG_LED_RANGE_LED_RGE_NA2        (2)

#define MAX86150_REG_LED_RANGE_LED1_RGE_Pos       (0)                                                /*!< REG_LED_RANGE: LED1_RGE field Position           */
#define MAX86150_REG_LED_RANGE_LED1_RGE_Msk       (0x3 << MAX86150_REG_LED_RANGE_LED1_RGE_Pos)   /*!< REG_LED_RANGE: LED1_RGE field Mask               */

/* MAX86150 Register: LED PILOT PA (0x15) */
typedef union
{
  struct
  {
    uint8_t PILOT_PA:8;                         /*!< bit: 0..7                            */
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_LED_PILOT_PA_Type;

/* LED PILOT PA (0x15) Register Definitions */
#define MAX86150_REG_LED_PILOT_PA_PILOT_PA_Pos       (0)                                                    /*!< REG_LED_PILOT_PA: PILOT_PA field Position           */
#define MAX86150_REG_LED_PILOT_PA_PILOT_PA_Msk       (0xFF << MAX86150_REG_LED_PILOT_PA_PILOT_PA_Pos)   /*!< REG_LED_PILOT_PA: PILOT_PA field Mask               */

/* MAX86150 Register: ECG Configuration 1 (0x3C) */
typedef union
{
  struct
  {
    uint8_t ECG_ADC_OSR:2;                      /*!< bit: 0..1                            */
    uint8_t ECG_ADC_CLK:1;                      /*!< bit: 2                               */
    uint8_t _reserved0:5;                       /*!< bit: 3..7                            */    
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_ECG_CONFIG_1_Type;

/* ECG Configuration 1 (0x3C) Register Definitions */
#define MAX86150_REG_ECG_CONFIG_1_ECG_ADC_CLK_Pos       (2)                                                       /*!< REG_ECG_CONFIG_1: ECG_ADC_CLK field Position           */
#define MAX86150_REG_ECG_CONFIG_1_ECG_ADC_CLK_Msk       (0x1 << MAX86150_REG_ECG_CONFIG_1_ECG_ADC_CLK_Pos)    /*!< REG_ECG_CONFIG_1: ECG_ADC_CLK field Mask               */
#define MAX86150_REG_ECG_CONFIG_1_ECG_ADC_CLK_SPS_200   (0) // baseline ECG OSR of 200 SPS
#define MAX86150_REG_ECG_CONFIG_1_ECG_ADC_CLK_SPS_400   (1) // baseline ECG OSR of 400 SPS

#define MAX86150_REG_ECG_CONFIG_1_ECG_ADC_OSR_Pos       (0)                                                       /*!< REG_ECG_CONFIG_1: ECG_ADC_OSR field Position           */
#define MAX86150_REG_ECG_CONFIG_1_ECG_ADC_OSR_Msk       (0x3 << MAX86150_REG_ECG_CONFIG_1_ECG_ADC_OSR_Pos)    /*!< REG_ECG_CONFIG_1: ECG_ADC_OSR field Mask               */
#define MAX86150_REG_ECG_CONFIG_1_ECG_ADC_OSR_MUL_8     (0) // multiplier for baseline ECG OSR
#define MAX86150_REG_ECG_CONFIG_1_ECG_ADC_OSR_MUL_4     (1)
#define MAX86150_REG_ECG_CONFIG_1_ECG_ADC_OSR_MUL_2     (2)
#define MAX86150_REG_ECG_CONFIG_1_ECG_ADC_OSR_MUL_1     (3)

/* MAX86150 Register: ECG Configuration 2 (0x3E) */
typedef union
{
  struct
  {
    uint8_t IA_GAIN:2;                          /*!< bit: 0..1                            */
    uint8_t PGA_ECG_GAIN:2;                     /*!< bit: 2..3                            */
    uint8_t _reserved0:4;                       /*!< bit: 4..7                            */    
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_ECG_CONFIG_3_Type;

/* ECG Configuration 3 (0x3E) Register Definitions */
#define MAX86150_REG_ECG_CONFIG_3_PGA_ECG_GAIN_Pos        (2)                                                        /*!< REG_ECG_CONFIG_3: PGA_ECG_GAIN field Position           */
#define MAX86150_REG_ECG_CONFIG_3_PGA_ECG_GAIN_Msk        (0x3 << MAX86150_REG_ECG_CONFIG_3_PGA_ECG_GAIN_Pos)    /*!< REG_ECG_CONFIG_3: PGA_ECG_GAIN field Mask               */
#define MAX86150_REG_ECG_CONFIG_3_PGA_ECG_GAIN_1          (0)
#define MAX86150_REG_ECG_CONFIG_3_PGA_ECG_GAIN_2          (1)
#define MAX86150_REG_ECG_CONFIG_3_PGA_ECG_GAIN_4          (2)
#define MAX86150_REG_ECG_CONFIG_3_PGA_ECG_GAIN_8          (3)

#define MAX86150_REG_ECG_CONFIG_3_IA_GAIN_Pos             (0)                                                         /*!< REG_ECG_CONFIG_3: IA_GAIN field Position               */
#define MAX86150_REG_ECG_CONFIG_3_IA_GAIN_Msk             (0x3 << MAX86150_REG_ECG_CONFIG_3_IA_GAIN_Pos)          /*!< REG_ECG_CONFIG_3: IA_GAIN field Mask                   */
#define MAX86150_REG_ECG_CONFIG_3_IA_GAIN_5               (0)
#define MAX86150_REG_ECG_CONFIG_3_IA_GAIN_9_5             (1)
#define MAX86150_REG_ECG_CONFIG_3_IA_GAIN_20              (2)
#define MAX86150_REG_ECG_CONFIG_3_IA_GAIN_50              (3)

#define APSR_N_Pos                         31U                                            /*!< APSR: N Position */
#define APSR_N_Msk                         (1UL << APSR_N_Pos)                            /*!< APSR: N Mask     */

/* MAX86150 Register: Part ID (0xFF)) */
typedef union
{
  struct
  {
    uint8_t PART_ID:8;                          /*!< bit: 0..7                            */  
  } reg_bits;                                   /*!< Structure used for bit level access  */
  uint8_t reg_byte;                             /*!< Type used for byte level access      */
} MAX86150_REG_PART_ID_Type;

/* Part ID (0xFF) Register Definitions */
#define MAX86150_REG_PART_ID_PART_ID_Pos        (0)                                                         /*!< REG_PART_ID: PART_ID field Position           */
#define MAX86150_REG_PART_ID_PART_ID_Msk        (0xFF << MAX86150_REG_PART_ID_PART_ID_Pos)              /*!< REG_PART_ID: PART_ID field Mask               */

/* FIFO Data Masks */
#define Bits_18_Msk   (uint32_t)(0x3FFFF)
#define Bits_19_Msk   (uint32_t)(0x7FFFF)
#define FIFO_PPG_Msk  Bits_19_Msk
#define FIFO_ECG_Msk  Bits_18_Msk


#ifdef __cplusplus
}
#endif

#endif  /* SENSOR_MAX86150_REGISTER_MAP_H_ */
