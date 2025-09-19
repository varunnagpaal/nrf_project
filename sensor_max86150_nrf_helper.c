#include "sensor_max86150_nrf_helper.h"
#include "max86150.h"
#include <stdbool.h>
#include "nrf_drv_twi.h"
#include "app_error.h"
#include "nrf_assert.h"
#include "helpers.h"
#include "common.h"
#define NRF_LOG_MODULE_NAME "MOD"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"

extern nrf_drv_twi_t i2c_controller_instance;
extern volatile bool bI2CXferDone;

/* Set to true iff local copies of registers of MAX86150 sensor was reset */
bool bMax86150RegReset = false;

/* Buffer to accumulate 17 samples read from FIFO at a time, 
    - Each sample has 3 data elements: FD1 (LED1 PPG), FD2 (LED2 PPG) and FD3(ECG) 
    - Each data element is 3-bytes (24-bit) long
    We need total 17 x 3 x 3 = 153 byte sample buffer. 
    But max FIFO size is 32 samples. So we need max 32 x 3 x 3 = 288 byte buffer
*/
static uint8_t m_fifo_bytes[288];

/* Generic buffer */
static uint8_t m_i2c_tx_buffer[8];

/* Generic transfer descriptor */
nrf_drv_twi_xfer_desc_t m_i2c_xfer_desc;

/* Buffer for reading 21 out of 22 MAX86150 registers */
static uint8_t m_sensor_max86150_regs[21];
//static SENSOR_MAX86150_TYPE m_sensor_max86150_registers;

/* Storage for MAX86150 registers */
MAX86150_REG_INT_STATUS_1_Type          max86150_reg_int_status_1;
MAX86150_REG_INT_STATUS_2_Type          max86150_reg_int_status_2;
MAX86150_REG_INT_ENABLE_1_Type          max86150_reg_int_enable_1;
MAX86150_REG_INT_ENABLE_2_Type          max86150_reg_int_enable_2;
MAX86150_REG_FIFO_WR_PTR_Type           max86150_reg_fifo_wr_ptr;
MAX86150_REG_OVF_COUNTER_Type           max86150_reg_ovf_counter;
MAX86150_REG_FIFO_RD_PTR_Type           max86150_reg_fifo_rd_ptr;
MAX86150_REG_FIFO_RD_PTR_Type           max86150_reg_fifo_rd_ptr;
MAX86150_REG_FIFO_DATA_Type             max86150_reg_fifo_data;
MAX86150_REG_FIFO_CONFIG_Type           max86150_reg_fifo_config;
MAX86150_REG_FIFO_DATA_CONTROL_1_Type   max86150_reg_fifo_data_control_1;
MAX86150_REG_FIFO_DATA_CONTROL_2_Type   max86150_reg_fifo_data_control_2;
MAX86150_REG_SYS_CONTROL_Type           max86150_reg_sys_control;
MAX86150_REG_PPG_CONFIG_1_Type          max86150_reg_ppg_config_1;
MAX86150_REG_PPG_CONFIG_2_Type          max86150_reg_ppg_config_2;
MAX86150_REG_PROX_INT_THRESH_Type       max86150_reg_prox_int_thresh;
MAX86150_REG_LED1_PA_Type               max86150_reg_led1_pa;
MAX86150_REG_LED2_PA_Type               max86150_reg_led2_pa;
MAX86150_REG_LED_RANGE_Type             max86150_reg_led_range;
MAX86150_REG_LED_PILOT_PA_Type          max86150_reg_led_pilot_pa;
MAX86150_REG_ECG_CONFIG_1_Type          max86150_reg_ecg_config_1;
MAX86150_REG_ECG_CONFIG_3_Type          max86150_reg_ecg_config_3;
MAX86150_REG_PART_ID_Type               max86150_reg_part_id;

/* Static Function Declarations */
static void m_sensor_max86150_reg_copy_reset(void);
static void sensor_max86150_check_reg_address(void);
static void nrf_i2c_write(  uint8_t responderId, 
                                uint8_t *pRegAddress,
                                uint8_t data_byte,
                                bool bRepeatStartRaised,
                                bool bStopRaised );
static void nrf_i2c_read( uint8_t responderId, 
                              uint8_t regAddress,
                              uint8_t * pRegData,
                              uint8_t byteCount,
                              bool bRepeatStartRaised,
                              bool bStopRaised );
static void sensor_max86150_fifo_get_sample(void);


/* Reset all local copies of MAX86150 registers */
static void m_sensor_max86150_reg_copy_reset(void)
{
  if( false == bMax86150RegReset)
  {
    NRF_LOG_INFO( "[%d] Resetting all local copies of MAX86150 registers.\r\n", order++ );
    max86150_reg_int_status_1.reg_byte = 0x00;          // READ ONLY
    max86150_reg_int_status_2.reg_byte = 0x00;          // READ ONLY
    max86150_reg_int_enable_1.reg_byte = 0x00;
    max86150_reg_int_enable_2.reg_byte = 0x00;
    max86150_reg_fifo_wr_ptr.reg_byte = 0x00;
    max86150_reg_ovf_counter.reg_byte = 0x00;           // READ ONLY
    max86150_reg_fifo_rd_ptr.reg_byte = 0x00;
    max86150_reg_fifo_data.reg_byte = 0x00;             // READ ONLY
    max86150_reg_fifo_config.reg_byte = 0x00;
    max86150_reg_fifo_data_control_1.reg_byte = 0x00;
    max86150_reg_fifo_data_control_2.reg_byte = 0x00;
    max86150_reg_sys_control.reg_byte = 0x00;
    max86150_reg_ppg_config_1.reg_byte = 0x00;
    max86150_reg_ppg_config_2.reg_byte = 0x00;
    max86150_reg_prox_int_thresh.reg_byte = 0x00;
    max86150_reg_led1_pa.reg_byte = 0x00;
    max86150_reg_led2_pa.reg_byte = 0x00;
    max86150_reg_led_range.reg_byte = 0x00;
    max86150_reg_led_pilot_pa.reg_byte = 0x00;
    max86150_reg_ecg_config_1.reg_byte = 0x00;
    max86150_reg_ecg_config_3.reg_byte = 0x00;
    max86150_reg_part_id.reg_byte = 0x00;               // READ ONLY

    bMax86150RegReset = true;
  }
}

static void sensor_max86150_check_reg_address(void)
{
  NRF_LOG_INFO( "[%d] Checking if all local copies of MAX86150 registers have correct address.\r\n", order++ );
  NRF_LOG_INFO( "[%d] (uint8_t)0x00 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_INT_STATUS_1): %d\r\n",
                order++,
                ((uint8_t)0x00 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_INT_STATUS_1)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x01 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_INT_STATUS_2): %d\r\n",
                order++,
                ((uint8_t)0x01 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_INT_STATUS_2)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x02 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_INT_ENABLE_1): %d\r\n",
                order++,
                ((uint8_t)0x02 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_INT_ENABLE_1)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x03 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_INT_ENABLE_2): %d\r\n",
                order++,
                ((uint8_t)0x03 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_INT_ENABLE_2)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x04 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_FIFO_WR_PTR): %d\r\n",
                order++,
                ((uint8_t)0x04 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_FIFO_WR_PTR)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x05 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_OVF_COUNTER): %d\r\n",
                order++,
                ((uint8_t)0x05 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_OVF_COUNTER)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x06 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_FIFO_RD_PTR): %d\r\n",
                order++,
                ((uint8_t)0x06 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_FIFO_RD_PTR)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x07 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_FIFO_DATA): %d\r\n",
                order++,
                ((uint8_t)0x07 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_FIFO_DATA)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x08 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_FIFO_CONFIG): %d\r\n",
                order++,
                ((uint8_t)0x08 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_FIFO_CONFIG)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x09 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_FIFO_DATA_CONTROL_1): %d\r\n",
                order++,
                ((uint8_t)0x09 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_FIFO_DATA_CONTROL_1)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x0A == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_FIFO_DATA_CONTROL_2): %d\r\n",
                order++,
                ((uint8_t)0x0A == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_FIFO_DATA_CONTROL_2)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x0D == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_SYS_CONTROL): %d\r\n",
                order++,
                ((uint8_t)0x0D == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_SYS_CONTROL)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x0E == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_PPG_CONFIG_1): %d\r\n",
                order++,
                ((uint8_t)0x0E == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_PPG_CONFIG_1)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x0F == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_PPG_CONFIG_2): %d\r\n",
                order++,
                ((uint8_t)0x0F == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_PPG_CONFIG_2)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x010 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_PROX_INT_THRESH): %d\r\n",
                order++,
                ((uint8_t)0x10 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_PROX_INT_THRESH)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x011 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_LED1_PA): %d\r\n",
                order++,
                ((uint8_t)0x11 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_LED1_PA)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x012 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_LED2_PA): %d\r\n",
                order++,
                ((uint8_t)0x12 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_LED2_PA)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x014 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_LED_RANGE): %d\r\n",
                order++,
                ((uint8_t)0x14 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_LED_RANGE)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x015 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_LED_PILOT_PA): %d\r\n",
                order++,
                ((uint8_t)0x15 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_LED_PILOT_PA)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x03C == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_ECG_CONFIG_1): %d\r\n",
                order++,
                ((uint8_t)0x3C == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_ECG_CONFIG_1)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] SENSOR_MAX86150_REG_ADDR(REG_ECG_CONFIG_1): %d\r\n",
                order++,
                (uint32_t)(SENSOR_MAX86150_REG_ADDR(REG_ECG_CONFIG_1)));
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0x3E == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_ECG_CONFIG_3): %d\r\n",
                order++,
                ((uint8_t)0x3E == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_ECG_CONFIG_3)) );
  NRF_LOG_FLUSH();
  NRF_LOG_INFO( "[%d] (uint8_t)0xFF == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_PART_ID): %d\r\n",
                order++,
                ((uint8_t)0xFF == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_PART_ID)) );

  ASSERT((uint8_t)0x00 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_INT_STATUS_1));
  ASSERT((uint8_t)0x01 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_INT_STATUS_2));
  ASSERT((uint8_t)0x02 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_INT_ENABLE_1));
  ASSERT((uint8_t)0x03 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_INT_ENABLE_2));
  ASSERT((uint8_t)0x04 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_FIFO_WR_PTR));
  ASSERT((uint8_t)0x05 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_OVF_COUNTER));
  ASSERT((uint8_t)0x06 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_FIFO_RD_PTR));
  ASSERT((uint8_t)0x07 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_FIFO_DATA));
  ASSERT((uint8_t)0x08 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_FIFO_CONFIG));
  ASSERT((uint8_t)0x09 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_FIFO_DATA_CONTROL_1));
  ASSERT((uint8_t)0x0A == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_FIFO_DATA_CONTROL_2));
  ASSERT((uint8_t)0x0D == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_SYS_CONTROL));
  ASSERT((uint8_t)0x0E == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_PPG_CONFIG_1));
  ASSERT((uint8_t)0x0F == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_PPG_CONFIG_2));
  ASSERT((uint8_t)0x10 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_PROX_INT_THRESH));
  ASSERT((uint8_t)0x11 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_LED1_PA));
  ASSERT((uint8_t)0x12 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_LED2_PA));
  ASSERT((uint8_t)0x14 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_LED_RANGE));
  ASSERT((uint8_t)0x15 == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_LED_PILOT_PA));
  ASSERT((uint8_t)0x3C == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_ECG_CONFIG_1));
  ASSERT((uint8_t)0x3E == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_ECG_CONFIG_3));
  ASSERT((uint8_t)0xFF == (uint8_t)SENSOR_MAX86150_REG_ADDR(REG_PART_ID));
}

/* For the write operation, 
  1. Send the responder (MAX86150) ID as the first byte with R/!W=0 (WRITE) followed by the MAX86150 register address
     byte that one wishes to write and data byte to be written to that register
  2. Optional: The MAX86150 register address pointer increments automatically if any subsequent data bytes are 
     received treating the next byte received as content of register at next address. Using this,  
     the entire register bank can be written by at one time.
  3. Terminate the data transfer with a STOP condition.
*/
static void nrf_i2c_write(  uint8_t responderId, 
                                uint8_t *pRegAddress,
                                uint8_t data_byte,
                                bool bRepeatStartRaised,
                                bool bStopRaised )
{

  if(bStopRaised){
    /* Raise STOP condition */
  } else if(bRepeatStartRaised)
  {
    /* WITHOUT REPEAT START condition: send one or more bytes to the responder (MAX86150) id with R/!W =0 (WRITE). */
  } else
  {
    /* START: Send the responder (MAX86150) ID as the first byte with R/!W=0 (WRITE) 
       followed by the MAX86150 register address byte that one wishes to write
    */
  }
}

/* For the read operation, 
  1. Send the responder (MAX86150) ID as the first byte with R/!W=1 (READ) followed by the register address byte
     for the MAX86150 register that one wishes to read
  2. Then a REPEAT START (Sr) condition is sent followed by the responder ID with R/!W =1 (READ)
  3. MAX86150 then begins sending data beginning with the register selected in the first operation.
  3. Optionally: If controller doesn't invoke STOP condition after ACK received data byte and
     decides to continue to read subsequent bytes it receives from MAX86150,  the MAX86150 continues sending 
     data from additional registers in sequential order (read pointer is incremented automatically) starting from
     the register address sent in initial I2C operation.
     NOTE: The exception to this is the FIFO_DATA register, at which the read pointer no longer increments 
     when reading additional bytes. To read the next register after FIFO_DATA, an I2C write command is
     necessary to change the location of the read pointer. If the FIFO_DATA register is read, the read pointer does
     not automatically increment, and subsequent bytes of data contain the contents of the FIFO.
  5. Terminate the data transfer with a STOP condition.
*/
static void nrf_i2c_read( uint8_t responderId, 
                              uint8_t regAddress,
                              uint8_t * pRegData,
                              uint8_t byteCount,
                              bool bRepeatStartRaised,
                              bool bStopRaised )
{
  ret_code_t err_code = NRF_SUCCESS;
  if(bStopRaised){
    /* Raise STOP condition */
  } else if(bRepeatStartRaised)
  {
    /* REPEAT START condition: send responder (MAX86150) id with R/!W =1 (READ). Then read byte send by responder(MAX86150) */
  } else
  {
    /* START: Send the responder (MAX86150) ID as the first byte with R/!W=0 (WRITE) 
       followed by the MAX86150 register address byte that one wishes to read
    */
    err_code = nrf_drv_twi_tx(&i2c_controller_instance,
                              regAddress,
                              pRegData,
                              byteCount,
                              false);
  }
  UNUSED_VARIABLE(err_code);
}

/* READ FIFO DATA
  FIFO READ & WRITE POINTERS 
  - point to next SAMPLE location to be read from and written to
  - advance to next SAMPLE location each time POP (READ a sample from FIFO) and PUSH (WRITE a sample in FIFO)

  FIFO Organization
  - upto 32 SAMPLES/FIFO x (1-4 valid elements of type FDx)/Sample x 3-byte(24-bit)/element
  
  Since we configured FD1=LED1 (IR PPG), FD2= LED2 (Visible PPG), FD3 = ECG, FD4 = None    
  Number of valid elements of type FDx = 3 (since FD4 = None, we dont count it)    
  Thus reading a sample for FIFO requires I2C burst reading following number of bytes starting with MSByte
  - Number of valid elements of type FDx  x 3-byte/element  = 3x3 = 9 bytes per sample
  
  Only after fetching 9-bytes(POP) from FIFO Data register, would the FIFO READ POINTER advance
    Bytes[0:2] = FD1[23:16, 15:8, 7:0] = LED1 (IR PPG) = Only LS 19-bits valid, so mask remaining 5 MSBits
    Bytes[3:5] = FD2[23:16, 15:8, 7:0] = LED2 (Visible PPG) = Only LS 19-bits valid, so mask remaining 5 MSBits
    Bytes[6:8] = FD3[23:16, 15:8, 7:0] = ECG = No need to mask as the 6 MSbits are set to 0 by chip and rest LS 18-bits hold ECG value. So just read the entire 24-bit as ECG value
*/
static void sensor_max86150_fifo_get_sample(void)
{
  /* TBU */
}

/*  The part under-goes a forced power-on-reset sequence. 
    All configuration, threshold and data registers including distributed 
    registers are reset to their power-on-state. This bit then automatically 
    becomes ‘0’ after the reset sequence is completed.
*/
void sensor_max86150_power_on_reset(void)
{
  ret_code_t err_code = NRF_SUCCESS;
  max86150_reg_sys_control.reg_bits.RESET = MAX86150_REG_SYS_CONTROL_RESET_Enabled;

  // START
  // TX &REG_SYS_CONTROL
  // TX REG_SYS_CONTROL
  // STOP
  NRF_LOG_INFO( "[%d] Forcing MAX86150 sensor into reset.\r\n", order++ );
  m_i2c_tx_buffer[0] = (uint8_t)(SENSOR_MAX86150_REG_ADDR(REG_SYS_CONTROL));
  m_i2c_tx_buffer[1] = max86150_reg_sys_control.reg_byte;
  err_code = nrf_drv_twi_tx(&i2c_controller_instance,
                            SENSOR_MAX86150_I2C_RESPONDER_ADDR,
                            m_i2c_tx_buffer,
                            2,
                            false);
  APP_ERROR_CHECK(err_code);
}

/* The sensor can be put into a power-save mode by writing a ‘1’ to this bit.
    While in this mode all registers remain accessible and retain their data.
    ADC conversion data contained in the registers are previous values. 
    Writable registers also remain accessible in shutdown. All interrupts are
    cleared. In
this mode the oscillator is shutdown and the part draws minimum
    current. If this bit is asserted during a active conversion then the 
    conversion completes before the part shuts down.
*/
void sensor_max86150_shutdown(void)
{
  ret_code_t err_code = NRF_SUCCESS;
  max86150_reg_sys_control.reg_bits.SHDN = MAX86150_REG_SYS_CONTROL_SHDN_Enabled;

  // START
  // TX &REG_SYS_CONTROL
  // TX REG_SYS_CONTROL
  // STOP
  NRF_LOG_INFO( "[%d] Forcing MAX86150 sensor into shutdown mode.\r\n", order++ );
  m_i2c_tx_buffer[0] = (uint8_t)(SENSOR_MAX86150_REG_ADDR(REG_SYS_CONTROL));
  m_i2c_tx_buffer[1] = max86150_reg_sys_control.reg_byte;
  err_code = nrf_drv_twi_tx(&i2c_controller_instance,
                            SENSOR_MAX86150_I2C_RESPONDER_ADDR,
                            m_i2c_tx_buffer,
                            2,
                            false);
  APP_ERROR_CHECK(err_code);
}

void sensor_max86150_wakeup(void)
{
  ret_code_t err_code = NRF_SUCCESS;
  max86150_reg_sys_control.reg_bits.SHDN = MAX86150_REG_SYS_CONTROL_SHDN_Disabled;

  // START
  // TX &REG_SYS_CONTROL
  // TX REG_SYS_CONTROL
  // STOP
  NRF_LOG_INFO( "[%d] Waking up MAX86150 sensor from shutdown.\r\n", order++ );
  m_i2c_tx_buffer[0] = (uint8_t)(SENSOR_MAX86150_REG_ADDR(REG_SYS_CONTROL));
  m_i2c_tx_buffer[1] = max86150_reg_sys_control.reg_byte;
  err_code = nrf_drv_twi_tx(&i2c_controller_instance,
                            SENSOR_MAX86150_I2C_RESPONDER_ADDR,
                            m_i2c_tx_buffer,
                            2,
                            false);
  APP_ERROR_CHECK(err_code);
}

/*  FIFO gets flushed iff
    - FIFO_EN = 1 and I2C write to any of the PPG/ECG Configuration register, FIFO Data Control registers, rising edge of FIFO_EN, Enter-n-Exit PROX mode
    When the FIFO gets flushed, FIFO_WR_PTR and FIFO_RD_PTR are reset to zero, and the contents of the FIFO are lost.
    To prevent loss of samples inside FIFO, set FIFO_EN=1 before writing to PPG/ECG Configuration and data control registers
*/
void sensor_max86150_fifo_flush(void)
{
  /* TBU */
}

void sensor_max86150_registers_read_all()
{
  /* Read all registers except FIFO Data register. Requires restart condition */
  ret_code_t err_code = NRF_SUCCESS;

  // READ 7 REGISTERS: REG_INT_STATUS_1 to REG_FIFO_RD_PTR
  m_i2c_tx_buffer[0] = (uint8_t)(SENSOR_MAX86150_REG_ADDR(REG_INT_STATUS_1));
  m_i2c_xfer_desc.type             = NRF_DRV_TWI_XFER_TXRX;
  m_i2c_xfer_desc.address          = SENSOR_MAX86150_I2C_RESPONDER_ADDR;
  m_i2c_xfer_desc.p_primary_buf    = &(m_i2c_tx_buffer[0]);
  m_i2c_xfer_desc.primary_length   = 1;
  m_i2c_xfer_desc.p_secondary_buf  = m_sensor_max86150_regs;
  m_i2c_xfer_desc.secondary_length = 7;

  bI2CXferDone = false;
  err_code = nrf_drv_twi_xfer(&i2c_controller_instance, &m_i2c_xfer_desc, 0 ); // Non-blocking since a handler is associated with this I2C instance. With TWI, the only supported flag is NRF_DRV_TWI_FLAG_TX_NO_STOP. All other flags require TWIM. 
  APP_ERROR_CHECK(err_code);

  /* Nap sleep until Xfer is not done */
  system_sleep_nap_mode(&bI2CXferDone);

  // SKIP REGISTER: REG_FIFO_DATA

  // READ 14 REGISTERS: REG_FIFO_CONFIG to REG_PART_ID
  m_i2c_tx_buffer[0] = (uint8_t)(SENSOR_MAX86150_REG_ADDR(REG_FIFO_CONFIG));
  m_i2c_xfer_desc.type             = NRF_DRV_TWI_XFER_TXRX;
  m_i2c_xfer_desc.address          = SENSOR_MAX86150_I2C_RESPONDER_ADDR;
  m_i2c_xfer_desc.p_primary_buf    = &(m_i2c_tx_buffer[0]);
  m_i2c_xfer_desc.primary_length   = 1;
  m_i2c_xfer_desc.p_secondary_buf  = &(m_sensor_max86150_regs[7]);
  m_i2c_xfer_desc.secondary_length = 14;
  
  bI2CXferDone = false;  
  err_code = nrf_drv_twi_xfer(&i2c_controller_instance, &m_i2c_xfer_desc, 0 );  // Non-blocking since a handler is associated with this I2C instance. With TWI, the only supported flag is NRF_DRV_TWI_FLAG_TX_NO_STOP. All other flags require TWIM. 
  APP_ERROR_CHECK(err_code);
  
  /* Nap sleep until Xfer is not done */
  system_sleep_nap_mode(&bI2CXferDone);  
}

/* "HOW TO CONFIGURE THE OPTIMAL SETTINGS FOR THE BEST PPG AND ECG PERFORMANCE IN THE MAX86150"
    Reference: MAXIM APPLICATION NOTE 6843

    READ ONLY REGISTERS (5 count)
    max86150_reg_int_status_1.reg_byte = 0x00;
    max86150_reg_int_status_2.reg_byte = 0x00;
    max86150_reg_ovf_counter.reg_byte = 0x00;
    max86150_reg_fifo_data.reg_byte = 0x00;
    max86150_reg_part_id.reg_byte = 0x00;

    CONFIGURABLE REGISTER (17 count)
    max86150_reg_int_enable_1.reg_byte = 0x00;			    // OK
    max86150_reg_int_enable_2.reg_byte = 0x00;			    // OK
    max86150_reg_fifo_wr_ptr.reg_byte = 0x00;			      // TBU for FIFO
    max86150_reg_fifo_rd_ptr.reg_byte = 0x00;			      // TBU for FIFO
    max86150_reg_fifo_config.reg_byte = 0x00;			      // OK
    max86150_reg_fifo_data_control_1.reg_byte = 0x00;		// OK
    max86150_reg_fifo_data_control_2.reg_byte = 0x00;		// OK
    max86150_reg_sys_control.reg_byte = 0x00;			      // OK
    max86150_reg_ppg_config_1.reg_byte = 0x00;		    	// OK
    max86150_reg_ppg_config_2.reg_byte = 0x00;			    // OK
    max86150_reg_prox_int_thresh.reg_byte = 0x00;			  // Check this
    max86150_reg_led1_pa.reg_byte = 0x00;				        // OK
    max86150_reg_led2_pa.reg_byte = 0x00;				        // OK
    max86150_reg_led_range.reg_byte = 0x00;				      // OK
    max86150_reg_led_pilot_pa.reg_byte = 0x00;			    // OK
    max86150_reg_ecg_config_1.reg_byte = 0x00;			    // OK
    max86150_reg_ecg_config_3.reg_byte = 0x00;			    // OK
*/

void sensor_max86150_registers_configure(void)
{
  ret_code_t err_code = NRF_SUCCESS;
  sensor_max86150_check_reg_address();
  m_sensor_max86150_reg_copy_reset();

  NRF_LOG_INFO( "[%d] Assigning optimal configuration settings to local register copies of MAX86150 sensor\r\n", order++ );
  
  /* 1. Configure the FIFO settings: Enable the FIFO and allow the FIFO to be read.
      {0x02, 0x80}; // 0x80 for A_FULL_EN
      {0x0D, 0x01}, // Reset part
      {0x0D, 0x04}, // Enable FIFO
      {0x08, 0x1F}, // 0x1F for FIFO_ROLLS_ON_FULL to 1 and lost old samples when FIFO is full, Read FIFO data when there are 17 samples
  */
  max86150_reg_int_enable_1.reg_bits.PROX_INT_EN = MAX86150_REG_INT_ENABLE_1_PROX_INT_EN_Enable;
  max86150_reg_int_enable_1.reg_bits.ALC_OVF_EN = MAX86150_REG_INT_ENABLE_1_ALC_OVF_EN_Enable;
  max86150_reg_int_enable_1.reg_bits.PPG_DATA_RDY_EN = MAX86150_REG_INT_ENABLE_1_PPG_DATA_RDY_EN_Enable;
  max86150_reg_int_enable_1.reg_bits.A_FULL_EN = MAX86150_REG_INT_ENABLE_1_A_FULL_EN_Enable;
  
  max86150_reg_int_enable_2.reg_bits.ECG_DATA_RDY_EN = MAX86150_REG_INT_ENABLE_2_ECG_DATA_RDY_EN_Enable;
  max86150_reg_int_enable_2.reg_bits.VDD_OOR_EN = MAX86150_REG_INT_ENABLE_2_VDD_OOR_EN_Enable;
  
  max86150_reg_fifo_config.reg_bits.FIFO_A_FULL = MAX86150_REG_FIFO_CONFIG_FIFO_A_FULL_FreeSpace_15;
  max86150_reg_fifo_config.reg_bits.FIFO_ROLLS_ON_FULL = MAX86150_REG_FIFO_CONFIG_FIFO_ROLLS_ON_FULL_ON;
  max86150_reg_fifo_config.reg_bits.A_FULL_TYPE = MAX86150_REG_FIFO_CONFIG_A_FULL_TYPE_RPT;
  max86150_reg_fifo_config.reg_bits.A_FULL_CLR = MAX86150_REG_FIFO_CONFIG_A_FULL_CLR_RD_DATA_NOCLR;

  max86150_reg_sys_control.reg_bits.RESET = MAX86150_REG_SYS_CONTROL_RESET_Disabled;
  max86150_reg_sys_control.reg_bits.SHDN = MAX86150_REG_SYS_CONTROL_SHDN_Disabled;
  max86150_reg_sys_control.reg_bits.FIFO_EN = MAX86150_REG_SYS_CONTROL_FIFO_EN_Enabled;

  /* 2. Enable the PPG and ECG modes: Enable PPG and ECG functions. */
  /* {0x09, 0x21}, // LED1 in slot 1 and LED2 in slot 2 */
  max86150_reg_fifo_data_control_1.reg_bits.FD1 = MAX86150_REG_FIFO_DATA_CONTROL_FD_PPG_LED1;
  max86150_reg_fifo_data_control_1.reg_bits.FD2 = MAX86150_REG_FIFO_DATA_CONTROL_FD_PPG_LED2;

  /* {0x0A, 0x09}, // ECG in slot 3 */
  max86150_reg_fifo_data_control_2.reg_bits.FD3 = MAX86150_REG_FIFO_DATA_CONTROL_FD_Pilot_ECG;
  max86150_reg_fifo_data_control_2.reg_bits.FD4 = MAX86150_REG_FIFO_DATA_CONTROL_FD_None;

  /* 3. Configure the acquisition settings for the best PPG performance: Depending on the human physiology,
        LED power, and LED pulse width, the ADC count detection range can be adjusted to ensure good
        signal to noise while avoiding overexposure on the human subjects or unnecessary power
        consumption. The sampling rate can also be adjusted depending on the use case.     
  */

  /* LED Range: 51 mA */
  max86150_reg_led_range.reg_bits.LED1_RGE = MAX86150_REG_LED_RANGE_LED_RGE_50_MA;
  max86150_reg_led_range.reg_bits.LED2_RGE = MAX86150_REG_LED_RANGE_LED_RGE_50_MA;

  /* {0x11, 0x55}, // LED1 current setting, optimal setting can vary depending on human physiology */
  max86150_reg_led1_pa.reg_bits.LED1_PA = 0x55;

  /* {0x12, 0x55}, // LED2 current setting, optimal setting can vary depending on human physiology */
  max86150_reg_led2_pa.reg_bits.LED2_PA = 0x55;

  /* {0x0E, 0xD3}, // 0xD3(incorrect: should be 0x93) for PPG_ADC_RGE= 32μA, PPG_SR = 100Hz, PPG_LED_PW = 400μs, actual sample rate can vary depending on the use case */
  max86150_reg_ppg_config_1.reg_bits.PPG_LED_PW = MAX86150_REG_PPG_CONFIG_1_PPG_LED_PW_US_400;
  max86150_reg_ppg_config_1.reg_bits.PPG_SR = MAX86150_REG_PPG_CONFIG_1_PPG_SR_PPS_1_SPS_100;
  max86150_reg_ppg_config_1.reg_bits.PPG_ADC_RGE = MAX86150_REG_PPG_CONFIG_1_PPG_ADC_RGE_16384;

  /* {0x0F, 0x18}, // 0x18 for 20μs delay from the rising edge of the LED to the start of integration */
  max86150_reg_ppg_config_2.reg_bits.SMP_AVE = MAX86150_REG_PPG_CONFIG_2_SMP_AVE_2;

  /* 4. Configure the acquisition settings for the best ECG performance: Adjust the sampling frequency and
        amplifier gain settings depending on the use case.
  */
  /* {0x3C, 0x03}, //0x03 for ECG_ADC_OSR = 200Hz, actual sample rate can vary depending on the use case */
  max86150_reg_ecg_config_1.reg_bits.ECG_ADC_OSR = MAX86150_REG_ECG_CONFIG_1_ECG_ADC_OSR_MUL_1;
  max86150_reg_ecg_config_1.reg_bits.ECG_ADC_CLK = MAX86150_REG_ECG_CONFIG_1_ECG_ADC_CLK_SPS_200;

  /* {0x3E, 0x0D}, // 0x0D for PGA_ECG_Gain = 8, and IA_Gain = 9.5, total gain = 76 V/V being the most accurate gain setting */
  max86150_reg_ecg_config_3.reg_bits.IA_GAIN = MAX86150_REG_ECG_CONFIG_3_IA_GAIN_9_5;
  max86150_reg_ecg_config_3.reg_bits.PGA_ECG_GAIN = MAX86150_REG_ECG_CONFIG_3_PGA_ECG_GAIN_8;

  /* 5. Configure the AFE settings for the best ECG performance: These settings should be implemented prior
        to all ECG measurements.
      
      {0xFF, 0x54}, // series of code to set appropriate AFE settings, to be executed in sequential order
      {0xFF, 0x4D},
      {0xCE, 0x0A},
      {0xCF, 0x18},
      {0xFF, 0x00}, // Complete 
  */

  /* 6. Proximity Settings
     Threshold >= ADC Count of 1023 (PROX_INT_THRESH 0x01)
     Pilot PA = (.2 or.4) * 50 mA drive current for LED1 (IR)
   */
  max86150_reg_prox_int_thresh.reg_bits.PROX_INT_THRESH = 0x01;  // To calculate ADC count, set this value to 8-MSBits of the 19-bit Delta-Sigma ADC i.e. S00_0000_0100_0000_0000=1024 ADC count. (S=sign bit). TBU
  max86150_reg_led_pilot_pa.reg_bits.PILOT_PA = 0x55;

  /* Configure MAX86150 registers via multiple I2C TX */
  NRF_LOG_INFO( "[%d] Configuring registers of MAX86150 sensor with optimal settings using local register copies.\r\n", order++ );

  // START
  // TX &REG_INT_ENABLE_1
  // TX REG_INT_ENABLE_1
  // TX REG_INT_ENABLE_2
  // STOP
  m_i2c_tx_buffer[0] = (uint8_t)(SENSOR_MAX86150_REG_ADDR(REG_INT_ENABLE_1));
  m_i2c_tx_buffer[1] = max86150_reg_int_enable_1.reg_byte;
  m_i2c_tx_buffer[2] = max86150_reg_int_enable_2.reg_byte;

  bI2CXferDone = false;
  err_code = nrf_drv_twi_tx(&i2c_controller_instance,
                            SENSOR_MAX86150_I2C_RESPONDER_ADDR,
                            m_i2c_tx_buffer,
                            3,
                            false);
  APP_ERROR_CHECK(err_code);

  /* Nap sleep until Xfer is not done */
  system_sleep_nap_mode(&bI2CXferDone);  
  
  // START
  // TX &REG_FIFO_CONFIG
  // TX REG_FIFO_CONFIG
  // TX REG_FIFO_DATA_CONTROL_1
  // TX REG_FIFO_DATA_CONTROL_2
  // STOP
  m_i2c_tx_buffer[0] = (uint8_t)(SENSOR_MAX86150_REG_ADDR(REG_FIFO_CONFIG));
  m_i2c_tx_buffer[1] = max86150_reg_fifo_config.reg_byte;
  m_i2c_tx_buffer[2] = max86150_reg_fifo_data_control_1.reg_byte;
  m_i2c_tx_buffer[3] = max86150_reg_fifo_data_control_2.reg_byte;

  bI2CXferDone = false;  
  err_code = nrf_drv_twi_tx(&i2c_controller_instance,
                            SENSOR_MAX86150_I2C_RESPONDER_ADDR,
                            m_i2c_tx_buffer,
                            4,
                            false);
  APP_ERROR_CHECK(err_code);

  /* Nap sleep until Xfer is not done */
  system_sleep_nap_mode(&bI2CXferDone);  

  // START
  // TX &REG_SYS_CONTROL
  // TX REG_SYS_CONTROL
  // TX REG_PPG_CONFIG_1
  // TX REG_PPG_CONFIG_2
  // TX REG_PROX_INT_THRESH
  // TX REG_LED1_PA
  // TX REG_LED2_PA
  // STOP
  m_i2c_tx_buffer[0] = (uint8_t)(SENSOR_MAX86150_REG_ADDR(REG_SYS_CONTROL));
  m_i2c_tx_buffer[1] = max86150_reg_sys_control.reg_byte;
  m_i2c_tx_buffer[2] = max86150_reg_ppg_config_1.reg_byte;
  m_i2c_tx_buffer[3] = max86150_reg_ppg_config_2.reg_byte;
  m_i2c_tx_buffer[4] = max86150_reg_prox_int_thresh.reg_byte;
  m_i2c_tx_buffer[5] = max86150_reg_led1_pa.reg_byte;
  m_i2c_tx_buffer[6] = max86150_reg_led2_pa.reg_byte;

  bI2CXferDone = false;  
  err_code = nrf_drv_twi_tx(&i2c_controller_instance,
                            SENSOR_MAX86150_I2C_RESPONDER_ADDR,
                            m_i2c_tx_buffer,
                            7,
                            false);
  APP_ERROR_CHECK(err_code);

  /* Nap sleep until Xfer is not done */
  system_sleep_nap_mode(&bI2CXferDone);  

  // START
  // TX &REG_LED_RANGE
  // TX REG_LED_RANGE
  // TX REG_LED_PILOT_PA
  // STOP
  m_i2c_tx_buffer[0] = (uint8_t)(SENSOR_MAX86150_REG_ADDR(REG_LED_RANGE));
  m_i2c_tx_buffer[1] = max86150_reg_led_range.reg_byte;
  m_i2c_tx_buffer[2] = max86150_reg_led_pilot_pa.reg_byte;

  bI2CXferDone = false;  
  err_code = nrf_drv_twi_tx(&i2c_controller_instance,
                            SENSOR_MAX86150_I2C_RESPONDER_ADDR,
                            m_i2c_tx_buffer,
                            3,
                            false);
  APP_ERROR_CHECK(err_code);

  /* Nap sleep until Xfer is not done */
  system_sleep_nap_mode(&bI2CXferDone);  
  
  // START
  // TX &REG_ECG_CONFIG_1
  // TX REG_ECG_CONFIG_1
  // STOP
  m_i2c_tx_buffer[0] = (uint8_t)(SENSOR_MAX86150_REG_ADDR(REG_ECG_CONFIG_1));
  m_i2c_tx_buffer[1] = max86150_reg_ecg_config_1.reg_byte;

  bI2CXferDone = false;  
  err_code = nrf_drv_twi_tx(&i2c_controller_instance,
                            SENSOR_MAX86150_I2C_RESPONDER_ADDR,
                            m_i2c_tx_buffer,
                            2,
                            false);
  APP_ERROR_CHECK(err_code);

  /* Nap sleep until Xfer is not done */
  system_sleep_nap_mode(&bI2CXferDone);

  // START
  // TX &REG_ECG_CONFIG_3
  // TX REG_ECG_CONFIG_3
  // STOP
  m_i2c_tx_buffer[0] = (uint8_t)(SENSOR_MAX86150_REG_ADDR(REG_ECG_CONFIG_3));
  m_i2c_tx_buffer[1] = max86150_reg_ecg_config_3.reg_byte;

  bI2CXferDone = false;
  err_code = nrf_drv_twi_tx(&i2c_controller_instance,
                            SENSOR_MAX86150_I2C_RESPONDER_ADDR,
                            m_i2c_tx_buffer,
                            2,
                            false);
  APP_ERROR_CHECK(err_code);

  /* Nap sleep until Xfer is not done */
  system_sleep_nap_mode(&bI2CXferDone);
}


/* Read FIFO data when there are 17 samples 
   For this, wait for interrupt A_FULL in Interrupt Status 1 register
*/
void sensor_max86150_get_fifo_data(volatile uint32_t * p_ppg_led1_samples,
                                       volatile uint32_t * p_ppg_led2_samples,
                                       volatile uint32_t * p_ecg_samples,
                                       volatile uint8_t  * p_num_valid_samples)
{
  ret_code_t err_code = NRF_SUCCESS;
  // uint8_t FIFO_WR_PTR;
  // uint8_t OVF_COUNTER;
  // uint8_t FIFO_RD_PTR;
  uint8_t NUM_AVAILABLE_SAMPLES;
  uint8_t bIsFifoFull = false;
  uint32_t PPG_LED1_SAMPLE=0;
  uint32_t PPG_LED2_SAMPLE=0;
  uint32_t ECG_SAMPLE=0;

  /* 1. TRANSACTION 1: Get the FIFO_WR_PTR and FIFO_RD_PTR:

        START;
        Send device address + write mode
        Send address of FIFO_WR_PTR;
        REPEATED_START;
        Send device address + read mode
        Read FIFO_WR_PTR;
        Read OVF_COUNTER;
        Read FIFO_RD_PTR;
        STOP;
  */
  m_i2c_tx_buffer[0] = (uint8_t)(SENSOR_MAX86150_REG_ADDR(REG_FIFO_WR_PTR));
  m_i2c_xfer_desc.type             = NRF_DRV_TWI_XFER_TXRX;
  m_i2c_xfer_desc.address          = SENSOR_MAX86150_I2C_RESPONDER_ADDR;
  m_i2c_xfer_desc.p_primary_buf    = &(m_i2c_tx_buffer[0]);
  m_i2c_xfer_desc.primary_length   = 1;
  m_i2c_xfer_desc.p_secondary_buf  = &(m_sensor_max86150_regs[4]);
  m_i2c_xfer_desc.secondary_length = 3;

  /* 2. Evaluate the number of samples to be read from the FIFO:
        If OVF_COUNTER is zero,
          NUM_AVAILABLE_SAMPLES = FIFO_WR_PTR – FIFO_RD_PTR
          (Note: pointer wrap around should be taken into account)
        else If OVF_COUNTER is non-zero 
          some samples are lost, and
          NUM_AVAILABLE_SAMPLES = 32
        
        NUM_SAMPLES_TO_READ <= NUM_AVAILABLE_SAMPLES
  */

  // do{
  //   bI2CXferDone == false;
  //   err_code = nrf_drv_twi_xfer(&i2c_controller_instance, &m_i2c_xfer_desc, 0); // Non-blocking since a handler is associated with this I2C instance. With TWI, the only supported flag is NRF_DRV_TWI_FLAG_TX_NO_STOP. All other flags require TWIM.
  //   APP_ERROR_CHECK(err_code);

  //   /* Nap sleep until Xfer is not done */
  //   system_sleep_nap_mode(&bI2CXferDone);

  //   FIFO_WR_PTR = m_sensor_max86150_regs[4];
  //   OVF_COUNTER = m_sensor_max86150_regs[5];
  //   FIFO_RD_PTR = m_sensor_max86150_regs[6];
  //   NUM_AVAILABLE_SAMPLES = FIFO_WR_PTR - FIFO_RD_PTR;
  //   if( OVF_COUNTER ) NUM_AVAILABLE_SAMPLES = 32;
  // } while ( NUM_AVAILABLE_SAMPLES < 17 );

  /* Alternatively, check if A_FULL interrupt is raised when 17 samples are written to FIFO
      START
      TX &REG_INT_STATUS_1
      RX REG_INT_STATUS_1
      STOP
  */
  m_i2c_tx_buffer[0] = (uint8_t)(SENSOR_MAX86150_REG_ADDR(REG_INT_STATUS_1));
  m_i2c_xfer_desc.type             = NRF_DRV_TWI_XFER_TXRX;
  m_i2c_xfer_desc.address          = SENSOR_MAX86150_I2C_RESPONDER_ADDR;
  m_i2c_xfer_desc.p_primary_buf    = &(m_i2c_tx_buffer[0]);
  m_i2c_xfer_desc.primary_length   = 1;
  m_i2c_xfer_desc.p_secondary_buf  = &(m_sensor_max86150_regs[0]);
  m_i2c_xfer_desc.secondary_length = 1;

  do{
    bI2CXferDone = false;
    err_code = nrf_drv_twi_xfer(&i2c_controller_instance, &m_i2c_xfer_desc, 0 ); // Non-blocking since a handler is associated with this I2C instance. With TWI, the only supported flag is NRF_DRV_TWI_FLAG_TX_NO_STOP. All other flags require TWIM.
    APP_ERROR_CHECK(err_code);

    /* Nap sleep until Xfer is not done */
    system_sleep_nap_mode(&bI2CXferDone);
    bIsFifoFull = IS_SET_MSK(m_sensor_max86150_regs[0],MAX86150_REG_INT_STATUS_1_A_FULL_Msk, uint8_t);
    if(bIsFifoFull) NUM_AVAILABLE_SAMPLES = 17;
  } while(!bIsFifoFull);

  /* 3. TRANSACTION 2: Read NUM_SAMPLES_TO_READ samples from the FIFO: 

        START;
        Send device address + write mode
        Send address of FIFO_DATA;
        REPEATED_START;
        Send device address + read mode
        for (i = 0; i < NUM_SAMPLES_TO_READ; i++) {
        Read FIFO_DATA;
        Save Data_Item1[23:16];
        Read FIFO_DATA;
        Save Data_Item1[15:8];
        Read FIFO_DATA;
        Save Data_Item1[7:0];
        Read FIFO_DATA;
        Save Data_Item2[23:16];
        Read FIFO_DATA;
        Save Data_Item2[15:8];
        Read FIFO_DATA;
        Save Data_Item2[7:0];
        Read FIFO_DATA;
        Save Data_Item3[23:16];
        Read FIFO_DATA;
        Save Data_Item3[15:8];
        Read FIFO_DATA;
        Save Data_Item3[7:0];
        }
        STOP;
  */
  m_i2c_tx_buffer[0] = (uint8_t)(SENSOR_MAX86150_REG_ADDR(REG_FIFO_DATA));
  m_i2c_xfer_desc.type             = NRF_DRV_TWI_XFER_TXRX;
  m_i2c_xfer_desc.address          = SENSOR_MAX86150_I2C_RESPONDER_ADDR;
  m_i2c_xfer_desc.p_primary_buf    = &(m_i2c_tx_buffer[0]);
  m_i2c_xfer_desc.primary_length   = 1;
  m_i2c_xfer_desc.p_secondary_buf  = m_fifo_bytes;
  m_i2c_xfer_desc.secondary_length = NUM_AVAILABLE_SAMPLES*9;

  bI2CXferDone = false;
  err_code = nrf_drv_twi_xfer(&i2c_controller_instance, &m_i2c_xfer_desc, 0 ); // Non-blocking since a handler is associated with this I2C instance. With TWI, the only supported flag is NRF_DRV_TWI_FLAG_TX_NO_STOP. All other flags require TWIM. 
  APP_ERROR_CHECK(err_code);

  /* Nap sleep until Xfer is not done */
  system_sleep_nap_mode(&bI2CXferDone);

  /* 4. TRANSACTION 3: Write to FIFO_RD_PTR register. 
      
      If the pervious transaction was successful, 
        FIFO_RD_PTR points to the next sample in the FIFO, and this transaction is not necessary. 
      Otherwise, 
        the AP updates the FIFO_RD_PTR appropriately to New_FIFO_RD_PTR, so that the samples are reread.

        START;
        Send device address + write mode
        Send address of FIFO_RD_PTR;
        Write New_FIFO_RD_PTR;
        STOP;
  */

  /* Extract FD1=LED1, FD2=LED2 and FD3=ECG samples. 
     - A sample read from FIFO has 3 valid elements (4th element is not used) in order FD1=LED1, FD2=LED2, FD3=ECG 
     - Each element is of length 3-bytes. 
     - So each sample has 3 elements/sample x 3-bytes/element = 9 bytes
     - First 3-byte transfer from FIFO form the bit slice [23:16], [15:8]. [7:0] for FD1 (0,1,2|9,10,11)
     - Second 3-byte transfer from FIFO form the bit slice [23:16], [15:8]. [7:0] for FD2 (3,4,5|)
     - Third 3-byte transfer from FIFO form the bit slice [23:16], [15:8]. [7:0] for FD3  (6,7,8|) 
   */
  for( uint8_t i = 0; i < NUM_AVAILABLE_SAMPLES; ++i)
  {
    PPG_LED1_SAMPLE = (((uint32_t) m_fifo_bytes[9*i]) << 16) | (((uint32_t) m_fifo_bytes[1+9*i]) << 8) | (((uint32_t) m_fifo_bytes[2+9*i]));
    PPG_LED1_SAMPLE = PPG_LED1_SAMPLE & FIFO_PPG_Msk;
    PPG_LED2_SAMPLE = (((uint32_t) m_fifo_bytes[3+9*i]) << 16) | (((uint32_t) m_fifo_bytes[4+9*i]) << 8) | (((uint32_t) m_fifo_bytes[5+9*i]));
    PPG_LED2_SAMPLE = PPG_LED2_SAMPLE & FIFO_PPG_Msk;
    ECG_SAMPLE = (((uint32_t) m_fifo_bytes[6+9*i]) << 16) | (((uint32_t) m_fifo_bytes[7+9*i]) << 8) | (((uint32_t) m_fifo_bytes[8+9*i]));
    ECG_SAMPLE = ECG_SAMPLE & FIFO_ECG_Msk;
    p_ppg_led1_samples[i] = PPG_LED1_SAMPLE;
    p_ppg_led2_samples[i] = PPG_LED2_SAMPLE;
    p_ecg_samples[i] = ECG_SAMPLE;
  }

  *p_num_valid_samples  = NUM_AVAILABLE_SAMPLES;
}
