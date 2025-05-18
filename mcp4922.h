/**
********************************************************************************
* @file     mcp4922.h
* @author  	Binh Pham
* @version 	1.0
* @date    	2025-05-18
* @brief    C driver for DAC 12bit dual channel MCP4922E.
* @details
* The MCP4922 is a dual 12-bit voltage output Digital-to-Analog Converter (DAC)
* with an SPI interface. It features two independent DAC channels (A and B),
* allowing for simultaneous or individual updates. The device supports fast
* data rates, low power consumption, and includes features such as selectable
* gain, output buffer, and shutdown modes for flexible analog signal generation.
* Typical applications include waveform generation, programmable voltage
* sources, and audio signal processing.
********************************************************************************
* @attention
*
********************************************************************************
*/

#include <stdint.h>

/**
 * MCP4922 is a very simple DAC, so I include an example below
 */

/*
void RTOS_DAC_task(void *argument)
{
  spi_hdl_t spi1;
  hal_spi_init(&spi1)

  // Set up neccessary function pointers to ADS1115
  mcp4922_ops_t ops = {
    .write = hal_spi_write,
  };
  mcp4922_set_hw_ops(&ops);

  for (;;)
  {
    mcp4922_set(MCP4922_CH_A, MCP4922_BUF_DISABLE, MCP4922_GAIN_1X, 50.0);
    mcp4922_set(MCP4922_CH_B, MCP4922_BUF_DISABLE, MCP4922_GAIN_1X, 75.0);
    osDelay(3000);
  }
}

*/
/* Public variables ----------------------------------------------------------*/

/**
 * @brief mcp4922 channels
 */
typedef enum
{
  MCP4922_CH_A,
  MCP4922_CH_B,
} mcp4922_ch_e;

/**
 * @brief mcp4922 buffered voltage ( + 0.040V)
 */
typedef enum
{
  MCP4922_BUF_DISABLE,
  MCP4922_BUF_ENABLE,
} mcp4922_buf_e;
/**
 * @brief Gain Vref to x1 or x2, gained value wont exceed Vdd.
 */
typedef enum
{
  MCP4922_GAIN_2X,
  MCP4922_GAIN_1X,
} mcp4922_gain_e;

/**
 * SETUPS FUNCTIONS
 * =============================================================================
 */

/**
 * @brief MCU independant variable
 *
 */
typedef struct
{
  void (*write)(uint8_t *data, uint8_t size);
} mcp4922_ops_t;

void mcp4922_set_hw_ops(mcp4922_ops_t *ops);
void mcp4922_write(uint8_t *data, uint8_t size);

/**
 * OPERATION FUNCTIONS
 * =============================================================================
 */

void mcp4922_set(mcp4922_ch_e ch, mcp4922_buf_e buf, mcp4922_gain_e gain,
                 double percentage);
void mcp4922_disable(mcp4922_ch_e ch);

/*****************************END OF FILE**************************************/