/**
********************************************************************************
* @file     mcp4922.c
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

#include "mcp4922.h"

/**
 * SETUPS COMPONENTS
 * =============================================================================
 */

/* Private defines -----------------------------------------------------------*/
/* Private enumerate/structure -----------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static mcp4922_ops_t hw_ops;
/* Private function prototypes -----------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/

/**
 * @brief   NONE
 *
 * @note    NONE
 *
 * @param   NONE
 *
 * @return  NONE
 */
void mcp4922_set_hw_ops(mcp4922_ops_t *ops)
{
  hw_ops = *ops; // Copy the function pointers
}

/**
 * @brief   NONE
 *
 * @note    NONE
 *
 * @param   NONE
 *
 * @return  NONE
 */
void mcp4922_write(uint8_t *data, uint8_t size)
{
  if (hw_ops.write)
  {
    hw_ops.write(data, size);
  }
  return; // Error: function not set
}

/**
 * OPERATION FUNCTIONS
 * =============================================================================
 */

/**
 * @brief   Set output value for MCP4922 DAC channel.
 *
 * @param   ch         Channel selection (MCP4922_CH_A or MCP4922_CH_B)
 * @param   buf        Buffer mode (MCP4922_BUF_UNBUFFERED or
 * MCP4922_BUF_BUFFERED)
 * @param   gain       Gain selection (MCP4922_GAIN_1X or MCP4922_GAIN_2X)
 * @param   percentage Output value as a percentage (0.0 - 100.0)
 *
 * @note    Converts the percentage to a 12-bit DAC value and sends the command
 *          to the MCP4922 via the configured hardware write function.
 */
void mcp4922_set(mcp4922_ch_e ch, mcp4922_buf_e buf, mcp4922_gain_e gain,
                 double percentage)
{
  // Check if percentage is within valid range
  if (percentage > 100.0 || percentage < 0.0)
  {
    return;
  }

  uint16_t cmd = 0;
  cmd |= ((ch & 0x01) << 15);   // Bit 15: Channel select (A=0, B=1)
  cmd |= ((buf & 0x01) << 14);  // Bit 14: Buffer (1 = buffered, 0 = unbuffered)
  cmd |= ((gain & 0x01) << 13); // Bit 13: Gain (1 = 1x, 0 = 2x)
  cmd |= (1 << 12);             // Bit 12: Shutdown (1 = active, 0 = shutdown)

  // Convert percentage to 12-bit DAC value (0-4095)
  uint16_t out_bit = (percentage * 4096.0 / 100.0);
  cmd |= (out_bit & 0x0FFF);

  // Prepare data buffer for transmission (MSB first)
  uint8_t tx_buf[2];
  tx_buf[0] = (cmd >> 8) & 0xFF;
  tx_buf[1] = cmd & 0xFF;

  // Send command to MCP4922
  mcp4922_write(tx_buf, 2);
}

/**
 * @brief   Disable a MCP4922 DAC channel.
 *
 * @param   ch         Channel selection (MCP4922_CH_A or MCP4922_CH_B)
 *
 */
void mcp4922_disable(mcp4922_ch_e ch)
{
  uint16_t cmd = 0;
  cmd |= ((ch & 0x01) << 15); // Bit 15: Channel select (A=0, B=1)
  cmd |= (0 << 12);           // Bit 12: Shutdown (1 = active, 0 = shutdown)

  // Prepare data buffer for transmission (MSB first)
  uint8_t tx_buf[2];
  tx_buf[0] = (cmd >> 8) & 0xFF;
  tx_buf[1] = cmd & 0xFF;

  // Send command to MCP4922
  mcp4922_write(tx_buf, 2);
}

/*****************************END OF FILE**************************************/