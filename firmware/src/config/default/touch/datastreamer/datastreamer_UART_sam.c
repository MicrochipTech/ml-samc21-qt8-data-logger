/*******************************************************************************
  Touch Library v3.7.0 Release

  Company:
    Microchip Technology Inc.

  File Name:
    datastreamer_UART_sam.c

  Summary:
    QTouch Modular Library

  Description:
    Provides the datastreamer protocol implementation, transmission of
          module data to data visualizer software using UART port.
	
*******************************************************************************/

/*******************************************************************************
Copyright (c)  2020 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/

/*----------------------------------------------------------------------------
  include files
----------------------------------------------------------------------------*/
#include "touch/datastreamer/datastreamer.h"
#include "definitions.h"

#if (DEF_TOUCH_DATA_STREAMER_ENABLE == 1u)

/*----------------------------------------------------------------------------
 *     defines
 *----------------------------------------------------------------------------*/
#define ACQ_MODULE_AUTOTUNE_OUTPUT 0
#define FREQ_HOP_AUTO_MODULE_OUTPUT 1
#define SCROLLER_MODULE_OUTPUT 0
#define SURFACE_MODULE_OUTPUT 1

/*----------------------------------------------------------------------------
  global variables
----------------------------------------------------------------------------*/
extern qtm_acquisition_control_t qtlib_acq_set1;
extern qtm_touch_key_control_t   qtlib_key_set1;
extern qtm_touch_key_config_t    qtlib_key_configs_set1[DEF_NUM_SENSORS];
extern qtm_freq_hop_autotune_control_t qtm_freq_hop_autotune_control1;
extern qtm_surface_cs_control_t qtm_surface_cs_control1;
extern uint8_t module_error_code;

uint8_t data[] = {
    0x5F, 0xB4, 0x00, 0x86, 0x4A, 0x03, 0xEB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA, 0x55, 0x01, 0x6E, 0xA0};

/*----------------------------------------------------------------------------
  prototypes
----------------------------------------------------------------------------*/
void datastreamer_transmit(uint8_t data);

/*----------------------------------------------------------------------------
 *   function definitions
 *----------------------------------------------------------------------------*/

/*============================================================================
void datastreamer_init(void)
------------------------------------------------------------------------------
Purpose: Initialization for datastreamer module
Input  : none
Output : none
Notes  :
============================================================================*/
void datastreamer_init(void)
{
}

/*============================================================================
void datastreamer_transmit(uint8_t data_byte)
------------------------------------------------------------------------------
Purpose: Transmits the single byte through the configured UART port.
Input  : Byte to be transmitted
Output : none
Notes  :
============================================================================*/
void datastreamer_transmit(uint8_t data_byte)
{
	/* Write the data bye */
   SERCOM4_USART_Write(&data_byte, 1);
}

/*============================================================================
void datastreamer_touch_position_output(void)
------------------------------------------------------------------------------
Purpose: Transmits the touch position from the touch library in data streamer
         format for decoding with MPLAB Data Visualizer
Input  : none
Output : none
Notes  :
============================================================================*/
void datastreamer_touch_position_output(void)
{
	uint16_t          u16temp_output;

	/* Start token */
	datastreamer_transmit(0x55);
        
	/* horizontal position */
	u16temp_output = qtm_surface_cs_control1.qtm_surface_contact_data->h_position;
	datastreamer_transmit((uint8_t)u16temp_output);
	datastreamer_transmit((uint8_t)(u16temp_output >> 8u));

	/* vertical position */
	u16temp_output = qtm_surface_cs_control1.qtm_surface_contact_data->v_position;
	datastreamer_transmit((uint8_t)u16temp_output);
	datastreamer_transmit((uint8_t)(u16temp_output >> 8u));

	/* End token */
	datastreamer_transmit(~0x55);
}

#endif
