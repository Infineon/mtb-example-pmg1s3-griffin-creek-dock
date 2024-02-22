/******************************************************************************
* File Name: solution.h
*
* Description: This header file defines the data structures and function
*              prototypes associated with Galactico Greek Dock solution.
*
* Related Document: See README.md
*
******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef SRC_APP_SOLUTION_SOLUTION_H_
#define SRC_APP_SOLUTION_SOLUTION_H_

#include "cy_pdaltmode_defines.h"
#include "cy_pdstack_common.h"
#include "cy_app_vdm.h"
#if CY_APP_SMART_POWER_ENABLE
#include "cy_app_smart_power.h"
#endif /* CY_APP_SMART_POWER_ENABLE */

/* CSA I2C Addresses. */
#define MAIN_BOARD_CURRENT_SENSE_ADDR        (0x29U)
#define IO_BOARD_CURRENT_SENSE_ADDR          (0x2AU)
#define AUDIO_BOARD_CURRENT_SENSE_ADDR       (0x2BU)

#define R_SENSE                              (5u)    /* 5 milli ohm resistance on Galactico Creek Board */
#define FULL_SCALE_RANGE                     (80u)   /* Default value 80mV. Can configure either 10mV, 20mV, 40mV or 80mV */
#define FULL_SCALE_CURRENT                   (FULL_SCALE_RANGE/R_SENSE)

#define FULL_SCALE_VOLTAGE                   (24u)   /* Constant value 23.9883 */
#define FULL_SCALE_POWER                     (FULL_SCALE_CURRENT * FULL_SCALE_VOLTAGE)

typedef enum
{
    VOLTAGE_SAMPLING_CONFIG = 0x50U,
    CURRENT_SENCE_SAMPLING_CONFIG = 0x51U,
    PEAK_DETECTION_CONFIG = 0x52U,
    SENSE_VOLTAGE_HIGH_BYTE = 0x54U,
    SENSE_VOLTAGE_LOW_BYTE = 0x55U,
    SOURCE_VOLTAGE_HIGH_BYTE = 0x58U,
    SOURCE_VOLTAGE_LOW_BYTE = 0x59U,
    POWER_RATIO_HIGH_BYTE = 0x5BU,
    POWER_RATIO_LOW_BYTE = 0x5CU,
    PRODUCT_ID = 0xFDU,
    PRODUCT_SMSC_ID = 0xFFU
}emc1702_reg_addr_t;

/* The adapter power watts definitions */
#define ADAPTER_POWER_120W              (120u)
#define ADAPTER_POWER_135W              (135u)
#define ADAPTER_POWER_150W              (150u)
#define ADAPTER_POWER_180W              (180u)
#define ADAPTER_POWER_200W              (200u)
#define ADAPTER_POWER_230W              (230u)
#define ADAPTER_POWER_280W              (280u)
#define ADAPTER_POWER_350W              (350u)

/**
 * @brief Initialize Button Interrupt callback functions.
 *
 * These functions are called by pin interrupts and handles different button states (press, relase and etc).
 *
 * @param None.
 *
 * @return None.
 */
#if (EXTENDED_ALERT_EVENTS_SUPPORT || BUTTON_PRESS_FACTORY_RESET)
void power_button_intr_cb (void);
#endif /* (EXTENDED_ALERT_EVENTS_SUPPORT || BUTTON_PRESS_FACTORY_RESET) */

#if EXTENDED_ALERT_EVENTS_SUPPORT
void lan_button_intr_cb (void);
#endif /* EXTENDED_ALERT_EVENTS_SUPPORT */

#if CY_APP_SMART_POWER_ENABLE
uint16_t measure_adapter_power(cy_stc_usbpd_context_t *context);

uint16_t measure_total_dock_current(cy_stc_app_smart_power_context_t *context);

uint16_t measure_us_current(cy_stc_app_smart_power_context_t *context);

uint16_t measure_adapter_voltage(void);
#endif /* CY_APP_SMART_POWER_ENABLE */

void solution_init (cy_stc_pdstack_context_t* ptrPdStackContext);
void sln_pd_event_handler(cy_stc_pdstack_context_t* ctx, cy_en_pdstack_app_evt_t evt, const void *data);

#endif /* SRC_APP_SOLUTION_SOLUTION_H_ */
