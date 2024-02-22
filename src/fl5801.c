/******************************************************************************
 * File Name: fl5801.c
 *
 * Description: This is source code for the communication with FL5801.
 *
 * Related Document: See README.md
 *
 *******************************************************************************
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

#include "cy_pdl.h"
#include "cybsp.h"
#include "config.h"
#include "fl5801.h"
#include "cy_app_i2c_master.h"
#include "cy_app_status.h"

#define FL5801_I2C_SLAVE_ADDR                   (0x38u)
#define FL5801_I2C_REG_SIZE                     (2u)
#define FL5801_REG_UUID_ADDR                    (0x2047)

extern cy_stc_scb_i2c_context_t  gl_i2c_pm_context;

bool ridge_usb2_hub_read_uuid(uint8_t *uuid)
{
    bool status;
    uint16_t reg_data = FL5801_REG_UUID_ADDR;
    status = Cy_App_I2CMaster_RegRead (I2C_PM_HW, FL5801_I2C_SLAVE_ADDR, (uint8_t *)(&reg_data), FL5801_I2C_REG_SIZE,
                            uuid, UUID_SIZE, &gl_i2c_pm_context);
    return status;
}

/* [] END OF FILE */
