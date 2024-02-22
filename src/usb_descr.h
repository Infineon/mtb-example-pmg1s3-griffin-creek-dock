/*******************************************************************************
* File Name: usb_descr.h
*
* Description: USB Descriptor header file.
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

#ifndef SRC_APP_USB_DESCR_H_
#define SRC_APP_USB_DESCR_H_

#define GET_LSB(a) ((a) & (0xFFU))
#define GET_MSB(a) (((a)>>(8)) & (0xFFU))

/* Interface number for Billboard interface. */
#define BILLBOARD_INTERFACE_NUMBER              0x00U

/* Interface number for Vendor interface. */
#define VENDOR_INTERFACE_NUMBER                 0x01U

/* Number of interfaces supported. */
#define NO_OF_INTERFACES                        0x02

/* Length of Configuration Descriptor. */
#define CONFIG_DESCR_LENGTH                     0x09U

/* Offset of Billboard interface descriptor in configuration descriptor. */
#define BILLBOARD_INTERFACE_DESCR_OFFSET        CONFIG_DESCR_LENGTH

/* Length of billboard interface descriptor. */
#define BILLBOARD_INTERFACE_DESCR_LENGTH        0x09U

/* Offset of Vendor interface descriptor in configuration descriptor. */
#define VENDOR_INTERFACE_DESCR_OFFSET           BILLBOARD_INTERFACE_DESCR_OFFSET + BILLBOARD_INTERFACE_DESCR_LENGTH

/* Length of vendor interface descriptor. */
#define VENDOR_INTERFACE_DESCR_LENGTH           0x09U

/* Offset of OUT endpoint descriptor for Vendor interface in configuration descriptor. */
#define VENDOR_INTERFACE_EP_OUT_DESCR_OFFSET    VENDOR_INTERFACE_DESCR_OFFSET + VENDOR_INTERFACE_DESCR_LENGTH

/* Length of OUT endpoint descriptor for vendor interface. */
#define VENDOR_INTERFACE_EP_OUT_DESCR_LENGTH    0x07U

/* Offset of IN endpoint descriptor for Vendor interface in configuration descriptor. */
#define VENDOR_INTERFACE_EP_IN_DESCR_OFFSET     VENDOR_INTERFACE_EP_OUT_DESCR_OFFSET + VENDOR_INTERFACE_EP_OUT_DESCR_LENGTH

/* Length of IN endpoint descriptor for vendor interface. */
#define VENDOR_INTERFACE_EP_IN_DESCR_LENGTH     0x07U

/* Total length of vendor interface descriptor and endpoint descriptors. */
#define VENDOR_INTERFACE_TOTAL_DESCR_LENGTH     VENDOR_INTERFACE_DESCR_LENGTH + \
                                                VENDOR_INTERFACE_EP_OUT_DESCR_LENGTH + VENDOR_INTERFACE_EP_IN_DESCR_LENGTH


/* Total Length of configuration descriptor. */
#define USB_CONFIG_DESC_SIZE                    CONFIG_DESCR_LENGTH + BILLBOARD_INTERFACE_DESCR_LENGTH + VENDOR_INTERFACE_TOTAL_DESCR_LENGTH

#define MAX_SIZE_OF_BOS_DESC                    (0xACu)

#endif /* SRC_APP_USB_DESCR_H_ */
