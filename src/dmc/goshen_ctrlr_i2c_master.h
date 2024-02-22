/******************************************************************************
* File Name: goshen_ctrlr_i2c_master.h
*
* Description: GoshenRidge (GR) Controller I2C master header file.
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

#ifndef _GOSHEN_CTRLR_I2C_MASTER_H_
#define _GOSHEN_CTRLR_I2C_MASTER_H_

#include <stdint.h>
#include "cy_app_status.h"

/*****************************************************************************
 ********************************* Macros ************************************
 *****************************************************************************/
#define NVM_OFFSET_READ_MAJOR_VER       (0x0Au)
#define NVM_OFFSET_READ_MINOR_VER       (0x09u)
#define NVM_VENDOR_DATA                 (0x2B4u)    
 
#define NVM_READ_VER_LENGTH             (1u)  
#define MAILBOX_REG_ADDR_07             (7u)
#define MAILBOX_REG_ADDR_06             (6u)
#define WRITE_READ_MIN_SIZE             (sizeof(uint32_t))    
#define NVM_OP_CMD_READ                 (0)
#define NVM_OP_CMD_WRITE                (1)    
#define NVM_OP_START                    (1)    
#define NVM_WAIT_FOR_COMLT_TIMEOUT_IN_MS    (1000u)
#define DELAY_BETWEEN_READS             (5u)
#define NVM_AUTH_STATUS_OFFSET          (24u)
#define NVM_AUTH_STATUS_MASK            (0x3Fu)

/*****************************************************************************
 ******************************** Data Struct Definition *********************
 *****************************************************************************/
/* USB4 1.0 Spec October-2020. */    
typedef enum
{
    NVM_WRITE_OP_CODE   = 0x20,
    NVM_AUTH_WRITE      = 0x21,
    NVM_READ            = 0x22,
    NVM_SET_OFFSET      = 0x23
}router_oper_op_code_t;

typedef enum
{
    SPI_DATA_00     = 0,
    SPI_DATA_01     = 1,
    SPI_DATA_02     = 2,
    SPI_DATA_03     = 3,
    SPI_DATA_04     = 4,
    SPI_DATA_05     = 5,
    SPI_DATA_06     = 6,
    SPI_DATA_07     = 7,
    SPI_DATA_08     = 8,
    SPI_DATA_09     = 9,
    SPI_DATA_0A     = 0xA,
    SPI_DATA_0B     = 0xB,
    SPI_DATA_0C     = 0xC,
    SPI_DATA_0D     = 0xD,
    SPI_DATA_0E     = 0xE,
    SPI_DATA_0F     = 0xF,
    METADATA        = 0x10,
    OPCODE          = 0x11    
}mailbox_reg6_byte0_vals;

typedef union
{
    uint32_t val;
    
    struct
    {
        uint32_t reserverd_01   : 2;
        uint32_t nvm_offset     : 22;
        uint32_t length         : 4;
        uint32_t reserved_02    : 4;
    }nvm_read;
}nvm_read_op_metadata_t;

typedef union
{
    uint32_t val;
    struct
    {
        uint8_t     data_opcode_metadata;
        uint32_t    reserved01          :  21;
        uint32_t    op_cmd_rd_wr        :  1;
        uint32_t    error_bit           :  1;
        uint32_t    operation_start     :  1;
    };
}mailbox_reg_6_t;

typedef union
{
    uint32_t data;
    
    struct
    {
        uint8_t length;
        uint8_t status[4];
    };
}mailbox_reg_read_t;

typedef struct
{
    uint8_t                 *pbuf;
    uint32_t                addr_offset;
    router_oper_op_code_t   opcode;
    uint8_t                 length;
}nvm_operation_t;

/*****************************************************************************
 **************************** Function prototypes ****************************
 *****************************************************************************/

/**
 * @brief Reads FW version of TBT controller.
 *
 * @param scb_index SCB block index.
 * @param slave_addr I2C slave address of TBT controller.
 * @param ver Buffer to hold version data
 *
 * @return cy_en_app_status_t: CCG_STAT_SUCCESS if success,
 *         CCG_STAT_TIMEOUT if response is command is not successful.
 */
cy_en_app_status_t goshen_nvm_page_write(CySCB_Type *base, uint8_t slave_addr, uint8_t *pinbuf, uint8_t length,cy_stc_scb_i2c_context_t *context);
cy_en_app_status_t goshen_nvm_authenticate_write(CySCB_Type *base, uint8_t slave_addr, cy_stc_scb_i2c_context_t *context);
cy_en_app_status_t goshen_nvm_set_offset(CySCB_Type *base, uint8_t slave_addr, uint32_t offset, cy_stc_scb_i2c_context_t *context);
cy_en_app_status_t goshen_nvm_fw_ver_read(CySCB_Type *base, uint8_t slave_addr, uint8_t *pbuf, cy_stc_scb_i2c_context_t *context);

#endif /* _GOSHEN_CTRLR_I2C_MASTER_H_ */
