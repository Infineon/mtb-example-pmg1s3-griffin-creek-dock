/******************************************************************************
* File Name: config.c
*
* Description: Contains the DMC configuration information. Please refer to
* the the EZ-PD Dock Reference Guide for details of the configuration table.
*
* Related Document: See README.md
*
*******************************************************************************
* $ Copyright 2023-YEAR Infineon Technologies $
*******************************************************************************/

#include <cy_usbpd_config_table.h>
/*
   The config table should be placed in the "configSection" so that the update
   tools can correctly retrieve the data.
 */
const unsigned char __attribute__((section (".configSection"), used)) gl_config_table[0xA00]=
{
    0x46, 0x49, 0x04, 0x00, 0x00, 0x60, 0xAC, 0x08, 0x4C, 0xF9, 0xFA, 0x70, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xD8, 0x01, 0x4C, 0x00, 0xA4, 0x03, 0x4C, 0x00, 0xF0, 0x03, 0x90, 0x00,
    0x80, 0x04, 0x10, 0x00, 0x90, 0x04, 0x2A, 0x02, 0xBC, 0x06, 0x90, 0x01, 0x4C, 0x08, 0x20, 0x00,
    0x43, 0x53, 0x44, 0x50, 0x00, 0x01, 0x0D, 0x00, 0xB4, 0x04, 0x05, 0xF5, 0x49, 0x6E, 0x66, 0x69,
    0x6E, 0x65, 0x6F, 0x6E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x01, 0xB4, 0x04, 0x05, 0xF5, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x64, 0x8C, 0x00, 0x00, 0x00,
    0xB4, 0x04, 0x05, 0xF5, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0A, 0x05, 0x05, 0x14, 0x00, 0x00, 0x00, 0x02, 0x01, 0x02, 0x14, 0x78, 0x02, 0x60, 0x02,
    0x01, 0x07, 0x01, 0x02, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x05, 0x1F, 0x01, 0x01,
    0x2C, 0x91, 0x81, 0x27, 0x2C, 0xD1, 0x02, 0x00, 0x2C, 0xB1, 0x04, 0x00, 0xF4, 0x41, 0x06, 0x00,
    0x64, 0x32, 0xA4, 0xC9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x01, 0x26,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x31, 0x8C, 0x8C, 0x8C, 0x00,
    0x02, 0x03, 0x00, 0x00, 0xF4, 0xC1, 0x08, 0x00, 0x8C, 0x96, 0x30, 0xD2, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0xB4, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x41, 0xA0, 0x00, 0xFF,
    0xB4, 0x04, 0x40, 0x4C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0xF5, 0x3B, 0x00, 0x00, 0x6D,
    0x10, 0x00, 0x00, 0x00, 0x42, 0xA0, 0x00, 0xFF, 0x01, 0xFF, 0x87, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x0C, 0x00, 0x00, 0x00, 0x43, 0xA0, 0x87, 0x80, 0x01, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00,
    0x43, 0xA0, 0x01, 0xFF, 0x45, 0x00, 0x1C, 0x00, 0x00, 0x03, 0x18, 0x00, 0x38, 0x01, 0x10, 0x00,
    0x50, 0x01, 0x18, 0x00, 0x60, 0x01, 0x00, 0x00, 0x02, 0x00, 0x87, 0x80, 0x02, 0x00, 0x01, 0xFF,
    0x03, 0x01, 0x01, 0x02, 0x02, 0x02, 0x0F, 0x00, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x02, 0x14, 0x0A, 0x02, 0x00, 0x00, 0x00, 0x01, 0x01, 0x14, 0x0A, 0x02, 0x05, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x46, 0x01, 0x02, 0x00, 0x00, 0x00, 0x01, 0x05, 0x32, 0x01,
    0x02, 0x00, 0x00, 0x00, 0x01, 0x1E, 0x01, 0x00, 0x30, 0x00, 0xF4, 0x00, 0x24, 0x01, 0x14, 0x00,
    0x78, 0x01, 0x18, 0x00, 0x90, 0x01, 0x08, 0x00, 0x98, 0x01, 0x08, 0x00, 0xA0, 0x01, 0x10, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xB0, 0x01, 0x08, 0x00, 0xB8, 0x01, 0x0C, 0x00, 0xC4, 0x01, 0x08, 0x00,
    0xCC, 0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD4, 0x01, 0x04, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x43, 0x53, 0x44, 0x50, 0x00, 0x01, 0x0D, 0x00, 0xB4, 0x04, 0x05, 0xF5,
    0x49, 0x6E, 0x66, 0x69, 0x6E, 0x65, 0x6F, 0x6E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xB4, 0x04, 0x05, 0xF5, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x0F,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x14,
    0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,
    0x01, 0x01, 0x00, 0x00, 0x2C, 0x91, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x31,
    0x0F, 0x0F, 0x0F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0xB4, 0x04, 0x00, 0x00,
    0x00, 0x01, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00,
    0x41, 0xA0, 0x00, 0xFF, 0xB4, 0x04, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0xF5,
    0x01, 0x00, 0x00, 0x47, 0x03, 0x00, 0x18, 0x00, 0x2C, 0x03, 0x00, 0x00, 0x44, 0x03, 0x00, 0x00,
    0x44, 0x03, 0x00, 0x00, 0x02, 0x00, 0x87, 0x80, 0x02, 0x00, 0x01, 0xFF, 0x03, 0x01, 0x01, 0x02,
    0x01, 0x01, 0x0F, 0x00, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x14, 0x0A,
    0x02, 0x00, 0x00, 0x00, 0x01, 0x01, 0x14, 0x0A, 0x02, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x02, 0x46, 0x01, 0x02, 0x00, 0x00, 0x00, 0x01, 0x05, 0x32, 0x01, 0x02, 0x00, 0x00, 0x00,
    0x01, 0x1E, 0x01, 0x00, 0x24, 0x02, 0xF4, 0x00, 0x18, 0x03, 0x14, 0x00, 0x44, 0x03, 0x18, 0x00,
    0x5C, 0x03, 0x08, 0x00, 0x64, 0x03, 0x08, 0x00, 0x6C, 0x03, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x7C, 0x03, 0x08, 0x00, 0x84, 0x03, 0x0C, 0x00, 0x90, 0x03, 0x08, 0x00, 0x98, 0x03, 0x08, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x43, 0x01, 0xB4, 0x04, 0x05, 0xF5, 0x01, 0x00, 0x49, 0x6E, 0x66, 0x69, 0x6E, 0x65, 0x6F, 0x6E,
    0x20, 0x54, 0x65, 0x63, 0x68, 0x6E, 0x6F, 0x6C, 0x6F, 0x67, 0x69, 0x65, 0x73, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x4D, 0x47, 0x31, 0x53, 0x33, 0x20, 0x47,
    0x72, 0x69, 0x66, 0x66, 0x69, 0x6E, 0x20, 0x43, 0x72, 0x65, 0x65, 0x6B, 0x20, 0x44, 0x6F, 0x63,
    0x6B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xF0, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFF, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x1D, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x1E, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x96, 0x00, 0x05, 0x05, 0xD0, 0x07, 0x0A, 0x8C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB4, 0x04, 0x05, 0xF5, 0x00, 0x00, 0xC0, 0x04,
    0x30, 0x05, 0x5C, 0x05, 0x72, 0x05, 0x7C, 0x05, 0xAC, 0x05, 0xD4, 0x05, 0xF0, 0x05, 0x2E, 0x06,
    0x5A, 0x06, 0x8A, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x05, 0x0F, 0x70, 0x00, 0x06, 0x07, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x14, 0x10, 0x04, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x38, 0x10, 0x0D, 0x07, 0x03, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x01, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x08,
    0x87, 0x80, 0x00, 0x09, 0x01, 0xFF, 0x00, 0x0A, 0x08, 0x10, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x08, 0x10, 0x0F, 0x01, 0x01, 0x00, 0x00, 0x00, 0x08, 0x10, 0x0F, 0x02, 0x45, 0x00, 0x1C, 0x00,
    0x2C, 0x03, 0x49, 0x00, 0x6E, 0x00, 0x66, 0x00, 0x69, 0x00, 0x6E, 0x00, 0x65, 0x00, 0x6F, 0x00,
    0x6E, 0x00, 0x20, 0x00, 0x54, 0x00, 0x65, 0x00, 0x63, 0x00, 0x68, 0x00, 0x6E, 0x00, 0x6F, 0x00,
    0x6C, 0x00, 0x6F, 0x00, 0x67, 0x00, 0x69, 0x00, 0x65, 0x00, 0x73, 0x00, 0x16, 0x03, 0x44, 0x00,
    0x4D, 0x00, 0x43, 0x00, 0x20, 0x00, 0x44, 0x00, 0x65, 0x00, 0x76, 0x00, 0x69, 0x00, 0x63, 0x00,
    0x65, 0x00, 0x0A, 0x03, 0x31, 0x00, 0x32, 0x00, 0x33, 0x00, 0x34, 0x00, 0x30, 0x03, 0x42, 0x00,
    0x69, 0x00, 0x6C, 0x00, 0x6C, 0x00, 0x62, 0x00, 0x6F, 0x00, 0x61, 0x00, 0x72, 0x00, 0x64, 0x00,
    0x20, 0x00, 0x43, 0x00, 0x6F, 0x00, 0x6E, 0x00, 0x66, 0x00, 0x69, 0x00, 0x67, 0x00, 0x75, 0x00,
    0x72, 0x00, 0x61, 0x00, 0x74, 0x00, 0x69, 0x00, 0x6F, 0x00, 0x6E, 0x00, 0x28, 0x03, 0x42, 0x00,
    0x69, 0x00, 0x6C, 0x00, 0x6C, 0x00, 0x62, 0x00, 0x6F, 0x00, 0x61, 0x00, 0x72, 0x00, 0x64, 0x00,
    0x20, 0x00, 0x49, 0x00, 0x6E, 0x00, 0x74, 0x00, 0x65, 0x00, 0x72, 0x00, 0x66, 0x00, 0x61, 0x00,
    0x63, 0x00, 0x65, 0x00, 0x1C, 0x03, 0x48, 0x00, 0x49, 0x00, 0x44, 0x00, 0x20, 0x00, 0x49, 0x00,
    0x6E, 0x00, 0x74, 0x00, 0x65, 0x00, 0x72, 0x00, 0x66, 0x00, 0x61, 0x00, 0x63, 0x00, 0x65, 0x00,
    0x3E, 0x03, 0x68, 0x00, 0x74, 0x00, 0x74, 0x00, 0x70, 0x00, 0x3A, 0x00, 0x2F, 0x00, 0x2F, 0x00,
    0x77, 0x00, 0x77, 0x00, 0x77, 0x00, 0x2E, 0x00, 0x63, 0x00, 0x79, 0x00, 0x70, 0x00, 0x72, 0x00,
    0x65, 0x00, 0x73, 0x00, 0x73, 0x00, 0x2E, 0x00, 0x63, 0x00, 0x6F, 0x00, 0x6D, 0x00, 0x2F, 0x00,
    0x54, 0x00, 0x79, 0x00, 0x70, 0x00, 0x65, 0x00, 0x2D, 0x00, 0x43, 0x00, 0x2F, 0x00, 0x2C, 0x03,
    0x54, 0x00, 0x79, 0x00, 0x70, 0x00, 0x65, 0x00, 0x2D, 0x00, 0x43, 0x00, 0x20, 0x00, 0x41, 0x00,
    0x6C, 0x00, 0x74, 0x00, 0x65, 0x00, 0x72, 0x00, 0x6E, 0x00, 0x61, 0x00, 0x74, 0x00, 0x65, 0x00,
    0x20, 0x00, 0x4D, 0x00, 0x6F, 0x00, 0x64, 0x00, 0x65, 0x00, 0x30, 0x03, 0x54, 0x00, 0x79, 0x00,
    0x70, 0x00, 0x65, 0x00, 0x2D, 0x00, 0x43, 0x00, 0x20, 0x00, 0x41, 0x00, 0x6C, 0x00, 0x74, 0x00,
    0x65, 0x00, 0x72, 0x00, 0x6E, 0x00, 0x61, 0x00, 0x74, 0x00, 0x65, 0x00, 0x20, 0x00, 0x4D, 0x00,
    0x6F, 0x00, 0x64, 0x00, 0x65, 0x00, 0x20, 0x00, 0x32, 0x00, 0x30, 0x03, 0x54, 0x00, 0x79, 0x00,
    0x70, 0x00, 0x65, 0x00, 0x20, 0x00, 0x43, 0x00, 0x20, 0x00, 0x41, 0x00, 0x6C, 0x00, 0x74, 0x00,
    0x65, 0x00, 0x72, 0x00, 0x6E, 0x00, 0x61, 0x00, 0x74, 0x00, 0x65, 0x00, 0x20, 0x00, 0x4D, 0x00,
    0x6F, 0x00, 0x64, 0x00, 0x65, 0x00, 0x20, 0x00, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const dock_config_t * get_config(void)
{
    return (dock_config_t *)&gl_config_table;
}

const cdtt_config_t * get_cdtt_config(void)
{
    return ((cdtt_config_t *)((uint32_t)&gl_config_table + get_config()->cdtt_offset));
}

const smart_power_config_t* get_smart_power_config(void)
{
    return ((smart_power_config_t *)((uint32_t)&gl_config_table + get_config()->smart_power_offset));
}

const sec_config_t * get_sec_config(void)
{
    return ((sec_config_t *)((uint32_t)&gl_config_table + get_config()->sec_config_offset));
}

/* End of File */