/******************************************************************************
 * File Name: solution.c
 *
 * Description: Galactico Greek Dock solution source file.
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

#include "cybsp.h"
#include "config.h"
#include "cy_pdutils_sw_timer.h"
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_gpio.h"
#include "cy_app.h"
#include "solution.h"
#include "cy_app_led_ctrl.h"
#include "cy_app_i2c_master.h"

#if BUTTON_PRESS_FACTORY_RESET
#include "cy_app_spi_comp_update.h"
#include "cy_app_dmc_metadata.h"
#endif /* BUTTON_PRESS_FACTORY_RESET */

#if CY_APP_SMART_POWER_ENABLE
#include "cy_app_smart_power.h"
#include "dmc_solution.h"
#endif /* CY_APP_SMART_POWER_ENABLE */

extern cy_stc_pdstack_context_t *get_pdstack_context(uint8_t portIdx);

#if CY_APP_DMC_ENABLE
extern cy_stc_dmc_params_t gl_dmcParams;
#endif /* CY_APP_DMC_ENABLE */

#if CY_APP_SMART_POWER_ENABLE
extern cy_stc_app_smart_power_context_t gl_SmartPowerCtx;
extern cy_stc_scb_i2c_context_t  gl_i2c_pm_context;
#endif /* CY_APP_SMART_POWER_ENABLE */

#if CY_APP_LED_CONTROL_ENABLE
extern cy_stc_led_ctrl_context_t gl_LedCtrlCtx;
#endif /* CY_APP_LED_CONTROL_ENABLE */

#if (BUTTON_PRESS_FACTORY_RESET == 1u)
void factory_reset_timer_cb (cy_timer_id_t id, void *callbackContext)
{
    cy_stc_app_dmc_dock_metadata_t* dock_md = Cy_App_Dmc_GetDockMetadata();

    if (Cy_GPIO_Read(POWER_BTN_PORT, POWER_BTN_PIN) == false)
    {
        if (dock_md->app_status.misc_status.trigger_phase2 == CY_APP_DMC_NO_TRIGGER)
        {
            dock_md->app_status.misc_status.trigger_phase2 = CY_APP_DMC_NEW_TRIGGER;
            /* Set primary package status as invalid */
            Cy_App_Dmc_UpdateRamImageStatus (Cy_App_Dmc_SpiGetCompId (), CY_APP_DMC_CANDIDATE_PACKAGE_PRIMARY, CY_APP_DMC_IMG_STATUS_INVALID);

            Cy_App_Dmc_WriteMetadata(false, &gl_dmcParams);

            Cy_App_Dmc_PrepareSoftReset(&gl_dmcParams);
            Cy_App_Dmc_SoftReset();
        }
    }
}
#endif /* (BUTTON_PRESS_FACTORY_RESET == 1u) */

#if EXTENDED_ALERT_EVENTS_SUPPORT
void send_extended_alerts(cy_stc_pdstack_context_t * ptrPdStackContext, cy_en_pd_extd_alert_type_t alert_type)
{
    if ((ptrPdStackContext->dpmConfig.attach) && (ptrPdStackContext->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP))
    {
        /* Set alert message */
        cy_pd_pd_do_t alert;
        alert.val = 0;
        /* set Extended Alert Event in Type_of_Alert */
        alert.ado_alert.extdAlertEvtType = true;
        alert.ado_alert.extdAlertEvtType = alert_type;
        ptrPdStackContext->dpmStat.alert = alert;
    }
}
#endif /* EXTENDED_ALERT_EVENTS_SUPPORT */

#if EXTENDED_ALERT_EVENTS_SUPPORT || BUTTON_PRESS_FACTORY_RESET
void power_button_led_cb (cy_timer_id_t id, void *callbackContext)
{
    /* Get the PD-Stack context from the USBPD context */
    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) callbackContext;

#if EXTENDED_ALERT_EVENTS_SUPPORT
    cy_stc_app_status_t* appStatus = Cy_App_GetStatus(ptrPdStackContext->port);
#endif /* EXTENDED_ALERT_EVENTS_SUPPORT */

    /* Used addition protection not to send the same (Press/Release) Alert twice. */
    static bool button_pressed = false;

    if (Cy_GPIO_Read(POWER_BTN_PORT, POWER_BTN_PIN) == false)
    {
        if (button_pressed == false)
        {
            button_pressed = true;
#if (CY_APP_LED_CONTROL_ENABLE && CY_APP_PD_ENABLE)
            ptrPdStackContext->dpmExtStat.pwrLed = CY_APP_LED_CTRL_LED_ON;
            Cy_App_LedCtrl_SwitchMode((cy_en_led_ctrl_mode_t)ptrPdStackContext->dpmExtStat.pwrLed, &gl_LedCtrlCtx);
#endif /* (CY_APP_LED_CONTROL_ENABLE && CY_APP_PD_ENABLE) */
#if EXTENDED_ALERT_EVENTS_SUPPORT
            if(appStatus->pd_revision >= CY_APP_MIN_PD_SPEC_VERSION_FOR_EXTD_ALERT_SUPPORT)
            {
                send_extended_alerts(ptrPdStackContext, CY_PD_EXTD_ALERT_TYPE_PWR_BTN_PRESS);
            }
            else
            {
                /* Hook for sending extended alerts using alternate modes */
            }
#endif /* EXTENDED_ALERT_EVENTS_SUPPORT */
        }
    }
    else
    {
        if (button_pressed == true)
        {
            button_pressed = false;
#if (CY_APP_LED_CONTROL_ENABLE && CY_APP_PD_ENABLE)
            ptrPdStackContext->dpmExtStat.pwrLed = CY_APP_LED_CTRL_LED_OFF;
            Cy_App_LedCtrl_SwitchMode((cy_en_led_ctrl_mode_t)ptrPdStackContext->dpmExtStat.pwrLed, &gl_LedCtrlCtx);
#endif /* (CY_APP_LED_CONTROL_ENABLE && CY_APP_PD_ENABLE) */
#if EXTENDED_ALERT_EVENTS_SUPPORT
            if(appStatus->pd_revision >= CY_APP_MIN_PD_SPEC_VERSION_FOR_EXTD_ALERT_SUPPORT)
            {
                send_extended_alerts(ptrPdStackContext, CY_PD_EXTD_ALERT_TYPE_PWR_BTN_RELEASE);
            }
            else
            {
                /* Hook for sending extended alerts using alternate modes */
            }
#endif /* EXTENDED_ALERT_EVENTS_SUPPORT */
        }
    }

#if (BUTTON_PRESS_FACTORY_RESET == 1u)
    if (Cy_GPIO_Read(POWER_BTN_PORT, POWER_BTN_PIN) == false)
    {
        /* Start timer for 20 seconds */
        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                APP_BUTTON_FACTORY_RESET_TIMER, APP_BUTTON_FACTORY_RESET_TIMER_PERIOD, factory_reset_timer_cb);
    }
    else
    {
        Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->ptrTimerContext, APP_BUTTON_FACTORY_RESET_TIMER);
    }
#endif /* (BUTTON_PRESS_FACTORY_RESET == 1u) */
}

#if EXTENDED_ALERT_EVENTS_SUPPORT
void lan_button_led_cb (cy_timer_id_t id, void *callbackContext)
{
    /* Get the PD-Stack context from the USBPD context */
    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) callbackContext;

    cy_stc_app_status_t* appStatus = Cy_App_GetStatus(ptrPdStackContext->port);

    if(Cy_GPIO_Read(LAN_WAKE_BTN_PORT, LAN_WAKE_BTN_PIN) == false)
    {
        if(appStatus->pd_revision >= CY_APP_MIN_PD_SPEC_VERSION_FOR_EXTD_ALERT_SUPPORT)
        {
            send_extended_alerts(ptrPdStackContext, CY_PD_EXTD_ALERT_TYPE_CTRLR_WAKE);
        }
        else
        {
            /* Hook for sending extended alerts using alternate modes */
        }
    }
}
#endif /* EXTENDED_ALERT_EVENTS_SUPPORT */

void power_button_intr_cb (void)
{
    /* Get the PD-Stack context from main port0 */
    cy_stc_pdstack_context_t * ptrPdStackContext = get_pdstack_context(TYPEC_PORT_0_IDX);

    /* Start a debounce button timer. */
    Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
            APP_POWER_BUTTON_TIMER_ID, APP_POWER_BUTTON_TIMER_PERIOD, power_button_led_cb);
}

#if EXTENDED_ALERT_EVENTS_SUPPORT
void lan_button_intr_cb (void)
{
    /* Get the PD-Stack context from main port0 */
    cy_stc_pdstack_context_t * ptrPdStackContext = get_pdstack_context(TYPEC_PORT_0_IDX);

    /* Start a debounce button timer. */
    Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
            APP_LAN_BUTTON_TIMER_ID, APP_LAN_BUTTON_TIMER_PERIOD, lan_button_led_cb);
}
#endif /* EXTENDED_ALERT_EVENTS_SUPPORT */

#endif /* (EXTENDED_ALERT_EVENTS_SUPPORT || BUTTON_PRESS_FACTORY_RESET) */

#if CY_APP_SMART_POWER_ENABLE
static uint16_t get_source_current(uint8_t slave_addr)
{
    bool status;
    uint8_t buffer[2];
    uint16_t Vsense;
    uint16_t source_current = 0u;
    uint8_t reg_data = SENSE_VOLTAGE_HIGH_BYTE;

    status = Cy_App_I2CMaster_RegRead(I2C_PM_HW, slave_addr, &reg_data, 1u, &buffer[0], 1u, &gl_i2c_pm_context);
    if(status != false)
    {
        reg_data = SENSE_VOLTAGE_LOW_BYTE;
        status = Cy_App_I2CMaster_RegRead(I2C_PM_HW, slave_addr, &reg_data, 1u, &buffer[1], 1u, &gl_i2c_pm_context);
        if(status != false)
        {
            Vsense = ((buffer[0] << 4u) | buffer[1] >> 4u);
            Vsense = ((Vsense & 0x800u) ? ((~(Vsense)+1) & 0xFFFu) : Vsense);
            source_current = ((FULL_SCALE_CURRENT * Vsense * 1000u)/2047u);
        }
    }
    return source_current;
}

static uint16_t get_source_voltage(uint8_t slave_addr)
{
    bool status;
    uint8_t buffer[2];
    uint16_t Vsource;
    uint16_t source_voltage = 0u;
    uint8_t reg_data = SOURCE_VOLTAGE_HIGH_BYTE;

    status = Cy_App_I2CMaster_RegRead(I2C_PM_HW, slave_addr, &reg_data, 1u, &buffer[0], 1u, &gl_i2c_pm_context);
    if(status != false)
    {
        reg_data = SOURCE_VOLTAGE_LOW_BYTE;
        status = Cy_App_I2CMaster_RegRead(I2C_PM_HW, slave_addr, &reg_data, 1u, &buffer[1], 1u, &gl_i2c_pm_context);
        if(status != false)
        {
            Vsource = ((buffer[0] << 4u) | buffer[1] >> 4u);
            source_voltage = ((FULL_SCALE_VOLTAGE * Vsource * 1000u)/4094u);
        }
    }
    return source_voltage;
}

uint16_t measure_adapter_power(cy_stc_usbpd_context_t *context)
{
    (void)context;

    return 0u;
}

uint16_t measure_total_dock_current(cy_stc_app_smart_power_context_t *context)
{
    (void)context;
    uint16_t current;

    current = get_source_current(IO_BOARD_CURRENT_SENSE_ADDR);
    current = current + get_source_current(MAIN_BOARD_CURRENT_SENSE_ADDR);
    current = current + get_source_current(AUDIO_BOARD_CURRENT_SENSE_ADDR);

    return current;
}

uint16_t measure_us_current(cy_stc_app_smart_power_context_t *context)
{
    (void)context;

    return 0u;
}

uint16_t measure_adapter_voltage(void)
{
    uint16_t voltage;
    voltage = get_source_voltage(MAIN_BOARD_CURRENT_SENSE_ADDR);

    return voltage;
}
#endif /* CY_APP_SMART_POWER_ENABLE */

void solution_init (cy_stc_pdstack_context_t* ptrPdStackContext)
{
#if CY_APP_SMART_POWER_ENABLE
    uint16_t adapter_power = 0u;
    uint8_t buffer[2];

    if (ptrPdStackContext->port == TYPEC_PORT_0_IDX)
    {
        if(get_smart_power_config()->adp_det_enable == 0u)
        {
            adapter_power = get_smart_power_config()->adp_pwr_watts;
        }
        else
        {
            adapter_power = measure_adapter_power(ptrPdStackContext->ptrUsbPdContext);
        }

        buffer[0] = (adapter_power >> 8u) & 0xFFu;
        buffer[1] = adapter_power & 0xFFu;

        CY_APP_DEBUG_LOG(0, CY_APP_DEBUG_PA_SIZE, buffer, 2, CY_APP_DEBUG_LOGLEVEL_INFO, true);

        Cy_App_SmartPower_Init(&gl_SmartPowerCtx, get_smart_power_config(),
                ptrPdStackContext, adapter_power, measure_adapter_voltage());
    }
#endif /* CY_APP_SMART_POWER_ENABLE */
}

/* Solution PD event handler */
void sln_pd_event_handler(cy_stc_pdstack_context_t* ctx, cy_en_pdstack_app_evt_t evt, const void *data)
{
#if CY_APP_DMC_ENABLE
    cy_stc_app_dmc_dock_metadata_t* dock_md = Cy_App_Dmc_GetDockMetadata();
#endif /* CY_APP_DMC_ENABLE */

    if(ctx->port == TYPEC_PORT_0_IDX)
    {
#if (CY_APP_SMART_POWER_ENABLE || CY_APP_DMC_ENABLE)
#if CY_APP_SMART_POWER_ENABLE
        if(
                (evt == APP_EVT_HARD_RESET_SENT) || (evt == APP_EVT_HARD_RESET_RCVD) ||
                (evt == APP_EVT_DISCONNECT) || (evt == APP_EVT_VBUS_PORT_DISABLE)
          )
        {
            Cy_App_SmartPower_Stop(&gl_SmartPowerCtx);
        }
        else if (evt == APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE)
        {
            Cy_App_SmartPower_Start(&gl_SmartPowerCtx);
        }
#endif /* CY_APP_SMART_POWER_ENABLE */

#if CY_APP_DMC_ENABLE
        if(evt == APP_EVT_DISCONNECT)
        {
            if (dock_md->app_status.misc_status.trigger_phase2 == CY_APP_DMC_NEW_TRIGGER)
            {
                Cy_App_Dmc_PrepareSoftReset(&gl_dmcParams);
                Cy_App_Dmc_SoftReset();
            }
        }
#endif /* CY_APP_DMC_ENABLE */
#endif /* (CY_APP_SMART_POWER_ENABLE || CY_APP_DMC_ENABLE) */

        if(evt == APP_EVT_HANDLE_EXTENDED_MSG)
        {
            if(((cy_stc_pd_packet_extd_t *)data)->msg == CY_PDSTACK_EXTD_MSG_STATUS)
            {
#if CY_APP_LED_CONTROL_ENABLE
                Cy_App_LedCtrl_SwitchMode((cy_en_led_ctrl_mode_t)ctx->dpmExtStat.pwrLed, &gl_LedCtrlCtx);
#endif /* CY_APP_LED_CONTROL_ENABLE */
            }
        }
    }
}

void soln_vbus_fet_on (cy_stc_pdstack_context_t *context)
{
    if(context->port == TYPEC_PORT_0_IDX)
    {
        APP_VBUS_SRC_FET_ON_P1();
    }
#if PMG1_PD_DUALPORT_ENABLE
    if(context->port == TYPEC_PORT_1_IDX)
    {
        APP_VBUS_SRC_FET_ON_P2();
    }
#endif /* PMG1_PD_DUALPORT_ENABLE */
}

void soln_vbus_fet_off (cy_stc_pdstack_context_t *context)
{
    if(context->port == TYPEC_PORT_0_IDX)
    {
        APP_VBUS_SRC_FET_OFF_P1();
    }
#if PMG1_PD_DUALPORT_ENABLE
    if(context->port == TYPEC_PORT_1_IDX)
    {
        APP_VBUS_SRC_FET_OFF_P2();
    }
#endif /* PMG1_PD_DUALPORT_ENABLE */
}

bool soln_sleep ()
{
#if CY_APP_DMC_ENABLE
    if(Cy_App_Dmc_IsIdle() != true)
    {
        return false;
    }
#endif /* CY_APP_DMC_ENABLE */
    return true;
}

void soln_resume ()
{
}

