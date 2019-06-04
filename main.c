/**
 * This software is subject to the ANT+ Shared Source License
 * www.thisisant.com/swlicenses
 * Copyright (c) Garmin Canada Inc. 2012
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 *
 *    1) Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *
 *    2) Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *
 *    3) Neither the name of Garmin nor the names of its
 *       contributors may be used to endorse or promote products
 *       derived from this software without specific prior
 *       written permission.
 *
 * The following actions are prohibited:
 *
 *    1) Redistribution of source code containing the ANT+ Network
 *       Key. The ANT+ Network Key is available to ANT+ Adopters.
 *       Please refer to http://thisisant.com to become an ANT+
 *       Adopter and access the key. 
 *
 *    2) Reverse engineering, decompilation, and/or disassembly of
 *       software provided in binary form under this license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE HEREBY
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; DAMAGE TO ANY DEVICE, LOSS OF USE, DATA, OR 
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED 
 * OF THE POSSIBILITY OF SUCH DAMAGE. SOME STATES DO NOT ALLOW 
 * THE EXCLUSION OF INCIDENTAL OR CONSEQUENTIAL DAMAGES, SO THE
 * ABOVE LIMITATIONS MAY NOT APPLY TO YOU.
 *
 */
/**@file
 * @defgroup ant_bpwr_sensor_main ANT Bicycle Power sensor example
 * @{
 * @ingroup nrf_ant_bicycle_power
 *
 * @brief Example of ANT Bicycle Power profile display.
 *
 * Before compiling this example for NRF52, complete the following steps:
 * - Download the S212 SoftDevice from <a href="https://www.thisisant.com/developer/components/nrf52832" target="_blank">thisisant.com</a>.
 * - Extract the downloaded zip file and copy the S212 SoftDevice headers to <tt>\<InstallFolder\>/components/softdevice/s212/headers</tt>.
 * If you are using Keil packs, copy the files into a @c headers folder in your example folder.
 * - Make sure that @ref ANT_LICENSE_KEY in @c nrf_sdm.h is uncommented.
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "boards.h"
#include "bsp.h"
#include "hardfault.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_drv_clock.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ant.h"
#include "ant_key_manager.h"
#include "ant_bpwr.h"
#include "ant_bpwr_simulator.h"
#include "ant_bsc.h"
#include "ant_bsc_simulator.h"
#include "ant_state_indicator.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define MODIFICATION_TYPE_BUTTON 0 /* predefined value, MUST REMAIN UNCHANGED */
#define MODIFICATION_TYPE_AUTO   1 /* predefined value, MUST REMAIN UNCHANGED */

#if (MODIFICATION_TYPE != MODIFICATION_TYPE_BUTTON) \
    && (MODIFICATION_TYPE != MODIFICATION_TYPE_AUTO)
    #error Unsupported value of MODIFICATION_TYPE.
#endif

#define HALL_EFFECT_PIN_IN NRF_GPIO_PIN_MAP(0, 23)
#define HALL_EFFECT_BTN 1 // BUTTON_2 is position 1 in BUTTONS_LIST in board header file.

#if NRF_PWR_MGMT_CONFIG_USE_SCHEDULER
#include "app_scheduler.h"
#define APP_SCHED_MAX_EVENT_SIZE    0   /**< Maximum size of scheduler events. */
#define APP_SCHED_QUEUE_SIZE        4   /**< Maximum number of events in the scheduler queue. */
#endif // NRF_PWR_MGMT_CONFIG_USE_SCHEDULER

#define SAMPLES_IN_BUFFER 1
volatile uint8_t state = 1;

uint8_t GEAR = 1;
uint8_t CADENCE = 0;
float SPEED = 0.0;
uint16_t POWER = 0;
static uint32_t CADENCE_NUMERATOR = APP_TIMER_TICKS(120000);

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(1);
APP_TIMER_DEF(m_idle_timer_id); //rpm cadence timer
static uint8_t               m_idle_timer_ctr = 0;
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter = 0;
static uint32_t              m_pin_hldr_counter = 0;
static uint32_t              m_pin_hldr_counter_last = 0;

static uint32_t              m_cadence_counter[] = {0, 0, 0};
static int                   m_cadence_position = 0;

/** @snippet [ANT BPWR TX Instance] */
void ant_bpwr_evt_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_evt_t event);
void ant_bpwr_calib_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_page1_data_t * p_page1);

BPWR_SENS_CHANNEL_CONFIG_DEF(m_ant_bpwr,
                             0,
                             5,
                             4,
                             ANTPLUS_NETWORK_NUM);
BPWR_SENS_PROFILE_CONFIG_DEF(m_ant_bpwr,
                            (ant_bpwr_torque_t)(0), // 0 = power meter only
                            ant_bpwr_calib_handler,
                            ant_bpwr_evt_handler);

static ant_bpwr_profile_t m_ant_bpwr;

/** @snippet [ANT BSC TX Instance] */
void ant_bsc_evt_handler(ant_bsc_profile_t * p_profile, ant_bsc_evt_t event);

BSC_SENS_CHANNEL_CONFIG_DEF(m_ant_bsc,
                            1,
                            1,
                            123, //123 = speed sensor only
                            5,
                            ANTPLUS_NETWORK_NUM);
BSC_SENS_PROFILE_CONFIG_DEF(m_ant_bsc,
                            true,
                            true,
                            ANT_BSC_PAGE_5,
                            ant_bsc_evt_handler);

static ant_bsc_profile_t m_ant_bsc;

NRF_SDH_ANT_OBSERVER(m_ant_bpwr_observer, ANT_BPWR_ANT_OBSERVER_PRIO,
                     ant_bpwr_sens_evt_handler, &m_ant_bpwr);
NRF_SDH_ANT_OBSERVER(m_ant_bsc_observer, ANT_BSC_ANT_OBSERVER_PRIO,
                     ant_bsc_sens_evt_handler, &m_ant_bsc);

/** @snippet [ANT BPWR TX Instance] */

static ant_bpwr_simulator_t  m_ant_bpwr_simulator;
static ant_bsc_simulator_t  m_ant_bsc_simulator;    /**< Simulator used to simulate profile data. */

static volatile bool m_stay_in_sysoff;  /**< True if the application should stay in system OFF mode. */
static volatile bool m_is_ready;        /**< True if the application is ready to enter sleep/system OFF mode. */
static volatile bool m_sysoff_started;  /**< True if the application started sleep preparation. */

// gear selector given noisy adc only sets gear if clear indication
uint8_t gear_selecta(uint16_t adc_value)
{
    if (adc_value <= 55) {
        return (1);
    } else if (adc_value >= 59 && adc_value <= 88) {
        return (2);
    } else if (adc_value >= 96 && adc_value <= 135) {
        return (3);
    } else if (adc_value >= 130 && adc_value <= 140) {
        return (4);
    } else if (adc_value >= 145 && adc_value <= 151) {
        return (5);
    } else if (adc_value >= 153 && adc_value <= 163) {
        return (6);
    } else if (adc_value >= 166 && adc_value <= 175) {
        return (7);
    } else if (adc_value >= 181 && adc_value <= 189) {
        return (8);
    } else if (adc_value >= 191 && adc_value <= 202) {
        return (9);
    } else if (adc_value >= 205 && adc_value <= 212) {
        return (10);
    } else if (adc_value >= 218 && adc_value <= 229) {
        return (11);
    } else if (adc_value >= 232 && adc_value <= 241) {
        return (12);
    } else if (adc_value >= 243 && adc_value <= 249) {
        return (13);
    } else if (adc_value >= 263 && adc_value <= 269) {
        return (14);
    } else if (adc_value >= 273 && adc_value <= 292) {
        return (15);
    } else if (adc_value >= 298 && adc_value <= 306) {
        return (16);
    } else if (adc_value >= 325 && adc_value <= 334) {
        return (17);
    } else if (adc_value >= 338 && adc_value <= 354) {
        return (18);
    } else if (adc_value >= 372 && adc_value <= 380) {
        return (19);
    } else if (adc_value >= 380 && adc_value <= 407) {
        return (20);
    } else if (adc_value >= 414 && adc_value <= 430) {
        return (21);
    } else if (adc_value >= 436 && adc_value <= 453) {
        return (22);
    } else if (adc_value >= 473 && adc_value <= 505) {
        return (23);
    } else if (adc_value >= 510 && adc_value <= 550) {
        return (24);
    } else {
        return (0);
    }
}

// mapping function given gear and candence set speed and power
void gear_cad_to_pwr_spd(void) 
{
    // GEAR, CADENCE, POWER and SPEED are all globals
    if (CADENCE && GEAR) {
        switch (GEAR) 
        {
            case 1:
                POWER = ((13/50.0) * (float)CADENCE) - 6;
                break;
            case 2:
                POWER = ((17/50.0) * (float)CADENCE) - 6;
                break;
            case 3:
                POWER = ((1/2.0) * (float)CADENCE) - 10;
                break;
            case 4:
                POWER = ((39/50.0) * (float)CADENCE) - 14;
                break;
            case 5:
                POWER = CADENCE - 19;
                break;
            case 6:
                POWER = ((57/50.0) * (float)CADENCE) - 21;
                break;
            case 7:
                POWER = (1.24 * (float)CADENCE) - 23;
                break;
            case 8:
                POWER = ((37/25.0) * (float)CADENCE) - 27;
                break;
            case 9:
                POWER = ((91/50.0) * (float)CADENCE) - 33;
                break;
            case 10:
                POWER = ((53/25.0) * (float)CADENCE) - 38;
                break;
            case 11:
                POWER = ((123/50.0) * (float)CADENCE) - 44;
                break;
            case 12:
                POWER = ((74/25.0) * (float)CADENCE) - 53;
                break;
            case 13:
                POWER = ((16/5.0) * (float)CADENCE) - 57;
                break;
            case 14:
                POWER = ((91/25.0) * (float)CADENCE) - 65;
                break;
            case 15:
                POWER = ((217/50.0) * (float)CADENCE) - 78;
                break;
            case 16:
                POWER = ((247/50.0) * (float)CADENCE) - 89;
                break;
            case 17:
                POWER = ((133/25.0) * (float)CADENCE) - 95;
                break;
            case 18:
                POWER = ((143/25.0) * (float)CADENCE) - 103;
                break;
            case 19:
                POWER = ((151/25.0) * (float)CADENCE) - 110;
                break;
            case 20:
                POWER = ((333/53.0) * (float)CADENCE) - 111;
                break;
            case 21:
                POWER = ((329/49.0) * (float)CADENCE) - 121;
                break;
            case 22:
                POWER = ((367/51.0) * (float)CADENCE) - 129;
                break;
            case 23:
                POWER = ((93/11.0) * (float)CADENCE) - 152;
                break;
            case 24:
                POWER = ((417/33.0) * (float)CADENCE) - 229;
                break;
            default:
                break;
        }
        SPEED = 0.277778 * (pow((3980.0 * (float)POWER),(1.0/3.54)) - 12.2); // convert km/h to m/s
        if (SPEED < 0.0) { SPEED = 0.0; }
    } else {
        POWER = 0; SPEED = 0.0;
    }
    //return;
}

/**@brief Handler for shutdown preparation.
 */
bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    uint32_t err_code;

    if (m_is_ready == false)
    {
        m_sysoff_started = true;
        return false;
    }

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_SYSOFF:
            NRF_LOG_INFO("NRF_PWR_MGMT_EVT_PREPARE_SYSOFF");
            err_code = bsp_buttons_disable();
            APP_ERROR_CHECK(err_code);
            break;

        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            NRF_LOG_INFO("NRF_PWR_MGMT_EVT_PREPARE_WAKEUP");
            err_code = bsp_buttons_disable();
            // Suppress NRF_ERROR_NOT_SUPPORTED return code.
            UNUSED_VARIABLE(err_code);

            err_code = bsp_wakeup_button_enable(HALL_EFFECT_BTN);
            // Suppress NRF_ERROR_NOT_SUPPORTED return code.
            UNUSED_VARIABLE(err_code);
            break;

        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_ERROR("Entering DFU is not supported by this example.");
            APP_ERROR_HANDLER(NRF_ERROR_API_NOT_IMPLEMENTED);
            break;

        case NRF_PWR_MGMT_EVT_PREPARE_RESET:
            NRF_LOG_INFO("NRF_PWR_MGMT_EVT_PREPARE_RESET");
            break;
    }
    err_code = app_timer_stop_all();
    APP_ERROR_CHECK(err_code);

    return true;
}

/**@brief Register application shutdown handler with priority 0. */
NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, 0);

/**@brief Function for handling bsp events.
 */
/** @snippet [ANT BPWR simulator button] */
void bsp_evt_handler(bsp_event_t event)
{
    uint32_t ctime = 0;
    
    nrf_pwr_mgmt_feed();

    switch (event)
    {
        case BSP_EVENT_KEY_1:
            m_pin_hldr_counter++;
            ctime = app_timer_cnt_get();
            m_cadence_counter[m_cadence_position] = ctime;
            m_cadence_position = (m_cadence_position + 1) % 3;
            if (m_pin_hldr_counter > 1) {
                CADENCE = CADENCE_NUMERATOR / \
                    app_timer_cnt_diff_compute(ctime, m_cadence_counter[m_cadence_position]);
            }

            //ant_bpwr_simulator_increment(&m_ant_bpwr_simulator);
            m_ant_bpwr_simulator._cb.power_sensorsim_state.current_val = (uint32_t)POWER;
            m_ant_bpwr_simulator._cb.cadence_sensorsim_state.current_val = (uint32_t)CADENCE;

            //ant_bsc_simulator_increment(&m_ant_bsc_simulator);
            m_ant_bsc_simulator._cb.sensorsim_s_state.current_val = (uint32_t)SPEED;

            ret_code_t err_code = app_timer_stop(m_idle_timer_id);
            APP_ERROR_CHECK(err_code);

            err_code = app_timer_start(m_idle_timer_id, APP_TIMER_TICKS(1000), NULL);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}
/** @snippet [ANT BPWR simulator button] */

/**@brief Function for handling ANT BPWR events.
 */
/** @snippet [ANT BPWR simulator call] */
void ant_bpwr_evt_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_evt_t event)
{
    switch (event)
    {
        case ANT_BPWR_PAGE_1_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_16_UPDATED:
            m_ant_bpwr_simulator.p_profile->BPWR_PROFILE_instantaneous_power =
                m_ant_bpwr_simulator._cb.power_sensorsim_state.current_val;
            m_ant_bpwr_simulator.p_profile->BPWR_PROFILE_accumulated_power +=
                m_ant_bpwr_simulator._cb.power_sensorsim_state.current_val;

            if (m_ant_bpwr_simulator.p_profile->BPWR_PROFILE_accumulated_power == UINT16_MAX)
            {
                m_ant_bpwr_simulator.p_profile->BPWR_PROFILE_accumulated_power = 0;
            }
            m_ant_bpwr_simulator.p_profile->BPWR_PROFILE_instantaneous_cadence =
                 m_ant_bpwr_simulator._cb.cadence_sensorsim_state.current_val;
            m_ant_bpwr_simulator.p_profile->BPWR_PROFILE_pedal_power.distribution =
                m_ant_bpwr_simulator._cb.pedal_sensorsim_state.current_val;
            m_ant_bpwr_simulator.p_profile->BPWR_PROFILE_power_update_event_count++;
            break;
        case ANT_BPWR_PAGE_17_UPDATED:
            break; // not implemented
        case ANT_BPWR_PAGE_18_UPDATED:
            break; // not implemented
        case ANT_BPWR_PAGE_80_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_81_UPDATED:
            ant_bpwr_simulator_one_iteration(&m_ant_bpwr_simulator, event);
            break;

        default:
            break;
    }
}
/** @snippet [ANT BPWR simulator call] */

/**@brief Function for handling ANT BSC events.
 */
/** @snippet [ANT BSC simulator call] */
void ant_bsc_evt_handler(ant_bsc_profile_t * p_profile, ant_bsc_evt_t event)
{
    //nrf_pwr_mgmt_feed();

    switch (event)
    {
        case ANT_BSC_PAGE_0_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_1_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_2_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_3_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_4_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_5_UPDATED:
            /* fall through */
        case ANT_BSC_COMB_PAGE_0_UPDATED:
            ant_bsc_simulator_one_iteration(&m_ant_bsc_simulator);
            break;

        default:
            break;
    }
}

/**@brief Function for handling ANT BPWR events.
 */
/** @snippet [ANT BPWR calibration] */
void ant_bpwr_calib_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_page1_data_t * p_page1)
{
    switch (p_page1->calibration_id)
    {
        case ANT_BPWR_CALIB_ID_MANUAL:
            m_ant_bpwr.BPWR_PROFILE_calibration_id     = ANT_BPWR_CALIB_ID_MANUAL_SUCCESS;
            m_ant_bpwr.BPWR_PROFILE_general_calib_data = CALIBRATION_DATA;
            break;

        case ANT_BPWR_CALIB_ID_AUTO:
            m_ant_bpwr.BPWR_PROFILE_calibration_id     = ANT_BPWR_CALIB_ID_MANUAL_SUCCESS;
            m_ant_bpwr.BPWR_PROFILE_auto_zero_status   = p_page1->auto_zero_status;
            m_ant_bpwr.BPWR_PROFILE_general_calib_data = CALIBRATION_DATA;
            break;

        case ANT_BPWR_CALIB_ID_CUSTOM_REQ:
            m_ant_bpwr.BPWR_PROFILE_calibration_id = ANT_BPWR_CALIB_ID_CUSTOM_REQ_SUCCESS;
            memcpy(m_ant_bpwr.BPWR_PROFILE_custom_calib_data, p_page1->data.custom_calib,
                   sizeof (m_ant_bpwr.BPWR_PROFILE_custom_calib_data));
            break;

        case ANT_BPWR_CALIB_ID_CUSTOM_UPDATE:
            m_ant_bpwr.BPWR_PROFILE_calibration_id = ANT_BPWR_CALIB_ID_CUSTOM_UPDATE_SUCCESS;
            memcpy(m_ant_bpwr.BPWR_PROFILE_custom_calib_data, p_page1->data.custom_calib,
                   sizeof (m_ant_bpwr.BPWR_PROFILE_custom_calib_data));
            break;

        default:
            break;
    }
}
/** @snippet [ANT BPWR calibration] */

static void idle_timer_handler(void * p_context)
{
    uint32_t ctime = 0;
    int8_t p_pos = 0;

    if (m_pin_hldr_counter_last == m_pin_hldr_counter) {
        if (m_idle_timer_ctr == 4) {
            m_pin_hldr_counter = 0;
            m_pin_hldr_counter_last = 0;
            memset(m_cadence_counter, 0, sizeof(m_cadence_counter));
            m_cadence_position = 0;
            CADENCE = 0;
            m_idle_timer_ctr = 0;
        } else {
            m_idle_timer_ctr++;
            // do decrement cadence to compensate
            ctime = app_timer_cnt_get();
            p_pos = (m_cadence_position + 5) % 3;
            if (m_cadence_counter[p_pos]) {
                m_cadence_counter[p_pos] = ctime;
                CADENCE = CADENCE_NUMERATOR / \
                    app_timer_cnt_diff_compute(ctime, m_cadence_counter[m_cadence_position]);
            }
        }
    } else {
        m_pin_hldr_counter_last = m_pin_hldr_counter;
    }
    ret_code_t err_code = app_timer_stop(m_idle_timer_id);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_idle_timer_id, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for setup all thinks not directly associated with ANT stack/protocol.
 * @desc Initialization of: @n
 *         - app_timer, pre-setup for bsp.
 *         - bsp for signaling LEDs and user buttons.
 */
static void utils_setup(void)
{
    // Initialize and start a single continuous mode timer, which is used to update the event time
    // on the main data page.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    //err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_evt_handler);
    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_event_to_button_action_assign(HALL_EFFECT_BTN,
                                                 BSP_BUTTON_ACTION_PUSH,
                                                 BSP_EVENT_KEY_1);
    APP_ERROR_CHECK(err_code);

    err_code = ant_state_indicator_init(m_ant_bpwr.channel_number, BPWR_SENS_CHANNEL_TYPE);
    APP_ERROR_CHECK(err_code);

    err_code = ant_state_indicator_init(m_ant_bsc.channel_number, BSC_SENS_CHANNEL_TYPE);
    APP_ERROR_CHECK(err_code);

    // Create timers
    err_code = app_timer_create(&m_idle_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                idle_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_idle_timer_id, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);
}

/**
 *@brief Function for initializing logging.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for the BPWR simulator initialization.
 */
void simulator_setup(void)
{
    /** @snippet [ANT BPWR simulator init] */
    const ant_bpwr_simulator_cfg_t bpwr_simulator_cfg =
    {
        .p_profile   = &m_ant_bpwr,
        .sensor_type = (ant_bpwr_torque_t)(0), // 0 = power meter only
    };
    /** @snippet [ANT BPWR simulator init] */
    bpwr_simulator_cfg.p_profile->page_16.pedal_power.byte = 0xFF; // not using pedal power distribution

    ant_bpwr_simulator_init(&m_ant_bpwr_simulator, &bpwr_simulator_cfg, false);

       /** @snippet [ANT BSC simulator init] */
    
    const ant_bsc_simulator_cfg_t bsc_simulator_cfg =
    {
        .p_profile      = &m_ant_bsc,
        .device_type    = 123, //123 = speed sensor only
    };
    /** @snippet [ANT BSC simulator init] */

    ant_bsc_simulator_init(&m_ant_bsc_simulator, &bsc_simulator_cfg, false);

    /** @snippet [ANT BSC simulator button init] */
}

/**
 * @brief Function for ANT stack initialization.
 *
 * @details Initializes the SoftDevice and the ANT event interrupt.
 */
static void softdevice_setup(void)
{    
    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    ASSERT(nrf_sdh_is_enabled());

    err_code = nrf_sdh_ant_enable();
    APP_ERROR_CHECK(err_code);

    err_code = ant_plus_key_set(ANTPLUS_NETWORK_NUM);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for Bicycle Power profile initialization.
 *
 * @details Initializes the Bicycle Power profile and open ANT channel.
 */
static void profile_setup(void)
{
/** @snippet [ANT BPWR TX Profile Setup] */
    ret_code_t err_code;

    err_code = ant_bpwr_sens_init(&m_ant_bpwr,
                                  BPWR_SENS_CHANNEL_CONFIG(m_ant_bpwr),
                                  BPWR_SENS_PROFILE_CONFIG(m_ant_bpwr));
    APP_ERROR_CHECK(err_code);

    // fill manufacturer's common data page.
    m_ant_bpwr.page_80 = ANT_COMMON_page80(BPWR_HW_REVISION,
                                           BPWR_MANUFACTURER_ID,
                                           BPWR_MODEL_NUMBER);
    // fill product's common data page.
    m_ant_bpwr.page_81 = ANT_COMMON_page81(BPWR_SW_REVISION_MAJOR,
                                           BPWR_SW_REVISION_MINOR,
                                           BPWR_SERIAL_NUMBER);

    m_ant_bpwr.BPWR_PROFILE_auto_zero_status = ANT_BPWR_AUTO_ZERO_OFF;

    err_code = ant_bpwr_sens_open(&m_ant_bpwr);
    APP_ERROR_CHECK(err_code);

/** @snippet [ANT BPWR TX Profile Setup] */

/** @snippet [ANT BSC TX Profile Setup] */

    err_code = ant_bsc_sens_init(&m_ant_bsc,
                                 BSC_SENS_CHANNEL_CONFIG(m_ant_bsc),
                                 BSC_SENS_PROFILE_CONFIG(m_ant_bsc));
    APP_ERROR_CHECK(err_code);

    m_ant_bsc.BSC_PROFILE_manuf_id     = BSC_MF_ID;
    m_ant_bsc.BSC_PROFILE_serial_num   = BSC_SERIAL_NUMBER;
    m_ant_bsc.BSC_PROFILE_hw_version   = BSC_HW_VERSION;
    m_ant_bsc.BSC_PROFILE_sw_version   = BSC_SW_VERSION;
    m_ant_bsc.BSC_PROFILE_model_num    = BSC_MODEL_NUMBER;

    err_code = ant_bsc_sens_open(&m_ant_bsc);
    APP_ERROR_CHECK(err_code);

    err_code = ant_state_indicator_channel_opened();
    APP_ERROR_CHECK(err_code);
/** @snippet [ANT BSC TX Profile Setup] */

}

void timer_handler(nrf_timer_event_t event_type, void * p_context)
{

}

void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 250ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 250);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}

void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
        int         avg = 0;
        int         tmp = 0;
        int         i;

        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            tmp = (uint16_t)p_event->data.done.p_buffer[i] >> 2; // get 10 bit resolution from 12 bit saadc
            avg += tmp;
        }

        tmp = (int)avg/SAMPLES_IN_BUFFER - 350; // pot minimal offset value
        if (tmp > 562) { tmp = 563; } // maximum setting
        else if (tmp < 0) { tmp = 0; }

        //tmp_gear = gear_selecta(tmp);
        //if (tmp_gear) { GEAR = tmp_gear; }

        GEAR = (uint8_t)((tmp / 24.0) + 1); // linear mapping

        gear_cad_to_pwr_spd();
#if DEBUG
        NRF_LOG_INFO("ADC: %d, GEAR: %d, CADENCE: %d", tmp, GEAR, CADENCE);
        NRF_LOG_INFO("POWER: %d, SPEED: " NRF_LOG_FLOAT_MARKER "", \
            POWER, NRF_LOG_FLOAT(SPEED));
#endif
        m_adc_evt_counter++;
    }
}

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN7);
    channel_config.gain = NRF_SAADC_GAIN1_5;
    channel_config.acq_time = NRF_SAADC_ACQTIME_5US;
    channel_config.burst = NRF_SAADC_BURST_ENABLED;

    nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    saadc_config.resolution = 2; // 0 8Bit, 1 10Bit, 2 12Bit
    saadc_config.oversample = 3; // 0 Disable, 1 2x, 2 4x, 3 8x, 4 16x

    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for application main entry, does not return.
 */
int main(void)
{
    log_init();
    utils_setup();
    softdevice_setup();
    simulator_setup();
    profile_setup();
    saadc_init();
    saadc_sampling_event_init();
    saadc_sampling_event_enable();

    NRF_LOG_INFO("ANT+ Bicycle Power and Speed started.");
    m_is_ready = true; //system can now shutdown

    for (;;)
    {
        NRF_LOG_FLUSH();
        nrf_pwr_mgmt_run();
    }
}
