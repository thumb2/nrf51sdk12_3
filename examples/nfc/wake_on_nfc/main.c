/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** @file
 *
 * @defgroup wake_on_nfc_example_main main.c
 * @{
 * @ingroup wake_on_nfc_example
 * @brief Wake on NFC Example Application main file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "app_timer.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_drv_clock.h"
#include "bsp.h"
#include "bsp_nfc.h"
#include "nrf.h"
#include "sdk_errors.h"

#define BTN_ID_SLEEP                0 /**< ID of button used to put the application into sleep mode. */

#define APP_TIMER_PRESCALER         0x00                                      /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE     0x04                                      /**< Size of timer operation queues. */
#define APP_TICK_EVENT_INTERVAL     APP_TIMER_TICKS(500, APP_TIMER_PRESCALER) /**< 500 miliseconds tick event interval in timer tick units. */

APP_TIMER_DEF(m_tick_timer);             /**< Timer used to toggle LED state. */

static volatile bool m_system_off_mode_on = false; /**< Indicates if the application should go into System OFF mode. */

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by BSP.
 */
static void bsp_event_handler(bsp_event_t event)
{
    switch (event) {
    case BSP_EVENT_SLEEP: {
        // Set up NFCT peripheral as the only wake up source.
        ret_code_t err_code = NRF_SUCCESS;
        err_code = bsp_nfc_sleep_mode_prepare();
        APP_ERROR_CHECK(err_code);
        err_code = bsp_buttons_disable();
        APP_ERROR_CHECK(err_code);
        err_code = app_timer_stop(m_tick_timer);
        APP_ERROR_CHECK(err_code);

        // Turn off LED to indicate that the device entered System OFF mode.
        bsp_board_led_off(BSP_BOARD_LED_0);
        m_system_off_mode_on = true;
        break;
    }

    default:
        break;
    }
}


/**
 * @brief 500 miliseconds tick handler for changing the LED state.
 */
static void app_tick_handler(void * p_context)
{
    bsp_board_led_invert(BSP_BOARD_LED_1);
}


/**@brief Function for the Timer and BSP initialization.
 */
static void utils_setup(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                        APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                        bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_nfc_btn_init(BTN_ID_SLEEP);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_tick_timer,
                                APP_TIMER_MODE_REPEATED,
                                app_tick_handler);
    APP_ERROR_CHECK(err_code);

    // Schedule a timeout event every 500 miliseconds
    err_code = app_timer_start(m_tick_timer, APP_TICK_EVENT_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    utils_setup();

    // Turn on LED to indicate that the device entered System ON mode.
    bsp_board_led_on(BSP_BOARD_LED_0);

    while (true) {
        if (m_system_off_mode_on) {
            nrf_delay_ms(100);
            // Enter System OFF mode.
            NRF_POWER->SYSTEMOFF = 1;
        }
    }
}


/** @} */
