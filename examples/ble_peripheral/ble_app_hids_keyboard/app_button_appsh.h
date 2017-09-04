/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#ifndef APP_BUTTON_APPSH_H
#define APP_BUTTON_APPSH_H

#include "app_button.h"

#ifdef __cplusplus
extern "C" {
#endif

    void xprintf(char* fmt, ...);

#define ROW_NUM 8
#define COL_NUM 12
//uint8_t scan[ROW_NUM] = {4, 5, 30, 0, 1, 2, 3};
//uint8_t resp[COL_NUM] = {16, 15, 29, 6, 8, 9, 11, 12, 13, 14};
//#define V1
#define APP_BUTTON_SCHED_EVT_SIZE     sizeof(app_button_event_t)  /**< Size of button events being passed through the scheduler (is to be used for computing the maximum size of scheduler events). */

/**@brief Macro for initializing the application timer module to use with app_scheduler.
 *
 * @param[in]  PRESCALER        Value of the RTC1 PRESCALER register. This will decide the
 *                              timer tick rate. Set to 0 for no prescaling.
 * @param[in]  OP_QUEUES_SIZE   Size of queues holding timer operations that are pending execution.
 * @param[in]  USE_SCHEDULER    TRUE if the application is using the app_scheduler,
 *                              FALSE otherwise.
 *
 * @note Since this macro allocates a buffer, it must only be called once (it is OK to call it
 *       several times as long as it is from the same location, e.g. to do a reinitialization).
 */
#define APP_BUTTON_APPSH_INIT(PRESCALER, OP_QUEUES_SIZE, USE_SCHEDULER) \
    APP_BUTTON_INIT(PRESCALER, OP_QUEUES_SIZE,                          \
                    (USE_SCHEDULER) ? app_timer_evt_schedule : NULL)

    typedef struct {
        app_button_handler_t button_handler;
        void *                      p_context;
    } app_button_event_t;

    uint32_t app_button_evt_schedule(void);
    void buttons_leds_init(bool * p_erase_bonds);
    void sleep_mode_enter(void);
    void snooze_mode_enter(void);
    

#ifdef __cplusplus
}
#endif

#endif // APP_TIMER_APPSH_H

