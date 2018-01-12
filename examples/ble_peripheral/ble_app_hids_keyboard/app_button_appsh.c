#include "app_button_appsh.h"
#include "app_scheduler.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advertising.h"
#include "ble_advdata.h"
#include "ble_hids.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "bsp.h"
#include "sensorsim.h"
#include "peer_manager.h"
#include "app_timer.h"
#include "app_timer_appsh.h"
#include <stdarg.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_gpiote.h"

//#include "bsp_btn_ble.h"
#define INPUT_REPORT_KEYS_INDEX          0                                           /**< Index of Input Report. */
#define INPUT_CCONTROL_KEYS_INDEX		 1
#define INPUT_SYSTEM_KEYS_INDEX		 2
#define INPUT_REPORT_KEYS_MAX_LEN        8                                           /**< Maximum length of the Input Report characteristic. */
#define APP_TIMER_PRESCALER              63                                        /**< Value of the RTC1 PRESCALER register. Tick is roughly 1.9ms */
#define APP_TIMER_OP_QUEUE_SIZE          8                                          /**< Size of timer operation queues. */
#define INPUT_CC_REPORT_KEYS_MAX_LEN	 3
#define INPUT_SYSTEM_KEYS_MAX_LEN	 1

APP_TIMER_DEF(m_debounce_timer_id);                          /**< Battery timer. */
APP_TIMER_DEF(m_sleep_timer_id);                          /**< Battery timer. */

extern uint16_t   m_conn_handle;
extern ble_hids_t m_hids;
extern bool       m_in_boot_mode;
static uint16_t pressed_key_d1[8];     /* Delay 1 interval */
static uint16_t pressed_key_curr[8];
static uint16_t pressed_key_prev[8];
//static uint8_t m_auth_key_input = 0;
//static uint8_t m_auth_key_cnt;
//static bool m_is_new_pairing;

typedef enum
{
    RELEASE_KEY                     = 0x00,
    CONSUMER_CTRL_PLAY              = 0x01,
    CONSUMER_CTRL_ALCCC             = 0x02,
    CONSUMER_CTRL_SCAN_NEXT_TRACK   = 0x04,
    CONSUMER_CTRL_SCAN_PREV_TRACK   = 0x08,
    CONSUMER_CTRL_VOL_DW            = 0x10,
    CONSUMER_CTRL_VOL_UP            = 0x20,
    CONSUMER_CTRL_AC_FORWARD        = 0x40,
    CONSUMER_CTRL_AC_BACK           = 0x80,    
} consumer_control_t;


#define KEY_RESERVED 0
volatile struct key_report {
    uint8_t modifiers;
    uint8_t reserved;
    uint8_t keycodes[6];
} __attribute__((packed)) key_report;

volatile uint32_t media_key;
volatile uint8_t system_key;

static uint32_t delay_before_sleep = 1000000;

//static uint8_t m_func_key;

#ifdef V1
    const uint32_t scan[ROW_NUM] = {12, 2, 28, 24, 23, 22, 21, 15};
    const uint32_t resp[COL_NUM] = {29, 25, 11, 3, 10, 1, 4, 5, 7, 6, 16, 14};
#else     
    const uint32_t scan[ROW_NUM] = {12, 15, 8, 29, 24, 23, 22, 21};
    const uint32_t resp[COL_NUM] = {16, 28, 14, 25, 7, 30, 11, 4, 5, 9, 10, 6};
#endif    


#ifdef V1
uint32_t row_mask[8] = {
0x00001000,
0x00008004,
0x10000000,
0x01000000,
0x00800000,
0x00400000,
0x00200000,
0x00000000};
uint32_t col_mask[10] = {
0x20010000,
0x02004000,
0x00000800,
0x00000008,
0x00000400,
0x00000002,
0x00000010,
0x00000020,
0x00000080,
0x00000040};
#else
uint32_t row_mask[8] = {
0x00001000,
0x00008100,
0x20000000,
0x01000000,
0x00800000,
0x00400000,
0x00200000,
0x00000000};
uint32_t col_mask[10] = {
0x10010000,
0x02004000,
0x00000080,
0x40000000,
0x00000800,
0x00000010,
0x00000020,
0x00000200,
0x00000400,
0x00000040};
#endif 

/*
uint8_t keycode[7][10] = {
{0x1e, 0x29, 0x14, 0x2b, 0x04, 0x39, 0x00, 0xe1, 0xe3, 0xe0},
{0x20, 0x1f, 0x08, 0x1a, 0x07, 0x16, 0x1b, 0x1d, 0xe2, 0xF1},
{0x22, 0x21, 0x17, 0x15, 0x0a, 0x09, 0x19, 0x06, 0x00, 0x2c},
{0x24, 0x23, 0x18, 0x1c, 0x0d, 0x0b, 0x11, 0x05, 0xe6, 0x2c},
{0x26, 0x25, 0x12, 0x0c, 0x0f, 0x0e, 0x36, 0x10, 0x50, 0x76},
{0x2d, 0x27, 0x2f, 0x13, 0x34, 0x33, 0x38, 0x37, 0x4f, 0x51},
{0x35, 0x2e, 0x2a, 0x30, 0x31, 0x00, 0xe5, 0x28, 0x4c, 0x52}
};
*/

uint8_t a_to_c[128] =
{
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x2a, 0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x29, 0x00, 0x00, 0x00, 0x00,
0x2c, 0x9e, 0xb4, 0xa0, 0xa1, 0xa2, 0xa4, 0x34,
0xa6, 0xa7, 0xa5, 0xae, 0xb6, 0x2d, 0x37, 0x38,
0x27, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24,
0x25, 0x26, 0xb3, 0x33, 0x36, 0x2e, 0xb7, 0xb8,
0x9f, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a,
0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92,
0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a,
0x9b, 0x9c, 0x9d, 0x2f, 0x31, 0x30, 0xa3, 0xad,
0x35, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a,
0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12,
0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a,
0x1b, 0x1c, 0x1d, 0xaf, 0xb1, 0xb0, 0xb5, 0x7f
};

uint8_t (*keycode)[7][10];
uint16_t (*fn_actions);

uint8_t pressed_key_value[7][10];
uint8_t layer_state;
uint8_t default_layer;

#define KEY_TRANS 0x01
#define ACTIONS_LAYER_MOMENTARY 0x02
#define ACTIONS_LAYER_TOGGLE    0x03
#define ACTIONS_LAYER_ON        0x04
#define ACTIONS_LAYER_OFF       0x05
#define ACTIONS_LAYER_SET       0x06

typedef struct conn_info 
{
    volatile uint32_t conn_id;
    ble_gap_addr_t addr;
} conn_info;

void get_base_mac_addr(ble_gap_addr_t *addr) 
{
    pm_id_addr_get(addr);    
    addr->addr[0] = (uint8_t)NRF_FICR->DEVICEADDR[0];
    addr->addr[1] = (uint8_t)(NRF_FICR->DEVICEADDR[0] >> 8);
    addr->addr[2] = (uint8_t)(NRF_FICR->DEVICEADDR[0] >> 16);
    addr->addr[3] = (uint8_t)(NRF_FICR->DEVICEADDR[0] >> 24);
    addr->addr[4] = (uint8_t)NRF_FICR->DEVICEADDR[1];
    addr->addr[5] = (uint8_t)((NRF_FICR->DEVICEADDR[1] >> 8) | 0xC0);         
}


extern conn_info *connection_info;
uint8_t set_key(uint8_t row, uint8_t col, uint8_t press) 
{
    uint8_t i;
    uint8_t layer;
    uint8_t layer_state_mask;
    uint8_t kc;
    uint8_t pressed_key;
    uint32_t conn_id;
    pm_peer_id_t peer_id;
    ble_gap_addr_t addr;
    conn_info c_info;
    uint16_t c_info_len = sizeof(c_info);
    uint8_t device_name[] = "Mickey-0112b-f";
    ble_gap_conn_sec_mode_t sec_mode;
    ret_code_t err_code;    

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    
    layer = 7;
    layer_state_mask = (1 << 7UL);
    if (press) {
        do {
            while((layer_state_mask & layer_state) == 0) {
                layer_state_mask >>= 1;
                layer--;
            }
            kc = keycode[layer][row][col];
        } while((kc == KEY_TRANS) && (layer_state_mask >>= 1) && (layer--));
        pressed_key_value[row][col] = kc;
        
        if ((kc & 0xE0) == 0xC0) {
            if ((kc & 0x1F) < 24 && (kc & 0x1F) >= 20) {
                conn_id = (kc & 0x1F) - 20;
                peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
                while ((peer_id != PM_PEER_ID_INVALID)) {
                    if (pm_peer_data_app_data_load(peer_id, (uint8_t*)&c_info, &c_info_len) == NRF_SUCCESS) {
                        if (conn_id == c_info.conn_id)  {
                            pm_peer_delete(peer_id);
                        }
                    }

                    peer_id = pm_next_peer_id_get(peer_id);                    
                }
                
                NRF_LOG_INFO("New conn %02x\r\n", conn_id);
                /* err_code = pm_id_addr_get(&addr); */
                /* if (err_code != NRF_SUCCESS) { */
                /*     NRF_LOG_INFO("pm_id_addr_get %02x\r\n", err_code); */
                /* } */
                
                /* addr.addr[4] += (conn_id - connection_info->conn_id); */
                connection_info->conn_id = conn_id;
                get_base_mac_addr(&addr);
                addr.addr[4] += (connection_info->conn_id);
                err_code = pm_id_addr_set(&addr);
                if (err_code != NRF_SUCCESS) {
                    NRF_LOG_INFO("pm_id_addr_set %02x\r\n", err_code);
                }
                
//                snprintf(device_name+7, 4+1, "%02x%02x", addr.addr[4], addr.addr[5]);
                snprintf((char *)device_name+13, 2, "%d", connection_info->conn_id + 1);
                err_code = sd_ble_gap_device_name_set(&sec_mode,
                                           device_name,
                                           sizeof(device_name));
                if (err_code != NRF_SUCCESS) {
                    NRF_LOG_INFO("device name set %02x\r\n", err_code);
                }
                NVIC_SystemReset();								                
            }
            if ((kc & 0x1F) >= 28) {
                conn_id = (kc & 0x1F) - 28;
                peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
                NRF_POWER->GPREGRET = 0;
                while ((peer_id != PM_PEER_ID_INVALID)) {
                    if (pm_peer_data_app_data_load(peer_id, (uint8_t*)&c_info, &c_info_len) == NRF_SUCCESS) {
                        if (conn_id == c_info.conn_id)  {
                            connection_info->conn_id = conn_id;
                            NVIC_SystemReset();								                            
                        }
                    }
                    peer_id = pm_next_peer_id_get(peer_id);                    
                }
                NRF_LOG_INFO("New conn %02x\r\n", conn_id);
                /* err_code = pm_id_addr_get(&addr); */
                /* if (err_code != NRF_SUCCESS) { */
                /*     NRF_LOG_INFO("pm_id_addr_get %02x\r\n", err_code); */
                /* } */
                get_base_mac_addr(&addr);
                addr.addr[4] += conn_id;
                connection_info->conn_id = conn_id;
                err_code = pm_id_addr_set(&addr);
                if (err_code != NRF_SUCCESS) {
                    NRF_LOG_INFO("pm_id_addr_set %02x\r\n", err_code);
                }
                snprintf((char *)device_name+13, 2, "%d", connection_info->conn_id + 1);
                err_code = sd_ble_gap_device_name_set(&sec_mode,
                                                      device_name,
                                                      sizeof(device_name));
                if (err_code != NRF_SUCCESS) {
                    NRF_LOG_INFO("device name set %02x\r\n", err_code);
                }
                NVIC_SystemReset();								                                
            }
            if ((kc & 0x1F) == 27) {
                /* Erase bonds */
                NRF_POWER->GPREGRET = 'E';			
                NVIC_SystemReset();								
            }
            if ((kc & 0x1F) == 26) {
                /* Enter bootloader */
                peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
                NRF_LOG_INFO("Latest peer id %d.\r\n", peer_id);                                    
                while ((peer_id != PM_PEER_ID_INVALID)) {
                    NRF_LOG_INFO("So, Current peer id %d.\r\n", peer_id);                    
                    peer_id = pm_next_peer_id_get(peer_id);                    
                }
                NRF_LOG_FLUSH();
                NRF_POWER->GPREGRET = 'U';			                
                sleep_mode_enter();
            }
            
            /* Function Key */
            switch(fn_actions[kc & 0x1F] >> 8) {
            case ACTIONS_LAYER_MOMENTARY:
                layer_state |= (1 << (fn_actions[kc & 0x1F] & 0x0F));
                break;
            case ACTIONS_LAYER_TOGGLE:
                layer_state ^= (1 << (fn_actions[kc & 0x1F] & 0x0F));
                break;
            case ACTIONS_LAYER_ON:
                layer_state |= ((((fn_actions[kc & 0x1F] & 0xF0) == 0x30) ||
                                     ((fn_actions[kc & 0x1F] & 0xF0) == 0x10)) ?
                                     (1 << (fn_actions[kc & 0x1F] & 0x0F)) : 0);
                break;
            case ACTIONS_LAYER_OFF:
                layer_state &= ~((((fn_actions[kc & 0x1F] & 0xF0) == 0x30) ||
                                     ((fn_actions[kc & 0x1F] & 0xF0) == 0x10)) ?
                                      (1 << (fn_actions[kc & 0x1F] & 0x0F)) : 0);
                break;
            case ACTIONS_LAYER_SET:
                layer_state = 1;
                layer_state |= ((((fn_actions[kc & 0x1F] & 0xF0) == 0x30) ||
                                     ((fn_actions[kc & 0x1F] & 0xF0) == 0x10)) ?
                                     (1 << (fn_actions[kc & 0x1F] & 0x0F)) : 0);
                break;
            }

        } else if (kc >= 0xA5 && kc <= 0xA7) {
            system_key = 0x81 + (kc - 0xA5);
        } else if (kc >= 0xA8 && kc <= 0xBC) {
            media_key |= (1UL << (uint32_t)(kc - 0xA8));
        } else if ((kc & 0xF0) == 0xE0) {            
            /* Modifier Key */
            key_report.modifiers |= (1 << (kc & 0x07));
        } else {
            for (i = 0; i < 6; i++) {
                pressed_key = key_report.keycodes[i];
                if (pressed_key == kc) {
                    return 1;
                }
            }
            for (i = 0; i < 6; i++) {
                pressed_key = key_report.keycodes[i];
                if (pressed_key == KEY_RESERVED) {
                    key_report.keycodes[i] = kc;
                    return 1;
                }
            }
        }
    } else {
        kc = pressed_key_value[row][col];
        if ((kc & 0xE0) == 0xC0) {
            /* Function Key */
            switch(fn_actions[kc & 0x1F] >> 8) {
            case ACTIONS_LAYER_MOMENTARY:
                layer_state &= ~(1 << (fn_actions[kc & 0x1F] & 0x0F));
                break;
            case ACTIONS_LAYER_ON:
                layer_state |= ((((fn_actions[kc & 0x1F] & 0xF0) == 0x30) ||
                                     ((fn_actions[kc & 0x1F] & 0xF0) == 0x20)) ?
                                     (1 << (fn_actions[kc & 0x1F] & 0x0F)) : 0);
                break;
            case ACTIONS_LAYER_OFF:
                layer_state &= ~((((fn_actions[kc & 0x1F] & 0xF0) == 0x30) ||
                                     ((fn_actions[kc & 0x1F] & 0xF0) == 0x20)) ?
                                      (1 << (fn_actions[kc & 0x1F] & 0x0F)) : 0);
                break;
            case ACTIONS_LAYER_SET:
                layer_state = 1;
                layer_state |= ((((fn_actions[kc & 0x1F] & 0xF0) == 0x30) ||
                                     ((fn_actions[kc & 0x1F] & 0xF0) == 0x20)) ?
                                     (1 << (fn_actions[kc & 0x1F] & 0x0F)) : 0);
                break;
            }
        } else if (kc >= 0xA5 && kc <= 0xA7) {            
            media_key &= ~(1UL << (uint32_t)(kc - 0xA5));            
        } else if (kc >= 0xA8 && kc <= 0xBC) {            
            media_key &= ~(1UL << (uint32_t)(kc - 0xA8));            
        } else if ((kc & 0xF0) == 0xE0) {
            /* Modifier Key */
            key_report.modifiers &= ~(1 << (kc & 0x07));
        } else {
            for (i = 0; i < 6; i++) {
                pressed_key = key_report.keycodes[i];
                if (pressed_key == kc) {
                    key_report.keycodes[i] = KEY_RESERVED;
                    return 1;
                }
            }
        }
    }
    return 0;
}
volatile int i;
char string[128];
void xprintf(char* fmt, ...)
{
    i = 0;
    va_list ap;
//    static uint8_t * p_key;
//    uint8_t key;
    va_start (ap, fmt);
    vsprintf(string, fmt, ap);
    va_end(ap);
}
void process_printf() 
{
    int k;
    int pre_i;

    uint32_t err_code;
    if (i < strlen(string)) {
        if (string[i] > 0) {
            pre_i = i;
            if (a_to_c[string[i]] & 0x80) {
                key_report.modifiers |= (1 << (0x01 & 0x07));
            } else {
                key_report.modifiers &= ~(1 << (0x01 & 0x07));
            }
            if (key_report.keycodes[0] != (a_to_c[string[i]] & ~0x80)) {
                key_report.keycodes[0] = a_to_c[string[i]] & ~0x80;
                i++;
            } else {
                key_report.keycodes[0] = 0;
            }
            for (k = 1; k < 6; k++) {
                key_report.keycodes[k] = 0;
            }
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
                if (!m_in_boot_mode) {
                    err_code = ble_hids_inp_rep_send(&m_hids,
                                          INPUT_REPORT_KEYS_INDEX,
                                          INPUT_REPORT_KEYS_MAX_LEN,
                                          (uint8_t*)&key_report);
                } else {
                    err_code = ble_hids_boot_kb_inp_rep_send(&m_hids,
                                                  INPUT_REPORT_KEYS_MAX_LEN,
                                                  (uint8_t*)&key_report);
                }
            }
            if (err_code != NRF_SUCCESS) {
                i = pre_i;
                key_report.keycodes[0] = 0;                
            }
        }
    } else {

    }
}

//static uint8_t passkey[6];
//static uint8_t passkey_prev = KEY_RESERVED;
#define NO_KEY_RETRY_NUM 10
static void button_handler(app_button_event_t* button_event)
{
    uint16_t i, j;
    uint16_t key_curr;
    uint16_t key_prev;
    uint32_t key_buf;
    uint32_t key_buf_tmp;
    uint32_t err_code;
    bool no_key_pressed = true;
    static bool no_key_pressed_p[NO_KEY_RETRY_NUM];
    bool no_key_pressed_prev;
    static uint16_t media_key_prev;

    keycode = (void*)(0x32000 + 64);
    fn_actions = (void*)(0x32000);
    
    for (i = 0; i < ROW_NUM; i++) {
        nrf_gpio_cfg_output(scan[i]);
    }
    for (i = 0; i < COL_NUM; i++) {
        nrf_gpio_cfg_input(resp[i], GPIO_PIN_CNF_PULL_Pulldown);
    }    
    no_key_pressed = true;
    media_key = 0;
    system_key = 0;    
    NRF_GPIO->OUT = row_mask[0];        
    for (i = 0; i < 7; i++) {
        key_buf_tmp = NRF_GPIO->IN;
        pressed_key_prev[i] = pressed_key_curr[i];
        key_buf = NRF_GPIO->IN;
        NRF_GPIO->OUT = row_mask[i + 1];        
        if (key_buf_tmp != key_buf) continue;
        for (j = 0; j < 10; j++) {
            if (key_buf & col_mask[j]) {
                if (pressed_key_d1[i] & (1 << j)) {
                    pressed_key_curr[i] |= (1 << j);
                }
                pressed_key_d1[i] |= (1 << j);
                no_key_pressed = false;                
            } else {
                if (!(pressed_key_d1[i] & (1 << j))) {
                    pressed_key_curr[i] &= ~(1 << j);
                }
                pressed_key_d1[i] &= ~(1 << j);
            }
        }
    }
    for (i = 0; i < COL_NUM; i++) {
        nrf_gpio_cfg_input(resp[i], GPIO_PIN_CNF_PULL_Disabled);
    }  
    for (i = 0; i < 7; i++) {
        key_curr = pressed_key_curr[i];
        if (key_curr != 0) no_key_pressed = false;
        key_prev = pressed_key_prev[i];
        if (key_curr != key_prev) {
            for (j = 0; j < 10; j++) {
                if ((key_curr ^ key_prev) & (1 << j)) {
                    if (!!(key_curr & (1 << j))) {
//                        NRF_LOG_INFO("%d %d %08x pressed!\r\n", i, j, key_buf_tmp);
                    } else {
//                        NRF_LOG_INFO("%d %d %08x released!\r\n", i, j, key_buf_tmp);
                    }
                    set_key(i, j, !!(key_curr & (1 << j)));
                }
            }
        }
    }
    /*
    if (m_auth_key_input == 1) {
        if (key_report.keycodes[0] == 0x28) { 
// Enter 
            m_auth_key_input = 0;
            sd_ble_gap_auth_key_reply(m_conn_handle, BLE_GAP_AUTH_KEY_TYPE_PASSKEY, passkey);
        } else {
            if (passkey_prev != key_report.keycodes[0] && key_report.keycodes[0] != KEY_RESERVED) {
                passkey[m_auth_key_cnt++] = 0x30 + (key_report.keycodes[0] == 0x27 ? 0 : key_report.keycodes[0] - 0x1D);
            }
        }
        passkey_prev = key_report.keycodes[0];
    }
    */

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID && button_event == NULL) {
        if (!m_in_boot_mode) {
            ble_hids_inp_rep_send(&m_hids,
                                  INPUT_REPORT_KEYS_INDEX,
                                  INPUT_REPORT_KEYS_MAX_LEN,
                                  (uint8_t*)&key_report);
            if (media_key != 0) {
                ble_hids_inp_rep_send(&m_hids,
                                      INPUT_CCONTROL_KEYS_INDEX,
                                      INPUT_CC_REPORT_KEYS_MAX_LEN,
                                      (uint8_t*)&media_key);
            } else if (media_key_prev != 0) {
                /* Media key release, so send all zero */
                ble_hids_inp_rep_send(&m_hids,
                                      INPUT_CCONTROL_KEYS_INDEX,
                                      INPUT_CC_REPORT_KEYS_MAX_LEN,
                                      (uint8_t*)&media_key);
            }
            
            if (system_key != 0) {
                ble_hids_inp_rep_send(&m_hids,
                                      INPUT_SYSTEM_KEYS_INDEX,
                                      INPUT_SYSTEM_KEYS_MAX_LEN,
                                      (uint8_t*)&system_key);
            }
        } else {
            ble_hids_boot_kb_inp_rep_send(&m_hids,
                                          INPUT_REPORT_KEYS_MAX_LEN,
                                          (uint8_t*)&key_report);
        }
    }

    no_key_pressed_prev = true;
    for (i = 0; i < NO_KEY_RETRY_NUM; i++) {
        if (no_key_pressed_p[i] == false) no_key_pressed_prev = false;
    }
    
    if (no_key_pressed && no_key_pressed_prev) {
        nrf_gpiote_event_clear(NRF_GPIOTE_EVENTS_PORT);        
        nrf_gpiote_int_enable(GPIOTE_INTENSET_PORT_Msk);                
        NRF_LOG_INFO("No KEY pressed!\r\n");        
        if (app_timer_stop(m_sleep_timer_id) == NRF_SUCCESS) {
            err_code = app_timer_start(m_sleep_timer_id, delay_before_sleep, NULL);
            APP_ERROR_CHECK(err_code);
        }
        snooze_mode_enter();
    } else {
        err_code = app_timer_start(m_debounce_timer_id, 10, NULL);
    }
    for (i = NO_KEY_RETRY_NUM - 1; i > 0; i--) {
        no_key_pressed_p[i] =  no_key_pressed_p[i - 1];
    }
    no_key_pressed_p[0] = no_key_pressed;
    if (button_event == NULL) {
        /* Only for timer */
        media_key_prev = media_key;
    }
}
uint32_t app_button_evt_schedule()
{
    app_button_event_t button_event;
    return app_sched_event_put(&button_event, sizeof(button_event), button_handler);
}
void snooze_mode_enter(void)
{
    int i;
//    uint32_t err_code;
    battery_level_update();
    /* uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE); */
    //NRF_LOG_INFO("Enter snooze mode!\r\n");
    /* APP_ERROR_CHECK(err_code); */

    // Prepare wakeup buttons.
    /* err_code = bsp_btn_ble_sleep_mode_prepare(); */
    /* APP_ERROR_CHECK(err_code); */
    for (i = 0; i < COL_NUM; i++) {
        nrf_gpio_cfg_input(resp[i], GPIO_PIN_CNF_PULL_Disabled);
    }
    for (i = 0; i < COL_NUM; i++) {
        nrf_gpio_cfg_sense_set(resp[i], NRF_GPIO_PIN_SENSE_HIGH);
    }
    for (i = 0; i < ROW_NUM; i++) {
        nrf_gpio_cfg_input(scan[i], GPIO_PIN_CNF_PULL_Pullup);
    }
    for (i = 0; i < COL_NUM; i++) {
        nrf_gpio_cfg_sense_set(resp[i], NRF_GPIO_PIN_SENSE_HIGH);
    }
    nrf_drv_common_irq_enable(GPIOTE_IRQn, GPIOTE_CONFIG_IRQ_PRIORITY);
    nrf_gpiote_event_clear(NRF_GPIOTE_EVENTS_PORT);
    nrf_gpiote_int_enable(GPIOTE_INTENSET_PORT_Msk);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
void sleep_mode_enter(void)
{
//    int i;
    uint32_t err_code;
    snooze_mode_enter();    
    
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
void buttons_leds_init(bool * p_erase_bonds)
{
//    int i;
    layer_state = 1;
    media_key = 0;
    system_key = 0;
    app_timer_start(m_sleep_timer_id, delay_before_sleep, NULL);    
    snooze_mode_enter();
    /* nrf_drv_common_irq_enable(GPIOTE_IRQn, GPIOTE_CONFIG_IRQ_PRIORITY); */
    /* nrf_gpiote_event_clear(NRF_GPIOTE_EVENTS_PORT); */
    /* nrf_gpiote_int_enable(GPIOTE_INTENSET_PORT_Msk); */
    button_handler(NULL);
}
static void debounce_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
//    NRF_LOG_INFO("Debounce Timeout Handler!\r\n");
    button_handler(NULL);
}

volatile uint32_t curr_time;
static void sleep_timeout_handler(void * p_context)
{
    volatile uint32_t err_code;    
    UNUSED_PARAMETER(p_context);
//    NRF_LOG_INFO("Sleep Timeout Handler!\r\n");
    sleep_mode_enter();
}
/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module, making it use the scheduler.
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);

    // Create battery timer.
    err_code = app_timer_create(&m_debounce_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                debounce_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_sleep_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                sleep_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


void GPIOTE_IRQHandler(void)
{
    /* collect PORT status event, if event is set read pins state. Processing is postponed to the
     * end of interrupt. */
    if (nrf_gpiote_event_is_set(NRF_GPIOTE_EVENTS_PORT)) {
        nrf_gpiote_int_disable(GPIOTE_INTENSET_PORT_Msk);        
        nrf_gpiote_event_clear(NRF_GPIOTE_EVENTS_PORT);
        
        app_button_evt_schedule();
        /* status |= (uint32_t)NRF_GPIOTE_INT_PORT_MASK; */
        /* nrf_gpio_ports_read(0, GPIO_COUNT, input); */
    }
}

