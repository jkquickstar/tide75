// Copyright 2024 SDK (@sdk66)
// SPDX-License-Identifier: GPL-2.0-or-later

#include "control.h"
#include "suspend.h"

static ioline_t col_pins[MATRIX_COLS] = MATRIX_COL_PINS;
extern bool lower_sleep;

/* Forward declarations for weak hooks defined in lowpower.c */
void lpwr_presleep_hook(void);
void lpwr_wakeup_hook(void);
void last_matrix_activity_trigger(void);

/* ─── LPWR overrides ─────────────────────────────────────────────────
 *
 * ROOT CAUSE of Cmd+Shift+4 modifier drop after idle sleep:
 *
 * The default lpwr_wakeup_cb() (lowpower.c) calls suspend_wakeup_init()
 * which calls clear_mods() / clear_weak_mods() / clear_keys().  This
 * fires ~200ms after MCU wake (LPWR_WAKEUP_DELAY), by which time the
 * main loop has already scanned the wake-up keys and registered their
 * modifiers.  The clear_mods() call then destroys that modifier state.
 *
 * Because held keys are not re-processed by the matrix (debounce sees
 * them as already pressed), the modifiers are never re-registered.
 * When the wireless module finally delivers the next report, mods=0
 * and e.g. Cmd+Shift+4 becomes Shift+4 = '$' (Cmd lost).
 *
 * The keyboard only enters sleep after 1 minute of idle (no keys
 * held), so there is no stale state to clear on wakeup.  We override
 * both lpwr_presleep_cb() and lpwr_wakeup_cb() to:
 *   - Track the RGB enabled state ourselves (rgb_enable_bak is static
 *     in lowpower.c, inaccessible from here)
 *   - Skip suspend_wakeup_init() entirely on wakeup
 *   - Still call suspend_wakeup_init_quantum() so the quantum layer
 *     can restore LED indicators and RGB matrix suspend state
 */
static bool tide75_rgb_backup = false;

void lpwr_presleep_cb(void) {
#if defined(RGB_MATRIX_ENABLE)
    tide75_rgb_backup = rgb_matrix_is_enabled();
    rgb_matrix_disable_noeeprom();
#elif defined(RGBLIGHT_ENABLE)
    tide75_rgb_backup = rgblight_is_enabled();
    rgblight_disable_noeeprom();
#else
    tide75_rgb_backup = false;
#endif
    suspend_power_down();
    lpwr_presleep_hook();
}

void lpwr_wakeup_cb(void) {
    if (tide75_rgb_backup) {
#if defined(RGB_MATRIX_ENABLE)
        rgb_matrix_enable_noeeprom();
#elif defined(RGBLIGHT_ENABLE)
        rgblight_enable_noeeprom();
#endif
    }

    /* Skip suspend_wakeup_init() — it calls clear_mods() which destroys
     * modifier state from the wake-up keypress.  Call only the quantum-
     * level wakeup (restores LED indicators, sets RGB matrix suspend
     * state to false, calls suspend_wakeup_init_kb()). */
    suspend_wakeup_init_quantum();

    lpwr_wakeup_hook();
    last_matrix_activity_trigger();
}

bool hs_modeio_detection(bool update, uint8_t *mode,uint8_t lsat_btdev) {
    static uint32_t scan_timer = 0x00;

    if ((update != true) && (timer_elapsed32(scan_timer) <= (HS_MODEIO_DETECTION_TIME))) {
        return false;
    }
    scan_timer = timer_read32();

#if defined(HS_BT_DEF_PIN) && defined(HS_2G4_DEF_PIN)
    uint8_t now_mode         = 0x00;
    uint8_t hs_mode          = 0x00;
    static uint8_t last_mode = 0x00;
    bool sw_mode             = false;

    now_mode  = (HS_GET_MODE_PIN(HS_USB_PIN_STATE) ? 3 : (HS_GET_MODE_PIN(HS_BT_PIN_STATE) ? 1 : ((HS_GET_MODE_PIN(HS_2G4_PIN_STATE) ? 2 : 0))));
    hs_mode   = (*mode >= DEVS_BT1 && *mode <= DEVS_BT5) ? 1 : ((*mode == DEVS_2G4) ? 2 : ((*mode == DEVS_USB) ? 3 : 0));
    sw_mode   = ((update || (last_mode == now_mode)) && (hs_mode != now_mode)) ? true : false;
    last_mode = now_mode;

    switch (now_mode)
    {
        case 1:
            *mode = hs_bt;
            if (sw_mode) {
                wireless_devs_change(wireless_get_current_devs(), lsat_btdev, false); 
                
            }
            break;
        case 2:
            *mode = hs_2g4;
            if (sw_mode) {
                wireless_devs_change(wireless_get_current_devs(), DEVS_2G4, false);  
               
            }
            break;
        case 3:
            *mode = hs_usb;
            if (sw_mode) 
                wireless_devs_change(wireless_get_current_devs(), DEVS_USB, false);

            break;
        default:
            break;
    }
    
    if (sw_mode) {
        hs_rgb_blink_set_timer(timer_read32());
        suspend_wakeup_init();
        return true;
    }
#else
    *mode = hs_none;
#endif

    return false;
}

static uint32_t hs_linker_rgb_timer = 0x00;

bool hs_mode_scan(bool update,uint8_t moude,uint8_t lsat_btdev) {
   
    if (hs_modeio_detection(update, &moude, lsat_btdev)) {

        return true;
    }
    hs_rgb_blink_hook();
    return false;
}

void hs_rgb_blink_set_timer(uint32_t time) {
    hs_linker_rgb_timer = time;
}

uint32_t hs_rgb_blink_get_timer(void) {
    return hs_linker_rgb_timer;
}

bool hs_rgb_blink_hook(){
    static uint8_t last_status;

    if (last_status != *md_getp_state()){
        last_status = *md_getp_state();
        hs_rgb_blink_set_timer(0x00);
    }
    
    switch (*md_getp_state())
    {
        case MD_STATE_NONE: {
                hs_rgb_blink_set_timer(0x00);
        } break;
        
        case MD_STATE_DISCONNECTED:
            if (hs_rgb_blink_get_timer() == 0x00){
                hs_rgb_blink_set_timer(timer_read32());
                extern void wireless_devs_change_kb(uint8_t old_devs, uint8_t new_devs, bool reset);
                wireless_devs_change_kb(wireless_get_current_devs(),wireless_get_current_devs(),false);
            }
            else{
                if (timer_elapsed32(hs_rgb_blink_get_timer()) >= HS_LBACK_TIMEOUT) {
                    hs_rgb_blink_set_timer(timer_read32());
                    md_send_devctrl(MD_SND_CMD_DEVCTRL_USB);
                    wait_ms(200);
                    lpwr_set_timeout_manual(true);
                    
                }
            }
        case MD_STATE_CONNECTED:
            if (hs_rgb_blink_get_timer() == 0x00){
                hs_rgb_blink_set_timer(timer_read32());
            }
            else{
                if (timer_elapsed32(hs_rgb_blink_get_timer()) >= HS_SLEEP_TIMEOUT) {
                    hs_rgb_blink_set_timer(timer_read32());
                    lpwr_set_timeout_manual(true);
                }
            }
        default:
            break;
    }
    return true;
}

void lpwr_exti_init_hook(void){

#ifdef HS_BT_DEF_PIN
    setPinInputHigh(HS_BT_DEF_PIN);
    waitInputPinDelay();
    palEnableLineEvent(HS_BT_DEF_PIN, PAL_EVENT_MODE_BOTH_EDGES);
#endif

#ifdef HS_2G4_DEF_PIN
    setPinInputHigh(HS_2G4_DEF_PIN);
    waitInputPinDelay();
    palEnableLineEvent(HS_2G4_DEF_PIN, PAL_EVENT_MODE_BOTH_EDGES);
#endif

    if (lower_sleep){
#if DIODE_DIRECTION == ROW2COL
        for (uint8_t i = 0; i < ARRAY_SIZE(col_pins); i++) {
            if (col_pins[i] != NO_PIN) {
                setPinOutput(col_pins[i]);
                writePinHigh(col_pins[i]);
            }
        }
#endif
    }
    setPinInput(HS_BAT_CABLE_PIN);
    waitInputPinDelay();
    palEnableLineEvent(HS_BAT_CABLE_PIN, PAL_EVENT_MODE_RISING_EDGE);
}

void palcallback_cb(uint8_t line){
    switch (line) {
        case PAL_PAD(HS_BAT_CABLE_PIN): {
            lpwr_set_sleep_wakeupcd(LPWR_WAKEUP_CABLE);
        } break;
#ifdef HS_2G4_DEF_PIN
        case PAL_PAD(HS_2G4_DEF_PIN): {
            lpwr_set_sleep_wakeupcd(LPWR_WAKEUP_SWITCH);
        } break;
#endif

#ifdef HS_BT_DEF_PIN
        case PAL_PAD(HS_BT_DEF_PIN): {
            lpwr_set_sleep_wakeupcd(LPWR_WAKEUP_SWITCH);
        } break;
#endif
        default: {
            
        } break;
    }
}



void lpwr_stop_hook_post(void){
    if (lower_sleep){
        switch (lpwr_get_sleep_wakeupcd()) {
            case LPWR_WAKEUP_USB:
            case LPWR_WAKEUP_CABLE: {
                lower_sleep = false;
                lpwr_set_state(LPWR_WAKEUP);
            } break;
            default: {
                lpwr_set_state(LPWR_STOP);
            } break;
        }
        return;  /* Low-battery path: don't prime radio */
    }

    /* Prime the wireless radio after waking from idle sleep.
     *
     * After sleep the radio link to the dongle is cold.  The first keypress
     * wakes the MCU but its HID report is sent before the radio
     * re-establishes, so the modifier (e.g. Cmd in Cmd+C) is lost.
     *
     * The working wireless implementation (keyboards/wireless/) calls
     * wireless_devs_change() on wakeup to re-issue the device-mode command,
     * forcing the module to re-establish the radio link.  The linker
     * implementation used by the TIDE 75 never does this.
     *
     * This runs inside lpwr_stop_cb() right after lpwr_enter_stop() returns
     * — before the main loop resumes and before matrix_scan() processes the
     * wake-up keypress.
     */
    uint8_t devs = wireless_get_current_devs();
    if (devs == DEVS_USB) {
        return;
    }

    /* Re-issue the device-mode command to wake the module radio.
     * For 2.4G this sends MD_SND_CMD_DEVCTRL_2G4 (0x30), for BT it sends
     * the matching BT channel command.  This is what Lofree/P75 keyboards
     * do in their wakeup path via wireless_devs_change().
     */
    wireless_devs_change(devs, devs, false);

    /* Drain UART queue (~35ms) to process the devctrl command.
     * The post-sleep resync mechanism (15ms intervals, 1200ms window)
     * re-delivers the first report once the radio warms up (~60-80ms).
     * No gate is needed — modifiers are preserved by our lpwr_wakeup_cb()
     * override which skips the destructive clear_mods() call.
     */
    for (uint8_t i = 0; i < 7; i++) {
        md_main_task();
        wait_ms(5);
    }
}
