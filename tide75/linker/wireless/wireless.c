
// Copyright 2024 Su (@isuua)
// SPDX-License-Identifier: GPL-2.0-or-later

#include "quantum.h"
#include "wireless.h"

#ifndef WLS_INQUIRY_BAT_TIME
#    define WLS_INQUIRY_BAT_TIME 3000
#endif

#ifndef WLS_KEEPALIVE_INTERVAL
#    define WLS_KEEPALIVE_INTERVAL 1000
#endif

#ifndef WLS_POSTSLEEP_RESYNC_DELAY
#    define WLS_POSTSLEEP_RESYNC_DELAY 50
#endif

#ifndef WLS_POSTSLEEP_RESYNC_COUNT
#    define WLS_POSTSLEEP_RESYNC_COUNT 30
#endif

static uint8_t wls_devs = DEVS_USB;
static bool report_dropped      = false;
static uint32_t keepalive_timer = 0;

static bool     postsleep_resync = false;
static uint32_t postsleep_timer  = 0;
static uint8_t  postsleep_count  = 0;

void last_matrix_activity_trigger(void);

uint8_t wireless_keyboard_leds(void);
void wireless_send_keyboard(report_keyboard_t *report);
void wireless_send_nkro(report_nkro_t *report);
void wireless_send_mouse(report_mouse_t *report);
void wireless_send_extra(report_extra_t *report);

host_driver_t wireless_driver = {
    .keyboard_leds = wireless_keyboard_leds,
    .send_keyboard = wireless_send_keyboard,
    .send_nkro     = wireless_send_nkro,
    .send_mouse    = wireless_send_mouse,
    .send_extra    = wireless_send_extra,
};

void wireless_init(void) {

    md_init();
}

uint8_t wireless_keyboard_leds(void) {

    if (*md_getp_state() == MD_STATE_CONNECTED) {
        return *md_getp_indicator();
    }

    return 0;
}

void wireless_send_keyboard(report_keyboard_t *report) {
    uint8_t wls_report_kb[MD_SND_CMD_KB_LEN] = {0};

    if (*md_getp_state() != MD_STATE_CONNECTED) {
        report_dropped = true;
        wireless_devs_change(wls_devs, wls_devs, false);
        return;
    }

    /* When waking from sleep (or during presleep/stop transitions) the
     * radio link is cold.  The first report will likely be lost during
     * radio/USB re-establishment.  Schedule aggressive resyncs to
     * re-deliver the keyboard state once the link is warm.
     * 1500ms of coverage (30 × 50ms) handles even slow USB resume.
     */
    lpwr_state_t state = lpwr_get_state();
    if (state == LPWR_WAKEUP || state == LPWR_STOP || state == LPWR_PRESLEEP) {
        postsleep_resync = true;
        postsleep_count  = 0;
        postsleep_timer  = sync_timer_read32();
    }

    if (report != NULL) {
        memcpy(wls_report_kb, (uint8_t *)report, sizeof(wls_report_kb));
    }
    md_send_kb(wls_report_kb);
}

void wireless_send_nkro(report_nkro_t *report) {
    static report_keyboard_t temp_report_keyboard = {0};
    uint8_t wls_report_nkro[MD_SND_CMD_NKRO_LEN]  = {0};

    if (*md_getp_state() != MD_STATE_CONNECTED) {
        report_dropped = true;
        wireless_devs_change(wls_devs, wls_devs, false);
        return;
    }

    if (report != NULL) {
        report_nkro_t temp_report_nkro = *report;
        uint8_t key_count              = 0;

        temp_report_keyboard.mods = temp_report_nkro.mods;
        for (uint8_t i = 0; i < NKRO_REPORT_BITS; i++) {
            key_count += __builtin_popcount(temp_report_nkro.bits[i]);
        }

        // find key up and del it.
        for (uint8_t i = 0; i < KEYBOARD_REPORT_KEYS && temp_report_keyboard.keys[i]; i++) {
            uint8_t usageid = 0x00;
            uint8_t n;

            for (uint8_t c = 0; c < key_count; c++) {
                for (n = 0; n < NKRO_REPORT_BITS && !temp_report_nkro.bits[n]; n++) {}
                usageid = (n << 3) | biton(temp_report_nkro.bits[n]);
                del_key_bit(&temp_report_nkro, usageid);
                if (usageid == temp_report_keyboard.keys[i]) {
                    break;
                }
            }

            if (usageid != temp_report_keyboard.keys[i]) {
                temp_report_keyboard.keys[i] = 0x00;
            }
        }

        /*
         * Use NKRO for sending when more than 6 keys are pressed
         * to solve the issue of the lack of a protocol flag in wireless mode.
         */

        temp_report_nkro = *report;

        for (uint8_t i = 0; i < key_count; i++) {
            uint8_t usageid;
            uint8_t idx, n = 0;

            for (n = 0; n < NKRO_REPORT_BITS && !temp_report_nkro.bits[n]; n++) {}
            usageid = (n << 3) | biton(temp_report_nkro.bits[n]);
            del_key_bit(&temp_report_nkro, usageid);

            for (idx = 0; idx < KEYBOARD_REPORT_KEYS; idx++) {
                if (temp_report_keyboard.keys[idx] == usageid) {
                    break;
                }
                if (temp_report_keyboard.keys[idx] == 0x00) {
                    temp_report_keyboard.keys[idx] = usageid;
                    break;
                }
            }

            if (idx == KEYBOARD_REPORT_KEYS && (usageid < (MD_SND_CMD_NKRO_LEN * 8))) {
                wls_report_nkro[usageid / 8] |= 0x01 << (usageid % 8);
            }
        }
    } else {
        memset(&temp_report_keyboard, 0, sizeof(temp_report_keyboard));
    }

    wireless_driver.send_keyboard(&temp_report_keyboard);

    /* Only send the NKRO overflow bitmap when it actually contains key
     * data (i.e. more than 6 keys are pressed simultaneously).
     *
     * For normal typing (≤6 keys), the NKRO bitmap is all zeros.
     * Sending that empty 0xA2 message after the 6KRO 0xA1 message
     * creates a race on the dongle: both arrive close together and
     * the empty NKRO can overwrite the 6KRO state (including
     * modifiers) within the same USB poll interval, causing the host
     * to see only the key release.  Skipping the empty NKRO
     * eliminates this race entirely.
     */
    bool has_nkro_overflow = false;
    for (uint8_t i = 0; i < MD_SND_CMD_NKRO_LEN; i++) {
        if (wls_report_nkro[i]) {
            has_nkro_overflow = true;
            break;
        }
    }
    if (has_nkro_overflow) {
        md_send_nkro(wls_report_nkro);
    }
}

void wireless_send_mouse(report_mouse_t *report) {
    typedef struct {
        uint8_t buttons;
        int8_t x;
        int8_t y;
        int8_t z;
        int8_t h;
    } __attribute__((packed)) wls_report_mouse_t;

    wls_report_mouse_t wls_report_mouse = {0};

    if (*md_getp_state() != MD_STATE_CONNECTED) {
        wireless_devs_change(wls_devs, wls_devs, false);
        return;
    }

    if (report != NULL) {
        wls_report_mouse.buttons = report->buttons;
        wls_report_mouse.x       = report->x;
        wls_report_mouse.y       = report->y;
        wls_report_mouse.z       = report->h;
        wls_report_mouse.h       = report->v;
    }

    md_send_mouse((uint8_t *)&wls_report_mouse);
}

void wireless_send_extra(report_extra_t *report) {
    uint16_t usage = 0;

    if (*md_getp_state() != MD_STATE_CONNECTED) {
        wireless_devs_change(wls_devs, wls_devs, false);
        return;
    }

    if (report != NULL) {
        usage = report->usage;

        switch (usage) {
            case 0x81:
            case 0x82:
            case 0x83: { // system usage
                usage = 0x01 << (usage - 0x81);
                md_send_system((uint8_t *)&usage);
            } break;
            default: {
                md_send_consumer((uint8_t *)&usage);
            } break;
        }
    }
}

void wireless_devs_change_user(uint8_t old_devs, uint8_t new_devs, bool reset) __attribute__((weak));
void wireless_devs_change_user(uint8_t old_devs, uint8_t new_devs, bool reset) {}

void wireless_devs_change_kb(uint8_t old_devs, uint8_t new_devs, bool reset) __attribute__((weak));
void wireless_devs_change_kb(uint8_t old_devs, uint8_t new_devs, bool reset) {}

void wireless_devs_change(uint8_t old_devs, uint8_t new_devs, bool reset) {
    bool changed = (old_devs == DEVS_USB) ? (new_devs != DEVS_USB) : (new_devs == DEVS_USB);

    if (changed) {
        set_transport((new_devs != DEVS_USB) ? TRANSPORT_WLS : TRANSPORT_USB);
    }

    if ((wls_devs != new_devs) || reset) {
        *md_getp_state()     = MD_STATE_DISCONNECTED;
        *md_getp_indicator() = 0;
    }

    wls_devs = new_devs;
    last_matrix_activity_trigger();

    md_devs_change(new_devs, reset);
    wireless_devs_change_kb(old_devs, new_devs, reset);
    wireless_devs_change_user(old_devs, new_devs, reset);
}

uint8_t wireless_get_current_devs(void) {
    return wls_devs;
}

void wireless_pre_task(void) __attribute__((weak));
void wireless_pre_task(void) {}

void wireless_post_task(void) __attribute__((weak));
void wireless_post_task(void) {}

void wireless_task(void) {

    wireless_pre_task();
    lpwr_task();
    md_main_task();
    wireless_post_task();

    /* Resync after module reconnection.
     * When reports are dropped because the module was not connected,
     * QMK's report dedup cache still considers them "sent."  Invalidate
     * the cache and force a resend.
     */
    if (report_dropped && *md_getp_state() == MD_STATE_CONNECTED) {
        report_dropped = false;
        keepalive_timer = sync_timer_read32();

#ifdef NKRO_ENABLE
        extern keymap_config_t keymap_config;
        if (keyboard_protocol && keymap_config.nkro) {
            nkro_report_dedup_invalidate();
        } else
#endif
        {
            keyboard_report_dedup_invalidate();
        }

        extern report_keyboard_t *keyboard_report;
#ifdef NKRO_ENABLE
        extern report_nkro_t *nkro_report;
        if (keyboard_protocol && keymap_config.nkro) {
            host_nkro_send(nkro_report);
        } else
#endif
        {
            host_keyboard_send(keyboard_report);
        }
    }

    /* Post-sleep resync: force-resend keyboard state at 100ms intervals
     * after waking from sleep.  The first report is sent immediately by
     * wireless_send_keyboard() but is typically lost while the radio/USB
     * link re-establishes.  These resyncs re-deliver the state once the
     * link is warm.
     */
    if (postsleep_resync && *md_getp_state() == MD_STATE_CONNECTED &&
        sync_timer_elapsed32(postsleep_timer) >= WLS_POSTSLEEP_RESYNC_DELAY) {
        postsleep_timer = sync_timer_read32();

        if (++postsleep_count >= WLS_POSTSLEEP_RESYNC_COUNT) {
            postsleep_resync = false;
        }

#ifdef NKRO_ENABLE
        extern keymap_config_t keymap_config;
        if (keyboard_protocol && keymap_config.nkro) {
            nkro_report_dedup_invalidate();
        } else
#endif
        {
            keyboard_report_dedup_invalidate();
        }

        extern report_keyboard_t *keyboard_report;
#ifdef NKRO_ENABLE
        extern report_nkro_t *nkro_report;
        if (keyboard_protocol && keymap_config.nkro) {
            host_nkro_send(nkro_report);
        } else
#endif
        {
            host_keyboard_send(keyboard_report);
        }
    }

    /* Periodic keep-alive: re-send the current keyboard state to prevent
     * the 2.4GHz radio link and dongle USB from entering power-saving
     * modes.  This replaces the reactive idle-detection resync which
     * failed to reliably recover the first report after idle.
     *
     * Does NOT reset the input activity timer, so the normal 5-minute
     * sleep timeout still works as designed.
     */
    if (get_transport() == TRANSPORT_WLS &&
        *md_getp_state() == MD_STATE_CONNECTED &&
        lpwr_get_state() == LPWR_NORMAL &&
        sync_timer_elapsed32(keepalive_timer) >= WLS_KEEPALIVE_INTERVAL) {

        keepalive_timer = sync_timer_read32();

        extern report_keyboard_t *keyboard_report;
#ifdef NKRO_ENABLE
        extern report_nkro_t *nkro_report;
        extern keymap_config_t keymap_config;
        if (keyboard_protocol && keymap_config.nkro) {
            host_nkro_send(nkro_report);
        } else
#endif
        {
            host_keyboard_send(keyboard_report);
        }
    }

    /* usb_remote_wakeup() should be invoked last so that we have chance
     * to switch to wireless after start-up when usb is not connected
     */
    if (get_transport() == TRANSPORT_USB) {
        usb_remote_wakeup();
    } else if (lpwr_get_state() == LPWR_NORMAL) {
        static uint32_t inqtimer = 0x00;

        if (sync_timer_elapsed32(inqtimer) >= (WLS_INQUIRY_BAT_TIME)) {
            if (md_inquire_bat()) {
                inqtimer = sync_timer_read32();
            }
        }
    }
}

void housekeeping_task_kb(void) {

    wireless_task();
}