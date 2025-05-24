#include "viewer.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(viewer, LOG_LEVEL_INF);

#define RAW_LEN 32

static lv_obj_t *gesture_lbl = NULL;
static lv_obj_t *distance_lbl = NULL;

struct ble_display_data {
    int cmd;
    int int_part;
    int frac_part;
};
K_MSGQ_DEFINE(ble_ui_msgq, sizeof(struct ble_display_data), 10, 4);

static bool parse_ble_payload(char *payload, struct ble_display_data *d)
{
    char *p = payload + 3; // skip "B1:"
    char *tok;

    tok = strtok(p, ",");
    if (!tok) return false;
    d->cmd = atoi(tok);

    tok = strtok(NULL, ",");
    if (!tok || !strchr(tok, '.')) return false;

    char *dot = strchr(tok, '.');
    *dot = '\0';
    d->int_part = atoi(tok);
    d->frac_part = atoi(dot + 1);

    return true;
}

static bool ad_extract_and_parse(struct bt_data *data, void *user_data)
{
    char *mfg_buf = user_data;

    if (data->type == BT_DATA_MANUFACTURER_DATA && data->data_len > 2) {
        size_t len = data->data_len - 2;
        if (len >= RAW_LEN) len = RAW_LEN - 1;

        memcpy(mfg_buf, data->data + 2, len);
        mfg_buf[len] = '\0';

        if (strncmp(mfg_buf, "B1:", 3) == 0) {
            struct ble_display_data d;
            if (parse_ble_payload(mfg_buf, &d)) {
                d.cmd = CLAMP(d.cmd, 0, 3);
                k_msgq_put(&ble_ui_msgq, &d, K_NO_WAIT);
            } else {
                printk("[BLE] Malformed: %s\n", mfg_buf);
            }
        }
    }

    return true;
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
                    uint8_t type, struct net_buf_simple *ad)
{
    char mfg_buf[RAW_LEN] = {0};
    bt_data_parse(ad, ad_extract_and_parse, mfg_buf);
}

static void draw_static_ui(void)
{
    lv_obj_t *bg = lv_obj_create(lv_scr_act());
    lv_obj_set_size(bg, SCR_W, SCR_H);
    lv_obj_clear_flag(bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(bg, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(bg, LV_OPA_COVER, 0);

    // Gesture label (centered)
    gesture_lbl = lv_label_create(lv_scr_act());
    lv_obj_remove_style_all(gesture_lbl);
    lv_obj_set_style_text_color(gesture_lbl, lv_color_hex(0xFF3030), 0);
    lv_obj_set_style_text_font(gesture_lbl, &lv_font_montserrat_28, 0);
    lv_label_set_text(gesture_lbl, "Waiting...");
    lv_obj_align(gesture_lbl, LV_ALIGN_CENTER, 0, -10);

    distance_lbl = lv_label_create(lv_scr_act());
    lv_obj_remove_style_all(distance_lbl);
    lv_obj_set_style_text_color(distance_lbl, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_style_text_font(distance_lbl, &lv_font_montserrat_20, 0);
    lv_label_set_text(distance_lbl, "0.00m");
    lv_obj_align(distance_lbl, LV_ALIGN_CENTER, 40, 25);
}

static void setup_display_and_ble(void)
{
    const struct device *disp = DEVICE_DT_GET(ILI_NODE);
    if (!device_is_ready(disp)) {
        printk("Display not ready!\n");
        return;
    }

    lv_init();
    display_blanking_off(disp);
    init_styles();
    draw_static_ui();

    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("bt_enable: %d", err);
        return;
    }

    static const struct bt_le_scan_param scan_param = {
            .type     = BT_HCI_LE_SCAN_ACTIVE,
            .options  = BT_LE_SCAN_OPT_NONE,
            .interval = 0x0008,
            .window   = 0x0008,
    };

    err = bt_le_scan_start(&scan_param, scan_cb);
    if (err) LOG_ERR("bt_le_scan_start: %d", err);
    else     LOG_INF("BLE scan started");
}

static void ui_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    setup_display_and_ble();

    while (1) {
        struct ble_display_data d;
        if (k_msgq_get(&ble_ui_msgq, &d, K_NO_WAIT) == 0) {
            const char *text = NULL;
            switch (d.cmd) {
                case 0: text = "Go Forward"; break;
                case 1: text = "Stop";       break;
                case 2: text = "Turn Left";  break;
                case 3: text = "Turn Right"; break;
                default: text = "None";      break;
            }

            char dist_buf[16];
            snprintf(dist_buf, sizeof(dist_buf), "%d.%02dm", d.int_part, d.frac_part);

            lv_label_set_text(gesture_lbl, text);
            lv_label_set_text(distance_lbl, dist_buf);

            printk("[Gesture] %s (%s)\n", text, dist_buf);
        }

        lv_timer_handler();
        k_sleep(K_MSEC(30));
    }
}

K_THREAD_STACK_DEFINE(ui_stack_area, 8192);
static struct k_thread ui_thread_data;

void start_ui_thread(void)
{
    k_thread_create(&ui_thread_data,
                    ui_stack_area,
                    K_THREAD_STACK_SIZEOF(ui_stack_area),
                    ui_thread,
                    NULL, NULL, NULL,
                    4, 0, K_NO_WAIT);
}
