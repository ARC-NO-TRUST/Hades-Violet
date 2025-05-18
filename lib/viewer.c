#include "viewer.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/printk.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

LOG_MODULE_REGISTER(viewer, LOG_LEVEL_INF);

#define RAW_LEN 16
struct adv_data { char buf[RAW_LEN]; int8_t rssi; };
K_MSGQ_DEFINE(adv_msgq, sizeof(struct adv_data), 10, 4);

static lv_obj_t *direction_label = NULL;

static const struct bt_le_scan_param scan_param = {
        .type     = BT_HCI_LE_SCAN_ACTIVE,
        .options  = BT_LE_SCAN_OPT_NONE,
        .interval = 0x0010,
        .window   = 0x0010,
};

static bool ad_find_mfg(struct bt_data *data, void *user_data)
{
    char *m = user_data;
    if (data->type == BT_DATA_MANUFACTURER_DATA) {
        size_t len = MIN(data->data_len, RAW_LEN - 1);
        memcpy(m, data->data, len);
        m[len] = '\0';
        return false;
    }
    return true;
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
                    uint8_t type, struct net_buf_simple *ad)
{
    char raw[RAW_LEN] = {0};
    bt_data_parse(ad, ad_find_mfg, raw);
    if (raw[0]) {
        struct adv_data it;
        memcpy(it.buf, raw, RAW_LEN);
        it.rssi = rssi;
        k_msgq_put(&adv_msgq, &it, K_NO_WAIT);
    }
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

    lv_obj_t *bg = lv_obj_create(lv_scr_act());
    lv_obj_set_size(bg, SCR_W, SCR_H);
    lv_obj_clear_flag(bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(bg, lv_color_black(), 0);

    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("bt_enable: %d", err);
        return;
    }

    err = bt_le_scan_start(&scan_param, scan_cb);
    if (err) {
        LOG_ERR("bt_le_scan_start: %d", err);
    } else {
        LOG_INF("BLE scan started");
    }
}

static void ui_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    setup_display_and_ble();

    while (1) {
        struct adv_data item;
        if (k_msgq_get(&adv_msgq, &item, K_NO_WAIT) == 0) {
            if (strncmp(item.buf, "B1:", 3) == 0) {
              int cmd = -1;
              int int_part = 0;
              int frac_part = 0;

              if (sscanf(item.buf + 3, "%d,%d.%d", &cmd, &int_part, &frac_part) == 3) {
                const char *text = NULL;
                switch (cmd) {
                case 0: text = "GO";    break;
                case 1: text = "STOP";  break;
                case 2: text = "LEFT";  break;
                case 3: text = "RIGHT"; break;
                default:
                  LOG_INF("Unknown gesture code: %d", cmd);
                  continue;
                }

                char display_buf[64];
                snprintf(display_buf, sizeof(display_buf), "%s\n%d.%02dm", text, int_part, frac_part);

                // Clear previous label
                if (direction_label) {
                    lv_obj_del(direction_label);
                    direction_label = NULL;
                }

                // Create new label
                direction_label = lv_label_create(lv_scr_act());
                lv_label_set_text(direction_label, text);
                lv_obj_set_style_text_color(direction_label, lv_color_white(), 0);
                lv_obj_set_style_text_font(direction_label, &lv_font_montserrat_20, 0);
                lv_obj_center(direction_label);

                printk("Displayed command: %s\n", text);
            }
        }

        lv_timer_handler();
        k_sleep(K_MSEC(30));
    }
}

/* ───────── Thread bootstrap ───────── */
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
