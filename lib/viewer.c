#include "viewer.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/printk.h>
#include <string.h>

LOG_MODULE_REGISTER(viewer, LOG_LEVEL_INF);

/* ───────── BLE advert queue ───────── */
#define RAW_LEN 32

/* ───────── UI handles ───────── */
#define SEG_CNT 8
static lv_obj_t *left_seg [SEG_CNT];
static lv_obj_t *right_seg[SEG_CNT];
static lv_obj_t *centre_lbl  = NULL;

/* basic styles (1-time init) */
static lv_style_t st_off, st_green, st_yellow, st_red;

/* ───────── BLE scan params ───────── */
static const struct bt_le_scan_param scan_param = {
        .type     = BT_HCI_LE_SCAN_ACTIVE,
        .options  = BT_LE_SCAN_OPT_NONE,
        .interval = 0x0010,
        .window   = 0x0010,
};

/* ───────── BLE advert helpers ───────── */
static bool ad_extract_rpi_msg(struct bt_data *data, void *user_data)
{
    char *mfg_buf = user_data;
    if (data->type == BT_DATA_MANUFACTURER_DATA) {
        size_t len = MIN(data->data_len, 30);
        // Skip the 2-byte manufacturer ID (e.g., 0xFFFF)
        if (data->data_len > 2) {
            memcpy(mfg_buf, data->data + 2, len - 2);
            mfg_buf[len - 2] = '\0';
        } else {
            mfg_buf[0] = '\0';  // Invalid manufacturer payload
        }
        return false;
    }
    return true;
}

/* ───────── UI helpers ───────── */
static void seg_set_style(lv_obj_t *seg, lv_style_t *st)
{
    lv_obj_remove_style_all(seg);
    lv_obj_add_style(seg, st, LV_PART_MAIN);
    lv_obj_add_style(seg, st, 0);
}

static void draw_static_ui(void)
{
    /* background */
    lv_obj_t *bg = lv_obj_create(lv_scr_act());
    lv_obj_set_size(bg, SCR_W, SCR_H);
    lv_obj_clear_flag(bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(bg, lv_color_black(), 0);

    /* centre label */
    centre_lbl = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(centre_lbl, lv_color_hex(0xFF3030), 0);
    lv_obj_set_style_text_font(centre_lbl, &lv_font_montserrat_28, 0);
    lv_label_set_text(centre_lbl, "0.0 m");
    lv_obj_align(centre_lbl, LV_ALIGN_CENTER, 0, 0);

    /* segment blocks */
    int seg_w = 10, seg_h = 22, gap = 2;
    int start_x_left  = (SCR_W/2) - gap - seg_w;
    int start_x_right = (SCR_W/2) + gap;

    for (int i = 0; i < SEG_CNT; ++i) {
        /* left side – draw outward from centre */
        left_seg[i] = lv_obj_create(lv_scr_act());
        lv_obj_set_size(left_seg[i], seg_w, seg_h);
        int x = start_x_left - i*(seg_w+gap);
        lv_obj_align(left_seg[i], LV_ALIGN_CENTER, -((SCR_W/2)-x), 0);

        /* right side – draw outward */
        right_seg[i] = lv_obj_create(lv_scr_act());
        lv_obj_set_size(right_seg[i], seg_w, seg_h);
        x = start_x_right + i*(seg_w+gap);
        lv_obj_align(right_seg[i], LV_ALIGN_CENTER, ((x-(SCR_W/2))), 0);

        seg_set_style(left_seg [i], &st_off);
        seg_set_style(right_seg[i], &st_off);
    }

    /* L / R letters */
    lv_obj_t *lblL = lv_label_create(lv_scr_act());
    lv_label_set_text(lblL, "L");
    lv_obj_set_style_text_color(lblL, lv_color_white(), 0);
    lv_obj_align(lblL, LV_ALIGN_LEFT_MID, 4, 0);

    lv_obj_t *lblR = lv_label_create(lv_scr_act());
    lv_label_set_text(lblR, "R");
    lv_obj_set_style_text_color(lblR, lv_color_white(), 0);
    lv_obj_align(lblR, LV_ALIGN_RIGHT_MID, -4, 0);
}

static void init_styles(void)
{
    lv_style_init(&st_off);
    lv_style_set_bg_color(&st_off, lv_color_black());
    lv_style_set_border_width(&st_off, 0);

    lv_style_init(&st_green);
    lv_style_set_bg_color(&st_green, lv_color_hex(0x00FF00));
    lv_style_set_border_width(&st_green, 0);

    lv_style_init(&st_yellow);
    lv_style_set_bg_color(&st_yellow, lv_color_hex(0xFFFF00));
    lv_style_set_border_width(&st_yellow, 0);

    lv_style_init(&st_red);
    lv_style_set_bg_color(&st_red, lv_color_hex(0xFF0000));
    lv_style_set_border_width(&st_red, 0);
}

static void light_segments(lv_obj_t **arr, int count)
{
    for (int i = 0; i < SEG_CNT; ++i) seg_set_style(arr[i], &st_off);

    for (int i = 0; i < count && i < SEG_CNT; ++i) {
        lv_style_t *st = (i < 3) ? &st_green :
                         (i < 5) ? &st_yellow : &st_red;
        seg_set_style(arr[i], st);
    }
}

struct ble_display_data {
    int cmd;
    int int_part;
    int frac_part;
};
K_MSGQ_DEFINE(ble_ui_msgq, sizeof(struct ble_display_data), 10, 4);


static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
                    uint8_t type, struct net_buf_simple *ad)
{
    char mfg_buf[RAW_LEN] = {0};
    bt_data_parse(ad, ad_extract_rpi_msg, mfg_buf);

    if (strncmp(mfg_buf, "B1:", 3) == 0) {
        printk("HIIIII1\n");

        struct ble_display_data d;
        char cam_pan_buf[5] = {0};
        char cam_tilt_buf[5] = {0};

        if (sscanf(mfg_buf + 3, "%d,%d.%d,%4s,%4s",
                   &d.cmd, &d.int_part, &d.frac_part,
                   cam_pan_buf, cam_tilt_buf) == 5) {
            printk("BLE parsed: cmd=%d, dist=%d.%02d\n", d.cmd, d.int_part, d.frac_part);
            k_msgq_put(&ble_ui_msgq, &d, K_NO_WAIT);
        }
    }
}

/* ───────── Display + BLE init ───────── */
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

    err = bt_le_scan_start(&scan_param, scan_cb);
    if (err) LOG_ERR("bt_le_scan_start: %d", err);
    else     LOG_INF("BLE scan started");
}

/* ───────── Main UI thread ───────── */
static void ui_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    setup_display_and_ble();

    while (1) {
        struct ble_display_data d;
        if (k_msgq_get(&ble_ui_msgq, &d, K_NO_WAIT) == 0) {
            const char *text = NULL;
            switch (d.cmd) {
                case 0: text = "GO";    break;
                case 1: text = "STOP";  break;
                case 2: text = "LEFT";  break;
                case 3: text = "RIGHT"; break;
                default: text = "NONE";  break;
            }

            char display_buf[64];
            snprintf(display_buf, sizeof(display_buf), "%s\n%d.%02dm", text, d.int_part, d.frac_part);
            lv_label_set_text(centre_lbl, display_buf);
            lv_obj_align(centre_lbl, LV_ALIGN_CENTER, 0, 0);

            light_segments(left_seg,  (d.cmd == 1) ? 8 : (d.cmd == 2) ? 6 : (d.cmd == 3) ? 2 : 2);
            light_segments(right_seg, (d.cmd == 1) ? 8 : (d.cmd == 2) ? 2 : (d.cmd == 3) ? 6 : 2);

            printk("Displayed command: %s\n", text);
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
