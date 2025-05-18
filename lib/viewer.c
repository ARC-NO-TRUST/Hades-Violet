#include "viewer.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/printk.h>
#include <string.h>

LOG_MODULE_REGISTER(viewer, LOG_LEVEL_INF);

/* ───────── BLE advert queue ───────── */
#define RAW_LEN 16
struct adv_data { char buf[RAW_LEN]; int8_t rssi; };
K_MSGQ_DEFINE(adv_msgq, sizeof(struct adv_data), 10, 4);

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
static bool ad_find_mfg(struct bt_data *d, void *ud)
{
    char *m = ud;
    if (d->type == BT_DATA_MANUFACTURER_DATA) {
        size_t len = MIN(d->data_len, RAW_LEN - 1);
        memcpy(m, d->data, len);
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
    /* colours: green (0-2), yellow (3-4), red (5-7) */
    for (int i = 0; i < SEG_CNT; ++i) seg_set_style(arr[i], &st_off);

    for (int i = 0; i < count && i < SEG_CNT; ++i) {
        lv_style_t *st = (i < 3) ? &st_green :
                         (i < 5) ? &st_yellow : &st_red;
        seg_set_style(arr[i], st);
    }
}

/* ───────── BLE → UI update ───────── */
static void update_display(char code)
{
    /* reset both sides */
    for (int i = 0; i < SEG_CNT; ++i) {
        seg_set_style(left_seg [i], &st_off);
        seg_set_style(right_seg[i], &st_off);
    }

    switch (code) {
        case '0':   /* GO */
            lv_label_set_text(centre_lbl, "GO");
            light_segments(left_seg,  2);
            light_segments(right_seg, 2);
            break;

        case '1':   /* STOP */
            lv_label_set_text(centre_lbl, "STOP");
            light_segments(left_seg,  8);
            light_segments(right_seg, 8);
            break;

        case '2':   /* LEFT warning */
            lv_label_set_text(centre_lbl, "LEFT");
            light_segments(left_seg,  6);
            light_segments(right_seg, 2);
            break;

        case '3':   /* RIGHT warning */
            lv_label_set_text(centre_lbl, "RIGHT");
            light_segments(left_seg,  2);
            light_segments(right_seg, 6);
            break;

        default:
            lv_label_set_text(centre_lbl, "?");
            break;
    }

    lv_obj_align(centre_lbl, LV_ALIGN_CENTER, 0, 0);
}

/* ───────── Display + BLE init ───────── */
static void setup_display_and_ble(void)
{
    const struct device *disp = DEVICE_DT_GET(ILI_NODE);
    if (!device_is_ready(disp)) { printk("Display not ready!\n"); return; }

    lv_init();
    display_blanking_off(disp);
    init_styles();
    draw_static_ui();

    int err = bt_enable(NULL);
    if (err) { LOG_ERR("bt_enable: %d", err); return; }

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
        struct adv_data item;
        if (k_msgq_get(&adv_msgq, &item, K_NO_WAIT) == 0) {
            if (strncmp(item.buf, "B1:", 3) == 0) {
                char code = item.buf[3];
                update_display(code);
                printk("Display cmd %c\n", code);
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
