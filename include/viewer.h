#ifndef VIEWER_H
#define VIEWER_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/display.h>
#include <lvgl.h>

#define ILI_NODE  DT_NODELABEL(ili9342c)
#define SCR_W     320
#define SCR_H     240

void start_ui_thread(void);

#endif /* VIEWER_H */
