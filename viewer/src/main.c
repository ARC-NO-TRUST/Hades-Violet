#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "viewer.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
    LOG_INF("Starting viewer application...");

    start_ui_thread();

    while (1) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
