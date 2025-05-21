#ifndef ULT_H
#define ULT_H

#include <zephyr/kernel.h>

#define MSGQ_STRLEN 16
#define MSGQ_SIZE 5

extern struct k_msgq json_msgq;

void start_ultrasonic_thread();
void start_bluetooth_thread(void);

#endif // ULT_H
