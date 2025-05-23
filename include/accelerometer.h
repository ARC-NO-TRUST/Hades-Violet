#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#define STACK_SIZE 1024
#define THREAD_PRIORITY 4
#define POLL_INTERVAL_MS 300
#define MOTION_THRESHOLD 0.8f

int accelerometer_init(void);

#endif // ACCELEROMETER_H
