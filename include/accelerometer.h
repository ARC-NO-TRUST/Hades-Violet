#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <stdbool.h>

struct accel_msg {
  bool moving;
};

int accelerometer_init(void);

// Message queue to fusion thread
extern struct k_mutex acce_mutex;

#endif // ACCELEROMETER_H
