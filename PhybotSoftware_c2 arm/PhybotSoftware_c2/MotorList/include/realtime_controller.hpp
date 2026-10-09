#include <ctime>
#include <cstdint>
#include <unistd.h>
#include <iostream>
#include <time.h>

struct period_info {
  struct timespec next_period;
  long period_ns;
};


void inc_period(struct period_info* pinfo);
void periodic_task_init(struct period_info* pinfo, double control_period);
void wait_rest_of_period(struct period_info* pinfo);