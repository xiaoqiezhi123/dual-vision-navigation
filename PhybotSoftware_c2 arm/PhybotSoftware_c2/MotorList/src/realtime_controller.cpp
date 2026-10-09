#include "../include/realtime_controller.hpp"



// 比较两个 timespec：a > b 返回1，a == b 返回0，a < b 返回-1
static inline int timespec_cmp(const struct timespec *a, const struct timespec *b)
{
    if (a->tv_sec > b->tv_sec)
        return 1;
    if (a->tv_sec < b->tv_sec)
        return -1;
    if (a->tv_nsec > b->tv_nsec)
        return 1;
    if (a->tv_nsec < b->tv_nsec)
        return -1;
    return 0;
}

// 计算 a - b，返回差值（纳秒）
static inline long timespec_diff_ns(const struct timespec *a, const struct timespec *b)
{
    return (a->tv_sec - b->tv_sec) * 1000000000LL + (a->tv_nsec - b->tv_nsec);
}



void inc_period(struct period_info* pinfo) {
  pinfo->next_period.tv_nsec += pinfo->period_ns;

  while (pinfo->next_period.tv_nsec >= 1000000000) {
    /* timespec nsec overflow */
    pinfo->next_period.tv_sec++;
    pinfo->next_period.tv_nsec -= 1000000000;
  }
}

void periodic_task_init(struct period_info* pinfo, double control_period) {
  /* for simplicity, hardcoding a 1ms period */
  pinfo->period_ns = control_period *1000000000;

  clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}

void wait_rest_of_period(struct period_info* pinfo) {
    inc_period(pinfo);

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    // 任务超时：追赶对齐最近周期，防止永久滞后漂移
    if (timespec_cmp(&now, &pinfo->next_period) > 0) {
        long over_ns = timespec_diff_ns(&now, &pinfo->next_period);
        long skip = over_ns / pinfo->period_ns + 1;
        pinfo->next_period.tv_sec += skip * pinfo->period_ns / 1000000000LL;
        pinfo->next_period.tv_nsec += (skip * pinfo->period_ns) % 1000000000LL;
        // 重新做纳秒进位归一化
        while (pinfo->next_period.tv_nsec >= 1000000000) {
            pinfo->next_period.tv_sec++;
            pinfo->next_period.tv_nsec -= 1000000000;
        }
    }

    int ret;
    while((ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, NULL)) == EINTR);
}