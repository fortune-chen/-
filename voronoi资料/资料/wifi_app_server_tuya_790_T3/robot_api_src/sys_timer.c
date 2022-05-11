#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <signal.h>

#include "sys_timer.h"

#if 0
#define log(...)        printf("[TIMER] " __VA_ARGS__)
#else
#define log(...)
#endif


static void sys_timer_notify_function(sigval_t val)
{
    int i;
    sys_timer_t *sys_timer = (sys_timer_t*)val.sival_ptr;

    log("sys_timer_notify_function: sigval = 0x%X\n", sys_timer);

    if (sys_timer && sys_timer->callback_thread)
    {
        sys_timer->callback_thread();
    }
}

sys_timer_t* sys_timer_create(int period_ms, SYS_TIMER_FUNC callback)
{
    sys_timer_t     *sys_timer;
    timer_t         *timer;
    struct sigevent sev;

    // 参数判断
    if (!callback)
    {
        return NULL;
    }

    // 申请系统定时器
    sys_timer = (sys_timer_t*)malloc(sizeof(sys_timer_t));
    if (!sys_timer)
    {
        log("malloc ERROR\n");
        return NULL;
    }

    // 申请Posix Timer
    timer = (timer_t*)malloc(sizeof(timer_t));
    if (!timer)
    {
        log("malloc ERROR\n");
        free(sys_timer);
        return NULL;
    }

    // 创建Posix Timer
    memset(&sev, 0, sizeof(struct sigevent));
    sev.sigev_signo             = SIGRTMIN;
    sev.sigev_notify            = SIGEV_THREAD;
    sev.sigev_value.sival_ptr   = (void*)sys_timer;
    sev.sigev_notify_function   = sys_timer_notify_function;
    sev.sigev_notify_attributes = NULL;
  
    if (timer_create(CLOCK_REALTIME, &sev, timer) == -1)  
    {
        log("timer_create ERROR\n");
        free(timer);
        free(sys_timer);
        return NULL;  
    }

    // 系统定时器创建成功，加入定时器列表
    sys_timer->start           = 0;
    sys_timer->period          = period_ms;
    sys_timer->timer_handle    = timer;
    sys_timer->callback_thread = callback;

    return sys_timer;
}

int sys_timer_start(sys_timer_t *sys_timer)
{
    if (!sys_timer)
    {
        log("sys_timer is NULL\n");
        return -1;
    }

    struct itimerspec its;
    int timeout = sys_timer->period;
 
    // 启动定时器
    its.it_value.tv_sec     = timeout / 1000;
    its.it_value.tv_nsec    = (timeout % 1000) * 1000000;
 
    its.it_interval.tv_sec  = 0;
    its.it_interval.tv_nsec = 0;
 
    if (timer_settime(*((timer_t*)sys_timer->timer_handle), 0, &its, NULL) == -1)
    {
        log("timer_settime ERROR\n");
        return -1;
    }

    sys_timer->start = 1;

    return 0;
}

int sys_timer_start_with_period(sys_timer_t *sys_timer, int period, int repeat_period)
{
    if (!sys_timer)
    {
        log("sys_timer is NULL\n");
        return -1;
    }

    struct itimerspec its;
    int timeout = 0;
    int repeat_timeout = 0;
    
    if(period > 0)
    {
        timeout = period;
    }
    else
    {
        timeout = sys_timer->period;
    }

    if(repeat_period > 0)
    {
        repeat_timeout = repeat_period;
    }
    else
    {
        repeat_timeout = 0;
    }
    
 
    // 启动定时器
    its.it_value.tv_sec     = timeout / 1000;
    its.it_value.tv_nsec    = (timeout % 1000) * 1000000;
 
    its.it_interval.tv_sec  = repeat_timeout / 1000;;
    its.it_interval.tv_nsec = (repeat_timeout % 1000) * 1000000;
 
    if (timer_settime(*((timer_t*)sys_timer->timer_handle), 0, &its, NULL) == -1)
    {
        log("timer_settime ERROR\n");
        return -1;
    }

    sys_timer->start = 1;

    return 0;
}

int sys_timer_start_repeat(sys_timer_t *sys_timer)
{
    if (!sys_timer)
    {
        log("sys_timer is NULL\n");
        return -1;
    }

    struct itimerspec its;
    int timeout = sys_timer->period;
 
    // 启动定时器
    its.it_value.tv_sec     = timeout / 1000;
    its.it_value.tv_nsec    = (timeout % 1000) * 1000000;
 
    its.it_interval.tv_sec  = timeout / 1000;
    its.it_interval.tv_nsec = (timeout % 1000) * 1000000;
 
    if (timer_settime(*((timer_t*)sys_timer->timer_handle), 0, &its, NULL) == -1)
    {
        log("timer_settime ERROR\n");
        return -1;
    }

    sys_timer->start = 1;

    return 0;
}

int sys_timer_stop(sys_timer_t *sys_timer)
{
    if (!sys_timer)
    {
        log("sys_timer is NULL\n");
        return -1;
    }

    struct itimerspec its;
 
    // 停止定时器
    its.it_value.tv_sec     = 0;
    its.it_value.tv_nsec    = 0;
 
    its.it_interval.tv_sec  = 0;
    its.it_interval.tv_nsec = 0;
 
    if (timer_settime(*((timer_t*)sys_timer->timer_handle), 0, &its, NULL) == -1)
    {
        log("timer_settime ERROR\n");
        return -1;
    }

    sys_timer->start = 0;

    return 0;
}

int sys_timer_delete(sys_timer_t *sys_timer)
{
    if (!sys_timer)
    {
        printf("sys_timer is NULL\n");
        return -1;
    }

    if (timer_delete(*((timer_t*)sys_timer->timer_handle)) == -1)
    {
        return -1;
    }

    free(sys_timer->timer_handle);
    free(sys_timer);

    //add by cmw
    sys_timer->timer_handle = NULL;
    sys_timer = NULL;
    
    return 0;
}