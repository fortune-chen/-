#ifndef __SYS_TIMER_H__ 
#define __SYS_TIMER_H__

typedef void (*SYS_TIMER_FUNC)(void);

typedef struct
{
    int             start;

    int             period;
    SYS_TIMER_FUNC  callback_thread;

    void            *timer_handle;

} sys_timer_t;  

// API
sys_timer_t* sys_timer_create(int period_ms, SYS_TIMER_FUNC callback);
int sys_timer_start(sys_timer_t *sys_timer);

int sys_timer_start_with_period(sys_timer_t *sys_timer, int period, int repeat_period);
int sys_timer_start_repeat(sys_timer_t *sys_timer);
int sys_timer_stop(sys_timer_t *sys_timer);
int sys_timer_delete(sys_timer_t *sys_timer);

#endif // !__SYS_TIMER_H__ 