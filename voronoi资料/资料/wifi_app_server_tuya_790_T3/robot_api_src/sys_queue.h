#ifndef __SYS_QUEUE_H__
#define __SYS_QUEUE_H__

typedef struct queue
{
    int     header;
    int     tail;
    int     size;
    int     capcity;
    void    **_buf;
    
} queue_t;

// API
queue_t *sys_queue_create(int size);
void sys_queue_free(queue_t *q);

int sys_queue_is_full(queue_t *q);
int sys_queue_is_empty(queue_t *q);

void sys_queue_push_tail(queue_t *q, void *data);
void *sys_queue_pop_head(queue_t *q);
void sys_queue_clear(queue_t *q);
#endif // !__SYS_QUEUE_H__