#include <stdio.h>
#include <stdlib.h>

#include "sys_queue.h"

#define QUEUE_DEFAULT_SIZE  1024


queue_t *sys_queue_create(int size)
{
    queue_t *q = (queue_t*)malloc(sizeof(queue_t));
    if (q != NULL)
    {
        if (size > 0)
        {
            q->_buf = (void**)malloc(size * sizeof(void*));
            q->capcity = size;
        }
        else
        {
            q->_buf = (void**)malloc(QUEUE_DEFAULT_SIZE * sizeof(void*));
            q->capcity = QUEUE_DEFAULT_SIZE;
        }
        q->header = q->tail = q->size = 0;
    }

    return q;
}

int sys_queue_is_full(queue_t *q)
{
    return q->size == q->capcity;
}

int sys_queue_is_empty(queue_t *q)
{
    return q->size == 0;
}

void sys_queue_push_tail(queue_t *q, void *data)
{
    if (!sys_queue_is_full(q))
    {
        q->_buf[q->tail] = data;
        q->tail = (q->tail + 1) % q->capcity;
        q->size++;
    }
}

void *sys_queue_pop_head(queue_t *q)
{
    void *data = NULL;
    if (!sys_queue_is_empty(q))
    {
        data = q->_buf[(q->header)];
        q->header = (q->header + 1) % q->capcity;
        q->size--;
    }

    return data;
}

void sys_queue_clear(queue_t *q)
{
    void *data = NULL;
    while (q->size)
    {
        data = q->_buf[(q->header)];
        if (data)
        {
            free(data);
        }
        q->header = (q->header + 1) % q->capcity;
        q->size--;
    }
}


void sys_queue_free(queue_t *q)
{
    free(q->_buf);
    free(q);
}