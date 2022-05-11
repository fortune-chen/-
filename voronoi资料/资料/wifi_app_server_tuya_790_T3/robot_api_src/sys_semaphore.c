#include <stdio.h>
#include <sys/sem.h>

#include "sys_semaphore.h"

int sys_sem_create(key_t key, int n)
{
    int semid = -1;
    semid = semget(key, n, IPC_CREAT | IPC_EXCL | 0666);
    if (semid == -1)
    {
        perror("sem_creat");
    }

    return semid;
}

int sys_sem_get(key_t key, int n)
{
    int semid = -1;
    semid = semget(key, n, 0666);
    if (semid == -1)
    {
        perror("semget");
    }

    return semid;
}

int sys_sem_delete(int id)
{
    if (semctl(id, 0, IPC_RMID) == -1)
    {
        perror("semctl");
        return -1;
    }

    return 0;
}

int sys_sem_init(int id, int index, int val)
{
    union semun su;

    su.val = val;
    if (semctl(id, index, SETVAL, su) == -1)
    {
        perror("semctl");
        return -1;
    }

    return 0;
}

void sys_sem_wait(int id, int index)
{
    struct sembuf buf;

    buf.sem_num = index;
    buf.sem_op  = -1;       // P(sv) 减1 获取信号量
    //buf.sem_flg = SEM_UNDO;   // 导致Numerical result out of range错误
    buf.sem_flg = 0;

    if (semop(id, &buf, 1) == -1)
    {
        perror("semop");
    }
}

void sys_sem_signal(int id, int index)
{
    struct sembuf buf;

    buf.sem_num = index;
    buf.sem_op  = 1;        // V(sv) 加1 释放信号量
    //buf.sem_flg = SEM_UNDO;
    buf.sem_flg = 0;

    if (semop(id, &buf, 1) == -1)
    {
        perror("semop");
    }
}
