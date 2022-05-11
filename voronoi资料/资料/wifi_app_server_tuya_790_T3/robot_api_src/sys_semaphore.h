#ifndef __SEMAPHORE_H__
#define __SEMAPHORE_H__


union semun
{
	int val;
	//struct semid_ds *buf;
	//unsigned short *array;
	//struct seminfo *__buf;
};


int sys_sem_create(key_t key, int n);
int sys_sem_get(key_t key, int n);
int sys_sem_delete(int id);
int sys_sem_init(int id, int index, int val);
void sys_sem_wait(int id, int index);
void sys_sem_signal(int id, int index);

#endif