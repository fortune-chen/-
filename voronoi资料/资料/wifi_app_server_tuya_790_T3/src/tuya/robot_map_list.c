#include  <stdio.h>
#include <stdint.h>
#include <stdlib.h> 
#include <string.h>
#include <unistd.h>
#include "robot_map_list.h"


MAP_ID_PACK  *g_p_map_head = NULL;   /* 房间名字链表表头 */
MAP_ID_PACK  map_id_pack;

/* 创建单个链表节点 */
static MAP_ID_PACK* creat_node(void)
{
    MAP_ID_PACK *p = NULL;

	p = (MAP_ID_PACK*)malloc (sizeof(MAP_ID_PACK));

	if(p==NULL) 
	{
		printf("malloc node failed\n");
		return NULL;
	}
	p  -> robot_map_id = -1;
    p -> tuya_map_id = -1;
    p -> pnext = NULL;
    p -> plast = NULL;

    printf("creat node successfully\n");

	return p;
}

/* 删除单个链表节点 */
void  delete_node(MAP_ID_PACK *p_delete)
{
    MAP_ID_PACK *p = p_delete;

    p ->plast->pnext = p->pnext;

    if(p ->pnext != NULL)
    {
        p ->pnext->plast = p ->plast;
    }
    printf("delete node\n");

    free(p);
}

/* 增加地图链表节点 */
uint8_t add_map_list_node(MAP_ID_PACK*phead,int robot_map_id,int tuya_map_id)
{
	MAP_ID_PACK*p = phead,*pnew = NULL;

	while(p->pnext)
	{
        if(p->pnext->robot_map_id == robot_map_id)
        {
            printf("not need add map node,robot_map_id = %d\n",robot_map_id);
            return 0;
        }
		p = p->pnext;
	
	}
    pnew = creat_node();

    if(pnew == NULL)
    {
        printf("creat node failed\n");
        return 0;
    }

    pnew ->robot_map_id = robot_map_id;
    pnew ->tuya_map_id=tuya_map_id;

    printf("add new map: robot_map_id = %d, tuya_map_id = %d\n",pnew ->robot_map_id,pnew ->tuya_map_id);

	p->pnext=pnew;
    pnew->plast = p;

    return 1;
}

/* 创建链表 */
static void creat_map_list(MAP_ID_PACK**phead,uint8_t list_num)
{
    MAP_ID_PACK *pnew = NULL,*ptail = NULL,*pcur = NULL; 
    uint8_t list_num_temp = list_num;

    if(*phead != NULL)   /* 若存在表头，则代表链表已经存在，不需要创建 */
    {
        printf("already have map list\n");
        return;
    }
    *phead = creat_node(); /* 创建表头 */

    pcur = *phead;

    while(list_num_temp)
    {
        pnew = creat_node();      
        pcur -> pnext = pnew;
        pnew->plast = pcur;
        pcur = pnew;
        ptail = pnew;
        list_num_temp --;
    }
    
}

/* 遍历链表 */
void search_map_list(MAP_ID_PACK*  phead)
{
	MAP_ID_PACK *p = phead;

	p=phead->pnext;	

	while(p)
	{
        printf("robot_map_id = %d,tuya_map_id = %d,p->pnext = %p,p->plast = %p ,p->plast->pnext =%p\n",p->robot_map_id,p->tuya_map_id,p->pnext,p->plast,p->plast->pnext);
		p=p->pnext;	
	}
    printf("map list num = %d\n",get_map_list_num(phead));
}

/* 删除指定tuya_map_id的节点 */
void delete_map_node(MAP_ID_PACK*phead,int robot_map_id,int tuya_map_id)
{
    MAP_ID_PACK*p = phead,*pdelet = NULL;;
	p=phead->pnext;
    
    if(robot_map_id != -1)   /*按机器mapid查找节点 */
    {
        while(p)
        {
            if(p->robot_map_id == robot_map_id)
            {
                printf("delete_map_node,robot_map_id = %d\n",p->robot_map_id);
                delete_node(p); 
                break;    
            }
            else
            {
                p=p->pnext;		
            }
        }
        return;
    }
    if(tuya_map_id != -1)                    /*按涂鸦mapid查找节点 */
    {
        while(p)
        {
            if(p->tuya_map_id == tuya_map_id)
            {
                printf("delete_map_node,tuya_map_id = %d\n",p->tuya_map_id);
                delete_node(p); 
                break;    
            }
            else
            {
                p=p->pnext;		
            }
        }
        return;
    }

}


/* 获取指定robot_map_id */
int get_robot_map_id(MAP_ID_PACK*  phead, uint8_t  tuya_map_id)
{
    MAP_ID_PACK *p = phead;

    p = phead->pnext;

    while(p)           /* 先检索是否存在该房间的节点 */
    {
        if(p ->tuya_map_id == tuya_map_id)
        {
            return p->robot_map_id;
        }
        p = p -> pnext;
    }
    return -1;
}

/* 获取指定tuya_map_id */
int get_tuya_map_id(MAP_ID_PACK*  phead, uint8_t  robot_map_id)
{
    MAP_ID_PACK *p = phead;

    p = phead->pnext;

    while(p)           /* 先检索是否存在该房间的节点 */
    {
        if(p ->robot_map_id == robot_map_id)
        {
            return p->tuya_map_id;
        }
        p = p -> pnext;
    }
    return -1;
}

/* 删除整条链表( 除表头 )*/
void delete_name_list(MAP_ID_PACK*phead)
{
    MAP_ID_PACK*p = phead ->pnext , *cur_p;

    while (p)
	{  
       delete_node(p); 
       p = phead ->pnext;
	}

	printf("delete map list\n");
}

/* 保存房间名字到文件 */
void save_robot_map_to_file(MAP_ID_PACK *p_head)
{
    FILE* fp = NULL;
    MAP_ID_PACK *p = p_head;

    p = p -> pnext;

    fp = fopen(MAP_LIST_FILE_PATH,"w+");
    if(fp == NULL)
    {
        printf("fopen room file failed\n");
        return;
    }
    while(p)
    {
         fwrite(p, sizeof(MAP_ID_PACK),1,fp);
         p = p->pnext;
    }
    fclose(fp);
   
}

/* 从文件中读取链表 */
void read_robot_map_by_file(MAP_ID_PACK *p_head)
{
    FILE *fp;
    MAP_ID_PACK * p = NULL, * cur_p = p_head;
    uint8_t map_num = 0;

    delete_name_list(g_p_map_head); /* 读取链表之前，先把本地链表删除，避免创造多个链表，造成内存泄漏 */

    fp = fopen(MAP_LIST_FILE_PATH,"r");
    if(fp==NULL)
    {
        printf("read map list  failed \n");
        return;
    } 
    p = creat_node();
    while(fread((char *)p,sizeof(MAP_ID_PACK),1,fp) >  0)
    {
        cur_p->pnext = p;
        p->plast = cur_p;
        p->pnext = NULL;
        printf("robot_map_id = %d,tuya_map_id = %d\n",p->robot_map_id,p->tuya_map_id);
            
        if(feof(fp) != 0) /* 文件已读完 */
        {
            printf("read map list over\n");
            break;
        }
        cur_p = p ;
        p = creat_node();

    }
	fclose(fp);
    free(p);
	
 }

/* 获取链表长度 */
int get_map_list_num(MAP_ID_PACK*phead)
{
    MAP_ID_PACK * p = phead ->pnext;
    int num = 0;
    while(p)
    {
        num ++;
        p = p ->pnext;
    }
    return num;
   
}

/* 根据涂鸦地图列表，更新本地地图列表 */
void delete_node_by_tuya_map_list(M_MAP_INFO *buf,int len,MAP_ID_PACK*phead)
{
    MAP_ID_PACK *p = phead ->pnext,*p_delete = NULL;
    int temp = 0,i = 0;
    while(p != NULL)
    {
        for(i  = 0; i < len; i++)
        {
            if(p ->tuya_map_id == buf[i].map_id)
            {
                  temp = 1;
                  break;
            }
        }

        if(temp == 0)
        {
            p_delete = p;
            p = p->pnext;
            delete_map_node(g_p_map_head,-1,p_delete->tuya_map_id);
        }
        else
        {
            p = p->pnext;
        }
        temp = 0;
    }
}

/* 地图链表初始化 */
void map_list_init(void)
{
    creat_map_list(&g_p_map_head,0);
    read_robot_map_by_file(g_p_map_head); 
}