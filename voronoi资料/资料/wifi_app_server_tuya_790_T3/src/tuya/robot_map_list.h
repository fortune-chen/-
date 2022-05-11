#ifndef __ROBOT_MAP_LIST_H__
#define __ROBOT_MAP_LIST_H__

#include "robot_api.h"
#include "tuya_iot_sweeper_api.h"

#define MAP_LIST_FILE_PATH  "/root/app_server/map_list_file"
#define MAP_LIST_MAX_LEN  5

#pragma pack(1)

typedef struct map_id_pack {

     int robot_map_id;
     int tuya_map_id;
     struct map_id_pack  *pnext; 
     struct map_id_pack  *plast; 

}MAP_ID_PACK;

#pragma pack()

extern MAP_ID_PACK  *g_p_map_head;   /* 房间名字链表表头 */
MAP_ID_PACK map_id_pack;
void read_robot_map_by_file(MAP_ID_PACK *p_head);
int get_map_list_num(MAP_ID_PACK*phead);
void  delete_node(MAP_ID_PACK *p_delete);
void save_robot_map_to_file(MAP_ID_PACK *p_head);
void delete_name_list(MAP_ID_PACK*phead);
void delete_map_node(MAP_ID_PACK*phead,int robot_map_id,int tuya_map_id);
void search_map_list(MAP_ID_PACK*  phead);
static void creat_map_list(MAP_ID_PACK**phead,uint8_t list_num);
void map_list_init(void);
uint8_t add_map_list_node(MAP_ID_PACK*phead,int robot_map_id,int tuya_map_id);
int get_robot_map_id(MAP_ID_PACK*  phead, uint8_t  tuya_map_id);
int get_tuya_map_id(MAP_ID_PACK*  phead, uint8_t  robot_map_id);
void delete_node_by_tuya_map_list(M_MAP_INFO *buf,int len,MAP_ID_PACK*phead);
#endif