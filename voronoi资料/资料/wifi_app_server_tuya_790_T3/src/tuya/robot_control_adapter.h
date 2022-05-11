#ifndef __ROBOT_CONTROL_ADAPTER_H__
#define __ROBOT_CONTROL_ADAPTER_H__

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "robot_api.h"


#define MAX_VIRTUAL_WALL  8
#define MAX_FORBIDDEN     8
#define MAX_DEFINED_RECTS 10

#define MAX_ROOM_SPEC     64


typedef struct {
    uint8_t   num;
    DATA_GOTO_POSITION pose;
} defined_pose_arg_t;

typedef struct {
    uint32_t    num;
    DATA_RECT   data[MAX_DEFINED_RECTS];
} defined_rects_arg_t;

typedef struct {
    uint32_t    num;
    DATA_VIRTUAL   data[MAX_VIRTUAL_WALL];
} virtual_wall_arg_t;

typedef struct {
    uint32_t     num;
    DATA_FOBIDEEN data[MAX_FORBIDDEN];
} forbidden_arg_t;

typedef struct {
   uint32_t num;  // 房间数量
   ROOM_SPEC_T room_spec[MAX_ROOM_SPEC]; // 房间属性
} room_spec_arg_t;

typedef struct{
    int room_id;      //房间id
    int order;        //清扫顺序
    int clean_count;  //清扫次数
    int rev;          //预留
}SelectedCleanRoomInfo;

typedef struct{
    uint32_t num;//房间个数
    SelectedCleanRoomInfo rooms[32];
}SelectedCleanRoom;

typedef enum {
    SCENE_AUTO_CLEAN = 0,
    SCENE_DOCK,
    SCENE_GOTO_POS,
    SCENE_SPOT_CLEAN,
    SCENE_RECT_CLEAN,
    SCENE_CHARGING,
    SCENE_WALLFOLLOW,
    SCENE_CHOICE_ROOMS_CLEAN,
} STM_SCENE_E;

typedef enum {
    SCENE_SWITCH_START = 0,
    SCENE_SWITCH_PAUSE,
    SCENE_SWITCH_RESUME,
    SCENE_SWITCH_STOP
} STM_SCENE_SWITCHER;

typedef enum
{
    ROOMS_INITIALIZE                = 0x00,//无障碍值，预留(app不需要显示)
    ROOMS_SPLIT_FAILED,                     //房间分割失败
    ROOMS_MERGER_FAILED,                  //房间合并失败
    MAP_INSTABILITY,                      //地图不稳定
    ROBOT_IS_CLEANING,                    //机器正在清扫
    ACTIVE_ROOM_SPLITTING_FAILED,         //主动分房间失败
    ROOMS_NOT_SET_BY_HUMAN,               //当前不是人为设置的房间信息 
    ROOMS_SET_FAILED,                     //房间设置失败
    ROOMS_SPLIT_SUCESS,                   //房间分割成功
    ROOMS_MERGER_SUCESS,                  //房间合并成功 
    CHIOCE_NOT_FOUND,                     //选区房间不存在
    ROOM_COUNT,                           //选区房间数量错误
    ROOM_CHOIOCE_SUCESS,                  //房间选区成功
    SEGMENT_SET_SUCCESS,                    //分割或合并的房间复位成功
    SEGMENT_SET_FAILED,                   // 恢复分区失败
}ROOM_CONTROL_STATE;


typedef struct
{
    int len;
    int id_data[32];
}CLEAN_ROOMS_ID;




typedef enum {
    GOTO_POSE = 0,
    RECT_CLEAN,
    SELECT_ROOM,
} STM_CLEAN_ARG;

char *robot_ctl_wait_robot_ready(void);
int robot_ctl_scene_switch(STM_SCENE_E scene, STM_SCENE_SWITCHER action);
int robot_ctl_exec_with_param(ROBOT_CMD_E cmd, void *param);
//缓存参数
defined_pose_arg_t *robot_ctl_get_goto_pos_arg(void);
defined_rects_arg_t *robot_ctl_get_rect_clean_arg(void);
virtual_wall_arg_t *robot_ctl_get_vwall_arg(void);
forbidden_arg_t *robot_ctl_get_forbid_arg(void);

room_spec_arg_t *robot_ctl_get_room_spec_arg(void);
SelectedCleanRoom *robot_ctl_get_select_room_arg(void);

void robot_ctl_reset_goto_pos_arg(void);
void robot_ctl_reset_rect_clean_arg(void);
void robot_ctl_reset_select_room_arg(void);
//地图重置消息
void robot_ctl_notify_map_reset(void);
//收到扫地机发送的虚拟墙时初始化
void robot_ctl_sync_vitual_wall(uint8_t *data);
// 往 tuya app 发送初始化虚拟墙信息
void upload_first_wirtual_wall_info(void);
void _reset_all_arg(void);
void _reset_tmp_arg(STM_CLEAN_ARG arg);
bool if_need_reset_tmp_arg(void);
void wifi_app_reset_navi();


void robot_ctl_sync_room_spec(uint8_t *data);
void robot_ctl_sync_edit_room_spec(uint8_t *data);


int rooms_segment_process(uint8_t *data);
int rooms_set_process(uint8_t *data);
int  map_split_process(uint8_t *data);
int  rooms_merger_process(uint8_t *data);
int  map_is_stable_process(uint8_t *data);
int  rooms_is_manual_process(uint8_t *data);
int rooms_clean_rooms_process(uint8_t *data);
void send_hoslam_save_room(void);
//void send_hoslam_set_constituency_cleaning(int id,int count,int order,int forbidden);


void set_room_msg(int len,int *id_data);
void clean_up_save_forbidden();
void set_choice_rooms_clean(uint8_t clean_count,uint8_t len,uint8_t *rooms_id);
void wait_wifi_load(void);

#endif /* __ROBOT_CONTROL_ADAPTER_H__ */
