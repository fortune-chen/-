#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <arpa/inet.h> 
#include <math.h>
#include "tuya_dp_data_mapping.h"
#include "tuya_dp_enum.h"
#include "tuya_hal_wifi.h"
#include "tuya_iot_msg_parse.h"
#include "tuya_iot_uploader.h"
#include "tuya_iot_com_api.h"
#include "tuya_iot_sweeper_api.h"
#include "tuya_cloud_types.h"
#include "uni_log.h"
#include "../include/sys_api.h"
#include "robot_app_msg.h"
#include "tuya_iot_uploader.h"
#include "tuya_iot_state_adapter.h"
#include "robot_sys_msg.h"
#include "robot_control_adapter.h"
#include "robot_map_list.h"
extern char time_save[32];

void wifi_app_reset_navi();
void wifi_app_reset_path();
extern char Hex2Char(uint8_t vdata);

bool s_task_sw = false;            // 清扫任务开关 本地缓存
//bool s_send_his = false;            //是否上传清扫记录
int s_mode_tuya;

static STM_SCENE_E m_cur_scene;

extern pthread_mutex_t mtx_mapheart;
extern uint8_t getmapheart;  
static int g_force_update_state = 0;
static bool g_change_mode_flag = 0;
static unsigned int map_id;
static uint8_t segment_state = 0; /* 此变量用来存储当前机器是否处于恢复分区的状态  */
static bool type_clean_zone_flag = false;
static url_md5_voiceID pstru; /* 下载语音包结构体 */
bool get_editing_room_flag = false;

extern bool isAppointmentCmd(void);
extern int select_voice_path(DP_VOICE_CLASS value);
extern int find_voice_path(char * path_name);
extern int get_fan_mode_and_water_level(int type);
extern int set_custom_mode_switch(bool flag);

//////////////////////////////////////////////////////////////////////////
unsigned char get_check_sum(unsigned char *pack, unsigned short pack_len)
{
    unsigned short i;
    unsigned char check_sum = 0;

    for(i = 0; i < pack_len; i ++)
    {
      check_sum += *pack ++;
    }

    return check_sum;
}

static int get_goto_position(BYTE_T **databuf)
{

    DATA_POSITION *ppose = NULL;
    defined_pose_arg_t *p_arg = robot_ctl_get_goto_pos_arg();
    uint8_t pose_num = 0;

    printf("(p_arg->num = %d)\n", p_arg->num);
    
    if(p_arg->num == 1 && m_cur_scene == SCENE_GOTO_POS)
    {
        ppose = (DATA_POSITION *)&(p_arg->pose);
        pose_num = p_arg->num;
    }
    
    uint8_t head = RAW_APP_SEND_HEAD;
    uint8_t verson = RAW_APP_SEND_VERSION;
    uint8_t cmd = 0x17;
    uint8_t len = sizeof(cmd) + pose_num * 2 * 2; /* 一个点四个字节 */
    uint8_t chk = 0;
    uint32_t size = len + sizeof(verson) + sizeof(head) + sizeof(len) + sizeof(chk);
    uint8_t j = 0;

    BYTE_T* pbuf = (BYTE_T*)malloc(size);
    if(pbuf == NULL) {
        printf("get_goto_position %d malloc failed\n", size);
        return -1;
    }
    memset(pbuf, 0, size);
    float data;

    pbuf[j ++] = head;
    pbuf[j ++] = verson;
    pbuf[j ++] = len;
    pbuf[j ++] = cmd;
    if (ppose != NULL)
    {
        data = ppose->x * 10;
        pbuf[j ++] = (int16_t)data >> 8;
        pbuf[j ++] = (int16_t)data & 0x00FF;

        data = ppose->y * 10;
        pbuf[j ++] = (int16_t)data >> 8;
        pbuf[j ++] = (int16_t)data & 0x00FF;
    }

    chk = get_check_sum(&pbuf[3], pbuf[2]);
    pbuf[size - 1] = chk;

    (*databuf) = pbuf;

    return size;
}




static int get_clean_zone(BYTE_T **databuf)
{
     // 协议内容 head | len | cmd | num | x0 | y0 | x1 | y1 | x2 | y2 | x3 | y3 | ...
     //         U8   | U8 | U8 | U8 | U16 | U16 | ...

    defined_rects_arg_t *rect_arg = robot_ctl_get_rect_clean_arg();
    DATA_RECT *rect = rect_arg->data;

    uint8_t head = RAW_APP_SEND_HEAD;
    uint8_t version = RAW_APP_SEND_VERSION;
    uint8_t cmd = 0x29;
    uint8_t clean_count = 1;
    uint8_t num = rect_arg->num;
    uint8_t len = sizeof(cmd) + sizeof(clean_count) + sizeof(num) + 17 * num;/* 一个划区有17个字节 */
    uint8_t chk = 0;
    uint32_t size = len + sizeof(head) + sizeof(version) + sizeof(len) + sizeof(chk);
    uint8_t j = 0;
    BYTE_T* pbuf = (BYTE_T*)malloc(size);
    if(pbuf == NULL) {
        printf("get_clean_zone %d malloc failed\n", size);
        return -1;
    }
    memset(pbuf, 0, size);

    pbuf[j ++] = head;
    pbuf[j ++] = version;
    pbuf[j ++] = len;
    pbuf[j ++] = cmd;
    pbuf[j ++] = clean_count;
    pbuf[j ++] = num;

    float data;
    for(int i = 0; i < num; i++) //zone 只有2个点，再补2个
    {
        data = rect[i].x0 * 10;
        pbuf[j + 16*i]   = 4;  /* 固定为4 */
        pbuf[j + 16*i+1]   = ((int16_t)data) >> 8;
        pbuf[j + 16*i+2] = ((int16_t)data) & 0x00FF;
        data = rect[i].y0 *10;
        pbuf[j + 16*i+3] = ((int16_t)data) >> 8;
        pbuf[j + 16*i+4] = ((int16_t)data) & 0x00FF;

        data = rect[i].x1 * 10;
        pbuf[j + 16*i+5] = ((int16_t)data) >> 8;
        pbuf[j + 16*i+6] = ((int16_t)data) & 0x00FF;
        data = rect[i].y0 * 10;
        pbuf[j + 16*i+7] = ((int16_t)data) >> 8;
        pbuf[j + 16*i+8] = ((int16_t)data) & 0x00FF;

        data = rect[i].x1 * 10;
        pbuf[j + 16*i+9] = ((int16_t)data) >> 8;
        pbuf[j + 16*i+10] = ((int16_t)data) & 0x00FF;
        data = rect[i].y1 * 10;
        pbuf[j + 16*i+11] = ((int16_t)data) >> 8;
        pbuf[j + 16*i+12] = ((int16_t)data) & 0x00FF;

        data = rect[i].x0 * 10;
        pbuf[j + 16*i+13] = ((int16_t)data) >> 8;
        pbuf[j + 16*i+14] = ((int16_t)data) & 0x00FF;
        data = rect[i].y1 * 10;
        pbuf[j + 16*i+15] = ((int16_t)data) >> 8;
        pbuf[j + 16*i+16] = ((int16_t)data) & 0x00FF;
        
        j ++;
        printf("zone no %d, x0 %.2f, y0 %.2f, x1 %.2f, y1 %.2f, x2 %.2f, y2 %.2f, x3 %.2f, y3 %.2f \n", i,
               rect[i].x0, rect[i].y0, rect[i].x1, rect[i].y0, rect[i].x1, rect[i].y1, rect[i].x0, rect[i].y1);
    }

    chk = get_check_sum(&pbuf[3], pbuf[2]);
    pbuf[size - 1] = chk;

    (*databuf) = pbuf;

    return size;
}

static int get_select_room_inf(BYTE_T **databuf)
{
    SelectedCleanRoom *p_select_room_arg = robot_ctl_get_select_room_arg();

    uint8_t head = RAW_APP_SEND_HEAD;
    uint8_t version = RAW_APP_SEND_VERSION;
    uint8_t cmd = 0x15;
    uint8_t clean_count = p_select_room_arg->rooms[0].clean_count;
    uint8_t num = p_select_room_arg->num;
    uint8_t len = sizeof(cmd) + sizeof(clean_count) + sizeof(num) +  num;
    uint8_t chk = 0;
    uint32_t size = len + sizeof(head) + sizeof(version) + sizeof(len) + sizeof(chk);
    uint8_t j = 0;
    
    BYTE_T* pbuf = (BYTE_T*)malloc(size);

    if(pbuf == NULL) {
        size = 0;
        printf("get_clean_zone %d malloc failed\n", size);
        return -1;
    }
    memset(pbuf, 0, size);

    pbuf[j ++] = head;
    pbuf[j ++] = version;
    pbuf[j ++] = len;
    pbuf[j ++] = cmd;
    pbuf[j ++] = clean_count;
    pbuf[j ++] = num;

    for(int i = 0; i < num; i++) 
    {
        pbuf[j ++] = p_select_room_arg->rooms[i].room_id;
    }

    chk = get_check_sum(&pbuf[3], pbuf[2]);
    pbuf[size - 1] = chk;

    (*databuf) = pbuf;

    printf("get select room inf = %d\n", p_select_room_arg->num);

    return size;
}

int get_forbidden_zone(BYTE_T **databuf)
{
     // 协议内容 head | len | cmd | num | x0 | y0 | x1 | y1 | x2 | y2 | x3 | y3 |...
     //         U8 | U8 | U8 | U8 | U16 | U16 | ...
    forbidden_arg_t *p_fb_arg = robot_ctl_get_forbid_arg();

    uint8_t head = RAW_APP_SEND_HEAD;
    uint8_t version = RAW_APP_SEND_VERSION;
    uint8_t cmd = 0x19;
    uint8_t num = p_fb_arg->num;
    uint8_t len = sizeof(cmd) + sizeof(num) + 2 * 8 * num;
    uint8_t chk = 0;
    uint32_t size = len + sizeof(version) + sizeof(head) + sizeof(len) + sizeof(chk);
    uint8_t j = 0;
    BYTE_T* pbuf = (BYTE_T*)malloc(size);
    if(pbuf == NULL) {
        printf("get_forbidden_zone %d malloc failed\n", size);
        return -1;
    }
    memset(pbuf, 0, size);
    pbuf[j ++] = head;
    pbuf[j ++] = version;
    pbuf[j ++] = len;
    pbuf[j ++] = cmd;
    pbuf[j ++] = num;

    printf("send forbin zone\n");
    float data;
    for (int i = 0; i < 8 * num; i++)
    {
        memcpy(&data, (uint8_t*)p_fb_arg + sizeof(DATA_ZONE_EX) + i*sizeof(data), sizeof(data));

        pbuf[j + 2*i]   = ((int16_t)data * 10) >> 8;
        pbuf[j + 2*i+1] = ((int16_t)data * 10) & 0x00FF;

       printf("zone data H %d L %d,data %.2f\n", pbuf[j + 2*i], pbuf[j + 2*i+1], data);
    }
    printf("end zone\n");
    chk = get_check_sum(&pbuf[3], pbuf[2]);
    pbuf[size - 1] = chk;

    (*databuf) = pbuf;

    return size;
}


int get_forbidden_zone_new(BYTE_T **databuf)
{
     // 协议内容 head | len | cmd | num | x0 | y0 | x1 | y1 | x2 | y2 | x3 | y3 |...
     //         U8 | U8 | U8 | U8 | U16 | U16 | ...
    forbidden_arg_t *p_fb_arg = robot_ctl_get_forbid_arg();

    uint8_t head = RAW_APP_SEND_HEAD;
    uint8_t version = RAW_APP_SEND_VERSION;
    uint8_t cmd = 0x1B;
    uint8_t num = p_fb_arg->num;
    uint8_t len = sizeof(cmd) + sizeof(num) + 18 * num;
    uint8_t chk = 0;
    uint32_t size = len + sizeof(version) + sizeof(head) + sizeof(len) + sizeof(chk);
    uint8_t j = 0;
    BYTE_T* pbuf = (BYTE_T*)malloc(size);
    if(pbuf == NULL) {
        printf("get_forbidden_zone %d malloc failed\n", size);
        return -1;
    }
    memset(pbuf, 0, size);
    pbuf[j ++] = head;
    pbuf[j ++] = version;
    pbuf[j ++] = len;
    pbuf[j ++] = cmd;
    pbuf[j ++] = num;  
    printf("send forbin zone\n");
    float data;
    uint32_t offset = sizeof(DATA_ZONE_CLASS_EX);
        uint8_t temp_offset = 17;
    for(int i = 0; i < num; i++) //zone 只有2个点，再补2个
    {
        data = p_fb_arg->data[i].rect.x0 * 10;
        //printf("fb = %f\n",p_fb_arg->data[i].rect.x0)
        if(p_fb_arg->data[i].forbid_type == WET_FORBIDDEN)
        {
            pbuf[j + temp_offset*i]   = 2;  
        }
        else
        {
            pbuf[j + temp_offset*i]   = 0;  
        }

        pbuf[j + temp_offset*i+1]   = 4;  /* 固定为4 */

        pbuf[j + temp_offset*i+2]   = ((int16_t)data) >> 8;
        pbuf[j + temp_offset*i+3] = ((int16_t)data) & 0x00FF;

        data = p_fb_arg->data[i].rect.y0 *10;
        pbuf[j + temp_offset*i+4] = ((int16_t)data) >> 8;
        pbuf[j + temp_offset*i+5] = ((int16_t)data) & 0x00FF;

        data = p_fb_arg->data[i].rect.x1 * 10;
        pbuf[j + temp_offset*i+6] = ((int16_t)data) >> 8;
        pbuf[j + temp_offset*i+7] = ((int16_t)data) & 0x00FF;

        data = p_fb_arg->data[i].rect.y1 * 10;
        pbuf[j + temp_offset*i+8] = ((int16_t)data) >> 8;
        pbuf[j + temp_offset*i+9] = ((int16_t)data) & 0x00FF;

        data = p_fb_arg->data[i].rect.x2 * 10;
        pbuf[j + temp_offset*i+10] = ((int16_t)data) >> 8;
        pbuf[j + temp_offset*i+11] = ((int16_t)data) & 0x00FF;

        data = p_fb_arg->data[i].rect.y2 * 10;
        pbuf[j + temp_offset*i+12] = ((int16_t)data) >> 8;
        pbuf[j + temp_offset*i+13] = ((int16_t)data) & 0x00FF;

        data = p_fb_arg->data[i].rect.x3 * 10;
        pbuf[j + temp_offset*i+14] = ((int16_t)data) >> 8;
        pbuf[j + temp_offset*i+15] = ((int16_t)data) & 0x00FF;
        data = p_fb_arg->data[i].rect.y3 * 10;
        pbuf[j + temp_offset*i+16] = ((int16_t)data) >> 8;
        pbuf[j + temp_offset*i+17] = ((int16_t)data) & 0x00FF;
        
        j ++;
        //printf("fb zone no %d, x0 %.2f, y0 %.2f, x1 %.2f, y1 %.2f, x2 %.2f, y2 %.2f, x3 %.2f, y3 %.2f \n", i,
              // rect[i].x0, rect[i].y0, rect[i].x1, rect[i].y0, rect[i].x1, rect[i].y1, rect[i].x0, rect[i].y1);
    }
    printf("end zone\n");
    chk = get_check_sum(&pbuf[3], pbuf[2]);
    pbuf[size - 1] = chk;

    (*databuf) = pbuf;

    return size;
}

int get_virtual_wall(BYTE_T **databuf)
{
     // 协议内容 head | len | cmd | num | x1 | y1 | x2 | y2 |
     //         U8 | U8 | U8 | U8 | U16 | U16 | ...
    virtual_wall_arg_t *p_vwall_arg = robot_ctl_get_vwall_arg();

    uint8_t head = RAW_APP_SEND_HEAD;
    uint8_t version = RAW_APP_SEND_VERSION;
    uint8_t cmd = 0x13;
    uint8_t num = p_vwall_arg->num;
    uint8_t len = sizeof(cmd) + sizeof(num) + 2 * 4 * num;
    uint8_t chk = 0;
    uint32_t size = len + sizeof(head) + sizeof(version) + sizeof(len) + sizeof(chk);
    uint8_t j = 0;
    BYTE_T* pbuf = (BYTE_T*)malloc(size);
    if(pbuf == NULL) {
        printf("get_virtual_wall %d malloc failed\n", size);
        return -1;
    }
    memset(pbuf, 0, size);
    pbuf[j ++] = head;
    pbuf[j ++] = version;
    pbuf[j ++] = len;
    pbuf[j ++] = cmd;
    pbuf[j ++] = num;

    printf("send wall\n");
    float data;
    int offset =  sizeof(DATA_CLASS_WALL) ;
    for (int i = 0; i < 4 * num; i++)
    {
        if(i % 4 == 0)
        {
            offset = offset + sizeof(forbidden_type_t);
        }
        memcpy(&data, (uint8_t*)p_vwall_arg + offset+ i*sizeof(data), sizeof(data));

        pbuf[j + 2*i]   = ((int16_t)data * 10) >> 8;
        pbuf[j + 2*i+1] = ((int16_t)data * 10) & 0x00FF;

        printf("wall data H %d, L %d, data %.2f\n", pbuf[4 + 2*i], pbuf[4 + 2*i+1],data);
    }
    printf("end wall\n");
    chk = get_check_sum(&pbuf[3], pbuf[2]);
    pbuf[size - 1] = chk;

    (*databuf) = pbuf;

    return size;
}

static void free_mem(void *ptr) {
    if(ptr != NULL) {
        free(ptr);
    }
}

int wifi_app_msg_dp_parser_get_all_config(void *msg)
{
    int pose_size;
    int clean_zone_size;
    int forbid_zone_size;
    int virtual_wall_size;
    int select_room_size;
    int total_size;

    BYTE_T *pbuffer_pose = NULL;
    BYTE_T *pbuffer_zone = NULL;
    BYTE_T *pbuffer_forbid = NULL;
    BYTE_T *pbuffer_virwall = NULL;
    BYTE_T *pbuffer_select_room = NULL;
    BYTE_T *pbuffer_all = NULL;

    pose_size = get_goto_position(&pbuffer_pose);
    clean_zone_size = get_clean_zone(&pbuffer_zone);
    forbid_zone_size = get_forbidden_zone_new(&pbuffer_forbid);
    virtual_wall_size = get_virtual_wall(&pbuffer_virwall);
    select_room_size = get_select_room_inf(&pbuffer_select_room);
    if(pose_size < 0 || clean_zone_size < 0 || forbid_zone_size < 0 || virtual_wall_size < 0 || select_room_size < 0) {
        free_mem(pbuffer_pose);
        free_mem(pbuffer_zone);
        free_mem(pbuffer_forbid);
        free_mem(pbuffer_virwall);
        free_mem(pbuffer_select_room);
        printf("tuya get cache malloc failed1\n");
        return OPRT_MALLOC_FAILED;
    }

    total_size = pose_size + clean_zone_size + forbid_zone_size + virtual_wall_size + select_room_size;

    pbuffer_all = (BYTE_T *)malloc(total_size);
    if(pbuffer_all == NULL) {
        free_mem(pbuffer_pose);
        free_mem(pbuffer_zone);
        free_mem(pbuffer_forbid);
        free_mem(pbuffer_virwall);
        free_mem(pbuffer_select_room);
        printf("tuya get config malloc fail t:%d\n", total_size);
        return OPRT_MALLOC_FAILED;
    }
    memset(pbuffer_all, 0, total_size);

    memcpy(pbuffer_all, pbuffer_pose, pose_size);
    memcpy(pbuffer_all+pose_size, pbuffer_zone, clean_zone_size);
    memcpy(pbuffer_all+pose_size+clean_zone_size, pbuffer_forbid, forbid_zone_size);
    memcpy(pbuffer_all+pose_size+clean_zone_size+forbid_zone_size, pbuffer_virwall, virtual_wall_size);
    memcpy(pbuffer_all+pose_size+clean_zone_size+forbid_zone_size+virtual_wall_size, pbuffer_select_room, select_room_size);
    OPERATE_RET op_ret = dev_report_dp_raw_sync(NULL, DPID_COMMAND, pbuffer_all, total_size, 0);
    if(OPRT_OK != op_ret)
    {
        PR_ERR("dev_report_dp_json_async op_ret:%d",op_ret);
    }

    free(pbuffer_pose);
    free(pbuffer_zone);
    free(pbuffer_forbid);
    free(pbuffer_virwall);
    free(pbuffer_select_room);
    free(pbuffer_all);
    pbuffer_pose = NULL;
    pbuffer_zone = NULL;
    pbuffer_forbid = NULL;
    pbuffer_virwall = NULL;
    pbuffer_select_room = NULL;
    pbuffer_all = NULL;
    printf("report all data to app\n");
    return op_ret;
}

STM_SCENE_E get_cur_scene(void) {
    return m_cur_scene;
}

void set_cur_scene(STM_SCENE_E scene) {
    printf("parse set m_cur_scene mode %d,  cur mode  %d\n", m_cur_scene, scene);
    m_cur_scene = scene;
}

BOOL_T get_change_mode_flag(void)
{
    /* 在上面下发切换模式到清扫指令下来期间标记一下切换模式 */
    return g_change_mode_flag;
}


/////////////////////////////////////////////////////////////////////
// DP 响应
static int wifi_app_msg_dp_parser_work_mode(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;

    if (p_dp_obj->type != PROP_ENUM)
    {
        printf("ERORR dp type!\n");
        return -1;
    }

    int workmode = p_dp_obj->value.dp_enum;

    g_change_mode_flag = true;
    STM_SCENE_E curScene = wifi_app_data_map_clean_mode(workmode);
	printf("parse pre scene %d,  cur scene %d\n", m_cur_scene, curScene);
	
    if (workmode == WORK_MODE_PART) {
        workmode = WORK_MODE_POSE;
    }
    // 上报APP
    TY_OBJ_DP_S dp_data;
    dp_data.dpid = DPID_MODE;
    dp_data.type = PROP_ENUM;
    dp_data.value.dp_enum = workmode;
    dp_data.time_stamp = 0;

    OPERATE_RET op_ret = dev_report_dp_json_async(NULL, &dp_data, 1);
    if(OPRT_OK != op_ret) {
        printf("DPID_MODE report error: %d\r\n", op_ret);
    }
    if(curScene != m_cur_scene) {
        defined_pose_arg_t *pGotoPos = robot_ctl_get_goto_pos_arg();
        defined_rects_arg_t *pRect = robot_ctl_get_rect_clean_arg();
        SelectedCleanRoom *pSelect = robot_ctl_get_select_room_arg();
        if(pGotoPos->num > 0 && curScene != SCENE_GOTO_POS) {
            //当图钉存在时，切换到其他状态清除参数
            robot_ctl_reset_goto_pos_arg();
            wifi_app_msg_dp_parser_get_all_config(NULL);
        }
        if(pRect->num > 0 && curScene != SCENE_RECT_CLEAN) {
            //当划区清扫框存在时，切换到其他状态清除参数
            robot_ctl_reset_rect_clean_arg();
            wifi_app_msg_dp_parser_get_all_config(NULL);
        }
        if(pSelect->num > 0 && curScene != SCENE_CHOICE_ROOMS_CLEAN) {
            //当选区清扫标识存在时，切换到其他状态清除参数
            robot_ctl_reset_select_room_arg();
            wifi_app_msg_dp_parser_get_all_config(NULL);
        }
        m_cur_scene = curScene;
    }
	
    if(WORK_MODE_SMART == workmode)
    {
    	printf("workmode = %d ,curScene = %d\n",workmode,curScene);
        /* 若判断到处于重定位当中，则暂停清扫 */
        robot_ctl_scene_switch(curScene, return_robot_reposition_state_by_380()?SCENE_SWITCH_STOP:SCENE_SWITCH_START);
    }
    else if(WORK_MODE_CHARGEGO == workmode)
    {
         robot_ctl_scene_switch(curScene, SCENE_SWITCH_START);
    }

    // 局部清扫直接执行
    if(WORK_MODE_PART == p_dp_obj->value.dp_enum) {
        //清除导航线
        wifi_app_reset_navi();
        //启动局部清扫
        robot_ctl_scene_switch(SCENE_SPOT_CLEAN,SCENE_SWITCH_START);
    }
    return 0;
}

static int wifi_app_msg_dp_parser_charge_switch(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;
    if (p_dp_obj->type != PROP_BOOL)
    {
        printf("ERORR dp type!\n");
        return -1;
    }
    bool flag = p_dp_obj->value.dp_bool;

    printf("charge switch flag = %d\n",flag);
    if(flag)
    {
        TY_OBJ_DP_S dp_obj;
        dp_obj.type = PROP_ENUM;
        dp_obj.value.dp_enum = WORK_MODE_CHARGEGO;
       	wifi_app_msg_dp_parser_work_mode((void *)&dp_obj);
		
    }
}

void tuya_iot_robot_state_feedback(DP_ROBOT_STATE dp_state) {
#if 0
    BOOL_T mode_change_flag = get_change_mode_flag();
    if((dp_state == STATE_GOCHARGE
       || dp_state == STATE_CHARGING) && !mode_change_flag) {
        m_cur_scene = SCENE_DOCK;
    }
#else
	static char flag = 0;
    printf("m_cur_scene:%d,dp_state: %d\n",m_cur_scene,dp_state);	
    if( m_cur_scene != SCENE_DOCK && \
		(dp_state == STATE_GOCHARGE || dp_state == STATE_CHARGING)) 
	{
		//防止app某些操作设置场景，380还没状态切换成功，380定时上报的状态导致appserver缓存的场景不正确
		//当场景需要修正为回充时，延后一次设置
		flag++;
		if(flag == 2)
		{
			flag = 0;
			m_cur_scene = SCENE_DOCK;
		}	
		printf("\033[1m\033[40;33m m_cur_scene:%d , flag:%d\033[0m\n",m_cur_scene,flag);	
    }
#endif	   
}

//清扫任务开关，开启一次新的清扫
static int wifi_app_msg_dp_parser_task_switch(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;

    if (p_dp_obj->type != PROP_BOOL)
    {
        printf("ERORR dp type!\n");
        return -1;
    }

    // 任务开关
    bool flag = p_dp_obj->value.dp_bool;
	
    if(isAppointmentCmd()) {
        int robot_is_running_flag = 0;
        ROBOT_CMD_E cmd = flag?ROBOT_CMD_APPOINTMENT_ON:ROBOT_CMD_APPOINTMENT_OFF;
        
        robot_api_get_robot_state(&robot_is_running_flag); /** 判断机器是否处于清扫模式，如果是 且下发的是开始清扫命令，则过滤掉 */
        printf("parse ts appointment cmd:%d,robot_is_running_flag = %d\n", flag,robot_is_running_flag);
        
        if(robot_is_running_flag == 1 && cmd == ROBOT_CMD_APPOINTMENT_ON)
        {
            return 0;
        }

        robot_api_control(cmd, NULL);

        return 0;
    }
    s_task_sw = flag;
	
    printf(" m_cur_scene = %d, bool_flag:%d \n", m_cur_scene,flag);
    
    if((SCENE_GOTO_POS == m_cur_scene) && (0 == flag )) /* 如果当前的指令是指哪扫哪停止命令 */
    {
            //当图钉存在时，切换到其他状态清除参数
       robot_ctl_reset_goto_pos_arg();
       wifi_app_msg_dp_parser_get_all_config(NULL);
    }
	

	/* 这里不处理回充启动、暂停 和 自动清扫的启动, 但是响应自动清扫的暂停, 其他情况正常响应 */
	if((m_cur_scene != SCENE_DOCK && m_cur_scene != SCENE_AUTO_CLEAN) || (m_cur_scene == SCENE_AUTO_CLEAN && !flag ))
	{
		robot_ctl_scene_switch(m_cur_scene, flag?SCENE_SWITCH_START:SCENE_SWITCH_STOP);
	}
    else if((return_robot_state_by_380() == ROBOT_STATE_CLEAN_ROOM || return_robot_state_by_380() == ROBOT_STATE_CLEAN_AUTO)
     &&  m_cur_scene != SCENE_AUTO_CLEAN && !flag) /*若由380上报了清扫状态，但app未更新到位，在这里判断，直接更新*/
    {
         printf("set_cur_scene SCENE_AUTO_CLEAN %d\n",return_robot_state_by_380() );
         set_cur_scene(SCENE_AUTO_CLEAN); 
         robot_ctl_scene_switch(m_cur_scene, flag?SCENE_SWITCH_START:SCENE_SWITCH_STOP);
    }
    g_change_mode_flag = false;
    // 指哪扫哪任务结束，清除导航路线
    if (!flag && (m_cur_scene == SCENE_GOTO_POS || m_cur_scene == SCENE_SPOT_CLEAN)) {
        wifi_app_reset_navi();
    }
    if((SCENE_GOTO_POS == m_cur_scene) && (0 == flag )) /* 如果当前的指令是指哪扫哪停止命令 */
    {
       reset_last_data_robot_sta(); /* 清空上一次的清扫状态，避免出现状态不对应的情况 */
    }
    if((SCENE_CHOICE_ROOMS_CLEAN == m_cur_scene) && (0 == flag )) /* 如果当前的选区清扫停止命令 */
    {
       robot_ctl_reset_select_room_arg(); 
       wifi_app_msg_dp_parser_get_all_config(NULL);
    }  
    if(m_cur_scene == SCENE_RECT_CLEAN && (0 == flag ))
    {       
        //当划区清扫框存在时，切换到其他状态清除参数
        robot_ctl_reset_rect_clean_arg();
        wifi_app_msg_dp_parser_get_all_config(NULL);
        reset_last_data_robot_sta(); /* 清空上一次的清扫状态，避免出现状态不对应的情况 */
    }
    return 0;
}

//清扫暂停/恢复开关
static int wifi_app_msg_dp_parser_clean_switch(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;
    bool flag;

    if (p_dp_obj->type != PROP_BOOL)
    {
        printf("ERORR dp type!\n");
        return -1;
    }

    // 清扫开关
    flag = p_dp_obj->value.dp_bool;

    if(m_cur_scene == SCENE_DOCK && flag == SCENE_SWITCH_RESUME)
    {
        printf("find error charge mode exit !\n");
        return 0;
    }
    
    if(isAppointmentCmd()) {
        ROBOT_CMD_E cmd = flag?ROBOT_CMD_APPOINTMENT_ON:ROBOT_CMD_APPOINTMENT_OFF;
        printf("parse cs appointment cmd:%d\n", flag);
        robot_api_control(cmd, NULL);
        return 0;
    }
    if(!flag) { //区分 按键暂停 和 手机APP暂停
        s_task_sw = true;
    }
    return robot_ctl_scene_switch(m_cur_scene, flag?SCENE_SWITCH_PAUSE:SCENE_SWITCH_RESUME);
}

static int wifi_app_msg_dp_parser_manual_control(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;
    int protocol_enum;

    if (p_dp_obj->type != PROP_ENUM)
    {
        printf("ERORR dp type!\n");
        return -1;
    }

    // 手动控制
    protocol_enum = wifi_app_data_map_manual_control(p_dp_obj->value.dp_enum);
    if (protocol_enum != INVALID_ENUM) {
        robot_start_remote_control(protocol_enum);
    }
    return 0;
}


static int wifi_app_msg_dp_parser_fan_mode(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;
    int protocol_enum, ret = -1;

    if (p_dp_obj->type != PROP_ENUM)
    {
        printf("ERORR dp type!\n");
        return -1;
    }

    // 吸力模式
    protocol_enum = wifi_app_data_map_fan_mode(p_dp_obj->value.dp_enum);
    if (protocol_enum != INVALID_ENUM) {
        ret = robot_api_control(ROBOT_CMD_FAN_MODE, (void*)protocol_enum);
    }
    //上报APP
    if (ret != -1)
    {
        TY_OBJ_DP_S dp_data;
        dp_data.dpid = DPID_SUCTION;
        dp_data.type = PROP_ENUM;
        //dp_data.value.dp_enum = p_dp_obj->value.dp_enum;
        dp_data.value.dp_enum = get_fan_mode_and_water_level(0);/*appserver不知道是否设置成功,上报缓存的状态*/
        dp_data.time_stamp = 0;

        OPERATE_RET op_ret = dev_report_dp_json_async(NULL, &dp_data, 1);
        printf("#### 7 dev_report_dp_json_async return %d\r\n", op_ret);
    }
    return 0;
}

//水箱控制
static int wifi_app_msg_dp_parser_water_level(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;
    int protocol_enum, ret = -1;

    if (p_dp_obj->type != PROP_ENUM)
    {
        printf("ERORR dp type!\n");
        return -1;
    }

    protocol_enum = wifi_app_data_map_water_level(p_dp_obj->value.dp_enum);
    if (protocol_enum != INVALID_ENUM) {
        ret = robot_api_control(ROBOT_CMD_WATER_LEVEL, (void*)protocol_enum);
    }
    //上报APP
    if (ret != -1)
    {
        TY_OBJ_DP_S dp_data;
        dp_data.dpid = DPID_WATER_MODE;
        dp_data.type = PROP_ENUM;
        //dp_data.value.dp_enum = p_dp_obj->value.dp_enum;
		dp_data.value.dp_enum = get_fan_mode_and_water_level(1);/*appserver不知道是否设置成功,上报缓存的状态*/
        dp_data.time_stamp = 0;

        OPERATE_RET op_ret = dev_report_dp_json_async(NULL, &dp_data, 1);
        printf("#### 8 dev_report_dp_json_async return %d\r\n", op_ret);
    }
}



static int wifi_app_msg_dp_parser_volume_set(void *msg)
{
}

static int wifi_app_msg_dp_parser_goto_position(void *msg)
{
    // 清除上一次的路径
    //wifi_app_reset_path();

    uint8_t *pcmd_data = (uint8_t *)msg;

    // 协议内容 cmd | num | x | y == U8 | U8 | U16 | U16
    float dx = (int16_t)(pcmd_data[1] << 8 | pcmd_data[2]);
    float dy = (int16_t)(pcmd_data[3] << 8 | pcmd_data[4]);

    //float型放大10倍传输
    float x = dx / 10.0;
    float y = dy / 10.0;
    printf("CMD goto position:x %.2f, y %.2f\n", x, y);
    // 发送命令
    DATA_GOTO_POSITION pos = {
        .x = x,
        .y = y,
        .type = 2   //标案清扫次数为2次
    };
    set_cur_scene(SCENE_GOTO_POS);
    robot_ctl_exec_with_param(ROBOT_CMD_GOTO_POS, &pos);
    s_task_sw = true;
    wifi_app_msg_dp_parser_get_all_config(NULL);
    
    //set_send_cloud_flag(1);
    //robot_api_send_msg_to_hoslam(TUYA_MSG_TYPE_GET_CURRENT_MAP, NULL, 0);
    return 0;
}

int get_type_clean_zone_flag(void)
{
	return type_clean_zone_flag;
}
int set_type_clean_zone_flag(bool flag)
{
	type_clean_zone_flag = flag;
	return 0;
}

static int wifi_app_msg_dp_parser_clean_zone(void *msg)
{
    uint8_t *pcmd_data = (uint8_t *)msg;

    // 协议内容 cmd | count | x0 | y0 | x1 | y1 | x2 | y2 | x3 | y3 | ... == U8 | U8 | U16 | U16 | ...
    uint32_t count = pcmd_data[1];
    uint32_t size = sizeof(count) + count * sizeof(DATA_RECT);
    DATA_ZONE *zone = (DATA_ZONE *)malloc(size);
    zone->count = count;

    float dataxy;
    zone->count = count;
    for (int i = 0; i < count; i++)
    {
        dataxy = (int16_t)(pcmd_data[2 + 16 * i] << 8 | pcmd_data[2 + 16 * i + 1]);
        zone->rect[i].x0 = dataxy / 10.0;

        dataxy = (int16_t)(pcmd_data[2 + 16 * i + 2] << 8 | pcmd_data[2 + 16 * i + 3]);
        zone->rect[i].y0 = dataxy / 10.0;

        dataxy = (int16_t)(pcmd_data[2 + 16 * i + 8] << 8 | pcmd_data[2 + 16 * i + 9]);
        zone->rect[i].x1 = dataxy / 10.0;

        dataxy = (int16_t)(pcmd_data[2 + 16 * i + 10] << 8 | pcmd_data[2 + 16 * i + 11]);
        zone->rect[i].y1 = dataxy / 10.0;

        printf("CMD zone clean: count %d, cur %d, x0 %.2f, y0 %.2f, x1 %.2f, y1 %.2f\n",
               count, i, zone->rect[i].x0, zone->rect[i].y0, zone->rect[i].x1, zone->rect[i].y1); //count矩形框个数
    }
    if (zone->count > 0) {
        set_cur_scene(SCENE_RECT_CLEAN);
		set_type_clean_zone_flag(false);
        robot_ctl_exec_with_param(ROBOT_CMD_RECT_CLEAN_START, zone);
        s_task_sw = true;
    }
    free(zone);
    wifi_app_msg_dp_parser_get_all_config(NULL);

    return 0;
}


static int wifi_app_msg_dp_parser_type_clean_zone(void *msg)
{
    uint8_t *pcmd_data = (uint8_t *)msg;
    uint32_t datalen;
    uint8_t header_len = 4;
    // 协议内容 cmd |clean_count| num | x0 | y0 | x1 | y1 | x2 | y2 | x3 | y3 | ... == U8 | U8 | U16 | U16 | ...
    int clean_count = pcmd_data[1];//pcmd_data[2];  /* 清扫次数 */
    uint32_t number = pcmd_data[2];//pcmd_data[1];  /* 画框个数 */
    uint8_t polygon = pcmd_data[3];  /* 多边形点数：目前只支持四个点 ，故固定为4 */
    
    printf("zone polygon = %d\n",polygon);
    datalen = sizeof(DATA_TYPE_ZONE) + number * sizeof(DATA_RECT);

    int32_t size = sizeof(datalen) + sizeof(DATA_TYPE_ZONE) + number * sizeof(DATA_RECT);

    int32_t *send_zone = (int32_t*)malloc(size);

    *send_zone = datalen;

    DATA_TYPE_ZONE *zone = (DATA_TYPE_ZONE *)(send_zone + 1);

    zone->clean_count = clean_count;//清扫次数
    zone->data_zone.count = number;//矩形个数
    printf("zone clean count = %d,zone.count = %d\n",zone->clean_count,zone->data_zone.count);
    float dataxy;
    for (int i = 0; i < number; i++)
    {
        dataxy = (int16_t)(pcmd_data[header_len + 16 * i] << 8 | pcmd_data[header_len + 16 * i + 1]);
        zone->data_zone.rect[i].x0 = dataxy / 10.0;

        dataxy = (int16_t)(pcmd_data[header_len + 16 * i + 2] << 8 | pcmd_data[header_len + 16 * i + 3]);
        zone->data_zone.rect[i].y0 = dataxy / 10.0;

        dataxy = (int16_t)(pcmd_data[header_len + 16 * i + 8] << 8 | pcmd_data[header_len + 16 * i + 9]);
        zone->data_zone.rect[i].x1 = dataxy / 10.0;

        dataxy = (int16_t)(pcmd_data[header_len + 16 * i + 10] << 8 | pcmd_data[header_len + 16 * i + 11]);
        zone->data_zone.rect[i].y1 = dataxy / 10.0;
        
        header_len ++;
        printf("CMD zone clean: number %d, cur %d, x0 %.2f, y0 %.2f, x1 %.2f, y1 %.2f\n",
               number, i, zone->data_zone.rect[i].x0, zone->data_zone.rect[i].y0, zone->data_zone.rect[i].x1, zone->data_zone.rect[i].y1); //number矩形框个数
    }
	
    if (zone->data_zone.count > 0)
    {
        set_cur_scene(SCENE_RECT_CLEAN);
        robot_api_send_msg_to_hoslam(TUYA_MSG_TYPE_TYPE_ZOON_CLEAN, (char *)send_zone, size);	
		set_type_clean_zone_flag(true);
        robot_ctl_exec_with_param(ROBOT_CMD_RECT_CLEAN_START, &(zone->data_zone));
        s_task_sw = true;
    }
    free(send_zone);
    wifi_app_msg_dp_parser_get_all_config(NULL);

    return 0;
}

static int wifi_app_msg_dp_parser_get_clean_zone(void *msg)
{
    BYTE_T *pbuffer = NULL;
    int size = get_clean_zone(&pbuffer);

    OPERATE_RET op_ret = dev_report_dp_raw_sync(NULL, DPID_COMMAND, pbuffer, size, 0);
    if (OPRT_OK != op_ret)
    {
        PR_ERR("dev_report_dp_json_async op_ret:%d", op_ret);
    }

    free(pbuffer);
    pbuffer = NULL;

    return op_ret;
}

static int wifi_app_msg_dp_parser_forbidden_zone(void *msg)
{
    int robot_is_running_flag;
    uint8_t time_count = 0;

    robot_api_get_robot_state(&robot_is_running_flag);

    if(robot_is_running_flag)
    {
    	robot_ctl_scene_switch(m_cur_scene, SCENE_SWITCH_STOP);
    }

    do
    {
        usleep(200*1000);
        robot_api_get_robot_state(&robot_is_running_flag);
        if(robot_is_running_flag == 0)
        {
            break;
        }
        time_count ++;

    }while(time_count < 8);
    printf("time_count = %d\n", time_count);
    if(time_count >= 8)
    {
        return -1;
    }
    uint8_t *pcmd_data = (uint8_t *)msg;

    // 协议内容 cmd | num | x0 | y0 | x1 | y1 | x2 | y2 | x3 | y3 |... == U8 | U8 | U16 | U16 | ...
    uint32_t count = pcmd_data[1];
    printf("CMD forbidden zone: count=%d\n", count); //count矩形框个数

    uint32_t size = sizeof(count) + count * sizeof(DATA_RECT_EX);
    DATA_ZONE_EX *zone = malloc(size);
    if(zone == NULL)
    {
        printf("zone malloc failed! \n");
        return -1;
    }
    float dataxy;
    zone->count = count;
    for (int i = 0; i < count; i++) {
        dataxy = (int16_t)(pcmd_data[2 + 16 * i] << 8 | pcmd_data[2 + 16 * i + 1]);
        zone->rect[i].x0 = dataxy / 10.0;

        dataxy = (int16_t)(pcmd_data[2 + 16 * i + 2] << 8 | pcmd_data[2 + 16 * i + 3]);
        zone->rect[i].y0 = dataxy / 10.0;

        dataxy = (int16_t)(pcmd_data[2 + 16 * i + 4] << 8 | pcmd_data[2 + 16 * i + 5]);
        zone->rect[i].x1 = dataxy / 10.0;

        dataxy = (int16_t)(pcmd_data[2 + 16 * i + 6] << 8 | pcmd_data[2 + 16 * i + 7]);
        zone->rect[i].y1 = dataxy / 10.0;

        dataxy =  (int16_t)(pcmd_data[2 + 16 * i + 8] << 8 | pcmd_data[2 + 16 * i + 9]);
        zone->rect[i].x2 = dataxy / 10.0;

        dataxy =  (int16_t)(pcmd_data[2 + 16 * i + 10] << 8 | pcmd_data[2 + 16 * i + 11]);
        zone->rect[i].y2 = dataxy / 10.0;

        dataxy =  (int16_t)(pcmd_data[2 + 16 * i + 12] << 8 | pcmd_data[2 + 16 * i + 13]);
        zone->rect[i].x3 = dataxy / 10.0;

        dataxy =  (int16_t)(pcmd_data[2 + 16 * i + 14] << 8 | pcmd_data[2 + 16 * i + 15]);
        zone->rect[i].y3 = dataxy / 10.0;

        printf("CMD forbidon zone: count %d, cur %d, x0 %.2f, y0 %.2f, x1 %.2f, y1 %.2f, x2 %.2f, y2 %.2f, x3 %.2f, y3 %.2f,\n",
                   count, i, zone->rect[i].x0, zone->rect[i].y0, zone->rect[i].x1, zone->rect[i].y1,
                   zone->rect[i].x2, zone->rect[i].y2, zone->rect[i].x3, zone->rect[i].y3);
    }
    robot_ctl_exec_with_param(ROBOT_CMD_SET_FORBIDDEN, zone);
    free(zone);

    wifi_app_msg_dp_parser_get_all_config(NULL);
    // 发送数据包
    return 0;
}

static int wifi_app_msg_dp_parser_forbidden_zone_new(void *msg)
{
    int robot_is_running_flag;
    uint8_t time_count = 0;

    robot_api_get_robot_state(&robot_is_running_flag);

    if(robot_is_running_flag)
    {
    	robot_ctl_scene_switch(m_cur_scene, SCENE_SWITCH_STOP);
    }

    do
    {
        usleep(200*1000);
        robot_api_get_robot_state(&robot_is_running_flag);
        if(robot_is_running_flag == 0)
        {
            break;
        }
        time_count ++;

    }while(time_count < 8);
    printf("time_count = %d\n", time_count);
    if(time_count >= 8)
    {
        return -1;
    }
    uint8_t header_len = 4;

    uint8_t *pcmd_data = (uint8_t *)msg;
	//涂鸦公版新协议	
    // 协议内容 cmd | num | mode | Polygon | x0 | y0 | x1 | y1 | x2 | y2 | x3 | y3 |... == U8 | U8 | U16 | U16 | ...
    uint32_t count = pcmd_data[1];
    printf("CMD forbidden zone: count=%d\n", count); //num矩形框个数
    
	uint8_t forbidden_zone_tpye = pcmd_data[2];//mode :00：全禁、01：禁扫、02禁拖，
	uint8_t polygon_num = pcmd_data[3];//Polygon多边形点数

	printf("\033[1m\033[40;33m forbidden_zone_tpye:%d polygon_num:%d \033[0m\n",forbidden_zone_tpye,polygon_num);
    
    uint32_t size = sizeof(count) + count * sizeof(DATA_FOBIDEEN);
    DATA_ZONE_CLASS_EX *zone = malloc(size);
    if(zone == NULL)
    {
        printf("zone malloc failed! \n");
        return -1;
    }
    float dataxy;
    zone->count = count;
    for (int i = 0; i < count; i++) {
        if(pcmd_data[header_len + 16 * i-2] == 0x00)//全禁
        {
            zone ->fobidden[i].forbid_type = FORBIDDEN_ALL;   
        }
        else
        {
            zone ->fobidden[i].forbid_type = WET_FORBIDDEN;
        }
        

        dataxy = (int16_t)(pcmd_data[header_len + 16 * i] << 8 | pcmd_data[header_len + 16 * i + 1]);
        zone->fobidden[i].rect.x0 = dataxy / 10.0;

        dataxy = (int16_t)(pcmd_data[header_len + 16 * i + 2] << 8 | pcmd_data[header_len + 16 * i + 3]);
        zone->fobidden[i].rect.y0 = dataxy / 10.0;

        dataxy = (int16_t)(pcmd_data[header_len + 16 * i + 4] << 8 | pcmd_data[header_len + 16 * i + 5]);
        zone->fobidden[i].rect.x1 = dataxy / 10.0;

        dataxy = (int16_t)(pcmd_data[header_len + 16 * i + 6] << 8 | pcmd_data[header_len + 16 * i + 7]);
        zone->fobidden[i].rect.y1 = dataxy / 10.0;

        dataxy =  (int16_t)(pcmd_data[header_len + 16 * i + 8] << 8 | pcmd_data[header_len + 16 * i + 9]);
        zone->fobidden[i].rect.x2 = dataxy / 10.0;

        dataxy =  (int16_t)(pcmd_data[header_len + 16 * i + 10] << 8 | pcmd_data[header_len + 16 * i + 11]);
        zone->fobidden[i].rect.y2 = dataxy / 10.0;

        dataxy =  (int16_t)(pcmd_data[header_len + 16 * i + 12] << 8 | pcmd_data[header_len + 16 * i + 13]);
        zone->fobidden[i].rect.x3 = dataxy / 10.0;

        dataxy =  (int16_t)(pcmd_data[header_len + 16 * i + 14] << 8 | pcmd_data[header_len + 16 * i + 15]);
        zone->fobidden[i].rect.y3 = dataxy / 10.0;
        header_len = header_len + 2;
        printf("CMD forbidon zone: count %d, cur %d, x0 %.2f, y0 %.2f, x1 %.2f, y1 %.2f, x2 %.2f, y2 %.2f, x3 %.2f, y3 %.2f,\n",
                   count, i, zone->fobidden[i].rect.x0, zone->fobidden[i].rect.y0, zone->fobidden[i].rect.x1, zone->fobidden[i].rect.y1,
                   zone->fobidden[i].rect.x2, zone->fobidden[i].rect.y2, zone->fobidden[i].rect.x3, zone->fobidden[i].rect.y3);
    }
    robot_ctl_exec_with_param(ROBOT_CMD_SET_FORBIDDEN, zone);
    free(zone);

    wifi_app_msg_dp_parser_get_all_config(NULL);
    // 发送数据包
    return 0;
}

static int wifi_app_msg_dp_parser_get_forbidden_zone(void *msg)
{
    BYTE_T *pbuffer = NULL;
    int size = get_forbidden_zone(&pbuffer);

    OPERATE_RET op_ret = dev_report_dp_raw_sync(NULL, DPID_COMMAND, pbuffer, size, 0);
    if (OPRT_OK != op_ret)
    {
        PR_ERR("dev_report_dp_json_async op_ret:%d", op_ret);
    }

    free(pbuffer);
    pbuffer = NULL;

    return op_ret;
}

static int wifi_app_msg_dp_parser_get_forbidden_zone_new(void *msg)
{
    BYTE_T *pbuffer = NULL;
    int size = get_forbidden_zone_new(&pbuffer);

    OPERATE_RET op_ret = dev_report_dp_raw_sync(NULL, DPID_COMMAND, pbuffer, size, 0);
    if (OPRT_OK != op_ret)
    {
        PR_ERR("dev_report_dp_json_async op_ret:%d", op_ret);
    }

    free(pbuffer);
    pbuffer = NULL;

    return op_ret;
}

static int wifi_app_msg_dp_parser_virtual_wall(void *msg)
{
    int robot_is_running_flag;
    uint8_t time_count = 0;

    robot_api_get_robot_state(&robot_is_running_flag);

    if(robot_is_running_flag)
    {
    	robot_ctl_scene_switch(m_cur_scene, SCENE_SWITCH_STOP);
    }

    do
    {
        usleep(200*1000);
        robot_api_get_robot_state(&robot_is_running_flag);
        if(robot_is_running_flag == 0)
        {
            break;
        }
        time_count ++;

    }while(time_count < 8);
    printf("time_count = %d\n", time_count);

    if(time_count >= 8)
    {
        return -1;
    }
    uint8_t *pcmd_data = (uint8_t *)msg;

    // 协议内容 cmd | num | x1 | y1 | x2 | y2 |   == U8 | U8 | U16 | U16 | ...
    uint32_t count = pcmd_data[1];
    printf("CMD virtual zone: count=%d\n", count); //count虚拟墙的个数

    uint32_t size = sizeof(count) + count * sizeof(DATA_VIRTUAL);;
    DATA_CLASS_WALL *wall = (DATA_CLASS_WALL *)malloc(size);

    float dataxy;
    wall->count = count;

    for (int i = 0; i < count; i++)
    {
        wall->virtual[i].forbid_type = FORBIDDEN_ALL;
        dataxy = (int16_t)(pcmd_data[2 + 8 * i] << 8 | pcmd_data[2 + 8 * i + 1]);
        wall->virtual[i].line.x0 = dataxy / 10.0;

        dataxy = (int16_t)(pcmd_data[2 + 8 * i + 2] << 8 | pcmd_data[2 + 8 * i + 3]);
        wall->virtual[i].line.y0 = dataxy / 10.0;

        dataxy = (int16_t)(pcmd_data[2 + 8 * i + 4] << 8 | pcmd_data[2 + 8 * i + 5]);
        wall->virtual[i].line.x1 = dataxy / 10.0;

        dataxy = (int16_t)(pcmd_data[2 + 8 * i + 6] << 8 | pcmd_data[2 + 8 * i + 7]);
        wall->virtual[i].line.y1 = dataxy / 10.0;

        printf("CMD virtual wall: count %d, cur %d, x0 %.2f, y0 %.2f, x1 %.2f, y1 %.2f\n",
               count, i, wall->virtual[i].line.x0,wall->virtual[i].line.y0, wall->virtual[i].line.x1, wall->virtual[i].line.y1);
    }
    robot_ctl_exec_with_param(ROBOT_CMD_SET_VWALL, wall);
    free(wall);
    wifi_app_msg_dp_parser_get_all_config(NULL);
    return 0;
}

static int wifi_app_msg_dp_parser_get_virtual_wall(void *msg)
{
    BYTE_T *pbuffer = NULL;

    int size = get_virtual_wall(&pbuffer);

    OPERATE_RET op_ret = dev_report_dp_raw_sync(NULL, DPID_COMMAND, pbuffer, size, 0);
    if (OPRT_OK != op_ret)
    {
        PR_ERR("dev_report_dp_json_async op_ret:%d", op_ret);
    }

    free(pbuffer);
    pbuffer = NULL;

    return op_ret;
}

static int delete_robot_map(void)
{
    uint8_t buf[10];
    memset(buf, 0, sizeof(buf));
    
    int ret = robot_api_control(ROBOT_CMD_RESET_MAP, (void*)buf);

}
static bool delete_map_flag = 0;

void set_delete_map_flag(bool flag)
{
    delete_map_flag = flag;
    printf("set_delete_map_flag :%d\n", delete_map_flag);
}

bool get_delete_map_flag(void)
{
    //printf("get_delete_map_flag :%d\n", delete_map_flag);
    return delete_map_flag;
}

static bool reset_map_flag = 0;
bool get_reset_map_flag(void)
{
	return reset_map_flag;
}

bool set_reset_map_flag(bool flag)
{
	 reset_map_flag = flag;
	 printf("\033[1m\033[40;33m  reset_map_flag:%d \033[0m\n",reset_map_flag);
	 
}

void* set_reset_map_flag_thread(void *arg)
{
	set_reset_map_flag(true);
	sleep(2);
	set_reset_map_flag(false);
    return NULL;
}

int wifi_app_msg_dp_parser_reset_map(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;
    TY_OBJ_DP_S dp_datas;
    bool delete_map_switch;
    int config = 0;
    OPERATE_RET op_ret;
    int ret;

    if (p_dp_obj->type != PROP_BOOL)
    {
        printf("ERORR dp type!\n");
        return -1;
    }

    
    delete_map_switch = p_dp_obj->value.dp_bool;
    printf("delete_map_switch: %d\n",delete_map_switch);
    //wifi_app_voice_volume_set(value);
    if(delete_map_switch)
    {
		pthread_t thread_set_reset_map_flag;
		pthread_attr_t attr;
		pthread_attr_init(&attr);//初始化线程属性
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);//设置线程属性为分离状态
		ret = pthread_create(&thread_set_reset_map_flag, &attr,set_reset_map_flag_thread, NULL);
		if (ret)
		{
			printf("create thread_set_reset_map_flag ERROR!\n");
			return -1;
		}
        set_map_is_editable_flag(0);
        //ret = send_msg_to_hoslam_app(MSG_TYPE_DELETE_HOSLAM_MAP, &delete_map_switch, sizeof(delete_map_switch));
        /* save record before delete map */
        //force_record_map_path();
        set_delete_map_flag(true);
        delete_robot_map();
        
    }
    config = 1;

    // 设置完成后上报一次状态
    if (config)
    {
        wifi_app_reset_robot_map_data();
        wifi_app_reset_map();
        
        dp_datas.dpid = DPID_RESET_MAP;
        dp_datas.type = PROP_BOOL;
        dp_datas.value.dp_bool = delete_map_switch;
        dp_datas.time_stamp = 0;
        
        op_ret = dev_report_dp_json_async(NULL, &dp_datas, 1);
        printf("#### 15 dev_report_dp_json_async return %d\r\n", op_ret);
        return 0;
    }

}


#define MAX_VOICE           2
#define CN_VOICE            "/root/voice/zh_CN"
char * voice_choice[MAX_VOICE]={"zh_CN", "en_US"};
char send_voice_flag;

static int wifi_app_voice_language_write(char * language_path)
{
    int ret;
    int tmp_shell_ret;

    ret = sys_shell("sed -i 's;^VOICE_PATH=.*;VOICE_PATH=%s;' %s", language_path, SHELL_STARTUP_POWER);
    if (access(SHELL_AUDIO_ONESELF, R_OK) == 0)
    {
        
        tmp_shell_ret = sys_shell("cat %s | grep 'LANGUAGE'", SHELL_AUDIO_ONESELF);
        if(tmp_shell_ret == 1)
        {
            /* 命令正确但是不存在LANGUAGE */
            printf("add volume LANGUAGE %s\n", language_path);
            ret = sys_shell("echo LANGUAGE=%s >> %s", language_path, SHELL_AUDIO_ONESELF);
        }
        else if(tmp_shell_ret == 0)
        {
            /* 命令正确存在LANGUAGE, 替换LANGUAGE */
            printf("modify volume LANGUAGE => %s\n", language_path);
            ret = sys_shell("sed -i 's;^LANGUAGE=.*;LANGUAGE=%s;' %s", language_path, SHELL_AUDIO_ONESELF);
        }
        else
        {
            printf("prase LANGUAGE file fail\n");
        }
    }
    else
    {
        printf("create LANGUAGE file\n");
        ret = sys_shell("echo LANGUAGE=%s > %s", language_path, SHELL_AUDIO_ONESELF);
    }
    
    return ret;
}

int wifi_app_voice_language_read(void)
{
    char path[128];
    int ret = 0;
    int ret1 = 0;
    int path_id = 0;

    do 
    {
        // 判断配置文件
        if (access(SHELL_AUDIO_ONESELF, R_OK) != 0)
        {
            //sys_log(LOG_CRIT, "access S00voice FAILED");
            printf("2 access voice_config FAILED\n");
            ret = -1;
            break;
        }
        memset(path, 0, sizeof(path));
        ret = sys_shell_result_to_char(path, sizeof(path), "cat %s | grep 'LANGUAGE' | awk -F '=' '{print $2}'", SHELL_AUDIO_ONESELF);         
        printf("### voice choice path %s ret:%d ###\n", path, ret);
    }
    while (0);

    if(ret < 0)
    {
        /* 如果没有配置的话，就根据s10power脚本的配置, 如果s10power都没有配置就默认chinese */
        ret1 = sys_shell_result_to_char(path, sizeof(path), "cat %s | grep 'VOICE_PATH' | awk -F '=' '{print $2}'", SHELL_STARTUP_POWER);
        if(ret1 < 0)
        {
            printf("### 1 voice choice default path %d ###\n", VOICE_CHINESE);
            select_voice_path(VOICE_CHINESE);
            send_voice_flag = 0;
        }
        else
        {
            if(0 == strcmp(path,CN_VOICE))
            {
                send_voice_flag = 0;
            }
            else
            {
                send_voice_flag = 1;
            }
            path_id = find_voice_path(path);
            if(path_id < VOICE_MAX)
            {
                printf("### 2 voice choice default path follow S10power config %d ###\n", path_id);
                select_voice_path(path_id);
            }
            else
            {
                printf("### 3 voice choice default path %d ###\n", VOICE_CHINESE);
                select_voice_path(VOICE_CHINESE);
            }
        }
    }
    else
    {
        if(0 == strcmp(path,CN_VOICE))
        {
            send_voice_flag = 0;
        }
        else
        {
            send_voice_flag = 1;
        }
        //set_voice_path(path);
        if(access(path, F_OK) == 0)
        {
            printf("%s EXISITS!\n", path);
            set_voice_path(path);
        }    
        else
        {             
            printf("%s DOESN'T EXISIT!, use %s\n", path, SHELL_VOICE_MAIN_DIR);
            set_voice_path(SHELL_VOICE_MAIN_DIR);
        }
    }
    printf("### 4 send_voice_flag %d ###\n",send_voice_flag);

    return ret;
}

/* 设置固定的语音包路径 */
void set_fixed_voice_path(void)
{
    sys_shell("sed -i 's;^VOICE_PATH=.*;VOICE_PATH=%s;' %s", SHELL_VOICE_MAIN_DIR, SHELL_STARTUP_POWER);/* 将开机语音的路径固定配置为SHELL_VOICE_MAIN_DIR*/
    set_voice_path(SHELL_VOICE_MAIN_DIR); /* 支持多国语音下载，默认路径固定为 SHELL_VOICE_MAIN_DIR，机器端只保留一个语音包 */
}


int select_voice_path(DP_VOICE_CLASS value)
{
    char path[LANG_PATH_MAX_LEN];
    int ret = 0;
    
    switch (value)
    {
        case VOICE_CHINESE:
        case VOICE_ENGLISH:
        {
            snprintf(path, LANG_PATH_MAX_LEN, "%s/%s", SHELL_VOICE_MAIN_DIR, voice_choice[value]);
            printf("<<< voice choice path %s >>>\n", path);
            if(access(path, F_OK) == 0)
            {
                printf("%s EXISITS!\n", path);
                wifi_app_voice_language_write(path);
                set_voice_path(path);
				send_voice_flag = value;
            }    
            else
            {             
                printf("%s DOESN'T EXISIT!, use %s\n", path, SHELL_VOICE_MAIN_DIR);
                set_voice_path(SHELL_VOICE_MAIN_DIR);
            }
            ret = 1;
            break;
        }

        default:
            break;
    }
    return ret;
}

int find_voice_path(char * path_name)
{
    char path[LANG_PATH_MAX_LEN];
    int ret = 0;
    int i;

    memset(path, 0, sizeof(path));
    for(i=0; i<VOICE_MAX; i++)
    {
        snprintf(path, LANG_PATH_MAX_LEN, "%s/%s", SHELL_VOICE_MAIN_DIR, voice_choice[i]);
        printf("find voice path: %s =? %s\n", path_name, path);
        if(memcmp(path, path_name, strlen(path)) == 0)
        {
            printf("find default path[%d] = %s\n", i, path);
            break;
        }
    }
    return i;
}



int wifi_app_msg_dp_parser_robot_config(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;
    TY_OBJ_DP_S dp_datas;
    int value;
    int config = 0;
    OPERATE_RET op_ret;
    
    if (p_dp_obj->type != PROP_VALUE)
    {
        printf("ERORR dp type!\n");
        return -1;
    }

    
    value = p_dp_obj->value.dp_value;
    printf("SET_VOLUME: %d\n",value);
    wifi_app_voice_volume_set(value);
    config = 1;

    // 设置完成后上报一次状态
    if (config)
    {
        report_voice_config();
        
        dp_datas.dpid = DPID_SET_VOLUME;  //当前音量
        dp_datas.type = PROP_VALUE;
        dp_datas.value.dp_value = value;
        dp_datas.time_stamp = 0;
        
        op_ret = dev_report_dp_json_async(NULL, &dp_datas, 1);
        printf("#### 10 dev_report_dp_json_async return %d\r\n", op_ret);
        return 0;
    }


    //wifi_app_reset_robot_map_data();
}

static int wifi_app_msg_dp_parser_continue_clean(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;
    int protocol_bool, ret = -1;

    protocol_bool = p_dp_obj->value.dp_bool;
    printf("breakpoint_continuous_scanning = %d\n",protocol_bool);
    ret = robot_api_control(ROBOT_CMD_CONTINUE_CLEAN, (void*)protocol_bool);
    
    //上报APP
    if (ret != -1)
    {
        TY_OBJ_DP_S dp_data;
        dp_data.dpid = DPID_CONTINUE_CLEAN;
        dp_data.type = PROP_BOOL;
        dp_data.value.dp_bool = p_dp_obj->value.dp_bool;
        dp_data.time_stamp = 0;

        OPERATE_RET op_ret = dev_report_dp_json_async(NULL, &dp_data, 1);
        printf("#### 11 dev_report_dp_json_async return %d\r\n", op_ret);
    }


    //set_cur_scene(ROBOT_STATE_CONTINUE_CLEAN);
    //robot_ctl_exec_with_param(ROBOT_CMD_GOTO_POS, &pos);
    //s_task_sw = true;
    //wifi_app_msg_dp_parser_get_all_config(NULL);

}




static int calling_robot_func(int flag)
{
    int cmd = flag;
    int ret = robot_api_control(ROBOT_CMD_CALLING_ROBOT_SW, (void*)cmd);

    if(ret != 0) return -1;
    return 0;
    
}

int wifi_app_msg_dp_parser_calling_robot(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;
    TY_OBJ_DP_S dp_datas;
    int calling_robot_switch;
    int config = 0;
    OPERATE_RET op_ret;
    int ret;
    
    if (p_dp_obj->type != PROP_BOOL)
    {
        printf("ERORR dp type!\n");
        return -1;
    }
    
    calling_robot_switch = (int)p_dp_obj->value.dp_bool;
    printf("calling_robot_switch1: %d\n",calling_robot_switch);

    ret = calling_robot_func(calling_robot_switch);

    config = 1;

    // 设置完成后上报一次状态
    if (config)
    {
        dp_datas.dpid = DPID_CALLING_ROBOT;  //calling robot
        dp_datas.type = PROP_BOOL;
        dp_datas.value.dp_bool = calling_robot_switch;
        dp_datas.time_stamp = 0;
        
        op_ret = dev_report_dp_json_async(NULL, &dp_datas, 1);
        printf("#### 013 dev_report_dp_json_async return %d\r\n", op_ret);
        return 0;
    }


    //wifi_app_reset_robot_map_data();
}

static int not_disturb_func(bool flag)
{
    int ret = robot_api_control(ROBOT_CMD_NOT_DISTURB_SW, (void*)flag);

    if(ret != 0) return -1;
    return 0;
}

static int y_mop_func(bool flag)
{
    int ret = robot_api_control(ROBOT_CMD_Y_MOP_SW, (void*)flag);

    if(ret != 0) return -1;
    return 0;
}


int wifi_app_msg_dp_parser_not_disturb(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;
    TY_OBJ_DP_S dp_datas;
    bool not_disturb_switch;
    int config = 0;
    OPERATE_RET op_ret;
    int ret;
    
    if (p_dp_obj->type != PROP_BOOL)
    {
        printf("ERORR dp type!\n");
        return -1;
    }
    
    not_disturb_switch = p_dp_obj->value.dp_bool;
    printf("not_disturb_switch: %d\n",not_disturb_switch);
    //wifi_app_voice_volume_set(value);
    
    ret = not_disturb_func(not_disturb_switch);
    //ret = send_msg_to_hoslam_app(MSG_TYPE_DELETE_HOSLAM_MAP, &calling_robot_switch, sizeof(delete_map_switch));
    if(ret < 0)
    {
        return -1;
    }
    
    config = 1;

    // 设置完成后上报一次状态
    if (config)
    {
        dp_datas.dpid = DPID_OPEN_NOT_DISTURB;  //not disturb
        dp_datas.type = PROP_BOOL;
        dp_datas.value.dp_bool = not_disturb_switch;
        dp_datas.time_stamp = 0;
        
        op_ret = dev_report_dp_json_async(NULL, &dp_datas, 1);
        printf("#### 16 dev_report_dp_json_async return %d\r\n", op_ret);
        return 0;
    }


    //wifi_app_reset_robot_map_data();
}

int wifi_app_msg_dp_parser_mode_switch(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;
    
    bool custom_mode_switch;
    FILE *fbp;
    room_spec_arg_t *p_rspec_arg;
    uint8_t i = 0;
    p_rspec_arg = robot_ctl_get_room_spec_arg(); /* 获取本地房间信息结构体 */
    if (p_dp_obj->type != PROP_BOOL)
    {
        printf("ERORR dp type!\n");
        return -1;
    }
    
    custom_mode_switch = p_dp_obj->value.dp_bool;
	set_custom_mode_switch(custom_mode_switch);
    iot_report_mode_switch();

    if(custom_mode_switch == 0) /* 若是关闭定制模式，则将默认值设置给hoslam*/
    {
       for(i = 0; i < p_rspec_arg->num; i ++)
       {
           p_rspec_arg->room_spec[i].count = 1; /* 默认清扫一次*/
       }
        
    }
    else /* 若是打开定制模式，则将原先设置好的参数，再设置回去 */
    {
       for(i = 0; i < p_rspec_arg->num; i ++)
       {
           if(p_rspec_arg->room_spec[i].bundle[5] == 1)  /* 若该房间被设置过，则还原设置 */
           {
             p_rspec_arg->room_spec[i].count = p_rspec_arg->room_spec[i].bundle[4];
           }
           else
           {
             p_rspec_arg->room_spec[i].count = 1; /* 若房间没有被设置过，则默认清扫一次*/
           }

       }
    }

    send_hoslam_save_room();	 /* 通知hoslam更改的房间数据 */  
}

int wifi_app_msg_dp_parser_y_mop(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;
    TY_OBJ_DP_S mop_datas;
    bool not_y_mop_switch;
    int config = 0;
    OPERATE_RET op_ret;
    int ret;
    
    if (p_dp_obj->type != PROP_BOOL)
    {
        printf("ERORR dp type!\n");
        return -1;
    }
    
    not_y_mop_switch = p_dp_obj->value.dp_bool;
    printf(" not_y_mop_switch: %d\n", not_y_mop_switch);
    ret = y_mop_func(not_y_mop_switch);
   // printf("y_mop_func ret = %d\n",ret);
    if(ret < 0)
    {
        return -1;
    }
    
    config = 1;

    // 设置完成后上报一次状态
    if (config)
    {
        mop_datas.dpid = DPID_Y_MOP;
        mop_datas.type = PROP_BOOL;
        mop_datas.value.dp_bool = not_y_mop_switch;
        mop_datas.time_stamp = 0;
        
        op_ret = dev_report_dp_json_async(NULL, &mop_datas, 1);
        printf("#### 15 dev_report_dp_json_async return %d\r\n", op_ret);
        return 0;
    }
}

#if 0
int wifi_app_msg_dp_parser_delete_map(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;
    TY_OBJ_DP_S dp_datas;
    bool delete_map_switch;
    int config = 0;
    OPERATE_RET op_ret;
    int ret;
    
    if (p_dp_obj->type != PROP_BOOL)
    {
        printf("ERORR dp type!\n");
        return -1;
    }
    
    delete_map_switch = p_dp_obj->value.dp_bool;
    printf("delete_map_switch: %d\n",delete_map_switch);
    //wifi_app_voice_volume_set(value);
    if(delete_map_switch)
    {
        //ret = send_msg_to_hoslam_app(MSG_TYPE_DELETE_HOSLAM_MAP, &delete_map_switch, sizeof(delete_map_switch));
        delete_robot_map();
        if(ret < 0)
        {
            return -1;
        }
    }
    config = 1;

    // 设置完成后上报一次状态
    if (config)
    {
        dp_datas.dpid = DPID_DELETE_HOSLAM_MAP;  //delete map
        dp_datas.type = PROP_BOOL;
        dp_datas.value.dp_bool = delete_map_switch;
        dp_datas.time_stamp = 0;
        
        op_ret = dev_report_dp_json_async(NULL, &dp_datas, 1);
        printf("#### 15 dev_report_dp_json_async return %d\r\n", op_ret);
        return 0;
    }


    //wifi_app_reset_robot_map_data();
}
#endif

/* 恢复分区处理函数 */
int wifi_app_msg_dp_parser_segment(void *msg)
{
    robot_api_send_msg_to_hoslam(TUYA_MSG_ROOMS_SEGMENT, NULL,0);
    printf("SEGMENT\n");

}
static int wifi_app_msg_dp_parser_smart_rooms(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;
    int protocol_enum, ret = -1;

    if (p_dp_obj->type != PROP_ENUM)
    {
        printf("ERORR dp type!\n");
        return -1;
    }

    // 智能分区指令
    protocol_enum = p_dp_obj->value.dp_enum;
    printf(" smart_rooms=%d\n",protocol_enum);

    //指令执行
    switch(protocol_enum)
    {
         case CUR_ROOMS:  
            robot_api_send_msg_to_hoslam(TUYA_MSG_ROOMS_GET_WORKING_ROOMS, NULL,0);        
            printf("CUR_ROOMS\n");
            break;
         case EDIT_ROOMS:  
            robot_api_send_msg_to_hoslam(TUYA_MSG_ROOMS_GET_EDITING_ROOMS, NULL,0);
            printf("EDIT_ROOMS\n");
            break;
         case SEGMENT: 
            wifi_app_msg_dp_parser_segment(NULL);
            break;
         case CLEAR_ROOMS:  
            robot_api_send_msg_to_hoslam(TUYA_MSG_ROOMS_CLEAR_ROOMS, NULL,0);
            printf("CLEAR_ROOMS\n");
            break;
         case SAVE_ROOMS:  
            printf("SAVE_ROOMS\n");
            send_hoslam_save_room();
            break;
                               
    }
    return 0;
}

int split_room;
float split_x1,split_y1,split_x2,split_y2;

int send_hoslam_split(int extersion_lenth)
{
     uint8_t packet_buffer[WIFI_APP_DATA_PROTOCOL_PACKET_BUF];
     float k,sintheta,costheta;

	 uint32_t datalen = 20;
	 *((int*)packet_buffer) = datalen;
     *((int*)packet_buffer + 1) = split_room;
     uint32_t size = sizeof(datalen) + sizeof(int) + 4 * sizeof(float);

     do{
         if(split_x1 == split_x2)
         {
            if(split_y1 > split_y2)
            {
                split_y1 += extersion_lenth;
                split_y2 -= extersion_lenth;
            }
            else
            {
                split_y1 -= extersion_lenth;
                split_y2 += extersion_lenth;
            }
            break;

         }
         if(split_y1 == split_y2)
         {
            split_x1 -= extersion_lenth;
            split_x2 += extersion_lenth;
            break;
         }

         k = (split_y2 - split_y1)/(split_x2 - split_x1);
         sintheta = 1/sqrt(1 + (1/k)*(1/k));
         costheta = 1/sqrt(1 + k * k);
         
         if(k > 0)
         {
            split_x1 -= extersion_lenth * costheta;
            split_y1 -= extersion_lenth * sintheta;
    	    split_x2 +=  extersion_lenth * costheta;
            split_y2 += extersion_lenth * sintheta;
         }
         else
         {
            split_x1 -= extersion_lenth * costheta;
            split_y1 += extersion_lenth * sintheta;
    	    split_x2 +=  extersion_lenth * costheta;
            split_y2 -= extersion_lenth * sintheta;
         }
     }while(0);

     *((float*)packet_buffer + 2) = split_x1 ;
     *((float*)packet_buffer + 3) = split_y1 ;
     *((float*)packet_buffer + 4) = split_x2;
     *((float*)packet_buffer + 5) = split_y2;
     printf("area_split: room id=%d, x0=%.2f, y0=%.2f, x1=%.2f, y1=%.2f\n",
                *((int*)packet_buffer + 1),*((float*)packet_buffer + 2),*((float*)packet_buffer + 3), *((float*)packet_buffer + 4),*((float*)packet_buffer + 5));
     robot_api_send_msg_to_hoslam(TUYA_MSG_ROOMS_SPLIT, (const char*)packet_buffer, size);
	
     return 0;
}

int wifi_app_msg_dp_parser_area_cutting(void *msg)
{
	get_editing_room_flag = false;
	robot_api_send_msg_to_hoslam(TUYA_MSG_ROOMS_GET_EDITING_ROOMS, NULL,0);
	usleep(250*1000);
	printf(" area_cutting, get_editing_room_flag:%d \n",get_editing_room_flag);
	
	uint8_t *pcmd_data = (uint8_t *)msg;
	uint8_t packet_buffer[WIFI_APP_DATA_PROTOCOL_PACKET_BUF];
	uint32_t datalen = 20;
	
	 *((int*)packet_buffer) = datalen;
    // åè®®å†…å®¹ cmd  |id| x1 | y1 | x2 | y2 |   ==  U8 |U8 | U8 | U16 | U16 | 
    uint32_t id = pcmd_data[1];
	*((int*)packet_buffer + 1) = id;
    printf("area_split: id=%d\n", id); //æˆ¿é—´åˆ†å‰²çš„id
    split_room = id;

	uint32_t size = sizeof(datalen) + sizeof(int) + 4 * sizeof(float);

    
    float dx1 = (int16_t)(pcmd_data[2] << 8 | pcmd_data[3]);
    float dy1 = (int16_t)(pcmd_data[4] << 8 | pcmd_data[5]);
	float dx2 = (int16_t)(pcmd_data[6] << 8 | pcmd_data[7]);
    float dy2 = (int16_t)(pcmd_data[8] << 8 | pcmd_data[9]);

	 //floatåž‹æ”¾å¤§10å€ä¼ è¾“
	 *((float*)packet_buffer + 2) = dx1 / 200.0;
     *((float*)packet_buffer + 3) = dy1 / 200.0;
	 *((float*)packet_buffer + 4) = dx2 / 200.0;
     *((float*)packet_buffer + 5) = dy2 / 200.0;
      printf("area_split: room id=%d, x0=%.2f, y0=%.2f, x1=%.2f, y1=%.2f\n",
                *((int*)packet_buffer + 1)                                    ,*((float*)packet_buffer + 2)*20,*((float*)packet_buffer + 3)*20, *((float*)packet_buffer + 4)*20,*((float*)packet_buffer + 5)*20);
      printf("area_split: room id=%d, x0=%.2f, y0=%.2f, x1=%.2f, y1=%.2f\n",
                *((int*)packet_buffer + 1),*((float*)packet_buffer + 2),*((float*)packet_buffer + 3), *((float*)packet_buffer + 4),*((float*)packet_buffer + 5));
      if( *((float*)packet_buffer + 2) < *((float*)packet_buffer + 4))
      {
        split_x1 = *((float*)packet_buffer + 2);
        split_y1 = *((float*)packet_buffer + 3);
        split_x2 = *((float*)packet_buffer + 4);
        split_y2 = *((float*)packet_buffer + 5);
      }
      else
      {
        split_x1 = *((float*)packet_buffer + 4);
        split_y1 = *((float*)packet_buffer + 5);
        split_x2 = *((float*)packet_buffer + 2);
        split_y2 = *((float*)packet_buffer + 3);
      }

	robot_api_send_msg_to_hoslam(TUYA_MSG_ROOMS_SPLIT, (const char*)packet_buffer, size);
	
    return 0;
}

int wifi_app_msg_dp_parser_area_merger(void *msg)
{
	get_editing_room_flag = false;
	robot_api_send_msg_to_hoslam(TUYA_MSG_ROOMS_GET_EDITING_ROOMS, NULL,0);
	usleep(250*1000);
	printf(" marea_merger, get_editing_room_flag:%d \n",get_editing_room_flag);
	uint8_t *pcmd_data = (uint8_t *)msg;
	uint8_t packet_buffer[WIFI_APP_DATA_PROTOCOL_PACKET_BUF];
	uint32_t datalen = 8;  //int ID int ID
	
	*((int*)packet_buffer) = datalen;

    // åè®®å†…å®¹cmd  |id1|id2   ==  U8 |U8 | 
    uint32_t id1 = pcmd_data[1];
	*((int*)packet_buffer + 1) = id1;
    printf("area_merger: id=%d\n", id1); //æˆ¿é—´åˆå¹¶id1

	uint32_t id2 = pcmd_data[2];
	*((int*)packet_buffer + 2) = id2;
    printf("area_merger: id=%d\n", id2); //æˆ¿é—´åˆå¹¶id2

	uint32_t size = sizeof(datalen) + 2 * sizeof(int);
     printf("area_merger: room id=%d, room2id=%d\n",
               *((int*)packet_buffer + 1),*((int*)packet_buffer + 2));

	robot_api_send_msg_to_hoslam(TUYA_MSG_ROOMS_MERGE, (const char*)packet_buffer, size);
	
    return 0;
}

///发送分区清扫的信息
int wifi_app_msg_dp_parser_area_inf(void * msg)
{

    BYTE_T *pbuffer_select_room = NULL;
    int select_room_size = get_select_room_inf(&pbuffer_select_room);

    OPERATE_RET op_ret = dev_report_dp_raw_sync(NULL, DPID_COMMAND, pbuffer_select_room, select_room_size, 0);
    if (OPRT_OK != op_ret)
    {
        PR_ERR("dev_report_dp_json_async op_ret:%d", op_ret);
    }

    free(pbuffer_select_room);
    pbuffer_select_room = NULL;

    return op_ret;

}


int wifi_app_msg_dp_parser_choice_rooms_clean(void *msg)
{
     uint8_t rooms_id[32];
     uint8_t *pcmd_data = (uint8_t *)msg;
     SelectedCleanRoom *p_select_room_arg = robot_ctl_get_select_room_arg();
     uint8_t clean_count = pcmd_data[1];	//int count| N |id
     printf("choice room clean clean_count :%d\n", clean_count);
     uint8_t room_count = pcmd_data[2];
     printf("choice room clean room_count:%d\n", room_count);
     if(room_count > 32)
     {
         printf("choice room clean room_count = %d\n",room_count);
         return -1;
     }

     p_select_room_arg->num = room_count; /* 参数缓存 */ 

     for(int i = 0;i < room_count;i++)
     {
        rooms_id[i] = pcmd_data[3 + i];
        p_select_room_arg->rooms[i].room_id = rooms_id[i];
        p_select_room_arg->rooms[i].clean_count = clean_count;
        p_select_room_arg->rooms[i].order = -1;
        printf("rooms_is = %d\n",rooms_id[i]);
     }
     set_choice_rooms_clean(clean_count,room_count,rooms_id);

     set_cur_scene(SCENE_CHOICE_ROOMS_CLEAN);
    //robot_api_control(ROBOT_CMD_CHOICE_ROOMS_CLEAN, NULL);
     s_task_sw = true;
     wifi_app_msg_dp_parser_area_inf(NULL);
}

/* 按房间清扫策略设置 */
int wifi_app_msg_dp_parser_room_data(void *msg)
{
	uint8_t *pcmd_data = (uint8_t *)msg;
    uint32_t i,j,k,h;
    room_spec_arg_t *p_rspec_arg;
    uint8_t ret;

    p_rspec_arg = robot_ctl_get_room_spec_arg(); /* 获取本地房间信息结构体 */
    k = 0;
    h = 3;
    for(i = 0; i < pcmd_data[1]; i ++) /* 更改的房间数量 */
    {
       for(j = 0; j < p_rspec_arg->num; j ++)
       {
           if(pcmd_data[2 + k] == p_rspec_arg->room_spec[j].id)
           {
               p_rspec_arg->room_spec[j].bundle[0] = wifi_app_data_map_fan_mode(pcmd_data[(h ++) + k]);
               p_rspec_arg->room_spec[j].bundle[1] = wifi_app_data_map_water_level(pcmd_data[(h ++) + k]);
               p_rspec_arg->room_spec[j].bundle[2] = !(pcmd_data[(h ++) + k]);
               p_rspec_arg->room_spec[j].count = pcmd_data[(h ++) + k];   
               p_rspec_arg->room_spec[j].bundle[4] = (uint8_t)p_rspec_arg->room_spec[j].count;      
               p_rspec_arg->room_spec[j].mop_count =  p_rspec_arg->room_spec[j].count;
               
                p_rspec_arg->room_spec[j].bundle[5] = 1;  /* 房间已被设置标志位 */
               k = k + 5; /* 指向数据包下一个房间ID */
               h = 3;  
               printf("id = %d,count = %d,sweep_forbidden = %d,fan = %d,mop_count = %d,forbidden = %d,water = %d,Y_mop = %d,cliff = %d count_bak = %d\n",
               p_rspec_arg->room_spec[j].id, p_rspec_arg->room_spec[j].count, p_rspec_arg->room_spec[j].forbidden, p_rspec_arg->room_spec[j].bundle[0],
                p_rspec_arg->room_spec[j].mop_count, p_rspec_arg->room_spec[j].mop_forbidden, p_rspec_arg->room_spec[j].bundle[1], p_rspec_arg->room_spec[j].bundle[2],
                p_rspec_arg->room_spec[j].bundle[3],p_rspec_arg->room_spec[j].bundle[4]);
               break;
           }
       }
    }
    send_hoslam_save_room();	 /* 通知hoslam更改的房间数据 */                               
    send_app_data_by_raw(0x23, &pcmd_data[1], pcmd_data[1] * 5 + 1);  /* 把更改后的数据返还给app*/
   
	//开启客户定制模式 将数据发送给380
	set_custom_mode_switch(true);
	iot_report_mode_switch(); 
    return 0;
}

int wifi_app_msg_dp_parser_rooms_name(void *msg)
{
#if 1
	 uint8_t *pcmd_data = (uint8_t *)msg;
     int i = 0,j = 0,k= 0;
     room_spec_arg_t *p_rspec_arg = robot_ctl_get_room_spec_arg();

     uint8_t room_num = pcmd_data[1];
     
     for(i = 0; i < room_num; i ++)
     {
        for(j = 0; j < p_rspec_arg->num; j ++)
        {
            if(pcmd_data[2 + k] == p_rspec_arg->room_spec[j].id)
            {
                memset(p_rspec_arg->room_spec[j].name_pack.name,0,ROOM_NAME_LEN);
                p_rspec_arg->room_spec[j].name_pack.name_len = pcmd_data[3 + k];
                memcpy(p_rspec_arg->room_spec[j].name_pack.name,&pcmd_data[4 + k],p_rspec_arg->room_spec[j].name_pack.name_len);
                k = k + 21;
                printf("room id = %d,name len = %d,name = %s\n",p_rspec_arg->room_spec[j].id,p_rspec_arg->room_spec[j].name_pack.name_len,p_rspec_arg->room_spec[j].name_pack.name);
                break;
            }
        }
     }
    send_hoslam_save_room();	 /* 通知hoslam更改的房间数据 */
    send_app_data_by_raw(0x25, &pcmd_data[1], room_num *21 + 1); /* 通知app更改成功 */
    return 0;
#endif
}


/* 定制房间清扫顺序 */
int wifi_app_msg_dp_parser_room_order(void *msg)
{
	uint8_t *pcmd_data = (uint8_t *)msg;
    uint32_t i,j,k;
    room_spec_arg_t *p_rspec_arg;
    p_rspec_arg = robot_ctl_get_room_spec_arg(); /* 获取本地房间信息结构体 */
    printf("set room order\n");
    k = 0;
    for(j = 0; j < p_rspec_arg->num; j ++)
    {
        p_rspec_arg->room_spec[j].order = -1;  /* 先全部重置一遍，然后再设置 */
    }
    for(i = 0; i < pcmd_data[1]; i ++) /* 更改的房间数量 */
    {
       for(j = 0; j < p_rspec_arg->num; j ++)
       {
           if(pcmd_data[2 + k] == p_rspec_arg->room_spec[j].id)
           {
             p_rspec_arg->room_spec[j].order = i;
             //p_rspec_arg->room_spec[j].order ++;
             printf("room id = %d,order = %d\n",p_rspec_arg->room_spec[j].id, p_rspec_arg->room_spec[j].order);
             break;
           }
       }
       k ++;
    }
    send_hoslam_save_room();	 /* 通知hoslam更改的房间数据 */
    send_app_data_by_raw(0x27, &pcmd_data[1], (pcmd_data[1] + 1)); /* 通知app更改成功 */
}

int wifi_app_msg_dp_parser_room_msg(void *msg)
{
	 //send_msg_to_hoslam_app(MSG_ROOMS_GET_EDITING_ROOMS, NULL,0);
	 uint8_t *pcmd_data = (uint8_t *)msg;
     int i;
     int id_data[256];

    uint32_t len = pcmd_data[1];	//int len|id| count| order| forbidden
    printf("room_msg: len=%d\n", len);
    

    for(i = 0;i < len ;i++)
    {
        id_data[i] = pcmd_data[i + 2];
        //printf("id_data[%d] = %d\n",i,id_data[i]);
    }

	set_room_msg(len,id_data);

    return 0;
}

int wifi_app_msg_dp_send_rooms_msg_ret(ROOM_CONTROL_STATE dp_enum)
{
    unsigned int dp_ret; 
    dp_ret = 1 << (dp_enum - 1);
    TY_OBJ_DP_S dp_data;
    dp_data.dpid = DPID_ROOM_MSG_RET;
    dp_data.type = PROP_BITMAP;
    dp_data.value.dp_bitmap = dp_ret;
    dp_data.time_stamp = 0;
     printf("dp_send_rooms_msg_ret = %d\n",dp_enum);
    printf("dp_send_rooms_msg_ret = %d\n",dp_ret);

    OPERATE_RET op_ret = dev_report_dp_json_async(NULL, &dp_data, 1);
    if(0 == op_ret)
    {
        dp_data.value.dp_bitmap = ROOMS_INITIALIZE;
        dev_report_dp_json_async(NULL, &dp_data, 1);
    }
    else
    {
        printf("wifi_app_msg_dp_send_map_ret error = %d\n",op_ret);
    }
}

int wifi_app_msg_dp_send_rooms_msg_ret_v3(ROOM_CONTROL_STATE dp_enum)
{
    uint8_t cmd,ret;
    switch(dp_enum)
    {
        case ROOMS_SPLIT_SUCESS:
             cmd = 0x1d;
             ret = 0x01;
        break;

        case ROOMS_SPLIT_FAILED:
             cmd = 0x1d;
             ret = 0x00;
        break;

        case ROOMS_MERGER_SUCESS:
             cmd = 0x1f;
             ret = 0x01;
        break;

        case ROOMS_MERGER_FAILED:
             cmd = 0x1f;
             ret = 0x00;
        break;

        case SEGMENT_SET_SUCCESS:
             cmd = 0x21;
             ret = 0x01;
        break;     
        case SEGMENT_SET_FAILED:
             cmd = 0x21;
             ret = 0x00;
        break; 

        default:
        break;  
    }
    send_app_data_by_raw(cmd,&ret,sizeof(ret));
}

/* 将数据打包成协议发送 */
int send_app_data_by_raw(uint8_t raw_cmd, uint8_t *data,uint8_t len)
{
    Raw_app_data_t *data_pack;
    uint8_t chk,size; 

    size = sizeof(Raw_app_data_t) + len + sizeof(chk); /* 获取协议总长度 */

    data_pack = (Raw_app_data_t *)malloc(size);
    if(data_pack == NULL)
    {
        printf("molloc data pack failed\n");
        return -1;
    }
    data_pack->head = RAW_APP_SEND_HEAD;   
    data_pack->version = RAW_APP_SEND_VERSION;
    data_pack->len = len + sizeof(data_pack->cmd);
    data_pack->cmd = raw_cmd;

    memcpy(((uint8_t *)data_pack + sizeof(Raw_app_data_t)) ,data,len); /* 将数据拷贝进去 */
    
    chk = get_check_sum((uint8_t *)(&data_pack->cmd),data_pack->len);

    ((uint8_t *)data_pack)[size - sizeof(chk)] = chk;  /* 将校验和补到最后一个字节中 */
    
    printf("raw cmd = 0x%x\n",data_pack->cmd);

    OPERATE_RET op_ret;

    if(raw_cmd == 0x33)   /* 判断是否勿扰时间段上报  */
    {
        op_ret = dev_report_dp_raw_sync(NULL, DPID_NOT_DISTURB_TIME, (uint8_t *)data_pack ,size, 0);
    }
    else
    {   
        op_ret = dev_report_dp_raw_sync(NULL, DPID_COMMAND, (uint8_t *)data_pack ,size, 0);
    }

    if (OPRT_OK != op_ret)
    {
        PR_ERR("dev_report_dp_json_async op_ret:%d", op_ret);
    }
    free(data_pack);
    return op_ret;
}

/* 将数据打包成地图管理协议发送 */
int send_app_data_by_map(uint8_t raw_cmd, uint8_t *data,uint32_t len)
{
    Map_app_data_t *data_pack;
    uint8_t chk,size; 
    OPERATE_RET op_ret;
    size = sizeof(Map_app_data_t) + len + sizeof(chk); /* 获取协议总长度 */

    data_pack = (Map_app_data_t *)malloc(size);
    if(data_pack == NULL)
    {
        printf("molloc data pack failed\n");
        return -1;
    }
    data_pack->head = 0xAB;   
    data_pack->version = RAW_APP_SEND_VERSION;
    data_pack->len = len + sizeof(data_pack->cmd);
    data_pack->len = htonl(data_pack->len);
    data_pack->cmd = raw_cmd;

    memcpy(((uint8_t *)data_pack + sizeof(Map_app_data_t)) ,data,len); /* 将数据拷贝进去 */
    
    chk = get_check_sum((uint8_t *)(&data_pack->cmd),(len + sizeof(data_pack->cmd)));

    ((uint8_t *)data_pack)[size - sizeof(chk)] = chk;  /* 将校验和补到最后一个字节中 */
    
    printf("raw cmd = 0x%x\n",data_pack->cmd);

    if(raw_cmd == 0x35)   /* 判断是否是语音包状态上报 */
    {
        op_ret = dev_report_dp_raw_sync(NULL, DPID_VOICE_DATA, (uint8_t *)data_pack ,size, 0);
    }
    else
    {
        op_ret = dev_report_dp_raw_sync(NULL, DPID_COMMAND, (uint8_t *)data_pack ,size, 0);
    }

    if (OPRT_OK != op_ret)
    {
        PR_ERR("dev_report_dp_json_async op_ret:%d", op_ret);
    }
    free(data_pack);
    return op_ret;
}

int  wifi_app_msg_dp_send_map_is_editable(uint8_t *data)
{
    int ret;
    unsigned int dp_enum; 
    TY_OBJ_DP_S dp_data;
    ret =  *((uint32_t*)data);
    printf("wifi_app_msg_dp_send_set_map_is_editable = %d\n",ret);
    if(0 == ret)
    {
        //wifi_app_msg_dp_send_rooms_msg_ret(ROOMS_IS_EDIT);
    }
    else if(-1 == ret)
    {
       // wifi_app_msg_dp_send_rooms_msg_ret(ROOMS_IS_NOT_EDIT);
    }
}

int wifi_app_msg_dp_send_set_choice_rooms_ret(uint8_t *data)
{
    int ret =  *((uint32_t*)data);
    printf("wifi_app_msg_dp_send_set_choice_rooms_ret = %d\n",ret);
    if(0 == ret)
    {
        wifi_app_msg_dp_send_rooms_msg_ret(ROOM_CHOIOCE_SUCESS);
    }
    else if(-1 == ret)
    {
       wifi_app_msg_dp_send_rooms_msg_ret(CHIOCE_NOT_FOUND);
    }
    else if(-2 == ret)
    {
       wifi_app_msg_dp_send_rooms_msg_ret(ROBOT_IS_CLEANING);
    }
    else if(-3 == ret)
    {  
       wifi_app_msg_dp_send_rooms_msg_ret(ROOM_COUNT);
    }
    if(ret != 0)
    {
       robot_ctl_reset_select_room_arg();
    }
}


#if 0
int wifi_app_msg_dp_parser_constituency_cleaning(void *msg)
{
	 //send_msg_to_hoslam_app(MSG_ROOMS_GET_EDITING_ROOMS, NULL,0);
	 uint8_t *pcmd_data = (uint8_t *)msg;
     int i;
     int id_data[256];

    uint32_t len = pcmd_data[1];	//int len|id| count| order| forbidden
    printf("constituency_cleaning: len=%d\n", len);
    

    for(i = 0;i < len ;i++)
    {
        id_data[i] = pcmd_data[i + 2];
        //printf("id_data[%d] = %d\n",i,id_data[i]);
    }

	set_new_room_fobidden(len,id_data);

    return 0;
}
#endif

// type
// void wifi_app_msg_dp_set_not_disturb_time(uint8_t *msg)
// {
//     uint8_t *pcmd_data = (uint8_t *)msg;
// }



int wifi_app_msg_dp_send_set_map_rusult_ret(uint8_t *data)
{
    /*int ret;
    unsigned int dp_enum; 
    TY_OBJ_DP_S dp_data;*/
   int  ret =  *((uint32_t*)data);
    printf("wifi_app_msg_dp_send_set_map_rusult_ret = %d\n",ret);
    if(0 == ret)
    {
        //dp_enum =  MAP_SET_SUCCESS;
        reset_last_data_robot_sta(); /* 复用地图成功之后，清空上一次的清扫状态，避免出现状态不对应的情况 */
        robot_api_send_msg_to_hoslam(TUYA_MSG_ROOMS_MAP_IS_EDITABLE, NULL, 0);
        robot_api_send_msg_to_hoslam(TUYA_MSG_ROOMS_GET_EDITING_ROOMS, NULL, 0);
        send_map_management(0x2f,1);
    }
    else
    {
        //dp_enum = MAP_SET_FAILED;
        send_map_management(0x2f,0);
    }

   	#if 0
   	dp_data.dpid = DPID_SET_MAP_RESULT;
    dp_data.type = PROP_ENUM;
    dp_data.value.dp_enum = dp_enum;
    dp_data.time_stamp = 0;

    OPERATE_RET op_ret = dev_report_dp_json_async(NULL, &dp_data, 1);
  	#endif
}

int wifi_app_msg_dp_send_vwall_foridden_ret(uint8_t *data)
{
    int ret;
    uint32_t type, result;
    unsigned int dp_enum; 
    TY_OBJ_DP_S dp_data;
    //ret =  *((uint32_t*)data);
    type = ((uint32_t*)data)[0];    // 1:VW 2:FZ
    result = ((uint32_t*)data)[1];  // 0:OK 1~:NG
    printf("wifi_app_msg_dp_send_vwall_foridden_ret = %d\n",ret);
    if(1 == type)
    {
        dp_data.dpid = DPID_VWALL_REUSULT;
        dp_enum = result;

    }
    else if(2 == type)
    {
        dp_data.dpid = DPID_FORBIDDEN_RESULT;
        dp_enum = result;
    }
    
    dp_data.type = PROP_ENUM;
    dp_data.value.dp_enum = dp_enum;
    dp_data.time_stamp = 0;

    OPERATE_RET op_ret = dev_report_dp_json_async(NULL, &dp_data, 1);
}

void *download_map_from_could( void*data)
{
    char recv_filename[128];
    char *map_id;
    char *url;
    char send_buffer[256];
    char filename[128];
    uint32_t id, size;
    int robot_is_running_flag;

    snprintf(recv_filename, MAP_FILE_PATH_MAX_LEN, "/tmp/tmp.tar.gz");
    snprintf(filename, MAP_FILE_PATH_MAX_LEN, "/tmp/hoslam_map");

    id = map_id_pack.robot_map_id;

    printf("id = %d\n",id);

    url = (char *)data ;

    printf("url = %s\n",url);

    sys_shell("touch \"%s\"",recv_filename);
    sys_shell("chmod 777 \"%s\"",recv_filename);
    sys_shell("curl  -o \"%s\" \"%s\" -k",recv_filename,url);
    sys_shell("gzip -d /tmp/tmp.tar.gz");
    sys_shell("tar -xf /tmp/tmp.tar -C /tmp");
    sys_shell("rm -f /tmp/tmp.tar");
    
    printf("filename = %s\n",filename);
    
    memset(send_buffer, 0, sizeof(send_buffer));
    size = sizeof(id) + sizeof(filename);
    memcpy(send_buffer, &size, sizeof(size));
    memcpy(send_buffer + sizeof(size), &id, sizeof(id));
    memcpy(send_buffer + sizeof(size) + sizeof(id),filename, sizeof(filename));
    //usleep(1000000);  //防止机器状态值未及时改变导致使用地图失败
    robot_api_get_robot_state(&robot_is_running_flag);
    if(robot_is_running_flag)
    {
        send_map_management(0x2f,0);
        sys_shell("rm -f /tmp/hoslam_map");
        printf("robot_is_running_flag = %d\n",robot_is_running_flag);
        return NULL;
    }
    robot_api_send_msg_to_hoslam(TUYA_MSG_TYPE_SET_CURRENT_MAP, send_buffer, sizeof(id) + sizeof(filename));

}


void creat_pthread_get_map_from_could(unsigned char *data)
{
    pthread_t get_map_pthread;
    int ret;
    pthread_attr_t attr;
    
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    ret = pthread_create(&get_map_pthread, &attr,download_map_from_could, data);
    if (ret)
    {
        printf("create thread_recv_upgrade_msg ERROR!\n");
        return ;
    }
    
}


//地图管理
int wifi_app_msg_dp_parser_map_msg(void *msg)
{
	TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;
    char *data = NULL;
    static char *url[6];
    static char send_url_data[6][256];
    static int order_flag;
    //static char save_time[32];
    int len;
    if(0 == strncmp(p_dp_obj->value.dp_str,"save",4))
    {
       // save_map_to_cloud(p_dp_obj->value.dp_str + 4);
       printf("wifi_app_msg_dp_parser_map_msg save\n");
             
       char *mapid = NULL;
       char *file_map = NULL;   
       mapid = strtok(p_dp_obj->value.dp_str,";");
       file_map = strtok(NULL,";");
       strcpy(time_save,file_map);
       len = strlen(time_save);
       printf("len = %d,save_time1 = %s\n",len,time_save);
       set_send_cloud_flag(1);
       printf("wifi_app_msg_dp_parser_map_msg save over\n");
       printf("%s,time= %s\n",mapid,file_map);
       robot_api_send_msg_to_hoslam(TUYA_MSG_TYPE_GET_CURRENT_MAP, NULL, 0);
        
    }
    else if(0 == strncmp(p_dp_obj->value.dp_str,"use",3))
    {
       printf("wifi_app_msg_dp_parser_map_msg use\n");
       data = (char *)p_dp_obj->value.dp_str + 4;
       //creat_pthread_get_map_from_could(data);
       char *mapid = NULL;
       char *order = NULL;
       char *index = NULL;
       char *file_map = NULL;
       int num;
       char send_url[5012];
       int i;
       
       mapid = strtok(data,";");
       order = strtok(NULL,";");
       order_flag = atoi(order);
       index = strtok(NULL,";");
       num = atoi(index);
       printf("num = %d\n",num);
       url[num] = strtok(NULL,";");
       printf("wifi_app_msg_dp_parser_map_msg use over\n");
       printf(" mapid = %s,order = %s,index = %s,\n",mapid,order,index);
       //strncpy(send_url_data[num],url[num],256);
       sprintf(send_url_data[num],"%s",url[num]);
       printf("url = %s\n",send_url_data[num]);
       if(num == (order_flag - 1))
        {
           sprintf(send_url,"%s;%s",mapid,send_url_data[0]);
           //printf("send_url = %s%s\n",url[2],url[3]);
           for(i = 1;i < order_flag;i++)
           {
                sprintf(send_url,"%s%s",send_url,send_url_data[i]);
           }
           sprintf(send_url,"%s;",send_url);
           printf("send_url = %s\n",send_url);
           creat_pthread_get_map_from_could(send_url);
        }
    }
    else if(0 == strncmp(p_dp_obj->value.dp_str,"delete",6))
    {
        printf("wifi_app_msg_dp_parser_map_msg delete\n");
    }
	//send_hoslam_set_rooms(id,count,order,forbidden);

    return 0;
}



int wifi_app_msg_dp_send_current_map_updated(void)
{
    //unsigned int id;
    bool dp_enum; 
    TY_OBJ_DP_S dp_data;
    //id = *((uint32_t*)data);
   // printf("wifi_app_msg_dp_send_map_id = %d\n",id);
  
    dp_enum =  0;
    dp_data.dpid = DPID_CLEAN_UP;
    dp_data.type = PROP_VALUE;
    dp_data.value.dp_bool = dp_enum;
    dp_data.time_stamp = 0;

    OPERATE_RET op_ret = dev_report_dp_json_async(NULL, &dp_data, 1);
}

/* 地图管理回应函数 */
int send_map_management(uint8_t send_cmd,uint8_t ret)
{
    send_app_data_by_map(send_cmd,&ret,1);
}

/****************************************************************
*Function   :  send_voice_loading_ret
*Author     :  zcw    
*Date       :  2020.12.30
*Description:  下载语音包回应函数
*CallBy     :  任何地方
*Input      : language_id：语音版本（字符串形式）（x.x.x.x）
              status:下载状态
              schedu:下载进度（0 - 100）
*Output     :  无
*Return     :  无
******************************************************************/
int send_voice_loading_ret(uint8_t * language_id,uint8_t status,uint8_t schedu)
{
    uint8_t send_buffer[6];

    sscanf(language_id,"%d.%d.%d.%d",&send_buffer[0],&send_buffer[1],&send_buffer[2],&send_buffer[3]);
    send_buffer[4] = status;
    send_buffer[5] = schedu;
    printf("send voice loading ret: ");
    for(uint8_t i = 0; i < 6; i ++)
    {
        printf("%x,",send_buffer[i]);
    }
    printf("\n");
    send_app_data_by_map(0x35,send_buffer,6);

}

int wifi_app_msg_dp_send_map_id(void)
{

    printf("wifi_app_msg_dp_send_map_id = %d\n",map_id);
    
        // 协议内容 head | cmd | map_id
    uint8_t head = 0xAB;
    uint8_t cmd = 0x70;

    uint32_t size = sizeof(head) + sizeof(cmd) + sizeof(map_id);

    BYTE_T* pbuf = (BYTE_T*)malloc(size);
    if(pbuf == NULL) {
        printf("send_map_management %d malloc failed\n", size);
        return -1;
    }
    memset(pbuf, 0, size);
    pbuf[0] = head;
    pbuf[1] = cmd;
    pbuf[2] =(uint8_t)(( map_id & 0xff000000) >> 24);
    pbuf[3] =(uint8_t)((map_id & 0x00ff0000) >> 16);
    pbuf[4] =(uint8_t)(( map_id & 0x0000ff00) >> 8);
    pbuf[5] =(uint8_t)(map_id & 0x000000ff);
    printf("low byte [2] = %hhu,low byte [3] = %hhu,low byte [4] = %hhu,low byte [5] = %hhu,",pbuf[2],pbuf[3], pbuf[4],pbuf[5] );
    
    OPERATE_RET op_ret = dev_report_dp_raw_sync(NULL, DPID_COMMAND, pbuf, size, 0);
    printf("op_ret = %d\n",op_ret);

    free(pbuf);
    return 0;

}

int save_map_id(uint8_t *data)
{
    unsigned int id;
    unsigned int dp_enum; 
    TY_OBJ_DP_S dp_data;
    map_id = *((uint32_t*)data);
    printf("wifi_app_msg_dp_save_map_id = %d\n",map_id);
}

int wifi_app_msg_dp_parser_map_save(void *msg)
{
    uint8_t *pcmd_data = (uint8_t *)msg;
    printf("save space = %d\n",pcmd_data[6]);
    set_send_cloud_flag(1);
    robot_api_send_msg_to_hoslam(TUYA_MSG_TYPE_GET_CURRENT_MAP, NULL, 0);
}

void is_delete_last_save_map(int delete_tuya_map_id)
{

}

int wifi_app_msg_dp_parser_map_delete(void *msg)
{
    uint8_t *pcmd_data = (uint8_t *)msg;

    uint32_t delete_map_id = (uint32_t)(pcmd_data[6] << 24 | pcmd_data[7] << 16 | pcmd_data[8] << 8 | pcmd_data[9]);

    printf("delete map_id = %d\n",delete_map_id);
    
    OPERATE_RET op_ret = tuya_iot_map_delete(delete_map_id);
    if(OPRT_OK == op_ret)
    {
        delete_map_node(g_p_map_head,-1,delete_map_id);
        save_robot_map_to_file(g_p_map_head);
        send_map_management(0x2d,1);
    }
    else
    {
        printf("wifi_app_msg_dp_parser_map_delete error= %d\n",op_ret);
        send_map_management(0x2d,0);
    }
    
}

int wifi_app_msg_dp_parser_map_use(void *msg)
{
	
#if 0
	 robot_ctl_scene_switch(m_cur_scene, SCENE_SWITCH_STOP);
#else
	//充电情况下发送停止命令，380 会播 按键无效语音
	printf("\033[1m\033[40;33m m_cur_scene:%d \033[0m\n",m_cur_scene);
	if(m_cur_scene != SCENE_DOCK && m_cur_scene != SCENE_CHARGING)
	{
		robot_ctl_scene_switch(m_cur_scene, SCENE_SWITCH_STOP);
	}
#endif

    static unsigned char send_url[1024];
    uint8_t *pcmd_data =(uint8_t *) msg;
    uint32_t length =  (uint32_t) ((pcmd_data[1] << 24)| (pcmd_data[2]<< 16) |(pcmd_data[3] << 8)| pcmd_data[4]);

    pcmd_data = (uint8_t *)(&pcmd_data[6]);  /* 数据起始地址 */

    map_id_pack.robot_map_id = (uint32_t)(pcmd_data[0] << 24 | pcmd_data[1] << 16 | pcmd_data[2] << 8 | pcmd_data[3]);
    printf("robot_map_id = %d\n",map_id_pack.robot_map_id);

    uint32_t url_len = (uint32_t)(pcmd_data[4] << 24 | pcmd_data[5] << 16 | pcmd_data[6] << 8 | pcmd_data[7]);
    printf("length = %d,url_len = %d\n",length,url_len);
    if(url_len >= 1024)
    {
        printf("map url length > 1024 bytes\n");
        return -1;
    }
    memset(send_url,0,1024);
    memcpy(send_url,&pcmd_data[8],url_len);
    creat_pthread_get_map_from_could(send_url);
}

int wifi_app_voice_file_update(const char *file, const char *version, const char *uuid);
void *download_voice_pack_from_could(void* arg)
{
	url_md5_voiceID *data; 
    uint8_t temp = 0,i = 0,j = 0;
    char recv_filename[128];
    int ret;
    
    send_voice_loading_ret(pstru.voice_edition_id,GET_VOICE_LOADING,20);
	data = malloc(sizeof(url_md5_voiceID));
    if(data == NULL)
    {
        printf("malloc data failed\n");
        send_voice_loading_ret(pstru.voice_edition_id,GET_VOICE_FAILED,0);
        return NULL;
    }
    memcpy(data,&pstru,sizeof(url_md5_voiceID));
   	memset(recv_filename, 0, sizeof(recv_filename));

    snprintf(recv_filename, MAP_FILE_PATH_MAX_LEN, "/tmp/tmp.tar.gz");
    printf("data->url = %s\n",data->url);

    sys_shell("touch \"%s\"",recv_filename);
    sys_shell("chmod 777 \"%s\"",recv_filename);
	sys_shell("curl  -o \"%s\" \"%s\" -k",recv_filename,data->url);//阻塞  

    send_voice_loading_ret(pstru.voice_edition_id,GET_VOICE_LOADING,60);

    printf("data->url = %s\n",data->url);
    printf("data->md5 = %s\n",data->md5);

	ret = sys_file_check_md5(recv_filename,data->md5); /* MD5校验 */  
	printf("ret = %d\n",ret);

	if(!ret)
	{
		printf("sys_file_check_md5 fail\n");
        free(data);
        send_voice_loading_ret(pstru.voice_edition_id,GET_VOICE_FAILED,0);
		return NULL;
	}	

    ret = wifi_app_voice_file_update(recv_filename, data->voice_edition_id, NULL); /* 安装语音包 */
    if(ret == 1)
    {
        set_fixed_voice_path(); /*设置固定的语音播放路径，防止播放路径错误，导致语音无法播放 */
        robot_api_play_voice(7); /* 语音包安装成功后，播放开机语音提示成功，开机语音默认ID是7 */
        send_voice_loading_ret(pstru.voice_edition_id,GET_VOICE_SUCCESS,100);
        send_voice_loading_ret(pstru.voice_edition_id,VOICE_USEING,100);
    }
    else
    {
        send_voice_loading_ret(pstru.voice_edition_id,GET_VOICE_FAILED,0);
    }
    free(data);

	return NULL;
}


int wifi_app_msg_dp_parser_voice_pack_download(void *msg)
{  	
  	uint32_t length = 0; 
    uint8_t *pcmd_data = (uint8_t *)(msg + 5);
    
	memset(&pstru, 0 ,sizeof(url_md5_voiceID));
    
    length = sprintf(pstru.voice_edition_id,"%d.%d.%d.%d",pcmd_data[1],pcmd_data[2],pcmd_data[3],pcmd_data[4]);
    pstru.voice_edition_id[length] = '\0';   /* 获取语音版本号 */

    printf("pstru.voice_edition_id = %s\n",pstru.voice_edition_id);

    memcpy(pstru.md5,&pcmd_data[6],32);   /* 获取MD5校验 */
    pstru.md5[32] = '\0';
    printf("pstru.md5 = %s\n",pstru.md5);

    pstru.url_len = (uint32_t)((pcmd_data[38] << 24) | (pcmd_data[39] << 16) | (pcmd_data[40] << 8) | (pcmd_data[41]));
    memcpy(pstru.url,&pcmd_data[42],pstru.url_len); /* 获取URL */
    printf("pstru.ur = %s\n",pstru.url);
    
	//创建线程下载语音包
    pthread_t get_map_pthread;
    int ret;
    pthread_attr_t attr;
    
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	
    ret = pthread_create(&get_map_pthread, &attr,download_voice_pack_from_could, NULL);
    if (ret)
    {
        printf("create thread_recv_upgrade_msg ERROR!\n");
        send_voice_loading_ret(pstru.voice_edition_id,GET_VOICE_FAILED,0);
    }
	
	return ret;

}

/* 上报勿扰时间段给app */
void report_dnd_mode_time(uint8_t time_zone,uint8_t start_h,uint8_t start_m,uint8_t end_h,uint8_t end_m)
{
    uint8_t send_data[7];

    send_data[0] = time_zone;
    send_data[1] = start_h;
    send_data[2] = start_m;
    send_data[3] = 0;
    send_data[4] = end_h;
    send_data[5] = end_m;
    send_data[6] = 0;
    send_app_data_by_raw(0x33, &send_data[0],7); /* 将勿扰时间上报给app */
}

int wifi_app_msg_dp_parser_dnd_mode_time(void *msg)
{
    uint8_t *pcmd_data =(uint8_t *) msg;
    uint8_t send_380_app[5];
    uint8_t time_zone = pcmd_data[1];
    uint8_t start_time_h = pcmd_data[2];
    uint8_t start_time_m = pcmd_data[3];
    uint8_t end_time_h = pcmd_data[5];
    uint8_t end_time_m = pcmd_data[6];

    printf("time_zone:%d, start_time:%d_%d, end_time:%d_%d\n", time_zone, start_time_h,start_time_m,end_time_h,end_time_m);

    send_380_app[0] = time_zone;
    send_380_app[1] = start_time_h;
    send_380_app[2] = start_time_m;
    send_380_app[3] = end_time_h;
    send_380_app[4] = end_time_m;

    robot_api_send_packet(CMD_DND_MODE_TIME, send_380_app, 5);	

    report_dnd_mode_time(time_zone,start_time_h,start_time_m,end_time_h,end_time_m);

	return 0;
}

//边刷寿命重置
static int wifi_app_msg_dp_parser_edge_brush_life_reset(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;
    int edge_brush_value = 100 , ret = -1;
    if (p_dp_obj->type != PROP_BOOL)
    {
        printf("ERORR dp type! p_dp_obj->type = %d\n",p_dp_obj->type);
        return -1;
    }
	if(!p_dp_obj->value.dp_bool)
	{
		return 0;
	}

    robot_api_control(ROBOT_EDGE_BRUSH_LIFE, (void*)edge_brush_value);
    return 0;
}

//滚刷寿命重置
static int wifi_app_msg_dp_parser_roll_brush_life_reset(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;
    int roll_brush_value = 100 , ret = -1;
    if (p_dp_obj->type != PROP_BOOL)
    {
        printf("ERORR dp type! p_dp_obj->type = %d\n",p_dp_obj->type);
        return -1;
    }
	if(!p_dp_obj->value.dp_bool)
	{
		return 0;
	}

    robot_api_control(ROBOT_ROLL_BRUSH_LIFE, (void*)roll_brush_value);
    return 0;
}


//滤芯寿命重置
static int wifi_app_msg_dp_parser_filter_life_reset(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;
    int filter_value = 100 , ret = -1;
    if (p_dp_obj->type != PROP_BOOL)
    {
        printf("ERORR dp type! p_dp_obj->type = %d\n",p_dp_obj->type);
        return -1;
    }
	if(!p_dp_obj->value.dp_bool)
	{
		return 0;
	}

    robot_api_control(ROBOT_FILTER_LIFE, (void*)filter_value);
    return 0;
}


//拖布寿命重置
static int wifi_app_msg_dp_parser_mop_life_reset(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;
    int mop_value = 100 , ret = -1;
    if (p_dp_obj->type != PROP_BOOL)
    {
        printf("ERORR dp type! p_dp_obj->type = %d\n",p_dp_obj->type);
        return -1;
    }
	if(!p_dp_obj->value.dp_bool)
	{
		return 0;
	}

    robot_api_control(ROBOT_MOP_LIFE, (void*)mop_value);
    return 0;
}

int wifi_sent_all_state(void)
{
      iot_report_robot_all_state();
      wifi_app_msg_dp_parser_get_all_config(NULL);
      wifi_app_msg_dp_send_map_id();
}

int send_goto_pos_flag;

static int wifi_app_msg_dp_parser_commflag(void *msg)
{
    TY_OBJ_DP_S *p_dp_obj = (TY_OBJ_DP_S *)msg;

    if (p_dp_obj->type != PROP_ENUM)
    {
        printf("ERORR dp type!\n");
        return -1;
    }

    // 简单通讯：请求地图/路径
    int protocol_enum = p_dp_obj->value.dp_enum;
    if (protocol_enum != INVALID_ENUM)
    {
        switch (protocol_enum)
        {
        case COMMFLAG_GETMAP:
        {

        }
        break;
        case COMMFLAG_INMAP:  // 首次进入APP发，之后1min频率
            pthread_mutex_lock(&mtx_mapheart);
            getmapheart++;
            pthread_mutex_unlock(&mtx_mapheart);
            //send_map_rooms_data();
            if(send_goto_pos_flag)
            {
                app_get_send_navigation();
            }
            robot_api_send_msg_to_hoslam(TUYA_MSG_TYPE_GET_VIRTUAL_WALL,NULL,0);
            printf("inmap  getmap\n");
            break;

        case  COMMFLAG_RESET_MAP:  // 重置地图
            printf("COMMFLAG_RESET_MAP start\n");
            robot_api_control(ROBOT_CMD_RESET_MAP, NULL);
            printf("COMMFLAG_RESET_MAP\n");
            break;

       case  COMMFLAG_GET_STATUS:  // 获取机器dp状态值
            printf("COMMFLAG_GET_STATUSt\n");
            wifi_sent_all_state();
            //printf("COMMFLAG_RESET_MAP\n");
            break;

       case COMMFLAG_GET_MAP_ID:    //获取mapid
            printf("COMMFLAG_GET_MAP_ID\n");
            robot_api_send_msg_to_hoslam(TUYA_MSG_TYPE_GET_CURRENT_MAP, NULL, 0);
            break;
        default:
            break;
        }
    }
}

/* 根据app下发消息强制上报一次数据 */
int get_force_update_state(void)
{
    return g_force_update_state;
}

void set_force_update_state(int state)
{
    g_force_update_state = state;
}

static MSG_PARSER wifi_app_msg_dp_parsers[] =
{
    {DPID_POWER_GO,             wifi_app_msg_dp_parser_task_switch          },
    {DPID_SWITCH_CHARGE,        wifi_app_msg_dp_parser_charge_switch        },
    {DPID_MODE,                 wifi_app_msg_dp_parser_work_mode            },
    {DPID_DIRECTION_CONTROL,    wifi_app_msg_dp_parser_manual_control       },
    {DPID_CALLING_ROBOT,        wifi_app_msg_dp_parser_calling_robot        },
    {DPID_SUCTION,              wifi_app_msg_dp_parser_fan_mode             },
    {DPID_PAUSE,                wifi_app_msg_dp_parser_clean_switch         },
    {DPID_WATER_MODE,           wifi_app_msg_dp_parser_water_level          },
    {DPID_REQUEST,              wifi_app_msg_dp_parser_commflag             },
    {DPID_RESET_MAP,            wifi_app_msg_dp_parser_reset_map            },
    {DPID_SET_VOLUME,           wifi_app_msg_dp_parser_robot_config         },
    {DPID_OPEN_NOT_DISTURB,     wifi_app_msg_dp_parser_not_disturb          },
    {DPID_SMART_ROOMS,          wifi_app_msg_dp_parser_smart_rooms          },
    {DPID_MAP_MANAGEMENT,       wifi_app_msg_dp_parser_map_msg              },
    {DPID_CONTINUE_CLEAN,       wifi_app_msg_dp_parser_continue_clean       },
    {DPID_Y_MOP,                wifi_app_msg_dp_parser_y_mop                },
    {DPID_CUSTOM_MODE_SWITCH,   wifi_app_msg_dp_parser_mode_switch          },
	{DPID_EDGE_BRUSH_LIFE_RESET,wifi_app_msg_dp_parser_edge_brush_life_reset},
	{DPID_ROLL_BRUSH_LIFE_RESET,wifi_app_msg_dp_parser_roll_brush_life_reset},
	{DPID_FILTER_LIFE_RESET,   	wifi_app_msg_dp_parser_filter_life_reset	},
	{DPID_MOP_LIFE_RESET,   	wifi_app_msg_dp_parser_mop_life_reset  		},
};

int wifi_app_msg_parse(const TY_OBJ_DP_S *msg)
{
    int i;
    int ret = 0;
    int max = sizeof(wifi_app_msg_dp_parsers) / sizeof(MSG_PARSER);

    for (i = 0; i < max; i++)
    {
        if (wifi_app_msg_dp_parsers[i].type == msg->dpid)
        {
        	printf(" ** rev msg: %d\r\n", msg->dpid);
            ret = wifi_app_msg_dp_parsers[i].parser((void *)msg);
            set_force_update_state(1);
            break;
        }
    }

    return ret;
}

// static MSG_PARSER wifi_app_msg_raw_parsers[] =
// {
//     {DPID_GOTO_POSITION,        wifi_app_msg_dp_parser_goto_position        },
//     {DPID_CLEAN_ZONE,           wifi_app_msg_dp_parser_clean_zone           },
//     {DPID_FORBIDDEN_ZONE,       wifi_app_msg_dp_parser_forbidden_zone       },
// };


int wifi_app_map_raw_exectue(const unsigned char *msg)
{
    int ret = 0;

    uint8_t ucmd = msg[5];
    uint8_t chk = 0;
    printf("map ucmd = %x\n",ucmd);
    
    switch (ucmd)
    {
        case 0x2a:{ret = wifi_app_msg_dp_parser_map_save((void *)msg);break;}
        case 0x2c:{ret = wifi_app_msg_dp_parser_map_delete((void *)msg);break;}
        case 0x2e:{ret = wifi_app_msg_dp_parser_map_use((void *)msg);break;}
        case 0x34:{ret = wifi_app_msg_dp_parser_voice_pack_download((void *)msg);break;}
        default:break;
    }

    return ret;
}

int wifi_app_raw_exectue(const unsigned char *msg)
{
    int ret = 0;

    uint8_t ucmd = msg[0];
    printf(" raw ucmd = 0x%x\n",ucmd);

    switch (ucmd)
    {
        case 0x16:{ret = wifi_app_msg_dp_parser_goto_position((void *)msg);break;}
        case 0x10:{ret = wifi_app_msg_dp_parser_clean_zone((void *)msg);break;}
        case 0x29:{ret = wifi_app_msg_dp_parser_get_clean_zone((void *)msg);break;}
        case 0x18:{ret = wifi_app_msg_dp_parser_forbidden_zone((void *)msg);break;}
        case 0x19:{ret = wifi_app_msg_dp_parser_get_forbidden_zone((void *)msg);break;}
		case 0x1a:{ret = wifi_app_msg_dp_parser_forbidden_zone_new((void *)msg);break;}
		case 0x1b:{ret = wifi_app_msg_dp_parser_get_forbidden_zone_new((void *)msg);break;}
        case 0x12:{ret = wifi_app_msg_dp_parser_virtual_wall((void *)msg);break;}
        case 0x13:{ret = wifi_app_msg_dp_parser_get_virtual_wall((void *)msg);break;}
        case 0x28:{ret = wifi_app_msg_dp_parser_type_clean_zone((void *)msg);break;}
        case 0x30:{ret = wifi_app_msg_dp_parser_get_all_config((void *)msg);break;}
		case 0x1c:{ret = wifi_app_msg_dp_parser_area_cutting((void *)msg);break;}
		case 0x1e:{ret = wifi_app_msg_dp_parser_area_merger((void *)msg);break;}
       // case 0x33:{ret = wifi_app_msg_dp_parser_room_msg((void *)msg);break;}
        case 0x35:{ret = wifi_app_msg_dp_parser_room_msg((void *)msg);break;}
        case 0x14:{ret = wifi_app_msg_dp_parser_choice_rooms_clean((void *)msg);break;}
        case 0x15:{ret = wifi_app_msg_dp_parser_area_inf((void *)msg);break;}
        case 0x20:{ret = wifi_app_msg_dp_parser_segment((void *)msg);break;}
        case 0x22:{ret = wifi_app_msg_dp_parser_room_data((void *)msg);break;}
        case 0x24:{ret = wifi_app_msg_dp_parser_rooms_name((void *)msg);break;}
        case 0x26:{ret = wifi_app_msg_dp_parser_room_order((void *)msg);break;}
        case 0x32:{ret = wifi_app_msg_dp_parser_dnd_mode_time((void *)msg);break;}
        default:break;
    }

    return ret;
}

int wifi_app_raw_parse(const TY_RECV_RAW_DP_S *msg)
{
    int PROTOL_HEAD_BYTES = 3; //协议头字节数 Head + sersion + Len
    int PROTOL_LENH_NO = 2;    //Len字段位置

    int ret = -1;

    int i = 0;
    int icur = 0;
    int ilen = 0;
    uint8_t chk = 0;

    while (i < msg->len)
    {
        icur = msg->data[i];
       // printf("icur = %hhu\n",icur);
        if (msg->data[i] != RAW_APP_SEND_HEAD && msg->data[i] != 0xAB)
        {
            i++;
            continue;
        }
        if(msg->data[i] == 0xAB)
        {
            printf("msg = 0xAB\n");
            wifi_app_map_raw_exectue(&(msg->data[i + 1]));
            break;
        }
        ilen = msg->data[i + PROTOL_LENH_NO];

        if (i + ilen + PROTOL_HEAD_BYTES > msg->len)
        {
            printf("i + ilen + PROTOL_HEAD_BYTES = %d,msg->len = %d\n",i + ilen + PROTOL_HEAD_BYTES,msg->len);
            printf("i = %d,ilen = %d,PROTOL_HEAD_BYTES = %d\n",i,ilen,PROTOL_HEAD_BYTES);
            break;
        }

        chk = get_check_sum((unsigned char *)(msg->data + i + PROTOL_HEAD_BYTES), ilen);
        if (get_check_sum((unsigned char *)(msg->data + i + PROTOL_HEAD_BYTES), ilen) != msg->data[i + PROTOL_HEAD_BYTES + ilen])
        {
            //校验出错
            i = i + PROTOL_HEAD_BYTES + ilen + 1;
            printf("Error Communicaiton Raw Data check num error chk = %d\n",chk);
            continue;
        }

        BYTE_T *pcmdmsg = (BYTE_T *)malloc(ilen);
        memset(pcmdmsg, 0, ilen);

        memcpy((char *)pcmdmsg, (const char *)msg->data + i + PROTOL_HEAD_BYTES, ilen);

        ret = wifi_app_raw_exectue(pcmdmsg);

        free(pcmdmsg);
        pcmdmsg = NULL;

        i = i + PROTOL_HEAD_BYTES + ilen + 1;
    }

    return ret;
}

// 清空路径
void wifi_app_reset_path()
{
    SEND_PATH send_path;
	memset(&send_path, 0, sizeof(send_path));
    send_path.head = 1;
    send_path.flag = 0;
    send_path.num = 0;
    send_path.lz4len = 0;

    int clen = sizeof(SEND_PATH);
    CHAR_T * pbufpath = (CHAR_T *)malloc(clen);
    memset(pbufpath, 0, clen);

    send_path.head = htons(send_path.head);
    send_path.num = htonl(send_path.num);
    send_path.direction = htons(send_path.direction);
    send_path.lz4len = htons(send_path.lz4len);

    OPERATE_RET op_ret = tuya_iot_upload_route_buffer(send_path.head, pbufpath, clen);
    if (OPRT_OK != op_ret)
    {
        PR_ERR("upload path  tuya_iot_upload_total_data op_ret:%d", op_ret);
    }
    free(pbufpath);
    printf("---clear path data\n");
}

// 清空导航线
void wifi_app_reset_navi()
{
    wifi_app_msg_dp_parser_get_all_config(NULL);
	
    SEND_PATH send_navi;
	memset(&send_navi, 0, sizeof(send_navi));
	
    send_navi.head = 1;
    send_navi.flag = 3;
    send_navi.num = 0;
    send_navi.lz4len = 0;
    
    OPERATE_RET op_ret = tuya_iot_upload_navigation_buffer(send_navi.head, (CHAR_T *)&send_navi, sizeof(send_navi));
    if (OPRT_OK != op_ret)
    {
        PR_ERR("upload path  tuya_iot_upload_total_data op_ret:%d", op_ret);
    }
    printf("clear all map path navi\n");

}

// 清空地图
void wifi_app_reset_map()
{
    SEND_MAP send_map;
	memset(&send_map, 0, sizeof(send_map));

    send_map.head = 1;
    send_map.flag = 0x01;
    send_map.type = 0x00;
    send_map.w  = 0;
    send_map.h  = 0;
    send_map.ox = 0;
    send_map.oy = 0;
    send_map.resolution = 5;
    send_map.charger_x = 0;
    send_map.charger_y = 0;
    send_map.lz4len = 0;

    uint32_t clen = sizeof(SEND_MAP) ;
    CHAR_T *pbuffer = (CHAR_T *)malloc(clen);
    memset(pbuffer, 0, clen);

    send_map.head = htons(send_map.head);
    send_map.w = htons(send_map.w);
    send_map.h = htons(send_map.h);
    send_map.ox = htons(send_map.ox);
    send_map.oy = htons(send_map.oy);
    send_map.resolution = htons(send_map.resolution);
    send_map.charger_x = htons(send_map.charger_x);
    send_map.charger_y = htons(send_map.charger_y);
    send_map.len_before_lz4 = htonl(send_map.len_before_lz4);
    send_map.lz4len = htons(send_map.lz4len);

    memcpy(pbuffer, (char *)&send_map, sizeof(SEND_MAP));
    if(iot_state_cloud_ready())
    {
        OPERATE_RET op_ret = tuya_iot_upload_layout_buffer(send_map.head, pbuffer, clen);
        if (OPRT_OK != op_ret)
        {
            PR_ERR("upload map tuya_iot_upload_total_data op_ret:%d", op_ret);
        }
    }
    free(pbuffer);

    printf("---clear map data\n");
}



int wifi_app_reset_robot_map_data()
{
    wifi_app_reset_path();

    wifi_app_reset_navi();

    _reset_all_arg();
    
    wifi_app_msg_dp_parser_get_all_config(NULL);

    return 0;
}

/* 清扫完成后清导航线、导航点、划区框 */
int wifi_app_reset_clean_data(STM_CLEAN_ARG arg)
{

    if(if_need_reset_tmp_arg())
    {
        //wifi_app_reset_navi();
        _reset_tmp_arg(arg);
        wifi_app_msg_dp_parser_get_all_config(NULL);
    }

    return 0;
}
