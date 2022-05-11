#include <unistd.h>
#include <stdbool.h>
#include <pthread.h>
#include "robot_sys_msg.h"
#include "robot_app_msg.h"
#include "robot_control_adapter.h"
#include "tuya_iot_state_adapter.h"
#include "uni_network.h"
#include "tuya_hal_wifi.h"
#include "tuya_iot_msg_parse.h"
#include "../../robot_api_src/sys_semaphore.h"
#include "../debug/debug_tool.h"

#define DEBUG_ROBOT_STM   1

#if DEBUG_ROBOT_STM
    #define STM_LOG(...)      printf(__VA_ARGS__)
#else
    #define STM_LOG(...)
#endif
//========================参数缓存=======================
static defined_pose_arg_t s_goto_pos_arg;
static defined_rects_arg_t s_rects_arg;
static virtual_wall_arg_t s_vwall_arg;
static forbidden_arg_t s_fb_arg;
static room_spec_arg_t s_room_arg;
static SelectedCleanRoom s_select_room_arg;
//======================================================
static bool upload_virtual_wall_flag = false;

#if DEBUG_ROBOT_STM
static char *debug_scene_str[] = {
    "autoCleanScene",
    "dockScene",
    "gotoPosScene",
    "spotCleanScene",
    "rectCleanScene",
    "chargingScene",
    "wfCleanScene",
    "choiceclean"
};

static char *debug_sw_str[] = {
    "start",
    "pause",
    "resume",
    "stop"
};
#endif

static int save_rooms_flag;
void wait_wifi_load(void)
{
    NW_IP_S ip;
    ip.ip[0] = 0;
    while(1) 
    {
        tuya_hal_wifi_get_ip(WF_STATION, &ip);
        printf("wait ip:%s\n",ip.ip);
        if(ip.ip[0] == 0) 
        {
            sleep(1);
        } 
        else 
        {
            break;
        }
    }
}
//等待机器人准备好，wifi驱动加载完成，获取固件版本号
char *robot_ctl_wait_robot_ready(void) {
static char *def_ver = "1.0.0";
    NW_IP_S ip;
    int count = 0;
    char *fw_ver = robot_app_fw_version();
    while(fw_ver[0] == 0 && ++count < 20) {
        sleep(1);
    }
    if(fw_ver[0] == 0) {
        fw_ver = def_ver;
        printf("wait fw_ver time out %d\n", count);
    }
 /*
    while(hwl_wf_get_mac(WF_AP, &mac) == OPRT_COM_ERROR) {
        printf("waiting mac ready\n");
        sleep(1);
    }
*/
    count = 0;
    ip.ip[0] = 0;
    while(1) 
    {
        tuya_hal_wifi_get_ip(WF_STATION, &ip);
        if(ip.ip[0] == 0) 
        {
            sleep(1);
            count ++;
            if(count == 40) /* 40S超时，向380报错 */
            {
               inform_robot_wifi_state(WIFI_STATE_FAULT); 
            }
        } 
        else 
        {
            break;
        }
    }
    //这时候可以获取hoslam虚拟墙
    robot_api_request_virtual_wall();
    printf("fw_ver:%s, def_ver:%s ip:%s\n", fw_ver,def_ver, ip.ip);
    return fw_ver;
}

int robot_ctl_scene_switch(STM_SCENE_E scene, STM_SCENE_SWITCHER action) {
    STM_LOG("stm switch %s %s\n", debug_scene_str[scene],debug_sw_str[action]);
    if(action == SCENE_SWITCH_PAUSE
       || action == SCENE_SWITCH_STOP) {
        return robot_api_control(ROBOT_CMD_PAUSE, NULL);
    }

    ROBOT_CMD_E cmd = ROBOT_CMD_NO_ACTION;
    switch(scene) {
    case SCENE_AUTO_CLEAN:
        cmd = ROBOT_CMD_AUTO_CLEAN;
        break;
    case SCENE_DOCK:
        cmd = ROBOT_CMD_DOCK;
 #if 0       
        /* 充电时，涂鸦app没有发送下来的场景切换到 charging, 导致充电时定时清扫报错误 */
        DP_ROBOT_STATE robot_state = stm_get_last_robot_sta()->robot_state;
        STM_LOG("** cmw ** dock sence state, %d\r\n", robot_state);
        if((STATE_CHARGING == robot_state)||(STATE_CHARGEDONE == robot_state))
        {
            cmd = ROBOT_CMD_AUTO_CLEAN;
            STM_LOG("** cmw ** charging state, so send auto clean\r\n");
        }
#endif
        break;
    case SCENE_SPOT_CLEAN:
        cmd = ROBOT_CMD_SPOT_CLEAN;
        break;
    case SCENE_RECT_CLEAN:
        if(action == SCENE_SWITCH_RESUME) {
            cmd = ROBOT_CMD_RECT_CLEAN_RESUME;
        }
        break;
    case SCENE_WALLFOLLOW:
        cmd = ROBOT_CMD_WALLFOLLOW;
        break;
    case SCENE_CHOICE_ROOMS_CLEAN:
        if(action == SCENE_SWITCH_RESUME){
            cmd =  ROBOT_CMD_CHOICE_ROOMS_CLEAN_RESUME;
        }
        break;
    default:
        return -1;
    }

    return robot_api_control(cmd, NULL);
}
extern int get_type_clean_zone_flag(void);
extern int set_type_clean_zone_flag(bool flag);

int robot_ctl_exec_with_param(ROBOT_CMD_E cmd, void *param) {
    switch(cmd) {
    case ROBOT_CMD_GOTO_POS: {
        STM_LOG("stm exec goto pos\n");
        memcpy(&s_goto_pos_arg.pose, param, sizeof(DATA_GOTO_POSITION));
        s_goto_pos_arg.num = 1;
        break;
    }
    case ROBOT_CMD_RECT_CLEAN_START: {
        STM_LOG("stm exec rects clean start\n");
        DATA_ZONE *zone = (DATA_ZONE *)param;
        int size = sizeof(DATA_ZONE) + zone->count * sizeof(DATA_RECT);
        memcpy(&s_rects_arg, zone, size);
		//如果是划区清扫次数策略直接返回
		if(get_type_clean_zone_flag())
		{
			set_type_clean_zone_flag(false);
			printf("\033[1m\033[40;33m type_clean_zone ===> ROBOT_CMD_RECT_CLEAN_START  \033[0m\n");
			return 0;
		}
        break;
    }
    case ROBOT_CMD_SET_VWALL: {
        //缓存虚拟墙参数
        STM_LOG("stm exec set vwall\n");
        DATA_CLASS_WALL *wall = (DATA_CLASS_WALL *)param;
        int size = sizeof(DATA_CLASS_WALL) + (wall->count * sizeof(DATA_VIRTUAL));
        memcpy(&s_vwall_arg, wall, size);
        printf("virtual add\n");
        for(int i = 0; i <s_vwall_arg.num; i++) {
            printf("virtual3 %d: %f,%f,%f,%f\r\n", i, s_vwall_arg.data[i].line.x0, s_vwall_arg.data[i].line.y0,
                s_vwall_arg.data[i].line.x1, s_vwall_arg.data[i].line.y1);
        }
        break;
    }
    case ROBOT_CMD_SET_FORBIDDEN: {
        //缓存禁区参数
        STM_LOG("stm exec set forbidden\n");
        DATA_ZONE_CLASS_EX *fzone = (DATA_ZONE_CLASS_EX*)param;
        int size = sizeof(DATA_ZONE_CLASS_EX) + (fzone->count * sizeof(DATA_FOBIDEEN));
        memcpy(&s_fb_arg, fzone, size);
        printf("stm add2 s_fb_arg.num = %d\n",s_fb_arg.num);
        printf("s_fb_arg.data[i].forbid_type = %d\n",s_fb_arg.data[0].forbid_type );
        for(int i = 0; i <s_fb_arg.num; i++) {
            printf("2 i = %d: forbid_type = %d, %f,%f,%f,%f,%f,%f,%f,%f\r\n", i,s_fb_arg.data[0].forbid_type, s_fb_arg.data[i].rect.x0, s_fb_arg.data[i].rect.y0,
                s_fb_arg.data[i].rect.x1, s_fb_arg.data[i].rect.y1, s_fb_arg.data[i].rect.x2,s_fb_arg.data[i].rect.y2,
                s_fb_arg.data[i].rect.x3,s_fb_arg.data[i].rect.y3);
        }
        break;
    }
    default:
        //the other cmds should not call this api
        return -1;
    }
    robot_api_control(cmd, param);
    return 0;
}

defined_pose_arg_t *robot_ctl_get_goto_pos_arg(void) {
    return &s_goto_pos_arg;
}

defined_rects_arg_t *robot_ctl_get_rect_clean_arg(void) {
    return &s_rects_arg;
}

virtual_wall_arg_t *robot_ctl_get_vwall_arg(void) {
    return &s_vwall_arg;
}

forbidden_arg_t *robot_ctl_get_forbid_arg(void) {
    return &s_fb_arg;
}

room_spec_arg_t *robot_ctl_get_room_spec_arg(void) {
    return &s_room_arg;
}

SelectedCleanRoom *robot_ctl_get_select_room_arg(void) {
    return &s_select_room_arg;
}

void robot_ctl_reset_select_room_arg(void) {
    s_select_room_arg.num = 0;
    printf("clear select room arg\n");
}

void robot_ctl_reset_goto_pos_arg(void) {
    s_goto_pos_arg.num = 0;
}

void robot_ctl_reset_rect_clean_arg(void) {
    s_rects_arg.num = 0;
    printf("s_rects_arg.num1 = %d\n",s_rects_arg.num );
}

void _reset_all_arg(void) {
    s_goto_pos_arg.num = 0;
    s_rects_arg.num = 0;
    s_vwall_arg.num = 0;
    s_fb_arg.num = 0;
    s_select_room_arg.num = 0;
}

void _reset_tmp_arg(STM_CLEAN_ARG arg) {
    if(arg == GOTO_POSE)
    {
        s_goto_pos_arg.num = 0;
        wifi_app_reset_navi();
    }
    else if(arg == RECT_CLEAN)
    {
        s_rects_arg.num = 0;
    }
    else if(arg == SELECT_ROOM)
    {
        s_select_room_arg.num = 0;
    }
    else
    {
        s_goto_pos_arg.num = 0;
        s_rects_arg.num = 0;
        s_select_room_arg.num = 0;
        wifi_app_reset_navi();
    }
    printf("*** 2 _reset_all_arg %d,%d,%d ***\r\n", s_goto_pos_arg.num, s_rects_arg.num,s_select_room_arg.num);
}

bool if_need_reset_tmp_arg(void){
    printf("*** 3 stm_reset_all_arg %d,%d ***\r\n", s_goto_pos_arg.num, s_rects_arg.num,s_select_room_arg.num);
        return ((s_goto_pos_arg.num != 0)||(s_rects_arg.num != 0)||(s_select_room_arg.num != 0));
}

void robot_ctl_notify_map_reset(void) {
    //机器人 状态复位
    _reset_all_arg();
    //通知云端 状态复位
    iot_robot_map_reset();
}

void robot_ctl_sync_vitual_wall(uint8_t *data) {
     if(u_log_flag)
    {
        send_log_virtual_fobidden_wall(data);
    }
    int offset;
    s_vwall_arg.num = *(int *)data;
    printf("s_vwall_arg.num = %d\n",s_vwall_arg.num);
    if(s_vwall_arg.num > MAX_VIRTUAL_WALL)
    {
        printf(" ERROR: exceed the max vitual wall\r\n");
        s_vwall_arg.num = MAX_VIRTUAL_WALL;
    }
    offset = sizeof(DATA_VIRTUAL)*s_vwall_arg.num;
    memcpy(s_vwall_arg.data, data + 4, offset);
    
    offset += 4;
    s_fb_arg.num = *(int *)(data + offset);
    printf("s_fb_arg.num = %d\r\n", s_fb_arg.num);
    if(s_fb_arg.num > MAX_FORBIDDEN)
    {
        printf(" ERROR: exceed the max forbidden\r\n");
        s_fb_arg.num = MAX_FORBIDDEN;
    }
    memcpy(s_fb_arg.data, data + offset + 4, sizeof(DATA_FOBIDEEN)*s_fb_arg.num);
    upload_virtual_wall_flag = true;
    printf("stm add virtualwall:%d forbidden:%d,flag:%d\n", s_vwall_arg.num, s_fb_arg.num, upload_virtual_wall_flag);
    for(int i = 0; i <s_fb_arg.num; i++) {
        printf("%d: %f,%f,%f,%f,%f,%f,%f,%f\r\n", i, s_fb_arg.data[i].rect.x0, s_fb_arg.data[i].rect.y0,
            s_fb_arg.data[i].rect.x1, s_fb_arg.data[i].rect.y1, s_fb_arg.data[i].rect.x2,s_fb_arg.data[i].rect.y2,
            s_fb_arg.data[i].rect.x3,s_fb_arg.data[i].rect.y3);
    }
}

int rooms_segment_process(uint8_t *data)
{
    int ret;
    ret = *(int *)(data + 4);
    
    if(0 == ret)//成功
    {
        save_rooms_flag = 1;     
        robot_api_send_msg_to_hoslam( TUYA_MSG_ROOMS_GET_EDITING_ROOMS,NULL,0);    //发送hoslam     MSG_ROOMS_GET_ROOMS
        wifi_app_msg_dp_send_rooms_msg_ret_v3(SEGMENT_SET_SUCCESS);
    }
    else if(-1 == ret)//地图不稳定
    {
        //wifi_app_msg_data_send_ret_msg(size,data,MAP_INSTABILITY);  //发送给手机端地图不稳定
       wifi_app_msg_dp_send_rooms_msg_ret(MAP_INSTABILITY);
       wifi_app_msg_dp_send_rooms_msg_ret_v3(SEGMENT_SET_FAILED);
        
    }
    else if(-2 == ret)//机器在清扫中
    {
       //wifi_app_msg_data_send_ret_msg(size,data,ROBOT_IS_CLEANING);  //发送给手机端机器在清扫中
       wifi_app_msg_dp_send_rooms_msg_ret(ROBOT_IS_CLEANING);
        wifi_app_msg_dp_send_rooms_msg_ret_v3(SEGMENT_SET_FAILED);
    }
    else
    {
       wifi_app_msg_dp_send_rooms_msg_ret_v3(SEGMENT_SET_FAILED);
    }
    printf("rooms_segment_process %d\n",ret);

    return 0;
}


int rooms_set_process(uint8_t *data)
{
    int ret;
    ret = *(int *)(data + 4);

    if(0 == ret)
    {
        robot_api_send_msg_to_hoslam( TUYA_MSG_ROOMS_GET_EDITING_ROOMS,NULL,0);     //发送hoslam     MSG_ROOMS_GET_ROOMS
    }
    else if(0 > ret)
    {
       // wifi_app_msg_data_send_ret_msg(size,data,ROOMS_SET_FAILED);   //发送给手机端 设置房间失败
       wifi_app_msg_dp_send_rooms_msg_ret(ROOMS_SET_FAILED);
    }
    
    printf("rooms_set_process:ret= %d\n",ret);
    return 0;
}



int  map_split_process(uint8_t *data)
{
    static int fail_num = 0;
    int ret;
    ret = *(int *)(data + 4);
	/*
	失败：返回-1
	成功：返回分割的房间id
	*/
    if(-1 == ret)
    {   
        fail_num += 1;
        //wifi_app_msg_data_send_ret_msg(size,data,ROOMS_SPLIT_FAILED);   //发送给手机端 分割房间失败
        if(fail_num >= 6)
        {
            fail_num = 0;
            wifi_app_msg_dp_send_rooms_msg_ret_v3(ROOMS_SPLIT_FAILED );
            printf("***###ROOMS_SPLIT_FAILED = %d\n",ret);
        }
        else
        {
            send_hoslam_split(fail_num);
        }
    }
	//成功：返回分割的房间id
    else
    {
        save_rooms_flag = 2;
        fail_num = 0;
        robot_api_send_msg_to_hoslam( TUYA_MSG_ROOMS_GET_EDITING_ROOMS,NULL,0);      //发送hoslam     MSG_ROOMS_GET_ROOMS
    }

    printf("map_split_process:ret = %d\n",ret);
    return 0;
}


int  rooms_merger_process(uint8_t *data)
{
    int ret;
    ret = *(int *)(data + 4);

    if(0 > ret)
    {
        //wifi_app_msg_data_send_ret_msg(size,data,ROOMS_MERGER_FAILED);      //发送给手机端 合并房间失败
        wifi_app_msg_dp_send_rooms_msg_ret_v3(ROOMS_MERGER_FAILED);
    }
    else
    {
        save_rooms_flag = 3;
        robot_api_send_msg_to_hoslam( TUYA_MSG_ROOMS_GET_EDITING_ROOMS,NULL,0);      //发送hoslam     MSG_ROOMS_GET_ROOMS
    }

    printf("rooms_merger_process:ret = %d\n",ret);
    return 0;
}


int  map_is_stable_process(uint8_t *data)
{
    int ret;
    ret = *(int *)(data + 4);

    if(0 == ret)
    {
        //send_msg_to_hoslam_app(MSG_ROOMS_SEGMENT,NULL,0);       //发送hoslam MSG_ROOMS_SEGMENT
    }
    else if(0 > ret)
    {
       // wifi_app_msg_data_send_ret_msg(size,data,MAP_INSTABILITY);    //发送给手机端 地图不稳定
       wifi_app_msg_dp_send_rooms_msg_ret(MAP_INSTABILITY);
    }
    printf("map_is_stable_process:ret = %d\n",ret);
    return 0;
}

int  rooms_is_manual_process(uint8_t *data)
{
    int ret;
    ret = *(int *)(data + 4);

    if(0 == ret)
    {
       // send_msg_to_hoslam_app(MSG_ROOMS_CLEAR_ROOMS,NULL,0);           //发送hoslam   MSG_ROOMS_CLEAR_ROOMS
    }
    else if(0 > ret)
    {
       // wifi_app_msg_data_send_ret_msg(size,data,ROOMS_NOT_SET_BY_HUMAN);     //发送给手机端 当前不是人为设置的房间信息
       wifi_app_msg_dp_send_rooms_msg_ret(ROOMS_NOT_SET_BY_HUMAN);
    }
    printf("rooms_is_manual_process:ret = %d\n",ret);
    return 0;
}


int rooms_clean_rooms_process(uint8_t *data)
{
    int ret;
    ret = *(int *)(data + 4);

    if(0 == ret)
    {
        robot_api_send_msg_to_hoslam( TUYA_MSG_ROOMS_GET_EDITING_ROOMS,NULL,0);         //发送hoslam     MSG_ROOMS_GET_ROOMS
    }
    else if(0 > ret)
    {
       // wifi_app_msg_data_send_ret_msg(size,data,ROOMS_NOT_SET_BY_HUMAN);       //发送给手机端 当前不是人为设置的房间信息
       wifi_app_msg_dp_send_rooms_msg_ret(ROOMS_NOT_SET_BY_HUMAN);
    }
    printf("rooms_clean_rooms_process:ret = %d\n",ret);
    return 0;
}

#if 0
void send_hoslam_set_constituency_cleaning(int id,int count,int order,int forbidden)
{
    int i = 0;
	pthread_mutex_lock(&rooms_lock);
	s_room_arg.room_spec[id].count = count;
	s_room_arg.room_spec[id].order = order;
	s_room_arg.room_spec[id].forbidden = forbidden;
    for(i = 0;i < s_room_arg.num;i++)
    {
        if(i != id)
        {
            s_room_arg.room_spec[i].forbidden = 1;
        }
    }
	pthread_mutex_unlock(&rooms_lock);
	send_hoslam_save_room();
}
#endif

int constituency_cleaning_flag;
CLEAN_ROOMS_ID  clean_rooms;
int increase_new_room_flag;
static int first_recv_flag;


void clean_up_save_forbidden()
{
    int i;
    if(constituency_cleaning_flag == 1)
    {
        constituency_cleaning_flag = 0;
        pthread_mutex_lock(&rooms_lock);
        for(i = 0;i < s_room_arg.num;i++)
        {
         s_room_arg.room_spec[i].forbidden = 0;
        }
        pthread_mutex_unlock(&rooms_lock);
        printf("clean_up_save_forbidden\n");
    }
    send_hoslam_save_room();
}



void send_hoslam_flash_constituency_cleaning(int len,int *id_data)
{
    int i , j;
    int exist_flag;
    printf("send_hoslam_flash_constituency_cleaning\n");
	pthread_mutex_lock(&rooms_lock);
    for(i = 0;i < s_room_arg.num;i++)
    {
        exist_flag = 0;
        for(j = 0;j < len;j++)
        {
            if(i == id_data[j])
            {
                exist_flag = 1;
                break;
            }
        }
        if(exist_flag == 0)
        {
            s_room_arg.room_spec[i].forbidden = 1; 
        }
    }
	pthread_mutex_unlock(&rooms_lock);
	send_hoslam_save_room();
}



void set_new_room_fobidden(int len,int *id_data)
{
    int i = 0;
    clean_rooms.len = len;
    constituency_cleaning_flag = 1;
    for(i = 0;i < len;i++)
    {
        clean_rooms.id_data[i] = id_data[i];
    }
    send_hoslam_flash_constituency_cleaning(clean_rooms.len,clean_rooms.id_data);
}

void set_room_msg(int len,int *id_data)
{
    int i = 0;
    int clean_rooms_count = 0;
    int rooms_id;
    //clean_rooms.len = len;
    constituency_cleaning_flag = 0;
    for(i = 0;i < len;i++)
    {
        rooms_id = id_data[i*4];
        pthread_mutex_lock(&rooms_lock);
        s_room_arg.room_spec[rooms_id].order = id_data[i*4 + 1];
        s_room_arg.room_spec[rooms_id].order = id_data[i*4 + 2];
        pthread_mutex_unlock(&rooms_lock);
        if(1 == id_data[i *4 + 3])
        {
            constituency_cleaning_flag = 1;
        }
        else
        {
            clean_rooms.id_data[clean_rooms_count] = rooms_id;
            ++clean_rooms_count;
        }
    }
    clean_rooms.len = clean_rooms_count;
    if(constituency_cleaning_flag)
    {
        send_hoslam_flash_constituency_cleaning(clean_rooms.len,clean_rooms.id_data);
    }
}

void set_choice_rooms_clean(uint8_t clean_count,uint8_t len,uint8_t *rooms_id)
{
    int i,j;
    int datalen;
    datalen =  sizeof(uint32_t)+ len * sizeof(SelectedCleanRoomInfo); 
    int clean_rooms_len = sizeof(datalen)+ datalen;
    
    char *crl = malloc(clean_rooms_len);
    if(NULL == crl)
    {
        printf("SelectedCleanRoom malloc error\n");
    }
     *(int *)crl = datalen;

    SelectedCleanRoom *scrp =(SelectedCleanRoom *) (crl + sizeof(datalen));
    scrp->num = len;
    SelectedCleanRoomInfo *crp = scrp->rooms;

    for(i = 0; i < len;i++)
    {
      crp->room_id = *rooms_id;
      crp->order = -1;
      crp->clean_count = clean_count;
      printf("id = %d",*rooms_id);
      printf("room_id = %d,clean_count =%d\n",crp->room_id,crp->clean_count);
      crp += 1;
      rooms_id += 1;
    }
    robot_api_send_msg_to_hoslam(TUYA_MSG_ROOMS_SET_CLEAN_ROOMS,(char *)crl,clean_rooms_len);

    free(crl);
    return ;
}



void robot_ctl_sync_room_spec(uint8_t *data)
{ 
    int i;
	int len,rooms_count,rooms_size;
    memcpy(&len, data, sizeof(len));
    memcpy(&rooms_count, (data + 4), sizeof(len));
    if(-1 == rooms_count)
    {
        return;
    }
    pthread_mutex_lock(&rooms_lock);
    s_room_arg.num = rooms_count;
    
    rooms_size = rooms_count * sizeof(ROOM_SPEC_T);
    //s_room_arg.num = len / sizeof(ROOM_SPEC_T);
    if(s_room_arg.num > MAX_ROOM_SPEC)
    {
        printf(" ERROR: exceed the max room spec\r\n");
        s_room_arg.num = MAX_ROOM_SPEC;
        len = sizeof(ROOM_SPEC_T)*s_room_arg.num;
    }
    sys_sem_wait(rooms_from_hoslam_semid, 1);
    memcpy(s_room_arg.room_spec, rooms_from_hoslam_shmadd, rooms_size);
    sys_sem_signal(rooms_from_hoslam_semid, 0);
    pthread_mutex_unlock(&rooms_lock);
    if(u_log_flag)
    {
        send_log_room(s_room_arg.num,(char *)s_room_arg.room_spec);
    }
	printf("room_count:%d\n",s_room_arg.num);
}

void robot_ctl_sync_edit_room_spec(uint8_t *data)
{
    static int call_count;
	int len,rooms_count,rooms_size;
    memcpy(&len, data, sizeof(len));
    memcpy(&rooms_count, (data + 4), sizeof(len));
    if(-1 == rooms_count)
    {
        printf("rooms_count no charge\n");
        return;
    }
    pthread_mutex_lock(&rooms_lock);
    s_room_arg.num = rooms_count;
    rooms_size = rooms_count * sizeof(ROOM_SPEC_T);
    //s_room_arg.num = len / sizeof(ROOM_SPEC_T);
    if(s_room_arg.num > MAX_ROOM_SPEC)
    {
        printf(" ERROR: exceed the max room spec\r\n");
        s_room_arg.num = MAX_ROOM_SPEC;
        len = sizeof(ROOM_SPEC_T)*s_room_arg.num;
    }
    printf("App server Semapore wait\n");
    sys_sem_wait(rooms_from_hoslam_semid, 1);
    memcpy(s_room_arg.room_spec, rooms_from_hoslam_shmadd, rooms_size);
    sys_sem_signal(rooms_from_hoslam_semid, 0);
    printf("App server Semapore signal\n");
    pthread_mutex_unlock(&rooms_lock);
    if(u_log_flag)
    {
        send_log_room(s_room_arg.num,(char *)s_room_arg.room_spec);
    }
    //send_map_rooms_data();
    robot_api_send_msg_to_hoslam(TUYA_MSG_TYPE_GET_VIRTUAL_WALL,NULL,0);
   
	printf("room_count:%d, save_rooms_flag:%d\n",s_room_arg.num,save_rooms_flag);
    if(save_rooms_flag > 0)
    {
        printf("send_hoslam_save_room\n");
        send_hoslam_save_room();
       if(1 == save_rooms_flag)
        {
            wifi_app_msg_dp_send_rooms_msg_ret_v3(SEGMENT_SET_SUCCESS);
        }        
        else if(2 == save_rooms_flag)
        {
            wifi_app_msg_dp_send_rooms_msg_ret_v3(ROOMS_SPLIT_SUCESS);
        }
        else if(3 == save_rooms_flag)
        {
            wifi_app_msg_dp_send_rooms_msg_ret_v3(ROOMS_MERGER_SUCESS);
        }
        save_rooms_flag = 0;
    }
}


void send_hoslam_save_room(void)
{
    int packet[8];
    int size,room_size;

    room_size = s_room_arg.num * sizeof(ROOM_SPEC_T);
    sys_sem_wait(rooms_to_hoslam_semid, 0);
    memcpy(rooms_to_hoslam_shmadd, s_room_arg.room_spec, room_size);
    sys_sem_signal(rooms_to_hoslam_semid, 1);
    
    size = sizeof(int) * 2;
    memset(packet, 0, sizeof(packet));
    packet[0] =size; //datalen+data
    packet[1] = s_room_arg.num;
   
    robot_api_send_msg_to_hoslam(TUYA_MSG_ROOMS_SET_ROOMS, (const char*)packet, size);
   
}


void upload_first_wirtual_wall_info(void)
{
    if(upload_virtual_wall_flag)
    {
        wifi_app_msg_dp_parser_get_all_config(NULL);
        upload_virtual_wall_flag = false;
        printf("*** upload_first_wirtual_wall_info to app ***\r\n");
    }
}
