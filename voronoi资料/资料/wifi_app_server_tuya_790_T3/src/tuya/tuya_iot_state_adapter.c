#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <signal.h>


#include "tuya_iot_com_api.h"
#include "tuya_cloud_types.h"
#include "tuya_iot_wifi_api.h"
#include "uni_log.h"
#include "uni_time.h"
#include "tuya_hal_wifi.h"
#include "sys_timer.h"
#include "tuya_iot_state_adapter.h"
#include "robot_control_adapter.h"
#include "robot_api.h"
#include "tuya_iot_msg_parse.h"
#include "../include/sys_api.h"
#include "robot_sys_msg.h"
#include "../debug/debug_tool.h"

static char *cleanmodeStr[] = {
    "WORK_MODE_SMART",    // 自动清扫
    "WORK_MODE_ZONE",         // 区域清扫
    "WORK_MODE_POSE",         // 指哪扫哪
    "WORK_MODE_PART",         // 局部清扫
    "WORK_MODE_CHARGEGO",     // 自动充电
    "WORK_MODE_WALLFOLLOW"   //沿边清扫
    "WORK_CHOICE_CLEAN",      //选区清扫
    
};

static char *swStateStr[] = {
    "standby",
    "autoCleaning",
    "rectCleanning",
    "spotCleanning",
    "cleaning",
    "robotPaused",
    "gotoPos",
    "gotoArrived",
    "gotoFailed",
    "docking",
    "charging",
    "chargeFull",
    "sleep",
    "error",
    "wallfollow",
   "remoteControl",
   "choiceclean",
   "repositioning"
};

typedef enum
{
    STATE_RECORD_END = 0,                // 终止清扫记录的模式
    STATE_RECORD_PAUSE,                   // 暂停记录清扫记录的模式
    STATE_RECORD_RUN,                     // 开始记录清扫记录的模式
} STATE_RECORD;

// 清扫任务开关 本地缓存
extern bool s_task_sw;
//是否连接上云端
static bool is_cloud_ready = false;
sys_timer_t *timer = NULL;
//=====================================================================
static DATA_ROBOT_STATE last_data_robot_sta;
static DATA_CLEAN_STATIS last_data_clean_sta;
static DATA_CONTROL_VALVE_STATE last_data_valve_sta;//缓存风机和水箱状态
static DATA_CLEAN_STATIS last_data_clean_sta_remember;
static DATA_CONSUMABLE_STATIS last_data_consumable_sta;
bool s_send_his = false;            //是否上传清扫记录
pthread_mutex_t mtx_map_record;

/* 清空上一次的清扫状态 */
void reset_last_data_robot_sta(void) 
{
    set_cur_scene(SCENE_AUTO_CLEAN);
    last_data_robot_sta.error_no = 0;
    last_data_robot_sta.work_mode = WORK_MODE_SMART;
    last_data_robot_sta.robot_state = STATE_STANDBY;
}

/*获取缓存的风机和水箱状态*/
int get_fan_mode_and_water_level(int type)
{
	int ret;
	if(type == 1)
	{	 
		
		ret = last_data_valve_sta.water_level;
	}
	else
	{	
		ret = last_data_valve_sta.fan_mode;
	}
	return ret;
}

static DP_WORK_MODE curScene2WorkMode(void) {
    DP_WORK_MODE work_mode;

    switch(get_cur_scene()) {
    case SCENE_AUTO_CLEAN:
        work_mode = WORK_MODE_SMART;
        break;
    case SCENE_DOCK:
        work_mode = WORK_MODE_CHARGEGO;
        break;
    case SCENE_GOTO_POS:
    case SCENE_SPOT_CLEAN:
        work_mode = WORK_MODE_POSE;
        break;
    case SCENE_RECT_CLEAN:
        work_mode = WORK_MODE_ZONE;
        break;
    default:
        work_mode = WORK_MODE_SMART;
    }
    return work_mode;
}

// 将状态分为运动状态, 中间状态, 和终止状态; 分别返回2,1,0
static STATE_RECORD robot_divide_state(DP_ROBOT_STATE state)
{
    STATE_RECORD ret = STATE_RECORD_END;
    
    if ((state == STATE_SMARTCLEAN)||(state == STATE_ZONECLEAN)||
        (state == STATE_PARTCLEAN)||(state == STATE_WALLFOLLOW)||(state == STATE_CHOICE_CLEAN))
    {
        ret = STATE_RECORD_RUN;
    }
    else if((state == STATE_PAUSED)||(state == STATE_SLEEP)||(state == STATE_REPOSITING))
    {
        ret = STATE_RECORD_PAUSE;
    }
    else
    {
        ret = STATE_RECORD_END;
    }
    
    return ret;
}

extern ROBOT_RETCODE_E robot_api_send_msg_to_hoslam(int type, const char *data, const int len);

static bool robot_record_flag_set(ROBOT_STATE_INFO *pInfo, DP_ROBOT_STATE robot_sta){
    static DP_ROBOT_STATE last_robot_state = STATE_STANDBY;         //用于记录上一次的机器人状态
    
    // 在 pause 时用于暂存运动时的状态, pause 后用来比较
    static DP_ROBOT_STATE pause_remember_state = STATE_STANDBY;

    STATE_RECORD last_state_ret = robot_divide_state(last_robot_state);
    STATE_RECORD this_state_ret = robot_divide_state(robot_sta);
#if 0
    if((STATE_RECORD_RUN==this_state_ret) && (ROBOT_STATE_IDLE==pInfo->robot_state))
    {
        /* 排除 ROBOT_STATE_STANDBY 产生的运动模式 */
        printf("no record run mode from standby\r\n");
        return false;
    }
#endif
    /* 用上一次上传的清扫时间来做记录文件的判断 */
    DATA_CLEAN_STATIS *last_clean_statis = iot_get_last_clean_statis();
    
//    printf("---record robot_sta last=%d,now=%d\r\n", last_robot_state, robot_sta);
    printf("---record robot_sta string remember=%s,last=%s,now=%s\r\n",
        swStateStr[pause_remember_state], swStateStr[last_robot_state], swStateStr[robot_sta]);

    /* 如果上一次是运动状态 */
    if((STATE_RECORD_RUN==last_state_ret))
    {
        /* 现在是中间状态(pause sleep) */
        if(STATE_RECORD_PAUSE == this_state_ret)
        {
            /* 如果遇到中间状态先将之前的保留下来 */
            pause_remember_state = last_robot_state;
            send_history_record_mark();
        }
        else if(STATE_RECORD_END == this_state_ret)
        {
            /* 结束一次运动，记录数据 */
            if(last_clean_statis->clean_time > 0 && last_clean_statis->clean_area > 0)
            {
            	//更新动态分区房间信息
    			send_msg_to_hoslam_app(TUYA_MSG_ROOMS_GET_WORKING_ROOMS,NULL,0);
                printf("---record set true 1\r\n");
                s_send_his = true;
            }
            pause_remember_state = STATE_STANDBY;
        }
        else
        {
            pause_remember_state = STATE_STANDBY;
            /* 同样是运动状态不处理 */
        }
    }
    else if ((STATE_RECORD_PAUSE==last_state_ret))  /* 如果上一次是暂停状态 */
    {
        if(STATE_RECORD_RUN == this_state_ret)
        {
            /* 如果有这次的run状态和上一次不一样，就是切换了状态, 要记录 */
            if(pause_remember_state != robot_sta && pause_remember_state != STATE_STANDBY)
            {
                if(iot_get_clean_remember_statis()->clean_time > 0 && iot_get_clean_remember_statis()->clean_area > 0)
                {
					//更新动态分区房间信息
					send_msg_to_hoslam_app(TUYA_MSG_ROOMS_GET_WORKING_ROOMS,NULL,0);
					printf("---record set true 2\r\n");
                    s_send_his = true;
                }
            }
            else    /* 运动状态不变 或者是 之前为 standby 的都不记录 */
            {
                /* 休眠后如果run状态一样, 也记录上一次的文件 */
                if(pause_remember_state == robot_sta && pause_remember_state != STATE_STANDBY
                    && (last_robot_state == STATE_SLEEP))
                {
                    if(iot_get_clean_remember_statis()->clean_time > 0 && iot_get_clean_remember_statis()->clean_area > 0)
                    {
                    	//更新动态分区房间信息
						send_msg_to_hoslam_app(TUYA_MSG_ROOMS_GET_WORKING_ROOMS,NULL,0);
                        printf("---record set true 4\r\n");
                        s_send_his = true;
                    }
                }
            }
            pause_remember_state = STATE_STANDBY;
        }
        else if(STATE_RECORD_PAUSE == this_state_ret)
        {
            /* 两次都是pause 什么都不做 */
        }
        else    /* STATE_RECORD_END, 只要在 pause 之前是运行的状态都记录下来 */
        {
            if(pause_remember_state != STATE_STANDBY)
            {
                if(last_clean_statis->clean_time > 0 && last_clean_statis->clean_area > 0)
                {
                	//更新动态分区房间信息
					send_msg_to_hoslam_app(TUYA_MSG_ROOMS_GET_WORKING_ROOMS,NULL,0);
                    printf("---record set true 3\r\n");
                    s_send_his = true;
                }
            }
            else    /* 运动状态不变 或者是 之前为 standby 的都不记录 */
            {
                
            }
            pause_remember_state = STATE_STANDBY;
        }      
    }
    else    //  STATE_RECORD_END==last_state_ret
    {
        /* 上一次如果是终止状态, 这次暂时不用记录 */
        pause_remember_state = STATE_STANDBY;
    }

    /* 记录上一次状态 */
    last_robot_state = robot_sta;
   
    return true;
}

static bool robot_clean_robot_data(ROBOT_STATE_INFO *pInfo, DP_ROBOT_STATE robot_sta){
    static DP_ROBOT_STATE last_robot_state = STATE_STANDBY;         //用于记录上一次的机器人状态

    if(robot_sta != STATE_REPOSITING  && robot_sta != STATE_PAUSED && robot_sta != STATE_SLEEP)
    {
        if(robot_sta != STATE_GOTOPOS && last_robot_state == STATE_GOTOPOS)
        {
            wifi_app_reset_clean_data(GOTO_POSE);
        }
        else if(robot_sta != STATE_ZONECLEAN && last_robot_state == STATE_ZONECLEAN)
        {
            wifi_app_reset_clean_data(RECT_CLEAN);
        }
        else if(robot_sta != STATE_CHOICE_CLEAN && last_robot_state == STATE_CHOICE_CLEAN)
        {
            wifi_app_reset_clean_data(SELECT_ROOM);
        }
        last_robot_state = robot_sta;
    }
    return true;
}

static DP_ROBOT_STATE idle_to_tuya_robot_state(ROBOT_STATE_INFO *pInfo) {
    DP_ROBOT_STATE tuya_robot_state = STATE_STANDBY;
    static DP_ROBOT_STATE last_tuya_robot_state = 0xff;
    DP_WORK_MODE work_mode = last_data_robot_sta.work_mode;
	printf("\n last_data_robot_sta work_mode:%d \n",work_mode);
    switch(work_mode) {
    case WORK_MODE_POSE:
        if(pInfo->error_info == ERROR_INFO_NO_ERROR) 
        {          
            if(last_tuya_robot_state != STATE_UNARRIVABLE) 
            {
                tuya_robot_state = STATE_ARRIVED;//到达目标点
            }

            if(u_log_flag)
            {
               sys_clear_navigation_recieved();
            }        
        } 
        else
         { //STATUS_DONE
            tuya_robot_state = STATE_UNARRIVABLE;//目标点不可达
            if(u_log_flag)
            {
                sys_clear_navigation_recieved();
            }
        }
        last_tuya_robot_state = tuya_robot_state;  /* 记录上一次指哪扫哪的状态 */
        break;
    case WORK_MODE_ZONE:
        //if(pInfo->error_info != ERROR_INFO_NO_ERROR) 
           tuya_robot_state = STATE_UNARRIVABLE;//目标点不可达
        	break;
    case WORK_MODE_CHOICE_CLEAN:
        if(pInfo->error_info != ERROR_INFO_NO_ERROR) {
            tuya_robot_state = STATE_UNARRIVABLE;//目标点不可达
        }
        break;
        
    }
    return tuya_robot_state;
}

static bool robot_state_mapping(ROBOT_STATE_INFO *pInfo, DATA_ROBOT_STATE *robot_sta) {

    switch(pInfo->robot_state) {
    case ROBOT_STATE_IDLE:
        robot_sta->work_mode = last_data_robot_sta.work_mode;
        robot_sta->robot_state = idle_to_tuya_robot_state(pInfo);
        break;
    case ROBOT_STATE_PAUSED:
        robot_sta->work_mode = last_data_robot_sta.work_mode;
        robot_sta->robot_state = STATE_PAUSED;
        break;
    case ROBOT_STATE_SLEEP:
        robot_sta->work_mode = curScene2WorkMode();
        robot_sta->robot_state = STATE_SLEEP;
        break;
    case ROBOT_STATE_CLEAN_ROOM:
    case ROBOT_STATE_CLEAN_AUTO:
        robot_sta->work_mode = WORK_MODE_SMART;
        robot_sta->robot_state = STATE_SMARTCLEAN;
        break;
    case ROBOT_STATE_GOTO_POS:
        robot_sta->work_mode = WORK_MODE_POSE;
        robot_sta->robot_state = STATE_GOTOPOS;
        break;
    case ROBOT_STATE_CLEAN_SPOT:
        robot_sta->work_mode = WORK_MODE_PART;
        robot_sta->robot_state = STATE_PARTCLEAN;
        break;
    case ROBOT_STATE_CLEAN_RECTS:
        robot_sta->work_mode = WORK_MODE_ZONE;
        robot_sta->robot_state = STATE_ZONECLEAN;
        break;
    case ROBOT_STATE_DOCK:
        robot_sta->work_mode = WORK_MODE_CHARGEGO;
        robot_sta->robot_state = STATE_GOCHARGE;
        break;
    case ROBOT_STATE_SEAT_CHARGING:
    case ROBOT_STATE_DIRECT_CHARGING:
        robot_sta->work_mode = WORK_MODE_CHARGEGO;
        if(last_data_clean_sta.battery_level >= 100) {
            robot_sta->robot_state = STATE_CHARGEDONE;
        } else {
            robot_sta->robot_state = STATE_CHARGING;
        }
        break;
    case ROBOT_STATE_ERROR:
        robot_sta->work_mode = curScene2WorkMode();
        robot_sta->robot_state = STATE_ERROR;
		/*一个bit对应一个错误码*/
		robot_sta->error_no = pInfo->error_info ? 0x00000001 << pInfo->error_info : ERROR_INFO_NO_ERROR;
		printf("robot_sta->error_no = 0x%x\n", robot_sta->error_no);
        break;
    case ROBOT_STATE_WALLFOLLOW:
        robot_sta->work_mode = WORK_MODE_WALLFOLLOW;
        robot_sta->robot_state = STATE_WALLFOLLOW;
        break;
    case ROBOT_STATE_REMOTE_DRIVE:
        robot_sta->work_mode = curScene2WorkMode();
        robot_sta->robot_state = STATE_REMOTE_CONTROL;
        break;

    case ROBOT_STATE_CHOICE_CLEAN:
        robot_sta->work_mode = WORK_MODE_CHOICE_CLEAN;
        robot_sta->robot_state = STATE_CHOICE_CLEAN;
        break;
    default:
        robot_sta->work_mode = last_data_robot_sta.work_mode;
        robot_sta->robot_state = last_data_robot_sta.robot_state;
        break;
    }
		//故障优先显示，重定位中 状态次之
		if (pInfo->robot_state !=  ROBOT_STATE_ERROR && pInfo->robot_reposition_state == ROBOT_REPOSITING)
		{
			robot_sta->robot_state = STATE_REPOSITING;
		}
        { 
            static int32_t last_robot_reposition_state;  // 若重定位失败，则清除地图 
            printf("pInfo->robot_reposition_state = %d,last_robot_reposition_state = %d\n", pInfo->robot_reposition_state,last_robot_reposition_state);
            if( pInfo->robot_reposition_state == ROBOT_REPOSITION_FAILED && last_robot_reposition_state != ROBOT_REPOSITION_FAILED)
            {
                 wifi_app_reset_map();
            }
            last_robot_reposition_state = pInfo->robot_reposition_state;

        }

		printf("********robot_sta->robot_state:%d \n",robot_sta->robot_state);
//    robot_sta->fan_mode = pInfo->fan_mode;
    if(memcmp(&last_data_robot_sta, robot_sta, sizeof(DATA_ROBOT_STATE)) == 0) {
        if(!get_force_update_state())
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    memcpy(&last_data_robot_sta, robot_sta, sizeof(DATA_ROBOT_STATE));

    return true;
}

static bool clean_state_mapping(ROBOT_STATE_INFO *pInfo, DATA_CLEAN_STATIS *clean_sta) {

    clean_sta->battery_level = pInfo->battery_level;
    clean_sta->clean_area    = pInfo->clean_area;
    clean_sta->clean_time    = pInfo->clean_time;
    clean_sta->y_mod         = pInfo->y_type_mop;
    clean_sta->dnd_mode      = pInfo->dnd_mode;
    clean_sta->continue_clean = pInfo->continue_clean;
    if(memcmp(&last_data_clean_sta, clean_sta, sizeof(DATA_CLEAN_STATIS)) == 0) {
        if(!get_force_update_state())
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    memcpy(&last_data_clean_sta, clean_sta, sizeof(DATA_CLEAN_STATIS));
    return true;
}

static bool consumables_state_mapping(ROBOT_STATE_INFO *pInfo, DATA_CONSUMABLE_STATIS *consumable_sta) {

    consumable_sta->edge_brush_life 	= pInfo->edge_brush_life;
    consumable_sta->roll_brush_life   	= pInfo->roll_brush_life;
    consumable_sta->filter_life    		= pInfo->filter_life;
    consumable_sta->mop_life         	= pInfo->mop_life;
	
    if(memcmp(&last_data_consumable_sta, consumable_sta, sizeof(DATA_CONSUMABLE_STATIS)) == 0) 
	{
        if(!get_force_update_state())
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    memcpy(&last_data_consumable_sta, consumable_sta, sizeof(DATA_CONSUMABLE_STATIS));
    return true;
}


static bool robot_valve_mapping(ROBOT_STATE_INFO *pInfo, DATA_CONTROL_VALVE_STATE *valve_sta) {

    static uint8_t valve_config_fisrt = 0;  //记录第一次上报数据
    switch(pInfo->fan_mode)
    {
	    case STATE_FAN_NORMAL:
	        valve_sta->fan_mode = FAN_NORMAL;
	        break;
	    case STATE_FAN_STRONG:
	        valve_sta->fan_mode = FAN_STRONG;
	        break;
	    case STATE_FAN_SILENT:
	    case STATE_FAN_OFF:
	    case STATE_FAN_DISABLE:
	        valve_sta->fan_mode = FAN_SILENT;
	        break;
	    default:
	        valve_sta->fan_mode = FAN_SILENT;
	        break;
		
    }
    
    switch(pInfo->water_level)
    {
    case STATE_WATER_LEVEL_DEFAULT:
        valve_sta->water_level = WATER_LEVEL_2;
        break;
    case STATE_WATER_LEVEL_HIGH:
        valve_sta->water_level = WATER_LEVEL_3;
        break;
    case STATE_WATER_LEVEL_LOW:
        valve_sta->water_level = WATER_LEVEL_1;
        break;
    case STATE_WATER_LEVEL_OFF:
    case STATE_WATER_LEVEL_DISABLE:
        valve_sta->water_level = WATER_LEVEL_OFF;
        break;
    default:
        valve_sta->water_level = WATER_LEVEL_OFF;
        break;
    }

    if(0 == valve_config_fisrt)
    {
        memcpy(&last_data_valve_sta, valve_sta, sizeof(DATA_CONTROL_VALVE_STATE));
        valve_config_fisrt = 1;
        return true;
    }
    
    if(memcmp(&last_data_valve_sta, valve_sta, sizeof(DATA_CONTROL_VALVE_STATE)) == 0) {
        if(!get_force_update_state())
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    memcpy(&last_data_valve_sta, valve_sta, sizeof(DATA_CONTROL_VALVE_STATE));
    return true;
}

void iot_robot_state_received(ROBOT_STATE_INFO *pInfo) {

    DATA_ROBOT_STATE robot_sta;
    DATA_CLEAN_STATIS clean_sta;
    DATA_CONTROL_VALVE_STATE valve_sta;
	DATA_CONSUMABLE_STATIS consumable_state;
    memset(&robot_sta, 0, sizeof(DATA_ROBOT_STATE));
    memset(&clean_sta, 0, sizeof(DATA_CLEAN_STATIS));
    memset(&valve_sta, 0, sizeof(DATA_CONTROL_VALVE_STATE));
	memset(&consumable_state, 0, sizeof(DATA_CONSUMABLE_STATIS));
    if(robot_state_mapping(pInfo, &robot_sta)) {
        //上报的dp状态，用来修正一些场景切换问题
        tuya_iot_robot_state_feedback(robot_sta.robot_state);
        //变化时上报机器状态
        if(iot_state_cloud_ready()) {
            iot_report_robot_state(&robot_sta);
        }
    }
    
    /* 用于根据各个状态判断是否具备记录清扫记录,并上传, 放在这一次读取清扫时间之前 */
    robot_record_flag_set(pInfo,robot_sta.robot_state);
    if(iot_state_cloud_ready())
    {
        robot_clean_robot_data(pInfo,robot_sta.robot_state);
    }
    
    if(clean_state_mapping(pInfo, &clean_sta) && iot_state_cloud_ready()) {
        //变化时上报统计数据
        iot_report_clean_statis(&clean_sta);
    }
    if(robot_valve_mapping(pInfo, &valve_sta) && iot_state_cloud_ready()) {
        //变化时上报风机水箱数据
        iot_report_valve_statis(&valve_sta);
    }
	if(consumables_state_mapping(pInfo, &consumable_state) && iot_state_cloud_ready())
	{
        //变化时上报耗材使用数据
        iot_report_consumable_statis(&consumable_state);
	}
    set_force_update_state(0);
}


int init_map_record_lock(void)
{
    // 初始化互斥量
    int ret;
    ret = pthread_mutex_init(&mtx_map_record, NULL);
    if (ret)
    {
        printf("init init_map_record_lock error\n");
        return -1;
    }
}

void send_history_record_mark(void)
{
    DATA_CLEAN_STATIS *last_clean_statis = iot_get_last_clean_statis();
    int ret;
    ret = pthread_mutex_lock(&mtx_map_record);
    if(0 != ret)
    {
        printf("ERROR: record pthread_mutex_lock error:%d\n",ret);
    }
    memcpy(&last_data_clean_sta_remember, last_clean_statis, sizeof(DATA_CLEAN_STATIS));
    ret = pthread_mutex_unlock(&mtx_map_record);
    
    if(0 != ret)
    {
        printf("ERROR: record pthread_mutex_unlock error:%d\n",ret);
    }
    
    printf("--- turn new mode, use remember clean time 0=%d\r\n", last_data_clean_sta_remember.clean_time);
//    send_history_record_if_need();
}
void send_history_record_unmark(void)
{
    int ret;
    ret = pthread_mutex_lock(&mtx_map_record);
    if(0 != ret)
    {
        printf("ERROR: unmark pthread_mutex_lock error:%d\n",ret);
    }
    memset(&last_data_clean_sta_remember, 0, sizeof(DATA_CLEAN_STATIS));
    ret = pthread_mutex_unlock(&mtx_map_record);
    if(0 != ret)
    {
        printf("ERROR: unmark pthread_mutex_lock error:%d\n", ret);
    }
}

DATA_CLEAN_STATIS *iot_get_clean_remember_statis(void) {
    return &last_data_clean_sta_remember;
}

DATA_CLEAN_STATIS *iot_get_last_clean_statis(void) {
    return &last_data_clean_sta;
}

//设置配网模式后，SOC设备进程重启进入配网
void iot_start_config_network(void) {

    is_cloud_ready = false;
#ifndef WIFI_SMART_MODE
    WF_WK_MD_E wifi_mode;
    is_cloud_ready = false;
    hwl_wf_wk_mode_get(&wifi_mode);
    if(wifi_mode == WWM_SOFTAP) {
        PR_DEBUG("wifi already in ap mode\n");
        return;
    }
#endif

	tuya_hal_wifi_sniffer_stop();

    /* 保证在重启后配上一个默认的 ip 而不会因为没有ip导致卡死在ready中 */
    tuya_hal_wifi_station_disconnect();

    sleep(3);//延时，等待语音播放完成
    //进程重启
    pid_t pid = fork();
    if (pid == -1) {
        fprintf(stderr, "fork() error.errno:%d error:%s\n", errno, strerror(errno));
        return;
    }
    if (pid == 0) {
        char *argv[] = {"wifi_app_server", NULL};
        sleep(2);
        int ret = execv("wifi_app_server", argv);
        if (ret < 0) {
            fprintf(stderr, "execv ret:%d errno:%d error:%s\n", ret, errno, strerror(errno));
            return;
        }
        exit(0);
    }
    sys_shell("kill %d",getpid());
}

static void NetWork_Monitor_timer(void)
{
    //printf(" **** ping baidu and route ****\n");
    printf(" **** ping  route ****\n");
    sys_shell("killall -9 ping");
    sys_shell("killall -9 wifi_scan");
    //sys_shell("ping www.baidu.com -c 4 -w 4 &");
    sys_shell("ping `route -n | grep 'UG' | awk '{print $2}'` -c 4 -w 4 &");
    sys_shell("/root/bin/scanap");
    return;
}
int init_soft_timer(void)
{
    
    timer = sys_timer_create(5000, NetWork_Monitor_timer);
    if (!timer)
    {
        printf("1 sys_timer_create ERROR\n");
        return -1;
    }
    
    if (sys_timer_start_with_period(timer, 100, 60*1000) == -1)
    {
        sys_timer_delete(timer);
        timer = NULL;
        printf("1 sys_timer_start ERROR\n");
        return -1;
    }
}

void delete_timer(void)
{
    sys_timer_delete(timer);
    timer = NULL;
    printf("2 delete_timer\n");
}


/* 涂鸦的系统时间在建立成功后要过一段时间才生效，所以加入定时器来判断 */
void SigAlarm_Handler(int sig)
{
    static char count = 0;
    POSIX_TM_S tm;
    uni_local_time_get(&tm);

    if(tm.tm_year < 119)
    {
        if(count < 10)
        {
            alarm(5);
        }
        else
        {
            printf("alarm set time fail! %d\n", count);
            count = 0;
        }
    }
    else
    {
        char cmd[32];//同步涂鸦SDK时间
        sprintf(cmd, "date %02d%02d%02d%02d%02d.%02d", tm.tm_mon +1, tm.tm_mday,
                tm.tm_hour, tm.tm_min, (tm.tm_year +1900) -2000, tm.tm_sec);
        sys_shell(cmd);
        sync_time_2_380(&tm); /* 将时间同步给380 */
        printf("alarm set time success! %s\n", cmd);
    }
    
    count++;
    return;
}

static void sync_cloud_time(void) {
    POSIX_TM_S tm;
    uni_local_time_get(&tm);
    if(tm.tm_year < 119) {
        //同步服务器时间
        sys_shell("ntpdate time.nuri.net");
        signal(SIGALRM, SigAlarm_Handler);
        alarm(5);
    } else {
        char cmd[32];//同步涂鸦SDK时间
        sprintf(cmd, "date %02d%02d%02d%02d%02d.%02d", tm.tm_mon +1, tm.tm_mday,
                tm.tm_hour, tm.tm_min, (tm.tm_year +1900) -2000, tm.tm_sec);
        sys_shell(cmd);
        sync_time_2_380(&tm); /* 将时间同步给380 */
    }
}

void iot_state_change(IN CONST GW_WIFI_NW_STAT_E stat) {
    //printf("iot_state_change cloud connect state ***** %d\n", stat);
    sys_log(LOG_CRIT, "iot_state_change cloud connect state ***** %d\n", stat);
    if(STAT_CLOUD_CONN == stat) {
        if(!is_cloud_ready) {
            //通知机器人连接成功
            sys_log(LOG_CRIT, "cloud connect succeed");
            is_cloud_ready = true;
            inform_robot_wifi_state(WIFI_STATE_CONNECTED);
            //云端准备好后，机器上传初始状态到云端
            iot_report_init_state();
            //调用房间分区标识
            robot_api_send_msg_to_hoslam(TUYA_MSG_ROOMS_MAP_IS_EDITABLE, NULL,0);
            //时间同步
            sync_cloud_time();
            delete_timer();
            get_history_record_map();
        }
    } else if(STAT_STA_CONN == stat) {
        if(is_cloud_ready) {
            is_cloud_ready = false;
            inform_robot_wifi_state(WIFI_STATE_DISCONNECTED);
            //NetWork_Monitor_timer();
            init_soft_timer();
        }
    }
    else
    {
        printf("tuya wifi-net status %d no deal\n",stat);
    }
}

// 没切换状态前快速获取配网模式
void iot_wifi_mode_get(void)
{
    GW_WF_NWC_FAST_STAT_T nc_type = 0;
    tuya_iot_wf_fast_get_nc_type(&nc_type);
    if(GWNS_FAST_UNCFG_SMC == nc_type)
    {
        printf("### tuya wifi-net mode GWNS_FAST_UNCFG_SMC %d \n",nc_type);
        //inform_wifi_mode_state((uint8_t)WIFI_SMART_CONFIG_MODE); // 快速配网模式(普通配网模式)
    }
    else if(GWNS_FAST_UNCFG_AP == nc_type)
    {
        printf("### tuya wifi-net mode GWNS_FAST_UNCFG_AP %d\n",nc_type);
        //inform_wifi_mode_state((uint8_t)WIFI_AP_CONFIG_MODE);    // 兼容配网模式
    }
    else
    {
        printf("tuya wifi-net mode %d no deal\n",nc_type);
    }
}

bool iot_state_cloud_ready(void) {
    return is_cloud_ready;
}

static DEV_TYPE_T _tuya_upgrade_dev_type;
void iot_upgrade_download_complete(const char *filepath, DEV_TYPE_T dev_type) {
    _tuya_upgrade_dev_type = dev_type;
    robot_api_fw_upgrade(filepath, NULL);
}

void iot_upgrade_progress(int percent) {
    tuya_iot_dev_upgd_progress_rept(percent, NULL, _tuya_upgrade_dev_type);
}

static void _initSwitcher(DATA_ROBOT_STATE *robot_sta, bool *taskSW, bool *cleanSW) {

    //*taskSW = s_task_sw;
    switch(robot_sta->robot_state) {
    case STATE_SMARTCLEAN:                    // 自动清扫
    case STATE_ZONECLEAN:                    // 区域清扫
    case STATE_PARTCLEAN:                    // 局部清扫
    case STATE_GOTOPOS:                      // 指哪扫哪
    case STATE_GOCHARGE:                     // 回充
    case STATE_CHOICE_CLEAN:                 //选区清扫
    case STATE_REPOSITING:					//重定位中
        *taskSW = true;
        *cleanSW = robot_app_state_running();
        break;
    case STATE_PAUSED:                        // 清扫暂停
        *taskSW = s_task_sw;
        *cleanSW = false;
        break;
    case STATE_ARRIVED:                      // 到达目标点
    case STATE_UNARRIVABLE:                  // 目标点不可达
        *taskSW = s_task_sw;
        *cleanSW = false;
        break;
    case STATE_STANDBY:                     // 待机中
    case STATE_CHARGING:                     // 充电中
    case STATE_CHARGEDONE:                   // 充电完成
    case STATE_ERROR:                       // 故障
    case STATE_SLEEP:                        // 休眠
        *taskSW = false;
        *cleanSW = false;
        break;
    default:
        printf("unknown robot state:%d\n", robot_sta->robot_state);
        *taskSW = false;
        *cleanSW = false;
    }
}

//触发后上报机器的工作状态及风机、音量等配置，配和APP的开关操作
int iot_report_robot_state(DATA_ROBOT_STATE *robot_sta) {
    TY_OBJ_DP_S dp_datas[5];

    bool curTaskSW = 0;
    bool curCleanSW = 0;
    _initSwitcher(robot_sta, &curTaskSW, &curCleanSW);
    printf( "work_mode = %d, %s\n",robot_sta->work_mode,cleanmodeStr[robot_sta->work_mode]);
    printf("stm report st:%s <-> %s %d %d\n", cleanmodeStr[robot_sta->work_mode],
                                    swStateStr[robot_sta->robot_state],
                                    curTaskSW, curCleanSW);
    dp_datas[0].dpid = DPID_MODE; //清扫模式状态
    dp_datas[0].type = PROP_ENUM;
    dp_datas[0].value.dp_enum = robot_sta->work_mode;
    dp_datas[0].time_stamp = 0;

    dp_datas[1].dpid = DPID_POWER_GO; //任务开关状态
    dp_datas[1].type = PROP_BOOL;
    dp_datas[1].value.dp_bool = curTaskSW;
    dp_datas[1].time_stamp = 0;

    dp_datas[2].dpid = DPID_PAUSE; //清扫状态(复用暂停DP,取反理解)
    dp_datas[2].type = PROP_BOOL;
    dp_datas[2].value.dp_bool = curCleanSW;
    dp_datas[2].time_stamp = 0;

    dp_datas[3].dpid = DPID_STATUS; //机器状态
    dp_datas[3].type = PROP_ENUM;
    dp_datas[3].value.dp_bool = robot_sta->robot_state;
    dp_datas[3].time_stamp = 0;

    dp_datas[4].dpid = DPID_FAULT; //错误码
    dp_datas[4].type = PROP_BITMAP;
	dp_datas[4].value.dp_bitmap = robot_sta->error_no;
    dp_datas[4].time_stamp = 0;
    OPERATE_RET op_ret = dev_report_dp_json_async(NULL, dp_datas, 5);
    printf("#### 1 dev_report_dp_json_async return %d\r\n", op_ret);

    return 0;
}

void iot_report_not_disturb(uint32_t dnd_mode)
{
   static int32_t last_dnd_mode = -1;
   
   if(dnd_mode != last_dnd_mode)
   {
        TY_OBJ_DP_S dp_datas;

        dp_datas.dpid = DPID_OPEN_NOT_DISTURB; //勿扰开关
        dp_datas.type = PROP_BOOL;
        dp_datas.value.dp_bool = dnd_mode;
        dp_datas.time_stamp = 0;
        OPERATE_RET op_ret = dev_report_dp_json_async(NULL, &dp_datas, 1);

        last_dnd_mode = dnd_mode;

        printf("dnd_mode dev_report_dp_json_async return %d\r\n", op_ret);
   }
}
// 定时上报清扫结果数据，包括预留的总清扫数据、耗材使用数据，配合APP的结果显示
int iot_report_clean_statis(DATA_CLEAN_STATIS *clean_sta) {
    TY_OBJ_DP_S dp_datas[6];

    dp_datas[0].dpid = DPID_CLEAN_TIME;  //单次清扫时间
    dp_datas[0].type = PROP_VALUE;
    dp_datas[0].value.dp_value = clean_sta->clean_time;
    dp_datas[0].time_stamp = 0;

    dp_datas[1].dpid = DPID_CLEAN_AREA;  //单次清扫面积
    dp_datas[1].type = PROP_VALUE;
    dp_datas[1].value.dp_value = clean_sta->clean_area;
    dp_datas[1].time_stamp = 0;

    dp_datas[2].dpid = DPID_ELECTRICITY_LEFT;  //当前电池电量
    dp_datas[2].type = PROP_VALUE;
    dp_datas[2].value.dp_value = clean_sta->battery_level;
    dp_datas[2].time_stamp = 0;

    dp_datas[3].dpid = DPID_Y_MOP; //y字型拖地
    dp_datas[3].type = PROP_BOOL;
    dp_datas[3].value.dp_bool = clean_sta->y_mod;
    dp_datas[3].time_stamp = 0;

    dp_datas[4].dpid = DPID_OPEN_NOT_DISTURB; //勿扰开关
    dp_datas[4].type = PROP_BOOL;
    dp_datas[4].value.dp_bool = clean_sta->dnd_mode;
    dp_datas[4].time_stamp = 0;

    dp_datas[5].dpid = DPID_CONTINUE_CLEAN; //断点续扫开关
    dp_datas[5].type = PROP_BOOL;
    dp_datas[5].value.dp_bool = clean_sta->continue_clean;
    dp_datas[5].time_stamp = 0;
    
    OPERATE_RET op_ret = dev_report_dp_json_async(NULL, dp_datas,6);
    
    printf("#### 2 dev_report_dp_json_async return %d\r\n", op_ret);
    
    //iot_report_not_disturb(clean_sta->dnd_mode); 

    return 0;
}

// 上报风机状态和水位状态
int iot_report_valve_statis(DATA_CONTROL_VALVE_STATE *valve_sta) {
    TY_OBJ_DP_S dp_datas[2];

//    printf("*** cmw *** valve_sta state: fan:%d, water:%d\r\n", 
//        valve_sta->fan_mode, valve_sta->water_level);
    
    dp_datas[0].dpid = DPID_SUCTION; //吸力选择
    dp_datas[0].type = PROP_ENUM;
    dp_datas[0].value.dp_enum = valve_sta->fan_mode;
    dp_datas[0].time_stamp = 0;

    dp_datas[1].dpid = DPID_WATER_MODE; //水箱档位
    dp_datas[1].type = PROP_ENUM;
    dp_datas[1].value.dp_enum = valve_sta->water_level;
    dp_datas[1].time_stamp = 0;
    
    OPERATE_RET op_ret = dev_report_dp_json_async(NULL, dp_datas, 2);
    printf("#### 3 dev_report_dp_json_async return %d\r\n", op_ret);
    return 0;
}

// 耗材使用数据
int iot_report_consumable_statis(DATA_CONSUMABLE_STATIS *consumable_sta) {
    TY_OBJ_DP_S dp_datas[4];
	memset(dp_datas, 0, sizeof(dp_datas));
	printf(" edge_brush_life:%d, roll_brush_life:%d, filter_life:%d, mop_life:%d \n",\
		consumable_sta->edge_brush_life, consumable_sta->roll_brush_life, consumable_sta->filter_life, consumable_sta->mop_life);
    dp_datas[0].dpid 			= DPID_EDGE_BRUSH_LIFE; 
    dp_datas[0].type 			= PROP_VALUE;
    dp_datas[0].value.dp_value 	= consumable_sta->edge_brush_life;
    dp_datas[0].time_stamp 		= 0;

    dp_datas[1].dpid 			= DPID_ROLL_BRUSH_LIFE; 
    dp_datas[1].type 			= PROP_VALUE;
    dp_datas[1].value.dp_value 	= consumable_sta->roll_brush_life;
    dp_datas[1].time_stamp 		= 0;

	dp_datas[2].dpid 			= DPID_FILTER_LIFE; 
    dp_datas[2].type 			= PROP_VALUE;
    dp_datas[2].value.dp_value 	= consumable_sta->filter_life;
    dp_datas[2].time_stamp 	= 0;

	dp_datas[3].dpid 			= DPID_MOP_LIFE; 
    dp_datas[3].type 			= PROP_VALUE;
    dp_datas[3].value.dp_value 	= consumable_sta->mop_life;
    dp_datas[3].time_stamp 		= 0;
	
    OPERATE_RET op_ret = dev_report_dp_json_async(NULL, dp_datas, 4);
    printf("#### iot_report_consumable_statis, dev_report_dp_json_async return: %d\r\n", op_ret);
    return 0;
}

int iot_report_volume_statis(void) {
    TY_OBJ_DP_S dp_datas;
    OPERATE_RET op_ret;
    
    dp_datas.dpid = DPID_SET_VOLUME;  //当前音量
    dp_datas.type = PROP_VALUE;
    dp_datas.value.dp_value = wifi_app_voice_volume_get();
    dp_datas.time_stamp = 0;
    
    op_ret = dev_report_dp_json_async(NULL, &dp_datas, 1);
    printf("#### 9 dev_report_dp_json_async return %d, %d\r\n", op_ret, dp_datas.value.dp_value);
    
    wifi_app_voice_volume_set(dp_datas.value.dp_value);
    return 0;
}

/* 获取定制开关的值  */
bool get_custom_mode_switch(void)
{
    bool custom_mode_switch;
    FILE *fbp; 
    fbp = fopen("/root/app_server/custom_mode_switch","r");   /* 读取该文件的值 */
    if(NULL == fbp)
    {
        printf("custom_mode_switch fopen error \n");  
        custom_mode_switch = 0;  /* 未配置，默认不开启定制模式开关 */
    }
    else
    {
        fread(&custom_mode_switch,sizeof(bool),1,fbp);
        fclose(fbp);
    }   
    printf(" custom_mode_switch: %d\n", custom_mode_switch);
    return custom_mode_switch;
}

int set_custom_mode_switch(bool flag)
{
	bool custom_mode_switch = flag;
	FILE *fbp;
	printf("\033[1m\033[40;33m custom_mode_switch: %d \033[0m\n", custom_mode_switch);
	fbp = fopen("/root/app_server/custom_mode_switch","w+");   /* 将此开关数据(房间定制策略)存入文件保存 */
	if(NULL == fbp)
	{
		printf("custom_mode_switch fopen error \n");
		return -1;
	}
	 fwrite(&custom_mode_switch,sizeof(bool),1,fbp);
	 fclose(fbp);
	 return 0;
}



/* 上报定制开关的值 */
void iot_report_mode_switch(void)
{   
    TY_OBJ_DP_S switch_datas;
    OPERATE_RET op_ret;
 
    bool custom_mode_switch;
    switch_datas.dpid = DPID_CUSTOM_MODE_SWITCH;  
    switch_datas.type = PROP_BOOL;
    switch_datas.value.dp_bool = get_custom_mode_switch();
    switch_datas.time_stamp = 0;
    
    op_ret = dev_report_dp_json_async(NULL, &switch_datas, 1);
    printf("#### 16 dev_report_dp_json_async return %d\r\n", op_ret);
    
}

/* 上报语音包的信息  */
void iot_report_voice_version(void)
{   
    uint8_t voice_version[20];

    memset(voice_version, 0, sizeof(voice_version));

    wifi_app_voice_version_get(voice_version,sizeof(voice_version));
    send_voice_loading_ret(voice_version,VOICE_USEING,100); /* 上报当前语音包信息 */
    
}

int iot_report_init_state(void) {
    DATA_CLEAN_STATIS *p_clean_statis = iot_get_last_clean_statis();
    int map_flag;
    
    iot_report_valve_statis(&last_data_valve_sta);
    iot_report_robot_state(&last_data_robot_sta);
	//iot_report_consumable_statis(&last_data_consumable_sta);
    iot_report_volume_statis();
    iot_report_mode_switch();
    iot_report_voice_version();
    if(robot_app_state_running()) {
        iot_report_clean_statis(p_clean_statis);
    } else {
        TY_OBJ_DP_S dp_datas[2];
        dp_datas[0].dpid = DPID_ELECTRICITY_LEFT;  //非运动状态，只更新电量
        dp_datas[0].type = PROP_VALUE;
        dp_datas[0].value.dp_value = p_clean_statis->battery_level;
        dp_datas[0].time_stamp = 0;

        dp_datas[1].dpid = DPID_SUCTION;  //初始状态为正常
        dp_datas[1].type = PROP_ENUM;
        dp_datas[1].value.dp_value = FAN_NORMAL;
        dp_datas[1].time_stamp = 0;

        OPERATE_RET op_ret = dev_report_dp_json_async(NULL, dp_datas, 2);
        printf("#### 4 dev_report_dp_json_async return %d\r\n", op_ret);
        map_flag = get_map_is_emtry();
        if(0 == map_flag)
        {
            wifi_app_reset_path();
            wifi_app_reset_map();
        }
        else
        {
            //send_map_rooms_data();
            robot_api_send_msg_to_hoslam(TUYA_MSG_TYPE_GET_VIRTUAL_WALL,NULL,0);
        }
        robot_api_send_msg_to_hoslam(TUYA_MSG_ROOMS_MAP_IS_EDITABLE, NULL,0);   
    }
    return 0;
}

int iot_report_robot_all_state(void) {
    DATA_CLEAN_STATIS *p_clean_statis = iot_get_last_clean_statis();

    iot_report_valve_statis(&last_data_valve_sta);
    iot_report_robot_state(&last_data_robot_sta);
    //iot_report_consumable_statis(&last_data_consumable_sta);
	
    iot_report_clean_statis(p_clean_statis);

    printf("#### 8 dev_report_dp_json_async return %d\r\n");
    return 0;
}


void iot_robot_map_reset(void) {
    //通知云端 地图状态复位
    if(iot_state_cloud_ready()) {
        //清空手机APP(云端)地图信息复位
        wifi_app_reset_robot_map_data();
    }
}
