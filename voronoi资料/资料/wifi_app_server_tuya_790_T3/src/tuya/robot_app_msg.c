#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>
#include <string.h>
#include "tuya_iot_com_api.h"
#include "tuya_cloud_types.h"
#include "tuya_iot_wifi_api.h"
#include "uni_log.h"
#include "sys_timer.h"
#include "robot_api.h"
#include "robot_app_msg.h"
#include "robot_app_factory_test.h"
#include "../../robot_api_src/wifi_app_server.h"
#include "sys_api.h"
#include "tuya_iot_msg_parse.h"
#include "tuya_iot_state_adapter.h"
#include "tuya_iot_msg_parse.h"
#include "../../robot_api_src/robot_api_inner.h"
#include "tuya_iot_uploader.h"
#define APPSERVER_TYPE         TUYA   /* appserver类型 ： 涂鸦，愚公，米家*/

static int sleep_flag_switch;
// 协议命令






typedef struct
{
    int         start;          // 远程方向控制标志
    int         local_cmd;      // 正在执行的控制命令
    int         remote_cmd;     // 远程控制命令
    int         timeout_count;  // 超时计数
    sys_timer_t *sys_timer;     // 定时器句柄

} REMOTE_CONTROL_MGR;
static REMOTE_CONTROL_MGR   remote_control_mgr;

static ROBOT_STATE_INFO        robot_state_info;
static uint32_t             wifi_config;
static uint32_t             sleep_flag = 0;
static uint8_t              uuid_fail_flag = 0;
static uint8_t              voice_serial_num = 0;

/* 返回机器人睡眠状态 */
uint32_t get_robot_sleep_state(void)
{
	return sleep_flag;
}

/* 返回机器人380上报的当前状态 */
uint8_t return_robot_state_by_380(void)
{
    return robot_state_info.robot_state;
}

/* 返回机器人380上报的重定位状态  */
uint8_t return_robot_reposition_state_by_380(void)
{
    return robot_state_info.robot_reposition_state;
}
static int inform_voice_version(void);
// 远程方向控制处理，用于适配380和APP不同的响应速度
static void remote_control_timer(void) {
    int     cmd = remote_control_mgr.remote_cmd;

    if (cmd == CTRL_STOP_MOVING)
    {
        sys_timer_delete(remote_control_mgr.sys_timer);
        remote_control_mgr.start         = 0;
        remote_control_mgr.sys_timer     = NULL;
        remote_control_mgr.remote_cmd    = -1;
        remote_control_mgr.local_cmd     = -1;
        remote_control_mgr.timeout_count = 0;
        printf("remote control stop\n");
    }
    else if (cmd != -1)
    {
        robot_api_control(ROBOT_CMD_MANUAL_CONTROL, (void *)cmd);
        remote_control_mgr.local_cmd     = cmd;
        //remote_control_mgr.remote_cmd    = -1;  tuya
        remote_control_mgr.timeout_count = 0;
        sys_timer_start(remote_control_mgr.sys_timer);
    }
    else
    {
        remote_control_mgr.timeout_count++;
        if (remote_control_mgr.timeout_count > 60)
        {
            sys_timer_delete(remote_control_mgr.sys_timer);
            remote_control_mgr.start         = 0;
            remote_control_mgr.sys_timer     = NULL;
            remote_control_mgr.remote_cmd    = -1;
            remote_control_mgr.local_cmd     = -1;
            remote_control_mgr.timeout_count = 0;
            printf("remote control timeout\n");
        }
        else
        {
            cmd = (uint8_t)remote_control_mgr.local_cmd;
            robot_api_control(ROBOT_CMD_MANUAL_CONTROL, (void *)cmd);
            sys_timer_start(remote_control_mgr.sys_timer);
        }
    }
}

int robot_start_remote_control(int cmd) {
    sys_timer_t *timer;
    if (remote_control_mgr.start) {
        remote_control_mgr.remote_cmd = cmd;
        robot_api_control(ROBOT_CMD_MANUAL_CONTROL, (void *)cmd);
        return 0;
    }

    if (cmd == CTRL_STOP_MOVING) {
        return 0;
    }

    timer = sys_timer_create(200, remote_control_timer);
    if (!timer) {
        printf("sys_timer_create remote_control_timer ERROR\n");
        return -1;
    }

    remote_control_mgr.start         = 1;
    remote_control_mgr.sys_timer     = timer;
    remote_control_mgr.remote_cmd    = cmd;
    remote_control_mgr.local_cmd     = -1;
    remote_control_mgr.timeout_count = 0;

    if (sys_timer_start(remote_control_mgr.sys_timer) == -1) {
        sys_timer_delete(timer);
        remote_control_mgr.start         = 0;
        remote_control_mgr.sys_timer     = NULL;
        remote_control_mgr.remote_cmd    = -1;
        remote_control_mgr.local_cmd     = -1;
        remote_control_mgr.timeout_count = 0;
        return -1;
    }

    printf("remote control start\n");
    return 0;
}

OPERATE_RET iot_wf_gw_unactive_custom_mode(GW_WF_START_MODE wifi_mode);

static int start_config_network(uint8_t *pdata, int size) {
    //从SDK移除设备 回充键 + 电源键同时长按7s触发调用
    OPERATE_RET op_ret;
#if 0
    /* 随机配网模式 */
    op_ret = tuya_iot_wf_gw_unactive();
#else
    /* 通过底层来指定配网模式 */
    // 初始化默认配网模式为 smart
    GW_WF_START_MODE wifi_mode = WF_START_SMART_FIRST;
    //printf(" ### wifi_app_msg_config_network size:%d, data:%d,%d ###\r\n", size, pdata[0], pdata[1]);
    sys_log(LOG_CRIT, " ### wifi_app_msg_config_network size:%d, data:%d,%d ###\r\n", size, pdata[0], pdata[1]);
    if(1 == pdata[1])
    {
        wifi_mode = WF_START_SMART_FIRST;
        //printf("$$$$$ tuya_iot_wf_gw_unactive trun, mode: WF_START_SMART_FIRST $$$$$\r\n");
        sys_log(LOG_CRIT, " $$$$$ tuya_iot_wf_gw_unactive trun, mode: WF_START_SMART_FIRST $$$$$");
    }
    else if(0 == pdata[1])
    {
        wifi_mode = WF_START_AP_FIRST;
        //printf("$$$$$ tuya_iot_wf_gw_unactive trun, mode: WF_START_AP_FIRST $$$$$\r\n");
        sys_log(LOG_CRIT, " $$$$$ tuya_iot_wf_gw_unactive trun, mode: WF_START_AP_FIRST $$$$$");
    }
    else
    {
        printf("tuya_iot_wf_gw_unactive fail, mode:%d", pdata[1]);
        sys_log(LOG_CRIT, "tuya_iot_wf_gw_unactive fail, mode:%d", pdata[1]);
    }
    pthread_mutex_lock(&wifi_connect_ptx);  /* 检查涂鸦SDK是否初始化完成，若未完成，则等待SDK初始化完成 */
    op_ret = iot_wf_gw_unactive_custom_mode(wifi_mode);
#endif
    if (op_ret != OPRT_OK){
        //PR_ERR("tuya_iot_wf_gw_unactive op_ret:%d",op_ret);
        inform_robot_wifi_state(WIFI_STATE_CONFIG_FAILD);
        sys_log(LOG_CRIT, "tuya_iot_wf_gw_unactive op_ret:%d",op_ret);
        return op_ret;
    }
    pthread_mutex_unlock(&wifi_connect_ptx); 
    //PR_DEBUG("tuya_iot_wf_gw_unactive success");
    sys_log(LOG_CRIT, "tuya_iot_wf_gw_unactive success");
    return op_ret;
}
#if 0 //feng
static bool hoslam_wake_up;
static void try_enter_sleep(int32_t mode) {
   if (sleep_flag != SLEEP_DEEP_WIFI_ON && mode == SLEEP_DEEP_WIFI_ON)
    {	
        hoslam_wake_up = false;
        robot_api_hoslam_enter_sleep();
        sys_shell("%s stop", SHELL_STARTUP_AUDIO);
        sys_shell("echo powersave > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
    }
    else if (sleep_flag != SLEEP_DEEP_WIFI_OFF && mode == SLEEP_DEEP_WIFI_OFF)
    {
        hoslam_wake_up = false;
        robot_api_hoslam_enter_sleep();
        sys_shell("%s stop", SHELL_STARTUP_AUDIO);
        sys_shell("%s stop", SHELL_STARTUP_NETWORK);
        sys_shell("%s stop", SHELL_STARTUP_WIFI);
        sys_shell("echo powersave > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
    }
    if(sleep_flag == SLEEP_NONE) {
        printf("enter powersave: %d\n", sleep_flag);
    }
}
#endif
//feng
static void try_enter_sleep(int32_t mode){
   if (sleep_flag != SLEEP_DEEP_WIFI_ON && mode == SLEEP_DEEP_WIFI_ON)
    {
        //hoslam_wake_up = false;
        //robot_api_hoslam_enter_sleep();
        sys_shell("%s stop", SHELL_STARTUP_AUDIO);
        sys_shell("echo powersave > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
    }
    else if (sleep_flag != SLEEP_DEEP_WIFI_OFF && mode == SLEEP_DEEP_WIFI_OFF)
    {
        //hoslam_wake_up = false;
        //robot_api_hoslam_enter_sleep();
        sys_shell("%s stop", SHELL_STARTUP_AUDIO);
        sys_shell("%s stop", SHELL_STARTUP_NETWORK);
        sys_shell("%s stop", SHELL_STARTUP_WIFI);
        sys_shell("echo powersave > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
    }
    sleep_flag = mode;
    printf("enter powersave: %d\n", sleep_flag);
	printf("\033[1m\033[40;33m[%s:%d] try_enter_sleep... \033[0m\n",__FUNCTION__,__LINE__);
}

static void exit_sleep(void) {

    if (sleep_flag == SLEEP_DEEP_WIFI_ON) {
        sys_shell("echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
        sys_shell("%s start", SHELL_STARTUP_AUDIO);
		printf("\033[1m\033[40;33m[%s:%d] SLEEP_DEEP_WIFI_ON--->exit_sleep \033[0m\n",__FUNCTION__,__LINE__);
    } else if (sleep_flag == SLEEP_DEEP_WIFI_OFF) {
        sys_shell("echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
        sys_shell("%s start", SHELL_STARTUP_AUDIO);
        sys_shell("%s start", SHELL_STARTUP_WIFI);
        sys_shell("%s start", SHELL_STARTUP_NETWORK);
		printf("\033[1m\033[40;33m[%s:%d]  SLEEP_DEEP_WIFI_OFF--->exit_sleep \033[0m\n",__FUNCTION__,__LINE__);
    }
	sleep_flag = SLEEP_NONE;
    printf("exit powersave: %d\n", sleep_flag);
}

static int robot_state_info_process(ROBOT_STATE_INFO *info) {
    // 判断系统状态是否改变
    if (memcmp(info, &robot_state_info, sizeof(robot_state_info)) == 0)
    {
        // 没有下方数据就不上传
        if(!get_force_update_state())
        {
            return 0;            
        }
    }
    //set_force_update_state(0);
    
    memcpy(&robot_state_info, info, sizeof(robot_state_info));  

    printf( "state:%d battery:%d area:%d time:%d errno:%d fan:%d water:%d con_clean:%d y_type_mop:%d dnd_mode:%d obstacles_cross:%d cliff_detect:%d edge_brush_life:%d roll_brush_life:%d filter_life:%d mop_life:%d\n",
            info->robot_state, info->battery_level, info->clean_area, info->clean_time, info->error_info, info->fan_mode,
            info->water_level, info->continue_clean, info->y_type_mop, info->dnd_mode, info->obstacles_cross, info->cliff_detect,
             info->edge_brush_life, info->roll_brush_life, info->filter_life, info->mop_life);
#if 0
    // 休眠处理
    if (info->robot_state == ROBOT_STATE_SLEEP && info->sleep_mode != SLEEP_NONE)
    {
        try_enter_sleep(info->sleep_mode);
        sleep_flag = info->sleep_mode;
    }
    else if (info->robot_state != ROBOT_STATE_SLEEP && sleep_flag != SLEEP_NONE)
    {
        if(!hoslam_wake_up) {
            exit_sleep();
        }
        sleep_flag = SLEEP_NONE;
    }
#endif	
    iot_robot_state_received(&robot_state_info);
    return 0;
}

static int recv_robot_state_parse(uint8_t *data, uint32_t len) {
    int32_t value;
    ROBOT_STATE_INFO info;
    static char send_choice_clean_flag;

    // 初始化结构体
    memset(&info, 0, sizeof(info));

    // 主状态
    value = data[3];
    info.robot_state = value;

    if(info.robot_state == ROBOT_STATE_WALLFOLLOW || info.robot_state == ROBOT_STATE_CLEAN_ROOM
		|| info.robot_state == ROBOT_STATE_CLEAN_AUTO || info.robot_state == ROBOT_STATE_CLEAN_RECTS
		|| info.robot_state == ROBOT_STATE_CLEAN_SPOT || info.robot_state == ROBOT_STATE_GOTO_POS 
		|| info.robot_state == ROBOT_STATE_REMOTE_DRIVE || info.robot_state == ROBOT_STATE_DOCK || info.robot_state == ROBOT_STATE_CHOICE_CLEAN)
	{
		GET_WORKING_ROOMS_FLAG = 1;
	}
	else
	{
		GET_WORKING_ROOMS_FLAG = 0;
	}
    printf("GET_WORKING_ROOMS_FLAG = %d\n", GET_WORKING_ROOMS_FLAG);

    if(ROBOT_STATE_GOTO_POS == info.robot_state)
    {
        send_goto_pos_flag = 1;
    }
  
   if (send_goto_pos_flag && NULL != save_navigation_path 
                            && info.robot_state != ROBOT_STATE_GOTO_POS)
   {
        printf("free save_navigation_path\n");
        free(save_navigation_path);   
        save_navigation_path = NULL;
   }

    if(send_goto_pos_flag && info.robot_state != ROBOT_STATE_GOTO_POS)
    {
        send_goto_pos_flag = 0;
    }

    // 水箱调档
    value = data[4];
    info.water_level = value;

    // 风机设置
    value = data[5];
    info.fan_mode = value;

    // 休眠模式
    value = data[6];
    switch (value)
    {
        case 0x01:
            info.sleep_mode = SLEEP_SHALLOW;
            break;
        case 0x02:
            info.sleep_mode = SLEEP_DEEP_WIFI_ON;
            break;
        case 0x03:
            info.sleep_mode = SLEEP_DEEP_WIFI_OFF;
            break;
        default:
            info.sleep_mode = SLEEP_NONE;
            break;
    }

    // 断点续扫开关
    value = data[7];
    info.continue_clean = value;

    // Y字拖地开关
    value = data[8];
    info.y_type_mop = value;

    // 错误代码
    value = data[24];
    info.error_info = value;

    // 剩余电量
    value = data[25];
    info.battery_level = value;

    // 清扫面积
    value = (data[27] << 8) | data[26];
    info.clean_area = value;

    // 清扫时间
    value = (data[29] << 8) | data[28];
    info.clean_time = value;

    // 勿扰模式
    value = data[34];
    info.dnd_mode = value;

    // 越障功能
    value = data[35];
    info.obstacles_cross = value;

    // 地检开关
    value = data[36];
    if (value == 0x01)
    {
        info.cliff_detect = 0;
    }
    else
    {
        info.cliff_detect = 1;
    }
	//重定位状态
	info.robot_reposition_state = data[37];
	
	//耗材寿命
	info.edge_brush_life =  (data[38] << 24) |  (data[39] << 16) | (data[40] << 8) | (data[41] << 0) ;
	info.roll_brush_life =  (data[42] << 24) |  (data[43] << 16) | (data[44] << 8) | (data[45] << 0) ;
	info.filter_life   	 =  (data[46] << 24) |  (data[47] << 16) | (data[48] << 8) | (data[49] << 0) ;
	info.mop_life 		 =  (data[50] << 24) |  (data[51] << 16) | (data[52] << 8) | (data[53] << 0) ;

    // 版本号
    snprintf(info.version, sizeof(info.version), "%d.%d.%d", data[30], data[31], data[32]);

    // 系统状态处理
    robot_state_info_process(&info);
}

static uint8_t cal_packet_checksum(uint8_t *p) {
	uint8_t sum = 0, i = 0, lenght;
	uint8_t *data;

    lenght = p[1];
    if (lenght == 0) //长度的合法性
    {
        return 0;
    }

    data = p + 2;
    for (i = 0; i < lenght; i++)
    {
  	    sum += data[i];
    }

    return sum;
}

/* 获取appserver类型和系统版本 */
int get_sys_version(APPSERVER_TYPE_ENUM *appserver_type,uint8_t *sys_version)
{

    uint8_t sys_version_read[8] = {0};
    uint8_t tmp[256] = {0};
    uint32_t size;
    char * point = NULL;
    FILE *fp = NULL;

    *appserver_type = APPSERVER_TYPE;

    fp = fopen("/version","r");
    if(fp == NULL)
    {
        printf("open sys version failed\n");
        return -1;
    }
    //求得文件的大小
	fseek(fp, 0, SEEK_END);
	size = ftell(fp);
	rewind(fp);

    if(size >= 256)
    {
        printf("sys version size is too large = %d\n", size);
        return -1;
    }
    //读文件
	fread(tmp,1,size,fp);
    /*eg . 94a359afd 2019.09.16 17:54:38*/
    point = strstr(tmp,"."); /*检索第一个.号*/
    if(point != NULL)
    {
        if(point[3] != '.')  /*若存在第二个 . ,则证明数据格式是对的，则拷贝系统版本 */
        {
            printf("decode sys version failed \n");
            return -1;
        }
        memcpy(&sys_version_read[0],(point - 4),4);
        memcpy(&sys_version_read[4],(point + 1),2);
        memcpy(&sys_version_read[6],(point + 4),2);

        memcpy(sys_version,sys_version_read,8);
    }

	fclose(fp);

}
void robot_app_packet_cb(uint8_t *data, uint32_t len) {
    uint8_t header = data[0];
    uint8_t length = data[1];
    uint8_t cmd = data[2];
    uint8_t checksum = data[length + 2];
    uint8_t value;
    int ret;
#define WIFI_APP_DATA_PROTOCOL_HEAD         0xAA
    // 判断协议头
    if (header != WIFI_APP_DATA_PROTOCOL_HEAD)
    {
        printf("recv packet header ERROR\n");
        return;
    }

    // 判断校验和
    value = cal_packet_checksum(data);
    if (checksum != value)
    {
        printf("recv packet checksum ERROR\n");
        return;
    }

    /* 等到收到消息就报一下uuid 错误 */
    if(1 == uuid_fail_flag)
    {
        inform_robot_wifi_state(WIFI_STATE_UUID_FAILD);
        uuid_fail_flag = 0;
    }
    //printf("recv packet : cmd = %d,length = %d\n", cmd,length);
    // 根据命令类型分别处理
    switch (cmd)
    {
        case CMD_GET_ROBOT_STATE:       // 上报机器状态
            recv_robot_state_parse(data + 3, length -1);
            break;
    
        case CMD_PLAY_VOICE:            // 播放语音
            printf("length = %d\n",length);
            // if (length == 3 ||length == 4) {
            //     value = data[3];
			// 	voice_serial_num = data[5];
            //     //printf("recv voice = %d\n", value);
            //     robot_api_play_voice(value);
            // }
            printf("data[3] = %d,data[4] = %d,data[5 ]= %d\n",data[3],data[4],data[5]);
            voice_serial_num = data[5];
            if (length == 3)
            {
                wifi_app_voice_play_new(data[3], data[4], 0);
            }
            else if (length == 4)
            {
                wifi_app_voice_play_new(data[3], data[4], data[5]);
            }

            break;

        case CMD_CONFIG_NETWORK:        // 配网命令
            sys_log(LOG_CRIT, "wifi config start");
            wifi_config = 1;
            inform_robot_wifi_state(WIFI_STATE_WATIING_CONFIG);
            start_config_network(data + 3, length-1);
            break;

        case CMD_VOICE_VERSION:         // 获取语音版本
            printf("request get voice version\n");
            inform_voice_version();
            break;
        case CMD_VOICE_GAIN:            // 设置语音增益大小
            printf("request CMD_VOICE_GAIN\n");
            if (length == 2)
            {
                printf("request CMD_VOICE_GAIN: %d\n", data[3]);
                value = data[3];
                if (data[3] == 0xff)
                {
                    value = wifi_app_voice_volume_get();
                    robot_api_send_packet(CMD_VOICE_GAIN, &value, 1);
                }
                else
                {
                    wifi_app_voice_volume_set(value);
                }
            }
            break;
        
        case CMD_WIFI_TEST:             // WiFi测试
            printf("request to wifi test\n");
            robot_app_factory_wifi_test(data+3, data+35);			
            break;

        case CMD_FACTORY_TEST:          // 厂测命令
            printf("request to factory test");
            value = data[3];
            wifi_app_factory_test(value);
            break;

        case CMD_GET_SYS_APPSERVER_TYPE:          // 获取appserver类型以及系统版本
            printf("gei sys & appserver type\n");
            {
                static uint8_t packet_buffer[9]; 
                get_sys_version((APPSERVER_TYPE_ENUM *)&packet_buffer[0],&packet_buffer[1]);
                robot_api_send_packet(CMD_GET_SYS_APPSERVER_TYPE, packet_buffer, 9);
            }
             
            break;

        case CMD_EXTEND:          //高级指令
            value = data[3];
            switch (value)
            {
                case KEY_EXT_CLEAN_DONE:    // 清扫完成标志
                    break;
                case KEY_EXT_REBOOT_FACTOY:
                    sys_shell("rm /root/app_server/custom_mode_switch");
                    break;      
                default:
                    break;
            }
            break;

        case CMD_DND_MODE_TIME:          // 获取勿扰模式时间段
            printf("get CMD_DND_MODE_TIME\n");
            printf("dnd mode zone = %d,s_hour = %d,s_min= %d,e_hour = %d,e_min= %d\n",data[3],data[4],data[5],data[6],data[7]);
            if(iot_state_cloud_ready()) 
            {
                report_dnd_mode_time(data[3],data[4],data[5],data[6],data[7]);
            }
            break;    

        default:
            break;
    }
}

void set_uuid_fail_flag(int flag)
{
    uuid_fail_flag = flag;
}

bool robot_app_allowed_upgrade(void) {
    bool ret = false;

    if (// 电量为0，可能是380无法正常运行、380和580固件不匹配等原因造成，此时允许尝试升级
        robot_state_info.battery_level == 0
        // 待机或休眠状态且电量大于30%
        || ((robot_state_info.robot_state == ROBOT_STATE_IDLE
            || robot_state_info.robot_state == ROBOT_STATE_PAUSED
            || robot_state_info.robot_state == ROBOT_STATE_SLEEP)
            && robot_state_info.battery_level >= 30)
        // 直充或座充状态
        || robot_state_info.robot_state == ROBOT_STATE_SEAT_CHARGING
        || robot_state_info.robot_state == ROBOT_STATE_DIRECT_CHARGING) {
        ret = true;
    }
    
    return ret;
}

bool robot_app_state_running(void) {
    int32_t state = robot_state_info.robot_state;
	#if 0
    return (state >= ROBOT_STATE_WALLFOLLOW && state < ROBOT_STATE_SEAT_CHARGING);
	#else
	if ((state >= ROBOT_STATE_WALLFOLLOW && state < ROBOT_STATE_SEAT_CHARGING) || \
		robot_state_info.robot_reposition_state == ROBOT_REPOSITING )
	{
		return true;
	}
	else
	{
		return false;

	}
	#endif
}

bool robot_app_state_sleep(void) {
    return robot_state_info.robot_state == ROBOT_STATE_SLEEP;
}

char *robot_app_fw_version(void) {
    return robot_state_info.version;
}
#if 0
void robot_app_handle_sys_msg_wakeup(void) {
    if(robot_state_info.robot_state == ROBOT_STATE_SLEEP && !hoslam_wake_up) {
        hoslam_wake_up = true;
        exit_sleep();
    }
}
#endif

//feng
void robot_app_handle_sys_msg_wakeup(void) 
{
	printf("\033[1m\033[40;33m[%s:%d] robot_app_handle_sys_msg_wakeup \033[0m\n",__FUNCTION__,__LINE__);
}

void robot_app_handle_sys_msg_sleep(void) 
{
    sys_log(LOG_INFO, "arobot dev sleep");
    sleep(1);
    sys_shell("echo standby > /sys/power/state");
    sys_log(LOG_INFO, "arobot dev wake");
    
}



void robot_app_handle_sys_msg_voice_end(void) {
#if 0
    uint8_t value = 1;
    // 发送命令
    robot_api_send_packet(CMD_PLAY_VOICE, &value, sizeof(value));
#else
	uint8_t data[2];
	data[0] = 0x01;
	data[1] = voice_serial_num;
	// 发送命令
	robot_api_send_packet(CMD_PLAY_VOICE, data, sizeof(data));

    printf("voice audio disabled = %d\n",voice_serial_num);
#endif
}


int inform_robot_wifi_state(int wifi_state) {
    uint8_t buf[4];
    int i = 0;
    // 休眠状态下不需要通知WiFi状态
    if (sleep_flag) {
        return 0;
    }

    switch (wifi_state) {
        case WIFI_STATE_FAULT:
            buf[i++] = 0xFF;
            buf[i++] = 0xFF;
            buf[i++] = 0xFF;
            break;
        case WIFI_STATE_CONNECTED:
            if (wifi_config)
            {
                wifi_config = 0;
                buf[i++] = 1;
                buf[i++] = 1;
                buf[i++] = 1;
            }
            else
            {
                buf[i++] = 0;
                buf[i++] = 1;
                buf[i++] = 1;
            }
            break;
        case WIFI_STATE_CONFIG_FAILD:
            if (wifi_config)
            {
                //wifi_config = 0;
                buf[i++] = 1;
                buf[i++] = 1;
                buf[i++] = 0;
            }
            else
            {
                buf[i++] = 0;
                buf[i++] = 0;
                buf[i++] = 0;
            }
            break;
        case WIFI_STATE_WATIING_CONFIG:
            wifi_config = 1;
            buf[i++] = 1;
            buf[i++] = 0;
            buf[i++] = 0;
            break;
        case WIFI_STATE_DISCONNECTED:
            if (!wifi_config)
            {
                buf[i++] = 0;
                buf[i++] = 1;
                buf[i++] = 0;
            }
            break;
        case WIFI_STATE_UUID_FAILD:
            buf[i++] = 0xEE;
            buf[i++] = 0xEE;
            buf[i++] = 0xEE;
            break;
        default:
            printf("unkown wifi state: %d\n", wifi_state);
            break;
    }
    if (i > 0) {
        robot_api_send_packet(CMD_WIFI_STATE, buf, i);
    }
    return 0;
}

int inform_robot_wifi_mode_state(int state) {
    uint8_t value = state;
    int ret;

    printf("### tuya wifi-net inform_wifi_mode_state %d $$$\n",state);
    // 发送命令
    ret = robot_api_send_packet(CMD_CONFIG_NETWORK, &value, sizeof(value));
    if(-1 == ret) {
        printf("ERROR: send_packet return -1\r\n");
    }
    return ret;
}

int inform_robot_upgrade_state(int state) {
    uint8_t value = state;
    // 发送命令
    robot_api_send_packet(CMD_UPGRADE_STATUS, &value, sizeof(value));
    return 0;
}

int inform_robot_test_result(int result, int value) {
    uint8_t packet_buffer[2];
    packet_buffer[0] = result;
    packet_buffer[1] = value;

    robot_api_send_packet(CMD_WIFI_TEST, packet_buffer, 2);
    return 0;
}

static int inform_voice_version(void) {
    uint8_t v1 = 0, v2 = 0, v3 = 0, v4 = 0;
    char tmp[128];
    uint8_t packet_buffer[20];
    int i = 0, ret = 0;
    do {
        memset(tmp, 0, sizeof(tmp));
        ret = sys_cat_file(FILE_VOICE_VERSION, tmp, sizeof(tmp));
        if (ret)
        {
            printf("cat voice version failed\n");
            break;
        }

        sscanf(tmp, "%d.%d.%d.%d", &v1, &v2, &v3, &v4);
        printf("get voice version: %d.%d.%d.%d\n", v1, v2, v3, v4);

    } while(0);

    memset(packet_buffer, 0, sizeof(packet_buffer));
    packet_buffer[i++] = v1;
    packet_buffer[i++] = v2;
    packet_buffer[i++] = v3;
    packet_buffer[i++] = v4;
    robot_api_send_packet(CMD_VOICE_VERSION, packet_buffer, i);
    return 0;
}

void robot_set_wifi_config_state(void) {
    wifi_config = 1;
}

int  robot_get_wifi_config_state(void) {
   return wifi_config;
}


int recv_set_clean_strategy(uint8_t *data)
{
    int i;
    int result;
    room_spec_arg_t* p_rspec_arg = robot_ctl_get_room_spec_arg(); /* 获取本地房间信息结构体 */
    result = *(int *)data;
    printf( "recv rooms set result: %d", result);
    char buffer[8];
    memset(buffer,0,sizeof(buffer));
    buffer[0] = KEY_EXT_ROOM_SET;

    for(i = 0; i < p_rspec_arg->num;i++)
    {
        if(p_rspec_arg->room_spec[i].id == result)
        {
            buffer[1] = p_rspec_arg->room_spec[i].bundle[0];
            buffer[2] = p_rspec_arg->room_spec[i].bundle[1];
            buffer[3] = !(p_rspec_arg->room_spec[i].bundle[2]);
            buffer[4] = p_rspec_arg->room_spec[i].bundle[3];
            robot_api_send_packet(CMD_EXTEND,buffer,5);
            printf("room id = %d,fan = %d,water = %d,y_mod = %d,ground = %d\n",result,buffer[0],buffer[1],buffer[2],buffer[3]);
        }
    }
}

/* 将时间同步给380 */
void sync_time_2_380(POSIX_TM_S *tm)
{
    char buffer[5];
    memset(buffer,0,sizeof(buffer));

    buffer[0] = KEY_CTRL_SYNC_TIME_380;
    buffer[1] = tm -> tm_hour;
    buffer[2] = tm -> tm_min;
    buffer[3] = tm -> tm_sec;
    buffer[4] = tm -> tm_wday;

    robot_api_send_packet(CMD_CONTROL, buffer, 5);
    printf("sync time to 380 hour: %d,min: %d,sec: %d,wday %d\n", tm -> tm_hour, tm -> tm_min,tm -> tm_sec,tm -> tm_wday);

    
}
