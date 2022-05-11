#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#include "sys_api.h"
#include "sys_timer.h"
#include "robot_api.h"
#include "robot_api_inner.h"
#include "wifi_app_server.h"
#include "wifi_app_voice_mgr.h"
#include "wifi_app_firmware_upgrade.h"

char u_log_flag  = 0;
char tf_card;

static char *cmdStr[] = {
    "invalide",
    "autoclean",
    "dock",
    "pause",
    "goto",
    "spotclean",
    "rectclean",   //设置区域，并且重新开始划区清扫
    "rectresume",  //暂停后，使用该命令继续划区清扫
    "wallfollow",         //沿边清扫
    "setvwall",
    "setforbidden",     //10
    "fanmode",
    "waterlevel",
    "manual",
    "appointmenton",       //预约开
    "appointmentoff",      //预约关
    "fullgo",
    "resetmap",
    "disturb",
    "calling_robot",
    "continue_clean",
    "continueroomsclean",
    "continuenewroomsclean",
    "ymodswitch",
    "restorefactory",
    "sync_time_to_380",
    "edge_brush_life",
    "roll_brush_life",
    "filter_life",
    "mop_life"
};

//static uint8_t send_buffer[128];
static const ROBOT_CB_FUNS  *m_cbs;

static uint8_t cal_packet_checksum(uint8_t *p)
{
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

int is_allow_to_upgrade(void)
{
    int mem_free, disk_free;

    // 判断系统剩余内存，当内存少于10M时，不允许升级
    mem_free = sys_get_mem_free();
    if (mem_free < (10 * 1024 * 1024)) {
        printf("out of memory, can not upgrade, mem_free: %d Byte\n", mem_free);
        return -1;
    }

    // 先判断固件分区剩余空间
    disk_free = sys_get_firmware_partition_free();
    if (disk_free > 0)
    {
        if (disk_free < (4 * 1024 * 1024))
        {
            printf("out of flash space, can not upgrade, firmware partition free: %d Byte\n", disk_free);
            return -2;
        }
    }
    else
    {
        // 判断Flash剩余空间，当少于6M时，不允许升级
        disk_free = sys_get_disk_free();
        if (disk_free < (6 * 1024 * 1024))
        {
            printf("out of disk space, can not upgrade, disk_free: %d Byte\n", disk_free);
            return -2;
        }
    }

    return m_cbs->allow_upgrade_cb();
}

// 接收状态数据 WiFi App Server <-- Hoslam App
int recv_packet(uint32_t type, uint8_t *data) {
    uint32_t len;
    char map_file[128];

    memcpy(&len, data, sizeof(len));

    printf("type = %d\n",type);
    
    switch (type) {
        case MSG_TYPE_UPDATE_APP_INFO:
            m_cbs->app_cb(data + sizeof(len), len);
            break;
        case MSG_TYPE_RESET_MAP:
            m_cbs->sys_cb(ROBOT_SYS_MSG_MAP_RESET, NULL);
            break;
        case MSG_TYPE_WAKEUP:
			printf("\033[1m\033[40;33m[%s:%d]recv_packet form Hoslam MSG_TYPE_WAKEUP \033[0m\n",__FUNCTION__,__LINE__);
            m_cbs->sys_cb(ROBOT_SYS_MSG_WAKEUP, NULL);
            break;
        case MSG_TYPE_SLEEP:
			printf("\033[1m\033[40;33m[%s:%d] recv_packet form Hoslam MSG_TYPE_SLEEP \033[0m\n",__FUNCTION__,__LINE__);
            m_cbs->sys_cb(ROBOT_SYS_MSG_SLEEP, NULL);//feng
            break;
        case MSG_TYPE_SET_VW_FZ_RESULT:
            m_cbs->sys_cb(ROBOT_SET_VW_FZ_RESULT, (void *)(data + sizeof(len)));
            break;
        
        case MSG_TYPE_VIRTUAL_INFO:
            m_cbs->sys_cb(ROBOT_SYS_MSG_VIRTUAL_WALL, (void *)(data + sizeof(len)));
            break;
       case MSG_ROOMS_SEGMENT:     
            m_cbs->sys_cb(ROBOT_ROOMS_SEGMENT,(void *)data);
           break;
               
        case MSG_ROOMS_SET_ROOMS :
            printf("recv_packet MSG_ROOMS_SET_ROOMS\n"); 
             m_cbs->sys_cb(ROBOT_SET_ROOMS,(void *)data);
            break;
		case MSG_ROOMS_GET_ROOMS:
            printf("recv_packet MSG_ROOMS_GET_ROOMS\n"); 
			m_cbs->sys_cb(ROBOT_SYS_MSG_GET_ROOMS,(void *)data);
            break;

        case MSG_ROOMS_GET_WORKING_ROOMS:
             printf("recv_packet MSG_ROOMS_GET_WORKING_ROOMS\n"); 
	 		 m_cbs->sys_cb(ROBOT_SYS_MSG_GET_ROOMS,(void *)data);
             break;

        case MSG_ROOMS_GET_EDITING_ROOMS:
             printf("recv_packet MSG_ROOMS_GET_EDITING_ROOMS\n"); 
	 		 m_cbs->sys_cb(ROBOT_SYS_MSG_GET_EDIT_ROOMS,(void *)data);
             break;      

        case MSG_ROOMS_SPLIT:
             m_cbs->sys_cb(ROBOT_ROOMS_SPLIT,(void *)data);
             break;
        
        case MSG_ROOMS_MERGE:                   
             m_cbs->sys_cb(ROBOT_ROOMS_MERGER,(void *)data);
             break;
        
        case MSG_ROOMS_MAP_IS_STABLE:            
              m_cbs->sys_cb(ROBOT_MAP_IS_STABLE,(void *)data);
              break;
           
        case MSG_ROOMS_IS_MANUAL:         
             m_cbs->sys_cb(ROBOT_ROOMS_IS_MANUAL,(void *)data);
             break;
        
       case MSG_ROOMS_CLEAR_ROOMS:             
             m_cbs->sys_cb(ROBOT_CLEAR_ROOMS,(void *)data);
             break;
        
        case MSG_TYPE_CURRENT_MAP_ID:           // 当前地图ID
            printf("MSG_TYPE_CURRENT_MAP_ID\n");
            printf("MSG_TYPE_CURRENT_MAP_ID = %d\n",*(uint32_t *)(data + sizeof(len)));
            m_cbs->sys_cb(ROBOT_CURRENT_MAP_ID,(void *)(data + sizeof(len)));
           // wifi_app_msg_send_current_map_id(len, data + sizeof(len));
            break;

        case MSG_TYPE_CURRENT_MAP:              // 当前地图ID+文件
            
            printf("MSG_TYPE_CURRENT_MAP\n");
            strncpy(map_file, data + 8, sizeof(map_file));
            printf("MSG_TYPE_CURRENT_MAP_ID = %d,file = %s\n",*(uint32_t *)(data + sizeof(len)),map_file);
             
            m_cbs->sys_cb(ROBOT_CURRENT_MAP,(void *)(data + sizeof(len)));
            break;

        case MSG_TYPE_SET_CURRENT_MAP_RESULT:   // 设置当前地图结果
            m_cbs->sys_cb(ROBOT_CURRENT_MAP_RESULT,(void *)(data + sizeof(len)));
            break;
        
        case MSG_ROOMS_SET_CLEAN_ROOMS:
            printf("MSG_ROOMS_SET_CLEAN_ROOMS\n");
            m_cbs->sys_cb(ROROT_SET_CLEAN_ROOMS,(void *)(data + sizeof(len)));
            break;

        case MSG_ROOMS_MAP_IS_EDITABLE:   // 设置当前可编辑房间信息结果
            m_cbs->sys_cb(ROBOT_ROOMS_MAP_IS_EDITABLE,(void *)(data + sizeof(len)));
            break;

        case MSG_GET_LOG:
            m_cbs->sys_cb(ROBOT_MSG_GET_LOG,data);
            break;

        case MSG_ROOMS_GET_CURRENT_ROOM:   // 获取当前房间号
            m_cbs->sys_cb(ROBOT_ROOMS_GET_CURRENT_ROOM,(void *)(data + sizeof(len)));
            break;
            
        default:
            break;
    }

    return 0;
}

// 接收地图和路径数据
int recv_data(uint32_t type, uint32_t size, uint8_t *data) {
    ROBOT_SYS_MSG_E msg = ROBOT_SYS_MSG_INVALIDE;

    switch (type)
    {
        case TYPE_MAP:
            msg = ROBOT_SYS_MSG_MAP;
            break;
        case TYPE_POSITION:
            msg = ROBOT_SYS_MSG_TRAJECTORY;
            break;
        case TYPE_PATH:
            msg = ROBOT_SYS_MSG_NAVIGATION;
            break;
 //       case TYPE_CLEAN_LINE:
 //           msg = ROBOT_SYS_MSG_CLEAN_LINE;
 //           break;
         case TYPE_TRAJECTORY_PATH:
             msg = ROBOT_TRAJECTORY_PATH;
             break;
        default:
            return -1;
    }
    m_cbs->sys_cb(msg, (void *)data);
}

ROBOT_RETCODE_E robot_api_init(const ROBOT_CB_FUNS *cbs) {
    //注册callback函数
    m_cbs = cbs;
    //创建消息处理线程
    create_process_threads();
}

ROBOT_RETCODE_E robot_api_control(ROBOT_CMD_E cmd, void *param) {
    uint8_t cmd_buf[2];
    uint8_t *pbuf = cmd_buf;
    int shared_mem_msg_type = 0, size = 2;
    printf("robot cmd %s\n", cmdStr[cmd]);
    switch(cmd) {
    case ROBOT_CMD_AUTO_CLEAN:
        cmd_buf[0] = KEY_CTRL_CLEAN_MODE;
        cmd_buf[1] = CTRL_AUTO_CLEANING;
        break;
    case ROBOT_CMD_DOCK:
        cmd_buf[0] = KEY_CTRL_CLEAN_MODE;
        cmd_buf[1] = CTRL_GOING_BACK;
        break;
    case ROBOT_CMD_PAUSE:
        cmd_buf[0] = KEY_CTRL_START_PAUSE;
        cmd_buf[1] = CTRL_PAUSE;
        break;

    case ROBOT_CMD_GOTO_POS: {
        size = 1 + sizeof(DATA_GOTO_POSITION);
        pbuf = (uint8_t *)malloc(size);
        pbuf[0] = KEY_CTRL_GOTO_POSITION;
        memcpy(pbuf +1, param, sizeof(DATA_GOTO_POSITION));
        //memset(pbuf +1 + sizeof(DATA_POSITION), 0, sizeof(int));
       // *(int *)(pbuf + 1 + sizeof(DATA_POSITION)) = 2;
       // printf(" goto pos type = %d\n", *(int *)(pbuf + 1 + sizeof(DATA_POSITION)));
        break;
    }
    case ROBOT_CMD_SPOT_CLEAN:
        cmd_buf[0] = KEY_CTRL_CLEAN_MODE;
        cmd_buf[1] = CTRL_LOCAL_CLEANING;
        //cmd_buf[2] = (uint8_t)param;
        //size = 3;
        break;
    case ROBOT_CMD_RECT_CLEAN_START: {
        DATA_ZONE *zone = (DATA_ZONE *)param;
        size = sizeof(size) + sizeof(DATA_ZONE) + zone->count * sizeof(DATA_RECT);
        pbuf = (uint8_t *)malloc(size);
        memcpy(pbuf, &size, sizeof(size));
        memcpy(pbuf + sizeof(size), zone, size - sizeof(size));
#ifdef RECT_DEEP_CLEAN
        shared_mem_msg_type = MSG_TYPE_SET_ZONE_DEEP;
#else
        shared_mem_msg_type = MSG_TYPE_SET_ZONE;
#endif
        break;
    }
    case ROBOT_CMD_RECT_CLEAN_RESUME:
        cmd_buf[0] = KEY_CTRL_CLEAN_MODE;
        cmd_buf[1] = CTRL_ZONE_CLEAN;
        break;
    case ROBOT_CMD_WALLFOLLOW:
        cmd_buf[0] = KEY_CTRL_CLEAN_MODE;
        cmd_buf[1] = CTRL_EDGE_CLEANING;
        break;
        
    case ROBOT_CMD_SET_VWALL: {//设置虚拟墙
        DATA_CLASS_WALL *wall = (DATA_CLASS_WALL *)param;
        uint32_t count = wall->count;
        size = sizeof(size) + sizeof(count) + count * sizeof(DATA_VIRTUAL);
        pbuf = (uint8_t *)malloc(size);    
        memcpy(pbuf, &size, sizeof(size));
        wall = (DATA_CLASS_WALL *)(pbuf + sizeof(size));
        memcpy(wall, param, size - sizeof(size));    
        shared_mem_msg_type = MSG_TYPE_VIRTUAL_WALL;  
        break;
    }
    case ROBOT_CMD_SET_FORBIDDEN: {//设置禁区
        DATA_ZONE_CLASS_EX *fzone = (DATA_ZONE_CLASS_EX*)param;
        uint32_t count = fzone->count;
        size = sizeof(size) + sizeof(count) + count * sizeof(DATA_FOBIDEEN);
        pbuf = (uint8_t *)malloc(size);
        memcpy(pbuf, &size, sizeof(size));
        fzone = (DATA_ZONE_CLASS_EX *)(pbuf + sizeof(size));

        memcpy(fzone, param, size - sizeof(size));
        shared_mem_msg_type = MSG_TYPE_FORBIDDEN_ZONE;
        break;
    }
    case ROBOT_CMD_FAN_MODE:
        cmd_buf[0] = KEY_CTRL_FAN_MODE;
        cmd_buf[1] = (uint8_t)(long)param;
        break;
    case ROBOT_CMD_WATER_LEVEL:
        cmd_buf[0] = KEY_CTRL_WATER_LEVEL;
        cmd_buf[1] = (uint8_t)(long)param;
        break;
    case ROBOT_CMD_MANUAL_CONTROL:
        cmd_buf[0] = KEY_CTRL_MANUAL_CONTROL;
        cmd_buf[1] = (uint8_t)(long)param;
        break;
    case ROBOT_CMD_APPOINTMENT_ON://预约时间由手机APP管理，收到指令开始清扫
		printf("\033[1m\033[40;33m APP-->ROBOT_CMD_APPOINTMENT_ON \033[0m\n");
		#if 0
        cmd_buf[0] = KEY_CTRL_CLEAN_MODE;
        cmd_buf[1] = CTRL_APPOINTMENT_ON;
		#else
        cmd_buf[0] = KEY_CTRL_CLEAN_MODE;
        cmd_buf[1] = CTRL_AUTO_CLEANING;
		#endif
        break;  
    case ROBOT_CMD_APPOINTMENT_OFF:
		printf("\033[1m\033[40;33m APP-->ROBOT_CMD_APPOINTMENT_OFF \033[0m\n");
		#if 0
        cmd_buf[0] = KEY_CTRL_CLEAN_MODE;
        cmd_buf[1] = CTRL_APPOINTMENT_OFF;
		#else
		cmd_buf[0] = KEY_CTRL_START_PAUSE;
		cmd_buf[1] = CTRL_PAUSE;
		#endif
        break;
    case ROBOT_CMD_TEST_FULLGO:
        cmd_buf[0] = KEY_CTRL_CLEAN_MODE;
        cmd_buf[1] = CTRL_FULL_GO;
        break;
    case ROBOT_CMD_RESET_MAP:
        cmd_buf[0] = KEY_CTRL_CLEAR_MAP;
        cmd_buf[1] = 0x01;        // switch to open
        break;
    case ROBOT_CMD_NOT_DISTURB_SW:
        cmd_buf[0] = KEY_CTRL_NOT_DISTURB_SW;
        cmd_buf[1] = (uint8_t)(long)param;        // switch to open
        break;
    case ROBOT_CMD_CALLING_ROBOT_SW:
        cmd_buf[0] = KEY_CTRL_CALLING_ROBOT_SW;
        cmd_buf[1] = (uint8_t)(long)param;        // switch to open
        break;
    case ROBOT_CMD_CONTINUE_CLEAN:
        cmd_buf[0] = KEY_CTRL_CONTINUE_CLEAN_SW;
        cmd_buf[1] = (uint8_t)(long)param;
        break;

   case ROBOT_CMD_CHOICE_ROOMS_CLEAN:    
        cmd_buf[0] = KEY_CTRL_CLEAN_MODE;
        cmd_buf[1] = CTRL_NEW_CONTINUE_ROOMS_CLEAN;
        break;

    case ROBOT_CMD_CHOICE_ROOMS_CLEAN_RESUME:    
        cmd_buf[0] = KEY_CTRL_CLEAN_MODE;
        cmd_buf[1] = CTRL_CONTINUE_ROOMS_CLEAN;
        break;

    case ROBOT_CMD_Y_MOP_SW:
        cmd_buf[0] = KEY_CTRL_Y_SHAPE_MOP_SW;
        cmd_buf[1] = (uint8_t)(long)param;
        break;

    case ROBOT_CMD_RESTORE_FACTORY_SETTINGS:
        cmd_buf[0] = KEY_CTRL_RESTORE_FACTORY;
        cmd_buf[1] = 0x01;
        break;
		
	case ROBOT_EDGE_BRUSH_LIFE:
        cmd_buf[0] = KEY_CTRL_EDGE_BRUSH_LIFE;
        cmd_buf[1] = (uint8_t)(long)param;        // switch to open
        break; 
	case ROBOT_ROLL_BRUSH_LIFE:
        cmd_buf[0] = KEY_CTRL_ROLL_BRUSH_LIFE;
        cmd_buf[1] = (uint8_t)(long)param;       // switch to open
        break; 
	case ROBOT_FILTER_LIFE:
        cmd_buf[0] = KEY_CTRL_FILTER_LIFE;
        cmd_buf[1] = (uint8_t)(long)param;        // switch to open
        break; 
	case ROBOT_MOP_LIFE:
        cmd_buf[0] = KEY_CTRL_MOP_LIFE;
        cmd_buf[1] = (uint8_t)(long)param;        // switch to open
        break; 
        
    default:
        //printf("stm cmd %d not exist\n", cmd);
        return ROBOT_RETCODE_FAILED;
    }

    if(shared_mem_msg_type == 0) {
#define   CMD_CONTROL     0x02
        //发送到380的控制命令
        robot_api_send_packet(CMD_CONTROL, pbuf, size);
    } else {
        //?通过共享内存控制
        send_msg_to_hoslam_app(shared_mem_msg_type, pbuf, size);
    }
    if(pbuf != cmd_buf) {
        free(pbuf);
    }
    return ROBOT_RETCODE_SUCCESS;
}

ROBOT_RETCODE_E robot_api_send_packet(uint8_t cmd, uint8_t *databuf, uint8_t datalen) {
    uint8_t send_buffer[128];
    uint32_t len = 4;

    if((datalen + 8) > sizeof(send_buffer)) {
        printf("robot_api_send_packet large packet %d\n", datalen +8);
        return ROBOT_RETCODE_FAILED;
    }
    // 此部分数据需要传递给380解析
    send_buffer[len++]  = WIFI_APP_DATA_PROTOCOL_HEAD;
    send_buffer[len++]  = datalen + 1;
    send_buffer[len++]  = cmd;
    memcpy(send_buffer + len, databuf, datalen);
    len += datalen;
    send_buffer[len++]  = cal_packet_checksum(send_buffer + 4);
    // 增加数据长度，用于580接收
    send_buffer[0]      = (uint8_t)(len - 4);
    send_buffer[1]      = (uint8_t)((len - 4) >> 8);
    send_buffer[2]      = (uint8_t)((len - 4) >> 16);
    send_buffer[3]      = (uint8_t)((len - 4) >> 24);
#if 0
    // 打印数据包
    printf("send_packet: ");
    for (int i = 0; i < datalen; i++)
    {
        printf("%02x ", data_buf[i]);
    }
    printf("\n\n");
#endif
    // 发送数据包
    send_msg_to_hoslam_app(MSG_TYPE_REMOTE_CTRL, (const char*)send_buffer, len);

    return ROBOT_RETCODE_SUCCESS;
}

ROBOT_RETCODE_E robot_api_hoslam_enter_sleep(void) {
    send_msg_to_hoslam_app(MSG_TYPE_ENTER_SLEEP, NULL, 0);
    return ROBOT_RETCODE_SUCCESS;
}

ROBOT_RETCODE_E robot_api_fw_upgrade(const char *fw_file, const char *other_file) {
    wifi_app_firmware_upgrade(fw_file, other_file);
    return ROBOT_RETCODE_SUCCESS;
}

ROBOT_RETCODE_E robot_api_request_virtual_wall(void) {
    send_msg_to_hoslam_app(MSG_TYPE_GET_VIRTUAL_WALL, NULL, 0);
    return ROBOT_RETCODE_SUCCESS;
}

ROBOT_RETCODE_E robot_api_play_voice(uint32_t index) {
    //wifi_app_voice_play(index);
    wifi_app_voice_play_new( index, 1, 0);
    return ROBOT_RETCODE_SUCCESS;
}

ROBOT_RETCODE_E robot_api_send_msg_to_hoslam(int type, const char *data, const int len)
{
    send_msg_to_hoslam_app(type, data, len);
}

ROBOT_RETCODE_E robot_api_get_robot_state(int *state)
{
    *state = get_working_rooms_flag();
}


//==========内部使用函数，头文件是<robot_api_inner.h>========================
void inform_voice_play_complete(void) {
    m_cbs->sys_cb(ROBOT_SYS_PLAY_VOICE_END, NULL);
}

void inform_upgrade_progress(UPGRADE_STATUS *pstatus) {
    m_cbs->sys_cb(ROBOT_SYS_MSG_UPDATE_STATUS, (void *)pstatus);
}

void inform_upgrade_state(uint32_t state) {
    UPGRADE_STATUS status;
    
    status.upgrade_type = UPGRADE_TYPE_ROBOT_MSG;
    status.upgrade_status = state;
    //status.upgrade_progress = 0;
    m_cbs->sys_cb(ROBOT_SYS_MSG_UPDATE_STATUS, (void *)&status);
}
