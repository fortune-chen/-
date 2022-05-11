#ifndef __TUYA_IOT_MSG_PARSE_H__
#define __TUYA_IOT_MSG_PARSE_H__

#include "tuya_cloud_com_defs.h"
#include "robot_control_adapter.h"
#include "robot_api.h"
//#include"robot_api_inner.h"
//#include"wifi_app_server.h"
#include "tuya_dp_enum.h"


int send_goto_pos_flag;

#define WIFI_APP_DATA_PROTOCOL_PACKET_BUF   512
#define MAP_FILE_PATH_MAX_LEN 128

typedef struct
{
    int     type;
    int     (*parser)(void *msg);

} MSG_PARSER;

enum{
    CUR_ROOMS                       = 0,
    EDIT_ROOMS,
    SEGMENT,
    CLEAR_ROOMS,
    SAVE_ROOMS,
};

enum{
    MAP_SET_FAILED                =0,
    MAP_SAVE_FAILED,
    MAP_SET_SUCCESS,

};

enum{
    GET_VOICE_FAILED                =0, /* 下载失败 */
    GET_VOICE_LOADING,                  /* 安装中 */
    GET_VOICE_SUCCESS,                  /* 安装成功 */
    VOICE_USEING,                       /* 使用中 */
};
#pragma pack(1)

typedef struct
{
    uint8_t head;
    uint8_t version;
    uint8_t len;
    uint8_t cmd;
    uint8_t data[0];
}Raw_app_data_t;

#pragma pack(0)

#pragma pack(1)

typedef struct
{
    uint8_t head;
    uint8_t version;
    uint32_t len;
    uint8_t cmd;
    uint8_t data[0];
}Map_app_data_t;

#pragma pack(0)

 typedef struct 
 {	 
	 uint8_t md5[64];
	 uint8_t voice_edition_id[20];
     uint32_t url_len;
     uint8_t url[1024];
 }url_md5_voiceID; 

#define RAW_APP_SEND_HEAD 0xAA   /* 透传指令协议头帧 */
#define RAW_APP_SEND_VERSION 0x00   /* 透传指令协议版本 */

int wifi_app_reset_robot_map_data(void);

int wifi_app_msg_parse(const TY_OBJ_DP_S *msg);

int wifi_app_raw_parse(const TY_RECV_RAW_DP_S *msg);

int wifi_app_raw_exectue(const unsigned char *msg);

void *thread_check_app_need_map(void *arg);
int get_force_update_state(void);
void set_force_update_state(int state);
int wifi_app_msg_dp_parser_get_all_config(void *msg);
int wifi_app_msg_dp_send_rooms_msg_ret(ROOM_CONTROL_STATE dp_enum);
int wifi_app_msg_dp_send_rooms_msg_ret_v3(ROOM_CONTROL_STATE dp_enum);
int wifi_app_msg_dp_send_set_map_rusult_ret(uint8_t *data);
int wifi_app_msg_dp_send_vwall_foridden_ret(uint8_t *data);
int wifi_app_msg_dp_parser_segment(void *msg);

int get_virtual_wall(BYTE_T **databuf);
int get_forbidden_zone(BYTE_T **databuf);
int wifi_app_msg_dp_send_current_map_updated(void);

int save_map_id(uint8_t *data);
void set_cur_scene(STM_SCENE_E scene);
STM_SCENE_E get_cur_scene(void);
int get_force_update_state(void);
void tuya_iot_robot_state_feedback(DP_ROBOT_STATE dp_state);
void set_force_update_state(int state);
int wifi_app_reset_robot_map_data();
int send_voice_loading_ret(uint8_t * language_id,uint8_t status,uint8_t schedu);
int send_map_management(uint8_t send_cmd,uint8_t ret);
int wifi_app_msg_dp_send_set_choice_rooms_ret(uint8_t *data);
void wifi_app_reset_map();
void wifi_app_reset_path();
int send_app_data_by_raw(uint8_t raw_cmd, uint8_t *data,uint8_t len);
int send_app_data_by_map(uint8_t raw_cmd, uint8_t *data,uint32_t len);
int send_hoslam_split(int extersion_lenth);
void report_dnd_mode_time(uint8_t time_zone,uint8_t start_h,uint8_t start_m,uint8_t end_h,uint8_t end_m);








int wifi_sent_all_state(void);


STM_SCENE_E get_cur_scene(void);
//上报的dp状态，用来修正一些场景切换问题
void tuya_iot_robot_state_feedback(DP_ROBOT_STATE dp_state);
BOOL_T get_change_mode_flag(void);
int wifi_app_reset_clean_data(STM_CLEAN_ARG arg);

bool get_reset_map_flag(void);
bool set_reset_map_flag(bool flag);

#endif
