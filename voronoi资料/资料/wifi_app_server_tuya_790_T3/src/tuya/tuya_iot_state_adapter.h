#ifndef __TUYA_IOT_STATE_ADAPTER_H__
#define __TUYA_IOT_STATE_ADAPTER_H__

#include <stdint.h>
#include <stdbool.h>
#include "tuya_dp_enum.h"
#include "tuya_cloud_wifi_defs.h"
#include "tuya_cloud_com_defs.h"
#include "robot_app_msg.h"

typedef struct {
    DP_ROBOT_STATE  robot_state;            // 工作状态
    DP_WORK_MODE    work_mode;              // 清扫模式
    int32_t         error_no;               // 故障信息

} DATA_ROBOT_STATE;

typedef struct {
    DP_FAN_MODE     fan_mode;               // 风机模式
    DP_WATER_LEVEL  water_level;            // 水箱档位
} DATA_CONTROL_VALVE_STATE;


typedef struct {
    int32_t         battery_level;          // 剩余电量
    int32_t         clean_area;             // 清扫面积 单位：平方米
    int32_t         clean_time;             // 清扫时间 单位：秒
    int32_t         y_mod;                  //Y字型拖地
    int32_t         dnd_mode;               //勿扰开关
    int32_t         continue_clean;         //断点续扫开关

} DATA_CLEAN_STATIS;

typedef struct
{
	uint32_t         edge_brush_life;		//边刷剩余寿命 
	uint32_t         roll_brush_life;		//滚刷剩余寿命 
	uint32_t         filter_life;			//滤芯剩余寿命 
	uint32_t         mop_life;				//拖布剩余寿命 
}DATA_CONSUMABLE_STATIS;

//收到机器状态，映射到涂鸦定义的状态
void iot_robot_state_received(ROBOT_STATE_INFO *pInfo);
//获得扫地机历史状态
DATA_CLEAN_STATIS *iot_get_last_clean_statis(void);
DATA_CLEAN_STATIS *iot_get_clean_remember_statis(void);
// record 互斥初始
int init_map_record_lock(void);
void send_history_record_mark(void);
void send_history_record_unmark(void);
//涂鸦云删除当前配网设置，重新配网调用该接口
void iot_start_config_network(void);
//涂鸦云状态通知接口
void iot_state_change(IN CONST GW_WIFI_NW_STAT_E stat);
// 获取涂鸦 wifi 配网模式(smart 或兼容)
void iot_wifi_mode_get(void);
//是否连接上云端
bool iot_state_cloud_ready(void);
//等待机器人准备好，wifi驱动加载完成，获取固件版本号
char *iot_wait_robot_ready(void);
//升级文件下载完成SDK调用该接口
void iot_upgrade_download_complete(const char *filepath, DEV_TYPE_T dev_type);
void iot_upgrade_progress(int percent);
//状态上报涂鸦云端接口
int iot_report_robot_state(DATA_ROBOT_STATE *robot_sta);
int iot_report_clean_statis(DATA_CLEAN_STATIS *clean_sta);
int iot_report_init_state(void);
int iot_report_consumable_statis(DATA_CONSUMABLE_STATIS *consumable_sta);

//机器人地图重置时通知接口
void iot_robot_map_reset(void);
// 测试网络状况定时器
int init_soft_timer(void);
void get_history_record_map(void);
int init_history_record_lock(void);
int iot_report_valve_statis(DATA_CONTROL_VALVE_STATE *valve_sta);
int iot_report_robot_all_state(void);
void reset_last_data_robot_sta(void);
bool iot_state_cloud_ready(void);
int iot_report_robot_all_state(void);
void iot_report_mode_switch(void);
bool get_custom_mode_switch(void);

#endif /* __TUYA_IOT_STATE_ADAPTER_H__ */
