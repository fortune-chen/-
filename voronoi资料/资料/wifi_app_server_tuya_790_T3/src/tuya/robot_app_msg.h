#ifndef __ROBOT_APP_MSG_H__
#define __ROBOT_APP_MSG_H__

#include <stdint.h>
#include <stdbool.h>
#include "robot_api.h"
#include "uni_time.h"

//是否支持 智能配网模式
#define WIFI_SMART_MODE

#define FILE_VOICE_VERSION                  "/root/voice/version"
#define FILE_VOICE_UUID                     "/root/voice/uuid"

#define WIFI_SCAN_SH                        "wifi_scan"

#define SHELL_STARTUP_POWER                 "null"  //"/root/etc/init.d/S10power"

#define SHELL_STARTUP_HOSLAM                "null"  //"/root/etc/init.d/S51hoslam"
#define SHELL_STARTUP_WIFI                  "service wifi"  //"/root/etc/init.d/S60wifi"
#define SHELL_STARTUP_NETWORK               "service network"  //"/root/etc/init.d/S61network"
#define SHELL_STARTUP_DAEMON                "null"  //"/root/etc/init.d/S99daemon"
// 机器人主状态信息
enum {
    /* stop state */
    ROBOT_STATE_INIT            = 0,
    ROBOT_STATE_IDLE            = 1,
    ROBOT_STATE_PAUSED          = 2,
    ROBOT_STATE_ERROR           = 3,
    ROBOT_STATE_SLEEP           = 4,
    /* cleaning state */
    ROBOT_STATE_WALLFOLLOW      = 10,
    ROBOT_STATE_CLEAN_ROOM      = 11,
    ROBOT_STATE_CLEAN_AUTO      = 12,
    ROBOT_STATE_CLEAN_RECTS     = 13,
    ROBOT_STATE_CLEAN_SPOT      = 14,
    ROBOT_STATE_CHOICE_CLEAN    = 15,
    /* moving state */
    ROBOT_STATE_GOTO_POS        = 20,
    ROBOT_STATE_REMOTE_DRIVE    = 21,
    ROBOT_STATE_DOCK            = 22,
    /* charge state */
    ROBOT_STATE_SEAT_CHARGING   = 30,
    ROBOT_STATE_DIRECT_CHARGING = 31,
    ROBOT_STATE_CHARGING_ERROR  = 32,
    /* test state */
    ROBOT_STATE_TESTING         = 40,
};

enum
{
    CMD_CHEACK_BASE_INFO            = 0x01, // 查询基础信息
    CMD_CONTROL                     = 0x02, // 控制指令下发
    CMD_POSITION                    = 0x03, // 地图指令 - 当前点
    CMD_PRODUCT_INFO                = 0x04, // 产品信息查询
    CMD_WIFI_STATE                  = 0x05, // WIFI网络状态查询
    CMD_GET_ROBOT_STATE             = 0x06, // 云平台状态查询,即查询扫地机当前的状态
    CMD_CONFIG_NETWORK              = 0x07, // 配网命令
    CMD_UART_RESET    	            = 0x08, // UART初始化
    CMD_FACTORY_TEST                = 0x09, // 厂测命令
    CMD_PLAY_VOICE                  = 0x0A, // 播放语音
    CMD_UPGRADE_STATUS              = 0x0B, // 升级状态
    CMD_GET_SYS_APPSERVER_TYPE      = 0x0C, // 获取appserver类型以及系统版本
    CMD_VOICE_VERSION               = 0x21, // 语音版本
    CMD_VOICE_GAIN                  = 0x22, // 语音增益设置
    CMD_WIFI_TEST                   = 0x23, // WiFi测试
    CMD_EXTEND                      = 0x25, //高级
    CMD_DND_MODE_TIME               = 0x26, //勿扰时间段设置 
    CMD_DEBUG                       = 0xFF, // 调试专用指令
};

// 协议中所需要上报的错误状态码
enum {
    ERROR_INFO_NO_ERROR              = 0x00, // 预留
    ERROR_INFO_LEFT_WHEEL_HANGED,           // 左轮悬空
    ERROR_INFO_MAIN_BRUSH_ENTWINED,         // 主刷卡住
    ERROR_INFO_RIGHT_WHEEL_HANGED,          // 右轮悬空
    ERROR_INFO_LEFT_WHEEL_ENTWINED,         // 左轮卡住
    ERROR_INFO_RIGHT_WHEEL_ENTWINED,        // 右轮卡住
    ERROR_INFO_ALWAYS_CLIFFED,              // 一直检测到悬崖
    ERROR_INFO_SIDE_BRUSH_ENTWINED,         // 边刷卡住
    ERROR_INFO_CODEPANEL_ERROR,
    ERROR_INFO_BUMP_ERROR,                  // 碰撞异常
    ERROR_INFO_DUSTBIN_UNINSERT,            // 尘盒未放
    ERROR_INFO_STASIS_STALL,
    ERROR_INFO_CLIFF_ERROR,                 // 地检异常
    ERROR_INFO_13_RESERVE,
    ERROR_INFO_14_RESERVE,
    ERROR_INFO_VACUUM_MOTO_ENTWINED,        // 真空电机卡住
    ERROR_INFO_BIG_CURRENT,                 // 电流过大
    ERROR_INFO_SWITCH_OFF_CHARGING,
    ERROR_INFO_18_RESERVE,
    ERROR_INFO_STUCK,                       // 机器被卡住
    ERROR_INFO_TRAP,                        // 机器被困住
    ERROR_INFO_LASER_BUMP_ERROR,            // 激光头上盖被卡住
    ERROR_INFO_LASER_DATA_ERROR,            // 激光头被遮挡或卡住
    ERROR_INFO_TOF_DATA_ERROR,              // 沿墙传感器被遮挡
    ERROR_INFO_VIRTUAL_WALL_ERROR,          // 虚拟墙或禁区设置错误
    ERROR_INFO_MAX,

    ERROR_HINTS_RELOCATION_FAILED    = 80,
    ERROR_HINTS_UNREACHABLE          = 81,
};

typedef struct {
    int32_t         robot_state;            // 机器状态
    int32_t         fan_mode;               // 风机模式
    int32_t         water_level;            // 水箱调档
    int32_t         battery_level;          // 剩余电量
    int32_t         continue_clean;         // 断点续扫开关
    int32_t         y_type_mop;             // Y字型拖地
    int32_t         dnd_mode;               // 勿扰模式
    int32_t         obstacles_cross;        // 越障功能
    int32_t         cliff_detect;           // 地检开关
    int32_t         clean_area;             // 清扫面积 单位：平方米
    int32_t         clean_time;             // 清扫时间 单位：分钟
    int32_t         error_info;             // 故障信息
    int32_t         sleep_mode;             // 休眠模式
    char            version[40];            // 固件版本
    
    uint32_t         edge_brush_life;        // 边刷剩余寿命 
    uint32_t         roll_brush_life;        // 滚刷剩余寿命 
    uint32_t         filter_life;            // 滤芯剩余寿命 
    uint32_t         mop_life;            	// 拖布剩余寿命

	int32_t         robot_reposition_state;//机器重定位状态
} ROBOT_STATE_INFO;

// 遥控器控制子命令
enum {
    CTRL_GO_FORWARD                 = 0x00, // 向前
    CTRL_GO_LEFT,                           // 向左
    CTRL_GO_RIGHT,                          // 向右
    CTRL_GO_BACKWARDS,                      // 向后
    CTRL_STOP_MOVING,                       // 停止
};

enum {
    CTRL_FAN_NORMAL                 = 0x00, // 正常风力
    CTRL_FAN_STRONG,                        // 强力风力
    CTRL_FAN_SILENT,                        // 安静模式

    CTRL_FAN_OFF                    = 0x0A, // 关闭风机
    CTRL_FAN_DISABLE                = 0x0B, // 禁用风机
    CTRL_FAN_NO_SET                 = 0xff, // 风机未设置
};

enum {
    CTRL_WATER_LEVEL_DEFAULT        = 0x00, // 正常档，水箱二档
    CTRL_WATER_LEVEL_HIGH,                  // 大水量，水箱三档
    CTRL_WATER_LEVEL_LOW,                   // 小水量，水箱一档

    CTRL_WATER_LEVEL_OFF            = 0x0A, // 关闭水箱
    CTRL_WATER_LEVEL_DISABLE        = 0x0B, // 禁用水箱
    CTRL_WATER_LEVEL_NO_SET         = 0xFF, // 水箱未设置
};


enum {
    STATE_FAN_NORMAL                 = 0x00, // 正常风力
    STATE_FAN_STRONG,                        // 强力风力
    STATE_FAN_SILENT,                        // 安静模式

    STATE_FAN_OFF                    = 0x0A, // 关闭风机
    STATE_FAN_DISABLE                = 0x0B, // 禁用风机
};

typedef enum {
    GRIT            = 0,    
    TUYA,                       
    MIJIA,                               
}APPSERVER_TYPE_ENUM;

enum {
    STATE_WATER_LEVEL_DEFAULT        = 0x00, // 正常档，水箱二档
    STATE_WATER_LEVEL_HIGH,                  // 大水量，水箱三档
    STATE_WATER_LEVEL_LOW,                   // 小水量，水箱一档

    STATE_WATER_LEVEL_OFF            = 0x0A, // 关闭水箱
    STATE_WATER_LEVEL_DISABLE        = 0x0B, // 禁用水箱
};

enum {
    WIFI_STATE_NONE,
    WIFI_STATE_WATIING_CONFIG,              // 配网中
    WIFI_STATE_CONFIG_FAILD,                // 配网失败
    WIFI_STATE_DISCONNECTED,                // 设备离线
    WIFI_STATE_CONNECTED,                   // 设备在线

    WIFI_STATE_FAULT,                       // WiFi故障
    WIFI_STATE_UUID_FAILD,                // 无效uuid故障
};

enum {
    WIFI_SMART_CONFIG_MODE          = 0x00, // 快速配网模式(普通配网模式)
    WIFI_AP_CONFIG_MODE             = 0x01, // 兼容配网模式
    WIFI_AP_CONFIG_DONE             = 0x02, // 配网模式已经结束
};

enum
{
    KEY_EXT_CLEAN_DONE              = 0x01, // 清扫完成标志
    KEY_EXT_REBOOT_FACTOY           = 0x02, //恢复出厂设置
    KEY_EXT_ROOM_SET                = 0x03, //按房清扫设置

};

enum {
    SLEEP_NONE                      = 0,    // 无休眠
    SLEEP_SHALLOW,                          // 浅休眠
    SLEEP_DEEP_WIFI_ON,                     // 深度休眠（WiFi正常工作）
    SLEEP_DEEP_WIFI_OFF,                    // 深度休眠（WiFi关闭）
};

extern DATA_PATH *save_navigation_path;

/* 返回机器人睡眠状态 */
uint32_t get_robot_sleep_state(void);
//提供给SDK的callback函数
void robot_app_packet_cb(uint8_t *data, uint32_t len);
bool robot_app_allowed_upgrade(void);
//扫地机的状态获取函数
bool robot_app_state_running(void);
bool robot_app_state_sleep(void);
char *robot_app_fw_version(void);
//和380 app相关的系统消息的处理
void robot_app_handle_sys_msg_wakeup(void);
void robot_app_handle_sys_msg_voice_end(void);
//通知消息发送到380 app
int inform_robot_wifi_state(int wifi_state);
int inform_robot_wifi_mode_state(int state);
int inform_robot_upgrade_state(int state);
int inform_robot_test_result(int result, int value);

void robot_set_wifi_config_state();
int  robot_get_wifi_config_state(void);

void set_uuid_fail_flag(int flag);
void init_wifi_module_mutex(void);
int robot_start_remote_control(int cmd);
void robot_app_handle_sys_msg_sleep(void);
int recv_set_clean_strategy(uint8_t *data);
uint8_t return_robot_state_by_380(void);
void sync_time_2_380(POSIX_TM_S *tm);
uint8_t return_robot_reposition_state_by_380(void);

#endif /* __ROBOT_APP_MSG_H__ */
