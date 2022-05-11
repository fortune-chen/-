#ifndef __WIFI_APP_DATA_HOSLAM__
#define __WIFI_APP_DATA_HOSLAM__

#include <stdint.h>
#include "robot_api.h"

//
// 380协议数据结构
//

//                          数据通信帧格式
//      协议头      包长度       命令             数据            校验和
//      Header      Length      Cmd             Data           Checksum
//      1 Byte      1 Byte     1 Byte       [0...n] Bytes       1 Byte
//       0xAA      Cmd + Data                                 Cmd + Data

#define WIFI_APP_DATA_PROTOCOL_HEAD         0xAA
#define WIFI_APP_DATA_PROTOCOL_PACKET_BUF   512

enum
{
    KEY_CTRL_MANUAL_CONTROL         = 0x21, // 方向控制
    KEY_CTRL_CLEAN_MODE             = 0x22, // 清扫模式
    KEY_CTRL_FAN_MODE               = 0x24, // 风力模式
    KEY_CTRL_WATER_LEVEL            = 0x25, // 水箱调节
    KEY_CTRL_START_PAUSE            = 0x26, // 开始/停止
    KEY_CTRL_CONTINUE_CLEAN_SW      = 0x27, // 断点续扫开关
    KEY_CTRL_Y_SHAPE_MOP_SW         = 0x28, // Y 字行拖地开关
    KEY_CTRL_CALLING_ROBOT_SW       = 0x29, // 呼叫机器人
    KEY_CTRL_NOT_DISTURB_SW         = 0x2A, // 勿扰模式开关
    KEY_CTRL_CLEAR_MAP              = 0x2B, // 清除当前地图和历史地图
    
    KEY_CTRL_SYNC_TIME_380 			= 0x31, //时间通知380

    KEY_CTRL_EDGE_BRUSH_LIFE 			= 0x32, //边刷寿命设置
    KEY_CTRL_ROLL_BRUSH_LIFE 			= 0x33, //滚刷寿命设置
    KEY_CTRL_FILTER_LIFE 				= 0x34, //滤网寿命设置
    KEY_CTRL_MOP_LIFE 					= 0x36, //拖布寿命设置
    
    // 自定义命令
    KEY_CTRL_GOTO_POSITION          = 0x20, // 指哪到哪

    KEY_CTRL_RESTORE_FACTORY        = 0x40, //恢复出厂设置
};

enum
{
    CTRL_EDGE_CLEANING              = 0x00, // 沿边清扫
    CTRL_LOCAL_CLEANING,                    // 局部清扫
    CTRL_AUTO_CLEANING,                     // 自动清扫
    CTRL_GOING_BACK,                        // 回充
    CTRL_ZONE_CLEAN,                        // 划区清扫
    
    CTRL_APPOINTMENT_ON             = 0x08, //预约开始清扫
    CTRL_APPOINTMENT_OFF            = 0x09, //预约停止清扫
    // For Demo
    CTRL_FULL_GO                    = 0x0A, // 老化测试
    CTRL_CALIBRATION                = 0x0B, // 校准模式
    CTRL_CONTINUE_ROOMS_CLEAN       = 0x0C,//继续选区清扫
    CTRL_NEW_CONTINUE_ROOMS_CLEAN   = 0x0D, //新的选区清扫(已取消)
    CTRL_IEC_MODE_CLEAN             = 0x12, // IEC模式清扫
};

enum
{
    CTRL_PAUSE                      = 0x00, // 停止
    CTRL_START,                             // 开始
};

//
// 580协议数据结构
//
// 消息类型 app_server -> robot
enum {
    MSG_TYPE_REMOTE_CTRL            = 0x01, // 控制指令，需要向380传递
    MSG_TYPE_REQUEST_SYSTEM_INFO    = 0x02, // APP请求机器状态
    MSG_TYPE_SET_ZONE               = 0x04, // 划区清扫
    MSG_TYPE_VIRTUAL_WALL           = 0x05, // 虚拟墙
    MSG_TYPE_FORBIDDEN_ZONE         = 0x06, // 禁区

    MSG_TYPE_ENTER_SLEEP            = 0x08, // 进入休眠
    MSG_TYPE_GET_VIRTUAL_WALL       = 0x10, // 初始状态获取虚拟墙信息指令
    MSG_GET_LOG_INFO                = 0x12, // 获取log信息
/* 客户自定义需求 */
    MSG_TYPE_SET_ZONE_DEEP          = 0x30, // 划区清扫的深度清扫
    MSG_TYPE_PATH_NOT_NAVI          = 0x31, // 不要导航路径
    MSG_TYPE_TYPE_ZOON_CLEAN        = 0x32, //划区清扫次数策略
    
    MSG_TYPE_TEST                   = 0xF0, // 测试命令
};

// 消息类型 robot -> app_server
enum {
    MSG_TYPE_UPDATE_APP_INFO        = 0x03, // WiFi数据上报
    MSG_TYPE_RESET_MAP              = 0x07, // 重置地图
    MSG_TYPE_WAKEUP                 = 0x09, // Hoslam通知唤醒
    MSG_TYPE_SET_VW_FZ_RESULT       = 0x0A, // 虚拟墙与禁区设置结果返回
    MSG_TYPE_SLEEP                  = 0x0B, // Hoslam通知休眠 //feng
    MSG_TYPE_VIRTUAL_INFO           = 0x11, // Hoslam返回虚拟墙信息
    MSG_SEND_LOG_INFO               = 0x13, // 580 主动发送 log 信息

    MSG_TYPE_CURRENT_MAP_ID         = 0x15, // 当前地图ID
    MSG_TYPE_GET_CURRENT_MAP        = 0x16, // 请求获取当前地图
    MSG_TYPE_CURRENT_MAP            = 0x17, // 当前地图
    MSG_TYPE_SET_CURRENT_MAP        = 0x18, // 请求设置当前地图
    MSG_TYPE_SET_CURRENT_MAP_RESULT = 0x19, // 设置当前地图结果

    MSG_ROOMS_SEGMENT               = 0x20, //主动分房间，需要地图稳定，并且机器不动 
    MSG_ROOMS_GET_ROOMS             = 0x21, //主动获取房间 
    //MSG_ROOMS_SET_ROOMS             = 0x22, //主动设置房间 
    MSG_ROOMS_SET_STRATEGY          = 0x23, //设置清扫模式，不再使用  
    
    MSG_ROOMS_SPLIT                 = 0x24, // 房间分割
    MSG_ROOMS_MERGE                 = 0x25, // 房间合并
    MSG_ROOMS_MAP_IS_STABLE         = 0x26, // 地图是否稳定 
    MSG_ROOMS_IS_MANUAL             = 0x27, // 房间是否主动设置的 
    MSG_ROOMS_CLEAR_ROOMS           = 0x28, // 清掉主动设置的房间信息 
    //feng
    MSG_ROOMS_GET_WORKING_ROOMS     = 0x2C,//0x29, // 获取工作时候的房间
    MSG_ROOMS_GET_EDITING_ROOMS     = 0x2D,//0x2A, // 获取编辑时的房间
    MSG_ROOMS_SET_ROOMS             = 0x2E,//0x22, //主动设置房间 
    MSG_ROOMS_GET_CURRENT_ROOM      = 0x2F,  // 获取当前房间
    MSG_ROOMS_MAP_IS_EDITABLE       = 0x2B, // 判断是否为可编辑房间标志

    MSG_GET_LOG                     = 0x35, //获取380log
    MSG_TOGGLE_LOG                  = 0x36, //发送380log开关

    MSG_ROOMS_SET_CLEAN_ROOMS       =0x40,   //设置选区

};

// 地图与路径数据
enum
{
    TYPE_MAP                        = 0,    // 全局地图
    TYPE_POSITION,                          // 当前点
    TYPE_CLEAN_LINE,                        // 规划清扫线
    TYPE_PATH,                              // 导航路径
    TYPE_TRAJECTORY_PATH,                   // 含有type的清扫的路径(包含当前点)
};

typedef struct
{
    uint32_t        type;           // 0:MAP 1:POSITION
    uint32_t        size;
    uint64_t        timestamp;      // in us

} DATA_HEADER;

void inform_voice_play_complete();
void inform_upgrade_progress(UPGRADE_STATUS *pstatus);
void inform_upgrade_state(uint32_t state);
#endif
