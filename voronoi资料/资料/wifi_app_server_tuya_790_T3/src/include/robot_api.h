#ifndef __ROBOT_API__
#define __ROBOT_API__

#include <stdint.h>
#include <stdbool.h>
#include "../../robot_api_src/wifi_app_server.h"
#include "../../robot_api_src/wifi_app_voice_mgr.h"
#define SHELL_STARTUP_AUDIO                 "" //"/root/etc/init.d/S00voice"
#define SHELL_AUDIO_ONESELF                 "/root/app_server/voice_config"
#define SHELL_VOICE_MAIN_DIR                "/root/voice"
extern char u_log_flag;
extern char tf_card;

#define LANG_PATH_MAX_LEN   128
#define ROOM_NAME_LEN       83 //83 + 1(名字长度) + 12(房间策略 = 96)

#define RECT_DEEP_CLEAN

typedef enum {
    ROBOT_CMD_NO_ACTION = 0,
    ROBOT_CMD_AUTO_CLEAN = 1,
    ROBOT_CMD_DOCK,
    ROBOT_CMD_PAUSE,
    ROBOT_CMD_GOTO_POS,
    ROBOT_CMD_SPOT_CLEAN,
    ROBOT_CMD_RECT_CLEAN_START,   //设置区域，并且重新开始划区清扫
    ROBOT_CMD_RECT_CLEAN_RESUME,  //暂停后，使用该命令继续划区清扫
    ROBOT_CMD_WALLFOLLOW,         //沿边清扫
    ROBOT_CMD_SET_VWALL,
    ROBOT_CMD_SET_FORBIDDEN,      //10
    ROBOT_CMD_FAN_MODE,
    ROBOT_CMD_WATER_LEVEL,
    ROBOT_CMD_MANUAL_CONTROL,
    ROBOT_CMD_APPOINTMENT_ON,       //预约开
    ROBOT_CMD_APPOINTMENT_OFF,      //预约关
    ROBOT_CMD_TEST_FULLGO,
    ROBOT_CMD_RESET_MAP,
    ROBOT_CMD_NOT_DISTURB_SW,
    ROBOT_CMD_CALLING_ROBOT_SW,
    ROBOT_CMD_CONTINUE_CLEAN,      //断点续扫20
    ROBOT_CMD_CHOICE_ROOMS_CLEAN,
    ROBOT_CMD_CHOICE_ROOMS_CLEAN_RESUME,//暂停后，使用该命令继续选区清扫
    ROBOT_CMD_Y_MOP_SW,
    ROBOT_CMD_RESTORE_FACTORY_SETTINGS,
    ROBOT_CMD_SYNC_TIME_380,
    ROBOT_EDGE_BRUSH_LIFE,
    ROBOT_ROLL_BRUSH_LIFE,
    ROBOT_FILTER_LIFE,
    ROBOT_MOP_LIFE,
} ROBOT_CMD_E;

typedef enum {
    ROBOT_SYS_MSG_INVALIDE = 0,
    ROBOT_SYS_MSG_MAP,
    ROBOT_SYS_MSG_TRAJECTORY,
    ROBOT_SYS_MSG_NAVIGATION,
    ROBOT_SYS_MSG_CLEAN_LINE,
    ROBOT_SYS_MSG_UPDATE_STATUS,
    ROBOT_SYS_MSG_MAP_RESET,
    ROBOT_SYS_MSG_VIRTUAL_WALL,
    ROBOT_SYS_MSG_WAKEUP,
    ROBOT_SYS_MSG_SLEEP,
    ROBOT_SYS_PLAY_VOICE_END,
    ROBOT_ROOMS_SEGMENT,
    ROBOT_SET_ROOMS,
    ROBOT_SYS_MSG_GET_ROOMS,
    ROBOT_SYS_MSG_GET_EDIT_ROOMS,
    ROBOT_ROOMS_SPLIT,
    ROBOT_ROOMS_MERGER,
    ROBOT_MAP_IS_STABLE,
    ROBOT_ROOMS_IS_MANUAL,
    ROBOT_CLEAR_ROOMS,
    ROBOT_TRAJECTORY_PATH,
    ROBOT_CURRENT_MAP_ID,
    ROBOT_CURRENT_MAP,
    ROBOT_CURRENT_MAP_RESULT,
    ROBOT_SET_VW_FZ_RESULT,
    ROROT_SET_CLEAN_ROOMS,
    ROBOT_ROOMS_MAP_IS_EDITABLE,
    ROBOT_ROOMS_GET_CURRENT_ROOM,
    ROBOT_MSG_GET_LOG,
} ROBOT_SYS_MSG_E;

typedef enum {
    ROBOT_RETCODE_SUCCESS   = 0,
    ROBOT_RETCODE_FAILED    = 1,

} ROBOT_RETCODE_E;

// 固件升级状态定义（用于通知云端）
enum {
    UPGRADE_NONE                    = 0,    // 无效状态
    UPGRADE_READY,                          // 准备升级
    UPGRADE_START,                          // 开始升级
    UPGRADE_GOING,                          // 升级中
    UPGRADE_COMPLETE,                       // 升级完成
    UPGRADE_FAILED,                         // 升级失败
};

// 固件升级状态定义(用于通知机器人)
enum {
    UPGRADE_STATE_NONE              = 0x00,
    UPGRADE_STATE_NOT_ALLOWED,              // 无法升级
    UPGRADE_STATE_GOING,                    // 升级中
    UPGRADE_STATE_COMPLETE,                 // 升级成功
    UPGRADE_STATE_FAILED,                   // 升级失败
    UPGRADE_STATE_DOWNLOADING,              // 下载中
    UPGRADE_STATE_DOWNLOADE_FAILED,         // 下载失败
};

// 固件升级类型
enum {
    UPGRADE_TYPE_INVALID            = 0,    // 无效类型

    UPGRADE_TYPE_AFW,                       // AFW固件
    UPGRADE_TYPE_HOSLAM,                    // Linux Hoslam App
    UPGRADE_TYPE_APP_SERVER,                // WiFi App Server
    UPGRADE_TYPE_GRIT_CLIENT,               // Grit App Client
    UPGRADE_TYPE_VOICE,                     // Voice
    UPGRADE_TYPE_ROBOT_MSG,                 // 通知扫地机的消息类型
    
    UPGRADE_TYPE_MAX_INDEX,
    UPGRADE_TYPE_DEFAULT            = 0xFF  // 默认类型
};
// 固件升级状态
typedef struct  {
    uint32_t        upgrade_type;
    uint32_t        upgrade_status;
    uint32_t        upgrade_progress;
} UPGRADE_STATUS;


typedef struct {
    uint32_t        w;
    uint32_t        h;
    float           ox;
    float           oy;
    float           dock_x;
    float           dock_y;
    float           resolution;     // 地图每个点代表的实际大小，单位：m
    uint8_t         buf[0];
} DATA_MAP;

typedef struct {
 uint32_t w;
 uint32_t h;
 float ox;
 float oy;
 float dock_x;
 float dock_y;
 float resolution;
 char magic[4]; // '2' 'b' 'i' 't' 
 int size; // size of payload
 uint8_t buf[0];
} DATA_MAP_V2;

typedef struct {
    float           x;
    float           y;

} DATA_POSITION;            //位置

typedef struct {
    float           x;
    float           y;
    int             type; //0:不进行局部清扫      ;  >=1 代表清扫次数
} DATA_GOTO_POSITION;       //到达指定位置


typedef struct {
   int points_num;
   DATA_POSITION spec[32];
} DATA_ROOMS;               //房间轮廓点


typedef struct {
    uint32_t        count;
    DATA_POSITION   position[0];

} DATA_PATH;

/* 当前点 + 带 type 的path */
typedef struct
{
    float           x;
    float           y;
    char            type;

} DATA_POSITION_TYPE;

typedef struct
{
    float           x;
    float           y;
    float           theta;
} DATA_CURRENT_POSE;

typedef struct
{
    uint32_t                path_point_count;   //count for position[0]
    DATA_CURRENT_POSE       cur_pose;
    DATA_POSITION_TYPE      position[0];

} DATA_CURR_POSE_AND_PATH;

///////////////////////////////


typedef struct {
    float           x0;
    float           y0;
    float           x1;
    float           y1;

} DATA_RECT;

typedef struct {
    uint32_t        count;
    DATA_RECT       rect[0];

} DATA_ZONE;

typedef struct{
    uint32_t        type;//预留，清扫类型
    uint32_t        clean_count;//清扫次数
    DATA_ZONE       data_zone;
}DATA_TYPE_ZONE;

typedef struct {
    float           x0;
    float           y0;
    float           x1;
    float           y1;

} DATA_LINE;

typedef struct {
    uint32_t        count;
    DATA_LINE       line[0];

} DATA_WALL;

typedef struct {
    float           x0;
    float           y0;
    float           x1;
    float           y1;
    float           x2;
    float           y2;
    float           x3;
    float           y3;
} DATA_RECT_EX;

typedef enum 
{
    FORBIDDEN_NONE,
    WET_FORBIDDEN,//拖地禁区
    DRY_FORBIDDEN,//扫地禁区
    FORBIDDEN_ALL,//扫拖禁区
}forbidden_type_t;

typedef struct
{
    forbidden_type_t forbid_type;
    DATA_RECT_EX    rect;
}DATA_FOBIDEEN;

typedef struct
{
    uint32_t        count;
    DATA_FOBIDEEN    fobidden[0];

} DATA_ZONE_CLASS_EX;

typedef struct {
    uint32_t        count;
    DATA_RECT_EX    rect[0];
} DATA_ZONE_EX;

typedef struct 
{
    forbidden_type_t forbid_type;
    DATA_LINE  line;
}DATA_VIRTUAL;

typedef struct
{
    uint32_t        count;
    DATA_VIRTUAL    virtual[0];
} DATA_CLASS_WALL;
#if 0
typedef struct {
 int id;    // 房间标识，由segment产生，或app产生，不变
 int count;          // 清扫次数   
 int order;          // 清扫顺序
 int forbidden;  // 是否禁止清扫
 int n_points;  // 多边形个数
 DATA_POSITION points[32]; // 多边形顶点
} ROOM_SPEC_T;
#else

#pragma pack(1)
typedef struct {
   uint8_t name_len;
   uint8_t name[ROOM_NAME_LEN];
} NAME_SPEC_T;
#pragma pack(0)

#pragma pack(1)
typedef struct {
    int32_t id;                   // 房间标识，由segment产生，或app产生，不变
    int32_t order;                // 清扫顺序
    uint16_t count;         // 清扫次数
    uint16_t mop_count;           // 拖地次数
    int8_t color;                 // 颜色序号,-1为不确定
    int8_t rev1[7];
    uint32_t forbidden : 1; // 禁止清扫
    uint32_t mop_forbidden   : 1; // 禁止拖地
    uint32_t rev2            : 30;
    int8_t bundle[12]; //4.1.2->32   // 具体策略 (依次是，风机挡位，水箱挡位，Y字形拖地开关，地检开关,清扫次数(缓存),该房间是否设置过标志位，房间名字的长度（1byte），房间名(19byte))
    NAME_SPEC_T name_pack;
    int n_points;                 // 多边形点数
    DATA_POSITION points[32];             // 多边形顶点
} ROOM_SPEC_T;
#pragma pack(0)
#endif
#pragma pack(1)
typedef struct {
    uint16_t x;
    uint16_t y;
}XY_POINT;
#pragma pack(0)

#pragma pack(1)
typedef struct {
    int16_t x;
    int16_t y;
}XYPoint_2;
#pragma pack()

#define TUYA_NAME_LEN 19

#pragma pack(1)
typedef struct { 
    uint16_t id;          /* 房间ID */
    uint16_t order;       /* 清扫顺序*/
    uint16_t clean_count; /*清扫次数*/
    uint16_t mop_count;   /*拖地次数*/
    int8_t   color_order; /* 颜色序号，保留,-1为不确定 */
    uint8_t clean_state;  /* 禁止清扫（1byte）(1:禁止，0：不禁止) */
    uint8_t mop_state;    /* 禁止拖地（1byte）(1:禁止，0：不禁止) */
    uint8_t vac_state;    /*风机档位*/
    uint8_t water_state;  /*水箱档位*/
    uint8_t Y_mop_state;  /* Y字形拖地状态 */
    //uint8_t cliff_state;/*地检开关*/
    uint8_t rev[12];
    uint8_t room_name_len;  
    uint8_t room_name[TUYA_NAME_LEN];
    uint8_t n_points;  // 多边形顶点个数
    XYPoint_2 points[0]; // 多边形顶点坐标
} SEND_ROOM_SPEC_T;
#pragma pack(0)
// 消息类型     tuya -> robot
enum {
    TUYA_MSG_TYPE_UPDATE_APP_INFO        = 0x03, // WiFi数据上报
    TUYA_MSG_TYPE_RESET_MAP              = 0x07, // 重置地图
    TUYA_MSG_TYPE_WAKEUP                 = 0x09, // Hoslam通知唤醒
    TUYA_MSG_TYPE_SET_VW_FZ_RESULT       = 0x0A,  //虚拟墙与禁区设置结果返回
    TUYA_MSG_TYPE_SLEEP                  = 0x0B, // Hoslam通知休眠 //feng
    TUYA_MSG_TYPE_GET_VIRTUAL_WALL       = 0x10, // 请求获取虚拟墙信息

    TUYA_MSG_TYPE_VIRTUAL_INFO           = 0x11, // Hoslam返回虚拟墙信息

    TUYA_MSG_TYPE_CURRENT_MAP_ID         = 0x15, // 当前地图ID
    TUYA_MSG_TYPE_GET_CURRENT_MAP        = 0x16, // 请求获取当前地图
    TUYA_MSG_TYPE_CURRENT_MAP            = 0x17, // 当前地图
    TUYA_MSG_TYPE_SET_CURRENT_MAP        = 0x18, // 请求设置当前地图
    TUYA_MSG_TYPE_SET_CURRENT_MAP_RESULT = 0x19, // 设置当前地图结果

    TUYA_MSG_ROOMS_SEGMENT               = 0x20, //主动分房间，需要地图稳定，并且机器不动 
    TUYA_MSG_ROOMS_GET_ROOMS             = 0x21, //主动获取房间 
   // TUYA_MSG_ROOMS_SET_ROOMS             = 0x22, //主动设置房间 
    TUYA_MSG_ROOMS_SET_STRATEGY          = 0x23, //设置清扫模式，不再使用  
            
    TUYA_MSG_ROOMS_SPLIT                 = 0x24, //房间分割
    TUYA_MSG_ROOMS_MERGE                 = 0x25,  //房间合并
    TUYA_MSG_ROOMS_MAP_IS_STABLE         = 0x26,  // 地图是否稳定 
    TUYA_MSG_ROOMS_IS_MANUAL             = 0x27,  // 房间是否主动设置的 
    TUYA_MSG_ROOMS_CLEAR_ROOMS           = 0x28,  // 清掉主动设置的房间信息 
    //feng
    TUYA_MSG_ROOMS_GET_WORKING_ROOMS     = 0x2C,//0x29, //获取工作时候的房间
    TUYA_MSG_ROOMS_GET_EDITING_ROOMS     = 0x2D,//0x2A,  //获取编辑时的房间
    TUYA_MSG_ROOMS_SET_ROOMS             = 0x2E,//0x22, //主动设置房间 
    
    TUYA_MSG_ROOMS_MAP_IS_EDITABLE       = 0x2B,  //是否能够分区编辑

    TUYA_MSG_ROOMS_SET_CLEAN_ROOMS       = 0x40,   //设置选区
    
    TUYA_MSG_TYPE_TYPE_ZOON_CLEAN        = 0x32, //划区清扫次数策略
};


typedef void (*robot_notify_sys_cb_t)(ROBOT_SYS_MSG_E type, void *param);
typedef void (*robot_app_packet_cb_t)(uint8_t *data, uint32_t len);
typedef bool (*robot_allowed_upgrade_cb_t)(void);

// 用于需要提供的回调函数集
typedef struct {
    //机器人系统消息处理函数
    robot_notify_sys_cb_t       sys_cb;
    //380 app自定义消息处理函数
    robot_app_packet_cb_t       app_cb;
    //是否可以升级，如果返回false机器人不允许升级
    robot_allowed_upgrade_cb_t  allow_upgrade_cb;
} ROBOT_CB_FUNS;

// app_server SDK初始化 和 处理线程的创建
ROBOT_RETCODE_E robot_api_init(const ROBOT_CB_FUNS *cbs);
// 常用机器人控制指令
ROBOT_RETCODE_E robot_api_control(ROBOT_CMD_E cmd, void *param);
// 发送自定义数据给380 app， 用户可以根据该接口自定义协议
ROBOT_RETCODE_E robot_api_send_packet(uint8_t cmd, uint8_t *databuf, uint8_t datalen);
// 使580首先进入休眠停止spi通信，然后380才能休眠，
// 否则580 spi通信会唤醒380通信
ROBOT_RETCODE_E robot_api_hoslam_enter_sleep(void);
// 升级接口，fw_file - 升级包路径
ROBOT_RETCODE_E robot_api_fw_upgrade(const char *fw_file, const char *other_file);
// 请求机器人发送虚拟墙和禁区信息
ROBOT_RETCODE_E robot_api_request_virtual_wall(void);
// 播放 序号对应的 语音
ROBOT_RETCODE_E robot_api_play_voice(uint32_t index);
//获取机器是否处于运动状态
ROBOT_RETCODE_E robot_api_get_robot_state(int *state);

uint8_t wifi_app_voice_volume_get(void);
int wifi_app_voice_volume_set(uint8_t level);
void init_voice_path(void);
void set_voice_path(char * language_path);
void get_voice_path(char * language_path);
int wifi_app_voice_language_read(void);
int report_voice_config(void);

//发送消息给hoslam
ROBOT_RETCODE_E robot_api_send_msg_to_hoslam(int type, const char *data, const int len);

#endif /* __ROBOT_API__ */
