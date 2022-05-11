#ifndef __WIFI_APP_MSG_DP_ENUM__
#define __WIFI_APP_MSG_DP_ENUM__

enum
{
    DPID_POWER_GO               = 1,    // 清扫开关(可下发可上报)
    DPID_SWITCH_CHARGE          = 3,    // 回充开关(可下发可上报)
    DPID_MODE                   = 4,    // 工作模式(可下发可上报)
    DPID_DIRECTION_CONTROL      = 12,    // 方向(可下发可上报)
    DPID_STATUS                 = 5,    // 工作状态(只上报)
    DPID_ELECTRICITY_LEFT       = 8,    // 剩余电量(只上报)
    
	DPID_EDGE_BRUSH_LIFE             = 17,    //边刷剩余寿命(只上报)
	DPID_EDGE_BRUSH_LIFE_RESET		= 18,    //边刷寿命重置
    DPID_ROLL_BRUSH_LIFE             = 19,    //滚刷剩余寿命(只上报)
  	DPID_ROLL_BRUSH_LIFE_RESET		= 20,    //滚刷寿命重置
    DPID_FILTER_LIFE                 = 21,    //滤芯剩余寿命(只上报)
    DPID_FILTER_LIFE_RESET           = 22,    //滤芯寿命重置
    DPID_MOP_LIFE                    = 23,    //拖布剩余寿命(只上报)
    DPID_MOP_LIFE_RESET            	= 24,    //拖布寿命重置
    
    DPID_CALLING_ROBOT          = 11,   //呼叫机器人
    DPID_SUCTION                = 9,   // 吸力选择(可下发可上报)
    DPID_CLEAN_AREA             = 7,   // 清扫面积(只上报)
    DPID_CLEAN_TIME             = 6,   // 清扫时间(只上报)
    DPID_FAULT                  = 28,   // 故障告警(只上报)
    DPID_PAUSE                  = 2,  // 暂停开关(可下发可上报)
    DPID_WATER_MODE             = 10,  // 水箱档位(可下发可上报)
    DPID_RESET_MAP              = 13,  // 重置地图(可下发可上报)
    DPID_REQUEST                = 16,  // 请求指令(可下发可上报)
    DPID_COMMAND                = 15,  // 指令通信(可下发可上报)
    DPID_VOICE_DATA             = 35,  //语音包下载DP(可下发可上报)
    DPID_PATH_DATA              = 14,  //路径数据
    DPID_OPEN_NOT_DISTURB       = 25,  //勿扰模式开关
    DPID_NOT_DISTURB_TIME  		= 33,	//勿扰时间(只上报)
    DPID_SMART_ROOMS            = 101,  //智能分区（只下发）
    DPID_ROOM_MSG_RET           = 102,  //房间信息返回（只上报）
    DPID_CONTINUE_CLEAN         = 27, //断点续扫
    DPID_SET_VOLUME             = 26,  //设置音量增益
    DPID_Y_MOP                  = 104,  //Y字型拖地
    DPID_CUSTOM_MODE_SWITCH     = 105,  //定制模式开关
    DPID_VERSON                 = 106,  //固件版本上报

    
   


    DPID_CLEAN_UP               = 133,  //清扫完成标志(只上报)
        
        // 以下为非公版定义，公版支持的话需要修改DP定义
  //  DPID_PATH_DATA              = 120,  //路径数据处理函数
      
       
    //    DPID_DELETE_HOSLAM_MAP    = 109,  //删除扫地机上的历史地图

     DPID_FORBIDDEN_RESULT      =160,
     //DPID_SET_MAP_RESULT        = 161,  //地图设置结果返回（只上报）
     DPID_VWALL_REUSULT         = 163, //虚拟墙结果返回（只上报）
     DPID_MAP_ID                =164,   //地图id（只上报）
    DPID_MAP_MANAGEMENT         = 165,  //地图管理（只下发）

};

typedef enum {
    WORK_MODE_SMART = 0,    // 自动清扫
    WORK_MODE_ZONE,         // 区域清扫
    WORK_MODE_POSE,         // 指哪扫哪
    WORK_MODE_PART,         // 局部清扫
    WORK_MODE_CHARGEGO,     // 自动充电
    WORK_MODE_WALLFOLLOW,   // 沿边清扫
    WORK_MODE_CHOICE_CLEAN,        //选区清扫
} DP_WORK_MODE;

enum
{
    CTRL_FORWARD,                       // 前进
    CTRL_BACKWARD,                      // 后退
    CTRL_TURNLEFT,                      // 左转
    CTRL_TURNRIGHT,                     // 右转
    CTRL_STOP                           // 停止
};

typedef enum {
    //FAN_CLOSE,                          // 关闭
    FAN_SILENT,                         // 静音
    FAN_NORMAL,                         // 标准
    FAN_STRONG,                         // 强力  
    FAN_NO_SET = 0xff,                         // 风机未设置
} DP_FAN_MODE;

typedef enum {
    WATER_LEVEL_OFF,                       // 关闭水箱
    WATER_LEVEL_1,                         // 水箱1挡
    WATER_LEVEL_2,                         // 水箱2挡
    WATER_LEVEL_3,                         // 水箱3挡
    WATER_LEVEL_NO_SET = 0xff,                    // 水箱挡位未设置
} DP_WATER_LEVEL;

typedef enum
{
    VOICE_CHINESE = 0,                  // 中文语音
    VOICE_ENGLISH,                      // 英文语音

    VOICE_MAX
}DP_VOICE_CLASS;


typedef enum {
    STATE_STANDBY,                      // 待机中
    STATE_SMARTCLEAN,                    // 自动清扫
    STATE_ZONECLEAN,                    // 区域清扫
    STATE_PARTCLEAN,                    // 局部清扫
    STATE_PAUSED = 5,                  // 清扫暂停 枚举值等于5主要是为了适配涂鸦T3公版协议
    STATE_GOTOPOS,                      // 指哪扫哪
    STATE_ARRIVED,                      // 到达目标点
    STATE_UNARRIVABLE,                  // 目标点不可达
    STATE_GOCHARGE,                     // 回充
    STATE_CHARGING,                     // 充电中 10
    STATE_CHARGEDONE,                   // 充电完成
    STATE_SLEEP,                        // 休眠
    STATE_ERROR,                        // 故障
    STATE_WALLFOLLOW,                   // 沿边清扫
    STATE_REMOTE_CONTROL,               // 遥控器
    STATE_CHOICE_CLEAN,                 //选区清扫
    STATE_REPOSITING,      			//重定位中
} DP_ROBOT_STATE;
	
enum
{
	ROBOT_REPOSITING = 1,   	//重定位中
	ROBOT_REPOSITION_SUCCESS,	//重定位成功 暂使不使用
	ROBOT_REPOSITION_FAILED,	//重定位失败 暂使不使用
};

enum
{
    COMMFLAG_GETMAP = 0,                // 请求地图
    COMMFLAG_GETPATH,                   // 请求路径
    COMMFLAG_INMAP,                     // 地图显示状态
    COMMFLAG_GETNAVI,                   // 请求规划路径
    COMMFLAG_RESET_MAP,                  //请求重置地图
    COMMFLAG_GET_STATUS,                //请求机器状态
    COMMFLAG_GET_MAP_ID,               //请求获取地图id
    

};

#endif
