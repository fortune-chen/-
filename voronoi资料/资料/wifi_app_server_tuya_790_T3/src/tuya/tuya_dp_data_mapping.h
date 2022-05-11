#ifndef __WIFI_APP_DATA_MAPPING_H__
#define __WIFI_APP_DATA_MAPPING_H__

#define INVALID_ENUM    (int)(-1)

typedef struct {
    int     protocol_enum;      // 一微协议枚举数据
    int     dp_enum;            // 涂鸦DP枚举数据

} PROTOCOL_MAP;


// 下发的命令
int wifi_app_data_map_manual_control(int dp_enum);
int wifi_app_data_map_clean_mode(int dp_enum);
int wifi_app_data_map_fan_mode(int dp_enum);
int wifi_app_data_map_water_level(int dp_enum);

// 上报的状态
int wifi_app_data_map_robot_status(int protocol_enum);
int wifi_app_data_map_tuya_water_level(int protocol_enum);
int wifi_app_data_map_tuya_fan(int protocol_enum);

#endif
