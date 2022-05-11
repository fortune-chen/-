#ifndef __TUYA_IOT_UPLOADER_H__
#define __TUYA_IOT_UPLOADER_H__

#include <stdint.h>

#pragma pack(1)
typedef struct {
    uint16_t x;
    uint16_t y;
}XYPoint;
#pragma pack()


#pragma pack(1)
typedef struct
{
    uint8_t         flag;
    uint16_t        head;   
    uint8_t         type;
    uint16_t        w;
    uint16_t        h;
    uint16_t        ox;
    uint16_t        oy;
    uint16_t        resolution; /* 地图分辨率 */
    uint16_t        charger_x;
    uint16_t        charger_y;
    uint32_t        len_before_lz4; /* 数据压缩之前的总长度 (不包含地图报头)*/
    uint16_t        lz4len;
 //   uint8_t         editable;
 //   float           resolution;
    uint8_t         map[0];
} SEND_MAP;

#pragma pack()


#pragma pack(1)
typedef struct  
{
    uint8_t         flag;  /* 协议版本标志位 */
    uint16_t        head;
    uint8_t         init_flag; /* 初始路径标志位 
                                   0x00 初始化完成后数据标志位
                                   0x01 初始化路径ID从00开始标志位（路径ID从00开始的前5个包需使用高标志位上报）*/
    uint8_t         type;      /* 路径类型：
                                   0x01 全量路径 + 普通模式
                                   0x02 全量路径 + 复杂模式
                                   0x03 导航路径         */            
    uint32_t        num;
    uint16_t        direction; /* 角度 */
    uint16_t        lz4len;
    XYPoint         path[0];

} SEND_PATH;
#pragma pack()

extern pthread_mutex_t wifi_connect_ptx;

int tuya_iot_upload_navigation(SEND_PATH *send_path, int total_bytes);
int tuya_iot_upload_path(SEND_PATH *send_path, int total_bytes);
int tuya_iot_upload_map(SEND_MAP *send_map, int total_bytes);
int set_send_rooms_size_compress(int value);

void force_record_map_path(void);

int tuya_iot_upload_map_rooms(SEND_MAP *send_map, int total_bytes);

int creat_pthread_save_map_to_cloud(uint8_t *data);
void set_send_cloud_flag(int len);

int init_all_mutex(void);



#endif /* __TUYA_IOT_UPLOADER_H__ */
