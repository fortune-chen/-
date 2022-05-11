#ifndef __WIFI_APP_FIRMWARE_UPGRADE_H__
#define __WIFI_APP_FIRMWARE_UPGRADE_H__

#include <stdint.h>

#include "sys_api.h"
#include "robot_api.h"

// 固件升级消息
typedef struct
{
    uint32_t        upgrade_cmd;                        // 升级命令

    char            firmware_path[FILE_PATH_MAX_LEN];   // 解压后的固件升级目录

} UPGRADE_MSG;

typedef struct
{
    uint32_t        start;                              // 升级启动标志
    char            firmware_path[FILE_PATH_MAX_LEN];   // 解压后的固件升级目录

} UPGRADE_MGR;

#pragma pack(1)

// ARF Ident
typedef struct
{
    uint8_t     magic[4];           // 固定值
    uint16_t    crc16;              // Checksum
    uint16_t    machine;            // 平台
    uint16_t    version;            // Header版本
    uint16_t    size;               // Ident+Header大小

} ident_t;

// ARF Header V1
typedef struct
{
    uint16_t    flags;              // 文件标志
    uint16_t    mode;               // 文件权限
    uint32_t    size;               // 传输文件大小
    uint8_t     hash[128];          // 传输文件哈希值
    uint8_t     key[128];           // 传输文件密钥
    char        path[128];          // 文件路径
    char        startup[128];       // 启动文件

} header_v1_t;

typedef union
{
    header_v1_t v1;

} header_t;

// ARF文件信息
typedef struct
{
    ident_t     ident;              // 识别信息
    header_t    header;             // 文件头

} file_info_t;

#pragma pack()

#define FILE_PARSE_TEMP             "/tmp/parse_file.tmp"

int wifi_app_firmware_upgrade(const char *firmware_file, const char *other_file);
int wifi_app_firmware_upgrade_msg_parse(int type, const char *msg);
int wifi_app_firmware_upgrade_backup_check(void);

#endif // !__WIFI_APP_FIRMWARE_UPGRADE_H__