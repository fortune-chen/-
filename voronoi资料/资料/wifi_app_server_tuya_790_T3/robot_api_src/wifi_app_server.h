#ifndef __WIFI_APP_SERVER_H__
#define __WIFI_APP_SERVER_H__

#include <stdint.h>
#include <pthread.h>
#define MSG_MAX_SIZE                        4096

int rooms_to_hoslam_semid ;
char* rooms_to_hoslam_shmadd;
int rooms_from_hoslam_semid;
char* rooms_from_hoslam_shmadd;
pthread_mutex_t rooms_lock;
//pthread_mutex_t map_save_lock;
int GET_WORKING_ROOMS_FLAG;
// 文件路径定义
#define FILE_SYSTEM_VERSION                 "/version"
#define FILE_FIRMWARE_VERSION               "/root/version"

#define SHELL_STARTUP_HOSLAM                "null"
#define SHELL_STARTUP_DAEMON                "null"

#ifdef APP_GRIT
#define CONFIG_YGCLIENT                     "/root/yg/etc/config.json"
#endif // APP_GRIT

#define APP_PATH_GRIT_CLIENT                "/root/yg"

#define UPGRADE_DIR_NAME_FIRMWARE           "firmware"
#define UPGRADE_FILE_NAME_MD5               "md5.txt"
#define UPGRADE_FILE_NAME_GRIT_CLIENT       "ygclient"

#define FILE_NAME_FIRMWARE_UPGRADER         "firmware_upgrader"

#define FIRMWARE_PATH_VOICE_FILES           "/root/firmware/arobot/voice"
#define FIRMWARE_PATH_GRIT_CLIENT           "/root/firmware/arobot/yg"


typedef struct
{
    int     mtype;
    char    data[MSG_MAX_SIZE];
} MSG;

pthread_mutex_t rooms_lock;

int create_process_threads(void);
int send_msg_to_wifi_app(int type, const char *data);
int send_map_to_wifi_app(const char *data);

int send_msg_to_hoslam_app(int type, const char *data, const int len);

int send_msg_to_upgrade_app(int type, const char *data, const int len);
int get_working_rooms_flag();

#endif
