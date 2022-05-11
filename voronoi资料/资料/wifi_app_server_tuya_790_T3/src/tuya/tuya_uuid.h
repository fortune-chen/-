#ifndef __TUYA_UUID_H__
#define __TUYA_UUID_H__

#define DEV_FACTORY_CONFIG "DEV_FACTORY_CONFIG"

#define TUYA_CONFIG_OWNER    "tuya_app"
#define TUYA_CONFIG_KEY      "tuya_triplet"

#define KEY_TUYA_PID      "PRODUCT_KEY"
#define KEY_TUYA_UUID     "UUID"
#define KEY_TUYA_KEY      "AUTHKEY"

#define PROTOCOL_WIFI_MAC           (18)
#define PROTOCOL_TUYA_PRODUCT_ID    (20)        // 目前长度为16，预留20
#define PROTOCOL_TUYA_UUID          (24)        // 目前长度为20，预留24
#define PROTOCOL_TUYA_AUTHKEY       (36)        // 目前长度为32，预留36

#define FILE_TUYA_CERT      "/root/app_server/tuya_uuid"

//TUYA UUID KEY 配置文件
#define TUYA_UUID_FILE "tuya_uuid*"

int   tuya_uuid_init(const char *file);
char *tuya_uuid_get(void);
char *tuya_uuid_get_authkey(void);
char *tuya_get_product_key(void);
int tuya_cp_triplet(void);

#endif
