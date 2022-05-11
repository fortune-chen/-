#ifndef __WIFI_APP_VOICE_MGR_H_
#define __WIFI_APP_VOICE_MGR_H_

#include <stdint.h>

// 语音播放请求消息
typedef struct
{
    uint16_t    voice_index;
    uint16_t    request_id;

} DATA_VOICE_REQUEST;

typedef struct
{
    uint8_t     enable;
    uint8_t     volume;
    uint8_t     volume_level;
    uint8_t     playing;
    
    DATA_VOICE_REQUEST  request;

    char        version[16];
    char        uuid[36];

} DATA_VOICE_CONFIG;


// API
void* wifi_app_voice_mgr(void *arg);
int wifi_app_voice_play(uint32_t index);
int wifi_app_voice_play_new(uint16_t index, uint16_t replace, uint16_t request_id);

int wifi_app_voice_version_get(char *buf, int size);
int wifi_app_voice_uuid_get(char *buf, int size);
int wifi_app_voice_file_update(const char *file, const char *version, const char *uuid);

uint8_t wifi_app_voice_volume_get(void);
int wifi_app_voice_volume_set(uint8_t percent);

uint8_t wifi_app_voice_enable_get(void);
int wifi_app_voice_enable_set(uint8_t enable);


#endif