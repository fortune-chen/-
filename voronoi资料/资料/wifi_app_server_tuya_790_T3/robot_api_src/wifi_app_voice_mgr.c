#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libgen.h>
#include <fcntl.h>
#include <pthread.h>

#include "sys_api.h"
#include "sys_queue.h"
#include "robot_api_inner.h"
#include "wifi_app_voice_mgr.h"
#include "robot_api.h"
#include <dirent.h>

#define VOICE_FILES_PATH            "/root/voice"
#define VOICE_FILES_ENGLISH_PATH    "/root/voice_english"

#define CMD_VOICE_GAIN              0x22

#define VOICE_VOLUME_MIN            53  // 对应百分比0
#define VOICE_VOLUME_MAX            63  // 对应百分比100
#define VOICE_VOLUME_70             60  // 对应百分比70
#define VOICE_VOLUME_MUTE           0  // 对应百分比70
#define VOICE_PLAYER_QUEUE_MAX      5
#define VOICE_VOLUME_LEVEL          100  //音量档位


static pthread_mutex_t voice_player_mutex;
static queue_t *voice_player_queue = NULL;
static DATA_VOICE_CONFIG voice_config;
//static int voice_playing = 1;       // 开机由启动脚本播放开机语音，所以默认值为1
static pthread_mutex_t voice_language_mutex;
char g_voice_path[LANG_PATH_MAX_LEN] = {0};

void set_fixed_voice_path(void);

static int wifi_app_voice_volume_write(uint8_t volume, uint8_t level)
{
    int ret;
    int tmp_shell_ret;

    //ret = sys_shell("sed -i 's/^VOLUME=.*/VOLUME=%d/' %s", volume, SHELL_STARTUP_AUDIO);
    if (access(SHELL_AUDIO_ONESELF, R_OK) == 0)
    {
        
        tmp_shell_ret = sys_shell("cat %s | grep 'LEVEL'", SHELL_AUDIO_ONESELF);
        if(tmp_shell_ret == 1)
        {
            /* 命令正确但是不存在level */
            printf("add volume level\n");
            ret = sys_shell("echo LEVEL=%d >> %s", level, SHELL_AUDIO_ONESELF);
        }
        else if(tmp_shell_ret == 0)
        {
            /* 命令正确存在level, 替换level */
            printf("modify volume level\n");
            ret = sys_shell("sed -i 's/^LEVEL=.*/LEVEL=%d/' %s", level, SHELL_AUDIO_ONESELF);
        }
        else
        {
            printf("prase volume file fail\n");
        }
    }
    else
    {
        printf("create volume file\n");
        ret = sys_shell("echo LEVEL=%d > %s", level, SHELL_AUDIO_ONESELF);
    }
    
    return ret;
}

static uint8_t wifi_app_voice_volume_percent_exchange(uint8_t volume, uint8_t level)
{
    uint8_t ret;

    if (volume == 0xFF)
    {
        // 0-100档位转成音量设置值
        if(0 == level)
        {
            ret = VOICE_VOLUME_70;
        }
        else
        {
            ret = level * (VOICE_VOLUME_MAX - VOICE_VOLUME_MIN) / VOICE_VOLUME_LEVEL + VOICE_VOLUME_MIN;
        }
    }
    else if (level == 0xFF)
    {
        // 音量值转成档位
        if(volume <= VOICE_VOLUME_MIN)
        {
            ret = 0;
        }
        else
        {
            ret = VOICE_VOLUME_LEVEL * (volume - VOICE_VOLUME_MIN) / (VOICE_VOLUME_MAX - VOICE_VOLUME_MIN);
        }
    }

    return ret;
}

static int wifi_app_voice_volume_read(void)
{
    int ret;

    // 判断启动脚本
    if (access(SHELL_STARTUP_AUDIO, R_OK) != 0)
    {
        //sys_log(LOG_CRIT, "access S00voice FAILED");
        printf("access S00voice FAILED\n");
        return -1;
    }

    ret = sys_shell_result_to_int(SHELL_STARTUP_AUDIO " volume");
    
    //sys_log(LOG_INFO, "get volume from file: %d", ret);
    printf("get volume from file: %d\n", ret);

    if (ret > VOICE_VOLUME_MAX)
    {
        wifi_app_voice_volume_write(VOICE_VOLUME_MAX, 10);
        ret = VOICE_VOLUME_MAX;
    }
    else if(ret < VOICE_VOLUME_MIN)
    {
        wifi_app_voice_volume_write(VOICE_VOLUME_70, 0);
    }

    return ret;
}

static int wifi_app_voice_level_read(void)
{
    int ret;

    // 判断配置文件
    if (access(SHELL_AUDIO_ONESELF, R_OK) != 0)
    {
        //sys_log(LOG_CRIT, "access S00voice FAILED");
        printf("access voice_config FAILED\n");
        return -1;
    }

    ret = sys_shell_result_to_int("cat %s | grep 'LEVEL' | awk -F '=' '{print $2}'", SHELL_AUDIO_ONESELF);
    printf("get level from file: %d\n", ret);

    if (ret > VOICE_VOLUME_LEVEL)
    {
        ret = VOICE_VOLUME_LEVEL;
    }
    else if(ret < 0)
    {
        ret = 80;
        printf("voice ret = %d\n",ret);
    }

    return ret;
}


static int wifi_app_voice_version_read(const char *file, char *buf, int size)
{
    int ret;

    ret = sys_cat_file((char*)file, buf, size);
    //sys_log(LOG_INFO, "get voice version from file: %s-%s", file, buf);
    printf("get voice version from file: %s-%s\n", file, buf);

    return ret;
}

static int wifi_app_voice_uuid_read(const char *file, const char *version, char *buf, int size)
{
    int ret;

    if (!version || !buf)
    {
        return -1;
    }

    ret = sys_cat_file((char*)file, buf, size);
    if (!ret)
    {
        //sys_log(LOG_INFO, "get voice uuid from file: %s", buf);
        printf("get voice uuid from file: %s\n", buf);
    }
    else
    {
        uint8_t v1, v2, v3, v4;
        sscanf(version, "%d.%d.%d.%d", &v1, &v2, &v3, &v4);
        snprintf(buf, size, "%03d%03d%03d%03d", v1, v2, v3, v4);
        //sys_log(LOG_INFO, "get voice uuid from version: %s", buf);
        printf("get voice uuid from version: %s\n", buf);
    }

    return 0;
}

void init_voice_path(void)
{
    int ret;
    // 初始化队列互斥量
    ret = pthread_mutex_init(&voice_language_mutex, NULL);
    if (ret)
    {
        printf("init voice_player_mutex error\n");
        return;
    }
    memset(g_voice_path, 0, sizeof(g_voice_path));
}

void set_voice_path(char * language_path)
{
    pthread_mutex_lock(&voice_language_mutex);
    memcpy(g_voice_path, language_path, sizeof(g_voice_path));
    pthread_mutex_unlock(&voice_language_mutex);
    printf("set_voice_path %s\n", g_voice_path);
    return;
}
void get_voice_path(char * language_path)
{
    pthread_mutex_lock(&voice_language_mutex);
    memcpy(language_path, g_voice_path, sizeof(g_voice_path));
    pthread_mutex_unlock(&voice_language_mutex);
    printf("get_voice_path %s\n", language_path);
    return;
}


static int wifi_app_voice_init(void)
{
    int ret;

    // 初始化队列互斥量
    ret = pthread_mutex_init(&voice_player_mutex, NULL);
    if (ret)
    {
        printf("init voice_player_mutex error\n");
        return -1;
    }
    
    // 创建语音播放队列
    voice_player_queue = sys_queue_create(VOICE_PLAYER_QUEUE_MAX);
    if (!voice_player_queue)
    {
        printf("creat voice_player_queue ERROR\n");
        return -1;
    }

    init_voice_path();
    /* 开机就读一次配置或者会设置，不用做互斥操作 */
    //ret = wifi_app_voice_language_read();    
    set_fixed_voice_path();

    // 使能Audio PA控制IO PE19 
    // PA控制IO在S00voice启动脚本中使能和开启，如果在这里重复使能，则会造成语音中断
    // sys_shell("echo 147 > /sys/class/gpio/export 2>/dev/null");
    // sys_shell("echo out > /sys/class/gpio/gpio147/direction");
    // sys_shell("echo 1 > /sys/class/gpio/gpio147/value");

    // 通知380打开AP IO PB3
#ifndef USE_BOARD_780
    // sys_shell("echo 35 > /sys/class/gpio/export 2>/dev/null");
    // sys_shell("echo out > /sys/class/gpio/gpio35/direction");
    // sys_shell("echo 0 > /sys/class/gpio/gpio35/value");
#else
    sys_shell("echo 46 > /sys/class/gpio/export 2>/dev/null");
    sys_shell("echo out > /sys/class/gpio/gpio46/direction");
    sys_shell("echo 0 > /sys/class/gpio/gpio46/value");
#endif
    // 初始化
    voice_config.enable  = 1;
    voice_config.playing = 0xFF;    //开机由启动脚本播放开机语音，开机语音不允许打断，标记为其他值
    //voice_config.volume = wifi_app_voice_volume_read();

    ret = wifi_app_voice_level_read();
    if(ret != -1)
    {
        voice_config.volume_level = ret;
    }
    else
    {
        voice_config.volume_level = 80;
    }
    
    voice_config.volume = wifi_app_voice_volume_percent_exchange(0xFF, voice_config.volume_level);
    if ((voice_config.volume < VOICE_VOLUME_MIN || voice_config.volume > VOICE_VOLUME_MAX))
    {
        printf("voice_config.volume percent ERROR, %d\n", voice_config.volume);
    }
    //voice_config.volume_level = wifi_app_voice_volume_percent_exchange(voice_config.volume, 0xFF);
    wifi_app_voice_version_read("/root/voice/version", voice_config.version, sizeof(voice_config.version));
    //wifi_app_voice_uuid_read(FILE_VOICE_UUID, voice_config.version, voice_config.uuid, sizeof(voice_config.uuid));
    //sys_log(LOG_INFO, "voice volume percent: %d", voice_config.volume_percent);
    printf("voice volume level: %d\n", voice_config.volume_level);

    return 0;
}

static void wifi_app_voice_audio_enable(void)
{
#ifndef USE_BOARD_780
    // 方式1：580打开PA
    //sys_shell("echo 1 > /sys/class/gpio/gpio147/value");

    // 方式2：通知380打开PA
    //sys_shell("echo 1 > /sys/class/gpio/gpio35/value");
    usleep(20000);
    //sys_shell("echo 0 > /sys/class/gpio/gpio35/value");
#else
    sys_shell("echo 1 > /sys/class/gpio/gpio46/value");
    usleep(20000);
    sys_shell("echo 0 > /sys/class/gpio/gpio46/value");
#endif
    
}

static void wifi_app_voice_audio_disable(void)
{
    // 方式1：580关闭PA
    //sys_shell("echo 0 > /sys/class/gpio/gpio147/value");

    // 方式2：通知380关闭PA
    inform_voice_play_complete();
}

#if 1   // 改用系统函数获取语音文件，减少执行shell命令的系统资源占用
static int voice_file_index = 0;
static int wifi_app_voice_file_filter(const struct dirent *ent)
{
    int idx, ret;

    if (ent->d_type != DT_REG)
    {
        return 0;
    }

    ret = sscanf(ent->d_name, "%d_*", &idx);
    if (ret != 1)
    {
        return 0;
    }

    if (idx == voice_file_index)
    {
        return 1;
    }

    return 0;
}

static int wifi_app_voice_file_get(uint32_t index, char *file)
{
    struct dirent **namelist;
    int ret, i;
    char voice_path[128] = {0};

    get_voice_path(voice_path);
    voice_file_index = index;

    ret = scandir(voice_path, &namelist, wifi_app_voice_file_filter, alphasort);
    if (ret < 0)
    {
        printf("get voice file failed %d\n",index);
        return -1;
    }

    if (ret == 0)
    {
        return -1;
    }

    sprintf(file, "%s/%s", voice_path, namelist[0]->d_name);

    for (i = 0; i < ret; i++)
    {
        //printf("%s\n", namelist[i]->d_name);
        free(namelist[i]);
    }
    free(namelist);

    return 0;
}
#else
static int wifi_app_voice_file_get(uint32_t index, char *file)
{
    FILE *pf;
    char buff[512];
    char cmd[64];
    char voice_path[128] = {0};

    get_voice_path(voice_path);
    printf("wifi_app_voice_file_get path %s\n", voice_path);

    sprintf(cmd, "find %s/* | grep \"%s/%02d\\_.*\\|%s/%03d\\_.*\"", voice_path, voice_path, index, voice_path, index);
    printf("2 wifi_app_voice_file_get path %s\n", cmd);
    pf = sys_popen(cmd, "r");
    if (pf == NULL)
    {
        return -1;
    }
    memset(buff, 0, sizeof(buff));
    if (fgets(buff, sizeof(buff), pf) == NULL)
    {
        sys_pclose(pf);
        return -1;
    }
    sys_pclose(pf);

    sys_str_del_eol(buff);

    // 找不到语音文件
    if (strlen(buff) == 0)
    {
        return -1;
    }

    // 语音文件判断
    if (access(buff, R_OK) != 0)
    {
        return -1;
    }

    strcpy(file, buff);
    return 0;
}
#endif
static int wifi_app_voice_player(uint32_t index)
{
    int ret;
    char file[FILE_PATH_MAX_LEN];
    char *fname = NULL;
    char *suffix = NULL;

    int tmp_volume;
    if(voice_config.volume_level == 0)
    {
        tmp_volume = 0;
    }
    else
    {
        //-a 音量控制 范围（-175~+18）
        tmp_volume =  ((int)voice_config.volume_level/2) - 32;
    }
    if (voice_config.enable == 0)
    {
        return 0;
    }

    ret = wifi_app_voice_file_get(index, file);
    if (ret)
    {
        printf("cannot find voice file name, %d\n", index);
        return -1;
    }

    fname = basename(file);
    suffix = strrchr(fname, '.');
    if (strncmp(suffix, ".mp3", 4) == 0)
    {
        while (1)
        {
            if (sys_is_process_exist("madplay") == 0)
            {
                break;
            }
            usleep(100000);
        }

        ret = sys_shell("madplay -a %d -q %s &", tmp_volume,file);
        wifi_app_voice_audio_enable();
    }
    else if (strncmp(suffix, ".wav", 4) == 0)
    {
        while (1)
        {
            if (sys_is_process_exist("tinyplay") == 0)
            {
                break;
            }
            usleep(100000);
        }

        wifi_app_voice_audio_enable();
        ret = sys_shell("tinyplay %s &", file);
    }
    else
    {
        printf("unsuport voice file!\n");
        return -1;
    }

    printf("wifi_app_voice_player: %d\n", index);
    voice_config.playing = 1;

    return 0;
}

int wifi_app_voice_play(uint32_t index)
{
    int ret = -1;
    uint32_t *data = NULL;

    // 等待初始化完成
    if (voice_player_queue == NULL)
    {
        while (1)
        {
            if (voice_player_queue != NULL)
            {
                break;
            }
            sleep(1);
        }
    }

    pthread_mutex_lock(&voice_player_mutex);

    do 
    {
        // 判断播放队列是否已满
        if (sys_queue_is_full(voice_player_queue))
        {
            printf("voice_player_queue is full\n");
            break;
        }

        // 将语音索引放入队列
        data = (uint32_t*)malloc(sizeof(uint32_t));
        if (!data)
        {
            printf("malloc ERROR\n");
            break;
        }
        *data = index;
        sys_queue_push_tail(voice_player_queue, (void*)data);
        ret = 0;
    }
    while (0);

    pthread_mutex_unlock(&voice_player_mutex);

    return ret;
}

int wifi_app_voice_play_new(uint16_t index, uint16_t replace, uint16_t request_id)
{
    int ret = -1;
    DATA_VOICE_REQUEST *request = NULL;

    // 等待初始化完成
    if (voice_player_queue == NULL)
    {
        while (1)
        {
            if (voice_player_queue != NULL)
            {
                break;
            }
            sleep(1);
        }
    }

    pthread_mutex_lock(&voice_player_mutex);

    // 中断当前播放，强制播放当前语音
    if (replace)
    {
        if (!sys_queue_is_empty(voice_player_queue))
        {
            sys_queue_clear(voice_player_queue);
            sys_log(LOG_INFO, "clear voice queue");
        }
    }

    do 
    {
        // 判断播放队列是否已满
        if (sys_queue_is_full(voice_player_queue))
        {
            sys_log(LOG_CRIT, "voice_player_queue is full");
            break;
        }

        // 将语音索引放入队列
        request = (DATA_VOICE_REQUEST*)malloc(sizeof(DATA_VOICE_REQUEST));
        if (!request)
        {
            sys_log(LOG_CRIT, "malloc ERROR");
            break;
        }
        request->voice_index = index;
        request->request_id  = request_id;
        sys_queue_push_tail(voice_player_queue, (void*)request);
        ret = 0;
    }
    while (0);

    // 中断当前播放，强制播放当前语音
    if (ret == 0 && replace && voice_config.playing == 1)
    {
        sys_log(LOG_INFO, "quit voice player");
        sys_shell("tinymix set 2 0");
        sys_shell("killall -9 madplay tinyplay 2>/dev/nul");
        sys_shell("tinymix set 2 1");
    }

    pthread_mutex_unlock(&voice_player_mutex);

    return ret;
}

void* wifi_app_voice_mgr(void *arg)
{
    DATA_VOICE_REQUEST *request = NULL;
    int ret;

    // 初始化
    ret = wifi_app_voice_init();
    if (ret)
    {
        return NULL;
    }

    while (1)
    {
        usleep(100000);

        // 判断语音是否播放完毕
        if (sys_queue_is_empty(voice_player_queue))
        {
            if (voice_config.playing 
                && sys_is_process_exist("madplay") == 0
                && sys_is_process_exist("tinyplay") == 0)
            {
                voice_config.playing = 0;
                wifi_app_voice_audio_disable();
            }
            continue;
        }

        // 取出队列数据
        pthread_mutex_lock(&voice_player_mutex);        
        //index = (uint32_t*)sys_queue_pop_head(voice_player_queue);
        request = (DATA_VOICE_REQUEST*)sys_queue_pop_head(voice_player_queue);
        pthread_mutex_unlock(&voice_player_mutex);
        if (!request)
        {
            continue;
        }

        memcpy(&voice_config.request, request, sizeof(DATA_VOICE_REQUEST));

        // 释放数据
        free(request);
        request = NULL;

         // 播放语音
        wifi_app_voice_player(voice_config.request.voice_index);

    }
    
    // 释放队列
    sys_queue_free(voice_player_queue);
    voice_player_queue = NULL;

    return 0;
}

int wifi_app_voice_version_get(char *buf, int size)
{
    snprintf(buf, size, voice_config.version);

    return 0;
}

int wifi_app_voice_uuid_get(char *buf, int size)
{
    snprintf(buf, size, voice_config.uuid);

    return 0;
}

uint8_t wifi_app_voice_volume_get(void)
{
    return voice_config.volume_level;
}

int wifi_app_voice_volume_set(uint8_t level)
{
    uint8_t volume = wifi_app_voice_volume_percent_exchange(0xFF, level);

    if ((volume >= VOICE_VOLUME_MIN && volume <= VOICE_VOLUME_MAX))
    {
        if(level == 0)
        {
            sys_shell("tinymix set 1 %d", VOICE_VOLUME_MUTE);
        }
        else
        {
            sys_shell("tinymix set 1 %d", volume);
        }
        wifi_app_voice_volume_write(volume, level);
        voice_config.volume = volume;
        voice_config.volume_level = level;
        robot_api_send_packet(CMD_VOICE_GAIN, &level, 1);    // 通知380音量改变 
        //sys_log(LOG_INFO, "set voice volume: %d-%d", percent, volume);
        printf("set voice volume: %d-%d\n", level, volume);
    }
    else
    {
        //sys_log(LOG_CRIT, "voice percent ERROR, %d", percent);
        printf("voice percent ERROR, %d\n", level);
    }
    
    return 0;
}

uint8_t wifi_app_voice_enable_get(void)
{
    return voice_config.enable;
}

int wifi_app_voice_enable_set(uint8_t enable)
{
    voice_config.enable = enable;
    //sys_log(LOG_INFO, "set voice enable: %d", enable);
    printf("set voice enable: %d\n", enable);
}

int report_voice_config(void)
{
    //wifi_app_msg_send(CMD_ROBOT_CONFIG, sizeof(DATA_VOICE_CONFIG), (const char*)&voice_config);
    printf("report_voice_config\n");
}

int wifi_app_voice_file_update(const char *file, const char *version, const char *uuid)
{
    uint8_t md5[64];
    if ((!file) || !version)
    {
        printf("para is NULL");
        return 0;
    }

    if (strcmp(voice_config.version, version) ==0)  /* 判断当前语音版本和下载的版本是否一致，若一致，则不进行安装 ，直接返回成功 */
    {
        printf("voice update: voice pack version is the same\n");
        sys_shell("rm -f %s", file);
        return 1;
    }

    if (access(file, R_OK) != 0)
    {
        //sys_log(LOG_CRIT, "voice update: file is not exist(%s)", file);
        printf("voice update: file is not exist(%s)\n", file);
        return 0;
    }

    char path[FILE_PATH_MAX_LEN];
    char pack_version[16];
    int ret;

    // 将升级文件移动至tmp目录中解压
    sys_shell("mv -f %s /tmp/tmp.tar.gz", file);
    sys_shell("gzip -d /tmp/tmp.tar.gz");
    sys_shell("tar -xf /tmp/tmp.tar -C /tmp");
    sys_shell("rm -f /tmp/tmp.tar");

    snprintf(path, FILE_PATH_MAX_LEN, "/tmp/voice");
    if (access(path, R_OK) != 0)
    {
        //sys_log(LOG_CRIT, "voice update: folder is not exist(%s)", path);
        printf("voice update: folder is not exist(%s)\n", path);
        return 0;
    }

    // 判断解压出来的语音包版本与传入的参数是否一致
    wifi_app_voice_version_read("/tmp/voice/version", pack_version, sizeof(pack_version));
    if (strcmp(pack_version, version) !=0)
    {
        //sys_log(LOG_CRIT, "voice update: voice pack version ERROR");
        printf("voice update: voice pack version verification error\n");
        sys_shell("rm -rf %s", path);
        return 0;
    }

    sys_set_firmware_partition_writeable(1);
    do
    {
        char *dest;
        static uint8_t state;

        if (sys_get_firmware_partition_exist_flag())
        {
            dest = FIRMWARE_PATH_VOICE_FILES;
            state = 0;
        }
        else
        {
            dest = VOICE_FILES_PATH;
            state = 1;
        }
        
        ret = sys_shell("rm -rf %s", dest);
        if (ret)
        {
            break;
        }

        ret = sys_shell("mv -f %s %s", path, dest);
        if (ret)
        {
            break;
        }

        if(state == 0)  /* 若是新的系统，则把原来的符号链接删除，创建新的符号链接 */
        {
           ret = sys_shell("rm -rf %s", VOICE_FILES_PATH);
           if (ret)
           {
             break;
           }      

           ret = sys_shell("ln -s %s %s", FIRMWARE_PATH_VOICE_FILES,VOICE_FILES_PATH);
           if (ret)
           {
             break;
           }     
        }
        sys_shell("sync");
        
    } while (0);
    sys_set_firmware_partition_writeable(0);

    if (!ret)
    {
        strncpy(voice_config.version, pack_version, sizeof(voice_config.version));
        //sys_log(LOG_INFO, "voice update: succeed, verion=%s, uuid=%s", pack_version, pack_uuid);
        printf("voice update: succeed, verion=%s\n", pack_version);
        return 1;
    }

    return 0;
}
