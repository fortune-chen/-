#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <libgen.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/shm.h>
#include <sys/stat.h>

#include "sys_api.h"
#include "sys_timer.h"
#include "sys_semaphore.h"
#include "robot_api_inner.h"
#include "wifi_app_server.h"
#include "wifi_app_voice_mgr.h"
#include "wifi_app_firmware_upgrade.h"

#define WIFI_APP_SERVER_DEBUG               0
#define WIFI_APP_SERVER_FIRMWARE_UPGRADE    0
#define WIFI_APP_DONOT_SEND_TO_WIFI         0

#define IPC_KEY_PATH                        "/"

// YG App 进程通信：
#define MSG_WIFI_CTRL_ID                    1               // 消息队列：WiFi App Server <-- YG App
#define MSG_WIFI_STAT_ID                    2               // 消息队列：WiFi App Server --> YG App
#define SHM_WIFI_MAP_ID                     3               // 共享内存：WiFi App Server --> YG App
#define SHM_WIFI_MAP_SIZE                   (200 * 1024)     // 共享内存大小
#define SEM_WIFI_MAP_ID                     4               // 信号量

// HoSlam App 进程通信：
#define PIPE_HOSLAM_CTRL_PATH               "/tmp/app_ctrl" // 命名管道：WiFi App Server --> Hoslam App (弃用)
#define SHM_HOSLAM_MAP_ID                   11              // 共享内存：WiFi App Server <-- Hoslam App
#define SHM_HOSLAM_MAP_SIZE                 (400 * 1024)    // 共享内存大小
#define SEM_HOSLAM_MAP_ID                   12              // 信号量
#define MSG_HOSLAM_POSITION_ID              13              // 消息队列：WiFi App Server <-- Hoslam App (弃用)
#define MSG_HOSLAM_CTRL_ID                  14              // 消息队列：WiFi App Server --> Hoslam App
#define MSG_HOSLAM_STAT_ID                  15              // 消息队列：WiFi App Server <-- Hoslam App

// Firmware Upgrade 进程通信
#define MSG_UPGRADE_CTRL_ID                 21              // 消息队列：WiFi App Server --> Firmware Upgrade App
#define MSG_UPGRADE_STAT_ID                 22              // 消息队列：WiFi App Server <-- Firmware Upgrade App

// WiFi App Server --> Hoslam App
int send_msg_to_hoslam_app(int type, const char *data, const int len)
{
    static int  msgid = -1;
    int         ret;
    key_t       msgkey;
    MSG         msgbuf;

    if (len < 0 || len > MSG_MAX_SIZE)
    {
        printf("ERROR len!\n");
        return -1;
    }

    if (msgid == -1)
    {
        // 计算IPC键值
        msgkey = ftok(IPC_KEY_PATH, MSG_HOSLAM_CTRL_ID);

        // 获取消息队列标识符 (0666：存储权限控制符)
        msgid = msgget(msgkey, IPC_CREAT | 0666);
        if (msgid == -1)
        {
            perror("msget");
            return -1;
        }
        //printf("send_msg: key=0x%08x id=%d\n", msgkey, msgid);
    }

    memset(&msgbuf, 0, sizeof(msgbuf));
    msgbuf.mtype = type;
    if (len > 0 && data)
    {
        memcpy(msgbuf.data, data, len);
    }

    // 发送消息
    ret = msgsnd(msgid, &msgbuf, len, IPC_NOWAIT);
    if (ret == -1)
    {
        perror("msgsnd");
        return -1;
    }

    return 0;
}

// WiFi App Server <-- Hoslam App
static void* receive_msg_from_hoslam_app(void *arg)
{
    int     ret;
    key_t   msgkey;
    int     msgid;
    MSG     msgbuf;

    // 计算IPC键值
    msgkey = ftok(IPC_KEY_PATH, MSG_HOSLAM_STAT_ID);

    // 获取消息队列标识符 (0666：存储权限控制符)
    msgid = msgget(msgkey, IPC_CREAT | 0666);
    if (msgid == -1)
    {
        perror("msgget");
        return NULL;
    }
    printf("recv_hoslam_msg: key=0x%08x id=%d\n", msgkey, msgid);
    // 接收消息
    while (1)
    {
        memset(&msgbuf, 0, sizeof(msgbuf));
        ret = msgrcv(msgid, &msgbuf, sizeof(msgbuf.data), 0, 0);
        if (ret == -1)
        {
            perror("msgrcv from hoslam");
            return NULL;
        }

        //printf("hoslam msgrcv: type=%d\n", msgbuf.mtype);
        recv_packet(msgbuf.mtype, (uint8_t*)msgbuf.data);
    }

    return NULL;
}

// WiFi App Server <-- Hoslam App
static void* receive_map_from_hoslam_app(void *arg)
{
    int         shmid, semid;
    int         ret, size, count;
    key_t       shmkey, semkey;
    char        *p = NULL;
    DATA_HEADER header;
    DATA_MAP    *map = NULL;
    uint8_t     *data = NULL;

    // 计算IPC键值
    shmkey = ftok(IPC_KEY_PATH, SHM_HOSLAM_MAP_ID);
    semkey = ftok(IPC_KEY_PATH, SEM_HOSLAM_MAP_ID);

    // 创建信号量集：用于共享内存读写操作的同步
    semid = sys_sem_create(semkey, 2);
    if (semid == -1)
    {
        semid = sys_sem_get(semkey, 2);
        if (semid == -1)
        {
            return NULL;
        }
    }

    // 初始化信号量
    // ret = sys_sem_init(semid, 0, 1);    // 信号量0： p-wait_read v-read_done
    // if (ret == -1)
    // {
    //     return NULL;
    // }
    // ret = sys_sem_init(semid, 1, 0);    // 信号量1： p-wait_write v-write_done
    // if (ret == -1)
    // {
    //     return NULL;
    // }

    // 获取共享内存标识符
    shmid = shmget(shmkey, SHM_HOSLAM_MAP_SIZE, IPC_CREAT | 0666);
    if (shmid == -1)
    {
        perror("shmget");
        return NULL;
    }
    printf("recv_map shm: key=0x%08x id=%d\n", shmkey, shmid);

    // 映射共享内存
    p = (char*)shmat(shmid, 0, 0);
    if (p == (char*)-1)
    {
        perror("shmat");
        return NULL;
    }

    while (1)
    {
        // 等待Hoslam App写入数据
        sys_sem_wait(semid, 1);

        // 从共享内存中读取数据
        memcpy(&header, p, sizeof(DATA_HEADER));
        switch (header.type)
        {
            case TYPE_MAP:
                map = (DATA_MAP*)(p + sizeof(DATA_HEADER));
                //printf("recv_map: w=%d h=%d ox=%f oy=%f resolution=%f dx=%f dy=%f\n", map->w, map->h, map->ox, map->oy, map->resolution, map->dock_x, map->dock_y);
                size = header.size;//sizeof(DATA_MAP) + map->w * map->h;
                data = (uint8_t*)malloc(size);
                if (!data)
                {
                    printf("recv_map: malloc ERROR!\n");
                    sys_sem_signal(semid, 0);
                    continue;
                }
                memcpy(data, p + sizeof(DATA_HEADER), size);
                break;

            case TYPE_POSITION:
                count = header.size / sizeof(DATA_POSITION);
                //printf("recv_position: count = %d\n", count);

                size = header.size + sizeof(DATA_PATH);
                data = (uint8_t*)malloc(size);
                if (!data)
                {
                    printf("recv_position: malloc ERROR!\n");
                    sys_sem_signal(semid, 0);
                    continue;
                }

                DATA_PATH *path = (DATA_PATH*)data;
                path->count = count;
                memcpy(path->position, p + sizeof(DATA_HEADER), header.size);
                break;

#if 0 //APP_DEMO
            case TYPE_CLEAN_LINE:
                size = header.size;
                //printf("recv_lines: size = %d\n", size);
                data = (uint8_t*)malloc(size);
                if (!data)
                {
                    printf("recv_map: malloc ERROR!\n");
                    sys_sem_signal(semid, 0);
                    continue;
                }
                memcpy(data, p + sizeof(DATA_HEADER), size);
                break;
#endif // APP_DEMO

            case TYPE_PATH:
                count = header.size / sizeof(DATA_POSITION);
                //printf("recv_position: count = %d\n", count);

                size = header.size + sizeof(DATA_PATH);
                data = (uint8_t*)malloc(size);
                if (!data)
                {
                    printf("recv_path: malloc ERROR!\n");
                    sys_sem_signal(semid, 0);
                    continue;
                }
                else
                {
                    DATA_PATH *path = (DATA_PATH*)data;
                    path->count = count;
                    memcpy(path->position, p + sizeof(DATA_HEADER), header.size);
                }
                break;
                
            case TYPE_TRAJECTORY_PATH:
                count = (header.size - sizeof(DATA_CURRENT_POSE)) / sizeof(DATA_POSITION_TYPE);
            
                size = header.size + sizeof(DATA_CURR_POSE_AND_PATH);
                data = (uint8_t*)malloc(size);
                if (!data)
                {
                    printf("recv_path: malloc ERROR!\n");
                    sys_sem_signal(semid, 0);
                    continue;
                }
                else
                {
                    DATA_CURR_POSE_AND_PATH *path = (DATA_CURR_POSE_AND_PATH*)data;
                    path->path_point_count = count;
                    memcpy(&path->cur_pose, p + sizeof(DATA_HEADER), sizeof(DATA_CURRENT_POSE));
                    memcpy(path->position, p + sizeof(DATA_HEADER)+sizeof(DATA_CURRENT_POSE), header.size-sizeof(DATA_CURRENT_POSE));
                }
                break;
                
            default:
                break;
        }

        // 释放信号量，通知Hoslam App已读取数据
        sys_sem_signal(semid, 0);

        // 将地图原始数据解析、压缩、编码后发送至YG App
        recv_data(header.type, size, data);
        free(data);
        data = NULL;
        map = NULL;
    }

    // 解除内存映射
    ret = shmdt(p);
    if (ret == -1)
    {
        perror("shmdt");
    }

    return NULL;
}

int rooms_to_hoslam_semid = -1;
char* rooms_to_hoslam_shmadd = NULL;
int rooms_from_hoslam_semid = -1;
char* rooms_from_hoslam_shmadd = NULL;
static const int ROOM_SHM_SIZE = 32 * 1024;

#define ROOMS_KEY_PATH                      "/"        //命名共享内存
#define ROOMS_TO_HOSLAM_SEM_ID              16         //房间信息发送给hoslam 信号量
#define ROOMS_TO_HOSLAM_SHM_ID              17         //房间信息发送给hoslam 共享内存
#define ROOMS_FROM_HOSLAM_SEM_ID            18         //房间接收信息 信号量
#define ROOMS_FROM_HOSLAM_SHM_ID            19         //房间接收信息 共享内存

int init_rooms_shared_memory() {

    key_t semkey_from = ftok(ROOMS_KEY_PATH, ROOMS_TO_HOSLAM_SEM_ID);
    key_t shmkey_from = ftok(ROOMS_KEY_PATH, ROOMS_TO_HOSLAM_SHM_ID );
    key_t semkey_to = ftok(ROOMS_KEY_PATH, ROOMS_FROM_HOSLAM_SEM_ID);
    key_t shmkey_to = ftok(ROOMS_KEY_PATH, ROOMS_FROM_HOSLAM_SHM_ID);

    rooms_to_hoslam_semid = sys_sem_create(semkey_from, 2);
    if (rooms_to_hoslam_semid == -1) {
        rooms_to_hoslam_semid =  sys_sem_get(semkey_from, 2);
        if(rooms_to_hoslam_semid == -1){
            printf("rooms_from_appserver_semid create fail\n");
            return -1;
        }
    }
    rooms_from_hoslam_semid = sys_sem_create(semkey_to, 2);
    if (rooms_from_hoslam_semid == -1) {
        rooms_from_hoslam_semid  = sys_sem_get(semkey_to, 2);
        if(rooms_from_hoslam_semid == -1){
            printf("rooms_from_hoslam_semid create fail\n");
            return -1;
        }
    }
    sys_sem_init(rooms_to_hoslam_semid, 0, 1);
    sys_sem_init(rooms_to_hoslam_semid, 1, 0);
    sys_sem_init(rooms_from_hoslam_semid, 0, 1);
    sys_sem_init(rooms_from_hoslam_semid, 1, 0);

    int shmid_from = shmget(shmkey_from, ROOM_SHM_SIZE, 0666|IPC_CREAT);
    if (shmid_from == -1) {
        printf("shmid_from shmget fail\n"); 
        return -1;
    }
    if ((rooms_to_hoslam_shmadd = (char*)shmat(shmid_from, NULL, 0)) == (char*)-1) {
       printf( "rooms_to_hoslam_shmadd shmat fail\n");
        return -1;
    }

    int shmid_to = shmget(shmkey_to, ROOM_SHM_SIZE, 0666 | IPC_CREAT);
    if (shmid_to == -1) {
        printf( "shmid_to shmget fail\n"); 
        return -1;
    }
    if ((rooms_from_hoslam_shmadd = (char*)shmat(shmid_to, NULL, 0)) == (char*)-1) {
        printf( "rooms_from_hoslam_shmadd shmat fail\n");
        return -1;
    }
    return 0;
}


// WiFi App Server --> Firmware Upgrade App
int send_msg_to_upgrade_app(int type, const char *data, const int len)
{
    static int  msgid = -1;
    int         ret;
    key_t       msgkey;
    MSG         msgbuf;

    if (len < 0 || len > MSG_MAX_SIZE)
    {
        printf("ERROR len!\n");
        return -1;
    }

    if (msgid == -1)
    {
        // 计算IPC键值
        msgkey = ftok(IPC_KEY_PATH, MSG_UPGRADE_CTRL_ID);

        // 获取消息队列标识符 (0666：存储权限控制符)
        msgid = msgget(msgkey, IPC_CREAT | 0666);
        if (msgid == -1)
        {
            perror("msget");
            return -1;
        }
        //printf("send_msg: key=0x%08x id=%d\n", msgkey, msgid);
    }

    memset(&msgbuf, 0, sizeof(msgbuf));
    msgbuf.mtype = type;
    if (len > 0 && data)
    {
        memcpy(msgbuf.data, data, len);
    }

    // 发送消息
    ret = msgsnd(msgid, &msgbuf, sizeof(msgbuf.data), IPC_NOWAIT);
    if (ret == -1)
    {
        perror("msgsnd");
        return -1;
    }

    return 0;
}

// WiFi App Server <-- Firmware Upgrade App
static void* receive_msg_from_upgrade_app(void *arg)
{
    int     ret;
    key_t   msgkey;
    int     msgid;
    MSG     msgbuf;

    // 计算IPC键值
    msgkey = ftok(IPC_KEY_PATH, MSG_UPGRADE_STAT_ID);

    // 获取消息队列标识符 (0666：存储权限控制符)
    msgid = msgget(msgkey, IPC_CREAT | 0666);
    if (msgid == -1)
    {
        perror("msgget");
        return NULL;
    }
    printf("recv_upgrade_msg: key=0x%08x id=%d\n", msgkey, msgid);

    // 接收消息
    while (1)
    {
        memset(&msgbuf, 0, sizeof(msgbuf));
        ret = msgrcv(msgid, &msgbuf, sizeof(msgbuf.data), 0, 0);
        if (ret == -1)
        {
            perror("msgrcv from upgrade");
            return NULL;
        }

        //printf("upgrade msgrcv: type=%d, msg=%s\n", msgbuf.mtype, msgbuf.data);
        wifi_app_firmware_upgrade_msg_parse(msgbuf.mtype, msgbuf.data);
    }

    return NULL;
}

static void *firmware_auto_upgrade(void *arg)
{
    wifi_app_firmware_upgrade_backup_check();
}

// WiFi App Server Debug
#if WIFI_APP_SERVER_DEBUG
static void* wifi_app_server_debug(void *arg)
{
#if 0   // 自动进入配网模式
    sleep(10);
    wifi_app_msg_send(CMD_CONFIG_NETWORK, NULL);
#endif
#if 0
    // Grit升级
    sleep(10);
    //wifi_app_firmware_upgrade("/tmp/firmware.tar.gz", "/tmp/ygclient.tar.gz");
    wifi_app_firmware_upgrade("/tmp/firmware.tar.gz", NULL);
#endif
#if 0
    // 老化测试
    sleep(20);
    robot_start_fullgo();
#endif
#if 1
    int i;
    for (i = 1; i <= 90; i++)
    {
        wifi_app_voice_play(i);
        usleep(200000);
    }
#endif
    return NULL;
}
#endif

int GET_WORKING_ROOMS_FLAG;

int get_working_rooms_flag()
{
    return GET_WORKING_ROOMS_FLAG;
}

void *send_get_rooms(void *arg)	  //获取动态分区房间信息
{
    while(1)
    {
    	if(1)//(1 == GET_WORKING_ROOMS_FLAG) //适配4.5.9以上版本的hoslam 定时四秒获取一次房间信息 
    	{
        	sleep(4);
        	send_msg_to_hoslam_app(MSG_ROOMS_GET_WORKING_ROOMS,NULL,0);       //发送hoslam MSG_ROOMS_SEGMENT
            robot_api_send_msg_to_hoslam(TUYA_MSG_ROOMS_MAP_IS_EDITABLE, NULL, 0);
			printf("MSG_ROOMS_GET_WORKING_ROOMS\n");
    	}
		else
		{
			sleep(1);
		}
    }
}

#define SMART_ROOMS_SWITCH   //定义时带智能分区功能，屏蔽掉则不调智能分区
//pthread_mutex_t map_save_lock;


#if !WIFI_APP_SERVER_FIRMWARE_UPGRADE
int create_process_threads(void)
{
    int ret = 0;
    pthread_t thread_recv_wifi_msg, thread_recv_hoslam_map, thread_recv_hoslam_msg,thread_send_get_rooms;
    pthread_t thread_recv_upgrade_msg;
    pthread_t thread_voice_mgr;
	pthread_t thread_firmware_auto_upgrade;

	pthread_attr_t attr;

	//初始化线程属性
	pthread_attr_init(&attr);

	//设置线程属性为分离状态
    pthread_attr_setdetachstate(&attr , PTHREAD_CREATE_DETACHED);

    printf("wifi app server start ...\n");
    //pthread_mutex_init(&map_save_lock,NULL);

    init_rooms_shared_memory();
    // 创建定时调用房间分区线程
    #ifdef SMART_ROOMS_SWITCH
    ret = pthread_create(&thread_send_get_rooms, NULL,send_get_rooms, NULL);
    if (ret)
    {
        printf("create thread_recv_upgrade_msg ERROR!\n");
        return -1;
    }
    #endif

    // 创建接收升级状态线程
    ret = pthread_create(&thread_recv_upgrade_msg, NULL, receive_msg_from_upgrade_app, NULL);
    if (ret)
    {
        printf("create thread_recv_upgrade_msg ERROR!\n");
        return -1;
    }

    // 创建语音播放线程
    ret = pthread_create(&thread_voice_mgr, NULL, wifi_app_voice_mgr, NULL);
    if (ret)
    {
        printf("create thread_voice_mgr ERROR!\n");
        return -1;
    }

#ifdef APP_DEMO
    // 创建Socket线程与Demo App通信
    ret = pthread_create(&thread_recv_wifi_msg, NULL, wifi_app_tcp_server, NULL);
    if (ret)
    {
        printf("create thread_recv_wifi_msg ERROR!\n");
        return -1;
    }
#endif

    // 创建线程接收Hoslam App上传的地图
    ret = pthread_create(&thread_recv_hoslam_map, NULL, receive_map_from_hoslam_app, NULL);
    if (ret)
    {
        printf("create thread_recv_hoslam_map ERROR!\n");
        return -1;
    }

    // 创建线程接收Hoslam App上报的消息
    ret = pthread_create(&thread_recv_hoslam_msg, NULL, receive_msg_from_hoslam_app, NULL);
    if (ret)
    {
        printf("create thread_recv_hoslam_msg ERROR!\n");
        return -1;
    }
 
	//创建线程升级掉电之后自动重新升级固件
	ret = pthread_create(&thread_firmware_auto_upgrade, &attr, firmware_auto_upgrade, NULL);
	if (ret)
    {
        printf("create thread_firmware_auto_upgrade ERROR!\n");
        return -1;
    }

#if WIFI_APP_SERVER_DEBUG
    pthread_t thread_debug;

    // 创建测试线程
    ret = pthread_create(&thread_debug, NULL, wifi_app_server_debug, NULL);
    if (ret)
    {
        printf("create thread_debug ERROR!\n");
        return -1;
    }
#endif
    return 0;
}
#else
pthread_mutex_t rooms_lock;

int main(int argc, char** argv)
{
    int ret = 0;
	char *firmware_file = NULL;
    pthread_t thread_recv_upgrade_msg;

    pthread_mutex_init(&rooms_lock,NULL);
    init_rooms_shared_memory();
	if (argc < 2)
	{
		printf("Usage: %s firmware.tar.gz\n", basename(argv[0]));
		return 0;
	}

	firmware_file = argv[1];

    ret = wifi_app_firmware_upgrade(firmware_file, NULL);
    if (!ret)
    {
        // 创建接收升级状态线程
        ret = pthread_create(&thread_recv_upgrade_msg, NULL, receive_msg_from_upgrade_app, NULL);
        if (ret)
        {
            printf("create thread_recv_upgrade_msg ERROR!\n");
        }

        pthread_join(thread_recv_upgrade_msg, NULL);
    }

	return 0;
}
#endif

