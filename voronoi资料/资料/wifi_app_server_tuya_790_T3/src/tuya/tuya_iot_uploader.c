#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>
#include <libgen.h>
#include <sys/time.h>
#include <arpa/inet.h>

#include "tuya_iot_com_api.h"
#include "tuya_iot_sweeper_api.h"
#include "tuya_cloud_types.h"
#include "tuya_iot_wifi_api.h"
#include "uni_log.h"
#include "tuya_iot_state_adapter.h"
#include "tuya_iot_uploader.h"
#include "tuya_dp_enum.h"
#include "sys_api.h"
#include "robot_control_adapter.h"
#include "tuya_iot_msg_parse.h"
#include "../include/sys_api.h"
#include "lz4.h"
#include "robot_map_list.h"



#define OLD_PATH_SEND_MODE
extern int8_t getmapflag;

//缓存的地图数据
static CHAR_T* path_send_buf = NULL;
static int32_t path_send_len = 0;
//缓存的地图路径
static CHAR_T* map_send_buf = NULL;
static int32_t map_send_len = 0;
static int32_t map_send_head = 0;

extern bool s_send_his;

//发送给云端标识位
int send_cloud_flag;
char time_save[32];

static char map_file[128];

static int32_t chk_last_path_size = 0;
static int32_t chk_last_map_size = 0;
extern bool upload_cur_pose_flag;


//地图、路径，配网互斥锁
pthread_mutex_t map_path_ptx;
pthread_mutex_t path_ptx;
pthread_mutex_t wifi_connect_ptx;
int init_all_mutex(void)
{

        // 初始化互斥量
    int ret;
    ret = pthread_mutex_init(&map_path_ptx, NULL);
    if (ret)
    {
        printf("init map_path_ptx error\n");
        return -1;
    }
    ret = pthread_mutex_init(&wifi_connect_ptx, NULL);
    if (ret)
    {
        printf("init wifi_connect_ptx error\n");
        return -1;
    }

}


#define BigLittleSwap16(A)  ((((uint16_t)(A) & 0xff00) >> 8) | (((uint16_t)(A) & 0x00ff) << 8))

static int littleEnd_to_bigEnd_short(uint16_t *pBig_s, uint16_t *pLittle_s, int count)
{
    /* 注意 pLittle_s 和 pBig_s 不要越界(个数大于count) */
    for(int i = 0; i<count; i++)
    {
        pBig_s[i] = BigLittleSwap16(pLittle_s[i]);
    }
    return 0;
}

char Hex2Char(uint8_t vdata) {
    char cRes;
    if (vdata >= 0x00 && vdata <= 0x09)
    {
        cRes = '0' + vdata;
    }
    if (vdata >= 0x0a && vdata <= 0x0f)
    {
        cRes = 'a' + vdata - 10;
    }

    return cRes;
}

static void convert_HEX2CHAR(CHAR_T *out_char, uint8_t *in_hex, int cbytes) {
    uint8_t umemd;
    for (int i = 0; i < cbytes; i++) {
        int j = i % 2 == 0 ? i + 1 : i - 1;
        umemd = *(in_hex + i);
        out_char[2 * j] = Hex2Char(umemd >> 4);
        out_char[2 * j + 1] = Hex2Char(umemd & 0x0F);
    }
}

int tuya_iot_upload_navigation(SEND_PATH *send_path, int total_bytes)
{
    static uint8_t first_start_send_path;
    uint32_t nav_length = 0;
    uint8_t * nav_send_path = NULL;
    if((send_path == NULL)||(total_bytes <= sizeof(SEND_PATH))){
        PR_ERR("NULL");
        return -1;
    }

 // 路径 字节顺序调整
    int t_path_len =  send_path->num * 4;
    uint16_t* t_path_byte = (uint16_t*)malloc(t_path_len);
    if(t_path_byte == NULL){
        printf("Err Malloc - t_path_byte\n");
        return -1;
    }

    uint16_t tmp_d = 0;
     
    for(int i = 0; i < send_path->num; i++)
    {
        tmp_d = send_path->path[i].x;
        t_path_byte[2*i] = htons(tmp_d);
        tmp_d = send_path->path[i].y;
        t_path_byte[2*i+1] = htons(tmp_d);
    }

    // 路径LZ4压缩
    int t_path_byte_len = t_path_len;
    uint8_t* path_lz4buff = (uint8_t*)malloc(t_path_byte_len);
    if(path_lz4buff == NULL)
    {
        printf("Err Malloc - path_lz4buff\n");
        free(t_path_byte);
        return -1;
    }

    int path_lz4len = LZ4_compress_default((char*)t_path_byte, (char*)path_lz4buff, t_path_byte_len, t_path_byte_len);
    send_path->lz4len = (uint16_t)path_lz4len;

    printf("origin nav path len = %d, LZ4 compressed len = %d\n\n", t_path_byte_len, path_lz4len);

    // 压缩失败（压缩后长度更多）
    if(path_lz4len == 0)
    {
        memcpy(path_lz4buff, t_path_byte, t_path_byte_len);
        path_lz4len = t_path_byte_len;
        printf("Path data lz4compress fail, use origin byte\n\n");
    }
    
    // 拼接路径头部
    nav_length = sizeof(SEND_PATH) + path_lz4len; /* 路径属性 + 路径点数据 */
    nav_send_path = (uint8_t*)malloc(nav_length);
    if(nav_send_path == NULL){
        PR_ERR("Err Malloc - nav_send_path");
        free(path_lz4buff);
        free(t_path_byte);
        nav_length = 0;
        return -1;
    }

    send_path->head = htons(send_path->head);
    send_path->num = htonl(send_path->num);
    send_path->lz4len = htons(send_path->lz4len);
    send_path->direction = htons(send_path->direction);
    if(first_start_send_path <= 5)
    {
       printf("first_start_send_path = %d\n",first_start_send_path);
       send_path->init_flag = 0x01;
    }
    else
    {
        send_path->init_flag = 0x00;
    }
   
    memcpy(nav_send_path, (uint8_t*)send_path, sizeof(SEND_PATH));
    memcpy(nav_send_path + sizeof(SEND_PATH), path_lz4buff, path_lz4len);
    free(path_lz4buff);
    free(t_path_byte);
    printf("sended navi: %d\n",total_bytes);
     if (iot_state_cloud_ready()) {

        struct timeval tv1;
        gettimeofday(&tv1, NULL);
        int before = tv1.tv_sec*1000 + tv1.tv_usec/1000;

        OPERATE_RET op_ret = tuya_iot_upload_navigation_buffer(send_path->head, nav_send_path, nav_length);
        if(op_ret == OPRT_OK)
        {
            if(first_start_send_path <= 5)
            {
               first_start_send_path ++;
            }       
        }

        struct timeval tv2;
        gettimeofday(&tv2,NULL);
        int after = tv2.tv_sec*1000 + tv2.tv_usec/1000;

        PR_ERR("upload navi size %d , used time - %d ms op_ret:%d", nav_length, (after - before),op_ret);
    }
    free(nav_send_path);

    return 0;
}

pthread_mutex_t mtx_history_record;

int init_history_record_lock(void)
{
    // 初始化互斥量
    int ret;
    ret = pthread_mutex_init(&mtx_history_record, NULL);
    if (ret)
    {
        printf("init mtx_history_record error\n");
        return -1;
    }
}


void force_record_map_path(void)
{
    CHAR_T* record_send_buf = NULL;
    int32_t record_send_len = 0;
    char filename[128];
    int mutex_ret = 0;
    int ret;
    
    //static int record_id = 1;
    if(NULL == path_send_buf || NULL == map_send_buf)
    {
          printf("map_send_len = %p,map_send_buf = %p\n",map_send_len,map_send_buf);
          printf("path_send_len = %d,path_send_buf = %p\n",path_send_len,path_send_buf);
          return ;
    }
    mutex_ret = pthread_mutex_trylock(&mtx_history_record); 
    if(0 != mutex_ret)
    {
        printf(" *** last mtx_history_record no end, not need repeat lock %d*** \r\n",mutex_ret);
        pthread_mutex_unlock(&mtx_history_record);
        return;
    }
    
    record_send_len = map_send_len + path_send_len;
    record_send_buf = (CHAR_T *)malloc(record_send_len);
    if(record_send_buf == NULL) {
        printf("send history:malloc %d failed\n", record_send_len);
        pthread_mutex_unlock(&mtx_history_record);
        return;
    }
    memset(record_send_buf, 0, record_send_len);

    memcpy(record_send_buf, map_send_buf, map_send_len);
    memcpy(record_send_buf + map_send_len, path_send_buf, path_send_len);

    DATA_CLEAN_STATIS *last_clean_statis = iot_get_last_clean_statis();
    time_t nowt = time(0);
    time_t begin = nowt - last_clean_statis->clean_time * 60;
    char tmpnow[32];
    char tmpbegin[32];
    //strftime(tmpnow, sizeof(tmpnow), "%F %T", localtime(&nowt));
    //printf("time is %s\n", tmpnow);  
    
    strftime(tmpbegin, sizeof(tmpbegin), "%Y%m%d_%H%M%S", localtime(&begin));
    strftime(tmpnow, sizeof(tmpnow), "%H%M%S", localtime(&nowt));
    
    sprintf(filename, "%s_%s_%d_%d_%d_%d.txt", tmpbegin, tmpnow,
        last_clean_statis->clean_area,last_clean_statis->clean_time,map_send_len,path_send_len);

    bool is_cloud_ok = iot_state_cloud_ready();
    printf("* 2 tuya_iot_map_record_upload_buffer start %d\r\n", is_cloud_ok);
    if(is_cloud_ok)
    {
        OPERATE_RET op_ret = tuya_iot_map_record_upload_buffer(1, record_send_buf, record_send_len, filename);
        
        printf("---2 upload record, %s, return:%d\n", filename, op_ret);
    }
    else
    {
        char record_file[128] = {0};
        sprintf(record_file, "%s/%s", APPSERVER_CONFIG_DIR, filename);
        printf("2 record and cloud is not ok %s\r\n", record_file);
        write_appserver_config_record(record_file);
        write_raw_data(record_file, record_send_buf, record_send_len);
    }
   
    free(record_send_buf);
    pthread_mutex_unlock(&mtx_history_record);

}


static void send_history_record_if_need() {
    #if 1
    CHAR_T* record_send_buf = NULL;
    int32_t record_send_len = 0;
    char filename[128];
    int mutex_ret = 0;
    int ret;
    static uint32_t history_record_id;
	static char uplod_clean_record_flag = 0;
    // if (!is_job_done())
    // {
    //     //流程未完成，直接返回
    //     return;
    // }
    
    printf("* cmw * mtx_map_record null\r\n");
    if(s_send_his)
    {
    	uplod_clean_record_flag++;
		printf("uplod_clean_record_flag:%d \n",uplod_clean_record_flag);
		if(uplod_clean_record_flag < 2)
		{
			return;
		}
       if(NULL == path_send_buf || NULL == map_send_buf)
       {
                printf("map_send_len = %p,map_send_buf = %p\n",map_send_len,map_send_buf);
                printf("path_send_len = %d,path_send_buf = %p\n",path_send_len,path_send_buf);
                return ;
       }
             
        printf("s_send_his\n");
        DATA_CLEAN_STATIS *last_clean_statis = iot_get_last_clean_statis();
        DATA_CLEAN_STATIS *remember_clean_statis = iot_get_clean_remember_statis();
        if((last_clean_statis->clean_time <= 1) && (remember_clean_statis->clean_time > 1))
        {
            printf("--- turn new mode, use remember clean time =%d\r\n", remember_clean_statis->clean_time);
            //memcpy(&last_clean_statis, &last_data_clean_sta_remember, sizeof(DATA_CLEAN_STATIS));// add for test bug
            //last_clean_statis = &last_data_clean_sta_remember;
            last_clean_statis = remember_clean_statis;
        }

        if (last_clean_statis->clean_time > 0 && last_clean_statis->clean_area > 0) //清扫时长大于1分钟才上传
        {
            //static int record_id = 1;
            printf("s_send_his 1\n");
            BYTE_T *vir_data = NULL;
            BYTE_T *ford_data = NULL;
            int vir_lenth,forbidden_lenth;

            vir_lenth = get_virtual_wall(&vir_data);  /*获取虚拟墙禁区数据 */
            forbidden_lenth = get_forbidden_zone_new(&ford_data);

            mutex_ret = pthread_mutex_trylock(&mtx_history_record); 
            if(0 != mutex_ret)
            {
                printf(" *** last mtx_history_record no end, not need repeat lock %d*** \r\n",mutex_ret);
                free(vir_data);
                free(ford_data);
                return;
            }
            

            record_send_len = map_send_len + path_send_len + vir_lenth +forbidden_lenth;

            record_send_buf = (CHAR_T *)malloc(record_send_len);
            if(record_send_buf == NULL) {
                printf("send history:malloc %d failed\n", record_send_len);
                pthread_mutex_unlock(&mtx_history_record);
                free(vir_data);
                free(ford_data);
                return;
            }

            printf("path_send_len = %d,path_send_buf = %p\n",path_send_len,path_send_buf);
            printf("map_send_len = %d,vir_lenth = %d,forbidden_lenth = %d,record_send_len = %d\n",map_send_len,vir_lenth,forbidden_lenth,record_send_len);
            printf("record_send_len = %d\n",record_send_len);

            memset(record_send_buf, 0, record_send_len);           
            memcpy(record_send_buf, map_send_buf, map_send_len);
            memcpy(record_send_buf + map_send_len, path_send_buf, path_send_len);
            memcpy(record_send_buf + map_send_len + path_send_len, vir_data, vir_lenth);
            memcpy(record_send_buf + map_send_len + path_send_len + vir_lenth, ford_data, forbidden_lenth);


            time_t nowt = time(0);
            time_t begin = nowt - last_clean_statis->clean_time * 60;
            char tmpnow[32];
            char tmpbegin[32];
            //strftime(tmpnow, sizeof(tmpnow), "%F %T", localtime(&nowt));
            //printf("time is %s\n", tmpnow);  
            
            strftime(tmpbegin, sizeof(tmpbegin), "%Y%m%d_%H%M%S", localtime(&begin));
            strftime(tmpnow, sizeof(tmpnow), "%Y%m%d_%H%M%S", localtime(&nowt));
            printf("time is %s\n", tmpnow); 
            
            sprintf(filename, "%d_%s_%d_%d_%d_%d_%d.txt",history_record_id ++, tmpnow,
               last_clean_statis->clean_time,last_clean_statis->clean_area,map_send_len,path_send_len,(vir_lenth +forbidden_lenth));

            bool is_cloud_ok = iot_state_cloud_ready();
            printf("* tuya_iot_map_record_upload_buffer start %d\r\n", is_cloud_ok);
            if(is_cloud_ok)
            {
                //remember_record = 0;
                OPERATE_RET op_ret = tuya_iot_map_record_upload_buffer(1, record_send_buf, record_send_len, filename);
                
                printf("---upload record, %s, return:%d\n", filename, op_ret);
            }
            else
            {
                //memcpy(record_filename, filename, sizeof(filename));
                //remember_record = 1;
                char dir_file[128] = {0};
                sprintf(dir_file, "%s/%s", APPSERVER_CONFIG_DIR, filename);
                printf("record and cloud is not ok %s\r\n", dir_file);
                write_appserver_config_record(dir_file);
                write_raw_data(dir_file, record_send_buf, record_send_len);
            }

            //record_id++;
            s_send_his = false;
			uplod_clean_record_flag = 0;
            printf("s_send_his2\n");
            free(record_send_buf);
            free(vir_data);
            free(ford_data);
            pthread_mutex_unlock(&mtx_history_record);
            send_history_record_unmark();
            printf("s_send_his3\n");

        }
        else
        {
            s_send_his = false;
			uplod_clean_record_flag = 0;
        }
    }
    #endif

}

static void wifi_connect_send_history_record(void)
{
    char file[128];
    int ret;
    char *record_save_buf = NULL;
    int32_t record_save_len = 0;
    char *file_basename = NULL;

    int mutex_ret = pthread_mutex_trylock(&mtx_history_record); 
    if(0 != mutex_ret)
    {
        printf(" *** 2 last mtx_history_record no end %d *** \r\n",mutex_ret);
        return;
    }
    ret = read_appserver_config_record(file, sizeof(file));
    if(ret == -1)
    {
        pthread_mutex_unlock(&mtx_history_record);
        return;
    }
    ret = read_raw_data(file, &record_save_buf, &record_save_len);
    if(ret == -1)
    {
        pthread_mutex_unlock(&mtx_history_record);
        return;
    }
    file_basename = basename(file);
    OPERATE_RET op_ret1 = tuya_iot_map_record_upload_buffer(1, record_save_buf, record_save_len, file_basename);
    printf("--- remember upload record, %s, return:%d\n", file_basename, op_ret1);
    free_raw_data(record_save_buf);

    memset(file, 0, sizeof(file));
    //sprintf(file_dirname, "%s/*.txt", dirname(file));
    sprintf(file, "%s/*.txt", APPSERVER_CONFIG_DIR);
    delete_raw_file(file);
    pthread_mutex_unlock(&mtx_history_record);
    return;
}

int tuya_iot_upload_path(SEND_PATH *send_path, int total_bytes) 
{
    static uint8_t first_start_send_path = 0;

    if((send_path == NULL)||(total_bytes <= sizeof(SEND_PATH))){
        PR_ERR("NULL");
        return -1;
    }

    send_history_record_if_need();

    // 判断路径是否变化
    if(chk_last_path_size == total_bytes)
    {
        PR_DEBUG("Recv Path Data, Equal Last\n");
        return 1;
    }

    pthread_mutex_lock(&map_path_ptx);
    if(path_send_buf != NULL) {
        free(path_send_buf);
        path_send_buf = NULL;
        path_send_len = 0;
    }
 
    
#ifdef OLD_PATH_SEND_MODE


 // 路径 字节顺序调整
    int t_path_len =  send_path->num * 4;
    uint16_t* t_path_byte = (uint16_t*)malloc(t_path_len);
    if(t_path_byte == NULL){
        printf("Err Malloc - t_path_byte\n");
         pthread_mutex_unlock(&map_path_ptx);
        return -1;
    }

    uint16_t tmp_d = 0;
     
    for(int i = 0; i < send_path->num; i++)
    {
        tmp_d = send_path->path[i].x;
        t_path_byte[2*i] = htons(tmp_d);
        tmp_d = send_path->path[i].y;
        t_path_byte[2*i+1] = htons(tmp_d);
    }

    // 路径LZ4压缩
    int t_path_byte_len = t_path_len;
    uint8_t* path_lz4buff = (uint8_t*)malloc(t_path_byte_len);
    if(path_lz4buff == NULL)
    {
        printf("Err Malloc - path_lz4buff\n");
        free(t_path_byte);
        pthread_mutex_unlock(&map_path_ptx);
        return -1;
    }

    int path_lz4len = LZ4_compress_default((char*)t_path_byte, (char*)path_lz4buff, t_path_byte_len, t_path_byte_len);
    //int path_lz4len = 0;
    send_path->lz4len = (uint16_t)path_lz4len;

    printf("origin path len = %d, LZ4 compressed len = %d\n\n", t_path_byte_len, path_lz4len);

    // 压缩失败（压缩后长度更多）
    if(path_lz4len == 0)
    {
        memcpy(path_lz4buff, t_path_byte, t_path_byte_len);
        path_lz4len = t_path_byte_len;
        printf("Path data lz4compress fail, use origin byte\n\n");
    }
    
    // 拼接路径头部
    path_send_len = sizeof(SEND_PATH) + path_lz4len; /* 路径属性 + 路径点数据 */
    path_send_buf = (uint8_t*)malloc(path_send_len);
    if(path_send_buf == NULL){
        PR_ERR("Err Malloc - path_send_buf");
        free(path_lz4buff);
        free(t_path_byte);
        path_send_len = 0;
         pthread_mutex_unlock(&map_path_ptx);
        return -1;
    }

    send_path->head = htons(send_path->head);
    send_path->num = htonl(send_path->num);
    send_path->lz4len = htons(send_path->lz4len);
    send_path->direction = htons(send_path->direction);
    if(first_start_send_path <= 5)
    {
       printf("first_start_send_path = %d\n",first_start_send_path);
       send_path->init_flag = 0x01;
    }
    else
    {
        send_path->init_flag = 0x00;
    }
   
    memcpy(path_send_buf, (uint8_t*)send_path, sizeof(SEND_PATH));
    memcpy(path_send_buf + sizeof(SEND_PATH), path_lz4buff, path_lz4len);
    pthread_mutex_unlock(&map_path_ptx);
    free(path_lz4buff);
    free(t_path_byte);

#else
    path_send_len = total_bytes;
    path_send_buf = (CHAR_T *)malloc(path_send_len);
    if(path_send_buf == NULL) {
        printf("upload path malloc %d failed\n", path_send_len);
        return -1;
    }
    //memcpy(path_send_buf, send_path, path_send_len);
    littleEnd_to_bigEnd_short(path_send_buf, send_path, (path_send_len/sizeof(uint16_t)));
#endif

    PR_DEBUG("** tuya_iot_upload_path size = %d, getmapflag = %d\r\n", path_send_len, getmapflag);
    printf("send_path->type = 0x%x\n",send_path->type);
	
	if(get_reset_map_flag())
	{
		set_reset_map_flag(false);
		//重置地图后过滤2s内的第一次路径包括当前点
		printf("\033[1m\033[40;33m Filter path within 2 seconds after resetting the map!!!\033[0m\n");
		chk_last_path_size = total_bytes;
		return 0;
	}
	
     if ((getmapflag == COMMFLAG_INMAP) && (iot_state_cloud_ready())) 
    {

        struct timeval tv1;
         OPERATE_RET op_ret = -1;
        gettimeofday(&tv1, NULL);
        int before = tv1.tv_sec*1000 + tv1.tv_usec/1000;
        
        switch(send_path->type)
        {
            case 0x02:
                op_ret = tuya_iot_upload_route_buffer(send_path->head, path_send_buf, path_send_len);
                break;

            case 0x03:
                op_ret = tuya_iot_upload_navigation_buffer(send_path->head, path_send_buf, path_send_len);
                break;  

            default:
                break;
        } 

        struct timeval tv2;
        gettimeofday(&tv2,NULL);
        int after = tv2.tv_sec*1000 + tv2.tv_usec/1000;

        printf("upload path size %d , used time - %d ms, op_ret:%d \n", path_send_len, (after - before),op_ret);

        // 上报成功，更新判断临界
        if(op_ret == OPRT_OK)
        {
            if(first_start_send_path <= 5)
            {
                first_start_send_path ++;
            }
            chk_last_path_size = total_bytes;
        }

    }

    return 0;
}

int send_rooms_size_compress;
int set_send_rooms_size_compress(int value)
{
    send_rooms_size_compress = value;
}

/*****************************************************************
 * @Function: tuya_iot_upload_map
 * @Description: 增加一些安全校验
 * @Param[in]: Do not edit
 * @Return: void
 *****************************************************************/
int tuya_iot_upload_map(SEND_MAP *send_map, int total_bytes) {

    if((send_map == NULL)||(total_bytes <= sizeof(SEND_MAP))){
        PR_ERR("NULL");
        return -1;
    }
	send_history_record_if_need();
    // 判断地图是否变化
    // if(chk_last_map_size == total_bytes)
    // {
    //     PR_DEBUG("Recv Map Data, Equal Last\n");
    //     return 1;
    // }
    
    uint8_t utmp;
    uint16_t udata;
    uint8_t upixd;
    uint16_t szdata[8] = {send_map->head, send_map->flag, send_map->w, send_map->h, \
                          send_map->ox, send_map->oy,send_map->charger_x,send_map->charger_y};
    pthread_mutex_lock(&map_path_ptx);
    if(map_send_buf != NULL) {
        free(map_send_buf);
        map_send_buf = NULL;
    }
    
#ifdef OLD_PATH_SEND_MODE

    int t_map_len = total_bytes - sizeof(SEND_MAP);

    // 地图LZ4压缩
    uint8_t* lz4buff = (uint8_t*)malloc(t_map_len);
    if(lz4buff == NULL){
        PR_ERR("Err Malloc - lz4buff");
         pthread_mutex_unlock(&map_path_ptx);
        return -1;
    }
    memset(lz4buff,0,t_map_len);
    int lz4len = LZ4_compress_default((char*)send_map->map, (char*)lz4buff, t_map_len, t_map_len);
    //int lz4len = 0;
    send_map->lz4len = lz4len;
    printf("origin map len = %d, LZ4 compressed len = %d\n\n", t_map_len, lz4len);
  
    if(lz4len == 0){
        memcpy(lz4buff, send_map->map, t_map_len);
        lz4len = t_map_len;
        printf("Map data lz4compress fail, use origin byte\n\n");
    }

    // 拼接地图头部
    map_send_len = sizeof(SEND_MAP) + lz4len; 
    map_send_buf = (uint8_t*)malloc(map_send_len);
    if(map_send_buf == NULL){
        PR_ERR("Err Malloc - map_send_buf");
        free(lz4buff);
         pthread_mutex_unlock(&map_path_ptx);
        return -1;
    }
    memset(map_send_buf,0,map_send_len);

    send_map->head = htons(send_map->head);
    send_map->w = htons(send_map->w);
    send_map->h = htons(send_map->h);
    send_map->ox = htons(send_map->ox);
    send_map->oy = htons(send_map->oy);
    send_map->resolution = htons(send_map->resolution);
    send_map->charger_x = htons(send_map->charger_x);
    send_map->charger_y = htons(send_map->charger_y);
    send_map->len_before_lz4 = htonl(send_map->len_before_lz4);
    send_map->lz4len = htons(send_map->lz4len);

    memcpy(map_send_buf, send_map, sizeof(SEND_MAP));
    memcpy(map_send_buf+sizeof(SEND_MAP), lz4buff, lz4len);
    pthread_mutex_unlock(&map_path_ptx);
    
    free(lz4buff);

#else
    map_send_len = total_bytes;
    map_send_buf = (CHAR_T *)malloc(map_send_len);
    if(map_send_buf == NULL) {
        printf("upload map malloc %d failed\n", map_send_len);
        return -1;
    }
    memset(map_send_buf, 0, map_send_len);
    //memcpy(map_send_buf, szdata, sizeof(szdata));
    littleEnd_to_bigEnd_short(map_send_buf, szdata, (sizeof(szdata)/sizeof(uint16_t)));
    
    int iBgn = sizeof(SEND_MAP);
    int send_map_size = total_bytes - sizeof(SEND_MAP);
    memcpy(map_send_buf+sizeof(szdata), send_map->map, send_map_size);
    printf("no presess\n");
#endif

    // PR_DEBUG("Check net status map\n");

    // sys_shell("ping www.baidu.com -c 1");

    PR_DEBUG("** tuya_iot_upload_map, size = %d, cur getmapflag = %d\r\n", map_send_len, getmapflag);


	if(get_reset_map_flag())
	{
		printf("\033[1m\033[40;33m Filter the first map within 2 seconds after resetting the map!!!\033[0m\n");
		return 0;
	}

    if ((getmapflag == COMMFLAG_INMAP) && (iot_state_cloud_ready())) 
    {
   
        struct timeval tv1;
        gettimeofday(&tv1, NULL);
        int before = tv1.tv_sec*1000 + tv1.tv_usec/1000;

        //OPERATE_RET op_ret = tuya_iot_upload_total_data(send_map->head, map_send_buf, map_send_len, 1);
        OPERATE_RET op_ret = tuya_iot_upload_layout_buffer(send_map->head, map_send_buf, map_send_len);

        struct timeval tv2;
        gettimeofday(&tv2,NULL);
        int after = tv2.tv_sec*1000 + tv2.tv_usec/1000;

        printf("upload map size %d , used time - %d ms op_ret:%d \n", map_send_len, (after - before),op_ret);

        // 上报成功，更新判断临界
        if(op_ret == OPRT_OK)
        {
        	upload_cur_pose_flag = true;
            chk_last_map_size = total_bytes;
        }

        upload_first_wirtual_wall_info();
    }
	
    return 0;
}



void get_history_record_map(void)
{
    wifi_connect_send_history_record();
}

void set_send_cloud_flag(int len)
{
    send_cloud_flag = len;
    printf("send_cloud_flag = %d\n",send_cloud_flag);
}

/* 获取涂鸦地图列表，并更新本地地图列表 */
void get_tuya_map_info_update_local_map_list(void)
{
    M_MAP_INFO map_info[MAX_M_MAP_INFO_NUM] = {0};
    MAP_ID_PACK *p_map_pack = NULL;
    uint8_t temp = 0;
    int i = 0,j = 0;
    uint8_t len = MAX_M_MAP_INFO_NUM;

    int op_ret =tuya_iot_get_all_maps_info(map_info,&len);

    if(OPRT_OK != op_ret)
    {
        PR_ERR("tuya_iot_get_all_maps_info err op_ret = %d\n", op_ret);
        return;
    }

    for(i = 0; i < len; i ++)
    {
        printf("i = %d,map id: %d\n",i,map_info[i].map_id);
    }

    delete_node_by_tuya_map_list(map_info,len,g_p_map_head);

}

void *save_map_to_cloud(void *data)
{
   char file_map_name[256];
   char file_bin_name[256];
   FILE *fp;
   int length;
   int vir_lenth;
   int forbidden_lenth;
   char file[32];

   printf("---save_map_to_cloud begin = %d\n",send_cloud_flag);

   if(0 != send_cloud_flag)
   {     
        //发送bin文件
        BYTE_T *pbuf_bin = NULL;
        BYTE_T *vir_data = NULL;
        BYTE_T *ford_data = NULL;
        int op_ret;
        // 压缩map文件
        pthread_mutex_lock(&map_path_ptx);
        sys_shell("mv  %s /tmp/hoslam_map", map_file);
        sys_shell("tar -c -C /tmp -f - hoslam_map | gzip - > /tmp/map.tar.gz");
        strcpy(map_file, "/tmp/map.tar.gz");
        sys_shell("rm -rf /tmp/hoslam_map");
        printf("%s\n",map_file);

        vir_lenth = get_virtual_wall(&vir_data);
        forbidden_lenth = get_forbidden_zone_new(&ford_data);
        
        long len =  map_send_len + vir_lenth + forbidden_lenth;
        
        pbuf_bin =  (CHAR_T *)malloc(len);
        if(pbuf_bin == NULL) {
            printf("send history:malloc %d failed\n", len);
            pthread_mutex_unlock(&map_path_ptx);
            free(vir_data);
            free(ford_data);
            return NULL;
        }
        memset(pbuf_bin, 0, len);
        
        memcpy(pbuf_bin, map_send_buf,  map_send_len);
        memcpy(pbuf_bin +  map_send_len,vir_data,vir_lenth);   
        memcpy(pbuf_bin +  map_send_len + vir_lenth,ford_data,forbidden_lenth);
        printf("map_len = %d,vir_lenth = %d,forbidden_lenth = %d\n",map_send_len,vir_lenth,forbidden_lenth);

        char map_bin_file_path[128];
        sprintf(map_bin_file_path,"/tmp/map_bin");
     
        FILE *fbp = fopen("/tmp/map_bin","w+");
        if(NULL == fbp)
        {
             printf("fbp fopen error \n");
             free(vir_data);
             free(ford_data);
             free(pbuf_bin);
             pthread_mutex_unlock(&map_path_ptx);
             return NULL;  
        }
        
        fwrite(pbuf_bin,len,1,fbp);
        fclose(fbp);
        
        printf("vir_lenth = %d,forbidden_lenth = %d,robot_map_id = %d\n",vir_lenth,forbidden_lenth,map_id_pack.robot_map_id);
        
        sprintf(file_bin_name, "pmmm_bin_%d_%s.txt:%d:%s", map_id_pack.robot_map_id, time_save,map_id_pack.robot_map_id,time_save);

        get_tuya_map_info_update_local_map_list();

        map_id_pack.tuya_map_id = get_tuya_map_id(g_p_map_head, map_id_pack.robot_map_id);

	    if(map_id_pack.tuya_map_id == -1) /* 若查询不到，则代表要保存新的地图 */
        {
            printf("* cmw * tuya_iot_map_save_to_cloud start\r\n");
       
            op_ret = tuya_iot_map_upload_files(map_bin_file_path,map_file,file_bin_name,&map_id_pack.tuya_map_id); /*保存地图，同时获取云端map id*/
            
            PR_ERR("upload map file tuya_iot_map_upload_files op_ret:%d mapid:%d", op_ret,map_id_pack.tuya_map_id);
            if (OPRT_OK != op_ret)
            {
                send_map_management(0x2b,0); //保存失败
            }
            else
            {
                while(get_map_list_num(g_p_map_head) >= MAP_LIST_MAX_LEN) 
                {
                  delete_node(g_p_map_head->pnext);   
                }
                add_map_list_node(g_p_map_head,map_id_pack.robot_map_id,map_id_pack.tuya_map_id); /* 保存地图成功，则增加新地图节点 */
                save_robot_map_to_file(g_p_map_head); /* 将链表保存到文件 */
                send_map_management(0x2b,1);
            }
        }
        else   /* 若查询到了保存的地图，则对本次地图做更新处理 */
        {
            printf("* cmw * tuya_iot_map_record_upload_buffer start\r\n");

            op_ret = tuya_iot_map_update_files(map_id_pack.tuya_map_id,map_bin_file_path,map_file);
            
            PR_ERR("update map file tuya_iot_map_updata_files op_ret:%d mapid:%d", op_ret,map_id_pack.tuya_map_id);
            if (OPRT_OK != op_ret)
            {
                send_map_management(0x2b,0); //更新失败
            }
            else
            {
                send_map_management(0x2b,1);
            }
        }
        pthread_mutex_unlock(&map_path_ptx);
        printf("---upload record, %s, return:%d\n",file_bin_name, op_ret);
        
        free(pbuf_bin);
        free(vir_data);
        free(ford_data);
        sys_shell("rm -rf /tmp/map.tar.gz");
        sys_shell("rm -rf /tmp/map_bin");
        set_send_cloud_flag(0);

    }
}

int creat_pthread_save_map_to_cloud(uint8_t *data)
{
   pthread_t thread_send_cloud;
   pthread_attr_t attr;
   pthread_attr_init(&attr);
   pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
   int ret;

   map_id_pack.robot_map_id = *((uint32_t*)data);  /*获取新的map id*/
   printf("map id = %d\n",map_id_pack.robot_map_id);

   strncpy(map_file, data + 4, sizeof(map_file));
   printf("cloud map_file = %s\n",map_file);

   #if 1
   ret = pthread_create(&thread_send_cloud, &attr,save_map_to_cloud,(void *)data);
   if (ret)
   {
        printf("create thread_recv_upgrade_msg ERROR!\n");
        return -1;
   }
   printf("create thread_recv_upgrade_msg success!\n");
   #endif

}        




