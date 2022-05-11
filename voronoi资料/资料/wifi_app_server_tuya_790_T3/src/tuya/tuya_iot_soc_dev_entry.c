#include <unistd.h>
#include <pthread.h>

#include <stdio.h>

#include<stdlib.h>
#include<signal.h>
#include<string.h>
#include<execinfo.h>
#include <libgen.h>
#include "tuya_cloud_types.h"
#include "tuya_cloud_error_code.h"
#include "tuya_cloud_com_defs.h"
#include "tuya_iot_com_api.h"
#include "tuya_iot_sweeper_api.h"
#include "uni_log.h"

#include "tuya_cloud_wifi_defs.h"
#include "tuya_iot_wifi_api.h"
#include "tuya_iot_state_adapter.h"
#include "tuya_iot_msg_parse.h"
#include "tuya_uuid.h"
#include "tuya_hal_wifi.h"
#include "sys_api.h"
#include "sys_timer.h"
#include "robot_control_adapter.h"
#include "robot_app_msg.h"
#include "robot_sys_msg.h"
#include "tuya_iot_uploader.h"
#include "../../robot_api_src/robot_api_inner.h"
#include "robot_map_list.h"

// UserTODO
// SOC固件版本，用于OTA管理，格式必须为"XX.XX.XX"，其中XX必须为数字

#define APP_SERVER_VER         "T3_2.3.2_20211113_790_yunmi"  //智能分区、多楼层基础版本
// 涂鸦云上的产品KEY，需登陆tuya.com创建产品分配唯一key
#define PRODUCT_KEY         "8lw1fml6uycujtzc"
//#define PRODUCT_KEY         "j3exvamdamqhafgq"
// SD本地配置存储路径，该路径必须对应一个可读写文件系统分区
#define CFG_STORAGE_PATH    "./"
// UUID和AUTHKEY用于涂鸦云设备的安全认证，每个设备所用key均为唯一
// #define UUID                "tuya049fe5ce87f5614c"
// #define AUTHKEY             "Md13PqBKnfQ98EELickM7Kn1txRPU56Y"
// OTA文件本地保存绝对路径，SDK下载后保存为该文件
#define SOC_OTA_FILE        "/tmp/soc_upgrade.tar.gz"

STATIC int __soc_dev_rev_upgrade_info_cb(IN CONST FW_UG_S *fw);//SOC设备升级入口
STATIC VOID __soc_dev_status_changed_cb(IN CONST GW_STATUS_E status);//SOC设备云端状态变更回调
STATIC VOID __soc_dev_dp_query_cb(IN CONST TY_DP_QUERY_S *dp_qry);//SOC设备特定数据查询入口
STATIC VOID __soc_dev_obj_dp_cmd_cb(IN CONST TY_RECV_OBJ_DP_S *dp);//SOC设备格式化指令数据下发入口
STATIC VOID __soc_dev_raw_dp_cmd_cb(IN CONST TY_RECV_RAW_DP_S *dp);//SOC设备透传指令数据下发入口
STATIC VOID __soc_dev_reset_req_cb(GW_RESET_TYPE_E type);//SOC设备进程重启请求入口
STATIC VOID __soc_dev_net_status_cb(IN CONST GW_WIFI_NW_STAT_E stat);//SOC外网状态变动回调
STATIC VOID _soc_dev_app_log_path_cb(OUT CHAR_T *path, IN CONST INT_T len);//上传云端日志入口

pthread_mutex_t mtx_mapheart;
int8_t getmapflag = COMMFLAG_INMAP;
uint8_t getmapheart = 0;  //APP地图页面心跳

static int init_mtx_mapheart(void)
{
    int ret = 0;
    ret = pthread_mutex_init(&mtx_mapheart, NULL);
    
    printf("init mtx_mapheart :%d\n",ret);
    if (0 != ret)
    {
        printf("ERROR: init mtx_mapheart error:%d\n",ret);
        return -1;
    }
    return 0;
}

void SystemErrorHandler(int signum)
{
    const int len=1024;
    void *func[len];
    size_t size;
    int i;
    char **funs;

    printf("Dump stack start...\n");
    printf("SIG name is %s, SIG num is %d\n", strsignal(signum), signum);
    //signal(signum,SIG_DFL);
    size=backtrace(func,len);
    printf("backtrace() returned %d addresses\n", size);
    funs=(char**)backtrace_symbols(func,size);    
    if (funs == NULL) {
        fprintf(stderr,"null funs\n");
        perror("backtrace_symbols");
        exit(EXIT_FAILURE);
    }
    fprintf(stderr,"System error,Stack trace:\n");
    for(i=0;i<size;++i) 
        fprintf(stderr,"  [%d] %s \n",i,funs[i]);
    fprintf(stderr,"**************** Stack trace  end ************** cmw\n");
    free(funs);
    printf("Dump stack end...\n");
    
    signal(signum, SIG_DFL); /* 恢复信号默认处理 */
	raise(signum);           /* 重新发送信号 */
    //exit(1);
}

void Fun1()
{
    char *p=NULL;
    *p = 'A';
}

void testFun()
{
    Fun1();
}

int add_backtrace_signal(void)
{
    signal(SIGSEGV,SystemErrorHandler); //Invaild memory address
    signal(SIGABRT,SystemErrorHandler); // Abort signal
    signal(SIGINT,SystemErrorHandler); // 程序终止（interrupt）信号
    signal(SIGTERM,SystemErrorHandler); // 捕获软件终止的信号
    signal(SIGBUS, SystemErrorHandler);     // 总线错误
//    signal(SIGSEGV, SystemErrorHandler);    // SIGSEGV，非法内存访问
    signal(SIGFPE, SystemErrorHandler);       // SIGFPE，数学相关的异常，如被0除，浮点溢出，等等
//    signal(SIGABRT, SystemErrorHandler);     // SIGABRT，由调用abort函数产生，进程非正常退出
    signal(SIGILL, SystemErrorHandler);       // 执行了非法指令
    signal(SIGQUIT, SystemErrorHandler);       // Ctrl+

    signal(SIGIOT, SystemErrorHandler); 
    signal(SIGTRAP, SystemErrorHandler);
    signal(SIGXCPU, SystemErrorHandler);
    signal(SIGXFSZ, SystemErrorHandler);
    //signal(SIGALRM, SystemErrorHandler);      // 在链接成功后使用了 alarm, 这里就不用了
    signal(SIGHUP, SystemErrorHandler);
    //signal(SIGPIPE, SystemErrorHandler);
#if 1
    signal(SIGPIPE,SIG_IGN);        // 涂鸦要求先忽略
#else
    struct sigaction sa;
    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, 0);
#endif
    signal(SIGPOLL, SystemErrorHandler);
    signal(SIGPROF, SystemErrorHandler);
    signal(SIGSYS, SystemErrorHandler);
    signal(SIGVTALRM, SystemErrorHandler);

    /* 用户自定义的中断也会导致进程终止 */
    signal(SIGUSR1, SystemErrorHandler);
    signal(SIGUSR2, SystemErrorHandler);

    /* 进程停止的信号 */
    signal(SIGTSTP, SystemErrorHandler);
    signal(SIGTTIN, SystemErrorHandler);
    signal(SIGTTOU, SystemErrorHandler);
    
    //testFun();
    return 0;
}

void write_sys_version(void) {
    char ver_str[64]={0};
    int ret = 0;
    ret = snprintf(ver_str,sizeof(ver_str), "APP_SERVER:    %s", APP_SERVER_VER);
    if(ret < 0)
    {
        printf("write_sys_version fail\r\n");
        return;
    }
    sys_version_write(ver_str);
    
    memset(ver_str, 0, sizeof(ver_str));
    ret = snprintf(ver_str,sizeof(ver_str), "ESLAM_SDK_VER: %s", robot_app_fw_version());
    if(ret < 0)
    {
        printf("write_sys_version fail\r\n");
        return;
    }
    sys_version_write(ver_str);

    memset(ver_str, 0, sizeof(ver_str));
    ret = snprintf(ver_str,sizeof(ver_str), "ESLAM_APP_VER: %s", "NONE");
    if(ret < 0)
    {
        printf("write_sys_version fail\r\n");
        return;
    }
    sys_version_write(ver_str);
}

int wifi_fault_check(void) {
    static int wifi_fault = 0;
    char mac[32];

    if (robot_app_state_sleep() || wifi_fault) {
        return 1;
    }
    
    if (sys_get_mac(mac, sizeof(mac)) <= 0) {
        printf("inform_wifi_state: wifi FAULT\n");
        inform_robot_wifi_state(WIFI_STATE_FAULT);
        wifi_fault = 1;
        return 0;
    }

    return 1;
}

static ROBOT_CB_FUNS s_robot_cbs = {
    .sys_cb = robot_notify_sys_msg_cb,
    .app_cb = robot_app_packet_cb,
    .allow_upgrade_cb = robot_app_allowed_upgrade,
};
	
//设置adebug log模式
int set_adebug_log(void)
{
	int *packet_buff;
	int packet_datalen,packet_size;

    packet_datalen = 4;
    packet_size = sizeof(packet_datalen) + packet_datalen;
    packet_buff = malloc(sizeof(packet_datalen) + packet_datalen);
    if(packet_buff == NULL)
    {
		printf("\033[1m\033[40;33m[%s:%d] packet_buff malloc error  \033[0m\n",__FUNCTION__,__LINE__);
		return -1;
    }

    *packet_buff = packet_datalen;
    *(packet_buff + 1) = 1;
    if(send_msg_to_hoslam_app(MSG_TOGGLE_LOG, (char *)packet_buff, packet_size) == 0)
    {
		printf("\033[1m\033[40;33m[%s:%d] set adebug log OK \033[0m\n",__FUNCTION__,__LINE__);
		return 0;
	}
	else
	{
		printf("\033[1m\033[40;33m[%s:%d] set adebug log failed \033[0m\n",__FUNCTION__,__LINE__);
		return -1;
	}
		
}

void *u_check(void *msg)
{
    while(1)
    {
        sys_debug_usb_log_check();
        sleep(10);
    }
}

extern int WIFI_MODULE;

int main(int argc, char** argv) {
    pthread_t thread_u_check;

    if (argc == 2) {
        if((strcmp(argv[1], "-v") == 0)||(strcmp(argv[1], "-V") == 0)) {
            sys_version_read();
            return 0;
        }
    }
    
    if(argc > 1) {
        printf("app server Usage: [%s -v] to see version\n", basename(argv[0]));
        printf("your input(%d): %s %s ...\r\n", argc, argv[0], argv[1]);
        return -1;
    }

    printf("wifi app server start ...\n");
    init_map_record_lock();
    init_all_mutex();
	init_wifi_connet_mutex();
	
    pthread_mutex_lock(&wifi_connect_ptx);  /* 涂鸦SDK初始化完成之前不能进行配网 */
    WIFI_MODULE = read_wi_mode_value();//读取wifi模式
    WIFI_MODULE = WIFI_SSV_PUBLIC;
    printf("WIFI_MODE = %d\n",WIFI_MODULE);
    tuya_cp_triplet();
    /* 初始化扫地机api */
    robot_api_init(&s_robot_cbs);     
    printf("IOT SDK Version: %s \r\n", tuya_iot_get_sdk_info());
    printf("CFG_STORAGE_PATH: %s \r\n", CFG_STORAGE_PATH);
    //printf("APP_SERVER_VER: %s \r\n", APP_SERVER_VER);
    sys_log(LOG_CRIT, "APP_SERVER_VER: %s \r\n", APP_SERVER_VER);
    add_backtrace_signal();
    //init_soft_timer();
    init_wifi_module_mutex();
    init_history_record_lock();
    map_list_init();

    // int  ret = pthread_create(&thread_u_check, NULL,u_check, NULL);
    // if (ret)
    // {
    //     printf("create thread_u_check ERROR!\n");
    //     return -1;
    // }
    int tuya_error = 0;
    do{
        int mtx_ret = init_mtx_mapheart();
        if(OPRT_OK != mtx_ret) {
            tuya_error = -5;
            break;
        }

        OPERATE_RET op_ret = tuya_iot_init(CFG_STORAGE_PATH);
        if(OPRT_OK != op_ret) {
            //PR_ERR("tuya_iot_init err:%d PATH:%s", op_ret, CFG_STORAGE_PATH);
            sys_log(LOG_CRIT, "tuya_iot_init err:%d PATH:%s", op_ret, CFG_STORAGE_PATH);
            tuya_error = -1;
            break;
        }
        PR_DEBUG("tuya_iot_init success");

        SetLogManageAttr(TY_LOG_LEVEL_DEBUG);

        op_ret = tuya_uuid_init(TUYA_UUID_FILE);
        if(OPRT_OK != op_ret) {
            set_uuid_fail_flag(1);
            //PR_ERR("###### tuya_uuid_init err:%d, and don't use default uuid  ######", op_ret);
            sys_log(LOG_CRIT, "###### tuya_uuid_init err:%d, and don't use default uuid  ######", op_ret);
            tuya_error = -6;
            break;
        }
        
        WF_GW_PROD_INFO_S prod_info = {tuya_uuid_get(), tuya_uuid_get_authkey(), NULL, NULL};
  
        op_ret = tuya_iot_set_wf_gw_prod_info(&prod_info);
        if(OPRT_OK != op_ret) {
            //PR_ERR("tuya_iot_set_wf_gw_prod_info err:%d", op_ret);
            sys_log(LOG_CRIT, "tuya_iot_set_wf_gw_prod_info err:%d", op_ret);
            tuya_error = -2;
            break;
        }
        PR_DEBUG("tuya_iot_set_wf_gw_prod_info success");

        TY_IOT_CBS_S iot_cbs = {
            __soc_dev_status_changed_cb,
            __soc_dev_rev_upgrade_info_cb,
            __soc_dev_reset_req_cb,
            __soc_dev_obj_dp_cmd_cb,
            __soc_dev_raw_dp_cmd_cb,
            __soc_dev_dp_query_cb,
            NULL,
        };

#ifdef WIFI_SMART_MODE
        op_ret = tuya_iot_wf_soc_dev_init(GWCM_OLD, WF_START_SMART_FIRST,
                            &iot_cbs, tuya_get_product_key(), robot_ctl_wait_robot_ready());
#else
        op_ret = tuya_iot_wf_soc_dev_init(GWCM_OLD, WF_START_AP_ONLY,
                            &iot_cbs, tuya_get_product_key(), robot_ctl_wait_robot_ready());
#endif
        if(OPRT_OK != op_ret) {
            //PR_ERR("tuya_iot_wf_soc_dev_init err:%d",op_ret);
            sys_log(LOG_CRIT, "tuya_iot_wf_soc_dev_init err:%d",op_ret);
            tuya_error = -3;
            break;
        }
        PR_DEBUG("tuya_iot_wf_soc_dev_init success");
        printf("tuya_uuid = %s,tuya_pid = %s\n",tuya_uuid_get(),tuya_get_product_key());
        op_ret = tuya_iot_reg_get_wf_nw_stat_cb(__soc_dev_net_status_cb);
        if(OPRT_OK != op_ret) {
            //PR_ERR("tuya_iot_reg_get_wf_nw_stat_cb err:%d",op_ret);
            sys_log(LOG_CRIT, "tuya_iot_reg_get_wf_nw_stat_cb err:%d",op_ret);
            tuya_error = -4;
            break;
        }

        TY_IOT_APP_CBS_S iot_app_cbs = {
            _soc_dev_app_log_path_cb,
        };

        tuya_iot_app_cbs_init(&iot_app_cbs);  /* 初始化上报日志接口 */
        printf("tuya_iot_app_cbs_init\n");

   }while(0);
    
    if(tuya_error == 0)
    {
        PR_DEBUG("tuya_iot_reg_get_wf_nw_stat_cb success");
    }
    else
    {
        PR_DEBUG("tuya_iot init fail error:%d", tuya_error);
    }
    pthread_mutex_unlock(&wifi_connect_ptx); /* 涂鸦SDK初始化完毕后，就可以进行配网操作 */
    sys_is_and_create_dir();
    
    sys_version_delete();
    write_sys_version();
    printf("****************  Version Show ********************\r\n");
    sys_version_read();
    printf("****************  Version Show End ********************\r\n");
	send_msg_to_hoslam_app(0x33,NULL,0); /* 发送指令告诉hoslam，后续采用2bit地图传送模式	*/
    sys_shell("service miio stop");
	int heartcnt = 0;
    int wifi_fault_check_flag = 0;
    while (1) 
	{
        if (getmapheart > 0)
        {
            getmapflag = COMMFLAG_INMAP;
        }
        else
        {
//            getmapflag = -1;
        }

        if (heartcnt == 30 * 2)  //每2min检查一次
        {
            pthread_mutex_lock(&mtx_mapheart);
            if(0 == getmapheart)
            {
                getmapflag = -1;
            }
            getmapheart = 0;
            pthread_mutex_unlock(&mtx_mapheart);

            heartcnt = 0;
        }
        heartcnt++;
        PR_DEBUG("heartcnt = %d   getmapheart = %d   getmapflag = %d",heartcnt, getmapheart, getmapflag);
        /* 开机30s后就检测一次wifi故障,以后不检测 */
        if((wifi_fault_check_flag == 0) && (heartcnt > 15))
        {
            //wifi_fault_check();
            wifi_fault_check_flag = 1;
        }
        sleep(2);
    }

    return 0;
}

//SOC设备升级相关代码开始
STATIC VOID __upgrade_notify_cb(IN CONST FW_UG_S *fw, IN CONST INT_T download_result, IN PVOID_T pri_data)
{
    FILE *p_upgrade_fd = (FILE *)pri_data;
    fclose(p_upgrade_fd);

    if(download_result == 0) {
        PR_DEBUG("SOC Upgrade File Download Success");
        //User TODO
        iot_upgrade_download_complete(SOC_OTA_FILE, fw->tp);
    } else {
        PR_ERR("SOC Upgrade File Download Fail.ret = %d", download_result);
    }
}

STATIC OPERATE_RET __get_file_data_cb(IN CONST FW_UG_S *fw, IN CONST UINT_T total_len, IN CONST UINT_T offset,
                                      IN CONST BYTE_T *data, IN CONST UINT_T len, OUT UINT_T *remain_len, IN PVOID_T pri_data)
{
//    PR_DEBUG("Rev File Data");
//    PR_DEBUG("Total_len:%u", total_len);
//    PR_DEBUG("Offset:%u Len:%u", offset, len);
    FILE *p_upgrade_fd = (FILE *)pri_data;
    fwrite(data, 1, len, p_upgrade_fd);
    *remain_len = 0;

    return OPRT_OK;
}

//SOC设备升级入口
int __soc_dev_rev_upgrade_info_cb(IN CONST FW_UG_S *fw)
{
    PR_DEBUG("SOC Rev Upgrade Info");
    PR_DEBUG("fw->tp:%d", fw->tp);
    PR_DEBUG("fw->fw_url:%s", fw->fw_url);
    PR_DEBUG("fw->fw_hmac:%s", fw->fw_hmac);
    PR_DEBUG("fw->sw_ver:%s", fw->sw_ver);
    PR_DEBUG("fw->file_size:%u", fw->file_size);

    FILE *p_upgrade_fd = fopen(SOC_OTA_FILE, "w+b");
    if(NULL == p_upgrade_fd){
        PR_ERR("open upgrade file fail. upgrade fail %s", SOC_OTA_FILE);
        return -1;
    }
#if 0 // tuya_sdk上报进度
        OPERATE_RET op_ret = tuya_iot_upgrade_gw(fw, __get_file_data_cb, __upgrade_notify_cb, p_upgrade_fd);
#else // 关闭tuya_sdk进度上报，由应用层上报(由 tuya_iot_dev_upgd_progress_rept 上报)
        OPERATE_RET op_ret = tuya_iot_upgrade_gw_notify(fw, __get_file_data_cb, __upgrade_notify_cb, p_upgrade_fd,FALSE,0);
#endif
    if(OPRT_OK != op_ret) {
        PR_ERR("tuya_iot_upgrade_gw err:%d",op_ret);
    }
}
//升级相关代码结束

//SOC设备云端状态变更回调
VOID __soc_dev_status_changed_cb(IN CONST GW_STATUS_E status)
{
    //PR_DEBUG("SOC TUYA-Cloud Status:%d", status);
    sys_log(LOG_CRIT, "SOC TUYA-Cloud Status:%d", status);
}

//SOC设备特定数据查询入口
VOID __soc_dev_dp_query_cb(IN CONST TY_DP_QUERY_S *dp_qry)
{
    PR_DEBUG("SOC Rev DP Query Cmd");
    if(dp_qry->cid != NULL) PR_ERR("soc not have cid.%s", dp_qry->cid);

    if(dp_qry->cnt == 0) {
        PR_DEBUG("soc rev all dp query");
        //User TODO
    }else {
        PR_DEBUG("soc rev dp query cnt:%d", dp_qry->cnt);
        UINT_T index = 0;
        for(index = 0; index < dp_qry->cnt; index++) {
            PR_DEBUG("rev dp query:%d", dp_qry->dpid[index]);
            //User TODO
        }
    }
}

static bool appointmentCmdFlag = false;
bool isAppointmentCmd(void) {
    return appointmentCmdFlag;
}

static void checkAppointmentCmd(IN CONST TY_RECV_OBJ_DP_S *dp) {
    if(dp->cmd_tp == DP_CMD_TIMER) {
        appointmentCmdFlag = true;
    } else {
        appointmentCmdFlag = false;
    }
}

//SOC设备格式化指令数据下发入口
VOID __soc_dev_obj_dp_cmd_cb(IN CONST TY_RECV_OBJ_DP_S *dp)
{
    printf("SOC Rev DP Obj Cmd t1:%d t2:%d CNT:%u\n", dp->cmd_tp, dp->dtt_tp, dp->dps_cnt);
    if(dp->cid != NULL) PR_ERR("soc not have cid.%s", dp->cid);
    //检查是否预约命令
    checkAppointmentCmd(dp);
    UINT_T index = 0;
    for(index = 0; index < dp->dps_cnt; index++)
    {
        CONST TY_OBJ_DP_S *p_dp_obj = dp->dps + index;
        printf("idx:%d dpid:%d type:%d ts:%u ", index, p_dp_obj->dpid, p_dp_obj->type, p_dp_obj->time_stamp);
        switch (p_dp_obj->type) {
        case PROP_BOOL:     { printf("bool value:%d", p_dp_obj->value.dp_bool); break;}
        case PROP_VALUE:    { PR_DEBUG("INT value:%d", p_dp_obj->value.dp_value); break;}
        case PROP_STR:      { PR_DEBUG("str value:%s", p_dp_obj->value.dp_str); break;}
        case PROP_ENUM:     { PR_DEBUG("enum value:%u", p_dp_obj->value.dp_enum); break;}
        case PROP_BITMAP:   { PR_DEBUG("bits value:0x%X", p_dp_obj->value.dp_bitmap); break;}
        default:            { PR_ERR("idx:%d dpid:%d type:%d ts:%u is invalid", index, p_dp_obj->dpid, p_dp_obj->type, p_dp_obj->time_stamp); break;}
        }//end of switch

        wifi_app_msg_parse(p_dp_obj);
    }
}

//SOC设备透传指令数据下发入口
VOID __soc_dev_raw_dp_cmd_cb(IN CONST TY_RECV_RAW_DP_S *dp)
{
    printf("SOC Rev DP Raw Cmd t1:%d t2:%d dpid:%d len:%u", dp->cmd_tp, dp->dtt_tp, dp->dpid, dp->len);
    //printf("head = %hhu,cmd = %hhu\n",dp->data[0],dp->data[1]);
    
    //printf("url = %s\n",&(dp->data[6]));
    if(dp->cid != NULL) PR_ERR("soc not have cid.%s", dp->cid);

    //User TODO
    wifi_app_raw_parse(dp);

    //用户处理完成之后需要主动上报最新状态，这里简单起见，直接返回收到的数据，认为处理全部成功。
    // OPERATE_RET op_ret = dev_report_dp_raw_sync(dp->cid,dp->dpid,dp->data,dp->len,0);
    // if(OPRT_OK != op_ret) {
    //     PR_ERR("dev_report_dp_json_async op_ret:%d",op_ret);
    // }
}

//SOC设备进程重启请求入口
VOID __soc_dev_reset_req_cb(GW_RESET_TYPE_E type)
{
    WF_WK_MD_E wifi_mode;
    //PR_DEBUG("SOC Rev Reset Req %d", type);
    sys_log(LOG_CRIT, "SOC Rev Reset Req %d", type);
    // remote is reset factory and local is not,reset factory again.
    if(type == GW_RESET_DATA_FACTORY){
        return;
    }

    if(type == GW_REMOTE_RESET_FACTORY)
    {
        printf("remove foctory\n");
        robot_api_control(ROBOT_CMD_RESTORE_FACTORY_SETTINGS,NULL);
    }

#ifdef WIFI_SMART_MODE
    iot_wifi_mode_get();
#endif
    iot_start_config_network();
}

// SOC外网状态变动回调
STATIC VOID __soc_dev_net_status_cb(IN CONST GW_WIFI_NW_STAT_E stat)
{
    iot_state_change(stat);
}

// 上报日志回调函数
STATIC VOID _soc_dev_app_log_path_cb(OUT CHAR_T *path, IN CONST INT_T len)
{
    snprintf((char *)path,len,APPSERVER_SYS_LOG_FILE);
    sys_log(LOG_CRIT, "writing soc log");
}
