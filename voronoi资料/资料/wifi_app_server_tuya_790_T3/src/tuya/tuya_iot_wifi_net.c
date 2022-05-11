#include "tuya_hal_wifi.h"
#include "uni_log.h"

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "sys_api.h"
#include "sys_timer.h"
#include "robot_app_msg.h"
#include "tuya_iot_state_adapter.h"
#include "ssv_smartlink.h"


extern int WIFI_MODULE;

// WiFi模式是否是SSV6156P
//#define WIFI_MODULE_SSV6152

extern void set_wifi_config_state(void);
// WIFI设备的名称
//#define WLAN_DEV    "wlp3s0"
#define WLAN_DEV    "wlan0"
//#define WLAN_AP_DEV	"wlan1"

// 配置文件路径
//#define WPA_SUPPLICANT_CONF     "/root/etc/wpa_supplicant_ty.conf"
#define WPA_SUPPLICANT_CONF     "/tmp/wpa_supplicant_ty.conf"
#define UDHCPC_SCRIPT           "/firmware/etc/udhcpc.script"
//#define HOSTAPD_CONF            "/tmp/hostapd_ty.conf"
#define TMP_HOSTAPD_CONF        "/tmp/hostapd_ty.conf"
#define UDHCPD_CONF             "/tmp/udhcpd_ty.conf"
#define SSID_SEARCH_PREFIX      "ssid=SmartLife-"
// WIFI芯片是否是MTK7601
//#define WIFI_CHIP_7601

#define MAX_AP_SEARCH 64
#define DEFAULT_IP_ADDR "192.168.100.1"


#define WPA_FLIE_INFO "ctrl_interface=/var/run/wpa_supplicant\n"\
                        "ap_scan=1\n"\
                        "network={\n"\
                        "ssid=\"%s\"\n"\
                        "scan_ssid=1\n"\
                        "psk=\"%s\"\n"\
                        "}\n"

#define WPA_FLIE_INFO_NO_PASSWORD "ctrl_interface=/var/run/wpa_supplicant\n"\
							"ap_scan=1\n"\
							"update_config=1\n"\
							"network={\n"\
							"ssid=\"%s\"\n"\
							"scan_ssid=1\n"\
							"key_mgmt=NONE\n"\
							"}\n"

#define UDHCPD_TY_CONFIG "start		192.168.100.20\n"\
                        "end		192.168.100.254\n"\
                        "interface	wlan0\n"\
                        "pidfile	/var/run/udhcpd_ty.pid\n"\
                        "lease_file	/var/lib/misc/udhcpd_ty.leases\n"\
                        "opt	dns	192.168.100.1\n"\
                        "option	subnet	255.255.255.0\n"\
                        "opt	router	192.168.100.1\n"\
                        "option	domain	local\n"\
                        "option	lease	864000\n"\
                        "option	msstaticroutes	10.0.0.0/8 10.127.0.1\n"\
                        "option	staticroutes	10.0.0.0/8 10.127.0.1, 10.11.12.0/24 10.11.12.1\n"\
                        "option	0x08	01020304\n"
//IP地址是否变化 来区分 分否分配了新IP地址
static  NW_IP_S old_ip;
static sys_timer_t *pTimer = NULL;
static pthread_mutex_t wifi_restart_mutex; 
OPERATE_RET sys_wifi_all_ap_scan(void);

static volatile SNIFFER_CALLBACK s_pSnifferCall = NULL;
static volatile int s_enable_sniffer = -1;
static bool wifi_connet = false;
static pthread_mutex_t wifi_connet_mutex;



static void kill_cmd(char *pCmd)
{
    sys_shell("killall -9 %s", pCmd);
}


/* 初始化生成udhcpd_ty.conf 文件 */
void init_udhcpd_ty_conf_file(void)
{
   // uint8_t wpa_buf_file[10240];
    FILE * fp = NULL;  
   // memset(wpa_buf_file, 0, 1024);
    printf("init_udhcpd_ty_conf_file \n");
    fp = fopen(UDHCPD_CONF, "w+");
    if(fp == NULL)
    {
        printf("Error opening UDHCPD_CONF\n");
        return;
    }
    fwrite(UDHCPD_TY_CONFIG, strlen(UDHCPD_TY_CONFIG),1,fp);
    fflush(fp);
    fclose(fp);

}

/* 将账号密码写到 wpa配置文件中 */
void write_wpa_config_file(IN CONST CHAR_T *ssid,IN CONST CHAR_T *passwd)
{
    uint8_t wpa_buf_file[1024];
    FILE * fp = NULL;  
    uint8_t len = 0;
    memset(wpa_buf_file, 0, 1024);

    if(strlen(ssid) > 32 || strlen(passwd) > 64)
    {
        printf(" ssid passwd len Out of limit \n");
        return;
    }
	
	if(strlen(passwd) == 0)
	{
		len = sprintf(wpa_buf_file,WPA_FLIE_INFO_NO_PASSWORD,ssid);
	}
	else
	{	
		len = sprintf(wpa_buf_file,WPA_FLIE_INFO,ssid,passwd);
	}
    for(int i = 0; i < len; i++)
    {
        printf("%c",wpa_buf_file[i]);
    }
    printf("\n");
    fp = fopen(WPA_SUPPLICANT_CONF, "w+");
    if(fp == NULL)
    {
        printf("Error opening WPA_SUPPLICANT_CONF\n");
        return;
    }
    fwrite(wpa_buf_file, len,1,fp);
    fflush(fp);
    fclose(fp);

}

void init_wifi_connet_mutex(void)
{
    if(pthread_mutex_init(&wifi_connet_mutex,NULL) != 0)
    {
        printf("ERROR:init wifi_connet_mutex fail\r\n");
    }

    printf(" init wifi_connet_mutex success\r\n");
}

void* func_wifi_station_connect(void *arg)
{
	sys_log(LOG_CRIT, "wifi connect start \n");
	
	sys_shell("udhcpc -i " WLAN_DEV " -s " UDHCPC_SCRIPT " -T 2 -t 5 -A 10 ");

	pthread_mutex_lock(&wifi_connet_mutex); 
	wifi_connet = true;
	pthread_mutex_unlock(&wifi_connet_mutex); 
	
	sys_log(LOG_CRIT, "wifi connect success \n");
    return NULL;
}


// 使用指定SSID和PASSWD连接WIFI
OPERATE_RET tuya_hal_wifi_station_connect(IN CONST CHAR_T *ssid,IN CONST CHAR_T *passwd)
{
    //PR_DEBUG("STA Con AP ssid:%s passwd:%s", ssid, passwd);
    sys_log(LOG_CRIT, "STA Con AP ssid:%s passwd:%s", ssid, passwd);
	/*联网前，停止wifi操作*/
    sys_shell("killall -9 wpa_supplicant 2>/dev/null");
    sys_shell("killall -9 udhcpc 2>/dev/null");
    sys_shell("killall -9 hostapd 2>/dev/null");
    sys_shell("killall -9 udhcpd 2>/dev/null");
	
    pthread_mutex_lock(&wifi_connet_mutex); 
	wifi_connet = false;
	pthread_mutex_unlock(&wifi_connet_mutex); 
	
    sys_shell("ifconfig %s down",WLAN_DEV);
    sys_shell("iwconfig %s mode Managed",WLAN_DEV);
	sys_shell("ifconfig %s up %s",WLAN_DEV,DEFAULT_IP_ADDR);

	write_wpa_config_file(ssid,passwd);
	//sys_shell("wpa_passphrase \"%s\" \"%s\" > " WPA_SUPPLICANT_CONF, ssid, passwd);
	sys_shell("wpa_supplicant -Dnl80211 -B -i" WLAN_DEV " -c" WPA_SUPPLICANT_CONF); 


#if 0
	sys_shell("udhcpc -i " WLAN_DEV " -s " UDHCPC_SCRIPT " &");
#else
		pthread_t thread_wifi_connect;
		pthread_attr_t attr;
		pthread_attr_init(&attr);//初始化线程属性
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);//设置线程属性为分离状态
		int ret = pthread_create(&thread_wifi_connect, &attr,func_wifi_station_connect, NULL);
		if (ret)
		{
			sys_log(LOG_CRIT, "tuya_hal_wifi_station_connect create thread func_wifi_station_connect ERROR! \n");
			sys_shell("udhcpc -i " WLAN_DEV " -s " UDHCPC_SCRIPT " -T 2 -t 4 -A 10 -b ");
			pthread_mutex_lock(&wifi_connet_mutex); 
			wifi_connet = true;
			pthread_mutex_unlock(&wifi_connet_mutex); 
		}
		else
		{
			int i=8;
			NW_IP_S new_ip;
			memset(&new_ip, 0, sizeof(NW_IP_S));
			//等待连接上路由，最多等8秒
			while (i)
			{	
				i--;
				sleep(1);
				tuya_hal_wifi_get_ip(WF_STATION, &new_ip);
				if(new_ip.ip[0] != 0 && strcmp(new_ip.ip, DEFAULT_IP_ADDR) != 0 )
				{		
					printf("\n\n ***** connect route success, get_ip: %s ***** \n\n",new_ip.ip);
					break;
				}	
			}
		}
		
#endif
    //sys_wifi_all_ap_scan();  /* 扫描周围热点 */
    //sys_shell("/root/bin/scanap");

    sys_log(LOG_CRIT, "hwl_wf_station_connect return OPRT_OK \n");
    return OPRT_OK;
}

// 获取当前环境下的SSID列表
OPERATE_RET tuya_hal_wifi_all_ap_scan(OUT AP_IF_S **ap_ary,OUT UINT_T *num)
{
    static AP_IF_S s_aps[MAX_AP_SEARCH];
    memset(s_aps, 0, sizeof(s_aps));
    *ap_ary = s_aps;
    *num = 0;

    FILE *pp = sys_popen("iwlist "WLAN_DEV" scan", "r");
    if(pp == NULL)
        return OPRT_COM_ERROR;

    char tmp[256] = {0};
    memset(tmp, 0, sizeof(tmp));

    int recordIdx = -1;
    while (fgets(tmp, sizeof(tmp), pp) != NULL)
    {
        /* 首先找BSSID	  以此为基准 */
        char *pBSSIDStart = strstr(tmp, " - Address: ");
        if(pBSSIDStart != NULL)
        {
            recordIdx++;
            if(recordIdx >= MAX_AP_SEARCH)
            {
                printf(" Reach Max Record \r\n");
                recordIdx--;
                break;
            }

            BYTE_T *pTmp = s_aps[recordIdx].bssid;
            int x1,x2,x3,x4,x5,x6;

            sscanf(pBSSIDStart + strlen(" - Address: "), "%x:%x:%x:%x:%x:%x",&x1,&x2,&x3,&x4,&x5,&x6);
            pTmp[0] = x1 & 0xFF;
            pTmp[1] = x2 & 0xFF;
            pTmp[2] = x3 & 0xFF;
            pTmp[3] = x4 & 0xFF;
            pTmp[4] = x5 & 0xFF;
            pTmp[5] = x6 & 0xFF;

            goto ReadNext;
        }else
        {
            if(recordIdx < 0)
            {/* 只有找到第一个BSSID 才能继续读下去 */
                goto ReadNext;
            }
        }

        {
            /* 查找signal  */
            char *pSIGNALStart = strstr(tmp, "Quality=");
            if(pSIGNALStart != NULL)
            {
                int x = 0;
                int y = 0;
                int level = 0;
                sscanf(pSIGNALStart + strlen("Quality=") , "%d/%d  Signal level=%d dBm",&x,&y, &level);
                s_aps[recordIdx].rssi = x * 100/ (y+1);
                s_aps[recordIdx].resv1 = level; /* 获取信号强度 */
                goto ReadNext;
            }
        }

        {
            /* 查找channel	*/
            char *pCHANNELStart = strstr(tmp, "(Channel ");
            if(pCHANNELStart != NULL)
            {
                int x = 0;
                sscanf(pCHANNELStart + strlen("(Channel "), "%d)", &x);
                s_aps[recordIdx].channel = x;
                goto ReadNext;
            }
        }

        {
            /* 查找ssid  */
            char *pSSIDStart = strstr(tmp, "ESSID:\"");
            if(pSSIDStart != NULL)
            {
                sscanf(pSSIDStart + strlen("ESSID:\""), "%s", s_aps[recordIdx].ssid);
                s_aps[recordIdx].s_len = strlen(s_aps[recordIdx].ssid);
                if(s_aps[recordIdx].s_len != 0)
                {
                    s_aps[recordIdx].ssid[s_aps[recordIdx].s_len - 1] = 0;
                    s_aps[recordIdx].s_len--;
                }
                if(s_aps[recordIdx].s_len == 0)
                {
                    recordIdx --; /* 如果检索到的SSID为空，则不加入信道检索列表 增加配网成功率 */
                }
                goto ReadNext;
            }
        }

ReadNext:
        memset(tmp, 0, sizeof(tmp));
    }

    sys_pclose(pp);
    *num = recordIdx + 1;

    PR_DEBUG("WIFI Scan AP Begin");
    int index = 0;
    for(index = 0; index < *num; index++)
    {
        // PR_DEBUG("index:%d channel:%d bssid:%02X-%02X-%02X-%02X-%02X-%02X RSSI:%d SSID:%s",
        //        index, s_aps[index].channel, s_aps[index].bssid[0],  s_aps[index].bssid[1],  s_aps[index].bssid[2],  s_aps[index].bssid[3],
        //         s_aps[index].bssid[4],  s_aps[index].bssid[5], s_aps[index].rssi, s_aps[index].ssid);
          sys_log(LOG_CRIT,"index:%d channel:%d RSSI:%d level:%d SSID:%s SSIDlen:%d",
               index, s_aps[index].channel, s_aps[index].rssi,s_aps[index].resv1, s_aps[index].ssid,s_aps[index].s_len);
    }
    PR_DEBUG("WIFI Scan AP End");

    return OPRT_OK;
}

// 获取当前环境下的SSID列表( 一微内部使用 )
OPERATE_RET sys_wifi_all_ap_scan(void)
{
    AP_IF_S s_aps[MAX_AP_SEARCH];
    memset(s_aps, 0, sizeof(s_aps));
    FILE *pp = sys_popen(WIFI_SCAN_SH, "r");
    if(pp == NULL)
        return OPRT_COM_ERROR;

    char tmp[256] = {0};
    memset(tmp, 0, sizeof(tmp));

    int recordIdx = -1;
    while (fgets(tmp, sizeof(tmp), pp) != NULL)
    {
        /* 首先找BSSID	  以此为基准 */
        char *pBSSIDStart = strstr(tmp, " - Address: ");
        if(pBSSIDStart != NULL)
        {
            recordIdx++;
            if(recordIdx >= MAX_AP_SEARCH)
            {
                printf(" Reach Max Record \r\n");
                recordIdx--;
                break;
            }

            BYTE_T *pTmp = s_aps[recordIdx].bssid;
            int x1,x2,x3,x4,x5,x6;

            sscanf(pBSSIDStart + strlen(" - Address: "), "%x:%x:%x:%x:%x:%x",&x1,&x2,&x3,&x4,&x5,&x6);
            pTmp[0] = x1 & 0xFF;
            pTmp[1] = x2 & 0xFF;
            pTmp[2] = x3 & 0xFF;
            pTmp[3] = x4 & 0xFF;
            pTmp[4] = x5 & 0xFF;
            pTmp[5] = x6 & 0xFF;

            goto ReadNext;
        }else
        {
            if(recordIdx < 0)
            {/* 只有找到第一个BSSID 才能继续读下去 */
                goto ReadNext;
            }
        }

        {
            /* 查找signal  */
            char *pSIGNALStart = strstr(tmp, "Quality=");
            if(pSIGNALStart != NULL)
            {
                int x = 0;
                int y = 0;
                int level = 0;
                sscanf(pSIGNALStart + strlen("Quality=") , "%d/%d  Signal level=%d dBm",&x,&y, &level);
                s_aps[recordIdx].rssi = x * 100/ (y+1);
                s_aps[recordIdx].resv1 = level; /* 获取信号强度 */
                goto ReadNext;
            }
        }

        {
            /* 查找channel	*/
            char *pCHANNELStart = strstr(tmp, "(Channel ");
            if(pCHANNELStart != NULL)
            {
                int x = 0;
                sscanf(pCHANNELStart + strlen("(Channel "), "%d)", &x);
                s_aps[recordIdx].channel = x;
                goto ReadNext;
            }
        }

        {
            /* 查找ssid  */
            char *pSSIDStart = strstr(tmp, "ESSID:\"");
            if(pSSIDStart != NULL)
            {
                sscanf(pSSIDStart + strlen("ESSID:\""), "%s", s_aps[recordIdx].ssid);
                s_aps[recordIdx].s_len = strlen(s_aps[recordIdx].ssid);
                if(s_aps[recordIdx].s_len != 0)
                {
                    s_aps[recordIdx].ssid[s_aps[recordIdx].s_len - 1] = 0;
                    s_aps[recordIdx].s_len--;
                }
                goto ReadNext;
            }
        }

ReadNext:
        memset(tmp, 0, sizeof(tmp));
    }

    sys_pclose(pp);

    printf("Amrico WIFI Scan AP Begin\n");
    int index = 0;
    for(index = 0; index < (recordIdx + 1); index++)
    {
        printf("index:%d channel:%d bssid:%02X-%02X-%02X-%02X-%02X-%02X RSSI:%d level:%d SSID:%s SSIDlen:%d\n",
               index, s_aps[index].channel, s_aps[index].bssid[0],  s_aps[index].bssid[1],  s_aps[index].bssid[2],  s_aps[index].bssid[3],
                s_aps[index].bssid[4],  s_aps[index].bssid[5], s_aps[index].rssi,s_aps[index].resv1, s_aps[index].ssid,s_aps[index].s_len);
    }
    printf("Amrico WIFI Scan AP End\n");

    return OPRT_OK;
}

// 获取特定SSID的信息
OPERATE_RET tuya_hal_wifi_assign_ap_scan(IN CONST CHAR_T *ssid,OUT AP_IF_S **ap)
{
    /* 这里简单处理，扫描所有ap后进行查找 */
    AP_IF_S *pTotalAp = NULL;
    UINT_T tatalNum = 0;
    int index = 0;
    tuya_hal_wifi_all_ap_scan(&pTotalAp, &tatalNum);
    *ap = NULL;
    for(index = 0; index < tatalNum; index++)
    {
        if(memcmp(pTotalAp[index].ssid, ssid, pTotalAp[index].s_len) == 0)
        {
            sys_log(LOG_CRIT,"index:%d channel:%d bssid:%02X-%02X-%02X-%02X-%02X-%02X RSSI:%d SSID:%s",
               index, pTotalAp[index].channel, pTotalAp[index].bssid[0],  pTotalAp[index].bssid[1],  pTotalAp[index].bssid[2],  pTotalAp[index].bssid[3],
                pTotalAp[index].bssid[4],  pTotalAp[index].bssid[5], pTotalAp[index].rssi, pTotalAp[index].ssid);
            *ap = pTotalAp + index;
            break;
        }
    }
    return OPRT_OK;
}

// 释放内存
OPERATE_RET tuya_hal_wifi_release_ap(IN AP_IF_S *ap)
{// s_aps为静态变量，无需释放
    return OPRT_OK;
}

static int s_curr_channel = 1;
// 设置WIFI的工作信道
OPERATE_RET tuya_hal_wifi_set_cur_channel(CONST BYTE_T chan)
{
    PR_DEBUG("WIFI Set Channel:%d", chan);
    //sys_log(LOG_CRIT, "WIFI Set Channel:%d", chan);

    if(WIFI_MODULE == WIFI_SSV_PRIVATE) 
    {
        if (s_enable_sniffer == 1)
        {
            ssv_smartlink_set_channel(chan);
        }
    }
    else// WIFI_MODULE_SSV6152
    {
        sys_shell("iwconfig %s channel %d", WLAN_DEV, chan);
    }
    s_curr_channel = chan;

#ifdef WIFI_CHIP_7601
    tuya_linux_wf_wk_mode_set(WWM_SNIFFER);
#endif
    return OPRT_OK;
}

// 获取WIFI的工作信道
OPERATE_RET tuya_hal_wifi_get_cur_channel(OUT BYTE_T *chan)
{
    FILE *pp = sys_popen("iwlist "WLAN_DEV" channel", "r");

    if(pp == NULL)
        return OPRT_COM_ERROR;

    char tmp[128] = {0};
    memset(tmp, 0, sizeof(tmp));
    while (fgets(tmp, sizeof(tmp), pp) != NULL)
    {
        char *pIPStart = strstr(tmp, " (Channel ");
        if(pIPStart != NULL)
            break;
    }

    /* 查找channel	*/
    char *pCHANNELStart = strstr(tmp, " (Channel ");
    if(pCHANNELStart != NULL)
    {
        int x = 0;
        sscanf(pCHANNELStart + strlen(" (Channel "), "%d", &x);
        *chan = x;
    }else
    {
        *chan = s_curr_channel;
    }
    sys_pclose(pp);

    //PR_DEBUG("WIFI Get Curr Channel:%d", *chan);
    {
        static BYTE_T last_chan;
        if(last_chan != *chan)
        {
          //sys_log(LOG_CRIT, "WIFI Get Curr Channel:%d", *chan);
		  PR_DEBUG("WIFI Get Curr Channel:%d", *chan);
          last_chan = *chan;
        }
        
    } 
    
    return OPRT_OK;
}


static uint8_t  gBuf[MAX_PAYLOAD] = {0};
static uint32_t gBufLen = 0;

static void * func_Sniffer_ssv(void *pReserved)
{
    int ret;

    ret = ssv_smartlink_start();
    if (ret < 0)
    {
        printf("ssv_smartlink_start error: %d\n", ret);
        return NULL;
    }

    ret = ssv_smartlink_set_promisc(1);
    if (ret < 0)
    {
        printf("ssv_smartlink_set_promisc error: %d\n", ret);
        return NULL;
    }

    while ((s_pSnifferCall != NULL) && (s_enable_sniffer == 1))
    {
        ret = ssv_smartlink_recv_packet(gBuf, &gBufLen);
        if (ret < 0)
        {
            if ((errno == EINTR) || (errno == EAGAIN))
            {
                continue;
            }
            
            if ((errno == ENOMEM) || (errno == ENOBUFS))
            {
                continue;
            }
            
            printf("ssv_smartlink_recv_packet error: %d\n", ret);
        }

        s_pSnifferCall(gBuf, gBufLen);
    }

    s_pSnifferCall = NULL;

    ssv_smartlink_set_promisc(0);
    ssv_smartlink_stop();

    PR_DEBUG("Sniffer Proc Finish");
    return (void *)0;
}


#pragma pack(1)
/* http://www.radiotap.org/  */
typedef struct {
    BYTE_T it_version;
    BYTE_T it_pad;
    USHORT_T it_len;
    UINT_T it_present;
}ieee80211_radiotap_header;
#pragma pack()

static void * func_Sniffer(void *pReserved)
{
    PR_DEBUG("Sniffer Thread Create");

    int sock = socket(PF_PACKET, SOCK_RAW, htons(0x03));//ETH_P_ALL
    if(sock < 0)
    {
        printf("Sniffer Socket Alloc Fails %d \r\n", sock);
        perror("Sniffer Socket Alloc");
        return (void *)0;
    }

    {/* 强制绑定到wlan0 上。后续可以考虑去掉 */
        struct ifreq ifr;
        memset(&ifr, 0x00, sizeof(ifr));
        strncpy(ifr.ifr_name, WLAN_DEV , strlen(WLAN_DEV) + 1);
        setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, (char *)&ifr, sizeof(ifr));
    }

    #define MAX_REV_BUFFER 512
    BYTE_T rev_buffer[MAX_REV_BUFFER];

    int skipLen = 26;/* radiotap 默认长度为26 */

    while((s_pSnifferCall != NULL) && (s_enable_sniffer == 1))
    {
        int rev_num = recvfrom(sock, rev_buffer, MAX_REV_BUFFER, 0, NULL, NULL);
        ieee80211_radiotap_header *pHeader = (ieee80211_radiotap_header *)rev_buffer;
        skipLen = pHeader->it_len;

#ifdef WIFI_CHIP_7601
        skipLen = 144;
#endif
        if(skipLen >= MAX_REV_BUFFER)
        {/* 有出现过header全ff的情况，这里直接丢掉这个包 */
            continue;
        }

        if(0)
        {
            printf("skipLen:%d ", skipLen);
            int index = 0;
            for(index = 0; index < 180; index++)
            {
                printf("%02X-", rev_buffer[index]);
            }
            printf("\r\n");
        }
        if(rev_num > skipLen)
        {
            s_pSnifferCall(rev_buffer + skipLen, rev_num - skipLen);
        }
    }

     s_pSnifferCall = NULL;

    close(sock);

    PR_DEBUG("Sniffer Proc Finish");
    return (void *)0;
}


static pthread_t sniffer_thId; // 抓包线程ID
// 设置WIFI的sniffer抓包状态
OPERATE_RET tuya_hal_wifi_sniffer_set(IN CONST bool en,IN CONST SNIFFER_CALLBACK cb)
{
    if(en == TRUE)
    {
        //PR_DEBUG("Enable Sniffer");
        sys_log(LOG_CRIT, "Enable Sniffer");
        tuya_hal_wifi_set_work_mode(WWM_SNIFFER);
        s_pSnifferCall = cb;
        if(1 == s_enable_sniffer)
        {
            return OPRT_OK;
        }
        s_enable_sniffer = 1;
        if(WIFI_MODULE == WIFI_SSV_PRIVATE)
        {
            pthread_create(&sniffer_thId, NULL, func_Sniffer_ssv, NULL);
        }
        else
        {
            pthread_create(&sniffer_thId, NULL, func_Sniffer, NULL);
        }
    }else
    {
        //PR_DEBUG("Disable Sniffer");
        sys_log(LOG_CRIT, "Disable Sniffer");
        s_enable_sniffer = 0;
        pthread_join(sniffer_thId, NULL);
        tuya_hal_wifi_set_work_mode(WWM_STATION);
    }
    return OPRT_OK;
}


int tuya_hal_wifi_sniffer_stop()
{
	
	if(1 == s_enable_sniffer && WIFI_MODULE == WIFI_SSV_PRIVATE)
	{
		GW_WF_NWC_FAST_STAT_T nc_type = 0;
		tuya_iot_wf_fast_get_nc_type(&nc_type);
		printf("\n ##### nc_type:%d \n",nc_type);
		if(nc_type ==  GWNS_FAST_UNCFG_AP)
		{
			sys_log(LOG_CRIT, "#####Disable Sniffer");
			s_enable_sniffer = 0;
			pthread_join(sniffer_thId, NULL);
		}
	}
	return 0;
}

// 获取WIFI的IP地址，改用c接口实现
OPERATE_RET tuya_hal_wifi_get_ip(IN CONST WF_IF_E wf,OUT NW_IP_S *ip)
{
        struct ifreq ifr;

        int socketfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (socketfd < 0) {
                printf("socket error\n");
                return OPRT_COM_ERROR;
        }

        strncpy(ifr.ifr_name,WLAN_DEV,IFNAMSIZ-1);

        if (ioctl(socketfd, SIOCGIFADDR, &ifr) == 0){ //获取ip地址
                strncpy(ip->ip,inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr),\
                sizeof(ip->ip));
        }

        if (ioctl(socketfd, SIOCGIFBRDADDR, &ifr) == 0){ //获取广播地址
                strncpy(ip->gw,inet_ntoa(((struct sockaddr_in *)&ifr.ifr_broadaddr)->sin_addr),\
                sizeof(ip->gw));
        }

        if (ioctl(socketfd, SIOCGIFNETMASK, &ifr) == 0){ //获取广播地址
                strncpy(ip->mask,inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr),\
                sizeof(ip->mask));
        }    
        close(socketfd);

        PR_DEBUG("WIFI Get  ip->ip:%s", ip->ip);
        return OPRT_OK;
}

// 获取WIFI的MAC地址
OPERATE_RET tuya_hal_wifi_get_mac(IN CONST WF_IF_E wf,INOUT NW_MAC_S *mac)
{
    FILE *pp = sys_popen("ifconfig "WLAN_DEV, "r");
    if(pp == NULL)
       return OPRT_COM_ERROR;

    char tmp[256];
    memset(tmp, 0, sizeof(tmp));
    while (fgets(tmp, sizeof(tmp), pp) != NULL)
    {
        char *pMACStart = strstr(tmp, "HWaddr ");
        if(pMACStart != NULL)
        {
            int x1,x2,x3,x4,x5,x6;
            sscanf(pMACStart + strlen("HWaddr "), "%x:%x:%x:%x:%x:%x",&x1,&x2,&x3,&x4,&x5,&x6);
            mac->mac[0] = x1 & 0xFF;
            mac->mac[1] = x2 & 0xFF;
            mac->mac[2] = x3 & 0xFF;
            mac->mac[3] = x4 & 0xFF;
            mac->mac[4] = x5 & 0xFF;
            mac->mac[5] = x6 & 0xFF;
            sys_pclose(pp);
            //PR_DEBUG("WIFI Get MAC %02X-%02X-%02X-%02X-%02X-%02X",
            //            mac->mac[0],mac->mac[1],mac->mac[2],mac->mac[3],mac->mac[4],mac->mac[5]);
            return OPRT_OK;
        }
    }
    sys_pclose(pp);
    return OPRT_COM_ERROR;
}

/***********************************************************
*  Function: hwl_wifi_station_get_ap_mac
*  Desc:     get wifi connect ap mac
*  Output:   mac: the return ap mac.
*  Return:   OPRT_OK: success  Other: fail
***********************************************************/
OPERATE_RET tuya_hal_wifi_station_get_ap_mac(INOUT NW_MAC_S *mac)
{
    return OPRT_OK;
}

// 当前无需实现
OPERATE_RET tuya_hal_wifi_set_mac(IN CONST WF_IF_E wf,IN CONST NW_MAC_S *mac)
{
    return OPRT_OK;
}

static char *debug_wifi_mode[] = {
    "WWM_LOWPOWER",     // wifi work in lowpower mode
    "WWM_SNIFFER",      // wifi work in sniffer mode
    "WWM_STATION",      // wifi work in station mode
    "WWM_SOFTAP",       // wifi work in ap mode
    "WWM_STATIONAP",    // wifi work in station+ap mode
};

// 设置当前WIFI工作模式
OPERATE_RET tuya_hal_wifi_set_work_mode(IN CONST WF_WK_MD_E mode)
{
    char tmpCmd[100] = {0};

    WF_WK_MD_E wifi_mode;
    tuya_hal_wifi_get_work_mode(&wifi_mode);
    if(wifi_mode == mode) {
        printf("** wifi already in %s mode, no SET MODE **\r\n",debug_wifi_mode[mode]);
        return OPRT_OK;
    }

    sys_shell("ifconfig %s up", WLAN_DEV);
    //printf(" $$$$$$  WIFI Set Mode:%d\r\n", mode);
    sys_log(LOG_CRIT, "$$$$$$  WIFI Set Mode:%d\r\n", mode);
    switch (mode)
    {
        case WWM_LOWPOWER:
        {
            //linux系统不关心低功耗
            break;
        }
        case WWM_SNIFFER:
        {
#ifdef WIFI_SMART_MODE
            //如果现在处于 ap 模式,先要将 ap 模式对应的进程清掉
            sys_shell("killall -9 hostapd");
            sys_shell("killall -9 udhcpd");
            
#ifndef WIFI_CHIP_7601
            sys_shell("ifconfig %s down", WLAN_DEV);
#endif
            sys_shell("iwconfig %s mode Monitor", WLAN_DEV);
#ifndef WIFI_CHIP_7601
            sys_shell("ifconfig %s up", WLAN_DEV);
#endif
            inform_robot_wifi_mode_state(WIFI_AP_CONFIG_DONE);    // 配网模式完成
#endif
            break;
        }
        case WWM_STATION:
        {

            if(WIFI_MODULE == WIFI_SSV_PRIVATE)
            {
                // 如果进入监听模式后，AppServer异常重启，需要重新加载驱动，恢复网卡为STA模式
                if (wifi_mode == WWM_SNIFFER && s_enable_sniffer == -1)
                {
                    sys_shell("%s restart", SHELL_STARTUP_WIFI);
                    sys_shell("%s restart", SHELL_STARTUP_NETWORK);
                }
            }

#ifndef WIFI_CHIP_7601
            sys_shell("ifconfig %s down", WLAN_DEV);
#endif
            sys_shell("iwconfig %s mode Managed", WLAN_DEV);
#ifndef WIFI_CHIP_7601
            sys_shell("ifconfig %s up", WLAN_DEV);
#endif
            break;
        }
        case WWM_SOFTAP:
        {
        /* 涂鸦项目不支持切到 master 这个指令, 在 hwl_wf_ap_start 处理 */
#ifndef WIFI_CHIP_7601
            //sys_shell("ifconfig %s down", WLAN_DEV);
#endif
            //sys_shell("iwconfig %s mode Master", WLAN_DEV);
#ifndef WIFI_CHIP_7601
            //sys_shell("ifconfig %s up", WLAN_DEV);
#endif
            break;
        }
        case WWM_STATIONAP:
        {//reserved
            break;
        }
        default:
        {
            break;
        }
    }

    PR_DEBUG("WIFI Set Mode:%d", mode);
    return OPRT_OK;
}

// 获取当前WIFI工作模式
OPERATE_RET tuya_hal_wifi_get_work_mode(OUT WF_WK_MD_E *mode)
{
    *mode = WWM_STATION;
    FILE *pp = sys_popen("iwconfig "WLAN_DEV, "r");
    if(pp == NULL)
        return OPRT_OK;

    char scan_mode[10] = {0};
    char tmp[256] = {0};
    while (fgets(tmp, sizeof(tmp), pp) != NULL)
    {
        char *pModeStart = strstr(tmp, "Mode:");
        if(pModeStart != NULL)
        {
            int x1,x2,x3,x4,x5,x6;
            sscanf(pModeStart + strlen("Mode:"), "%s ", scan_mode);
            break;
        }
    }
    sys_pclose(pp);

    *mode = WWM_STATION;
    if(strncasecmp(scan_mode, "Managed", strlen("Managed")) == 0)
        *mode = WWM_STATION;
    if(strncasecmp(scan_mode, "Master", strlen("Master")) == 0)
        *mode = WWM_SOFTAP;
    if(strncasecmp(scan_mode, "Monitor", strlen("Monitor")) == 0)
        *mode = WWM_SNIFFER;

    PR_TRACE("WIFI Get Mode:%d", *mode);
    {
        static WF_WK_MD_E last_mode;
        if(last_mode != *mode)
        {
          sys_log(LOG_CRIT, "WIFI Get Mode:%d", *mode);
          last_mode = *mode;
        }
    }
   
    return OPRT_OK;
}

// 断开当前WIFI网络的连接
OPERATE_RET tuya_hal_wifi_station_disconnect(VOID)
{
    PR_DEBUG("Disconnect WIFI Conn");

    // UserTODO
    kill_cmd("udhcpc");
    kill_cmd("wpa_supplicant");
    sys_shell("ifconfig "WLAN_DEV" "DEFAULT_IP_ADDR);
	pthread_mutex_lock(&wifi_connet_mutex);
	wifi_connet = false;
	pthread_mutex_unlock(&wifi_connet_mutex);

    return OPRT_OK;
}

// 获取当前WIFI联网的RSSI
OPERATE_RET tuya_hal_wifi_station_get_conn_ap_rssi(OUT SCHAR_T *rssi)
{
    *rssi = 99;
    FILE *pp = sys_popen("iwconfig "WLAN_DEV, "r");
    if(pp == NULL)
        return OPRT_COM_ERROR;

    char tmp[128] = {0};
    while (fgets(tmp, sizeof(tmp), pp) != NULL)
    {
        /* 查找signal  */
        char *pSIGNALStart = strstr(tmp, "Quality=");
        if(pSIGNALStart != NULL)
        {
            int x = 0;
            int y = 0;
            sscanf(pSIGNALStart + strlen("Quality="), "%d/%d",&x,&y);
            *rssi = x * 100/ (y+1);
            break;
        }
    }
    sys_pclose(pp);

    //PR_DEBUG("Get Conn AP RSSI:%d", *rssi);
    sys_log(LOG_CRIT, "Get Conn AP RSSI:%d", *rssi);
    return OPRT_OK;
}

void init_wifi_module_mutex(void)
{
    if(pthread_mutex_init(&wifi_restart_mutex,NULL) != 0)
    {
        printf("ERROR: wifi pthread_mutex_init fail\r\n");
    }

    printf("wifi pthread_mutex_init success\r\n");
    init_udhcpd_ty_conf_file();
}

static void restart_wifi_module(void)
{
    printf("##### udhcpc restart ##### \r\n");
    sys_shell("killall -9 wpa_supplicant 2>/dev/null");
    sys_shell("killall -9 udhcpc 2>/dev/null");
	
    sys_shell("killall -9 ping 2>/dev/null");
    sys_shell("killall -9 wifi_scan 2>/dev/null");

    pthread_mutex_lock(&wifi_connet_mutex); 
	wifi_connet = false;
	pthread_mutex_unlock(&wifi_connet_mutex); 
	
    sys_shell("%s restart", SHELL_STARTUP_WIFI);
    sys_shell("ifconfig %s down",WLAN_DEV);
    sys_shell("iwconfig %s mode Managed",WLAN_DEV);
	sys_shell("ifconfig %s up %s",WLAN_DEV,DEFAULT_IP_ADDR);
	sys_shell("wpa_supplicant -Dnl80211 -B -i" WLAN_DEV " -c" WPA_SUPPLICANT_CONF); 

#if 0
	sys_shell("udhcpc -i " WLAN_DEV " -s " UDHCPC_SCRIPT " &");
#else	
	pthread_t thread_wifi_connect;
	pthread_attr_t attr;
	pthread_attr_init(&attr);//初始化线程属性
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);//设置线程属性为分离状态
	int ret = pthread_create(&thread_wifi_connect, &attr,func_wifi_station_connect, NULL);
	if (ret)
	{
		sys_log(LOG_CRIT, "restart_wifi_module create thread func_wifi_station_connect ERROR! \n");
		sys_shell("udhcpc -i " WLAN_DEV " -s " UDHCPC_SCRIPT " -T 2 -t 5 -A 10 -b ");
		pthread_mutex_lock(&wifi_connet_mutex);
		wifi_connet = true;
		pthread_mutex_unlock(&wifi_connet_mutex);
	}
#endif
    printf("#####       udhcpc set end\r\n");
}

static void restart_wifi_module_timer(void)
{
	bool flag = iot_state_cloud_ready();
	if(!robot_get_wifi_config_state() && flag == false)
	{
    	restart_wifi_module();
	}
#if 0
    if (sys_timer_start_with_period(pTimer, 1000*60*10) == -1)
    {
        sys_timer_delete(pTimer);
        printf("3 sys_timer_start ERROR\n");
        return -1;
    }
#endif
}

//#define MAX_COUNT 20      //断网后重启 wifi 模块, udhcpc 的次数, 20*30=600s=10min 后不处理
//#define RECOUNT_SECOND  30      //断网后重启 wifi 模块, udhcpc 的秒数
// 获取当前WIFI联网状态  临时采用ping baidu的方法实现测试
OPERATE_RET tuya_hal_wifi_station_get_status(OUT WF_STATION_STAT_E *stat)
{
    // UserTODO
    static BOOL_T s_wf_station_state = FALSE;
    BOOL_T wf_state = FALSE;
    NW_IP_S new_ip;
    int mutex_ret = 0;
    int tmp_shell_ret = 0;
    
    memset(&new_ip, 0, sizeof(new_ip));

    wf_state = iot_state_cloud_ready();
	
    if(wf_state) {
        *stat =  WSS_GOT_IP;
    } else {
        	tuya_hal_wifi_get_ip(WF_STATION, &new_ip);
		pthread_mutex_lock(&wifi_connet_mutex);
	        if(new_ip.ip[0] != 0 && strcmp(new_ip.ip, DEFAULT_IP_ADDR) != 0 && wifi_connet) {
				#if 0
	            tmp_shell_ret = sys_shell("ping www.baidu.com -c 1 -w 2");
	            printf("wait dns shell_ret : %d\r\n", tmp_shell_ret);
	            if(tmp_shell_ret != 0)
	            {
	                *stat =  WSS_CONN_FAIL;
	            }
	            else
	            {
	                *stat =  WSS_GOT_IP;
	            }
				#else
				printf("\n\n ***** get ip success, ip:%s ***** \n\n",new_ip.ip);
				*stat =  WSS_GOT_IP;		
				#endif
	        } else {
	            *stat =  WSS_CONN_FAIL;
	        }
		pthread_mutex_unlock(&wifi_connet_mutex);
    }
    {
        static WF_STATION_STAT_E last_stat;
        if(last_stat != *stat) 
        {
            PR_DEBUG("Curr WIFI Stat:%d", *stat);
            sys_log(LOG_CRIT, "Curr WIFI Stat:%d", *stat);
        }
        last_stat = *stat;
    }
   
    /* trylock */
    mutex_ret = pthread_mutex_trylock(&wifi_restart_mutex); 
    if(0 != mutex_ret)
    {
        printf(" *** last wifi restart no end, not need repeat lock %d*** \r\n",mutex_ret);
        return OPRT_OK;
    }
    
    /* 下面的程序用于断网恢复后 ip 地址失效(不存在)的情况, 需要重启 udhcpc */
    if(!wf_state)
    {
        //printf("## stat=%d, last_state=%d ##\r\n", *stat, s_wf_station_state);
        if(s_wf_station_state)
        {
            if(NULL != pTimer)
            {
                printf("2 wifi timer restart\n");
                sys_timer_delete(pTimer);
                pTimer = NULL;
            }
            /* 断网后 1 分钟后重连 */
            printf("### create restart_wifi_module_timer ###\r\n");
            pTimer = sys_timer_create(60*1000, restart_wifi_module_timer);
            if (!pTimer)
            {
                pthread_mutex_unlock(&wifi_restart_mutex);
                printf("2 sys_timer_create ERROR\n");
                return -1;
            }
            
            if (sys_timer_start(pTimer) == -1)
            {
                sys_timer_delete(pTimer);
                pTimer = NULL;
                pthread_mutex_unlock(&wifi_restart_mutex);
                printf("2 sys_timer_start ERROR\n");
                return -1;
            }
            
        }

    }
    else
    {
        if(NULL != pTimer)
        {
            printf("1 wifi timer restart\n");
            sys_timer_delete(pTimer);
            pTimer = NULL;
        }
    }
    s_wf_station_state = wf_state;
    mutex_ret = pthread_mutex_unlock(&wifi_restart_mutex);
    if(0 != mutex_ret)
    {
        printf("ERROR: wifi lock pthread_mutex_unlock error:%d\n", mutex_ret);
    }
    return OPRT_OK;
}

static void tmp_hostapd_write(void)
{
    char tmp_config[256];
    int cnt = 0;
    memset(tmp_config, 0, sizeof(tmp_config));
    FILE *pFile = fopen(TMP_HOSTAPD_CONF, "w+");
    cnt = sprintf(tmp_config,"interface=%s\n","wlan0");
    cnt += sprintf(tmp_config+cnt,"driver=%s\n","nl80211");
    cnt += sprintf(tmp_config+cnt,"ssid=%s\n","SmartLife-6329");
    cnt += sprintf(tmp_config+cnt,"channel=%s\n","1");
    cnt += sprintf(tmp_config+cnt,"hw_mode=%s\n","g");
    cnt += sprintf(tmp_config+cnt,"ignore_broadcast_ssid=%s\n","0");
    cnt += sprintf(tmp_config+cnt,"macaddr_acl=%s\n","0");
    cnt += sprintf(tmp_config+cnt,"auth_algs=%s\n","1");

    //printf( "Output:\n%s\ncharacter count = %d\n", tmp_config, cnt);
    sys_log(LOG_CRIT, "Output:\n%s\ncharacter count = %d\n", tmp_config, cnt);
    fwrite(tmp_config, strlen(tmp_config),1,pFile);
    fclose(pFile);
    return;
}

static void checkSSID(const char *filename) {
    const int suffix_pos = strlen(SSID_SEARCH_PREFIX);
    char ssid[256], mac[18], *retc;
    int curpos;
    FILE *pFile = fopen(filename, "rt+");
    memset(ssid, 0, sizeof(ssid));
    do {
        curpos = ftell(pFile);
        //ret = fscanf(pFile, SSID_SEARCH_PREFIX"%s\n", ssid);
        retc = fgets(ssid, sizeof(ssid), pFile);
        if(memcmp(ssid, SSID_SEARCH_PREFIX, suffix_pos) == 0) {
            break;
        }
    }while(retc != NULL);
    if(retc != NULL && sys_get_mac(mac, sizeof(mac)) > 0) {
        char *suffix = &ssid[suffix_pos];
        if(suffix[0] != mac[12] || suffix[1] != mac[13]
            || suffix[2] != mac[15] || suffix[3] != mac[16]) {
            suffix[0] = mac[12];
            suffix[1] = mac[13];
            suffix[2] = mac[15];
            suffix[3] = mac[16];
            fseek(pFile,curpos,SEEK_SET);
            fwrite(ssid, suffix_pos+4, 1, pFile);
        }
    }
    fclose(pFile);
}

#define NO_PASSWAD_AP   // 没有有密码的兼容配网
// AP配网模式下开启热点
OPERATE_RET tuya_hal_wifi_ap_start(IN CONST WF_AP_CFG_IF_S *cfg)
{
    if(cfg != NULL) {
        //PR_DEBUG("Start AP SSID:%s, passwd:%s", cfg->ssid, cfg->passwd);
       sys_log(LOG_CRIT, "Start AP SSID:%s, passwd:%s", cfg->ssid, cfg->passwd);
    }
#ifdef NO_PASSWAD_AP 
    // 没有密码的兼容配网
    tmp_hostapd_write();
    checkSSID(TMP_HOSTAPD_CONF);
#else
    checkSSID(HOSTAPD_CONF);
#endif
    /* 调用 hostapd 前先调用一下 iwconfig 切换一下工作模式 */
    sys_shell("iwconfig %s mode Managed", WLAN_DEV);
    
    sys_shell("killall -9 hostapd");
    sys_shell("killall -9 udhcpd");
#ifdef NO_PASSWAD_AP
    // 没有密码的兼容配网
    sys_shell("hostapd -B " TMP_HOSTAPD_CONF);
#else
    sys_shell("hostapd -B " HOSTAPD_CONF);
#endif
    sys_shell("udhcpd -f " UDHCPD_CONF " &");
    robot_set_wifi_config_state();
#ifdef WIFI_SMART_MODE
    inform_robot_wifi_mode_state(WIFI_AP_CONFIG_DONE);    // 配网模式完成
#endif
    return OPRT_OK;
}

// AP配网模式下停止热点
OPERATE_RET tuya_hal_wifi_ap_stop(VOID)
{
    //PR_DEBUG("Stop Ap Mode");
     sys_log(LOG_CRIT, "Stop Ap Mode");
    // UserTODO
    kill_cmd("hostapd");
    kill_cmd("udhcpd");

    return OPRT_OK;
}

int tuya_hal_wifi_set_country_code(const COUNTRY_CODE_E ccode)
{
    PR_DEBUG("Country Code:%d", ccode);
    return OPRT_OK;
}

OPERATE_RET tuya_hal_wifi_lowpower_enable(VOID)
{
    PR_DEBUG("Enable Low Power ..");

    return OPRT_OK;
}

OPERATE_RET tuya_hal_wifi_lowpower_disable(VOID)
{
    PR_DEBUG("Disble Low Power ..");
    return OPRT_OK;
}

int tuya_hal_wifi_register_recv_mgnt_callback(bool enable, WIFI_REV_MGNT_CB recv_cb)
{
    return OPRT_OK;
}

OPERATE_RET tuya_hal_wifi_set_socket_broadcast_all(IN CONST INT_T socket_fd, IN CONST bool enable)
{
    return OPRT_OK;
}

OPERATE_RET tuya_hal_wifi_close_concurrent_ap(VOID)
{
    return OPRT_OK;
}

OPERATE_RET tuya_hal_wifi_send_mgnt(CONST UINT8_T *buf, const UINT_T len)
{
    return OPRT_OK;
}

int tuya_hal_wifi_get_bssid(uint8_t mac[6])
{
    return OPRT_OK;
}

int tuya_hal_set_wifi_lp_mode(const bool en, const unsigned int dtim)
{
    return OPRT_OK;   
}
