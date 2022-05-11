#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "sys_api.h"
#include "tuya_uuid.h"
#include "robot_api.h"
#include <stdlib.h>
#include "factory_config_public.h"
typedef struct {
    char *key;
    char *value;
} item_t;

// Tuya 测试用
static char tuya_uuid[] = "tuya049fe5ce87f5614c";
static char tuya_authkey[]= "Md13PqBKnfQ98EELickM7Kn1txRPU56Y";
static char tuya_product_key[]= "8lw1fml6uycujtzc";
/*
 *去除字符串右端空格
 */
char *strtrimr(char *pstr) {

    int i;
    i = strlen(pstr) - 1;
    while (isspace(pstr[i]) && (i >= 0)) {
        pstr[i--] = '\0';
    }
    return pstr;
}

/*
 *去除字符串左端空格
 */
char *strtriml(char *pstr) {

    int i = 0,j;
    j = strlen(pstr) - 1;
    while (isspace(pstr[i]) && (i <= j)) {
        i++;
    }
    if (0 < i) {
        strcpy(pstr, &pstr[i]);
    }
    return pstr;
}

/*
 *去除字符串两端空格
 */
char *strtrim(char *pstr) {
    char *p;
    p = strtrimr(pstr);
    return strtriml(p);
}

/*
 *从配置文件的一行读出key或value,返回item指针
 *line--从配置文件读出的一行
 */
int get_item_from_line(char *line, item_t *item) {

    char *p = strtrim(line);
    int len = strlen(p);
    if(len <= 0 || p[0]=='#') {
        return 1;//空行 或者 注释
    } else {
        char *p2 = strchr(p, '=');
        *p2++ = '\0';
//        item->key = (char *)malloc(strlen(p) + 1);
//        item->value = (char *)malloc(strlen(p2) + 1);
//        strcpy(item->key,p);
//        strcpy(item->value,p2);
        item->key = p;
        item->value = p2;
    }
    return 0;//查询成功
}


static int find_tuya_uuid(const char *match, char *file) {
    FILE *pf;
    char buff[128];
    char cmd[20];
    sprintf(cmd, "find %s", match);
    pf = sys_popen(cmd, "r");
    if (pf == NULL) {
        return -1;
    }
    memset(buff, 0, sizeof(buff));
    if (fgets(buff, sizeof(buff), pf) == NULL) {
        sys_pclose(pf);
        return -1;
    }
    sys_pclose(pf);
    strtrim(buff);
    //文件判断是否存在
    if (access(buff, R_OK) != 0)
    {
        return -1;
    }
    strcpy(file, buff);
    return 0;
}

int tuya_cp_triplet(void)
{
    int ret;
    char *file;

    char mac[PROTOCOL_WIFI_MAC]; 
    char pid[PROTOCOL_TUYA_PRODUCT_ID];
    char uuid[PROTOCOL_TUYA_UUID];
    char key[PROTOCOL_TUYA_AUTHKEY];

    char mac_tmp[PROTOCOL_WIFI_MAC];  
    char pid_tmp[PROTOCOL_TUYA_PRODUCT_ID];
    char uuid_tmp[PROTOCOL_TUYA_UUID];
    char key_tmp[PROTOCOL_TUYA_AUTHKEY];

    char buf[256];
    int offset = 0;
    FILE *pf = NULL;
	char *p;
	char *str;

    uint8_t tmp[PROTOCOL_WIFI_MAC + PROTOCOL_TUYA_UUID + PROTOCOL_TUYA_PRODUCT_ID + PROTOCOL_TUYA_AUTHKEY];

    p = getenv(DEV_FACTORY_CONFIG);
    if(!p)
    {      
        return 0;
    }
    memset(mac, 0, sizeof(mac));
    memset(uuid, 0, sizeof(uuid));
    memset(pid, 0, sizeof(pid));
    memset(key, 0, sizeof(key));
    memset(tmp, 0, sizeof(tmp));

    memset(mac_tmp, 0, sizeof(mac_tmp));
    memset(pid_tmp, 0, sizeof(pid_tmp));
    memset(uuid_tmp, 0, sizeof(uuid_tmp));
    memset(key_tmp, 0, sizeof(key_tmp));
 
    ret = factory_config_init();
    if(ret)
    {
        sys_log(LOG_CRIT,"factory_config_init error\n");
        return -1;
    }

    ret = factory_config_get(TUYA_CONFIG_OWNER,TUYA_CONFIG_KEY,(void *)tmp,sizeof(tmp));
    if(ret <= 0)
    {
        sys_log(LOG_CRIT,"factory_config_get error\n");
        factory_config_free();
        return -1;
    }
    ret = factory_config_free();
    if(ret)
    {
        sys_log(LOG_CRIT,"factory_config_free error\n");
        return -1;
    }

     memcpy(mac, tmp + offset, PROTOCOL_WIFI_MAC);
     offset += PROTOCOL_WIFI_MAC;
     memcpy(pid, tmp + offset, PROTOCOL_TUYA_PRODUCT_ID);
     offset += PROTOCOL_TUYA_PRODUCT_ID;
     memcpy(uuid, tmp + offset, PROTOCOL_TUYA_UUID);
     offset += PROTOCOL_TUYA_UUID;
     memcpy(key, tmp + offset, PROTOCOL_TUYA_AUTHKEY);
     offset += PROTOCOL_TUYA_AUTHKEY;
	 sys_log(LOG_INFO,"local tuya cert info: mac=%s pid=%s uuid=%s  key=%s\n", mac, pid, uuid, key);
    
     file = FILE_TUYA_CERT;
     
     sys_log(LOG_INFO,"file = %s\n",file);
     // 打开涂鸦IoT认证信息文件
     pf = fopen(file, "r");
     if (pf)
     {
         // 解析文件
         while (fgets(buf, sizeof(buf), pf))
         {
             sys_str_del_eol(buf);

            if (strncmp(buf, KEY_TUYA_PID, strlen(KEY_TUYA_PID)) == 0)
             {
                 str = strrchr(buf, '=');
                 strncpy(pid_tmp, str + 1, sizeof(pid_tmp));
             }
             else if (strncmp(buf, KEY_TUYA_UUID, strlen(KEY_TUYA_UUID)) == 0)
             {
                 str = strrchr(buf, '=');
                 strncpy(uuid_tmp, str + 1, sizeof(uuid_tmp));
             }
             else if (strncmp(buf, KEY_TUYA_KEY, strlen(KEY_TUYA_KEY)) == 0)
             {
                 str = strrchr(buf, '=');
                 strncpy(key_tmp, str + 1, sizeof(key_tmp));
             }
         }
         fclose(pf);
         sys_log(LOG_INFO,"local tuya cert info: pid_tmp=%s uuid_tmp=%s key_tmp=%s\n", pid_tmp, uuid_tmp, key_tmp);
     }
     else
     {
         sys_log(LOG_CRIT,"open tuya cert failed\n");
     }
     
     if(0 == strncmp(uuid,uuid_tmp,sizeof(uuid)) && 0 == strncmp(pid,pid_tmp,sizeof(pid)) && 0 == strncmp(key,key_tmp,sizeof(key)))
     {
        printf("tuya uuid is same \n");
        return 0;
     }
     else
     {
        pf = fopen(file, "w+");
        if (!pf)
        {
            sys_log(LOG_CRIT,"fopen error");
            return -1;
        }    
        sprintf(buf, "%s=%s\n", KEY_TUYA_PID, pid);
        fwrite(buf, strlen(buf), 1, pf);
        sprintf(buf, "%s=%s\n", KEY_TUYA_UUID, uuid);
        fwrite(buf, strlen(buf), 1, pf);
        sprintf(buf, "%s=%s\n", KEY_TUYA_KEY, key);
        fwrite(buf, strlen(buf), 1, pf);
        fflush(pf);
        fclose(pf);
		sys_log(LOG_INFO,"sync success\n");
     }
}


int tuya_uuid_init(const char *file) {
    char line[256];
    if(file == NULL) {
        return -1;
    }
    
    if(find_tuya_uuid(file, line) < 0) {
        return -2;
    }
    printf("find %s\n", line);
    FILE *fp = fopen(line,"r");
    if(fp == NULL) {
        return -3;//文件打开错误
    }
    while (fgets(line, 255, fp)) {
        item_t item;
        get_item_from_line(line,&item);
        if(!strcmp(item.key,"UUID")) {
            strncpy(tuya_uuid,item.value, sizeof(tuya_uuid));
        } else if(!strcmp(item.key,"AUTHKEY")) {
            strncpy(tuya_authkey,item.value, sizeof(tuya_authkey));
        } else if(!strcmp(item.key,"PRODUCT_KEY")) {
            strncpy(tuya_product_key,item.value, sizeof(tuya_product_key));
        }
    }

    fclose(fp);
    return 0;//成功
}

char *tuya_uuid_get(void) {
    return tuya_uuid;
}

char *tuya_uuid_get_authkey(void) {
    return tuya_authkey;
}

char *tuya_get_product_key(void) {
    return tuya_product_key;
}
