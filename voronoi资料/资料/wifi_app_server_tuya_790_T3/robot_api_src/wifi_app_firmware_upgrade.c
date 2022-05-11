#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <libgen.h>

#include "sys_api.h"
#include "robot_api_inner.h"
#include "wifi_app_server.h"
#include "wifi_app_voice_mgr.h"
#include "wifi_app_firmware_upgrade.h"

#define UPGRADE_APP_SERVER_PATH             "/tmp/firmware/wifi_app_server_tuya"
#define FIRMWARE_FILE_BACKUP                "/root/firmware/arobot/backup"
#define UPGRADE_PATH                        "/tmp/firmware/firmware_upgrader"
#define STRINGS_GREP                        "/root/firmware/arobot"


// Firmware Backup 路径
#define FIRMWARE_FILE_BACKUP_FIRMWARE       "/root/firmware/arobot/backup/firmware.bak"
#define FIRMWARE_FILE_BACKUP_OTHER          "/root/firmware/arobot/backup/other.bak"

#define UPGRADE_FILE_BACKUP_FIRMWARE        "/root/firmware.bak"
#define UPGRADE_FILE_BACKUP_OTHER           "/root/other.bak"

//Firmware tmp Backup 路径
#define UPGRADE_FILE_TMP_BACKUP_FIRMWARE    "/tmp/firmware.bak"
#define UPGRADE_FILE_TMP_BACKUP_OTHER       "/tmp/other.bak"


static UPGRADE_MGR upgrade_mgr;

#ifdef APP_GRIT
static int get_app_path(int type, char *buffer, int size)
{
    int ret;
    FILE *p = NULL;

    if (!buffer || !size)
    {
        return -1;
    }

    switch (type)
    {
        case UPGRADE_TYPE_GRIT_CLIENT:
            p = sys_popen(SHELL_STARTUP_GRIT_CLIENT" yg_path", "r");
            break;
        default:
            p = NULL;
            break;
    }

    if (!p)
    {
        printf("cannot get app path, type=%d\n", type);
        return -1;
    }

    memset(buffer, 0, size);
    if (fgets(buffer, size, p))
    {
        sys_str_del_eol(buffer);
        ret = 0;
    }
    else
    {
        ret = -1;
    }

    sys_pclose(p);

    return ret;
}

static int wifi_app_firmware_upgrade_grit_client(const char *firmware_file)
{
    char file[FILE_PATH_MAX_LEN];
    char fname[FILE_NAME_MAX_LEN];
    char tmp[FILE_PATH_MAX_LEN];
    char *str;
    int ret;
    UPGRADE_STATUS upgrade_status;

    // 判断愚公固件包是否存在
    if (!firmware_file || access(firmware_file, R_OK) != 0)
    {
        return -1;
    }

    // 解压固件包
    strncpy(file, firmware_file, FILE_PATH_MAX_LEN);
    strncpy(fname, basename(file), FILE_NAME_MAX_LEN);
    strncpy(tmp, dirname(file), FILE_PATH_MAX_LEN);

    // 将升级文件移动至tmp目录中处理
    if (strncmp(tmp, "/tmp", 4) != 0)
    {
        sys_shell("mv -f %s /tmp", firmware_file);
        strcpy(tmp, "/tmp");
    }

    str = strrchr(fname, '.');
    if (strncmp(str, ".gz", 3) == 0)
    {
        sys_shell("gzip -d %s/%s", tmp, fname);
        *str = '\0';
        str = strrchr(fname, '.');
    }
    if (strncmp(str, ".tar", 4) == 0)
    {
        sys_shell("tar -xf %s/%s -C /tmp", tmp, fname);
        sys_shell("rm -f %s/%s", tmp, fname);
    }

    // 是否存在ygclient
    snprintf(file, FILE_PATH_MAX_LEN, "/tmp/%s", UPGRADE_FILE_NAME_GRIT_CLIENT);
    if (access(file, R_OK) != 0)
    {
        printf("ygclient is not exist(%s)\n", file);
        return -1;
    }

    sys_set_firmware_partition_writeable(1);

    // 升级ygclient
    do
    {
         // 获取ygclient路径
        if (sys_get_firmware_partition_exist_flag())
        {
            strncpy(tmp, FIRMWARE_PATH_GRIT_CLIENT, sizeof(tmp));
        }
        else
        {
            ret = get_app_path(UPGRADE_TYPE_GRIT_CLIENT, tmp, sizeof(tmp));
            if (ret || strncmp(tmp, "/", 1) != 0 || access(tmp, R_OK) != 0)
            {
                strncpy(tmp, APP_PATH_GRIT_CLIENT, sizeof(tmp));
            }
        }
        
        //inform_upgrade_state(UPGRADE_STATE_GOING);
        upgrade_status.upgrade_type     = UPGRADE_TYPE_GRIT_CLIENT;
        upgrade_status.upgrade_status   = UPGRADE_GOING;
        upgrade_status.upgrade_progress = 0;
        inform_upgrade_progress(&upgrade_status);

        ret = sys_shell("rm -f %s/%s", tmp, UPGRADE_FILE_NAME_GRIT_CLIENT);
        if (ret)
        {
            break;
        }

        upgrade_status.upgrade_progress = 50;
        inform_upgrade_progress(&upgrade_status);

        ret = sys_shell("cp -f %s %s", file, tmp);
        if (ret)
        {
            break;
        }

        upgrade_status.upgrade_progress = 90;
        inform_upgrade_progress(&upgrade_status);

        if (sys_get_firmware_partition_exist_flag())
        {
            sys_shell("rm -f %s/%s", APP_PATH_GRIT_CLIENT, UPGRADE_FILE_NAME_GRIT_CLIENT);
            sys_shell("ln -s %s/%s %s/%s", FIRMWARE_PATH_GRIT_CLIENT, UPGRADE_FILE_NAME_GRIT_CLIENT, APP_PATH_GRIT_CLIENT, UPGRADE_FILE_NAME_GRIT_CLIENT);
        }

        sys_shell("sync");
        
    } while (0);

    sys_set_firmware_partition_writeable(0);

    if (!ret)
    {
        printf("upgrade ygclient complete\n");
        inform_upgrade_state(UPGRADE_STATE_COMPLETE);
        upgrade_status.upgrade_type     = UPGRADE_TYPE_DEFAULT;
        upgrade_status.upgrade_status   = UPGRADE_COMPLETE;
        upgrade_status.upgrade_progress = 100;
        inform_upgrade_progress(&upgrade_status);

        // 5s后重启应用
        sleep(5);
        sys_shell(SHELL_STARTUP_HOSLAM " stop");
        sys_shell(SHELL_STARTUP_GRIT_CLIENT " stop");
        sys_shell("/root/bin/reset380");
        sys_shell(SHELL_STARTUP_HOSLAM " start");
        sys_shell(SHELL_STARTUP_GRIT_CLIENT " start");
    }
    else
    {
        upgrade_status.upgrade_type     = UPGRADE_TYPE_DEFAULT;
        upgrade_status.upgrade_status   = UPGRADE_FAILED;
        upgrade_status.upgrade_progress = 100;
        inform_upgrade_progress(&upgrade_status);
    }

    return ret;
}

static int wifi_app_firmware_upgrade_grit_check(const char *firmware_path, const char *firmware_file)
{
    char file[FILE_PATH_MAX_LEN];
    char fname[FILE_NAME_MAX_LEN];
    char tmp[FILE_PATH_MAX_LEN];
    char md5[64];
    char *str;
    FILE *pfile;

    // 判断愚公固件包是否存在
    if (!firmware_file || access(firmware_file, R_OK) != 0)
    {
        return -1;
    }

    // 解压固件包
    strncpy(file, firmware_file, FILE_PATH_MAX_LEN);
    strncpy(fname, basename(file), FILE_NAME_MAX_LEN);
    strncpy(tmp, dirname(file), FILE_PATH_MAX_LEN);

    // 将升级文件移动至tmp目录中处理
    if (strncmp(tmp, "/tmp", 4) != 0)
    {
        sys_shell("mv -f %s /tmp", firmware_file);
        strcpy(tmp, "/tmp");
    }

    str = strrchr(fname, '.');
    if (strncmp(str, ".gz", 3) == 0)
    {
        sys_shell("gzip -d %s/%s", tmp, fname);
        *str = '\0';
        str = strrchr(fname, '.');
    }
    if (strncmp(str, ".tar", 4) == 0)
    {
        sys_shell("tar -xf %s/%s -C /tmp", tmp, fname);
        sys_shell("rm -f %s/%s", tmp, fname);
    }

    // 是否存在ygclient
    snprintf(file, FILE_PATH_MAX_LEN, "/tmp/%s", UPGRADE_FILE_NAME_GRIT_CLIENT);
    if (access(file, R_OK) != 0)
    {
        printf("ygclient is not exist(%s)\n", file);
        return -1;
    }

    // 把ygclient移动至升级目录，便于升级程序统一升级
    if (sys_shell("mv -f %s %s", file, firmware_path))
    {
        printf("move ygclient ERROR\n");
        return -1;
    }

    snprintf(file, FILE_PATH_MAX_LEN, "%s/%s", firmware_path, UPGRADE_FILE_NAME_GRIT_CLIENT);
    if (access(file, R_OK) != 0)
    {
        printf("ygclient is not exist after move(%s)\n", file);
        return -1;
    }

    // 计算MD5
    memset(md5, 0, sizeof(md5));
    if (sys_get_md5(file, md5) != 0)
    {
        printf("ygclient get md5 failed(%s)\n", file);
        return -1;
    }

    // 把MD5信息写入文件
    snprintf(file, FILE_PATH_MAX_LEN, "%s/%s", firmware_path, UPGRADE_FILE_NAME_MD5);
    if (access(file, R_OK) != 0)
    {
        printf("md5 file is not exist (%s)\n", file);
        return -1;
    }

    pfile = fopen(file, "a+");
    if (!file)
    {
        printf("md5 file open error (%s)\n", file);
        return -1;
    }

    snprintf(tmp, sizeof(tmp), "\nygclient                :%s\n", md5);
    if (fwrite(tmp, 1, strlen(tmp), pfile) != strlen(tmp))
    {
        fclose(pfile);
        return -1;
    }

    fflush(pfile);
    fclose(pfile);

    return 0;
}
#endif // APP_GRIT

static int wifi_app_firmware_upgrade_file_check(char *path, char *file_name)
{
    char tmp[FILE_PATH_MAX_LEN];
    char md5[64], buff[128], *str;
    FILE *file;

    snprintf(tmp, FILE_PATH_MAX_LEN, "%s/%s", path, file_name);
    if (access(tmp, R_OK) != 0)
    {
        return 0;
    }

    snprintf(tmp, FILE_PATH_MAX_LEN, "%s/%s", path, UPGRADE_FILE_NAME_MD5);
    if (access(tmp, R_OK) != 0)
    {
        printf("md5 file is not exist (%s)\n", tmp);
        return 0;
    }

    file = fopen(tmp, "r");
    if (!file)
    {
        printf("md5 file open error (%s)\n", tmp);
        return 0;
    }

    memset(md5, 0, sizeof(md5));
    while (!feof(file))
    {
        fgets(buff, 128, file);
        if (strncmp(buff, file_name, strlen(file_name)) == 0)
        {
            str = strrchr(buff, ':');
            strncpy(md5, str + 1, 64);
            break;
        }
    }
    fclose(file);

    snprintf(tmp, FILE_PATH_MAX_LEN, "%s/%s", path, file_name);
    if (sys_file_check_md5(tmp, md5))
    {
        return 1;
    }

    printf("md5 check failed (%s, md5:%s)\n", tmp, md5);
    return 0;
}

static int wifi_app_firmware_upgrade_start(const char *firmware_file, const char *other_file)
{
    char path[FILE_PATH_MAX_LEN];

#ifdef APP_GRIT
    if (!firmware_file && other_file)
    {
        return wifi_app_firmware_upgrade_grit_client(other_file);
    }
#endif // APP_GRIT

    if (!firmware_file)
    {
        return -1;
    }

    // 将升级文件移动至tmp目录中解压
    sys_shell("mv -f %s /tmp/tmp.tar.gz", firmware_file);
    sys_shell("gzip -d /tmp/tmp.tar.gz");
    sys_shell("tar -xf /tmp/tmp.tar -C /tmp");
    sys_shell("rm -f /tmp/tmp.tar");

    snprintf(path, FILE_PATH_MAX_LEN, "/tmp/%s", UPGRADE_DIR_NAME_FIRMWARE);
    if (access(path, R_OK) != 0)
    {
        printf("firmware_upgrade: folder is not exist(%s)\n", path);
        return -1;
    }

#ifdef APP_GRIT
    if (other_file)
    {
        if (wifi_app_firmware_upgrade_grit_check(path, other_file))
        {
            printf("wifi_app_firmware_upgrade_grit_check ERROR\n");
            return -1;
        }
    }
#endif // APP_GRIT

    // 是否存在升级程序
    if (wifi_app_firmware_upgrade_file_check(path, FILE_NAME_FIRMWARE_UPGRADER) == 0)
    {
        printf("firmware_upgrade: upgrader is not exist(%s)\n", path);
        sys_shell("rm -rf %s", path);
        return -1;
    }

    // TODO: 解密Firmware Upgrade
    if (sys_get_firmware_partition_exist_flag() )
    {
    	#if 0
        if( 0 == access(UPGRADE_APP_SERVER_PATH, R_OK))
        {
            int i = 0;
            printf("begin\n");
            i = sys_shell("strings -f %s | grep \"%s\"",UPGRADE_APP_SERVER_PATH,FIRMWARE_FILE_BACKUP);
            if( i != 0)
            {
                printf("no find /root/firmware/arobot/back,old app_server\n");
                printf("end fail r = %d\n",i);
                sys_shell("rm %s",path);
                return -1;
            }
            printf("end success r = %d\n",i);
        }
        else
        {
            printf("%s is not exist\n",UPGRADE_APP_SERVER_PATH);
        }
		#endif
        if( 0 == access(UPGRADE_PATH, R_OK))
        {
            int i = 0;
            printf("begin\n");
            i = sys_shell("strings -f %s | grep \"%s\"",UPGRADE_PATH,STRINGS_GREP);
            if( i != 0)
            {
                printf("no find /root/firmware/arobot/,old upgrade\n");
                printf("end fail r = %d\n",i);
                sys_shell("rm %s",path);
                return -1;
            }
            printf("end success r = %d\n",i);
        }
        else
        {
            printf("%s is not exist\n",UPGRADE_PATH);
        }
    }

    // 启动固件升级进程
    upgrade_mgr.start = 1;
    strncpy(upgrade_mgr.firmware_path, path, FILE_PATH_MAX_LEN);

    sys_shell("chmod 777 %s/%s", path, FILE_NAME_FIRMWARE_UPGRADER);
    sys_shell("%s/%s &", path, FILE_NAME_FIRMWARE_UPGRADER);

    printf("firmware upgrader start\n");

    return 0;
}

static int wifi_app_firmware_upgrade_done(void)
{
    if (upgrade_mgr.start == 0)
    {
        return -1;
    }

    // 删除升级目录
    sys_shell("rm -rf %s", upgrade_mgr.firmware_path);
    printf("firmware_upgrade: delete upgrader\n");

    upgrade_mgr.start = 0;

    return 0;
}

// 清除量产烧录版本
static void wifi_app_firmware_upgrade_version_clear(void)
{
    sys_shell("echo > %s", FILE_FIRMWARE_VERSION);
    printf("wifi_app_firmware_upgrade_version_clear\n");
}

// 保存升级包到Flash，升级完成后删除，在升级过程中因意外断电导致升级失败时，可以重新升级
static void wifi_app_firmware_upgrade_file_backup_save(const char *firmware_file, const char *other_file)
{
    char *backup_firmware, *backup_other;

    printf("firmware_file = %s,other_file = %s\n", firmware_file,other_file);
    sys_set_firmware_partition_writeable(1);

    if (sys_get_firmware_partition_exist_flag())
    {
        printf("sys_get_firmware_partition_exist_flag = 1\n");
        backup_firmware = FIRMWARE_FILE_BACKUP_FIRMWARE;
        backup_other = FIRMWARE_FILE_BACKUP_OTHER;
    }
    else
    {
        backup_firmware = UPGRADE_FILE_BACKUP_FIRMWARE;
        backup_other = UPGRADE_FILE_BACKUP_OTHER;
    }
    if (firmware_file)
    {
        sys_shell("cp -f %s %s", firmware_file, backup_firmware);
    }
    if (other_file)
    {
        sys_shell("cp -f %s %s", other_file, backup_other);
    }
    
    if (sys_get_firmware_partition_exist_flag())
    {
        if (firmware_file)
        {
            sys_shell("ln -s %s %s", FIRMWARE_FILE_BACKUP_FIRMWARE, UPGRADE_FILE_BACKUP_FIRMWARE);
        }
        if (other_file)
        {
            sys_shell("ln -s %s %s", FIRMWARE_FILE_BACKUP_OTHER, UPGRADE_FILE_BACKUP_OTHER);
        }
    }

    sys_set_firmware_partition_writeable(0);

    printf("firmware_upgrade: copy upgrade file to flash\n");
}

static void wifi_app_firmware_upgrade_file_backup_delete(void)
{
    sys_shell("rm -f %s", UPGRADE_FILE_BACKUP_FIRMWARE);
    sys_shell("rm -f %s", UPGRADE_FILE_BACKUP_OTHER);
    if (sys_get_firmware_partition_exist_flag())
    {
        sys_set_firmware_partition_writeable(1);
        sys_shell("rm -f %s", FIRMWARE_FILE_BACKUP_FIRMWARE);
        sys_shell("rm -f %s", FIRMWARE_FILE_BACKUP_OTHER);
        sys_shell("sync");
        sys_set_firmware_partition_writeable(0);
    }
    printf("firmware_upgrade: delete upgrade file from flash\n");
}

static int wifi_app_firmware_upgrade_file_backup_is_exist(void)
{
	if (0 == access(UPGRADE_FILE_BACKUP_FIRMWARE, F_OK) || 0 == access(UPGRADE_FILE_BACKUP_OTHER,F_OK))
	{
		return 1;
	}
	else
	{	
	    if (sys_get_firmware_partition_exist_flag())
        {   
    	    if(0 == access(FIRMWARE_FILE_BACKUP_FIRMWARE,F_OK) || 0 == access(FIRMWARE_FILE_BACKUP_OTHER,F_OK))
            {
                printf("delete incomplete save firmware\n");
                sys_set_firmware_partition_writeable(1);
                sys_shell("rm -f %s", FIRMWARE_FILE_BACKUP_FIRMWARE);
                sys_shell("rm -f %s", FIRMWARE_FILE_BACKUP_OTHER);
                sys_shell("sync");
                sys_set_firmware_partition_writeable(0);
            }  
        }
		return 0;

	}
}

int wifi_app_firmware_upgrade_backup_check(void)
{
    int ret;
	char *firmwarepath = NULL;
    char *otherpath = NULL;
    
    //等待接收完upgrade的消息队列的信息
	sleep(8);
    
	if (wifi_app_firmware_upgrade_file_backup_is_exist())
	{
		if (0 == access(UPGRADE_FILE_BACKUP_FIRMWARE, F_OK))
		{
		    sys_shell("cp -f %s %s", UPGRADE_FILE_BACKUP_FIRMWARE, UPGRADE_FILE_TMP_BACKUP_FIRMWARE);
			firmwarepath = UPGRADE_FILE_TMP_BACKUP_FIRMWARE;
		}
		else
		{
			firmwarepath = NULL;
		}
		if (0 == access(UPGRADE_FILE_BACKUP_OTHER, F_OK))
		{
		    sys_shell("cp -f %s %s", UPGRADE_FILE_BACKUP_OTHER, UPGRADE_FILE_TMP_BACKUP_OTHER);
			otherpath = UPGRADE_FILE_TMP_BACKUP_OTHER;
		}
		else
		{
			otherpath = NULL;
		}
        
        printf("firmwarepath=%s, otherpath=%s\n", firmwarepath, otherpath);
        ret = wifi_app_firmware_upgrade_start(firmwarepath, otherpath);
        if (ret)
        {
            printf("backup_firmware_upgrade: start upgrader failed\n");
            wifi_app_firmware_upgrade_file_backup_delete();
        }
	}

    firmwarepath = NULL;
    otherpath = NULL;  
}

int wifi_app_firmware_upgrade_msg_parse(int type, const char *msg)
{
    int ret = 0;

    UPGRADE_STATUS *upgrade_status = (UPGRADE_STATUS*)msg;

    UPGRADE_MSG upgrade_msg;

    if (upgrade_mgr.start == 0)
    {
        upgrade_mgr.start = 1;
        snprintf(upgrade_mgr.firmware_path, FILE_PATH_MAX_LEN, "/tmp/%s", UPGRADE_DIR_NAME_FIRMWARE);
        printf("firmware_upgrade: recv last msg, backup:%d\n", wifi_app_firmware_upgrade_file_backup_is_exist());
    }

    printf("firmware_upgrade: recv status [%d] [%d] [%d]\n", type, upgrade_status->upgrade_status, upgrade_status->upgrade_progress);

    switch (type)
    {
        case UPGRADE_TYPE_DEFAULT:
            if (upgrade_status->upgrade_status == UPGRADE_READY)
            {
                //inform_upgrade_state(UPGRADE_STATE_GOING);
                upgrade_msg.upgrade_cmd = UPGRADE_START;
                strncpy(upgrade_msg.firmware_path, upgrade_mgr.firmware_path, FILE_PATH_MAX_LEN);
                send_msg_to_upgrade_app(UPGRADE_TYPE_DEFAULT, (const char *)&upgrade_msg, sizeof(UPGRADE_MSG));
            }
            else if (upgrade_status->upgrade_status == UPGRADE_COMPLETE)
            {
                inform_upgrade_state(UPGRADE_STATE_COMPLETE);
                wifi_app_firmware_upgrade_version_clear();
                wifi_app_firmware_upgrade_file_backup_delete();
                wifi_app_firmware_upgrade_done();
            }
            else if (upgrade_status->upgrade_status == UPGRADE_FAILED)
            {
                inform_upgrade_state(UPGRADE_STATE_FAILED);
				wifi_app_firmware_upgrade_file_backup_delete();
                wifi_app_firmware_upgrade_done();
            }
            break;

        case UPGRADE_TYPE_AFW:
        case UPGRADE_TYPE_HOSLAM:
        case UPGRADE_TYPE_APP_SERVER:
        case UPGRADE_TYPE_GRIT_CLIENT:
        case UPGRADE_TYPE_VOICE:
            break;

        default:
            printf("firmware_upgrade: UNKNOWN TYPE(%d)\n", type);
            break;
    }

    // 将升级状态发送给远程App
    inform_upgrade_progress(upgrade_status);

    return ret;
}

static void request_stop_monitor(void)
{
    // int ret;

    // ret = sys_shell("/root/app_server/daemon -c app_server -r 0x80000002");
    // if (ret != 1)
    // {
    //     sys_shell("%s stop", SHELL_STARTUP_DAEMON);
    // }
}

static void request_start_monitor(void)
{
    // int ret;
    // ret = sys_shell("/root/app_server/daemon -c app_server -r 0x80000004");
    // if (ret != 1)
    // {
    //     sys_shell(SHELL_STARTUP_DAEMON " start");
    // }
}
static int file_header_parse_v1(FILE *file, file_info_t *info)
{
    int ret, size, fd, total;
    uint16_t crc16;
    FILE *file_tmp;
    uint8_t tmp[1024];
    int save_crc;

    // Header
    size = info->ident.size - sizeof(info->ident);
    if (size != sizeof(info->header.v1))
    {
        sys_log(LOG_CRIT,"file parse v1: size ERROR\n");
        return -1;
    }

    ret = fread((void*)&info->header.v1, 1, size, file);
    if (ret != size)
    {
        sys_log(LOG_CRIT,"file parse v1: fread header failed, ret=%d\n", ret);
        return -1;
    }

#if 0
    // CRC16
    save_crc = info->ident.crc16;
    info->ident.crc16 = 0;
    crc16 = sys_get_crc16_xmodem((const char*)&info->ident.magic, info->ident.size);
    info->ident.crc16 = save_crc;
    if (crc16 != info->ident.crc16)
    {
        log("file parse v1: crc16 ERROR, 0x%X - 0x%X\n", crc16, info->ident.crc16);
        return -1;
    }
#endif

    sys_log(LOG_INFO,"file parse v1: file flags=0x%X, mode=%d, size=%d, hash=%s, key=%s, path=%s, startup=%s\n",
        info->header.v1.flags, info->header.v1.mode, info->header.v1.size, info->header.v1.hash, info->header.v1.key, info->header.v1.path, info->header.v1.startup);

    // Read file data
    file_tmp = fopen(FILE_PARSE_TEMP, "wb+");
    if (!file_tmp)
    {
        sys_log(LOG_CRIT,"file parse v1: fopen error\n");
        return -1;
    }

    while (!feof(file))
    {
        memset(tmp, 0, sizeof(tmp));
        size = fread(tmp, 1, sizeof(tmp), file);
        if (size == 0 || size > sizeof(tmp))
        {
            sys_log(LOG_CRIT,"file parse v1: file read failed\n");
            ret = -1;
            break;
        }

        ret = fwrite(tmp, 1, size, file_tmp);
        if (ret != size)
        {
            sys_log(LOG_CRIT,"file parse v1: file write failed\n");
            ret = -1;
            break;
        }

        ret = 0;
    }

    if (ret)
    {
        fclose(file_tmp);
        remove(FILE_PARSE_TEMP);
        return -1;
    }

    // Write file data
    fseek(file, 0, SEEK_SET);
    fseek(file_tmp, 0, SEEK_SET);
    total = 0;
    while (!feof(file_tmp))
    {
        memset(tmp, 0, sizeof(tmp));
        size = fread(tmp, 1, sizeof(tmp), file_tmp);
        if (size == 0 || size > sizeof(tmp))
        {
            sys_log(LOG_CRIT,"file parse v1: tmp file read failed\n");
            ret = -1;
            break;
        }

        ret = fwrite(tmp, 1, size, file);
        if (ret != size)
        {
            sys_log(LOG_CRIT,"file parse v1: tmp file write failed\n");
            ret = -1;
            break;
        }

        total += size;
        ret = 0;
    }

    fclose(file_tmp);
    remove(FILE_PARSE_TEMP);
    if (ret)
    {
        return -1;
    }

#if 1
    if (total != info->header.v1.size)
    {
        sys_log(LOG_CRIT,"file parse v1: file size error,total =%d,info->header.v1.size = %d\n",total,info->header.v1.size);
        //return -1;
    }
#endif

    fd = fileno(file);
    ftruncate(fd, total);
    fflush(file);
    fdatasync(fd);

    return 0;
}


int file_format_parse(const char *file, file_info_t *info)
{
    FILE *pf;
    int ret, size;

    if (!file || !info)
    {
        sys_log(LOG_CRIT,"file parse: params error\n");
        return -1;
    }

    pf = fopen(file, "rb+");
    if (!pf)
    {
        sys_log(LOG_CRIT,"file parse: fopen error\n");
        return -1;
    }

    // Ident
    size = sizeof(info->ident);
    ret = fread((void*)&info->ident, 1, size, pf);
    if (ret != size)
    {
        sys_log(LOG_INFO,"file parse: fread ident failed, ret=%d\n", ret);
        fclose(pf);
        return -1;
    }

    sys_log(LOG_INFO,"file parse: magic=0x%X, version=%d, machien=%d, size=%d, crc16=0x%04X\n",
        *(uint32_t*)info->ident.magic, info->ident.version, info->ident.machine, info->ident.size, info->ident.crc16);
    
#if 0
    // Magic
    if (info->ident.magic[0] != FILE_HEADER_MAGIC_1
        || info->ident.magic[1] != FILE_HEADER_MAGIC_2
        || info->ident.magic[2] != FILE_HEADER_MAGIC_3
        || info->ident.magic[3] != FILE_HEADER_MAGIC_4)
    {
        log("file parse: magic ERROR\n");
        fclose(pf);
        return -1;
    }

    // Machine
    if (info->ident.machine != usb_protocol_get_machine())
    {
        log("file parse: machine ERROR\n");
        fclose(pf);
        return -1;
    }
#endif

    switch (info->ident.version)
    {
        case 0x0001:
            ret = file_header_parse_v1(pf, info);
            break;

        default:
            sys_log(LOG_CRIT,"file parse: unknow version\n");
            ret = -1;
            break;
    }

    fclose(pf);

    return ret;
}

int wifi_app_firmware_upgrade(const char *firmware_file, const char *other_file)
{
    int ret;
    int state;
    file_info_t info;
    if (!firmware_file && !other_file)
    {
        return -1;
    }

    ret = is_allow_to_upgrade();
    if (ret != 1)
    {
        if (ret == 0)
        {
            // 电量状态不满足时，提示“无法升级”
            state = UPGRADE_STATE_NOT_ALLOWED;
        }
        else
        {
            // 内存不足时，提示“升级失败”
            state = UPGRADE_STATE_FAILED;
        }

        if (firmware_file)
        {
            sys_shell("rm -f %s", firmware_file);
        }
        if (other_file)
        {
            sys_shell("rm -f %s", other_file);
        }
        inform_upgrade_state(state);
        ret = -1;
        return ret;
    }
    file_format_parse(firmware_file,&info);
    // 升级前退出hoslam
    inform_upgrade_state(UPGRADE_STATE_GOING);
    sleep(2);
    request_stop_monitor();
    sys_shell("%s stop", SHELL_STARTUP_HOSLAM);

    ret = 0;
    state = UPGRADE_STATE_FAILED;
    do
    {
        // 升级前判断电量
        ret = is_allow_to_upgrade();
        if (ret != 1)
        {
            if (ret == 0)
            {
                // 电量状态不满足时，提示“无法升级”
                state = UPGRADE_STATE_NOT_ALLOWED;
            }
            else
            {
                // 内存不足时，提示“升级失败”
                state = UPGRADE_STATE_FAILED;
            }

            if (firmware_file)
            {
                sys_shell("rm -f %s", firmware_file);
            }
            if (other_file)
            {
                sys_shell("rm -f %s", other_file);
            }

            ret = -1;
            break;
        }
 
        // 检查固件升级包是否存在
        if (firmware_file && access(firmware_file, R_OK) != 0)
        {
            printf("firmware_upgrade: file is not exist(%s)\n", firmware_file);
            ret = -1;
            break;
        }
        if (other_file && access(other_file, R_OK) != 0)
        {
            printf("firmware_upgrade: file is not exist(%s)\n", other_file);
            ret = -1;
            break;
        }

        // 保存升级包到Flash，升级完成后删除，在升级过程中因意外断电导致升级失败时，可以重新升级
        wifi_app_firmware_upgrade_file_backup_save(firmware_file, other_file);

        // 解压固件包，并运行升级程序
        ret = wifi_app_firmware_upgrade_start(firmware_file, other_file);
        if (ret)
        {
            printf("firmware_upgrade: start upgrader failed\n");
        }
    }
    while (0);
    printf("ota test ret = %d\n", ret);
    if (ret)
    {
        printf("sdfadsfasfsdaf\n");
        // 重启hoslam
        wifi_app_firmware_upgrade_file_backup_delete();
        sys_shell("/root/bin/reset380");
        sleep(2);
        sys_shell(SHELL_STARTUP_HOSLAM " start");
        sleep(2);
        request_start_monitor();
        inform_upgrade_state(state);
    }
    else
    {
        request_start_monitor();
    }

    return ret;
}
