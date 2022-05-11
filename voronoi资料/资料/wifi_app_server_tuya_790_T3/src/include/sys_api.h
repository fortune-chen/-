#ifndef __SYS_API_H__ 
#define __SYS_API_H__

#include <stdint.h>

#define FILE_PATH_MAX_LEN       128
#define FILE_NAME_MAX_LEN       64
#define ROBOT_CMD_DEBUG         0xff

#define APPSERVER_VERSION_FILE  "/root/app_server/appserver_version"
#define APPSERVER_CONFIG_DIR    "/root/app_server/appserver_config"
#define APPSERVER_CONFIG_FILE   "/root/app_server/appserver_config/config_menu"
#define APPSERVER_SYS_LOG_FILE  "/tmp/appserver_sys_log"

// Log Level
enum
{
    LOG_DBG,        // ������Ϣ
    LOG_INFO,       // ��ͨ��Ϣ
    LOG_WARN,       // ������Ϣ
    LOG_CRIT,       // �ؼ���Ϣ

};
	
#if 0
enum
{
    WIFI_REALTCK,      //
    WIFI_SSV6152,      //�Ϸ����
};
#else
enum
{
    WIFI_REALTCK,      //
    WIFI_SSV_PRIVATE, //南硅WiFi使用私有接口
    WIFI_SSV_PUBLIC,  //南硅WiFi使用公共接口
};
#endif

// �ı���ɫ����ת������
#define LOG_CLRSTR_NONE         "\033[0m"               // ��
#define LOG_CLRSTR_RED          "\033[0;32;31m"         // ��ɫ
#define LOG_CLRSTR_GREEN        "\033[0;32;32m"         // ��ɫ
#define LOG_CLRSTR_BLUE         "\033[0;32;34m"         // ��ɫ
#define LOG_CLRSTR_DARK_GRAY    "\033[1;30m"            // ���
#define LOG_CLRSTR_CYAN         "\033[0;36m"            // ����
#define LOG_CLRSTR_PURPLE       "\033[0;35m"            // ��ɫ
#define LOG_CLRSTR_BROWN        "\033[0;33m"            // ��ɫ
#define LOG_CLRSTR_YELLOW       "\033[1;33m"            // ��ɫ
#define LOG_CLRSTR_WHITE        "\033[1;37m"            // ��ɫ

#define FILE_MAX                (100 * 1024)            // 100K

#define LOG_FILE_LEVEL          LOG_INFO                // ���浽��־�ļ���LOG�ȼ�

extern int WIFI_MODULE;


int sys_uart_open(const char *dev_name);
int sys_uart_set(int fd, int baud, int c_flow, int data_bits, int stop_bits, char parity);
int sys_uart_set_timeout(int fd, int msec);
int sys_uart_read(int fd, char *buf, int len);
int sys_uart_write(int fd, const char *buf, int len);
void sys_uart_flush_input(int fd);
void sys_uart_flush_output(int fd);
int sys_uart_close(int fd);

int sys_system(char *command);
FILE* sys_popen(const char* cmdstring, const char *type);
int sys_pclose(FILE *fp);

int sys_shell(const char *fmt, ...);
int sys_shell_result_to_int(const char *fmt, ...);

int sys_shell_result_to_char(char * path, int length, const char *fmt, ...);
int sys_is_process_exist(char *name);

int sys_get_mac(char *mac, int len);
int sys_get_ip(char *ip);

int sys_cat_file(char *file, char *buf, int size);

int sys_get_md5(char *file, char *md5);
int sys_file_check_md5(char *file, char *str);

int sys_str_del_eol(char *line);
int sys_strcat(char *dest, char *src);
int sys_str_cnt(const char *p, const char chr);

int round_float_to_int(float f);

int sys_get_system_uptime(void);
int sys_get_mem_free(void);
int sys_get_disk_free(void);

int sys_version_write(char *pVersion);
void sys_version_read(void);
void sys_version_delete(void);
void sys_is_and_create_dir(void);
void write_appserver_config_record(char *file);

int read_appserver_config_record(char *file, int len);
int read_raw_data(char *filename, char **buf, int *p_len);
void free_raw_data(char *p_buf);
int if_config_file_exit(void);
void delete_raw_file(char *file);
int write_raw_data(char *filename, char *data, int len);
int is_allow_to_upgrade(void);
int recv_packet(uint32_t type, uint8_t *data);
int recv_data(uint32_t type, uint32_t size, uint8_t *data);
void sys_delete_db_file(void);

void sys_debug_usb_log_check(void);
uint8_t sys_tx_checksum(uint8_t *buf,int  len);
void sys_log(int level, const char *fmt, ...);
int read_wi_mode_value(void);

extern int set_adebug_log(void);

int sys_grep_string_from_file(const char *file, const char *str);
int sys_get_firmware_partition_exist_flag(void);
void sys_set_firmware_partition_writeable(int enable);
int sys_get_firmware_partition_free(void);




#endif // !__SYS_API_H__ 
