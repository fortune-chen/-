#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <fcntl.h>
#include <math.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/sysinfo.h>
#include <errno.h>
#include <sys/wait.h>
#include <sys/vfs.h>
#include <sys/stat.h>
#include <stdarg.h>
#include <termios.h>


#include "md5c.h"
#include "sys_api.h"
#include "robot_api.h"

#define WLAN_DEV                    "wlan0"
#define SYS_SHELL_STARTUP_HOSLAM        "/root/etc/init.d/S51hoslam"
#define SYS_SHELL_STARTUP_HOSLAM        "/root/etc/init.d/S51hoslam"
#define SYS_WIFI                        "/root/etc/init.d/S60wifi"


#define SYSTEM_SHELL_MAX_LEN        4096
#define APPSERVER_VERSION_FILE      "/root/app_server/appserver_version"

static pid_t    *childpid = NULL;  /* ptr to array allocated at run-time */
static int      maxfd;  /* from our open_max(), {Prog openmax} */
static long     openmax = 0;
static FILE     *file_log;
int WIFI_MODULE;


/*
 * If OPEN_MAX is indeterminate, we're not
 * guaranteed that this is adequate.
 */
#define OPEN_MAX_GUESS 1024

int sys_uart_open(const char *dev_name)
{
    int fd;

    if (!dev_name)
    {
        return -1;
    }

    // 打开串口设备
    fd = open(dev_name, O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
        perror("open");
        return -1;
    }

    // 清除串口非阻塞标志
    if (fcntl(fd, F_SETFL, 0) < 0)
    {
        printf("fcntl failed!\n");
        return -1;
    }

    return fd;
}

int sys_uart_set(int fd, int baud, int c_flow, int data_bits, int stop_bits, char parity)
{
    struct termios options;

    // 获取串口属性
    if (tcgetattr(fd, &options) < 0)
    {
        perror("tcgetattr");
        return -1;
    }

    //printf("c_iflag:%x, c_oflag:%x, c_cflag:%x, c_lflag:%x, c_line:%c, c_ispeed:%d, c_ospeed:%d\n", 
    //        options.c_iflag, options.c_oflag, options.c_cflag, options.c_lflag, options.c_line, options.c_ispeed, options.c_ospeed);

    // 设置串口输入输出波特率
    switch (baud)
    {
        case 4800:
            cfsetispeed(&options, B4800);
            cfsetospeed(&options, B4800);
            break;
        case 9600:
            cfsetispeed(&options, B9600);
            cfsetospeed(&options, B9600);
            break;
        case 19200:
            cfsetispeed(&options, B19200);
            cfsetospeed(&options, B19200);
            break;
        case 38400:
            cfsetispeed(&options, B38400);
            cfsetospeed(&options, B38400);
            break;
        case 115200:
            cfsetispeed(&options, B115200);
            cfsetospeed(&options, B115200);
            break;
        default:
            printf("Unkown baud!\n");
            return -1;
    }

    // 设置控制模式
    options.c_cflag |= CLOCAL;          // 保证程序不占用串口
    options.c_cflag |= CREAD;           // 保证程序可以从串口中读取数据

    // 设置数据流控制
    switch (c_flow)
    {
        case 0: // 不进行流控制
            options.c_cflag &= ~CRTSCTS;
            break;
        case 1: // 进行硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
        case 2: // 进行软件流控制
            options.c_cflag |= IXON | IXOFF | IXANY;
            break;
        default:
            printf("Unkown c_flow!\n");
            return -1;
    }

    // 设置数据位
    switch (data_bits)
    {
        case 5:
            options.c_cflag &= ~CSIZE;  // 屏蔽其它标志位
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag &= ~CSIZE;  // 屏蔽其它标志位
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag &= ~CSIZE;  // 屏蔽其它标志位
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag &= ~CSIZE;  // 屏蔽其它标志位
            options.c_cflag |= CS8;
            break;
        default:
            printf("Unkown data_bits!\n");
            return -1;
    }

    // 设置校验位
    switch (parity)
    {
        // 无奇偶校验位
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB; // PARENB：产生奇偶位，执行奇偶校验
            options.c_iflag &= ~INPCK;  //INPCK：使奇偶校验起作用
            break;
        // 设为空格,即停止位为2位
        case 's':
        case 'S':
            options.c_cflag &= ~PARENB; // PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~CSTOPB; // CSTOPB：使用两位停止位
            break;
        // 设置奇校验
        case 'o':
        case 'O':
            options.c_cflag |= PARENB;  // PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag |= PARODD;  // PARODD：若设置则为奇校验,否则为偶校验
            options.c_cflag |= INPCK;   // INPCK：使奇偶校验起作用
            options.c_cflag |= ISTRIP;  // ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
            break;
        // 设置偶校验
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;  // PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~PARODD; // PARODD：若设置则为奇校验,否则为偶校验
            options.c_cflag |= INPCK;   // INPCK：使奇偶校验起作用
            options.c_cflag |= ISTRIP;  // ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
            break;
        default:
            printf("Unkown parity!\n");
            return -1;
    }

    // 设置停止位
    switch (stop_bits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB; // CSTOPB：使用两位停止位
            break;
        case 2:
            options.c_cflag |= CSTOPB;  // CSTOPB：使用两位停止位
            break;
        default:
            printf("Unkown stop_bits!\n");
            return -1;
    }

    options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

    // 设置输出模式为原始输出
    //options.c_oflag &= ~OPOST;          // OPOST：若设置则按定义的输出处理，否则所有c_oflag失效
	options.c_oflag &= ~(OPOST | ONLCR | OCRNL | ONOCR | ONLRET);

    // 设置本地模式为原始模式
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    /*
     *ICANON：允许规范模式进行输入处理
     *ECHO：允许输入字符的本地回显
     *ECHOE：在接收EPASE时执行Backspace,Space,Backspace组合
     *ISIG：允许信号
     */

    // 设置等待时间和最小接受字符
    options.c_cc[VTIME] = 0;            // 可以在select中设置
    options.c_cc[VMIN]  = 1;            // 最少读取一个字符

    // 如果发生数据溢出，只接受数据，但是不进行读操作
    tcflush(fd, TCIFLUSH);

    // 激活配置
    if (tcsetattr(fd, TCSANOW, &options) < 0)
    {
        perror("tcsetattr");
        return -1;
    }

    //printf("c_iflag:%x, c_oflag:%x, c_cflag:%x, c_lflag:%x, c_line:%c, c_ispeed:%d, c_ospeed:%d\n", 
    //        options.c_iflag, options.c_oflag, options.c_cflag, options.c_lflag, options.c_line, options.c_ispeed, options.c_ospeed);

    return 0;
}

int sys_uart_set_timeout(int fd, int msec)
{
    struct termios options;
    cc_t vtime;

    // 获取串口属性
    if (tcgetattr(fd, &options) < 0)
    {
        perror("tcgetattr");
        return -1;
    }

    vtime = msec / 100;
    if (vtime == 0)
    {
        vtime = 1;
    }

    // 设置等待时间和最小接受字符
    options.c_cc[VTIME] = vtime;
    options.c_cc[VMIN]  = 0;

     // 激活配置
    if (tcsetattr(fd, TCSANOW, &options) < 0)
    {
        perror("tcsetattr");
        return -1;
    }

    return 0;
}

int sys_uart_read(int fd, char *buf, int len)
{
    return read(fd, buf, len);
}

int sys_uart_write(int fd, const char *buf, int len)
{
    int write_count = 0;
     
    write_count = write(fd, buf, len);  
    if (len == write_count)  
	{  
		return write_count;  
	}
    else     
	{  
		tcflush(fd, TCOFLUSH);
		return -1;
    }
}

void sys_uart_flush_input(int fd)
{
    tcflush(fd, TCIFLUSH);
}

void sys_uart_flush_output(int fd)
{
    tcflush(fd, TCOFLUSH);
}

int sys_uart_close(int fd)
{
    if (fd == 0)
    {
        return -1;
    }

    close(fd);

    return 0;
}



static long open_max(void)
{
    if (openmax == 0) {      /* first time through */
        errno = 0;
        if ((openmax = sysconf(_SC_OPEN_MAX)) < 0) {
           if (errno == 0)
               openmax = OPEN_MAX_GUESS;    /* it's indeterminate */
           else
               printf("sysconf error for _SC_OPEN_MAX");
        }
    }
 
    return openmax;
}

FILE* sys_popen(const char* cmdstring, const char *type)
{
	int pfd[2];
	FILE *fp;
	pid_t	pid;
 
	if ((type[0]!='r' && type[0]!='w')||type[1]!=0)
	{
		errno = EINVAL;
		return NULL;
	}
 
    if (childpid == NULL) {     /* first time through */  
                /* allocate zeroed out array for child pids */  
        maxfd = open_max();  
        if ((childpid = (pid_t *)calloc(maxfd, sizeof(pid_t))) == NULL)  
            return NULL;  
    }
 
	if (pipe(pfd)!=0)
	{
		return NULL;
	}
	
	if ((pid = vfork())<0)
	{
		return NULL;   /* errno set by fork() */  
	}
	else if (pid == 0) {	/* child */
		if (*type == 'r')
		{
			close(pfd[0]);  
            if (pfd[1] != STDOUT_FILENO) {  
                dup2(pfd[1], STDOUT_FILENO);  
                close(pfd[1]);  
            } 			
		}
		else
		{
            close(pfd[1]);  
            if (pfd[0] != STDIN_FILENO) {  
                dup2(pfd[0], STDIN_FILENO);  
                close(pfd[0]);  
            }  			
		}
 
		/* close all descriptors in childpid[] */  
		for (int i = 0; i < maxfd; i++)  
		if (childpid[ i ] > 0)
			close(i);
 
        execl("/bin/sh", "sh", "-c", cmdstring, (char *) 0); 
        _exit(127);
	}

    if (*type == 'r') {
        close(pfd[1]);
        if ((fp = fdopen(pfd[0], type)) == NULL)
            return(NULL);
    } else {
        close(pfd[0]);
        if ((fp = fdopen(pfd[1], type)) == NULL)
            return(NULL);
    }
 
    childpid[fileno(fp)] = pid; /* remember child pid for this fd */
    return fp;
}

int sys_pclose(FILE *fp)
{
    int     fd, stat;  
    pid_t   pid;  
  
    if (childpid == NULL)  
        return(-1);     /* popen() has never been called */  
  
    fd = fileno(fp);  
    if ( (pid = childpid[fd]) == 0)  
        return(-1);     /* fp wasn't opened by popen() */  
  
    childpid[fd] = 0;  
    if (fclose(fp) == EOF)  
        return(-1);  
  
    while (waitpid(pid, &stat, 0) < 0)  
        if (errno != EINTR)  
            return(-1); /* error other than EINTR from waitpid() */  
  
    return(stat);   /* return child's termination status */  
 
}

int sys_system(char *command)
{
	int status, pid;
	__sighandler_t save_quit, save_int, save_chld;

	if (command == NULL)
		return 1;

	save_quit = signal(SIGQUIT, SIG_IGN);
	save_int = signal(SIGINT, SIG_IGN);
	save_chld = signal(SIGCHLD, SIG_DFL);

	if ((pid = vfork()) < 0) {
		signal(SIGQUIT, save_quit);
		signal(SIGINT, save_int);
		signal(SIGCHLD, save_chld);
		return -1;
	}
	
	if (pid == 0) {
		signal(SIGQUIT, SIG_DFL);
		signal(SIGINT, SIG_DFL);
		signal(SIGCHLD, SIG_DFL);

		execl("/bin/sh", "sh", "-c", command, (char *) 0);
		_exit(127);
	}
	
	/* Signals are not absolutly guarenteed with vfork */
	signal(SIGQUIT, SIG_IGN);
	signal(SIGINT, SIG_IGN);

	if (waitpid(pid, &status, 0) == -1)
		status = -1;

	signal(SIGQUIT, save_quit);
	signal(SIGINT, save_int);
	signal(SIGCHLD, save_chld);
	
	return status;
}

int sys_shell(const char *fmt, ...)
{
    int ret;
    char cmd[SYSTEM_SHELL_MAX_LEN];

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(cmd, SYSTEM_SHELL_MAX_LEN, fmt, ap); 
    va_end(ap);

    cmd[SYSTEM_SHELL_MAX_LEN - 1] = '\0';
 
    ret = sys_system(cmd);
    if (ret == -1)
    {
        // 创建子进程失败
        return -1;
    }
    if (!WIFEXITED(ret))
    {
        // shell脚本执行错误
        return -1;
    }
    // shell返回值
    ret = WEXITSTATUS(ret);

    return ret;
}

int sys_shell_result_to_int(const char *fmt, ...)
{
    int ret;
    int result = -1;
    char cmd[SYSTEM_SHELL_MAX_LEN];
    char buff[32];
    FILE *pf;
 
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(cmd, SYSTEM_SHELL_MAX_LEN, fmt, ap); 
    va_end(ap);

    cmd[SYSTEM_SHELL_MAX_LEN - 1] = '\0';

    pf = sys_popen(cmd, "r");
    if (pf == NULL)
    {
        return -1;
    }
    
    if (fgets(buff, sizeof(buff), pf) != NULL)
    {
        result = atoi(buff);
        if(result == 0 && buff[0] != '0')
        {
            result = -1;
        }        
    }

    sys_pclose(pf);

    return result;
}

int sys_shell_result_to_char(char * path, int length, const char *fmt, ...)
{
    int ret = -1;
    //int result = -1;
    char cmd[SYSTEM_SHELL_MAX_LEN];
    //char buff[128];
    FILE *pf;
 
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(cmd, SYSTEM_SHELL_MAX_LEN, fmt, ap); 
    va_end(ap);

    cmd[SYSTEM_SHELL_MAX_LEN - 1] = '\0';

    pf = sys_popen(cmd, "r");
    if (pf == NULL)
    {
        return -1;
    }
    
    if (fgets(path, length, pf) != NULL)
    {
        sys_str_del_eol(path);
        ret = 0;
    }

    sys_pclose(pf);

    return ret;
}


int sys_is_process_exist(char *name)
{
    FILE *pf;
    char buff[512];
    char cmd[SYSTEM_SHELL_MAX_LEN];

    sprintf(cmd, "ps -ef | grep %s | grep -v grep | wc -l", name);
    pf = sys_popen(cmd, "r");
    if (pf == NULL)
    {
        return -1;
    }
    
    if (fgets(buff, 512, pf) != NULL)
    {
        if (atoi(buff) >= 1)
        {
            sys_pclose(pf);
            return 1;
        }
    }
          
    sys_pclose(pf);

    return 0;
}

// 获取Mac地址
int sys_get_mac(char *mac, int len)
{
    struct ifreq ifreq;
    int sock, ret;

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        perror ("socket");
        return -1;
    }

    strcpy(ifreq.ifr_name, WLAN_DEV);

    if (ioctl(sock, SIOCGIFHWADDR, &ifreq) < 0)
    {
        perror ("ioctl");
        close(sock);
        return -1;
    }
    
    ret = snprintf(mac, len, "%02X:%02X:%02X:%02X:%02X:%02X", ifreq.ifr_hwaddr.sa_data[0], ifreq.ifr_hwaddr.sa_data[1], ifreq.ifr_hwaddr.sa_data[2],
                                                              ifreq.ifr_hwaddr.sa_data[3], ifreq.ifr_hwaddr.sa_data[4], ifreq.ifr_hwaddr.sa_data[5]);

    close(sock);

    return ret;
}

// 获取IP地址
int sys_get_ip(char *ip)
{
    FILE *pp = sys_popen("ifconfig "WLAN_DEV, "r");
    if (pp == NULL)
    {
        return -1;
    }

    char tmp[256];
    char *str = NULL;
    memset(tmp, 0, sizeof(tmp));
    while (fgets(tmp, sizeof(tmp), pp) != NULL)
    {
        str = strstr(tmp, "inet addr:");
        if (str != NULL)
        {
            break;
        }
    }
    sys_pclose(pp);

    if (str != NULL)
    {
        sscanf(str + strlen("inet addr:"), "%s", ip);
        return 0;
    }
    
    return -1;
}

// 从文件中读取第一行字符串
int sys_cat_file(char *file, char *buf, int size)
{
    FILE    *pf;
    char    *ret = NULL;

    if (!file || !buf)
    {
        return -1;
    }

    if (access(file, R_OK) != 0)
    {
        return -1;
    }

    pf = fopen(file, "r");
    if (!pf)
    {
        return -1;
    }
    
    ret = fgets(buf, size, pf);
    if (!ret)
    {
        return -1;
    }

    return sys_str_del_eol(buf);
}

int sys_get_md5(char *file, char *md5)
{
    int ret, i;
    char digest[16];

    memset(digest, 0, sizeof(digest));
    ret = MD5File(file, (unsigned char*)digest);
    if (ret)
    {
        return -1;
    }

    for (i = 0; i < 16; i++)
    {
        snprintf(md5 + 2 * i, 3, "%X%X", digest[i] / 16, digest[i] % 16);
    }

    return 0;
}

int sys_file_check_md5(char *file, char *str)
{
    int ret, i;
    char md5[64], tmp[64];
    char *p;

    memset(md5, 0, sizeof(md5));
    ret = sys_get_md5(file, md5);
    if (ret)
    {
        return 0;
    }

    strncpy(tmp, str, 64);
    p = tmp;
    while (*p)
    {
        if (*p >= 'a' && *p <= 'z')
        {
            *p = *p + ('A' - 'a');
        }
        p++;
    }

    if (strncmp(md5, tmp, 32) == 0)
    {
        return 1;
    }

    return 0;
}

int sys_str_del_eol(char *line)
{
    char *ch = line;

    if (!ch)
    {
        return -1;
    }

    while (*ch)
    {
        if (*ch == '\r' || *ch == '\n')
        {
            *ch = '\0';
            break;
        }
        ch++;
    }

    return 0;
}

int sys_strcat(char *dest, char *src)
{
    int i = 0;

    if (!dest || !src)
    {
        return -1;
    }

    while (1)
    {
        if (dest[i] == ' ' || dest[i] == '\0')
        {
            strcpy(dest+i, src);
            return 0;
        }
        i++;
    }

    return -1;
}

int sys_str_cnt(const char *p, const char chr)
{	
	int count = 0, i = 0;

	while (*p)
	{
		if (*p == chr)
        {
			++count;
        }
		++p;
	}

	return count;
}

int round_float_to_int(float f)
{
    int i;
    
	if (f < 0)
	{
		i = (int)(f - 0.5);
	}
	else
	{
		i = (int)(f + 0.5);
	}
    
    return i;
}

int sys_get_system_uptime(void)
{
    struct sysinfo info;
    int ret;

    ret = sysinfo(&info);
    if (ret)
    {
        perror("sysinfo");
        return -1;
    }

    return info.uptime;
}

// 获取系统剩余内存
int sys_get_mem_free(void)
{
    struct sysinfo info;
 	char buf[60];
	FILE *fp;
    int ret;
	int cached, free; // 单位：Byte

    // 获取系统信息
    ret = sysinfo(&info);
    if (ret)
    {
        perror("sysinfo");
        return -1;
    }

    // 从/proc/meminfo中获取cached
	fp = fopen("/proc/meminfo", "r");
    if (!fp)
    {
        return -1;
    }
	while (fgets(buf, sizeof(buf), fp) != NULL)
    {
		if (sscanf(buf, "Cached: %lu %*s\n", &cached) == 1)
        {
			break;
        }
	}
	fclose(fp);

    cached *= 1024;
    free = info.freeram + info.bufferram + cached;
    return free;
}

// 获取磁盘剩余空间
int sys_get_disk_free(void)
{
    struct statfs info;
    int free;

    statfs("/root", &info);  
    free = info.f_bfree * info.f_bsize;

    return free;
}

int sys_version_write(char *pVersion)
{
    
    if (!pVersion)
    {
        return -1;
    }
    
    if (access(APPSERVER_VERSION_FILE, R_OK) != -1)
    {
        sys_shell("echo \"%s\" >> %s", pVersion, APPSERVER_VERSION_FILE);
    }
    else
    {
        sys_shell("echo \"%s\" > %s", pVersion, APPSERVER_VERSION_FILE);
    }
    return 0;
}

void sys_version_read(void)
{
        
    if (access(APPSERVER_VERSION_FILE, R_OK) != -1)
    {
        sys_shell("cat %s", APPSERVER_VERSION_FILE);        
    }
    else
    {
        printf("can't exist file %s\r\n", APPSERVER_VERSION_FILE);
    }
}

void sys_version_delete(void)
{
    if (access(APPSERVER_VERSION_FILE, R_OK) != -1)
    {
        printf("****************  Delete Version file ********************\r\n");
        sys_shell("rm -rf %s", APPSERVER_VERSION_FILE);        
    }
    else
    {
        printf("can't delete non-existent file %s\r\n", APPSERVER_VERSION_FILE);
    }
}

void sys_is_and_create_dir(void)
{
    if(access(APPSERVER_CONFIG_DIR, F_OK) == 0)
    {
        printf("APPSERVER_CONFIG_DIR EXISITS!\n");
    }    
    else
    {             
        printf("APPSERVER_CONFIG_DIR DOESN'T EXISIT!\n");
        sys_shell("mkdir -vp %s", APPSERVER_CONFIG_DIR);        
    }
     
    return;
}

static int write_appserver_config(char *argv, char *content)
{
    int ret;
    int tmp_shell_ret;

    if (access(APPSERVER_CONFIG_FILE, R_OK) == 0)
    {
        
        tmp_shell_ret = sys_shell("cat %s | grep '%s'", APPSERVER_CONFIG_FILE, argv);
        if(tmp_shell_ret == 1)
        {
            ret = sys_shell("echo %s=%s >> %s", argv, content, APPSERVER_CONFIG_FILE);
        }
        else if(tmp_shell_ret == 0)
        {
            printf("modify config =>%s %s\n",argv, content);
            //sys_shell("echo %s=%s", argv, content);
            ret = sys_shell("sed -i 's;^%s=.*;%s=%s;' %s", argv, argv, content, APPSERVER_CONFIG_FILE);
        }
        else
        {
            printf("prase config file fail\n");
        }
    }
    else
    {
        printf("create config file\n");
        ret = sys_shell("echo %s=%s > %s", argv, content, APPSERVER_CONFIG_FILE);
    }

    return ret;
}

static int read_appserver_config(char *argv, char *value, int len)
{
    //char value[128];
    int ret = 0;
    int ret1 = 0;
    //int path_id = 0;

    do 
    {
        // 判断配置文件
        if (access(APPSERVER_CONFIG_FILE, R_OK) != 0)
        {
            //sys_log(LOG_CRIT, "access S00voice FAILED");
            printf("2 access appserver_config FAILED\n");
            ret = -1;
            break;
        }
        memset(value, 0, len);
        ret = sys_shell_result_to_char(value, len, "cat %s | grep '%s' | awk -F '=' '{print $2}'|head -1", APPSERVER_CONFIG_FILE, argv);
        printf("### appserver_config read %s ret:%d ###\n", value, ret);
    }
    while (0);
    
    return ret;
}

int if_config_file_exit(void)
{
    if (access(APPSERVER_CONFIG_FILE, R_OK) != 0)
    {
        //sys_log(LOG_CRIT, "access S00voice FAILED");
        printf("2 access appserver_config FAILED\n");
        return -1;
    }
    else
    {
        return 0;
    }
}

void write_appserver_config_record(char *file)
{
    char *arg = "record_file";
    int ret = write_appserver_config(arg, file);    //record_file="file"
    if(ret != 0)
    {
        printf("write_appserver_config fail\n");
    }
    return;
}


int read_appserver_config_record(char *file, int len)
{
    char *arg = "record_file";
    int ret = read_appserver_config(arg, file, len);    //record_file="file"
    if(ret != 0)
    {
        printf("read_appserver_config fail\n");
    }
    return ret;
}

int write_raw_data(char *filename, char *data, int len)
{
    FILE *pFile = fopen(filename, "w+");
    if(pFile == NULL)
    {
        printf("write_raw_data fopen faile\n");
        return -1;
    }

    fwrite(data, len,1,pFile);
    fclose(pFile);
    return 0;
}

/* 这里读取文件并且 malloc 对应的大小空间, 故数据用完要记得free */
char *p_read = NULL;
int read_raw_data(char *filename, char **buf, int *p_len)
{
    FILE *fp = fopen(filename, "r");
    //char *p;
    if (NULL == fp) {
        printf("read_raw_data fopen faile\n");
        return -1;
    }
    /*获取文件字节大小size*/
    fseek(fp, 0, SEEK_END);
    //The ftell() function obtains the current value  of  the  file  position indicator for the stream pointed to by stream
    int size = ftell(fp);

    if(size > 0)
    {
      printf("size of the fiel is %d\n",size);
      p_read = (char *)malloc(size);
    }
    /*读文件内容并且打印出来*/
    fseek(fp, 0, SEEK_SET);
    fread(p_read, size, 1, fp);
    fclose(fp);
    *buf = p_read;
    *p_len = size;

    printf("the size of the file is:%d\n",size);
    //free(p);
    return 0;
}

void delete_raw_file(char *file)
{
    int ret;
    ret = sys_shell("rm -rf %s", file);
}

void free_raw_data(char *p_buf)
{
    if(p_read != p_buf)
    {
        printf("warnning: if free the right buffer?(%x != %x)\n", p_read, p_buf);
    }
    if(p_read != NULL)
    {
        free(p_read);
        p_read = NULL;
        printf("free_raw_data\n");
    }
    return;
}

void sys_delete_db_file(void)

{
    int ret;
    ret = sys_shell("rm tuya_enckey.db tuya_user.db tuya_user.db_bak");
    sys_shell("ls");
    printf("sys_delete_db_file\n");
}

// 在打开U盘LOG之后，定时检查U盘是否可读写
void sys_debug_usb_log_check(void)
{
    static int check_flag = -1;
    static int tmp;
    int app_debug;

    FILE *pf = NULL;
 	char buf[128];

    if (check_flag == -1)
    {
        // 首次进入，判断是否开启Appserver Log
        pf = fopen(SYS_SHELL_STARTUP_HOSLAM, "r");
        if (!pf)
        {
            printf( "open S50app failed\n");
            return;
        }

        tmp = 0;
        while (fgets(buf, sizeof(buf), pf) != NULL)
        {
            if (sscanf(buf, "\texport HOSLAM_ENABLE_DEBUG=%d\n", &app_debug) == 1)
            {
                break;
            }
            else if (sscanf(buf, "USB_LOG=%d\n", &app_debug) == 1)
            {
                break;
            }
            else
            {
                
            }
        }
        fclose(pf);
        printf("app_debug = %d\n", app_debug);
        if (app_debug == 1)
        {
			check_flag = 1;
            printf("hoslam usb log on\n");
			if(set_adebug_log() == 0)
			{
				u_log_flag = 1;
			}	
    
        }

        if(app_debug == 2)
        {
            check_flag = 0;
            tf_card = 1;
			
			if(set_adebug_log() == 0)
			{
				u_log_flag = 1;
			}
        }
    }
    else if (check_flag == 1)
    {
        tmp++;
        printf("tmp = %d", tmp);
        if (tmp >= 3)   // 30s检查一次
        {
            buf[0] = '\0';
            pf = fopen("/mnt/.usb_check", "a+");
            if (pf)
            {
                buf[0] = 'A';
                fwrite(buf, 1, 1, pf);
                fclose(pf);
                buf[0] = '\0';
                pf = fopen("/mnt/.usb_check", "r");
                if (pf)
                {
                    fread(buf, 1, 1, pf);
                    fclose(pf);
                }
            }
            remove("/mnt/.usb_check");
            
            if (buf[0] != 'A')
            {
                printf("usb log ERROR\n");
                buf[0] = 0x01;
                buf[1] = 0x01;
                robot_api_send_packet(ROBOT_CMD_DEBUG, buf, 2);
                check_flag = 0;
            }
            tmp = 0;
        }
    }
}

uint8_t sys_tx_checksum(uint8_t *buf,int  len) //buf为数组，len为数组长度
{ 
    uint8_t  ret = 0;
    int i;
 
    for(i=0; i<len; i++)
    {
        ret += *(buf++);
    }
    // ret = ~ret;
    return ret;
}
void sys_log(int level, const char *fmt, ...)
{
    //FILE *fbp = fopen(APPSERVER_SYS_LOG_FILE,"a+");
    static char str_uart[40960];
    static char str_file[512];
    va_list ap;
    int len;
    char *log_color;

        // 创建LOG文件
    if (!file_log)
    {
        //sys_shell("rm -f %s", APPSERVER_SYS_LOG_FILE);
        file_log = fopen(APPSERVER_SYS_LOG_FILE, "a+");
        if (!file_log)
        {
            return;
        }

        // 设置close_on_exec标志，防止此句柄被子进程继承
        fcntl(fileno(file_log), F_SETFD, FD_CLOEXEC);
    }
    // 文件大小判断
    int fd = fileno(file_log);
    struct stat fs;
    fstat(fd, &fs);
    if (fs.st_size > FILE_MAX)
    {
        printf("log file clear\n");
        ftruncate(fd, 0);
        fflush(file_log);
        fdatasync(fd);
        fclose(file_log);

        file_log = fopen(APPSERVER_SYS_LOG_FILE, "w");
        if (!file_log)
        {
            return;
        }
    
        sprintf(str_file, "log file clear\r\n");
        fwrite(str_file, strlen(str_file), 1, file_log);
    }

    // 增加前缀
	time_t now;
	time(&now);
	strftime(str_file, sizeof(str_file), "[%Y-%m-%d %H:%M:%S ", localtime(&now));

    if (level == LOG_DBG)
    {
        strcat(str_file, "D] ");
        log_color = LOG_CLRSTR_GREEN;
    }
    else if (level == LOG_INFO)
    {
        strcat(str_file, "I] ");
        log_color = LOG_CLRSTR_BLUE;
    }
    else if (level == LOG_WARN)
    {
        strcat(str_file, "W] ");
        log_color = LOG_CLRSTR_PURPLE;
    }
    else if (level == LOG_CRIT)
    {
        strcat(str_file, "C] ");
        log_color = LOG_CLRSTR_RED;
    }
    else
    {
        strcat(str_file, "L] ");
        log_color = LOG_CLRSTR_YELLOW;
    }
#if DEBUG
    sprintf(str_uart, "%s", log_color);
#else
    str_uart[0] = '\0';
#endif
    strcat(str_uart, str_file);

    va_start(ap, fmt);
    len = strlen(str_file);
    vsnprintf(str_file + len, sizeof(str_file) - len, fmt, ap);
    len = strlen(str_uart);
    vsnprintf(str_uart + len, sizeof(str_uart) - len, fmt, ap);      
    va_end(ap);

    printf("%s\n",str_uart);

    if (level >= LOG_FILE_LEVEL)
    {
        // 把LOG写到文件
        strcat(str_file, "\r\n");
        fwrite(str_file, strlen(str_file), 1, file_log);

        // 同步文件
        fflush(file_log);
        fdatasync(fileno(file_log));
    }
}

int read_wi_mode_value(void)
{
    static int check_flag = -1;
    static int tmp;
    char wifi_mode[128]={"hello"};

    FILE *pf = NULL;
 	char buf[128];

    if (check_flag == -1)
    {
        // 首次进入，判断是否开启Appserver Log
        pf = fopen(SYS_WIFI, "r");
        if (!pf)
        {
            printf( "open S60wifi failed\n");
            return -1;
        }

        tmp = 0;
        while (fgets(buf, sizeof(buf), pf) != NULL)
        {
            if (sscanf(buf, "WIFI_DRV=%s\n", &wifi_mode) == 1)
            {
                break;
            }
        }
        fclose(pf);
    }
    printf("wifi_dev = %s\n",wifi_mode);
    if(0 == strcmp(wifi_mode,"ssv6x5x.ko"))
    {
        //return WIFI_SSV6152;
        return WIFI_SSV_PUBLIC;
    }
    else
    {
        return WIFI_REALTCK;
    }

}

int sys_grep_string_from_file(const char *file, const char *str)
{
    char buff[128];
    int result = 0;

    if (!file || !str)
    {
        return -1;
    }

    FILE *pf = fopen(file, "r");
    if (!pf)
    {
        return -1;
    }

    while (!feof(pf))
    {
        fgets(buff, 128, pf);
        if (strstr((const char*)buff, str))
        {
            result = 1;
            break;
        }
    }
    fclose(pf);

    if (result)
    {
        return 1;
    }

    return 0;
}

static int firmware_partition_exist = 0;
int sys_get_firmware_partition_exist_flag(void)
{
    static int first_check;

    if (!first_check)
    {
        if (sys_grep_string_from_file("/proc/mtd", "firmware") == 1)
        {
            firmware_partition_exist = 1;
        }

        first_check = 1;
    }
    
    return firmware_partition_exist;
}

/**
 * @brief       在文本文件中搜索字符串
 * @param file  文本文件
 * @param str   字符串
 * @return int  -1：失败，0：不存在，1：存在
 */
int sys_file_search(const char *file, const char *str)
{
    char buff[512];
    int result = 0;

    if (!file || !str) {
        return -1;
    }

    FILE *pf = fopen(file, "r");
    if (!pf) {
        return -1;
    }

    while (!feof(pf)) {
        fgets(buff, 128, pf);
        if (strstr((const char*)buff, str)) {
            result = 1;
            break;
        }
    }
    fclose(pf);

    if (result) {
        return 1;
    }

    return 0;
}

// 设置固件分区是否可写
void sys_set_firmware_partition_writeable(int enable)
{
    int ret = 0;
    if (!sys_get_firmware_partition_exist_flag())
    {
        return;
    }

    if (enable == 0) {
        if (sys_file_search("/proc/mounts", "/firmware jffs2 rw")) {
            ret = sys_shell("mount -o ro,remount /firmware");
            if (ret < 0) {
                return;
            } else {
                return;
            }
        }
    } else {
        if (sys_file_search("/proc/mounts", "/firmware jffs2 ro")) {
            ret = sys_shell("mount -o rw,remount /firmware");
            if (ret < 0) {
                return;
            } else {
                return;
            }
        }
    }
}

// 获取firmware分区剩余空间
int sys_get_firmware_partition_free(void)
{
    struct statfs info;
    int free;
    
    // 判断是否存在固件分区
    if (sys_grep_string_from_file("/proc/mtd", "firmware") != 1)
    {
        return -1;
    }

    statfs("/root/firmware/arobot", &info);
    free = info.f_bfree * info.f_bsize;

    return free;
}


