#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include "sys_api.h"
#include "robot_api.h"
#include "robot_app_msg.h"
#include "robot_app_factory_test.h"

#define WLAN_DEV                "wlan0"
#define WPA_SUPPLICANT_CONF     "/tmp/wpa_supplicant.conf"
#define UDHCPC_SCRIPT           "/firmware/etc/udhcpc.script"
#define IO_HOSLAM_SPI_IRQ       37 
#define WIFI_APP_DATA_PROTOCOL_PACKET_BUF   512



static void stop_wifi_connect(void)
{
    sys_shell(SHELL_STARTUP_DAEMON " stop 2>/dev/null");
//    sys_shell(SHELL_STARTUP_GRIT_CLIENT " stop 2>/dev/null");
    sys_shell("killall -9 wpa_supplicant 2>/dev/null");
    sys_shell("killall -9 udhcpc 2>/dev/null");
    sys_shell("killall -9 hostapd 2>/dev/null");
    sys_shell("killall -9 udhcpd 2>/dev/null");
    sys_shell("iwconfig %s mode Managed", WLAN_DEV);
    sys_shell("%s restart", SHELL_STARTUP_NETWORK);
}

static int inform_wifi_test_result(int result, int value)
{
    uint8_t packet_buffer[WIFI_APP_DATA_PROTOCOL_PACKET_BUF];
    int i = 0;

    packet_buffer[i++] = result;
    packet_buffer[i++] = value;

    robot_api_send_packet(CMD_WIFI_FACTORY_TEST, packet_buffer, i);

    return 0;
}

static int inform_factory_test_result(int type, int result)
{
    uint8_t packet_buffer[WIFI_APP_DATA_PROTOCOL_PACKET_BUF];
    int i = 0;

    packet_buffer[i++] = type;
    packet_buffer[i++] = result;

    robot_api_send_packet(CMD_FACTORY_FACTORY_TEST, packet_buffer, i);

    return 0;
}


int robot_app_factory_wifi_test(const char *ssid, const char *password)
{
    int ret, connected;
	int timeout, rssi;
    char tmp_ssid[64], tmp_password[64];
    char ip_old[16], ip[16];
    FILE *pp = NULL;

    if (!ssid || !password)
    {
        printf("para error\n");
        return -1;
    }

    // 数据解析
    memset(tmp_ssid, 0, sizeof(tmp_ssid));
    memset(tmp_password, 0, sizeof(tmp_password));
    strncpy(tmp_ssid, ssid, sizeof(tmp_ssid));
    strncpy(tmp_password, password, sizeof(tmp_password));
    printf("wifi_app_factory_wifi_test: ssid=%s, password=%s\n", tmp_ssid, tmp_password);
    stop_wifi_connect();

    // 判断wlan0是否存在
    timeout = 20;
    while (timeout--)
    {
        ret = sys_get_ip(ip_old);
        if (ret == 0)
        {
            break;
        }

        sleep(1);
    }
    if (ret < 0)
    {
        printf("wifi_app_factory_wifi_test: wlan is not ready\n");
        inform_robot_test_result(RESULT_FAILED, WIFI_WLAN_DEV_ERROR);
        return -1;
    }

    // 搜索指定AP
    ret = sys_shell_result_to_int("iwlist %s scanning | grep -w \"%s\" | wc -l", WLAN_DEV, tmp_ssid);
    if (ret <= 0)
    {
        printf("wifi_app_factory_wifi_test: ap not found, ret=%d\n", ret);
        inform_robot_test_result(RESULT_FAILED, WIFI_AP_NOT_FOUND);
        return -1;
    }

    sys_get_ip(ip_old);

    // 连接指定AP
    printf("wifi_app_factory_wifi_test: wifi connecting\n");
    if (strlen(tmp_password) == 0)
    {
        sys_shell("iwconfig %s essid \"%s\"", WLAN_DEV, tmp_ssid);
        sys_shell("udhcpc -i %s -s \"%s\" &", WLAN_DEV, UDHCPC_SCRIPT);
    }
    else
    {
        sys_shell("wpa_passphrase %s %s > %s", tmp_ssid, tmp_password, WPA_SUPPLICANT_CONF);
        sys_shell("wpa_supplicant -Dnl80211 -B -i%s -c%s", WLAN_DEV, WPA_SUPPLICANT_CONF);
        sys_shell("udhcpc -i %s -s \"%s\" &", WLAN_DEV, UDHCPC_SCRIPT);
    }
    
    // 等待AP连接，40s超时
    connected = 0;
    timeout = 40;
	while (timeout--)
	{
		sys_get_ip(ip);
		if (strcmp(ip, ip_old) != 0)
		{
            connected = 1;
			break;
		}

		sleep(1);
    }

    if (!connected)
    {
        printf("wifi_app_factory_wifi_test: wifi connect error\n");
        inform_robot_test_result(RESULT_FAILED, WIFI_AP_CONNECT_FAIL);
        return -1;
    }

    // AP连接成功，获取信道强度
    pp = sys_popen("iwconfig "WLAN_DEV, "r");
    if (pp == NULL)
    {
        inform_robot_test_result(RESULT_FAILED, WIFI_GET_RSSI_FAIL);
        return -1;
    }

    char tmp[128] = {0};
    while (fgets(tmp, sizeof(tmp), pp) != NULL)
    {
        char *str = strstr(tmp, "Quality=");
        if(str != NULL)
        {
            int x = 0;
            int y = 0;
            sscanf(str + strlen("Quality="), "%d/%d",&x,&y);
            rssi = x * 100 / (y+1);
            break;
        }
    }
    sys_pclose(pp);

    printf("wifi_app_factory_wifi_test: RSSI=%d\n", rssi);
    inform_robot_test_result(RESULT_SUCCEED, rssi);

    return 0;
}

static int wifi_app_factory_wifi_test(void)
{
    int ret;
	int timeout;
    char mac[32];

    // 停止WiFi连接
    stop_wifi_connect();

    // 判断wlan0是否存在
    timeout = 10;
    while (timeout--)
    {
        ret = sys_get_mac(mac, sizeof(mac));
        if (ret > 0)
        {
            ret = 0;
            break;
        }

        sleep(1);
    }

    // 获取到MAC，代表WiFi功能OK
    if (ret == 0)
    {
        printf("wifi test OK\n");
        inform_factory_test_result(FACTORY_TEST_WIFI, RESULT_SUCCEED);
        return 0;
    }

    // 获取MAC超时，尝试重启WiFi驱动
    sys_shell(SHELL_STARTUP_WIFI " restart");
    sys_shell(SHELL_STARTUP_NETWORK " restart");
    ret = sys_get_mac(mac, sizeof(mac));
    if (ret > 0)
    {
        printf( "reset wifi, wifi test OK\n");
        inform_factory_test_result(FACTORY_TEST_WIFI, RESULT_SUCCEED);
        return 0;
    }

    printf( "wifi_app_factory_wifi_test: wifi test NG\n");
    inform_factory_test_result(FACTORY_TEST_WIFI, RESULT_FAILED);
    return -1;
}

static int wifi_app_factory_usb_test(void)
{
    int timeout_sec, result;
    int uart_fd, ret;
    char tmp[64];

    printf( "start usb test\n");

    // 退出相关进程
    sys_shell(SHELL_STARTUP_DAEMON " stop 2>/dev/null");
    //sys_shell(SHELL_STARTUP_GRIT_CLIENT " stop 2>/dev/null");
    sys_shell("killall -9 wpa_supplicant 2>/dev/null");
    sys_shell("killall -9 udhcpc 2>/dev/null");
    sys_shell("killall -9 hostapd 2>/dev/null");
    sys_shell("killall -9 udhcpd 2>/dev/null");
    sys_shell(SHELL_STARTUP_USB " stop 2>/dev/null");

    // 加载USB驱动
    sys_shell("modprobe sunxi");
    sys_shell("modprobe ehci-platform");
    sys_shell("modprobe cdc-acm");
    sys_shell("modprobe usb_f_serial");
    sys_shell("modprobe usb_f_obex");
    sys_shell("modprobe usb_f_acm");
    sys_shell("modprobe g_serial");

    // 打开串口设备
    uart_fd = sys_uart_open("/dev/ttyGS0");
    if (uart_fd < 0)
    {
        printf( "uart open failed\n");
        return -1;
    }

    // 配置串口
    ret = sys_uart_set(uart_fd, 115200, 0, 8, 1, 'N');
    if (ret)
    {
        printf( "uart set failed\n");
        return -1;
    }
    ret = sys_uart_set_timeout(uart_fd, 100);
    if (ret)
    {
        printf( "uart set timeout failed\\n");
        return -1;
    }
    sys_uart_flush_input(uart_fd);

    // 等待驱动加载完成
    printf( "wait usb connect\n");
    timeout_sec = 20;
    result = 0xFF;
    memset(tmp, 0, sizeof(tmp));
    while (1)
    {
        ret = sys_uart_read(uart_fd, tmp, sizeof(tmp));
        if (ret > 0)
        {
            //sys_log(LOG_INFO, "sys_uart_read tmp=%s", tmp);
            if (strncmp(tmp, "usb host", 8) == 0)
            {
                printf("usb device connected\n");
                result = 1;
                break;
            }
        }

        sleep(1);
        timeout_sec--;
        if (timeout_sec <= 0)
        {
            printf("usb test timeout\n");
            result = 0;
            break;
        }
    }

    // 关闭串口
    sys_uart_close(uart_fd);

    // 卸载USB驱动
    sys_shell("modprobe -r g_serial");
    sys_shell("modprobe -r usb_f_serial");
    sys_shell("modprobe -r usb_f_obex");
    sys_shell("modprobe -r usb_f_acm");
    sys_shell("modprobe -r ehci-platform");
    sys_shell("modprobe -r cdc-acm");
    sys_shell("modprobe -r sunxi");

    if (result == 1)
    {
        inform_factory_test_result(FACTORY_TEST_USB, RESULT_SUCCEED);
    }
    else
    {
        inform_factory_test_result(FACTORY_TEST_USB, RESULT_FAILED);
    }
    
    return 0;
}

static int wifi_app_factory_upgrade_test(void)
{
    int uart_fd, ret, count;
    int high, low;
    int uart_result, irq_result, reset_result;
    char tmp[64];

    printf( "380 upgrade test\n");

    // 退出相关进程
    sys_shell(SHELL_STARTUP_DAEMON " stop 2>/dev/null");
    //sys_shell(SHELL_STARTUP_GRIT_CLIENT " stop 2>/dev/null");
    sys_shell("killall -9 wpa_supplicant 2>/dev/null");
    sys_shell("killall -9 udhcpc 2>/dev/null");
    sys_shell("killall -9 hostapd 2>/dev/null");
    sys_shell("killall -9 udhcpd 2>/dev/null");
    sys_shell(SHELL_STARTUP_HOSLAM " stop 2>/dev/null");

    // 打开串口
    uart_fd = sys_uart_open("/dev/ttyS0");
    if (uart_fd < 0)
    {
        printf("uart0 open failed\n");
        return -1;
    }

    // 配置串口0
    ret = sys_uart_set(uart_fd, 115200, 0, 8, 1, 'N');
    if (ret)
    {
        printf("uart0 set failed\n");
        return -1;
    }
    ret = sys_uart_set_timeout(uart_fd, 100);
    if (ret)
    {
        printf( "uart1 set timeout failed\n");
        return -1;
    }

    // 发送测试命令
    ret = sys_uart_write(uart_fd, "580\n", 4);
	if (ret != 4)
	{
		printf("sys_uart_write ERROR\n");
		return -1;
	}

    // 等待380回应
    uart_result = 0xFF;
    count = 0;
    while (1)
    {
        ret = sys_uart_read(uart_fd, tmp, sizeof(tmp));
        if (ret > 0)
        {
            if (strncmp(tmp, "380", 3) == 0)
            {
                uart_result = 1;
                printf("uart test OK\n");
                break;
            }
        }

        count++;
        if (count >= 20)
        {
            uart_result = 0;
            printf("uart test NG\n");
            break;
        }
    }

    // 串口测试不通过，由380超时检测显示错误
    if (uart_result == 0)
    {
        // 关闭串口
        sys_uart_close(uart_fd);
        return -1;
    }

    // 再次退出守护进程
    sys_shell(SHELL_STARTUP_DAEMON " stop 2>/dev/null");

    // SPI IRQ测试 配置IRQ引脚PB5为下拉输入
	sys_shell("devmem 0x01c20840 32 0x1404");
    sys_shell("echo %d > /sys/class/gpio/export 2>/dev/null", IO_HOSLAM_SPI_IRQ);
    sys_shell("echo in > /sys/class/gpio/gpio%d/direction", IO_HOSLAM_SPI_IRQ);

    high = low = count = 0;
    while (1)
    {
        ret = sys_shell_result_to_int("cat /sys/class/gpio/gpio%d/value", IO_HOSLAM_SPI_IRQ);
        if (ret == 1)
        {
            high++;
        }
        else if (ret == 0)
        {
            low++;
        }

        count++;
        if (high >= 5 && low >= 5)
        {
            printf( "irq test OK\n");
            irq_result = 1;
            break;
        }
        else if (count >= 200)
        {
            printf("irq test NG\n");
            irq_result = 0;
            break;
        }

        usleep(5000);
    }

    sys_shell("echo %d > /sys/class/gpio/unexport 2>/dev/null", IO_HOSLAM_SPI_IRQ);

    // IRQ测试不通过，由380超时检测显示错误
    if (irq_result == 0)
    {
        return -1;
    }

    // 再次退出守护进程
    sys_shell(SHELL_STARTUP_DAEMON " stop 2>/dev/null");

    // 复位测试
    sys_shell("/root/bin/reset380 &");

    // 等待重启，通过判断bootloader版本号判断
    count = 0;
    while (1)
    {
        ret = sys_uart_read(uart_fd, tmp, sizeof(tmp));
        if (ret > 0)
        {
            printf("bootloader version:%s\n", tmp);
            if (strncmp(tmp, "Amicro", 6) == 0)
            {
                reset_result = 1;
                printf( "reset test OK\n");
                break;
            }
        }

        count++;
        if (count >= 20)
        {
            reset_result = 0;
            printf( "reset test NG");
            break;
        }
    }

    // 关闭串口
    sys_uart_close(uart_fd);

    // 启动通信进程
    sys_shell(SHELL_STARTUP_HOSLAM " start 2>/dev/null");
    sleep(2);

    // 发送测试结果
    if (uart_result && irq_result && reset_result)
    {
        inform_factory_test_result(FACTORY_TEST_UPGRADE, RESULT_SUCCEED);
    }
    else
    {
        inform_factory_test_result(FACTORY_TEST_UPGRADE, RESULT_FAILED);
    }
    
    return 0;
}

int wifi_app_factory_wifi_connection_test(const char *ssid, const char *password)
{
    int ret, connected;
	int timeout, rssi;
    char tmp_ssid[64], tmp_password[64];
    char ip_old[16], ip[16];
    FILE *pp = NULL;

    if (!ssid || !password)
    {
        printf( "para error\n");
        return -1;
    }

    // 数据解析
    memset(tmp_ssid, 0, sizeof(tmp_ssid));
    memset(tmp_password, 0, sizeof(tmp_password));
    strncpy(tmp_ssid, ssid, sizeof(tmp_ssid));
    strncpy(tmp_password, password, sizeof(tmp_password));
    printf( "wifi_app_factory_wifi_connection_test: ssid=%s, password=%s\n", tmp_ssid, tmp_password);
    stop_wifi_connect();

    // 判断wlan0是否存在
    timeout = 20;
    while (timeout--)
    {
        ret = sys_get_ip(ip_old);
        if (ret == 0)
        {
            break;
        }

        sleep(1);
    }
    if (ret < 0)
    {
        printf( "wifi_app_factory_wifi_connection_test: wlan is not ready\n");
        inform_wifi_test_result(RESULT_FAILED, WIFI_WLAN_DEV_ERROR);
        return -1;
    }

    // 搜索指定AP
    ret = sys_shell_result_to_int("iwlist %s scanning | grep -w \"%s\" | wc -l", WLAN_DEV, tmp_ssid);
    if (ret <= 0)
    {
        printf( "wifi_app_factory_wifi_connection_test: ap not found, ret=%d\n", ret);
        inform_wifi_test_result(RESULT_FAILED, WIFI_AP_NOT_FOUND);
        return -1;
    }

    sys_get_ip(ip_old);

    // 连接指定AP
    printf( "wifi_app_factory_wifi_connection_test: wifi connecting\n");
    if (strlen(tmp_password) == 0)
    {
        sys_shell("iwconfig %s essid \"%s\"", WLAN_DEV, tmp_ssid);
        sys_shell("udhcpc -i %s -s \"%s\" &", WLAN_DEV, UDHCPC_SCRIPT);
    }
    else
    {
        sys_shell("wpa_passphrase \"%s\" \"%s\" > %s", tmp_ssid, tmp_password, WPA_SUPPLICANT_CONF);
        sys_shell("wpa_supplicant -Dnl80211 -B -i%s -c%s", WLAN_DEV, WPA_SUPPLICANT_CONF);
        sys_shell("udhcpc -i %s -s \"%s\" &", WLAN_DEV, UDHCPC_SCRIPT);
    }
    
    // 等待AP连接，40s超时
    connected = 0;
    timeout = 40;
	while (timeout--)
	{
		sys_get_ip(ip);
		if (strcmp(ip, ip_old) != 0)
		{
            connected = 1;
			break;
		}

		sleep(1);
    }

    if (!connected)
    {
        printf( "wifi_app_factory_wifi_connection_test: wifi connect error\n");
        inform_wifi_test_result(RESULT_FAILED, WIFI_AP_CONNECT_FAIL);
        return -1;
    }

    // AP连接成功，获取信道强度
    pp = sys_popen("iwconfig "WLAN_DEV, "r");
    if (pp == NULL)
    {
        inform_wifi_test_result(RESULT_FAILED, WIFI_GET_RSSI_FAIL);
        return -1;
    }

    char tmp[128] = {0};
    while (fgets(tmp, sizeof(tmp), pp) != NULL)
    {
        char *str = strstr(tmp, "Quality=");
        if (str != NULL)
        {
            int x = 0;
            int y = 0;
            sscanf(str + strlen("Quality="), "%d/%d", &x, &y);
            rssi = x * 100 / (y+1);
            break;
        }
    }
    sys_pclose(pp);

    printf( "wifi_app_factory_wifi_connection_test: RSSI=%d\n", rssi);
    inform_wifi_test_result(RESULT_SUCCEED, rssi);

    return 0;
}


int wifi_app_factory_test(int type)
{
    printf( "wifi_app_factory_test: type=%d\n", type);

    switch (type)
    {
        case FACTORY_TEST_WIFI:
            wifi_app_factory_wifi_test();
            break;
        
        case FACTORY_TEST_USB:
            wifi_app_factory_usb_test();
            break;

        case FACTORY_TEST_UPGRADE:
            wifi_app_factory_upgrade_test();
            break;

        default:
            break;
    }
}

