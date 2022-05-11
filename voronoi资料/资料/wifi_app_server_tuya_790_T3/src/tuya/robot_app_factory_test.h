#ifndef __ROBOT_APP_FACTORY_TEST_H__
#define __ROBOT_APP_FACTORY_TEST_H__

#define SHELL_STARTUP_USB                   "/root/etc/init.d/S40usb"

// 测试结果
enum
{
    RESULT_FAILED,
    RESULT_SUCCEED,

};

// WiFi测试错误代码
enum
{
    WIFI_WLAN_DEV_ERROR ,       // WiFi故障
    WIFI_AP_NOT_FOUND,          // AP没有找到
    WIFI_AP_CONNECT_FAIL,       // AP连接失败
    WIFI_GET_RSSI_FAIL,         // 获取信号强度失败

};

// 厂测命令相关协议
enum
{
    FACTORY_TEST_WIFI,                      // WiFi测试
    FACTORY_TEST_USB,                       // USB测试
    FACTORY_TEST_UPGRADE,                   // 380升级测试，包括升级串口、复位、SPI IRQ

};

enum
{

    CMD_FACTORY_FACTORY_TEST                = 0x09, // 厂测命令
    CMD_WIFI_FACTORY_TEST                   = 0x23, // WiFi测试
    
};

// API
int robot_app_factory_wifi_test(const char *ssid, const char *password);
int wifi_app_factory_test(int type);

#endif // !__ROBOT_APP_FACTORY_TEST_H__