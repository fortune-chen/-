/**
 * @file tuya_hal_output.h
 * @brief 日志操作接口
 * 
 * @copyright Copyright(C),2018-2020, 涂鸦科技 www.tuya.com
 * 
 */

#ifndef _TUYA_HAL_OUTPUT_H
#define _TUYA_HAL_OUTPUT_H

#include <stddef.h>


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief tuya_hal_output_log用于输出log信息
 * 
 * @param[in] *str log buffer指针
 * 函数实现在tuya_hal_output.c
 */
void tuya_hal_output_log(const char *str);

/**
 * @brief 用于关闭原厂sdk默认的log口
 * 
 */
int tuya_hal_log_close(void);

/**
 * @brief 用于恢复原厂sdk默认的log口
 * 
 */
int tuya_hal_log_open(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

