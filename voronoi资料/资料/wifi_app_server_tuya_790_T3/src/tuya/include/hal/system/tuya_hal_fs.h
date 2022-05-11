#ifndef __TUYA_HAL_FS_H__
#define __TUYA_HAL_FS_H__


#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif


// 文件访问权限
#define	TUYA_IRUSR  0400    /* Read by owner.  */
#define	TUYA_IWUSR  0200    /* Write by owner.  */
#define	TUYA_IXUSR  0100    /* Execute by owner.  */


// 缓冲区搜索起始位置类型
#define TUYA_SEEK_SET   0   /* Seek from beginning of file.  */
#define TUYA_SEEK_CUR   1   /* Seek from current position.  */
#define TUYA_SEEK_END   2   /* Seek from end of file.  */


typedef void* TUYA_DIR;
typedef void* TUYA_FILEINFO;
typedef void* TUYA_FILE;


/*****************************************************************
 * 文件目录管理
 *****************************************************************/

/**
 * @brief 创建目录
 * 
 * @param[in]       path        目录路径
 * @return  0=成功，非0=失败
 */
int tuya_hal_fs_mkdir(const char* path);

/**
 * @brief 删除目录或文件
 * 
 * @param[in]       path        目录或文件的路径
 * @return  0=成功，非0=失败
 */
int tuya_hal_fs_remove(const char* path);

/**
 * @brief 获取文件属性
 * 
 * @param[in]       path        目录或文件的路径
 * @param[out]      mode        属性位，同 linux struct stat 的 st_mode 字段
 * @return  0=成功，非0=失败
 */
int tuya_hal_fs_mode(const char* path, uint32_t* mode);

/**
 * @brief 判断文件或目录是否存在
 * 
 * @param[in]       path        目录或文件的路径
 * @param[out]      is_exist    是否存在
 * @return  0=成功，非0=失败
 */
int tuya_hal_fs_is_exist(const char* path, bool* is_exist);

/**
 * @brief 文件重命名
 * 
 * @param[in]       path_old    老的文件名
 * @param[in]       path_new    新的文件名
 * @return  0=成功，非0=失败
 */
int tuya_hal_fs_rename(const char* path_old, const char* path_new);


/*****************************************************************
 * 子目录遍历
 *****************************************************************/

/**
 * @brief 打开目录
 * 
 * @param[in]       path        目录的路径
 * @param[out]      dir         返回目录句柄
 * @return  0=成功，非0=失败
 */
int tuya_hal_dir_open(const char* path, TUYA_DIR* dir);

/**
 * @brief 关闭目录句柄
 * 
 * @param[in]       dir         目录句柄
 * @return  0=成功，非0=失败
 */
int tuya_hal_dir_close(TUYA_DIR dir);

/**
 * @brief 读取当前节点的文件信息，内部指针指向下一个节点
 * 
 * @param[in]       dir         目录句柄
 * @param[out]      info        返回文件信息句柄
 * @return  0=成功，非0=失败
 */
int tuya_hal_dir_read(TUYA_DIR dir, TUYA_FILEINFO* info);

/**
 * @brief 获取节点的名称
 * 
 * @param[in]       info        文件信息句柄
 * @param[out]      name        返回名称字符串地址
 * @return  0=成功，非0=失败
 */
int tuya_hal_dir_name(TUYA_FILEINFO info, const char** name);

/**
 * @brief 判断节点是否为目录
 * 
 * @param[in]       info        文件信息句柄
 * @param[out]      is_dir      返回是否为目录
 * @return  0=成功，非0=失败
 */
int tuya_hal_dir_is_directory(TUYA_FILEINFO info, bool* is_dir);

/**
 * @brief 判断节点是否为普通文件
 * 
 * @param[in]       info        文件信息句柄
 * @param[out]      is_regular  返回是否为普通文件
 * @return  0=成功，非0=失败
 */
int tuya_hal_dir_is_regular(TUYA_FILEINFO info, bool* is_regular);


/*****************************************************************
 * 文件IO
 *****************************************************************/

/**
 * @brief 打开文件
 * 
 * @param[in]       path        文件路径
 * @param[in]       mode        文件打开模式: "r","w"...
 * @return  返回文件句柄
 */
TUYA_FILE tuya_hal_fopen(const char* path, const char* mode);

/**
 * @brief 关闭文件句柄
 * 
 * @param[in]       file        文件句柄
 * @return  0=成功，非0=失败
 */
int tuya_hal_fclose(TUYA_FILE file);

/**
 * @brief 读文件
 * 
 * @param[in]       buf         缓冲区地址
 * @param[in]       bytes       缓冲区大小
 * @param[in]       file        文件句柄
 * @return  <0: 失败; >= 实际读取的字节数
 */
int tuya_hal_fread(void* buf, int bytes, TUYA_FILE file);

/**
 * @brief 写文件
 * 
 * @param[in]       buf         缓冲区地址
 * @param[in]       bytes       缓冲区大小
 * @param[in]       file        文件句柄
 * @return  <0: 失败; >= 实际写入的字节数
 */
int tuya_hal_fwrite(void* buf, int bytes, TUYA_FILE file);

/**
 * @brief 将缓存写入存储器
 * 
 * @param[in]       file        My Param doc
 * @return  0=成功，非0=失败
 */
int tuya_hal_fsync(TUYA_FILE file);

/**
 * @brief 读取字符串
 * 
 * @param[in]       buf         缓冲区地址
 * @param[in]       len         缓冲区大小
 * @param[in]       file        文件句柄
 * @return  读入的字符串指针
 */
char* tuya_hal_fgets(char* buf, int len, TUYA_FILE file);

/**
 * @brief 判断是否到达文件尾
 * 
 * @param[in]       file        文件句柄
 * @return  0=未到文件尾，非0=到达文件尾
 */
int tuya_hal_feof(TUYA_FILE file);

/**
 * @brief 文件定位
 * 
 * @param[in]       file        文件句柄
 * @param[in]       offs        偏移量
 * @param[in]       whence      起始点类型
 * @return  0=成功，非0=失败
 */
int tuya_hal_fseek(TUYA_FILE file, int64_t offs, int whence);

/**
 * @brief 获取文件位置
 * 
 * @param[in]       file        文件句柄
 * @return  返回当前位置
 */
int64_t tuya_hal_ftell(TUYA_FILE file);


#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif // __TUYA_HAL_FS_H__

