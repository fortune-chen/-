/*
tuya_iot_com_api.h
Copyright(C),2018-2020, 涂鸦科技 www.tuya.comm
*/

#ifndef __TUYA_IOT_SWEEPER_API_H
#define __TUYA_IOT_SWEEPER_API_H

#include "tuya_cloud_types.h"
#include "tuya_cloud_com_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_M_MAP_INFO_NUM 5

#define DESCRIPT_STR_LMT  64
#define BITMAP_FILE_STR_LMT  64
#define DATAMAP_FILE_STR_LMT  64
typedef struct {
    UINT_T map_id;
    UINT_T time;
    CHAR_T descript[DESCRIPT_STR_LMT];
    CHAR_T bitmap_file[BITMAP_FILE_STR_LMT];
    CHAR_T datamap_file[DATAMAP_FILE_STR_LMT];
} M_MAP_INFO;

/***********************************************************
*  Function: tuya_iot_upload_incre_data
*  Input:
   map_id: Each map has an Id
   offset: Map data offset
   pbuffer:Map data cache pointer
   buf_len:Map data cache length
*  Output: none
*  Return: OPERATE_RET
***********************************************************/
OPERATE_RET tuya_iot_upload_incre_data(IN CONST USHORT_T map_id,IN CONST UINT_T offset,IN CONST BYTE_T *pbuffer, IN CONST USHORT_T buf_len);

/***********************************************************
*  Function: tuya_iot_media_data_report
*  Input: dt_body->media data
*         timeout->need report time
*  Output: none
*  WANGING: please dont use tuya_iot_media_data_report and tuya_iot_media_data_report_v2 in one application
*  Return: OPERATE_RET
***********************************************************/
OPERATE_RET tuya_iot_media_data_report(IN CONST FLOW_BODY_ST *dt_body,IN CONST UINT_T timeout);

/***********************************************************
*  Function: tuya_iot_media_data_report_v2
*  Input: dt_body->media data version 2
*         timeout->need report time
*  Output: none
*  WANGING: please dont use tuya_iot_media_data_report and tuya_iot_media_data_report_v2 in one application
*  Return: OPERATE_RET
***********************************************************/
OPERATE_RET tuya_iot_media_data_report_v2(IN CONST FLOW_BODY_V2_ST *dt_body,IN CONST UINT_T timeout);

#define tuya_iot_upload_layout_buffer(map_id, buffer, len) \
    tuya_iot_map_cleaner_upload_buffer(map_id, buffer, len, "layout/lay.bin", UP_CLEANER_MAP)
#define tuya_iot_upload_route_buffer(map_id, buffer, len) \
    tuya_iot_map_cleaner_upload_buffer(map_id, buffer, len, "route/rou.bin", UP_CLEANER_PATH)
#define tuya_iot_upload_navigation_buffer(map_id, buffer, len) \
    tuya_iot_map_cleaner_upload_buffer(map_id, buffer, len, "route/nav.bin", UP_CLEANER_NAVI)

#define tuya_iot_upload_layout_file(map_id, local_file_name) \
    tuya_iot_map_cleaner_upload_file(map_id, local_file_name, "layout/lay.bin", UP_CLEANER_MAP)
#define tuya_iot_upload_route_file(map_id, local_file_name) \
    tuya_iot_map_cleaner_upload_file(map_id, local_file_name, "route/rou.bin", UP_CLEANER_PATH)

/***********************************************************
*  Function: tuya_iot_map_upload_files
*  Desc:  sweeper function. upload cleaner map info
*  Input: bitmap & data map
*  output: map_id
*  Return: OPERATE_RET
***********************************************************/
OPERATE_RET tuya_iot_map_upload_files(IN CONST CHAR_T *bitmap_file, IN CONST CHAR_T *datamap_file, IN CONST CHAR_T *descript, OUT CONST UINT_T *map_id);

/***********************************************************
*  Function: tuya_iot_map_update_files
*  Desc:  sweeper function. update cleaner map info
*  Input: map_id
*  Input: bitmap & data map
*  Return: OPERATE_RET
***********************************************************/
OPERATE_RET tuya_iot_map_update_files(IN CONST UINT_T map_id, IN CONST CHAR_T *bitmap_file, IN CONST CHAR_T *data_map_file);

/***********************************************************
*  Function: tuya_iot_map_delete
*  Desc:  sweeper function. delete map files in cloud
*  Input: map_id
*  Return: OPERATE_RET
***********************************************************/
OPERATE_RET tuya_iot_map_delete(IN CONST UINT_T map_id);

/***********************************************************
*  Function: tuya_iot_get_map_files
*  Desc:  sweeper function. get map files
*  Input: map_id
*  Input: map_path
*  Output: map files under map_path path.
*  Return: OPERATE_RET
***********************************************************/
OPERATE_RET tuya_iot_get_map_files(IN CONST UINT_T map_id, IN CONST CHAR_T *map_path);


/***********************************************************
*  Function: tuya_iot_get_all_maps_info
*  Desc:  sweeper function. get map files
*  Input: map_info
*  Input: info_len
*  Output: get all map info (Maximum 5 sets of data)
*  Return: OPERATE_RET
***********************************************************/
OPERATE_RET tuya_iot_get_all_maps_info(OUT M_MAP_INFO *map_info, INOUT UINT8_T *info_len);


/***********************************************************
*  Function: tuya_iot_map_cleaner_upload_buffer
*  Desc:  sweeper function. upload cleaner map info
*  Input: map_id
*  Input: buffer
*  Input: len
*  Input: cloud_file_name
*  Input: map_type
*  Return: OPERATE_RET
***********************************************************/
OPERATE_RET tuya_iot_map_cleaner_upload_buffer(IN CONST INT_T map_id, IN CONST BYTE_T *buffer, IN CONST UINT_T len, \
                                               IN CONST CHAR_T *cloud_file_name, IN CONST UP_MAP_TYPE_E map_type);

/***********************************************************
*  Function: tuya_iot_map_cleaner_upload_file
*  Desc:  sweeper function. upload cleaner map info
*  Input: map_id
*  Input: local_file_name
*  Input: cloud_file_name
*  Input: map_type
*  Return: OPERATE_RET
***********************************************************/
OPERATE_RET tuya_iot_map_cleaner_upload_file(IN CONST INT_T map_id, IN CONST CHAR_T *local_file_name, \
                                             IN CONST CHAR_T *cloud_file_name, IN CONST UP_MAP_TYPE_E map_type);

/***********************************************************
*  Function: tuya_iot_map_record_upload_buffer
*  Desc:  sweeper function. upload record map info
*  Input: map_id
*  Input: buffer
*  Input: len
*  Input: descript
*  Return: OPERATE_RET
***********************************************************/
OPERATE_RET tuya_iot_map_record_upload_buffer(IN CONST INT_T map_id, IN CONST BYTE_T *buffer, IN CONST UINT_T len, IN CONST CHAR_T *descript);

/***********************************************************
*  Function: tuya_iot_map_record_upload_buffer_with_filename
*  Desc:  sweeper function. upload record map info
*  Input: map_id
*  Input: buffer
*  Input: len
*  Input: cloud file name
*  Input: descript
*  Return: OPERATE_RET
***********************************************************/
OPERATE_RET tuya_iot_map_record_upload_buffer_with_filename(IN CONST INT_T map_id, IN CONST BYTE_T *buffer, IN CONST UINT_T len, IN CONST CHAR_T *cloud_file_name, IN CONST CHAR_T *descript);

#ifdef __cplusplus
}
#endif

#endif
