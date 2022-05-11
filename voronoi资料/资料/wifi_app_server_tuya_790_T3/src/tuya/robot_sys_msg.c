#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>
#include <string.h>
#include <sys/time.h>

#include "tuya_iot_com_api.h"
#include "tuya_cloud_types.h"
#include "tuya_iot_wifi_api.h"
#include "uni_log.h"

#include "robot_sys_msg.h"
#include "robot_app_msg.h"
#include "tuya_iot_uploader.h"
#include "tuya_iot_state_adapter.h"
#include "robot_api.h"
#include "robot_control_adapter.h"
//#include"wifi_app_server.h"
#include "tuya_iot_msg_parse.h"
#include "robot_control_adapter.h"
#include "tuya_iot_msg_parse.h"
#include "../debug/debug_tool.h"
#include "tuya_dp_data_mapping.h"
#include "arpa/inet.h"

#define HIDE_CLEAN_NAVI_PATH

#define min(x,y) x > y ? y : x
#define max(x,y) x < y ? y : x


//记录当前路径点个数，用于地图reset后，过滤延时的路径
static bool path_reset_flag;
static int last_path_num;
static int path_filter_count;

//保留房间分区的数据
//static int rooms_map_flag;  //有无分区房间地图标志位
#define ROOMS_MAP_FLAG //有无分区房间地图标志位

static DATA_ROOMS room_vertex_coordinate[MAX_ROOM_SPEC];

extern bool get_editing_room_flag;
bool upload_cur_pose_flag = false;

/************************************************************
** 函数名称:  InOrOutPolygon
** 功能描述:  判断点在多边形内外
** 输入参数:  nvert 顶点个数 vertx 多边形顶点x坐标数组 verty 多边形顶点y坐标数组 
              testx 被判断点位置x坐标 testy 被判断点位置y坐标
** 输出参数:1在点在多边形内
            0点在多边形外
**************************************************************/
static int InOrOutPolygon(int nvert, DATA_POSITION *vert, float testx, float testy)
{
  int i, j, crossings = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++) 
  {
   // 点在两个x之间 且以点垂直y轴向上做射线
   // if(((vertx[i]>testx) != (vertx[j]>testx)) 
   // && (testx > (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]))
   //    crossings++;
    if(((vert[i].x >testx) != (vert[j].x>testx)) 
    && (testx > (vert[j].x-vert[i].x) * (testy-vert[i].y) / (vert[j].y-vert[i].y) + vert[i].x))
       crossings++;
  }
  return (crossings % 2 != 0);
}

/**
 * 功能：判断点是否在多边形内
 * 方法：求解通过该点的水平线（射线）与多边形各边的交点
 * 结论：单边交点为奇数，成立!
 * 参数：p 指定的某个点
         ptPolygon 多边形的各个顶点坐标（首末点可以不一致） 
         nCount 多边形定点的个数
 * 说明：
 */
int PtInPolygon(int nCount,DATA_POSITION* ptPolygon,float testx, float testy) 
{ 
#if 0
    int nCross = 0, i;
    long x;
    DATA_POSITION p1, p2;
    
    for (i = 0; i < nCount; i++) 
    { 
        p1 = ptPolygon[i]; 
        p2 = ptPolygon[(i + 1) % nCount];
        // 求解 y=p.y 与 p1p2 的交点
        if ( p1.y == p2.y ) // p1p2 与 y=p.y平行 
            continue;
        if ( testy < min(p1.y, p2.y) ) // 交点在p1p2延长线上 
            continue; 
        if ( testy>= max(p1.y, p2.y) ) // 交点在p1p2延长线上 
            continue;
        // 求交点的 X 坐标 -------------------------------------------------------------- 
        x = (long)(testy - p1.y) * (long)(p2.x - p1.x) / (long)(p2.y - p1.y) + p1.x;
        if ( x > testx ) 
        {
            nCross++; // 只统计单边交点 
        }
    }
    // 单边交点为偶数，点在多边形之外 --- 
    return (nCross % 2 == 1); 
 #endif
    int j=0, count = 0;
    for (int i = 0; i < nCount; i++)
    {
        j = (i == nCount - 1) ? 0 : j + 1;
        if ((ptPolygon[i].y!=ptPolygon[j].y)&&(((testy >= ptPolygon[i].y) 
        && (testy < ptPolygon[j].y)) || ((testy >= ptPolygon[j].y) 
        && (testy < ptPolygon[i].y))) 
        && (testx < (ptPolygon[j].x - ptPolygon[i].x) * (testy - ptPolygon[i].y) / (ptPolygon[j].y - ptPolygon[i].y) + ptPolygon[i].x)) 
        {
            count++;
        }
    }
    return (count%2>0)?1:0;
}




//房间顶点坐标转化
void room_vertex_coordinate_conversion(float map_ox,float map_oy)
{
    int i,j;
    room_spec_arg_t *p_rspec_arg = robot_ctl_get_room_spec_arg();

    for(i = 0; i < p_rspec_arg->num;++i)
    {
        //printf("point_num%d\n",i);
        room_vertex_coordinate[i].points_num = p_rspec_arg->room_spec[i].n_points;
        //printf("n_points=%d\n",p_rspec_arg->room_spec[i].n_points);
        for(j = 0; j < p_rspec_arg->room_spec[i].n_points;++j)
        {
            room_vertex_coordinate[i].spec[j].x = map_ox + p_rspec_arg->room_spec[i].points[j].x * 20;
            room_vertex_coordinate[i].spec[j].y = map_oy - p_rspec_arg->room_spec[i].points[j].y * 20;
            //printf("x = %f,y = %f\n",room_vertex_coordinate[i].spec[j].x,room_vertex_coordinate[i].spec[j].y);
        }
    }
    
}

uint8_t map_is_editable_flag;
int set_map_is_editable_flag(uint8_t enable_value)
{
    map_is_editable_flag = enable_value;
}
int set_map_is_editable(uint8_t *data)
{
    int ret =  *((uint32_t*)data);
    printf("wifi_app_msg_dp_send_set_is_editable = %d\n",ret);
    if(0 == ret)
    {
        map_is_editable_flag = 1;
    }
    else if(-1 == ret)
    {
        map_is_editable_flag = 0;
    }
}

uint8_t map_is_emtry;
int get_map_is_emtry(void)
{
    return map_is_emtry;
}

//static DATA_MAP *save_rooms_map = NULL;
static uint8_t power_on_rooms = 5;

/* 检测是否是新版本的地图数据 */
static uint8_t check_is_new_map_data(char *data)
{
    if(strncmp(data,"2bit",strlen("2bit")) == 0)
    {
		printf("map_v2--->2bit \n");
        return 1;	
    }
    else
    {
        printf("map_v1 \n"); 
        return 0;
    }
}

/* 处理每一个像素点 判断每一个像素点的属性*/
static void deal_with_map_room_pixel(uint8_t pix_data,uint8_t *type,uint8_t *identify_room,float point_x,float point_y) 
{
    room_spec_arg_t *p_rspec_arg = robot_ctl_get_room_spec_arg();
    uint8_t ret,not_in_polygon_flag;
    int k;
    not_in_polygon_flag = 0;
    //printf("pix_data & 0x03 = 0x%x\n",pix_data & 0x03);
    if((pix_data & 0x03) == 0x03)
    {
        *type = 3;	// 未知/背景
        *identify_room = 63; //未知的房间标识
    }
    else if((pix_data & 0x03) == 0x01)
    {
        *type = 1;	// 障碍物
        *identify_room = 62; //障碍物的房间标识
    } 
    else
    {
        *type = 0;	// 无障碍区域
        if(0 == p_rspec_arg->num)
        {
            *identify_room = 60; //没有分房间数据
        }
        else
        {
            for(k = 0;k < p_rspec_arg->num;++k)
            {
                ret = PtInPolygon(room_vertex_coordinate[k].points_num,room_vertex_coordinate[k].spec, point_x, point_y);
                if(1 == ret)
                {
                    *identify_room = p_rspec_arg->room_spec[k].id;
                    not_in_polygon_flag = 1;
                    //printf("identify_room = %d\n",p_rspec_arg->room_spec[k].id);
                    break;
                }
            }
            if(0 == not_in_polygon_flag)  //不在多边形中
            {
                *identify_room = 61; //无障碍区域不在分区房间中的房间标识  
            }
        }
    }   
}
static int  sys_msg_map_recieved(DATA_MAP *map) {

    static uint32_t map_frame_id = 0;
	if(NULL == map)
	{
        printf("NULL == map\n");
		return -1;
	}
    if(u_log_flag)
    {
        send_map_log((char *)map);
    }
   	//DATA_MAP *map = (DATA_MAP *)map_data;   /* V1版本的hoslam地图协议 */
    DATA_MAP_V2 *map_v2 = (DATA_MAP_V2 *)map;  /* V2版本的hoslam地图协议 */
   	int send_map_size;
    
     #ifndef ROOMS_MAP_FLAG
     { 
        int map_size = map->w * map->h;
		 send_map_size = ((map_size + 3) - ((map_size + 3) % 4)) / 4;

        SEND_MAP *send_map = (SEND_MAP *)malloc(sizeof(SEND_MAP) + send_map_size);
        memset(send_map, 0, sizeof(SEND_MAP) + send_map_size);
        if (send_map == NULL) {
            printf("send_map: malloc ERROR!\n");
            return -1;
        }

        send_map->head = map_frame_id++;
        send_map->flag = 1;
        send_map->w  = map->w;
        send_map->h  = map->h;
        send_map->ox  = map->ox * 10;  //放大10倍传输
        send_map->oy  =((float)map->h - map->oy) * 10;
        //充电座坐标
        send_map->charger_x = map->dock_x * 10;
        send_map->charger_y = map->dock_y * 10;
        
        unsigned char p[4];
        unsigned char d, type;
        int i, j;

#if 0
    	for (i = 0; i < 16; i++)
    	{
    		printf("%d ", (unsigned char)(map->buf[i]));
    	}
    	puts("");
#endif
    	for (i = 0; i < send_map_size; i++) {
    		for (j = 0; j < 4; j++) {
                if ((4 * i + j) < map_size) {
    			    p[j] = map->buf[4 * i + j];
                } else {
                    p[j] = 0;
                }
    		}
    		d = 0;
    		for (j = 0; j < 4; j++) {
    			if (p[j] < 120) {
    				type = 1;	// 障碍物
    			} else if (p[j] < 130) {
    				type = 3;	// 未知/背景
    			} else {
    				type = 0;	// 无障碍区域
    			}
    			d |= (type << (6 - 2*j));
    		}
    		send_map->map[i] = d;
    	}
        printf("map->w=%lu\n",map->w);
        printf("map->h=%lu\n",map->h);
        tuya_iot_upload_map(send_map, sizeof(SEND_MAP) + send_map_size);
        free(send_map);
        return 0;
    }
    #else
    {
        if(1 == map->w) //重置地图，压缩地图版本的过滤
        {
            wifi_app_reset_map();
            #if 0
            pthread_mutex_lock(&map_save_lock);
            if(save_rooms_map != NULL) /*清空地图后，保存的地图也清空*/
            {
              free(save_rooms_map);
              save_rooms_map = NULL;
            }
            pthread_mutex_unlock(&map_save_lock);
            #endif
            map_is_emtry = 0;
            return 0;
        }

        //static DATA_MAP *map = NULL;
        int map_size=0;
        int send_rooms_size = 0;
        int n = 0;
        float mapox,mapoy;
      	#if 0
     	/*if(rec_map!=NULL)
        	map = rec_map;*/
        if(map==NULL)
        {
           printf("map data is NULL!\n");
           // map = &default_map_data;
           return -1;
        }
      	#endif
		
		if(check_is_new_map_data(map_v2->magic))  /* 判断地图协议版本 */
        {
            map_size = map_v2->size;
           // save_data_size = map_size + sizeof(DATA_MAP_V2);
            send_map_size = map_size * 4; /* 一个字节保存四个像素，故要乘以4 */	
            send_map_size =  map->w * map->h;
        }  
        else
        {
            map_size = map->w * map->h;
            //save_data_size = map_size + sizeof(DATA_MAP);
            send_map_size = map_size;
        }     

        #if 0
        if(flag)
        {
            pthread_mutex_lock(&map_save_lock);
            if(save_rooms_map != NULL) {
            free(save_rooms_map);
            save_rooms_map = NULL;
            }
            save_rooms_map = malloc(sizeof(DATA_MAP) + map_size);
            if(NULL == save_rooms_map)
            {
                printf("save_rooms_map: malloc ERROR!\n");
                return -1;
            }

            
            memset(save_rooms_map,0,sizeof(DATA_MAP) + map_size);
            memcpy(save_rooms_map,map,sizeof(DATA_MAP) + map_size);
            pthread_mutex_unlock(&map_save_lock);
            printf("w = %d, h = %d, 0x = %f, 0y = %f,map_size = %d\n",save_rooms_map->w,save_rooms_map->h,save_rooms_map->ox,save_rooms_map->oy,map_size);
        } 
        #endif

        map_is_emtry = 1;
        if(power_on_rooms)//开机更新房间数据,防止开机时APP主界面地图没有分区
        {
           robot_api_send_msg_to_hoslam(TUYA_MSG_ROOMS_MAP_IS_EDITABLE, NULL,0);  
           robot_api_send_msg_to_hoslam(TUYA_MSG_ROOMS_GET_EDITING_ROOMS,NULL,0);
		   power_on_rooms--;
           printf("power_on_rooms:%d\n",power_on_rooms);
        }
        room_spec_arg_t *p_rspec_arg = robot_ctl_get_room_spec_arg();

        pthread_mutex_lock(&rooms_lock);

        //int send_map_size = ((map_size + 3) - ((map_size + 3) % 4)) / 4;
        //int send_map_size = map_size;

        send_rooms_size = 2;
        for(n = 0 ;n < p_rspec_arg->num;++n)
        {
            send_rooms_size = send_rooms_size + sizeof(SEND_ROOM_SPEC_T) + p_rspec_arg->room_spec[n].n_points * sizeof(XYPoint);
            printf("send_rooms_size = %d\n",send_rooms_size);
        }

        SEND_MAP *send_map = (SEND_MAP *)malloc(sizeof(SEND_MAP) + send_map_size + send_rooms_size);
        if (send_map == NULL) {
            printf("send_map: malloc ERROR!\n");
            return -1;
        }
        memset(send_map, 0, sizeof(SEND_MAP) + send_map_size + send_rooms_size);
        send_map->head = map_frame_id++;
       // map_is_editable_flag = 1;
        send_map->flag = 0x01;  //带智能分区tuya版
        send_map->type = map_is_editable_flag; /* 分区是否可编辑标志位 */
        send_map->resolution = 5; /* 地图分辨率 ，0.05m,放大100倍传输 */
        
        printf("map_is_editable_flag = %d\n",map_is_editable_flag);
        printf("send_map->flag  =%d \n",send_map->flag);
    
        unsigned char type,identify_room;
        int i,j,k,ret,not_in_polygon_flag;
        float point_x,point_y;

        if(check_is_new_map_data(map_v2->magic))  /* 判断地图协议版本 */
        {
            send_map->w  = map_v2->w;
            send_map->h  = map_v2->h;
            send_map->ox  = map_v2->ox * 10;  //放大10倍传输
            send_map->oy  =((float)map_v2->h - map_v2->oy) * 10;
            mapox = map_v2->ox;
            mapoy = (float)(map_v2->h - map_v2->oy);
            //充电座坐标
            send_map->charger_x = map_v2->dock_x * 10;
            send_map->charger_y = map_v2->dock_y * 10;
            //地图是否可以编辑
            //send_map->editable = map_is_editable_flag;
            room_vertex_coordinate_conversion(mapox,mapoy);
         
            for (i = 0; i < send_map_size; i++)
            {
                point_x = i % map_v2->w;   //该点在坐标系中的横坐标
                point_y = i / map_v2->w;   //该点在坐标系中的纵坐标

                switch(i % 4)
                {
                    case 0: 
                            deal_with_map_room_pixel((map_v2->buf[i/4] >> 6),&type,&identify_room,point_x,point_y);                          
                            break;
                    case 1:
                            deal_with_map_room_pixel((map_v2->buf[i/4] >> 4),&type,&identify_room,point_x,point_y);
                            break;
                    case 2:
                            deal_with_map_room_pixel((map_v2->buf[i/4] >> 2),&type,&identify_room,point_x,point_y);
                            break;
                    case 3:
                            deal_with_map_room_pixel((map_v2->buf[i/4] >> 0),&type,&identify_room,point_x,point_y);
                            break;
                    default:
                            break;
                }
                 send_map->map[i] = (identify_room << 2) | type;
            }
        }
        else
        {
            send_map->w  = map->w;
            send_map->h  = map->h;
            send_map->ox  = map->ox * 10;  //放大10倍传输
            send_map->oy  =((float)map->h - map->oy) * 10;
            mapox = map->ox;
            mapoy = (float)(map->h - map->oy);
            //充电座坐标
            send_map->charger_x = map->dock_x * 10;
            send_map->charger_y = map->dock_y * 10;
            //地图是否可以编辑
            //send_map->editable = map_is_editable_flag;

            room_vertex_coordinate_conversion(mapox,mapoy);
            for (i = 0; i < map_size; i++)
            {
                not_in_polygon_flag = 0;
                point_x = i % map->w;   //该点在坐标系中的横坐标
                point_y = i / map->w;   //该点在坐标系中的纵坐标
                if (map->buf[i] < 120) 
                {
                    type = 1;	// 障碍物
                    identify_room = 62; //障碍物的房间标识
                } 
                else if (map->buf[i] < 130) 
                {
                    type = 3;	// 未知/背景
                    identify_room = 63; //未知的房间标识
                } 
                else 
                {
                    type = 0;	// 无障碍区域
                    if(0 == p_rspec_arg->num)
                    {
                        identify_room = 60; //没有分房间数据
                    // printf("no room data\n");
                    }
                    else
                    {
                        for(k = 0;k < p_rspec_arg->num;++k)
                        {
                            ret = PtInPolygon(room_vertex_coordinate[k].points_num,room_vertex_coordinate[k].spec, point_x, point_y);
                            if(1 == ret)
                            {
                                identify_room = p_rspec_arg->room_spec[k].id;
                                not_in_polygon_flag = 1;
                                //printf("identify_room = %d\n",p_rspec_arg->room_spec[k].id);
                                break;
                            }
                        }
                        if(0 == not_in_polygon_flag)  //不在多边形中
                        {
                            identify_room = 61; //无障碍区域不在分区房间中的房间标识  
                        }
                    }
                }
                send_map->map[i] = (identify_room << 2) | type;
            }
        }

        send_map->len_before_lz4 = send_rooms_size + send_map_size;

        printf("send_map->len_before_lz4 = %d\n",send_map->len_before_lz4);

        SEND_ROOM_SPEC_T *send_rooms;
        uint8_t *send_rooms_id;
        int x,y;
        uint16_t send_room_x,send_room_y;

        send_rooms_id = (((char *)send_map) +  sizeof(SEND_MAP) + send_map_size);
        *send_rooms_id = 0x01;
        *(send_rooms_id + 1) = p_rspec_arg->num;
       
        if(0 < p_rspec_arg->num)
        {
            send_rooms = (SEND_ROOM_SPEC_T *)(send_rooms_id + 2);
        }

        for(i = 0 ; i < p_rspec_arg->num;++i)
        {
            send_rooms->id = p_rspec_arg->room_spec[i].id;                     
            send_rooms->id  = htons(send_rooms->id);
			
            send_rooms->order = p_rspec_arg->room_spec[i].order + 1;
            send_rooms->order = htons(send_rooms->order);

            if(p_rspec_arg->room_spec[i].bundle[5] == 1) /*判断该房间是否被设置过，若被设置过，则上传保存的参数 */
            {
			  send_rooms->clean_count = (uint16_t)p_rspec_arg->room_spec[i].bundle[4]; 
              send_rooms->clean_count = htons(send_rooms->clean_count); 
            }
            else   /* 若未设置过，则上传默认值 */
            {
			  send_rooms->clean_count = p_rspec_arg->room_spec[i].count;
              send_rooms->clean_count = htons(send_rooms->clean_count);
            }
            send_rooms->mop_count = p_rspec_arg->room_spec[i].mop_count;
            send_rooms->mop_count = htons(send_rooms->mop_count);
			
            send_rooms->color_order = p_rspec_arg->room_spec[i].color;

            send_rooms->clean_state = p_rspec_arg->room_spec[i].forbidden;

            send_rooms->mop_state = p_rspec_arg->room_spec[i].mop_forbidden;
			
            send_rooms->vac_state = wifi_app_data_map_tuya_fan(p_rspec_arg->room_spec[i].bundle[0]);
			
            send_rooms->water_state = wifi_app_data_map_tuya_water_level(p_rspec_arg->room_spec[i].bundle[1]);
			
            send_rooms->Y_mop_state = !(p_rspec_arg->room_spec[i].bundle[2]);
			
            send_rooms->n_points = (uint8_t)p_rspec_arg->room_spec[i].n_points;

            send_rooms->room_name_len = p_rspec_arg->room_spec[i].name_pack.name_len;  /* 拷贝房间名字 */
            memset(send_rooms->room_name,0,TUYA_NAME_LEN);
            memcpy(send_rooms->room_name,p_rspec_arg->room_spec[i].name_pack.name,p_rspec_arg->room_spec[i].name_pack.name_len);

            printf("room_name_len = %d,name = %s\n",send_rooms->room_name_len,send_rooms->room_name);
            printf("room id :%d,order :%d,count :%d,mop_count :%d,color :%d,forbidden :%d,mop_forbidden :%d,vac :%d,water :%d,Y :%d\n", 
            p_rspec_arg->room_spec[i].id,
            p_rspec_arg->room_spec[i].order,
            p_rspec_arg->room_spec[i].count,
            p_rspec_arg->room_spec[i].mop_count,
            p_rspec_arg->room_spec[i].color,
            p_rspec_arg->room_spec[i].forbidden,
            p_rspec_arg->room_spec[i].mop_forbidden,
            wifi_app_data_map_tuya_fan(p_rspec_arg->room_spec[i].bundle[0]),
            wifi_app_data_map_tuya_water_level(p_rspec_arg->room_spec[i].bundle[1]),
            !(p_rspec_arg->room_spec[i].bundle[2]));
            printf("room_n_points = %d,i = %d\n", p_rspec_arg->room_spec[i].n_points, i);
            for(j = 0;j <  p_rspec_arg->room_spec[i].n_points;++j)
            {
                x = (-p_rspec_arg->room_spec[i].points[j].y * 20 * 10); //float型，放大10倍传输
                y = (p_rspec_arg->room_spec[i].points[j].x * 20 * 10);
                //printf("x = %d,y = %d\n",x,y);
                send_rooms->points[j].x = (int16_t)x;
                send_rooms->points[j].y = (int16_t)y;
                //printf("x = %f,y = %f\n",p_rspec_arg->room_spec[i].points[j].y ,p_rspec_arg->room_spec[i].points[j].x);
                //printf("x = %d,y = %d\n",send_rooms->points[j].x,send_rooms->points[j].y);
                send_rooms->points[j].x =  htons(send_rooms->points[j].x);
                send_rooms->points[j].y =  htons(send_rooms->points[j].y);
            }

            send_rooms = (SEND_ROOM_SPEC_T *)(((char *)send_rooms) + sizeof(SEND_ROOM_SPEC_T) + sizeof(XYPoint) * p_rspec_arg->room_spec[i].n_points);
            printf("SEND_ROOM_SPEC_T = %d,one room size = %d\n",sizeof(SEND_ROOM_SPEC_T),sizeof(SEND_ROOM_SPEC_T)+sizeof(XYPoint) * p_rspec_arg->room_spec[i].n_points);

        }
        pthread_mutex_unlock(&rooms_lock);
        
        printf("p_rspec_arg->num = %d\n",p_rspec_arg->num);
        printf("send_rooms_size=%d\n",send_rooms_size);
        printf("map->w=%lu，map->h=%lu\n",map->w,map->h);
        printf("map->ox = %f,map->oy = %f\n",map->ox,map->oy);
        printf("mapox=%f,mapoy=%f\n",mapox,mapoy);
        printf("send_map->head=%d\n",send_map->head);
        printf("send_map->flag=%d\n",send_map->flag);
        printf("send_map_size=%d\n",send_map_size);
        //set_send_rooms_size_compress(send_rooms_size);
        tuya_iot_upload_map(send_map, sizeof(SEND_MAP) + send_map_size + send_rooms_size);
        free(send_map);
        return 0;
   }
    #endif
}

#if 0
void send_map_rooms_data(void)
{
    sys_msg_map_recieved((DATA_MAP *)save_rooms_map,0);
}
#endif

// -1 reset后的路径，直接过滤掉
static int check_path_reset(const char *data) {
    DATA_PATH *path = (DATA_PATH*)data;

    if(path_reset_flag && last_path_num > 0)
    {
        path_filter_count++;
        if(path_filter_count < 3 && path->count >= last_path_num)
        {
            last_path_num = path->count;
            return -1;
        }
        path_filter_count = 0;
        path_reset_flag = false;
    }

    last_path_num = path->count;
    return 0;
}

#ifndef HIDE_CLEAN_NAVI_PATH
static int sys_msg_trajectory_recieved(DATA_PATH *path) {
    static uint32_t path_frame_id = 0;
    if(check_path_reset((char*)path) < 0) {
        printf("skip path\n");
        return -1;
    }
    if(u_log_flag)
    {
        sys_msg_trajectory_recieved_log((char *)path);
    }


    int send_len = sizeof(SEND_PATH) + path->count * sizeof(XYPoint);
    SEND_PATH *send_path = (SEND_PATH *)malloc(send_len);

    if (send_path == NULL) {
        printf("send_path: malloc ERROR!\n");
        return -1;
    }
    memset(send_path, 0, send_len);
    //printf("recv_path count:%d\n", path->count);

    int x, y;
    for (int i = 0; i < path->count; i++) {
        x = -(path->position[i].y * 20 * 10); //float型，放大10倍传输
        y = -(path->position[i].x * 20 * 10);
        send_path->path[i].x = (uint16_t)x;
        send_path->path[i].y = (uint16_t)y;
        //printf("ox %.2f, oy %.2f, x %d, y %d, sx %d, sy %d \n", path->position[i].y, path->position[i].x, x, y, send_path->path[i].x, send_path->path[i].y);

        //cal_grid_position(path->position + i);
        //send_path->path[i].x = map_ox - (path->position[i].y*20);  //APP执行计算
        //send_path->path[i].y = map_oy - (path->position[i].x*20);
    }
    send_path->head = path_frame_id++;
    send_path->flag = 0;
    send_path->num = path->count;
    //上传涂鸦云端
    tuya_iot_upload_path(send_path, send_len);
    free(send_path);
    return 0;
}

static int wifi_app_msg_send_inc_type_path(const char *data) {
    return 0;
}
#else
static int sys_msg_trajectory_recieved(DATA_PATH *path) {
    return 0;
}

static int wifi_app_msg_send_inc_type_path(const char *data,uint8_t path_type) {
    static uint32_t path_frame_id = 0;
    static int last_time = 0;
    uint16_t direction;
    int32_t theta;
    DATA_CURR_POSE_AND_PATH *path = (DATA_CURR_POSE_AND_PATH*)data;
   
    if(check_path_reset(data) < 0) {
        printf("skip path\n");
        return -1;
    }

    if(u_log_flag && path_type == 0x02)
    {
        send_path_log((char *)data);
    }
    if(u_log_flag && path_type == 0x03)
    {
        sys_msg_navigation_recieved_log((char *)path);
    }
    /* 1 x sizeof(XYPoint) 是current point 的坐标 */
    int send_len = sizeof(SEND_PATH) + path->path_point_count * sizeof(XYPoint) + 1*sizeof(XYPoint);
    SEND_PATH *send_path = (SEND_PATH *)malloc(send_len);

    if (send_path == NULL) {
        printf("send_path: malloc ERROR!\n");
        return -1;
    }
    memset(send_path, 0, send_len);
    send_path->type = path_type;
    //printf(" ### path point : \n");
    int x, y;
    for (int i = 0; i < path->path_point_count; i++) {
        // x = -round_float_to_int(path->position[i].y * 20 * 10);
       // y = -round_float_to_int(path->position[i].x * 20 * 10);
        x = -(path->position[i].y * 20 * 10); //float型，放大10倍传输
        y = (path->position[i].x * 20 * 10);
        send_path->path[i].x = (uint16_t)x;
        send_path->path[i].y = (uint16_t)y;

        if(path->position[i].type == 1) /* 不显示的路径 */
        {
            send_path->path[i].y = (send_path->path[i].y & (0xfffe));
            send_path->path[i].x = (send_path->path[i].x | (0x0001));
        }
        else
        {
            //type==0 要显示的路径, to app 协议要在y的最后1bit中设为1
            send_path->path[i].y = (send_path->path[i].y & (0xfffe));
            send_path->path[i].x = (send_path->path[i].x & (0xfffe));
        }
        if(send_path->type  == 0x03)
        {
           printf("[%x %x] ", send_path->path[i].x, send_path->path[i].y);
        }

    }
    //printf("\n");

    /* 增加当前点到路径最后 */
     //x = -round_float_to_int(path->cur_pose.y * 20 * 10);
    //y = -round_float_to_int(path->cur_pose.x * 20 * 10);
    printf(" ### cur_pose %f, %f ### \n", path->cur_pose.x, path->cur_pose.y);
    x = -(path->cur_pose.y * 20 * 10); //float型，放大10倍传输
    y = (path->cur_pose.x * 20 * 10);
    send_path->path[path->path_point_count].x = (uint16_t)x;
    send_path->path[path->path_point_count].y = (uint16_t)y;
    /* 默认显示当前点 */
    send_path->path[path->path_point_count].y = (send_path->path[path->path_point_count].y & (0xfffe));
    send_path->path[path->path_point_count].x = (send_path->path[path->path_point_count].x & (0xfffe));
    printf("send_path->path[path->path_point_count].x = %d,send_path->path[path->path_point_count].y = %d\n",send_path->path[path->path_point_count].x,send_path->path[path->path_point_count].y);
    theta = (int32_t)(path->cur_pose.theta/3.1415 * 180  + 90) ; /* 获取机器运动方向的角度 */
    if(theta  < 0)
    {
            theta += 360; 
    }
    send_path->direction = (uint16_t)theta;

    printf(" ### current point %x, %x ### \n", send_path->path[path->path_point_count].x, send_path->path[path->path_point_count].y);
   
    send_path->head = path_frame_id++;
    printf("send_path->head = %d\n",send_path->head);
    send_path->flag = 0;
    send_path->num = path->path_point_count + 1;
    printf("send_path->num = %d ,upload_cur_pose_flag:%d\n",send_path->num, upload_cur_pose_flag);
    
    if(0 == send_path->num)
    {
        printf("ERROR: send_path->num=0\n");
        return -1;
    }

    // 统计时间
    struct timeval tv2;
    gettimeofday(&tv2,NULL);
    int now_time = tv2.tv_sec*1000 + tv2.tv_usec/1000;
    if(last_time == 0)
    {
        last_time = now_time;
    }

    last_time = now_time;
  	if(upload_cur_pose_flag || path->path_point_count > 1)
    {
    	//上传涂鸦云端
    	tuya_iot_upload_path(send_path, send_len);
    }
    free(send_path);
    return 0;
}
#endif

DATA_PATH *save_navigation_path;

static int sys_msg_navigation_recieved(DATA_PATH *path,char flag) {
    static uint32_t navi_frame_id = 0;
    static uint8_t first_send_path;
    int len;
    if(save_navigation_path != NULL && flag) {
        printf("free save_navigation_path2\n");
        free(save_navigation_path);
        save_navigation_path = NULL;

    }
    
    if(u_log_flag)
    {
        sys_msg_navigation_recieved_log((char *)path);
    }

    if(flag)
    {
        len = sizeof(DATA_PATH) +  path->count*sizeof(DATA_POSITION);
        save_navigation_path = (DATA_PATH *)malloc(len);
        if (!save_navigation_path)
        {
            printf("save_navigation_path: malloc ERROR!\n");
            return -1;
        }
        memcpy(save_navigation_path,path,len);
        printf("new nav\n");
    }
    
    int send_len = sizeof(SEND_PATH) + path->count*sizeof(XYPoint);
    SEND_PATH *send_path = (SEND_PATH *)malloc(send_len);
    if (!send_path)
    {
        printf("send_navi: malloc ERROR!\n");
        return -1;
    }
    memset(send_path, 0, send_len);

    int x, y;
    printf("path->count = %d\n",path->count);
    for (int i = 0; i < path->count; i++)
    {
        //printf("nav0 : x = %f,y = %f\n",path->position[i].x,path->position[i].y);
        x = (path->position[i].x * 20 * 10); //float型，放大10倍传输
        y = (path->position[i].y * 20 * 10);
        send_path->path[i].x = (uint16_t)x;
        send_path->path[i].y = (uint16_t)y;
        send_path->path[i].y = (send_path->path[i].y & (0xfffe));  /* 导航线默认显示 */
        send_path->path[i].x = (send_path->path[i].x & (0xfffe));

    }

    send_path->head = navi_frame_id++;
    printf("nav send_path->head = %d\n",send_path->head);
    send_path->flag = 0x00;
    send_path->type = 0x03;
    send_path->num = path->count;

    tuya_iot_upload_navigation(send_path, send_len);
    free(send_path);

    return 0;
}

int app_get_send_navigation(void)
{
    if(save_navigation_path)
   {
        sys_msg_navigation_recieved(save_navigation_path,0);
   }
}

static int sys_msg_upgrade_status_recieved(UPGRADE_STATUS *upgrade_status) {
    switch (upgrade_status->upgrade_type)
    {
        case UPGRADE_TYPE_DEFAULT:
            if (upgrade_status->upgrade_status == UPGRADE_FAILED)
            {
                printf("upgrade failed\n");
                iot_upgrade_progress(100);
            }
            else if (upgrade_status->upgrade_status == UPGRADE_COMPLETE)
            {
                printf("upgrade success\n");
                iot_upgrade_progress(100);
            }
            break;

        case UPGRADE_TYPE_APP_SERVER:
        case UPGRADE_TYPE_HOSLAM:
        case UPGRADE_TYPE_VOICE:
        case UPGRADE_TYPE_AFW:
            if (upgrade_status->upgrade_status == UPGRADE_GOING) {
                printf("upgrade %d\n", upgrade_status->upgrade_progress);
                iot_upgrade_progress(upgrade_status->upgrade_progress);
            }
            break;
        case UPGRADE_TYPE_ROBOT_MSG:
            inform_robot_upgrade_state(upgrade_status->upgrade_status);
            break;
        default:
            break;
    }

    return 0;
}

#if 0
int sys_msg_get_rooms(uint8_t *data)
{
	uint32_t len;

    memcpy(&len, data, sizeof(len));
	
	int room_count,send_rooms_msg_size,i,j;

	ROOM_SPEC_T *room_msg = (ROOM_SPEC_T *)(data + sizeof(len));
    room_count = len / sizeof(ROOM_SPEC_T);
    save_room_count = room_count;
	printf("room_count:%d\n",room_count);
	
    send_rooms_msg_size = 2 + room_count * sizeof(SEND_ROOM_SPEC_T); //智能分区ID （1bytes）+ 个数(1bytes)

	char *send_get_room = (char *)malloc(send_rooms_msg_size);
    memset(send_get_room, 0, send_rooms_msg_size);
    if (send_get_room == NULL) {
        printf("send_get_room: malloc ERROR!\n");
        return -1;
    }

	*send_get_room = 0x01;  //智能分区ID
	*(send_get_room + 1) = (char)room_count;
	SEND_ROOM_SPEC_T* room = (SEND_ROOM_SPEC_T*)(send_get_room + 2);

    for(j = 0;j < room_count;j++ )
    {
    //    sys_log(LOG_INFO, "id:%f,clean_count:%f",send_rooms_msg->room[j].id,send_rooms_msg->room[j].count);
    //   sys_log(LOG_INFO, "order=:%d,forbidden=:%d,n_points=%d",send_rooms_msg->room[j].order,send_rooms_msg->room[j].forbidden,send_rooms_msg->room[j].n_points);
		room[j].forbidden = room_msg[j].forbidden;   //高bytes标识区域属性
		room[j].n_points= room_msg[j].n_points;
		printf("room[%d]:%d\n",j,room_msg[j].forbidden);
		printf("room[%d]->n_points:%d\n",j,room_msg[j].n_points);

        //保存分区房间信息
        save_room[j].id = room_msg[j].id;
        save_room[j].n_points = room_msg[j].n_points;

        for(i = 0; i < room_msg[j].n_points;i++)
        {
        	room[j].points[i].x =room_msg[j].points[i].x * 20 * 10;
			room[j].points[i].y =room_msg[j].points[i].y * 20 * 10;
            printf("room[%d]->points[%d]:%hd\n",j,i,room[j].points[i].x);
			printf("room[%d]->points[%d]:%hd\n",j,i,room[j].points[i].y);

            //保存保存分区房间顶点信息
            save_room[j].x[i] = room_msg[j].points[i].x * 20;
            save_room[j].y[i] = room_msg[j].points[i].y * 20;
        }
        
    }
	//send_get_room; //发送给anker的数据
	
    free(send_get_room);
	return 0;
}
#endif

void robot_notify_sys_msg_cb(ROBOT_SYS_MSG_E type, void *param) {
    char map_file[128];
    switch(type) {
    case ROBOT_SYS_MSG_MAP:
        sys_msg_map_recieved((DATA_MAP *)param);
        break;
    case ROBOT_SYS_MSG_TRAJECTORY:
        sys_msg_trajectory_recieved((DATA_PATH *)param);
        break;
    case ROBOT_SYS_MSG_NAVIGATION:
        sys_msg_navigation_recieved((DATA_PATH *)param,1);
        //wifi_app_msg_send_inc_type_path((uint8_t *)param,0x03);
        break;
    case ROBOT_TRAJECTORY_PATH:   //隐藏导航线   
        wifi_app_msg_send_inc_type_path((uint8_t *)param,0x02);
        break;
    case ROBOT_SYS_MSG_UPDATE_STATUS:
        sys_msg_upgrade_status_recieved((UPGRADE_STATUS *)param);
        break;
    case ROBOT_SYS_MSG_MAP_RESET:
        path_reset_flag = true;
        path_filter_count = 0;
        robot_ctl_notify_map_reset();
        break;
    case ROBOT_SYS_MSG_VIRTUAL_WALL:
        robot_ctl_sync_vitual_wall((uint8_t *)param);
        break;
    case ROBOT_SYS_MSG_WAKEUP:
		printf("\033[1m\033[40;33m[%s:%d] robot_notify_sys_msg_cb ROBOT_SYS_MSG_WAKEUP \033[0m\n",__FUNCTION__,__LINE__);
        robot_app_handle_sys_msg_wakeup();
		robot_api_send_msg_to_hoslam(TUYA_MSG_TYPE_WAKEUP, NULL, 0);
        break;
	case ROBOT_SYS_MSG_SLEEP:
		printf("\033[1m\033[40;33m[%s:%d] robot_notify_sys_msg_cb ROBOT_SYS_MSG_SLEEP \033[0m\n",__FUNCTION__,__LINE__);
        robot_app_handle_sys_msg_sleep();
        break;
    case ROBOT_SYS_PLAY_VOICE_END:
        robot_app_handle_sys_msg_voice_end();
        break;
    case ROBOT_ROOMS_SEGMENT:
        rooms_segment_process((uint8_t *)param);
        break;
    case ROBOT_SET_ROOMS:
         rooms_set_process((uint8_t *)param);
        break;
    case ROBOT_SYS_MSG_GET_ROOMS:
        //rooms_map_flag = 1;
        robot_ctl_sync_room_spec((uint8_t *)param);
        break;
    case ROBOT_SYS_MSG_GET_EDIT_ROOMS:
        //rooms_map_flag = 1;
        robot_ctl_sync_edit_room_spec((uint8_t *)param);
		get_editing_room_flag = true;
        break;
    case ROBOT_ROOMS_SPLIT:
        map_split_process((uint8_t *)param);
        break;
    case ROBOT_ROOMS_MERGER:
        rooms_merger_process((uint8_t *)param);
        break;
    case ROBOT_MAP_IS_STABLE:
        map_is_stable_process((uint8_t *)param); 
        break;
    case ROBOT_ROOMS_IS_MANUAL:
        rooms_is_manual_process((uint8_t *)param);
        break;
    case ROBOT_CLEAR_ROOMS:
        rooms_clean_rooms_process((uint8_t *)param);
        break;
   
    case  ROBOT_CURRENT_MAP_ID:
        save_map_id((uint8_t *)param);
        break;
      
	case  ROBOT_CURRENT_MAP:
        creat_pthread_save_map_to_cloud((uint8_t *)param);;
         strncpy(map_file, (uint8_t *)param + 4, sizeof(map_file));
         printf("ROBOT_CURRENT_MAP = %d,file = %s\n",*(uint32_t *)(param),map_file);
        break;

    case  ROBOT_CURRENT_MAP_RESULT:
         wifi_app_msg_dp_send_set_map_rusult_ret((uint8_t *)param);
         break;

    case ROBOT_SET_VW_FZ_RESULT:
         //wifi_app_msg_dp_send_vwall_foridden_ret((uint8_t *)param);
         break;

   case ROROT_SET_CLEAN_ROOMS:
        wifi_app_msg_dp_send_set_choice_rooms_ret((uint8_t *)param);
        break;
   case ROBOT_ROOMS_MAP_IS_EDITABLE:
        set_map_is_editable((uint8_t *)param);
        //wifi_app_msg_dp_send_map_is_editable((uint8_t *)param);
        break;

   case ROBOT_MSG_GET_LOG:
        if(u_log_flag)
            {
                write_u_log((uint8_t *)param);
            }
        break;
   case ROBOT_ROOMS_GET_CURRENT_ROOM:
        printf("ROBOT_ROOMS_GET_CURRENT_ROOM = %d\n",*(uint8_t *)param);
        if(get_custom_mode_switch())
         {
            recv_set_clean_strategy((uint8_t *)param);
         }
        break;  
    }            
    


}
