#include "tuya_cloud_com_defs.h"
#include "tuya_dp_enum.h"
#include "robot_app_msg.h"
#include "robot_control_adapter.h"
#include "tuya_dp_data_mapping.h"
// 从指定的数据结构映射表，获取对应的协议数据
static int wifi_app_data_map(const PROTOCOL_MAP *map, int items, int *protocol_enum, int *dp_enum)
{
    int i;

    if (*protocol_enum != INVALID_ENUM)
    {
        for (i = 0; i < items; i++)
        {
            if (map[i].protocol_enum == *protocol_enum)
            {
                *dp_enum = map[i].dp_enum;
                return 0;
            }
        }

        return -1;
    }

    if (*dp_enum != INVALID_ENUM)
    {
        for (i = 0; i < items; i++)
        {
            if (map[i].dp_enum == *dp_enum)
            {
                *protocol_enum = map[i].protocol_enum;
                return 0;
            }
        }

        return -1;
    }

    return -1;
}

// 上报的状态
#if 0
static const PROTOCOL_MAP RobotStatusMap[] =
{
    {STATUS_EDGE_CLEANING,      STATE_WALLFOLLOW        },
    {STATUS_LOCAL_CLEANING,     STATE_SPRIAL            },
    {STATUS_AUTO_CLEANING,      STATE_AUTOCLEAN         },
    {STATUS_GOING_BACK,         STATE_GOBACK            },
    {STATUS_STANDBY,            STATE_STANDBY           },
    {STATUS_SLEEP,              STATE_STANDBY           },
    {STATUS_SEAT_CHARGING,      STATE_CHARGING          },
    {STATUS_DIRECT_CHARGING,    STATE_CHARGING          },
    {STATUS_CHARGE_DONE,        STATE_STANDBY           },
    {STATUS_MALFUNCTION,        STATE_STANDBY           },
    {STATUS_MANUAL_CONTROL,     STATE_STANDBY           },
};

int wifi_app_data_map_robot_status(int protocol_enum)
{
    int dp_enum = INVALID_ENUM;
    int items = sizeof(RobotStatusMap) / sizeof(PROTOCOL_MAP);

    wifi_app_data_map(RobotStatusMap, items, &protocol_enum, &dp_enum);
    return dp_enum;
}
#endif

// 下发的命令
static const PROTOCOL_MAP CleanModeMap[] =
{
    {SCENE_SPOT_CLEAN,      WORK_MODE_PART          },
    {SCENE_AUTO_CLEAN,      WORK_MODE_SMART         },
    {SCENE_DOCK,            WORK_MODE_CHARGEGO      },
    {SCENE_GOTO_POS,        WORK_MODE_POSE          },
    {SCENE_RECT_CLEAN,      WORK_MODE_ZONE          },
    {SCENE_WALLFOLLOW,      WORK_MODE_WALLFOLLOW    },
};

int wifi_app_data_map_clean_mode(int dp_enum)
{
    int protocol_enum = INVALID_ENUM;
    int items = sizeof(CleanModeMap) / sizeof(PROTOCOL_MAP);

    wifi_app_data_map(CleanModeMap, items, &protocol_enum, &dp_enum);
    return protocol_enum;
}

static const PROTOCOL_MAP ManualControlMap[] =
{
    {CTRL_GO_FORWARD,           CTRL_FORWARD            },
    {CTRL_GO_LEFT,              CTRL_TURNLEFT           },
    {CTRL_GO_RIGHT,             CTRL_TURNRIGHT          },
    {CTRL_GO_BACKWARDS,         CTRL_BACKWARD           },
    {CTRL_STOP_MOVING,          CTRL_STOP               },
};

int wifi_app_data_map_manual_control(int dp_enum)
{
    int protocol_enum = INVALID_ENUM;
    int items = sizeof(ManualControlMap) / sizeof(PROTOCOL_MAP);

    wifi_app_data_map(ManualControlMap, items, &protocol_enum, &dp_enum);
    return protocol_enum;
}

static const PROTOCOL_MAP FanModeMap[] =
{
	{CTRL_FAN_SILENT,           FAN_SILENT              },
    {CTRL_FAN_NORMAL,           FAN_NORMAL              },
    {CTRL_FAN_STRONG,           FAN_STRONG              },
    {CTRL_FAN_NO_SET,           FAN_NO_SET              },
};

int wifi_app_data_map_fan_mode(int dp_enum)
{
    int protocol_enum = INVALID_ENUM;
    int items = sizeof(FanModeMap) / sizeof(PROTOCOL_MAP);

    wifi_app_data_map(FanModeMap, items, &protocol_enum, &dp_enum);
    return protocol_enum;
}

int wifi_app_data_map_tuya_fan(int protocol_enum)
{
    int dp_enum = INVALID_ENUM;
    int items = sizeof(FanModeMap) / sizeof(PROTOCOL_MAP);

    wifi_app_data_map(FanModeMap, items, &protocol_enum, &dp_enum);
    return dp_enum;
}

static const PROTOCOL_MAP WaterLevelMap[] = {
    {CTRL_WATER_LEVEL_DEFAULT,       WATER_LEVEL_2 },
    {CTRL_WATER_LEVEL_HIGH,          WATER_LEVEL_3 },
    {CTRL_WATER_LEVEL_LOW,           WATER_LEVEL_1 },
    {CTRL_WATER_LEVEL_OFF,           WATER_LEVEL_OFF},
};

int wifi_app_data_map_water_level(int dp_enum)
{
    int protocol_enum = INVALID_ENUM;
    int items = sizeof(WaterLevelMap) / sizeof(PROTOCOL_MAP);

    wifi_app_data_map(WaterLevelMap, items, &protocol_enum, &dp_enum);
    return protocol_enum;
}

int wifi_app_data_map_tuya_water_level(int protocol_enum)
{
    int dp_enum = INVALID_ENUM;
    int items = sizeof(WaterLevelMap) / sizeof(PROTOCOL_MAP);

    wifi_app_data_map(WaterLevelMap, items, &protocol_enum, &dp_enum);
    return dp_enum;
}