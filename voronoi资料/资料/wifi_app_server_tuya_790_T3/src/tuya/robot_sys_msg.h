#ifndef __ROBOT_SYS_MSG_H__
#define __ROBOT_SYS_MSG_H__

#include "robot_api.h"

void robot_notify_sys_msg_cb(ROBOT_SYS_MSG_E type, void *param);
int get_map_is_emtry(void);
int set_map_is_editable_flag(uint8_t enable_value);

int app_get_send_navigation(void);



#endif /* __ROBOT_SYS_MSG_H__ */
