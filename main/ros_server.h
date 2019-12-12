#ifndef _ROS_SERVER_H_
#define _ROS_SERVER_H_

#include "freertos/FreeRTOS.h"

typedef struct {
    char*           ros_master_ip;
    char*           unique_name;
} ros_server_config_t;

typedef struct ros_server_* ros_server_handle_t;

ros_server_handle_t ros_server_init(ros_server_config_t* ros_server_cfg);
esp_err_t ros_server_new_publisher(ros_server_handle_t server_handle, char* topic);

#endif