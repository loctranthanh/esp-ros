#ifndef _ROS_PUBLISHER_H_
#define _ROS_PUBLISHER_H_

#include "freertos/FreeRTOS.h"

typedef struct {
    char*       topic;
    int         id;
    
} ros_publisher_config_t;

typedef struct ros_publisher_* ros_publisher_handle_t;

ros_publisher_handle_t ros_publisher_init(ros_publisher_config_t* pub_cfg);
esp_err_t ros_publisher_publish(ros_publisher_handle_t pub_handle, )

#endif