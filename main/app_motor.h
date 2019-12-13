#ifndef _APP_MOTOR_H_
#define _APP_MOTOR_H_

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

// D   A
//   o
// B   C

#define PWMD 0
#define DIRD 2
#define PWMC 25
#define DIRC 4
#define PWMB 26
#define DIRB 27
#define PWMA 21
#define DIRA 22

typedef struct app_motor_* app_motor_handle_t;

app_motor_handle_t app_motor_init();
esp_err_t app_motor_control_forward(app_motor_handle_t motor_handle, int speed);
esp_err_t app_motor_control_backward(app_motor_handle_t motor_handle, int speed);
esp_err_t app_motor_control_turn_left(app_motor_handle_t motor_handle, int speed);
esp_err_t app_motor_control_turn_right(app_motor_handle_t motor_handle, int speed);
esp_err_t app_motor_control_rotate_left(app_motor_handle_t motor_handle, int speed);
esp_err_t app_motor_control_rotate_right(app_motor_handle_t motor_handle, int speed);
esp_err_t app_motor_control_stop(app_motor_handle_t motor_handle);

#endif