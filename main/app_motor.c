#include "app_motor.h"
#include "math.h"

typedef struct app_motor_ {
    ledc_channel_t      motor_a;
    ledc_channel_t      motor_b;
    ledc_channel_t      motor_c;
    ledc_channel_t      motor_d;
    uint8_t             dir_a;
    uint8_t             dir_b;
    uint8_t             dir_c;
    uint8_t             dir_d;
} app_motor_t;

app_motor_handle_t app_motor_init()
{
    app_motor_handle_t motor_handle = malloc(sizeof(app_motor_t));
    motor_handle->motor_a = LEDC_CHANNEL_0;
    motor_handle->motor_b = LEDC_CHANNEL_1;
    motor_handle->motor_c = LEDC_CHANNEL_2;
    motor_handle->motor_d = LEDC_CHANNEL_3;
    motor_handle->dir_a = DIRA;
    motor_handle->dir_b = DIRB;
    motor_handle->dir_c = DIRC;
    motor_handle->dir_d = DIRD;
    
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_HIGH_SPEED_MODE,           // timer mode
        .timer_num = LEDC_TIMER_0,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t motor_channel_cfg_a = {
        .channel    = motor_handle->motor_a,
        .duty       = 0,
        .gpio_num   = PWMA,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&motor_channel_cfg_a);

    ledc_channel_config_t motor_channel_cfg_b = {
        .channel    = motor_handle->motor_b,
        .duty       = 0,
        .gpio_num   = PWMB,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&motor_channel_cfg_b);

    ledc_channel_config_t motor_channel_cfg_c = {
        .channel    = motor_handle->motor_c,
        .duty       = 0,
        .gpio_num   = PWMC,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&motor_channel_cfg_c);

    ledc_channel_config_t motor_channel_cfg_d = {
        .channel    = motor_handle->motor_d,
        .duty       = 0,
        .gpio_num   = PWMD,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&motor_channel_cfg_d);

    gpio_pad_select_gpio(motor_handle->dir_a);
    gpio_set_direction(motor_handle->dir_a, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(motor_handle->dir_b);
    gpio_set_direction(motor_handle->dir_b, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(motor_handle->dir_c);
    gpio_set_direction(motor_handle->dir_c, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(motor_handle->dir_d);
    gpio_set_direction(motor_handle->dir_d, GPIO_MODE_OUTPUT);

    return motor_handle;
}

void _set_duty(ledc_channel_t channel, int duty_percent)
{
    int target_duty = (pow(2, LEDC_TIMER_13_BIT) - 1) * duty_percent / 100;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, target_duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
}

esp_err_t app_motor_control_forward(app_motor_handle_t motor_handle, int speed)
{
    if (!motor_handle) {
        return ESP_FAIL;
    }
    speed = speed < 0 ? 0 : speed;
    speed = speed > 100 ? 100 : speed;

    gpio_set_level(motor_handle->dir_a, 1);
    gpio_set_level(motor_handle->dir_b, 0);
    gpio_set_level(motor_handle->dir_c, 1);
    gpio_set_level(motor_handle->dir_d, 0);

    _set_duty(motor_handle->motor_a, speed);
    _set_duty(motor_handle->motor_b, speed);
    _set_duty(motor_handle->motor_c, speed);
    _set_duty(motor_handle->motor_d, speed);
    return ESP_OK;
}

esp_err_t app_motor_control_backward(app_motor_handle_t motor_handle, int speed)
{
    if (!motor_handle) {
        return ESP_FAIL;
    }
    speed = speed < 0 ? 0 : speed;
    speed = speed > 100 ? 100 : speed;

    gpio_set_level(motor_handle->dir_a, 0);
    gpio_set_level(motor_handle->dir_b, 1);
    gpio_set_level(motor_handle->dir_c, 0);
    gpio_set_level(motor_handle->dir_d, 1);

    _set_duty(motor_handle->motor_a, speed);
    _set_duty(motor_handle->motor_b, speed);
    _set_duty(motor_handle->motor_c, speed);
    _set_duty(motor_handle->motor_d, speed);
    return ESP_OK;
}

esp_err_t app_motor_control_turn_left(app_motor_handle_t motor_handle, int speed)
{
    if (!motor_handle) {
        return ESP_FAIL;
    }
    speed = speed < 0 ? 0 : speed;
    speed = speed > 100 ? 100 : speed;

    gpio_set_level(motor_handle->dir_a, 1);
    gpio_set_level(motor_handle->dir_b, 0);
    gpio_set_level(motor_handle->dir_c, 0);
    gpio_set_level(motor_handle->dir_d, 1);

    _set_duty(motor_handle->motor_a, speed);
    _set_duty(motor_handle->motor_b, speed);
    _set_duty(motor_handle->motor_c, speed);
    _set_duty(motor_handle->motor_d, speed);
    return ESP_OK;
}

esp_err_t app_motor_control_turn_right(app_motor_handle_t motor_handle, int speed)
{
    if (!motor_handle) {
        return ESP_FAIL;
    }
    speed = speed < 0 ? 0 : speed;
    speed = speed > 100 ? 100 : speed;

    gpio_set_level(motor_handle->dir_a, 0);
    gpio_set_level(motor_handle->dir_b, 1);
    gpio_set_level(motor_handle->dir_c, 1);
    gpio_set_level(motor_handle->dir_d, 0);

    _set_duty(motor_handle->motor_a, speed);
    _set_duty(motor_handle->motor_b, speed);
    _set_duty(motor_handle->motor_c, speed);
    _set_duty(motor_handle->motor_d, speed);
    return ESP_OK;
}

esp_err_t app_motor_control_rotate_left(app_motor_handle_t motor_handle, int speed)
{
    if (!motor_handle) {
        return ESP_FAIL;
    }
    speed = speed < 0 ? 0 : speed;
    speed = speed > 100 ? 100 : speed;

    gpio_set_level(motor_handle->dir_a, 1);
    gpio_set_level(motor_handle->dir_b, 1);
    gpio_set_level(motor_handle->dir_c, 1);
    gpio_set_level(motor_handle->dir_d, 1);

    _set_duty(motor_handle->motor_a, speed);
    _set_duty(motor_handle->motor_b, speed);
    _set_duty(motor_handle->motor_c, speed);
    _set_duty(motor_handle->motor_d, speed);
    return ESP_OK;
}

esp_err_t app_motor_control_rotate_right(app_motor_handle_t motor_handle, int speed)
{
    if (!motor_handle) {
        return ESP_FAIL;
    }
    speed = speed < 0 ? 0 : speed;
    speed = speed > 100 ? 100 : speed;

    gpio_set_level(motor_handle->dir_a, 0);
    gpio_set_level(motor_handle->dir_b, 0);
    gpio_set_level(motor_handle->dir_c, 0);
    gpio_set_level(motor_handle->dir_d, 0);

    _set_duty(motor_handle->motor_a, speed);
    _set_duty(motor_handle->motor_b, speed);
    _set_duty(motor_handle->motor_c, speed);
    _set_duty(motor_handle->motor_d, speed);
    return ESP_OK;
}

esp_err_t app_motor_control_stop(app_motor_handle_t motor_handle)
{
    if (!motor_handle) {
        return ESP_FAIL;
    }
    gpio_set_level(motor_handle->dir_a, 0);
    gpio_set_level(motor_handle->dir_b, 0);
    gpio_set_level(motor_handle->dir_c, 0);
    gpio_set_level(motor_handle->dir_d, 0);

    _set_duty(motor_handle->motor_a, 0);
    _set_duty(motor_handle->motor_b, 0);
    _set_duty(motor_handle->motor_c, 0);
    _set_duty(motor_handle->motor_d, 0);
    return ESP_OK;
}

