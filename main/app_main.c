/* ESP HTTP Client Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include <urosNode.h>
#include "app.h"
#include "uros_lld_conn.h"
#include "app_motor.h"

static const char *TAG = "APP_MAIN";

void control_listener(void *pv)
{
    app_motor_handle_t motor_handle = pv;
    app_command_t required_command = STOP;
    while (1)
    {
        urosMutexLock(&control.lock);
        required_command = control.command;
        urosMutexUnlock(&control.lock);
        if (required_command == FORWARD)
        {
            app_motor_control_forward(motor_handle, 50);
        }
        else if (required_command == STOP)
        {
            app_motor_control_stop(motor_handle);
        }
        else if (required_command == BACKWARD)
        {
            app_motor_control_backward(motor_handle, 50);
        }
        else if (required_command == TURN_LEFT)
        {
            app_motor_control_turn_left(motor_handle, 50);
        }
        else if (required_command == TURN_RIGHT)
        {
            app_motor_control_turn_right(motor_handle, 50);
        }
        else if (required_command == ROTATE_LEFT)
        {
            app_motor_control_rotate_left(motor_handle, 50);
        }
        else if (required_command == ROTATE_RIGHT)
        {
            app_motor_control_rotate_right(motor_handle, 50);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    esp_netif_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    app_motor_handle_t motor_handle = app_motor_init();
    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    ESP_LOGI(TAG, "Connected to AP, begin http example");

    app_initialize();
    xTaskCreate(control_listener, "control_listener", 4096, motor_handle, 5, NULL);
    while (1)
    {
        ESP_LOGI(TAG, "free memory: %d", esp_get_free_heap_size());
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}
