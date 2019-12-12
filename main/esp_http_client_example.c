/* ESP HTTP Client Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "esp_tls.h"

#include "lwip/api.h"
#include "lwip/ip.h"
#include "lwip/udp.h"
#include "lwip/prot/iana.h"

#include <urosNode.h>
#include "esp_http_client.h"
#include "app.h"
#include "uros_lld_conn.h"

#define MAX_HTTP_RECV_BUFFER 512
static const char *TAG = "HTTP_CLIENT";

#define PORT 40000

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // Write out data
                // printf("%.*s", evt->data_len, (char*)evt->data);
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error(evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
                ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            break;
    }
    return ESP_OK;
}

static void http_rest_with_url(void)
{
    esp_http_client_config_t config = {
        .url = "http://172.16.5.177:11311",
        .event_handler = _http_event_handler,
        // .port = 11311,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    char* buffer = malloc(1024);

    esp_err_t err;
    // POST
    const char *post_data = "<?xml version=\"1.0\"?> \
    <methodCall> \
        <methodName>registerPublisher</methodName> \
        <params> \
            <param> \
                <value>/ultrasonic_sensor</value> \
            </param> \
            <param> \
                <value>chatter</value> \
                <param> \
                    <value>std_msgs/Byte</value> \
                </param> \
                <param> \
                    <value>http://172.16.5.238:40000</value> \
                </param> \
            </param> \
        </params> \
    </methodCall>";
    esp_http_client_set_url(client, "http://172.16.5.177:11311");
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    esp_http_client_set_header(client, "User-Agent:", "curl/7.35.0");
    esp_http_client_set_header(client, "Content-Type:", "application/x-www-form-urlencoded");
    err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
        printf("\n==== Response ====\n");
        int len = esp_http_client_read(client, buffer, 1024);
        buffer[len] = '\0';
        printf("%s\n",buffer);
        printf("\n==== End ====\n");
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

static void http_test_task(void *pvParameters)
{
    http_rest_with_url();

    ESP_LOGI(TAG, "Finish http example");
    vTaskDelete(NULL);
}

static void do_retransmit(const int sock)
{
    int len;
    char rx_buffer[2048];

    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);

            // send() can return less bytes than supplied length.
            // Walk-around for robust implementation. 
            int to_write = len;
            // while (to_write > 0) {
            //     int written = send(sock, rx_buffer + (len - to_write), to_write, 0);
            //     if (written < 0) {
            //         ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            //     }
            //     to_write -= written;
            // }
        }
    } while (len > 0);
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family;
    int ip_protocol;


    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
        uint addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Convert ip address to string
        if (source_addr.sin6_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        } else if (source_addr.sin6_family == PF_INET6) {
            inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);
        do_retransmit(sock);
        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    esp_netif_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    ESP_LOGI(TAG, "Connected to AP, begin http example");

    // struct netconn *xNetConn = NULL;

    // struct ip_addr local_ip; 
    // struct ip_addr remote_ip; 
    // int rc1, rc2; 
    
    // xNetConn = netconn_new ( NETCONN_TCP ); 
    
    // if ( xNetConn == NULL ) { 
    
    // /* No memory for new connection? */
    //     ESP_LOGE(TAG, "errr");
    // }

    // local_ip.u_addr.ip4.addr = inet_addr("192.168.1.46");

    // rc1 = netconn_bind ( xNetConn, &local_ip, 0 ); 
    
    // remote_ip.u_addr.ip4.addr = inet_addr("192.168.1.82");
    // rc2 = netconn_connect ( xNetConn, &remote_ip, htons(11311) ); 
    
    // if ( rc1 != ERR_OK || rc2 != ERR_OK )
    // {
    //     ESP_LOGE(TAG, "connection fail %d", rc2);
    //     netconn_delete ( xNetConn );
    // } else {
    //     ESP_LOGI(TAG, "Connected");
    // }


    // struct sockaddr_in remaddr;

    // remaddr.sin_family = AF_INET;
    // remaddr.sin_port = htons(11311);
    // remaddr.sin_addr.s_addr = inet_addr("192.168.1.82");
    // // memset(remaddr.sin_zero, 0, sizeof(remaddr.sin_zero));
    // int sock =  socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    // int err = connect(sock, (struct sockaddr *)&remaddr, sizeof(remaddr));
    // if (err < 0) {
    //     ESP_LOGE(TAG, "Connect fail %d", err);
    // } else {
    //     ESP_LOGI(TAG, "Sock connect ok");
    // }
    // urosError(err == ETIMEDOUT, return UROS_ERR_NOCONN,
    //             ("Connection to "UROS_ADDRFMT" timed out\n",
    //             UROS_ADDRARG(remaddrp)));
    // urosError(err == ECONNREFUSED, return UROS_ERR_NOCONN,
    //             ("Connection to "UROS_ADDRFMT" refused\n",
    //             UROS_ADDRARG(remaddrp)));
    // urosError(err != 0, return UROS_ERR_NOCONN,
    //             ("Socket error [%s] while connecting to "UROS_ADDRFMT"\n",
    //             strerror(errno), UROS_ADDRARG(remaddrp)));
    // UrosConn conn;
    // uros_err_t err;
    // urosConnObjectInit(&conn);
    // err = urosConnCreate(&conn, UROS_PROTO_UDP);
    // err = urosConnConnect(&conn, addrp);
    // vTaskDelay(5000 / portTICK_PERIOD_MS);
    app_initialize();
    while (1) {
        ESP_LOGI(TAG, "free memory: %d", esp_get_free_heap_size());
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
    // xTaskCreate(&http_test_task, "http_test_task", 8192, NULL, 5, NULL);
    // xTaskCreate(tcp_server_task, "tcp_server_task", 1024 * 4, NULL, 5, NULL);
}
