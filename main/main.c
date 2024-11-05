#include <stdio.h>
#include <string.h>
#include "esp_wifi_types_generic.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
//#include "wifi.h"
//#include "led.h"
//#include "http.h"

#define WIFI_SSID      "Bartek J"
#define WIFI_PASS      "IoT2425jk"
#define WIFI_MAX_RETRY 100

#define LED_PIN GPIO_NUM_2
#define WEB_SERVER "example.com"
#define WEB_PORT 80

#define AP_SSID      "ESP32_AccessPoint"  
#define AP_PASSWORD  "0987654321"         
#define MAX_STA_CONN 4                   

static const char *TAG = "wifi_station";
static int retry_count = 0;
static bool wifi_connected = false;

static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// Event handler for WiFi events
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
	// Check if the event is related to WiFi
    if (event_base == WIFI_EVENT) {
		// Handle the event when the WiFi station starts
        if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect(); // Initiate connection to the WiFi network
        // Handle the event when the WiFi station disconnects
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            wifi_connected = false; // Update the connection status
            retry_count++;
            ESP_LOGI(TAG, "Proba polaczenia %d.", retry_count);
            // Attempt to reconnect 
            if (retry_count < WIFI_MAX_RETRY) {
                esp_wifi_connect();
            } else {
                xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT); // Clear the connection bit if max retries reached
                retry_count = 0; // Reset the retry count
            }
        }
        // Check if the event is related to IP acquisition
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data; // Cast event data to the correct type
        ESP_LOGI(TAG, "Polaczono, adres IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_count = 0;
        wifi_connected = true; // Update the connection status to true
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT); // Set the connected bit to indicate successful connection
    }
}

// Initialize WiFi in station mode
void wifi_init_sta_ap() {
    wifi_event_group = xEventGroupCreate(); // Create an event group to manage WiFi events
    ESP_ERROR_CHECK(esp_netif_init()); // Initialize the network interface
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // Create the default event loop
    esp_netif_create_default_wifi_sta(); // Create the default WiFi station interface
    esp_netif_create_default_wifi_ap(); // Create the default AP interface
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // Initialize WiFi configuration with default values
    ESP_ERROR_CHECK(esp_wifi_init(&cfg)); // Initialize the WiFi driver

    esp_event_handler_instance_t instance_any_id; // Declare an event handler instance
    // Register the event handler for all WiFi events
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id);
    // Register the event handler for IP acquisition events
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_any_id);

    // Configure the WiFi connection settings
    wifi_config_t wifi_config_sta = {
        .sta = {
            .ssid = WIFI_SSID, // Set the SSID of the WiFi network
            .password = WIFI_PASS // Set the password for the WiFi network
        },
    };
    
    wifi_config_t wifi_config_ap = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .password = AP_PASSWORD,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA)); // Set the AP+WiFi mode to station mode
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config_sta)); // Apply the WiFi configuration
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config_ap)); // Apply the AP configuration
    ESP_ERROR_CHECK(esp_wifi_start()); // Start the WiFi driver
    
    ESP_LOGI(TAG, "ESP32 uruchomiono w trybie Access Point. SSID:%s, Haslo:%s", AP_SSID, AP_PASSWORD);

    ESP_LOGI(TAG, "Inicjalizacja WiFi zakonczona.");
}

void led_blink_task(void *pvParameters) {
	//set LED_PIN as output
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    while (1) {
        if (!wifi_connected) {
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            gpio_set_level(LED_PIN, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        } else {
            gpio_set_level(LED_PIN, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

void http_get_task(void *pvParameters) {
    // Define the HTTP GET request
    char request[] = "GET / HTTP/1.1\r\nHost: " WEB_SERVER "\r\nConnection: close\r\n\r\n";
    char recv_buf[1024]; // Buffer to store the response from the server

    // Configure the server address
    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET; // Set the address family to IPv4
    dest_addr.sin_port = htons(WEB_PORT); // Set the server port (convert to network byte order)

    struct hostent *he = gethostbyname(WEB_SERVER); // Resolve the host name to an IP address
    if (he == NULL) {
        ESP_LOGE(TAG, "Cannot resolve server address"); // Log error if resolution fails
        vTaskDelete(NULL); // Terminate the task
        return;
    }
    dest_addr.sin_addr.s_addr = *(u_long *)he->h_addr; // Set the server IP address

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP); // Create a TCP socket
    if (sock < 0) {
        ESP_LOGE(TAG, "Cannot create socket: errno %d", errno); // Log error if socket creation fails
        vTaskDelete(NULL); // Terminate the task
        return;
    }
    ESP_LOGI(TAG, "Socket created, connecting to %s:%d", WEB_SERVER, WEB_PORT); // Log the connection attempt

    int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)); // Connect to the server
    if (err != 0) {
        ESP_LOGE(TAG, "Socket connection failed: errno %d", errno); // Log error if connection fails
        close(sock); // Close the socket
        vTaskDelete(NULL); // Terminate the task
        return;
    }
    ESP_LOGI(TAG, "Connected to the server"); // Log successful connection

    // Send the HTTP GET request to the server
    int sent = send(sock, request, strlen(request), 0);
    if (sent < 0) {
        ESP_LOGE(TAG, "Error sending request: errno %d", errno); // Log error if sending fails
        close(sock); // Close the socket
        vTaskDelete(NULL); // Terminate the task
        return;
    }

    // Receive the server's response
    int len;
    do {
        len = recv(sock, recv_buf, sizeof(recv_buf) - 1, 0); // Receive data into the buffer
        if (len < 0) {
            ESP_LOGE(TAG, "recv failed: errno %d", errno); // Log error if receiving fails
        } else if (len > 0) {
            recv_buf[len] = 0; // Null-terminate the buffer for printing
            printf("%s", recv_buf); // Print the received HTML content to the console
        }
    } while (len > 0); // Continue until all data is received

    ESP_LOGI(TAG, "Closing socket"); // Log socket shutdown
    shutdown(sock, 0); // Shut down the socket
    close(sock); // Close the socket
    vTaskDelete(NULL); // Terminate the task
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init(); // Initialize the flash memory (NVS)
    
    // Check if the flash memory is initialized correctly
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) { 
        // Erase flash memory if necessary
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret); // Check for any errors during initialization

    // Initialize WiFi in station mode by calling wifi_init_sta in station.c
    wifi_init_sta_ap();
    // Create a task for led_blink_task to manage LED blinking, stack size: 2048, priority: 5
    xTaskCreate(&led_blink_task, "led_blink_task", 2048, NULL, 5, NULL);

    // Wait for the WIFI_CONNECTED_BIT to be set in the wifi_event_group
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        // Create the http_get_task to retrieve data from an HTTP server, defined in get_http_data.c
        xTaskCreate(&http_get_task, "http_get_task", 4096, NULL, 5, NULL);
    }
}

