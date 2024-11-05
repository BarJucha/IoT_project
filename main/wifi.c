/*#include "wifi.h"
#include "esp_wifi.h"
#include "esp_log.h"

EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;
bool wifi_connected = false;

static const char *TAG = "wifi_station";
static int retry_count = 0;

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
            ESP_LOGI(TAG, "Proba polaczenia %d. Miganie LED", retry_count);
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
void wifi_init_sta() {
    wifi_event_group = xEventGroupCreate(); // Create an event group to manage WiFi events
    ESP_ERROR_CHECK(esp_netif_init()); // Initialize the network interface
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // Create the default event loop
    esp_netif_create_default_wifi_sta(); // Create the default WiFi station interface
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // Initialize WiFi configuration with default values
    ESP_ERROR_CHECK(esp_wifi_init(&cfg)); // Initialize the WiFi driver

    esp_event_handler_instance_t instance_any_id; // Declare an event handler instance
    // Register the event handler for all WiFi events
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id);
    // Register the event handler for IP acquisition events
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_any_id);

    // Configure the WiFi connection settings
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID, // Set the SSID of the WiFi network
            .password = WIFI_PASS // Set the password for the WiFi network
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA)); // Set the WiFi mode to station mode
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config)); // Apply the WiFi configuration
    ESP_ERROR_CHECK(esp_wifi_start()); // Start the WiFi driver

    ESP_LOGI(TAG, "Inicjalizacja WiFi zakonczona.");
}*/