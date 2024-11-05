/*#include "http.h"


void http_get_task(void *pvParameters) {
    char request[] = "GET / HTTP/1.1\r\nHost: " WEB_SERVER "\r\nConnection: close\r\n\r\n";
    char recv_buf[1024];

    // Konfiguracja adresu serwera
    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(WEB_PORT);

    struct hostent *he = gethostbyname(WEB_SERVER); // Rozwiązywanie nazwy hosta
    if (he == NULL) {
        ESP_LOGE(TAG, "Nie mozna przetworzyc adresu serwera");
        vTaskDelete(NULL);
        return;
    }
    dest_addr.sin_addr.s_addr = *(u_long *)he->h_addr;

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Nie mozna stworzyc socketu: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket stworzony, laczenie z %s:%d", WEB_SERVER, WEB_PORT);

    int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket nie moze sie polaczyc: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Polaczono z serwerem");

    int sent = send(sock, request, strlen(request), 0);
    if (sent < 0) {
        ESP_LOGE(TAG, "Blad przy wysylaniu zapytania: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    // Odbieranie odpowiedzi z serwera
    int len;
    do {
        len = recv(sock, recv_buf, sizeof(recv_buf) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "recv failed: errno %d", errno);
        } else if (len > 0) {
            recv_buf[len] = 0; // Dodaj zakończenie stringa
            printf("%s", recv_buf); // Wyświetl HTML na konsoli
        }
    } while (len > 0);

    ESP_LOGI(TAG, "Zamykanie socketu");
    shutdown(sock, 0);
    close(sock);
    vTaskDelete(NULL);
}*/