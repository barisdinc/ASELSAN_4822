#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include <string.h>

#define WIFI_SSID "TA6-NET24"
#define WIFI_PASS "ankara12"

// Basit HTML Yanıtı
const char *http_response = "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n"
                            "<html><body><h1>Aselsan Pico Radio</h1>"
                            "<p>Durum: Calisiyor</p></body></html>";

static err_t http_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (!p) {
        tcp_close(tpcb);
        return ERR_OK;
    }
    // Gelen veriyi (GET isteği) al ama şimdilik sadece sabit yanıt dön
    tcp_write(tpcb, http_response, strlen(http_response), TCP_WRITE_FLAG_COPY);
    pbuf_free(p);
    return ERR_OK;
}

static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_recv(newpcb, http_callback);
    return ERR_OK;
}

void start_web_server() {
    if (cyw43_arch_init()) {
        printf("WiFi Modülü Hatası\n");
        return;
    }
    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("WiFi Bağlantı Hatası\n");
        return;
    }
    printf("IP Adresi: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));

    struct tcp_pcb *pcb = tcp_new();
    tcp_bind(pcb, IP_ADDR_ANY, 80);
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, connection_callback);
}
