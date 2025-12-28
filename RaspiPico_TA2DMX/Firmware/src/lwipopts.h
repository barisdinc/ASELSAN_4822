#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// --- Temel Ayarlar ---
// İşletim sistemi (RTOS) kullanmıyoruz (NO_SYS = 1)
#define NO_SYS                      1
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0
#define LWIP_IGMP                   0
#define LWIP_ICMP                   1

// --- Bellek Ayarları ---
#define MEM_LIBC_MALLOC             0
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    4000
#define MEMP_NUM_TCP_SEG            32
#define MEMP_NUM_ARP_QUEUE          10
#define PBUF_POOL_SIZE              24

// --- TCP Ayarları ---
#define LWIP_TCP                    1
#define TCP_MSS                     1460
#define TCP_WND                     (8 * TCP_MSS)
#define TCP_SND_BUF                 (8 * TCP_MSS)
#define TCP_SND_QUEUELEN            16

// --- UDP Ayarları ---
#define LWIP_UDP                    1

// --- DHCP Ayarları ---
#define LWIP_DHCP                   1
#define LWIP_DNS                    1

// --- Link Ayarları ---
#define LWIP_ETHERNET               1
#define ETH_PAD_SIZE                0
#define LWIP_RAW                    1

// --- Pico'ya Özgü Ayarlar ---
// Rastgele sayı üretimi vs. için gerekli
#define LWIP_RAND() ((u32_t)rand())

#endif