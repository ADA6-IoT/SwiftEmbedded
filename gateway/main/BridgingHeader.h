#pragma once

// Standard C Libraries
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// ESP-IDF Core
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"

// HTTP Client
#include "esp_http_client.h"

// Console & UART
#include "argtable3/argtable3.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "linenoise/linenoise.h"

// JSON
#include "cJSON.h"

// Helper for WIFI_INIT_CONFIG_DEFAULT
static inline wifi_init_config_t get_wifi_init_config_default(void) {
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  return cfg;
}

void keep_me_alive(void);
void log_msg(const char *msg);
void log_int(const char *msg, int val);
