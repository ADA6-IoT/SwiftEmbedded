#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_http_client.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_sntp.h"
#include "cJSON.h"

// ===== 설정 상수 =====
#define AP_SSID "Gateway_Network"
#define AP_PASSWORD ""
#define AP_MAX_CONNECTIONS 10
#define STA_WIFI_SSID "S-Guest"
#define STA_WIFI_PASSWORD ""
#define NVS_NAMESPACE "gateway_cfg"
#define SERVER_URL "http://52.78.98.182:8080/api/locations/calculate"
#define FLOOR_BROADCAST_INTERVAL_MS 1000    // 층 브로드캐스트 간격 (1초)
#define MAX_HTTP_RETRY_COUNT 3              // HTTP 전송 최대 재시도 횟수
#define SNTP_SERVER "pool.ntp.org"
#define TIMEZONE "KST-9"                    // 한국 표준시 (UTC+9)

static const char *TAG = "GATEWAY";

// ESP-NOW 브로드캐스트 MAC 주소
static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ===== 전역 변수 =====
static char my_device_name[32] = {0};       // 게이트웨이 장치 이름
static int32_t my_floor_number = 0;         // 게이트웨이 층 번호
static QueueHandle_t data_recv_queue;       // 비콘 데이터 수신 큐
static EventGroupHandle_t wifi_event_group;
static const int STA_CONNECTED_BIT = BIT0;
static const int AP_STARTED_BIT = BIT1;
static bool config_loaded = false;

// ===== 데이터 구조 =====

// 비콘 데이터 패킷 구조체 (비콘과 동일해야 함)
typedef struct {
    char serial_number[10];                 // 비콘 시리얼 번호
    uint8_t battery_level;                  // 배터리 잔량 (%)
    int8_t floor;                           // 층 번호 (-99~99)
    char timestamp[128];                    // ISO 8601 형식: "2025-10-22T21:15:30.123Z"
    struct {
        uint8_t anchor_mac[6];              // 앵커(게이트웨이) MAC 주소
        float distance_meters;              // 거리 (미터)
        float variance;                     // 측정 분산 (칼만 필터용)
        int8_t rssi;                        // 신호 강도
        uint8_t sample_count;               // 사용된 유효 샘플 개수
        uint32_t rtt_nanoseconds;           // RTT (왕복 시간, 나노초)
    } measurements[3];                      // 앵커 측정값 (1~3개, 빈 슬롯은 MAC=0)
} beacon_data_packet_t;

// 칼만 필터 상태 구조체
typedef struct {
    float x;                                // 추정 거리 (상태)
    float P;                                // 추정 오차 공분산
    float Q;                                // 프로세스 노이즈 공분산
    float R;                                // 측정 노이즈 공분산
    uint32_t last_update_time;              // 마지막 업데이트 타임스탬프 (밀리초)
    bool initialized;                       // 초기화 플래그
} kalman_filter_state_t;

// 비콘-앵커 추적 엔트리
#define MAX_BEACONS 10
#define MAX_ANCHORS_PER_BEACON 6
#define BEACON_TIMEOUT_MS 60000             // 비콘 타임아웃 (1분)

typedef struct {
    char serial_number[10];                 // 비콘 시리얼 번호
    uint8_t anchor_mac[6];                  // 앵커 MAC 주소
    kalman_filter_state_t kf_state;         // 칼만 필터 상태
    uint32_t last_seen;                     // 마지막 수신 시간
} beacon_anchor_entry_t;

// 전역 상태 추적
static beacon_anchor_entry_t beacon_anchor_states[MAX_BEACONS * MAX_ANCHORS_PER_BEACON];
static int beacon_anchor_count = 0;

// ===== 함수 선언 =====
static esp_err_t load_config_from_nvs(void);
static esp_err_t save_config_to_nvs(const char *name, int32_t floor);
static void register_console_commands(void);
static void run_provisioning_console(void);
static void wifi_init_apsta(void);
static void initialize_sntp(void);
static void floor_broadcast_task(void *pvParameters);
static void data_relay_task(void *pvParameters);
static void beacon_data_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
static esp_err_t send_json_to_server(const char *json_data);

// 칼만 필터 함수
static void kalman_filter_init(kalman_filter_state_t *kf, float initial_value, float initial_variance);
static float kalman_filter_update(kalman_filter_state_t *kf, float measurement, float measurement_variance, float dt);
static beacon_anchor_entry_t* find_or_create_entry(const char *serial_number, const uint8_t *anchor_mac);
static void cleanup_old_entries(void);


// ===== 콘솔 명령 핸들러 =====

static struct {
    struct arg_str *name;
    struct arg_end *end;
} set_name_args;

static struct {
    struct arg_int *floor;
    struct arg_end *end;
} set_floor_args;

// 장치 이름 설정 명령 핸들러
static int set_name_handler(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&set_name_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_name_args.end, argv[0]);
        return 1;
    }

    const char *name = set_name_args.name->sval[0];
    if (strlen(name) >= 32) {
        printf("오류: 이름이 너무 깁니다 (최대 31자)\n");
        return 1;
    }

    strncpy(my_device_name, name, sizeof(my_device_name) - 1);
    printf("장치 이름 설정: %s\n", my_device_name);

    // 두 값이 모두 있으면 저장
    if (my_floor_number != 0) {
        if (save_config_to_nvs(my_device_name, my_floor_number) == ESP_OK) {
            printf("설정 저장 완료. 재부팅 중...\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
        }
    }

    return 0;
}

// 층 번호 설정 명령 핸들러
static int set_floor_handler(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&set_floor_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_floor_args.end, argv[0]);
        return 1;
    }

    int floor = set_floor_args.floor->ival[0];
    if (floor < -99 || floor > 99 || floor == 0) {
        printf("오류: 층 번호는 -99~99 사이여야 합니다 (0 제외)\n");
        return 1;
    }

    my_floor_number = floor;
    printf("층 번호 설정: %" PRId32 "\n", my_floor_number);

    // 두 값이 모두 있으면 저장
    if (strlen(my_device_name) > 0) {
        if (save_config_to_nvs(my_device_name, my_floor_number) == ESP_OK) {
            printf("설정 저장 완료. 재부팅 중...\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
        }
    }

    return 0;
}


// ===== NVS 설정 관리 =====

// NVS에서 설정 로드
static esp_err_t load_config_from_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS 네임스페이스를 찾을 수 없음");
        return err;
    }

    size_t name_len = sizeof(my_device_name);
    err = nvs_get_str(nvs_handle, "device_name", my_device_name, &name_len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS에서 장치 이름을 찾을 수 없음");
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_get_i32(nvs_handle, "floor", &my_floor_number);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS에서 층 번호를 찾을 수 없음");
        nvs_close(nvs_handle);
        return err;
    }

    nvs_close(nvs_handle);

    ESP_LOGI(TAG, "설정 로드 완료: 이름=%s, 층=%" PRId32, my_device_name, my_floor_number);
    config_loaded = true;
    return ESP_OK;
}

// NVS에 설정 저장
static esp_err_t save_config_to_nvs(const char *name, int32_t floor) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS 열기 실패");
        return err;
    }

    err = nvs_set_str(nvs_handle, "device_name", name);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "장치 이름 저장 실패");
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_set_i32(nvs_handle, "floor", floor);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "층 번호 저장 실패");
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS 커밋 실패");
    }

    nvs_close(nvs_handle);
    return err;
}


// ===== 콘솔 프로비저닝 =====

// 콘솔 명령 등록
static void register_console_commands(void) {
    // 이름 설정 명령
    set_name_args.name = arg_str1(NULL, NULL, "<name>", "장치 이름");
    set_name_args.end = arg_end(2);

    const esp_console_cmd_t set_name_cmd = {
        .command = "set_name",
        .help = "게이트웨이 장치 이름 설정",
        .hint = NULL,
        .func = &set_name_handler,
        .argtable = &set_name_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_name_cmd));

    // 층 설정 명령
    set_floor_args.floor = arg_int1(NULL, NULL, "<floor>", "층 번호");
    set_floor_args.end = arg_end(2);

    const esp_console_cmd_t set_floor_cmd = {
        .command = "set_floor",
        .help = "게이트웨이 층 번호 설정",
        .hint = NULL,
        .func = &set_floor_handler,
        .argtable = &set_floor_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_floor_cmd));
}

// 프로비저닝 콘솔 실행
static void run_provisioning_console(void) {
    ESP_LOGI(TAG, "프로비저닝 콘솔 시작");
    printf("\n===========================================\n");
    printf("게이트웨이 설정이 필요합니다\n");
    printf("===========================================\n");
    printf("게이트웨이를 설정하세요:\n");
    printf("1. set_name <장치이름>  (예: set_name GW_01)\n");
    printf("2. set_floor <층번호>   (예: set_floor 3)\n");
    printf("===========================================\n\n");

    // 라인 엔딩 설정
    uart_vfs_dev_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);
    uart_vfs_dev_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);

    // UART RX 버퍼 비우기
    esp_err_t uart_err = uart_flush_input(CONFIG_ESP_CONSOLE_UART_NUM);
    if (uart_err != ESP_OK) {
        ESP_LOGW(TAG, "UART flush 실패 (무시): %s", esp_err_to_name(uart_err));
    }

    // 버퍼링 비활성화
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    vTaskDelay(pdMS_TO_TICKS(200));

    // 콘솔 초기화
    esp_console_config_t console_config = {
        .max_cmdline_length = 256,
        .max_cmdline_args = 8,
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));

    // 명령 등록
    register_console_commands();

    // 입력 버퍼
    char line[256];
    int pos = 0;
    const char *prompt = "gateway> ";
    bool prompt_shown = false;

    while (!config_loaded) {
        // 프롬프트 출력 (라인 시작 시에만)
        if (!prompt_shown) {
            printf("%s", prompt);
            fflush(stdout);
            prompt_shown = true;
        }

        // 한 문자 읽기
        int c = fgetc(stdin);

        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // 백스페이스 처리
        if (c == '\b' || c == 127) {
            if (pos > 0) {
                pos--;
                printf("\b \b");
                fflush(stdout);
            }
            continue;
        }

        // 개행 문자 확인
        if (c == '\n' || c == '\r') {
            if (pos > 0) {
                line[pos] = '\0';
                printf("\n");

                // 명령 실행
                int ret;
                esp_err_t err = esp_console_run(line, &ret);
                if (err == ESP_ERR_NOT_FOUND) {
                    printf("알 수 없는 명령: %s\n", line);
                } else if (err == ESP_OK && ret != 0) {
                    printf("명령이 0이 아닌 오류 코드를 반환: %d\n", ret);
                } else if (err != ESP_OK) {
                    printf("명령 실행 오류: %s\n", esp_err_to_name(err));
                }

                pos = 0;
                prompt_shown = false;
            } else {
                printf("\n");
                prompt_shown = false;
            }
            continue;
        }

        // 제어 문자 무시
        if (c < 32 || c > 126) {
            continue;
        }

        // 일반 문자 추가
        if (pos < (int)sizeof(line) - 1) {
            line[pos++] = (char)c;
            printf("%c", c);
            fflush(stdout);
        }
    }

    esp_console_deinit();
}


// ===== WiFi 이벤트 핸들러 =====

// WiFi 이벤트 핸들러
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "AP 시작됨");
                xEventGroupSetBits(wifi_event_group, AP_STARTED_BIT);

                // AP 채널 확인 및 로깅
                uint8_t primary;
                wifi_second_chan_t second;
                esp_wifi_get_channel(&primary, &second);
                ESP_LOGI(TAG, "AP 동작 채널: %d", primary);
                break;

            case WIFI_EVENT_AP_STACONNECTED: {
                wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
                ESP_LOGI(TAG, "스테이션 "MACSTR" AP에 연결됨", MAC2STR(event->mac));
                break;
            }

            case WIFI_EVENT_AP_STADISCONNECTED: {
                wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
                ESP_LOGI(TAG, "스테이션 "MACSTR" AP에서 연결 해제됨", MAC2STR(event->mac));
                break;
            }

            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "STA 시작됨, AP에 연결 중...");
                esp_wifi_connect();
                break;

            case WIFI_EVENT_STA_CONNECTED: {
                wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;
                ESP_LOGI(TAG, "STA가 %s에 연결됨 (채널 %d)",
                        event->ssid, event->channel);

                // 연결 후 채널 로깅
                uint8_t primary;
                wifi_second_chan_t second;
                esp_wifi_get_channel(&primary, &second);
                ESP_LOGI(TAG, "WiFi 동작 채널 (AP 및 STA 모두): %d", primary);
                break;
            }

            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
                ESP_LOGW(TAG, "STA가 AP에서 연결 해제됨 (이유: %d), 재연결 중...", event->reason);
                xEventGroupClearBits(wifi_event_group, STA_CONNECTED_BIT);
                esp_wifi_connect();
                break;
            }
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            ESP_LOGI(TAG, "STA IP 획득: " IPSTR, IP2STR(&event->ip_info.ip));
            xEventGroupSetBits(wifi_event_group, STA_CONNECTED_BIT);

            // 최종 채널 설정 로깅
            uint8_t primary;
            wifi_second_chan_t second;
            esp_wifi_get_channel(&primary, &second);
            ESP_LOGI(TAG, "최종 채널 설정: %d", primary);
        }
    }
}


// ===== WiFi 초기화 =====

// APSTA 모드로 WiFi 초기화
static void wifi_init_apsta(void) {
    // 이벤트 그룹 생성
    wifi_event_group = xEventGroupCreate();

    // TCP/IP 스택 초기화
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // AP 및 STA 네트워크 인터페이스 생성
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    (void)esp_netif_create_default_wifi_sta();

    // AP 네트워크 인터페이스 설정
    esp_netif_ip_info_t ap_ip_info;
    IP4_ADDR(&ap_ip_info.ip, 192, 168, 4, 1);
    IP4_ADDR(&ap_ip_info.gw, 192, 168, 4, 1);
    IP4_ADDR(&ap_ip_info.netmask, 255, 255, 255, 0);

    esp_netif_dhcps_stop(ap_netif);
    esp_netif_set_ip_info(ap_netif, &ap_ip_info);
    esp_netif_dhcps_start(ap_netif);

    // 기본 설정으로 WiFi 초기화
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 이벤트 핸들러 등록
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                       ESP_EVENT_ANY_ID,
                                                       &wifi_event_handler,
                                                       NULL,
                                                       NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                       IP_EVENT_STA_GOT_IP,
                                                       &wifi_event_handler,
                                                       NULL,
                                                       NULL));

    // STA 설정 (채널 결정을 위해 먼저 설정)
    wifi_config_t sta_config = {
        .sta = {
            .ssid = STA_WIFI_SSID,
            .password = STA_WIFI_PASSWORD,
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .failure_retry_cnt = 5,
            .threshold.authmode = WIFI_AUTH_OPEN, //WIFI_AUTH_WPA2_PSK WIFI_AUTH_OPEN
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };

    // AP 설정
    wifi_config_t ap_config = {
        .ap = {
            .ssid = AP_SSID,
            .password = AP_PASSWORD,
            .ssid_len = strlen(AP_SSID),
            .channel = 0,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = AP_MAX_CONNECTIONS,
            .beacon_interval = 100,
            .pmf_cfg = {
                .required = false,
                .capable = true,
            },
            .ftm_responder = true,
        },
    };

    // 비밀번호 없으면 오픈 인증
    if (strlen(AP_PASSWORD) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    // WiFi 모드를 APSTA로 설정
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    // 설정 적용
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

    // WiFi 시작
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi APSTA 모드 초기화 완료");
    ESP_LOGI(TAG, "AP SSID: %s (FTM 응답기 활성화)", AP_SSID);
    ESP_LOGI(TAG, "STA 연결 대상: %s", STA_WIFI_SSID);

    // AP 시작 대기
    xEventGroupWaitBits(wifi_event_group, AP_STARTED_BIT, false, true, portMAX_DELAY);

    // WiFi 대역폭 설정 (20MHz)
    esp_err_t bw_err = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
    if (bw_err == ESP_OK) {
        ESP_LOGI(TAG, "AP 대역폭을 HT20 (20MHz)로 설정 (최적 FTM 정확도)");
    } else {
        ESP_LOGW(TAG, "AP 대역폭 설정 실패: %s", esp_err_to_name(bw_err));
    }

    // WiFi 프로토콜 설정 (802.11b/g/n)
    esp_err_t proto_err = esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    if (proto_err == ESP_OK) {
        ESP_LOGI(TAG, "AP 프로토콜을 802.11b/g/n으로 설정 (FTM은 802.11n 필요)");
    } else {
        ESP_LOGW(TAG, "AP 프로토콜 설정 실패: %s", esp_err_to_name(proto_err));
    }

    // AP MAC 주소 확인 및 로깅
    uint8_t ap_mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, ap_mac);

    // FTM 응답기 상태 로깅
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "FTM 응답기 상태:");
    ESP_LOGI(TAG, "  - FTM 응답기 플래그: 활성화");
    ESP_LOGI(TAG, "  - AP SSID: %s", AP_SSID);
    ESP_LOGI(TAG, "  - AP MAC (BSSID): "MACSTR, MAC2STR(ap_mac));
    ESP_LOGI(TAG, "  - 장치: ESP32-C6 (FTM 지원)");
    ESP_LOGI(TAG, "  - 대역폭: 20MHz (HT20)");
    ESP_LOGI(TAG, "  - 프로토콜: 802.11b/g/n");
    ESP_LOGI(TAG, "  - 비콘 간격: 100ms");
    ESP_LOGI(TAG, "======================================");

    ESP_LOGI(TAG, "게이트웨이 WiFi 초기화 완료");
}


// ===== SNTP 시간 동기화 =====

// SNTP 초기화
static void initialize_sntp(void) {
    ESP_LOGI(TAG, "SNTP 초기화 중");

    // 타임존 설정
    setenv("TZ", TIMEZONE, 1);
    tzset();

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, SNTP_SERVER);
    esp_sntp_init();

    // 시간 설정 대기
    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    const int retry_count = 15;

    while (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "시스템 시간 설정 대기 중... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    time(&now);
    localtime_r(&now, &timeinfo);

    if (timeinfo.tm_year > (2020 - 1900)) {
        ESP_LOGI(TAG, "시간 동기화 성공: %04d-%02d-%02d %02d:%02d:%02d",
                timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
        ESP_LOGW(TAG, "시간 동기화 실패, 기본값 사용");
    }
}


// ===== 층 브로드캐스트 태스크 =====

// 층 브로드캐스트 태스크 (1초마다 ESP-NOW로 전송)
static void floor_broadcast_task(void *pvParameters) {
    ESP_LOGI(TAG, "층 브로드캐스트 태스크 시작");

    int8_t floor_data = (int8_t)my_floor_number;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        // 충돌 방지 지터
        int jitter = esp_random() % 200 - 100;

        // ESP-NOW 브로드캐스트로 층 번호 전송
        esp_err_t result = esp_now_send(broadcast_mac, (const uint8_t*)&floor_data, sizeof(floor_data));

        if (result == ESP_OK) {
            ESP_LOGD(TAG, "층 브로드캐스트 전송: %d", floor_data);
        } else {
            ESP_LOGW(TAG, "층 브로드캐스트 실패: %s", esp_err_to_name(result));
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(FLOOR_BROADCAST_INTERVAL_MS + jitter));
    }
}


// ===== 칼만 필터 함수 =====

// 칼만 필터 초기화
static void kalman_filter_init(kalman_filter_state_t *kf, float initial_value, float initial_variance) {
    kf->x = initial_value;
    kf->P = initial_variance;
    kf->Q = 0.05;  // 프로세스 노이즈
    kf->R = 0.0;   // 측정 분산
    kf->last_update_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    kf->initialized = true;
}

// 칼만 필터 업데이트
static float kalman_filter_update(kalman_filter_state_t *kf, float measurement, float measurement_variance, float dt) {
    if (!kf->initialized) {
        ESP_LOGE(TAG, "칼만 필터가 초기화되지 않음");
        return measurement;
    }

    // 예측 단계
    float x_pred = kf->x;
    float P_pred = kf->P + kf->Q * dt;

    // 업데이트 단계
    kf->R = measurement_variance;

    // 칼만 이득
    float K = P_pred / (P_pred + kf->R);

    // 추정값 업데이트
    kf->x = x_pred + K * (measurement - x_pred);
    kf->P = (1.0f - K) * P_pred;

    // 타임스탬프 업데이트
    kf->last_update_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    ESP_LOGI(TAG, "칼만 업데이트: 측정=%.2f, 분산=%.4f, 예측=%.2f, 이득=%.3f, 추정=%.2f, P=%.4f",
            measurement, measurement_variance, x_pred, K, kf->x, kf->P);

    return kf->x;
}

// 기존 엔트리 찾기 또는 새로 생성
static beacon_anchor_entry_t* find_or_create_entry(const char *serial_number, const uint8_t *anchor_mac) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // 먼저 기존 엔트리 찾기
    for (int i = 0; i < beacon_anchor_count; i++) {
        if (strcmp(beacon_anchor_states[i].serial_number, serial_number) == 0 &&
            memcmp(beacon_anchor_states[i].anchor_mac, anchor_mac, 6) == 0) {
            beacon_anchor_states[i].last_seen = current_time;
            return &beacon_anchor_states[i];
        }
    }

    // 없으면 공간이 있을 때 새 엔트리 생성
    if (beacon_anchor_count < MAX_BEACONS * MAX_ANCHORS_PER_BEACON) {
        beacon_anchor_entry_t *entry = &beacon_anchor_states[beacon_anchor_count];
        strncpy(entry->serial_number, serial_number, sizeof(entry->serial_number) - 1);
        memcpy(entry->anchor_mac, anchor_mac, 6);
        entry->last_seen = current_time;
        entry->kf_state.initialized = false;
        beacon_anchor_count++;

        ESP_LOGI(TAG, "새 엔트리 생성: %s - "MACSTR" (총 %d개)",
                serial_number, MAC2STR(anchor_mac), beacon_anchor_count);
        return entry;
    }

    // 공간이 없으면 오래된 엔트리 정리 후 재시도
    ESP_LOGW(TAG, "새 엔트리 공간 없음, 오래된 엔트리 정리 중");
    cleanup_old_entries();

    // 정리 후 재시도
    if (beacon_anchor_count < MAX_BEACONS * MAX_ANCHORS_PER_BEACON) {
        beacon_anchor_entry_t *entry = &beacon_anchor_states[beacon_anchor_count];
        strncpy(entry->serial_number, serial_number, sizeof(entry->serial_number) - 1);
        memcpy(entry->anchor_mac, anchor_mac, 6);
        entry->last_seen = current_time;
        entry->kf_state.initialized = false;
        beacon_anchor_count++;
        return entry;
    }

    ESP_LOGE(TAG, "엔트리 생성 실패, 배열 가득 참");
    return NULL;
}

// 오래된 엔트리 정리 (BEACON_TIMEOUT_MS 이상)
static void cleanup_old_entries(void) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    int write_idx = 0;
    int removed = 0;

    for (int read_idx = 0; read_idx < beacon_anchor_count; read_idx++) {
        if (current_time - beacon_anchor_states[read_idx].last_seen < BEACON_TIMEOUT_MS) {
            if (write_idx != read_idx) {
                beacon_anchor_states[write_idx] = beacon_anchor_states[read_idx];
            }
            write_idx++;
        } else {
            removed++;
            ESP_LOGI(TAG, "오래된 엔트리 제거: %s - "MACSTR,
                    beacon_anchor_states[read_idx].serial_number,
                    MAC2STR(beacon_anchor_states[read_idx].anchor_mac));
        }
    }

    beacon_anchor_count = write_idx;
    if (removed > 0) {
        ESP_LOGI(TAG, "정리 완료: %d개 제거, %d개 남음", removed, beacon_anchor_count);
    }
}


// ===== HTTP 서버 전송 =====

// JSON 데이터를 서버로 전송
static esp_err_t send_json_to_server(const char *json_data) {
    esp_http_client_config_t config = {
        .url = SERVER_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000,
        .buffer_size = 2048,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "HTTP 클라이언트 초기화 실패");
        return ESP_FAIL;
    }

    // 헤더 설정
    esp_http_client_set_header(client, "Content-Type", "application/json");

    // POST 데이터 설정
    esp_http_client_set_post_field(client, json_data, strlen(json_data));

    // HTTP 요청 수행 (재시도 포함)
    esp_err_t err = ESP_FAIL;
    for (int retry = 0; retry < MAX_HTTP_RETRY_COUNT; retry++) {
        err = esp_http_client_perform(client);

        if (err == ESP_OK) {
            int status_code = esp_http_client_get_status_code(client);
            if (status_code == 200 || status_code == 201) {
                ESP_LOGI(TAG, "HTTP POST 성공, 상태: %d", status_code);
                break;
            } else {
                ESP_LOGW(TAG, "HTTP POST 상태 코드 반환: %d", status_code);
                err = ESP_FAIL;
            }
        } else {
            ESP_LOGW(TAG, "HTTP POST 실패 (시도 %d/%d): %s",
                    retry + 1, MAX_HTTP_RETRY_COUNT, esp_err_to_name(err));
        }

        if (retry < MAX_HTTP_RETRY_COUNT - 1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    esp_http_client_cleanup(client);
    return err;
}


// ===== 데이터 중계 태스크 =====

// 데이터 중계 태스크
static void data_relay_task(void *pvParameters) {
    ESP_LOGI(TAG, "데이터 중계 태스크 시작");
    beacon_data_packet_t packet;

    // STA 연결 대기
    xEventGroupWaitBits(wifi_event_group, STA_CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "STA 연결됨, 시간 동기화 초기화 중");

    // SNTP 초기화
    initialize_sntp();

    ESP_LOGI(TAG, "데이터 중계 준비 완료");

    while (1) {
        // 큐에서 비콘 데이터 대기
        if (xQueueReceive(data_recv_queue, &packet, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "비콘 데이터 처리 중: %s", packet.serial_number);

            // 타임스탬프 업데이트 (ISO 8601 UTC with milliseconds)
            struct timeval tv;
            gettimeofday(&tv, NULL);
            struct tm timeinfo;
            gmtime_r(&tv.tv_sec, &timeinfo);

            // 밀리초 계산
            int milliseconds = tv.tv_usec / 1000;

            // 포맷: YYYY-MM-DDTHH:MM:SS.sssZ
            snprintf(packet.timestamp, sizeof(packet.timestamp),
                    "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
                    timeinfo.tm_year + 1900,
                    timeinfo.tm_mon + 1,
                    timeinfo.tm_mday,
                    timeinfo.tm_hour,
                    timeinfo.tm_min,
                    timeinfo.tm_sec,
                    milliseconds);
            ESP_LOGI(TAG, "타임스탬프 업데이트: %s (UTC)", packet.timestamp);

            // JSON 객체 생성
            cJSON *root = cJSON_CreateObject();
            if (root == NULL) {
                ESP_LOGE(TAG, "JSON 객체 생성 실패");
                continue;
            }

            // 비콘 데이터 추가 (순서: battery_level, floor, measurements, serial_number, timestamp)
            cJSON_AddNumberToObject(root, "battery_level", packet.battery_level);
            cJSON_AddNumberToObject(root, "floor", packet.floor);

            // 측정값 배열 추가 (칼만 필터링)
            cJSON *measurements = cJSON_CreateArray();
            for (int i = 0; i < 3; i++) {
                // 측정값이 유효한지 확인 (0이 아닌 MAC)
                bool valid = false;
                for (int j = 0; j < 6; j++) {
                    if (packet.measurements[i].anchor_mac[j] != 0) {
                        valid = true;
                        break;
                    }
                }

                if (valid) {
                    cJSON *measurement = cJSON_CreateObject();

                    // MAC 주소 포맷
                    char mac_str[18];
                    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                            packet.measurements[i].anchor_mac[0],
                            packet.measurements[i].anchor_mac[1],
                            packet.measurements[i].anchor_mac[2],
                            packet.measurements[i].anchor_mac[3],
                            packet.measurements[i].anchor_mac[4],
                            packet.measurements[i].anchor_mac[5]);

                    // 칼만 필터 적용
                    beacon_anchor_entry_t *entry = find_or_create_entry(
                        packet.serial_number,
                        packet.measurements[i].anchor_mac
                    );

                    float filtered_distance = packet.measurements[i].distance_meters;

                    if (entry != NULL) {
                        // 칼만 필터 초기화
                        if (!entry->kf_state.initialized) {
                            kalman_filter_init(&entry->kf_state,
                                             packet.measurements[i].distance_meters,
                                             packet.measurements[i].variance);
                            filtered_distance = entry->kf_state.x;
                            ESP_LOGI(TAG, "%s - "MACSTR" 칼만 필터 초기화: 거리=%.2f, 분산=%.4f",
                                    packet.serial_number, MAC2STR(packet.measurements[i].anchor_mac),
                                    packet.measurements[i].distance_meters,
                                    packet.measurements[i].variance);
                        } else {
                            // 시간 간격 계산
                            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                            float dt = (current_time - entry->kf_state.last_update_time) / 1000.0f;  // 초 단위

                            // 칼만 필터 업데이트
                            filtered_distance = kalman_filter_update(
                                &entry->kf_state,
                                packet.measurements[i].distance_meters,
                                packet.measurements[i].variance,
                                dt
                            );

                            ESP_LOGI(TAG, "%s - "MACSTR" 칼만 필터 업데이트: 원본=%.2f -> 필터=%.2f (dt=%.2fs)",
                                    packet.serial_number, MAC2STR(packet.measurements[i].anchor_mac),
                                    packet.measurements[i].distance_meters, filtered_distance, dt);
                        }
                    } else {
                        ESP_LOGW(TAG, "칼만 필터 엔트리 획득 실패, 원본 거리 사용");
                    }

                    // 칼만 필터링된 거리 사용
                    cJSON_AddStringToObject(measurement, "anchor_mac", mac_str);
                    cJSON_AddNumberToObject(measurement, "distance_meters", filtered_distance);
                    cJSON_AddNumberToObject(measurement, "rssi", packet.measurements[i].rssi);
                    cJSON_AddNumberToObject(measurement, "rtt_nanoseconds", packet.measurements[i].rtt_nanoseconds);

                    ESP_LOGI(TAG, "측정값 추가: %s 거리=%.2f (원본=%.2f) rssi=%d RTT=%"PRIu32" ns",
                            mac_str, filtered_distance, packet.measurements[i].distance_meters,
                            packet.measurements[i].rssi, packet.measurements[i].rtt_nanoseconds);

                    cJSON_AddItemToArray(measurements, measurement);
                }
            }
            cJSON_AddItemToObject(root, "measurements", measurements);

            // serial_number와 timestamp 추가 (순서 유지)
            cJSON_AddStringToObject(root, "serial_number", packet.serial_number);
            cJSON_AddStringToObject(root, "timestamp", packet.timestamp);

            // 문자열로 변환
            char *json_string = cJSON_PrintUnformatted(root);
            if (json_string) {
                ESP_LOGI(TAG, "JSON 데이터: %s", json_string);

                // 서버로 전송
                if (send_json_to_server(json_string) == ESP_OK) {
                    ESP_LOGI(TAG, "데이터 서버 전송 성공");
                } else {
                    ESP_LOGE(TAG, "데이터 서버 전송 실패");
                }

                free(json_string);
            }

            cJSON_Delete(root);
        }
    }
}


// ===== ESP-NOW 수신 콜백 =====

// 비콘 데이터 수신 콜백
static void beacon_data_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len == sizeof(beacon_data_packet_t)) {
        ESP_LOGI(TAG, "비콘 데이터 수신: "MACSTR, MAC2STR(recv_info->src_addr));

        // 큐에 전송
        beacon_data_packet_t packet;
        memcpy(&packet, data, sizeof(beacon_data_packet_t));

        if (xQueueSend(data_recv_queue, &packet, 0) != pdTRUE) {
            ESP_LOGW(TAG, "비콘 데이터 큐 전송 실패");
        }
    } else if (len == 1) {
        // 다른 게이트웨이의 층 브로드캐스트
        ESP_LOGD(TAG, "다른 게이트웨이로부터 층 브로드캐스트 수신");
    } else {
        ESP_LOGW(TAG, "알 수 없는 ESP-NOW 데이터 수신 (길이 %d)", len);
    }
}


// ===== 메인 애플리케이션 =====

void app_main(void) {
    ESP_LOGI(TAG, "게이트웨이 디바이스 시작 (v11 - 칼만 필터 활성화)");

    // NVS 초기화
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // NVS에서 설정 로드
    if (load_config_from_nvs() != ESP_OK) {
        // 설정을 찾을 수 없으면 프로비저닝 콘솔 실행
        run_provisioning_console();
        // 여기 도달하면 설정이 저장되고 시스템이 재시작됨
    }

    ESP_LOGI(TAG, "설정 로드 성공");
    ESP_LOGI(TAG, "장치 이름: %s", my_device_name);
    ESP_LOGI(TAG, "층 번호: %" PRId32, my_floor_number);

    // APSTA 모드로 WiFi 초기화
    wifi_init_apsta();

    // ESP-NOW 초기화
    ESP_ERROR_CHECK(esp_now_init());

    // ESP-NOW 수신 콜백 등록
    ESP_ERROR_CHECK(esp_now_register_recv_cb(beacon_data_recv_cb));

    // 층 브로드캐스팅용 브로드캐스트 피어 추가
    esp_now_peer_info_t broadcast_peer = {
        .peer_addr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
        .channel = 0,
        .encrypt = false,
    };
    ESP_ERROR_CHECK(esp_now_add_peer(&broadcast_peer));

    // 비콘 데이터용 큐 생성
    data_recv_queue = xQueueCreate(10, sizeof(beacon_data_packet_t));
    if (data_recv_queue == NULL) {
        ESP_LOGE(TAG, "데이터 수신 큐 생성 실패");
        return;
    }

    // 층 브로드캐스트 태스크 생성
    xTaskCreate(floor_broadcast_task, "floor_broadcast", 4096, NULL, 5, NULL);

    // 데이터 중계 태스크 생성
    xTaskCreate(data_relay_task, "data_relay", 8192, NULL, 10, NULL);

    ESP_LOGI(TAG, "게이트웨이 운영 중 - AP: %s, 층: %" PRId32, AP_SSID, my_floor_number);
    ESP_LOGI(TAG, "비콘 데이터 대기 중...");
}
