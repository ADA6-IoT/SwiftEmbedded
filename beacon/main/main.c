#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include <inttypes.h>
#include <math.h>

// ===== 설정 상수 =====
#define WIFI_SSID "Gateway_Network"
#define WIFI_PASSWORD ""
#define FTM_RSSI_THRESHOLD -85              // FTM 측정을 위한 최소 신호 강도
#define MAX_FTM_CANDIDATES 6                // FTM 측정 최대 후보 AP 개수
#define MAX_RETRY_ATTEMPTS 3                // 데이터 전송 최대 재시도 횟수
#define FLOOR_DISCOVERY_DURATION_MS 1000    // 층 정보 수집 시간 (ms)
#define SLEEP_DURATION_SEC 5                // Deep Sleep 지속 시간 (초)

// ===== FTM 최적화 파라미터 =====
#define FTM_FRAME_COUNT 24                  // FTM 프레임 개수 (24프레임 = 6버스트 x 4)
#define FTM_BURST_PERIOD 2                  // 버스트 간격 (200ms)
#define MAX_FTM_RETRY 2                     // 분산이 높을 경우 재시도 횟수
#define MIN_VALID_SAMPLES 6                 // 필요한 최소 유효 샘플 개수

// ===== FTM 보정 파라미터 =====
// 실제 측정 결과 기반:
// - 실제 0.5m → 측정값 ~3m (비율: 6배)
// - 실제 1.5m → 측정값 ~6m (비율: 4배)
// 평균 보정 계수: 0.2 (1/5)
// 하드웨어 및 환경에 따라 조정 필요
#define FTM_CALIBRATION_FACTOR 0.20f        // 시스템 오차 보정 스케일 계수

// 보정 후 분산 임계값 조정
// 원래 임계값: 2.0 (보정 전)
// 보정 후 (0.2배): 분산은 계수^2 = 0.04배로 스케일링
// 새 임계값: 2.0 * 0.04 = 0.08, 여유를 위해 0.10 사용
#define MAX_VARIANCE_THRESHOLD 0.10f        // 보정 후 최대 허용 분산 (m²)

static const char *TAG = "BEACON";
static const char* serial_number = "S-03";
static int battery_level = 91;

// ===== 데이터 구조 =====
// 비콘 데이터 패킷 구조체 (게이트웨이와 동일해야 함)
typedef struct {
    char serial_number[10];                 // 비콘 시리얼 번호
    uint8_t battery_level;                  // 배터리 잔량 (%)
    uint8_t floor;                          // 층 번호
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

// AP 레코드 구조체
typedef struct {
    uint8_t mac[6];                         // AP MAC 주소
    int8_t rssi;                            // 신호 강도
    uint8_t channel;                        // 채널 번호
} ap_record_t;

// 게이트웨이 정보 구조체 (채널 순회용)
typedef struct {
    uint8_t mac[6];                         // 게이트웨이 MAC 주소
    uint8_t channel;                        // 채널 번호
    int8_t rssi;                            // 신호 강도
} gateway_info_t;

// 층 정보 구조체
typedef struct {
    uint8_t gateway_mac[6];                 // 게이트웨이 MAC 주소
    uint8_t floor;                          // 층 번호
    int8_t rssi;                            // 신호 강도
    uint8_t channel;                        // 채널 번호 (ESP-NOW 전송용)
} floor_info_t;

// ===== 전역 변수 =====
static bool upload_successful = false;
static floor_info_t floor_list[20];         // 발견된 게이트웨이 목록
static int floor_count = 0;
static EventGroupHandle_t ftm_event_group;
static const int FTM_REPORT_BIT = BIT0;
static const int FTM_FAILURE_BIT = BIT1;
static uint8_t ftm_report_num_entries = 0;
static wifi_ftm_report_entry_t *ftm_report_data = NULL;

// ===== 함수 선언 =====
static void floor_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
static void data_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
static uint8_t calculate_floor_mode(void);
static esp_err_t send_data_with_retry(beacon_data_packet_t *packet);
static esp_err_t perform_ftm_measurement(uint8_t *bssid, uint8_t channel, float *distance, float *variance, int *valid_count, uint32_t *rtt_ns);
static int compare_floats(const void *a, const void *b);
static float calculate_median(float *data, int count);
static void remove_outliers_iqr(float *data, int *count);


// ===== 통계 유틸리티 함수 =====

static int compare_floats(const void *a, const void *b) {
    float fa = *(const float*)a;
    float fb = *(const float*)b;
    return (fa > fb) - (fa < fb);
}

// 중앙값 계산
static float calculate_median(float *data, int count) {
    if (count == 0) return 0.0f;

    // 복사 후 정렬
    float *sorted = malloc(count * sizeof(float));
    memcpy(sorted, data, count * sizeof(float));
    qsort(sorted, count, sizeof(float), compare_floats);

    float median;
    if (count % 2 == 0) {
        median = (sorted[count/2 - 1] + sorted[count/2]) / 2.0f;
    } else {
        median = sorted[count/2];
    }

    free(sorted);
    return median;
}

// IQR 방법으로 이상치 제거
static void remove_outliers_iqr(float *data, int *count) {
    if (*count < 4) return;  // IQR 계산에는 최소 4개 샘플 필요

    // 정렬된 복사본 생성
    float *sorted = malloc(*count * sizeof(float));
    memcpy(sorted, data, *count * sizeof(float));
    qsort(sorted, *count, sizeof(float), compare_floats);

    // Q1, Q3 계산
    int q1_idx = (*count) / 4;
    int q3_idx = (3 * (*count)) / 4;
    float q1 = sorted[q1_idx];
    float q3 = sorted[q3_idx];
    float iqr = q3 - q1;

    // 범위 계산
    float lower_bound = q1 - 1.5f * iqr;
    float upper_bound = q3 + 1.5f * iqr;

    ESP_LOGI(TAG, "IQR 필터: Q1=%.2f, Q3=%.2f, IQR=%.2f, 범위=[%.2f, %.2f]",
            q1, q3, iqr, lower_bound, upper_bound);

    free(sorted);

    // 원본 배열에서 이상치 제거
    int new_count = 0;
    for (int i = 0; i < *count; i++) {
        if (data[i] >= lower_bound && data[i] <= upper_bound) {
            data[new_count++] = data[i];
        } else {
            ESP_LOGW(TAG, "이상치 제거: %.2f m", data[i]);
        }
    }

    *count = new_count;
    ESP_LOGI(TAG, "이상치 제거 후 샘플 개수: %d", new_count);
}


// ===== ESP-NOW 콜백 함수 =====

// 층 브로드캐스트 수신 콜백
static void floor_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len == 1 && floor_count < 20) {
        // 현재 WiFi 채널 가져오기
        uint8_t primary_channel = 0;
        wifi_second_chan_t second_channel = WIFI_SECOND_CHAN_NONE;
        esp_wifi_get_channel(&primary_channel, &second_channel);

        // 게이트웨이 MAC, 층, RSSI, 채널 저장
        memcpy(floor_list[floor_count].gateway_mac, recv_info->src_addr, 6);
        floor_list[floor_count].floor = data[0];
        floor_list[floor_count].rssi = recv_info->rx_ctrl->rssi;
        floor_list[floor_count].channel = primary_channel;
        floor_count++;

        ESP_LOGI(TAG, "층 정보 수신: %d층 from "MACSTR" (채널 %d, RSSI: %d)",
                data[0], MAC2STR(recv_info->src_addr), primary_channel, recv_info->rx_ctrl->rssi);
    }
}

// 데이터 전송 콜백
static void data_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    upload_successful = (status == ESP_NOW_SEND_SUCCESS);
    ESP_LOGI(TAG, "전송 상태 to "MACSTR": %s",
            MAC2STR(mac_addr), upload_successful ? "성공" : "실패");
}


// ===== 층 계산 함수 =====

// 층 최빈값 계산
static uint8_t calculate_floor_mode(void) {
    if (floor_count == 0) return 0;

    // 각 층의 출현 횟수 계산
    uint8_t floor_counts[10] = {0};  // 최대 10층 가정
    for (int i = 0; i < floor_count; i++) {
        if (floor_list[i].floor < 10) {
            floor_counts[floor_list[i].floor]++;
        }
    }

    // 최빈값 찾기
    uint8_t mode_floor = 0;
    uint8_t max_count = 0;
    for (int i = 0; i < 10; i++) {
        if (floor_counts[i] > max_count) {
            max_count = floor_counts[i];
            mode_floor = i;
        }
    }

    ESP_LOGI(TAG, "층 최빈값 계산: %d층 (출현 횟수: %d)", mode_floor, max_count);
    return mode_floor;
}


// ===== FTM 이벤트 핸들러 =====

// FTM 리포트 이벤트 핸들러
static void ftm_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data) {
    if (event_id == WIFI_EVENT_FTM_REPORT) {
        wifi_event_ftm_report_t *event = (wifi_event_ftm_report_t *)event_data;
        ESP_LOGI(TAG, "FTM 리포트 수신");

        // 엔트리 개수 저장
        ftm_report_num_entries = event->ftm_report_num_entries;
        ESP_LOGI(TAG, "FTM 상태: %d, 엔트리 개수: %d", event->status, ftm_report_num_entries);

        // 이전 리포트 메모리 해제
        if (ftm_report_data != NULL) {
            free(ftm_report_data);
            ftm_report_data = NULL;
        }

        // 리포트 엔트리를 즉시 복사 (유효한 동안)
        if (ftm_report_num_entries > 0 && event->ftm_report_data != NULL) {
            ftm_report_data = malloc(sizeof(wifi_ftm_report_entry_t) * ftm_report_num_entries);
            if (ftm_report_data != NULL) {
                memcpy(ftm_report_data, event->ftm_report_data,
                       sizeof(wifi_ftm_report_entry_t) * ftm_report_num_entries);
                ESP_LOGI(TAG, "%d개 FTM 엔트리 복사 완료", ftm_report_num_entries);
            } else {
                ESP_LOGE(TAG, "FTM 리포트 메모리 할당 실패");
                ftm_report_num_entries = 0;
            }
        }

        // 이벤트 비트 설정
        if (event->status == FTM_STATUS_SUCCESS && ftm_report_num_entries > 0) {
            xEventGroupSetBits(ftm_event_group, FTM_REPORT_BIT);
        } else {
            ESP_LOGW(TAG, "FTM 실패: 상태=%d 또는 엔트리 없음", event->status);
            xEventGroupSetBits(ftm_event_group, FTM_FAILURE_BIT);
        }
    }
}


// ===== FTM 측정 함수 =====

// 특정 AP와 FTM 거리 측정 수행
static esp_err_t perform_ftm_measurement(uint8_t *bssid, uint8_t channel,
                                        float *distance, float *variance, int *valid_count, uint32_t *rtt_ns) {
    ESP_LOGI(TAG, "FTM 측정 시작: "MACSTR" (채널 %d)", MAC2STR(bssid), channel);

    esp_err_t final_result = ESP_FAIL;
    float best_distance = 0;
    float best_variance = 999999.0f;
    int best_valid_count = 0;
    uint32_t best_rtt_ns = 0;

    // 좋은 측정값을 얻기 위해 최대 MAX_FTM_RETRY번 시도
    for (int attempt = 0; attempt < MAX_FTM_RETRY; attempt++) {
        ESP_LOGI(TAG, "FTM 시도 %d/%d", attempt + 1, MAX_FTM_RETRY);

        // FTM 이벤트 핸들러 등록
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                                                  WIFI_EVENT_FTM_REPORT,
                                                  &ftm_event_handler,
                                                  NULL));

        // FTM 파라미터 설정 (실내 측위 최적화)
        wifi_ftm_initiator_cfg_t ftm_cfg = {
            .resp_mac = {0},
            .channel = channel,
            .frm_count = FTM_FRAME_COUNT,
            .burst_period = FTM_BURST_PERIOD,
        };
        memcpy(ftm_cfg.resp_mac, bssid, 6);

        ESP_LOGI(TAG, "FTM 설정: 프레임=%d, 버스트주기=%d, 채널=%d",
                 ftm_cfg.frm_count, ftm_cfg.burst_period, ftm_cfg.channel);

        // 이벤트 비트 및 리포트 개수 초기화
        xEventGroupClearBits(ftm_event_group, FTM_REPORT_BIT | FTM_FAILURE_BIT);
        ftm_report_num_entries = 0;

        // 이전 리포트 데이터 해제
        if (ftm_report_data != NULL) {
            free(ftm_report_data);
            ftm_report_data = NULL;
        }

        // FTM 세션 시작
        esp_err_t err = esp_wifi_ftm_initiate_session(&ftm_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "FTM 세션 시작 실패: %s", esp_err_to_name(err));
            esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_FTM_REPORT, &ftm_event_handler);

            // RSSI 기반 추정값으로 폴백
            ESP_LOGW(TAG, "FTM 미지원, RSSI 추정값 사용");
            int8_t rssi = -70;
            float path_loss_exp = 2.0;
            float ref_rssi = -40.0;  // 1미터 거리에서의 RSSI
            *distance = pow(10.0, (ref_rssi - rssi) / (10.0 * path_loss_exp));
            *variance = 10.0;  // RSSI 기반 추정은 분산이 높음
            return ESP_OK;
        }

        // FTM 리포트 대기
        EventBits_t bits = xEventGroupWaitBits(
            ftm_event_group,
            FTM_REPORT_BIT | FTM_FAILURE_BIT,
            pdTRUE,
            pdFALSE,
            pdMS_TO_TICKS(6000)  // 타임아웃: 6초
        );

        esp_err_t result = ESP_FAIL;
        float attempt_distance = 0;
        float attempt_variance = 999999.0f;
        int attempt_valid_count = 0;

        if (bits & FTM_REPORT_BIT) {
            // 엔트리 및 데이터 확인
            if (ftm_report_num_entries == 0 || ftm_report_data == NULL) {
                ESP_LOGW(TAG, "FTM 리포트 엔트리 또는 데이터 없음");
                result = ESP_FAIL;
            } else {
                // 유효한 거리 측정값 수집
                float *distances = malloc(ftm_report_num_entries * sizeof(float));
                int valid_count = 0;

                for (int i = 0; i < ftm_report_num_entries; i++) {
                    // RTT 유효성 확인 (피코초 단위: 1000-333000ps = 0.15-50m)
                    if (ftm_report_data[i].rtt == 0 || ftm_report_data[i].rtt == UINT32_MAX ||
                        ftm_report_data[i].rtt < 1000 || ftm_report_data[i].rtt > 333000) {
                        ESP_LOGW(TAG, "FTM 엔트리 %d: 유효하지 않은 RTT %" PRIu32 "ps (범위: 1000-333000ps = 0.15-50m)",
                                i, ftm_report_data[i].rtt);
                        continue;
                    }

                    // 거리 계산 (RTT * 광속 / 2)
                    float dist_raw = (ftm_report_data[i].rtt * 1e-12 * 299792458.0) / 2.0;

                    // 시스템 오차 보정 계수 적용
                    float dist_calibrated = dist_raw * FTM_CALIBRATION_FACTOR;

                    // 보정 후 거리 범위 확인 (실내: 0.15m ~ 50m)
                    if (dist_calibrated >= 0.15 && dist_calibrated <= 50.0) {
                        distances[valid_count++] = dist_calibrated;
                        ESP_LOGI(TAG, "FTM 샘플 %d: RTT=%" PRIu32 "ps (%.2fns), 원본=%.2f m, 보정=%.2f m - 유효",
                                i, ftm_report_data[i].rtt, ftm_report_data[i].rtt / 1000.0, dist_raw, dist_calibrated);
                    } else {
                        ESP_LOGW(TAG, "FTM 엔트리 %d: 거리 %.2f m (원본: %.2f m) 범위 밖 (0.15-50m)", i, dist_calibrated, dist_raw);
                    }
                }

                ESP_LOGI(TAG, "이상치 제거 전 유효 샘플: %d개", valid_count);

                // IQR 이상치 제거 적용
                if (valid_count >= MIN_VALID_SAMPLES) {
                    remove_outliers_iqr(distances, &valid_count);
                }

                attempt_valid_count = valid_count;
                if (valid_count > 0) {
                    // 중앙값 사용
                    attempt_distance = calculate_median(distances, valid_count);

                    // 분산 계산
                    float sum_sq = 0;
                    for (int i = 0; i < valid_count; i++) {
                        float diff = distances[i] - attempt_distance;
                        sum_sq += diff * diff;
                    }
                    attempt_variance = sum_sq / valid_count;

                    ESP_LOGI(TAG, "FTM 시도 결과: 거리=%.2f m (중앙값), 분산=%.4f (%d개 샘플)",
                            attempt_distance, attempt_variance, valid_count);
                    result = ESP_OK;
                } else {
                    ESP_LOGW(TAG, "필터링 후 유효한 FTM 측정값 없음");
                }

                free(distances);
            }
        } else {
            ESP_LOGW(TAG, "FTM 타임아웃 (시도 %d)", attempt + 1);
        }

        // FTM 세션 종료
        esp_wifi_ftm_end_session();

        // 이벤트 핸들러 등록 해제
        esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_FTM_REPORT, &ftm_event_handler);

        // 가장 좋은 결과인지 확인
        if (result == ESP_OK && attempt_variance < best_variance) {
            best_distance = attempt_distance;
            best_variance = attempt_variance;
            best_valid_count = attempt_valid_count;
            // RTT 계산 (거리에서 역산): distance = (RTT_ns * 0.299792458) / 2
            best_rtt_ns = (uint32_t)((best_distance / FTM_CALIBRATION_FACTOR) * 2.0 / 0.299792458);
            final_result = ESP_OK;

            ESP_LOGI(TAG, "최선의 결과 갱신: 거리=%.2f m, RTT=%"PRIu32" ns, 분산=%.4f, 샘플=%d개",
                    best_distance, best_rtt_ns, best_variance, best_valid_count);

            // 분산이 충분히 낮으면 재시도 중단
            if (best_variance < MAX_VARIANCE_THRESHOLD) {
                ESP_LOGI(TAG, "분산 허용 범위 (%.4f < %.4f), 재시도 중단",
                        best_variance, MAX_VARIANCE_THRESHOLD);
                break;
            }
        }

        // 재시도 전 짧은 대기
        if (attempt < MAX_FTM_RETRY - 1) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    // 최선의 결과 반환
    if (final_result == ESP_OK) {
        *distance = best_distance;
        *variance = best_variance;
        *valid_count = best_valid_count;
        if (rtt_ns != NULL) {
            *rtt_ns = best_rtt_ns;
        }
        ESP_LOGI(TAG, "최종 FTM 결과: 거리=%.2f m, RTT=%"PRIu32" ns, 분산=%.4f, 샘플=%d개",
                *distance, best_rtt_ns, *variance, *valid_count);
    } else {
        ESP_LOGE(TAG, "모든 FTM 시도 실패");
    }

    return final_result;
}




// ===== 데이터 전송 함수 =====

// 재시도 로직을 포함한 데이터 전송 (RSSI 순)
static esp_err_t send_data_with_retry(beacon_data_packet_t *packet) {
    // RSSI로 게이트웨이 정렬 (재시도 순서)
    for (int i = 0; i < floor_count - 1; i++) {
        for (int j = i + 1; j < floor_count; j++) {
            if (floor_list[j].rssi > floor_list[i].rssi) {
                floor_info_t temp = floor_list[i];
                floor_list[i] = floor_list[j];
                floor_list[j] = temp;
            }
        }
    }

    // 상위 2개 게이트웨이에 전송 시도
    int max_gateways = (floor_count < 2) ? floor_count : 2;

    for (int gw = 0; gw < max_gateways; gw++) {
        ESP_LOGI(TAG, "게이트웨이 %d에 전송 시도: "MACSTR" (채널 %d, RSSI: %d)",
                gw+1, MAC2STR(floor_list[gw].gateway_mac), floor_list[gw].channel, floor_list[gw].rssi);

        // 게이트웨이 채널로 변경
        ESP_LOGI(TAG, "채널 %d로 변경", floor_list[gw].channel);
        esp_wifi_set_channel(floor_list[gw].channel, WIFI_SECOND_CHAN_NONE);
        vTaskDelay(pdMS_TO_TICKS(100));  // 채널 변경 안정화 대기

        // 피어가 추가되지 않았으면 추가
        esp_now_peer_info_t peer_info = {0};
        memcpy(peer_info.peer_addr, floor_list[gw].gateway_mac, 6);
        peer_info.channel = floor_list[gw].channel;  // 올바른 채널 설정
        peer_info.encrypt = false;

        if (!esp_now_is_peer_exist(floor_list[gw].gateway_mac)) {
            esp_err_t add_result = esp_now_add_peer(&peer_info);
            if (add_result != ESP_OK) {
                ESP_LOGE(TAG, "피어 추가 실패: %s", esp_err_to_name(add_result));
                continue;
            }
            ESP_LOGI(TAG, "피어 추가 성공");
        }

        // 재시도하며 전송
        for (int retry = 0; retry < MAX_RETRY_ATTEMPTS; retry++) {
            upload_successful = false;

            esp_err_t result = esp_now_send(
                floor_list[gw].gateway_mac,
                (uint8_t*)packet,
                sizeof(beacon_data_packet_t)
            );

            if (result == ESP_OK) {
                // 전송 콜백 대기
                vTaskDelay(pdMS_TO_TICKS(100));

                if (upload_successful) {
                    ESP_LOGI(TAG, "게이트웨이 %d에 데이터 전송 성공", gw+1);
                    return ESP_OK;
                }
            }

            ESP_LOGW(TAG, "전송 시도 %d/%d 실패", retry+1, MAX_RETRY_ATTEMPTS);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    ESP_LOGE(TAG, "모든 게이트웨이에 데이터 전송 실패");
    return ESP_FAIL;
}


// ===== 메인 애플리케이션 =====

void app_main(void) {
    ESP_LOGI(TAG, "비콘 디바이스 시작 (v11 - 칼만 필터 지원)");

    // NVS 초기화
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Wi-Fi 초기화 (스캔/FTM 전용 STA 모드)
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 대역폭을 20MHz (HT20)로 설정 (최적의 FTM 정확도)
    esp_err_t bw_err = esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
    if (bw_err == ESP_OK) {
        ESP_LOGI(TAG, "STA 대역폭을 HT20 (20MHz)로 설정 (최적 FTM 정확도)");
    } else {
        ESP_LOGW(TAG, "STA 대역폭 설정 실패: %s", esp_err_to_name(bw_err));
    }

    // WiFi 프로토콜을 802.11n으로 설정 (최적의 FTM 지원)
    esp_err_t proto_err = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    if (proto_err == ESP_OK) {
        ESP_LOGI(TAG, "STA 프로토콜을 802.11b/g/n으로 설정 (FTM은 802.11n 필요)");
    } else {
        ESP_LOGW(TAG, "STA 프로토콜 설정 실패: %s", esp_err_to_name(proto_err));
    }

    // FTM 이벤트 그룹 생성
    ftm_event_group = xEventGroupCreate();

    // 메인 작업 (Deep Sleep 전 1회 실행)
    ESP_LOGI(TAG, "=== 메인 측정 사이클 시작 ===");

    // 1단계: 모든 게이트웨이 AP 스캔 및 정보 수집
    ESP_LOGI(TAG, "1단계: 게이트웨이 AP 스캔하여 모든 채널 정보 수집");

    wifi_scan_config_t scan_config = {
        .ssid = NULL,  // 모든 SSID 스캔하여 게이트웨이 찾기
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time = {
            .active = {
                .min = 100,
                .max = 300,
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));

    uint16_t ap_count = 0;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));

    // 게이트웨이 리스트 동적 할당
    gateway_info_t *gateway_list = NULL;
    int gateway_count = 0;
    int *unique_channel_list = NULL;
    int unique_channel_count = 0;

    if (ap_count > 0) {
        wifi_ap_record_t *ap_records = malloc(ap_count * sizeof(wifi_ap_record_t));
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_records));

        // 최대 게이트웨이 개수만큼 메모리 할당
        gateway_list = malloc(ap_count * sizeof(gateway_info_t));
        unique_channel_list = malloc(ap_count * sizeof(int));

        if (gateway_list == NULL || unique_channel_list == NULL) {
            ESP_LOGE(TAG, "메모리 할당 실패");
            free(ap_records);
            if (gateway_list) free(gateway_list);
            if (unique_channel_list) free(unique_channel_list);
            esp_deep_sleep(SLEEP_DURATION_SEC * 1000000);
            return;
        }

        // 모든 게이트웨이 AP 수집 (break 제거)
        for (int i = 0; i < ap_count; i++) {
            // 게이트웨이 AP 확인
            if (strcmp((char*)ap_records[i].ssid, "Gateway_Network") == 0 ||
                strcmp((char*)ap_records[i].ssid, WIFI_SSID) == 0) {

                // 게이트웨이 정보 저장
                memcpy(gateway_list[gateway_count].mac, ap_records[i].bssid, 6);
                gateway_list[gateway_count].channel = ap_records[i].primary;
                gateway_list[gateway_count].rssi = ap_records[i].rssi;

                ESP_LOGI(TAG, "게이트웨이 %d: "MACSTR" (채널 %d, RSSI: %d)",
                        gateway_count + 1, MAC2STR(ap_records[i].bssid),
                        ap_records[i].primary, ap_records[i].rssi);

                // unique_channel_list에 채널 추가 (중복 제거)
                bool channel_exists = false;
                for (int j = 0; j < unique_channel_count; j++) {
                    if (unique_channel_list[j] == ap_records[i].primary) {
                        channel_exists = true;
                        break;
                    }
                }
                if (!channel_exists) {
                    unique_channel_list[unique_channel_count] = ap_records[i].primary;
                    unique_channel_count++;
                    ESP_LOGI(TAG, "새 채널 추가: %d (총 %d개 채널)",
                            ap_records[i].primary, unique_channel_count);
                }

                gateway_count++;
            }
        }

        free(ap_records);
    }

    if (gateway_count == 0) {
        ESP_LOGW(TAG, "게이트웨이를 찾을 수 없음, Deep Sleep 진입");
        if (gateway_list) free(gateway_list);
        if (unique_channel_list) free(unique_channel_list);
        esp_deep_sleep(SLEEP_DURATION_SEC * 1000000);
        return;
    }

    ESP_LOGI(TAG, "스캔 완료: %d개 게이트웨이, %d개 채널 발견",
            gateway_count, unique_channel_count);

    // ESP-NOW 초기화 (채널 순회 전 1회)
    ESP_LOGI(TAG, "ESP-NOW 초기화");
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(data_send_cb));
    vTaskDelay(pdMS_TO_TICKS(100));

    // FTM 결과를 저장할 구조체
    typedef struct {
        uint8_t mac[6];
        float distance;
        float variance;
        int8_t rssi;
        int sample_count;
        uint32_t rtt_nanoseconds;
    } ftm_result_t;

    // 최종 FTM 결과 리스트 동적 할당
    ftm_result_t *final_ftm_results = malloc(gateway_count * sizeof(ftm_result_t));
    int final_ftm_count = 0;

    if (final_ftm_results == NULL) {
        ESP_LOGE(TAG, "FTM 결과 메모리 할당 실패");
        free(gateway_list);
        free(unique_channel_list);
        esp_deep_sleep(SLEEP_DURATION_SEC * 1000000);
        return;
    }

    // 2~5단계: 채널 순회 메인 루프
    ESP_LOGI(TAG, "=== 채널 순회 시작 (%d개 채널) ===", unique_channel_count);
    floor_count = 0;  // 전역 floor_count 초기화

    for (int ch_idx = 0; ch_idx < unique_channel_count; ch_idx++) {
        int current_channel = unique_channel_list[ch_idx];
        ESP_LOGI(TAG, "\n--- 채널 %d 처리 중 (%d/%d) ---",
                current_channel, ch_idx + 1, unique_channel_count);

        // 채널 고정
        ESP_LOGI(TAG, "채널 %d로 변경", current_channel);
        ESP_ERROR_CHECK(esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE));

        // 안정화 대기
        vTaskDelay(pdMS_TO_TICKS(200));

        // 층 발견 (ESP-NOW) - 모든 채널에서 누적
        ESP_LOGI(TAG, "채널 %d에서 층 발견 시작", current_channel);
        ESP_ERROR_CHECK(esp_now_register_recv_cb(floor_recv_cb));
        vTaskDelay(pdMS_TO_TICKS(FLOOR_DISCOVERY_DURATION_MS));
        ESP_ERROR_CHECK(esp_now_unregister_recv_cb());
        ESP_LOGI(TAG, "층 발견 완료: 현재까지 총 %d개 게이트웨이", floor_count);

        // 현재 채널의 게이트웨이에 대해 FTM 측정
        ESP_LOGI(TAG, "채널 %d의 게이트웨이 FTM 측정 시작", current_channel);

        for (int gw_idx = 0; gw_idx < gateway_count; gw_idx++) {
            // 현재 채널에 속한 게이트웨이만 측정
            if (gateway_list[gw_idx].channel != current_channel) {
                continue;
            }

            ESP_LOGI(TAG, "FTM 측정 중: "MACSTR" (채널 %d, RSSI: %d)",
                    MAC2STR(gateway_list[gw_idx].mac),
                    gateway_list[gw_idx].channel,
                    gateway_list[gw_idx].rssi);

            float distance, variance;
            int valid_samples = 0;

            // 안정화를 위한 짧은 대기
            vTaskDelay(pdMS_TO_TICKS(50));

            uint32_t rtt_ns = 0;
            if (perform_ftm_measurement(gateway_list[gw_idx].mac,
                                       gateway_list[gw_idx].channel,
                                       &distance, &variance, &valid_samples, &rtt_ns) == ESP_OK) {
                // 성공한 측정값을 final_ftm_results에 누적
                memcpy(final_ftm_results[final_ftm_count].mac, gateway_list[gw_idx].mac, 6);
                final_ftm_results[final_ftm_count].distance = distance;
                final_ftm_results[final_ftm_count].variance = variance;
                final_ftm_results[final_ftm_count].rssi = gateway_list[gw_idx].rssi;
                final_ftm_results[final_ftm_count].sample_count = valid_samples;
                final_ftm_results[final_ftm_count].rtt_nanoseconds = rtt_ns;
                final_ftm_count++;

                ESP_LOGI(TAG, "FTM 성공 [%d]: 거리=%.2f m, 분산=%.4f, 샘플=%d개",
                        final_ftm_count, distance, variance, valid_samples);
            } else {
                ESP_LOGW(TAG, "FTM 실패: "MACSTR, MAC2STR(gateway_list[gw_idx].mac));
            }
        }

        ESP_LOGI(TAG, "채널 %d 처리 완료 (현재까지 FTM 성공: %d개)",
                current_channel, final_ftm_count);
    }

    ESP_LOGI(TAG, "=== 채널 순회 완료 ===");
    ESP_LOGI(TAG, "총 FTM 성공: %d개, 층 정보: %d개", final_ftm_count, floor_count);

    // 메모리 해제
    free(gateway_list);
    free(unique_channel_list);

    // 데이터 취합 및 필터링
    beacon_data_packet_t packet = {0};

    // 최소 1개 이상의 FTM 측정값이 있어야 전송
    if (final_ftm_count < 1) {
        ESP_LOGW(TAG, "FTM 측정값 없음 (%d < 1), Deep Sleep 진입", final_ftm_count);
        free(final_ftm_results);
        esp_deep_sleep(SLEEP_DURATION_SEC * 1000000);
        return;
    }

    ESP_LOGI(TAG, "FTM 측정 완료: %d개 앵커 데이터 수집 (최소 1개 이상 충족)", final_ftm_count);

    // 분산 기준으로 정렬 (버블 정렬)
    ESP_LOGI(TAG, "분산 기준으로 결과 정렬");
    for (int i = 0; i < final_ftm_count - 1; i++) {
        for (int j = i + 1; j < final_ftm_count; j++) {
            if (final_ftm_results[j].variance < final_ftm_results[i].variance) {
                ftm_result_t temp = final_ftm_results[i];
                final_ftm_results[i] = final_ftm_results[j];
                final_ftm_results[j] = temp;
            }
        }
    }

    // 측정 결과를 패킷에 저장 (최대 3개, 최소 1개)
    int result_count = (final_ftm_count < 3) ? final_ftm_count : 3;
    for (int i = 0; i < result_count; i++) {
        memcpy(packet.measurements[i].anchor_mac, final_ftm_results[i].mac, 6);
        packet.measurements[i].distance_meters = final_ftm_results[i].distance;
        packet.measurements[i].variance = final_ftm_results[i].variance;
        packet.measurements[i].rssi = final_ftm_results[i].rssi;
        packet.measurements[i].sample_count = (uint8_t)final_ftm_results[i].sample_count;
        packet.measurements[i].rtt_nanoseconds = final_ftm_results[i].rtt_nanoseconds;

        ESP_LOGI(TAG, "최종 측정 %d: "MACSTR" 거리=%.2f m, 분산=%.4f, RTT=%"PRIu32" ns, rssi=%d, 샘플=%d개",
                i+1, MAC2STR(final_ftm_results[i].mac),
                final_ftm_results[i].distance, final_ftm_results[i].variance,
                final_ftm_results[i].rtt_nanoseconds,
                final_ftm_results[i].rssi, final_ftm_results[i].sample_count);
    }

    // FTM 결과 메모리 해제
    free(final_ftm_results);

    // 6단계: 층 계산
    ESP_LOGI(TAG, "6단계: %d개 게이트웨이 리포트에서 층 계산", floor_count);
    uint8_t my_floor = calculate_floor_mode();

    // 7단계: 패킷 생성
    ESP_LOGI(TAG, "7단계: 데이터 패킷 생성");
    strncpy(packet.serial_number, serial_number, sizeof(packet.serial_number) - 1);
    packet.battery_level = battery_level;
    packet.floor = my_floor;
    // 타임스탬프는 게이트웨이 수신 시 채워짐
    strcpy(packet.timestamp, "");

    ESP_LOGI(TAG, "패킷 준비 완료: SN=%s, 배터리=%d%%, 층=%d",
            packet.serial_number, packet.battery_level, packet.floor);

    // 8단계: 데이터 전송
    ESP_LOGI(TAG, "8단계: 게이트웨이로 데이터 전송");
    esp_err_t send_result = send_data_with_retry(&packet);

    if (send_result == ESP_OK) {
        ESP_LOGI(TAG, "✓ 데이터 전송 성공");
    } else {
        ESP_LOGE(TAG, "✗ 데이터 전송 실패");
    }

    // 9단계: Deep Sleep 진입
    ESP_LOGI(TAG, "9단계: %d초 동안 Deep Sleep 진입", SLEEP_DURATION_SEC);
    esp_deep_sleep(SLEEP_DURATION_SEC * 1000000);
}
