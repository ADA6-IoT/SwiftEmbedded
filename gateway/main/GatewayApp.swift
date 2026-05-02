
//
//  GatewayApp.swift
//  Gateway Firmware
//
//  Created by SwiftEmbedded on 2025.
//

// #include "BridgingHeader.h"

// C 매크로 대체 (전역 함수)
func ESP_ERROR_CHECK(_ err: esp_err_t) {
    if err != ESP_OK {
        log_int("ESP_ERROR_CHECK failed", Int32(err))
        while true {}
    }
}

// ===== 설정 상수 =====
struct GatewayConfig {
    static let apSSID = "Gateway_Network"
    static let apPassword = ""
    static let apMaxConnections: Int32 = 10
    static let staWifiSSID = "S-Guest"
    static let staWifiPassword = ""
    static let nvsNamespace = "gateway_cfg"
    // static let serverURL = "http://52.78.98.182:8080/api/locations/calculate" // String 제거
    static let serverURLBytes: [UInt8] = [0x68, 0x74, 0x74, 0x70, 0x3A, 0x2F, 0x2F, 0x35, 0x32, 0x2E, 0x37, 0x38, 0x2E, 0x39, 0x38, 0x2E, 0x31, 0x38, 0x32, 0x3A, 0x38, 0x30, 0x38, 0x30, 0x2F, 0x61, 0x70, 0x69, 0x2F, 0x6C, 0x6F, 0x63, 0x61, 0x74, 0x69, 0x6F, 0x6E, 0x73, 0x2F, 0x63, 0x61, 0x6C, 0x63, 0x75, 0x6C, 0x61, 0x74, 0x65, 0x00] // Null-terminated
    static let floorBroadcastIntervalMs: UInt32 = 1000
    static let maxHttpRetryCount = 3
    static let sntpServer = "pool.ntp.org"
    static let timezone = "KST-9"
    static let tag = "GATEWAY"
}

// ===== 데이터 구조 =====

struct BeaconDataPacket {
    var serialNumber: [UInt8] // String -> [UInt8]
    var batteryLevel: UInt8
    var floor: Int8
    var timestamp: [UInt8] // String -> [UInt8]
    var measurements: [AnchorMeasurement]
    
    struct AnchorMeasurement {
        var anchorMac: [UInt8]
        var distanceMeters: Float
        var variance: Float
        var rssi: Int8
        var sampleCount: UInt8
        var rttNanoseconds: UInt32
    }
    
    // 바이트 배열에서 파싱 (수신용)
    static func fromBytes(_ data: UnsafePointer<UInt8>, length: Int) -> BeaconDataPacket? {
        // C 구조체 레이아웃에 맞춰 파싱 필요
        // 여기서는 간소화된 예시로 구현 (실제로는 바이트 오프셋 계산 필요)
        // Swift의 UnsafePointer를 이용한 직접 매핑은 구조체 패딩 문제로 주의 필요
        // 안전하게는 바이트 단위로 읽어야 함.
        
        // [10 bytes serial] [1 byte battery] [1 byte floor] [128 bytes timestamp] [measurements...]
        if length < 140 { return nil } // 최소 길이 체크
        
        var offset = 0
        
        // Serial
        // Data 제거 -> 포인터 연산
        var serialNumber = [UInt8]()
        for i in 0..<10 {
            let byte = data[offset + i]
            if byte == 0 { break }
            serialNumber.append(byte)
        }
        serialNumber.append(0)
        offset += 10
        
        let batteryLevel = data[offset]
        offset += 1
        
        let floor = Int8(bitPattern: data[offset])
        offset += 1
        
        // Timestamp
        // Data 제거 -> 포인터 연산
        var timestamp = [UInt8]()
        for i in 0..<128 {
            let byte = data[offset + i]
            if byte == 0 { break }
            timestamp.append(byte)
        }
        timestamp.append(0)
        offset += 128
        
        var measurements = [AnchorMeasurement]()
        
        // 3 measurements
        for _ in 0..<3 {
            if offset + 20 > length { break }
            
            let mac = Array(UnsafeBufferPointer(start: data + offset, count: 6))
            offset += 6
            
            // MAC이 0이면 빈 슬롯
            if mac == [0,0,0,0,0,0] {
                offset += 14 // 나머지 스킵
                continue
            }
            
            let dist = (data + offset).withMemoryRebound(to: Float.self, capacity: 1) { $0.pointee }
            offset += 4
            
            let varVal = (data + offset).withMemoryRebound(to: Float.self, capacity: 1) { $0.pointee }
            offset += 4
            
            let rssi = Int8(bitPattern: data[offset])
            offset += 1
            
            let count = data[offset]
            offset += 1
            
            let rtt = (data + offset).withMemoryRebound(to: UInt32.self, capacity: 1) { $0.pointee }
            offset += 4
            
            measurements.append(AnchorMeasurement(
                anchorMac: mac,
                distanceMeters: dist,
                variance: varVal,
                rssi: rssi,
                sampleCount: count,
                rttNanoseconds: rtt
            ))
        }
        
        return BeaconDataPacket(
            serialNumber: serialNumber,
            batteryLevel: batteryLevel,
            floor: floor,
            timestamp: timestamp,
            measurements: measurements
        )
    }
}

// 칼만 필터 상태
struct KalmanFilterState {
    var x: Float // 추정 거리
    var P: Float // 오차 공분산
    var Q: Float = 0.05 // 프로세스 노이즈
    var R: Float = 0.0 // 측정 노이즈
    var lastUpdateTime: UInt32
    var initialized: Bool
    
    mutating func update(measurement: Float, variance: Float, dt: Float) -> Float {
        if !initialized {
            x = measurement
            P = variance
            initialized = true
            return x
        }
        
        // 예측
        let xPred = x
        let pPred = P + Q * dt
        
        // 업데이트
        R = variance
        let K = pPred / (pPred + R)
        
        x = xPred + K * (measurement - xPred)
        P = (Float(1.0) - K) * pPred
        
        return x
    }
}

// ===== 매니저 클래스 =====

class WifiAPManager {
    static let wifiEventGroup = xEventGroupCreate()
    static let AP_STARTED_BIT = BIT1
    static let STA_CONNECTED_BIT = BIT0
    
    static func initAPSTA() {
        ESP_ERROR_CHECK(esp_netif_init())
        ESP_ERROR_CHECK(esp_event_loop_create_default())
        
        let apNetif = esp_netif_create_default_wifi_ap()
        esp_netif_create_default_wifi_sta()
        
        // AP IP 설정
        var ipInfo = esp_netif_ip_info_t()
        // IP4_ADDR 매크로 대체 (직접 할당)
        // 192.168.4.1
        // 192.168.4.1
        ipInfo.ip.addr = UInt32(1) | (UInt32(4) << 8) | (UInt32(168) << 16) | (UInt32(192) << 24) // Little Endian
        ipInfo.gw.addr = ipInfo.ip.addr
        ipInfo.netmask.addr = UInt32(0) | (UInt32(255) << 8) | (UInt32(255) << 16) | (UInt32(255) << 24)
        
        esp_netif_dhcps_stop(apNetif)
        esp_netif_set_ip_info(apNetif, &ipInfo)
        esp_netif_dhcps_start(apNetif)
        
        var cfg = get_wifi_init_config_default()
        ESP_ERROR_CHECK(esp_wifi_init(&cfg))
        
        // 이벤트 핸들러 등록 (C 래퍼 필요)
        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, swift_wifi_event_handler, nil, nil)
        esp_event_handler_instance_register(IP_EVENT, Int32(IP_EVENT_STA_GOT_IP.rawValue), swift_wifi_event_handler, nil, nil)
        
        // STA Config
        var staConfig = wifi_config_t()
        // Swift에서 C Union/Struct 초기화가 복잡하므로 memcpy 사용 권장
        // 여기서는 개념적으로 설정
        // staConfig.sta.ssid = ...
        
        // AP Config
        var apConfig = wifi_config_t()
        // apConfig.ap.ssid = ...
        // apConfig.ap.ftm_responder = true
        
        ESP_ERROR_CHECK(esp_wifi_set_mode(wifi_mode_t(UInt32(WIFI_MODE_APSTA.rawValue))))
        // ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &staConfig))
        // ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &apConfig))
        
        ESP_ERROR_CHECK(esp_wifi_start())
        
        // 대역폭 및 프로토콜 설정 (FTM 최적화)
        // ...
    }
}

@_cdecl("swift_wifi_event_handler")
func swift_wifi_event_handler(arg: UnsafeMutableRawPointer?, eventBase: esp_event_base_t?, eventId: Int32, eventData: UnsafeMutableRawPointer?) {
    if eventBase == WIFI_EVENT {
        if eventId == WIFI_EVENT_AP_START.rawValue {
            log_msg("AP Started")
            xEventGroupSetBits(WifiAPManager.wifiEventGroup, UInt32(WifiAPManager.AP_STARTED_BIT))
        } else if eventId == WIFI_EVENT_STA_START.rawValue {
            log_msg("STA Started, Connecting...")
            esp_wifi_connect()
        } else if eventId == WIFI_EVENT_STA_DISCONNECTED.rawValue {
            log_msg("STA Disconnected, Reconnecting...")
            esp_wifi_connect()
        }
    } else if eventBase == IP_EVENT {
        if eventId == IP_EVENT_STA_GOT_IP.rawValue {
            log_msg("STA Got IP")
            xEventGroupSetBits(WifiAPManager.wifiEventGroup, UInt32(WifiAPManager.STA_CONNECTED_BIT))
        }
    }
}

class HttpUploader {
    static func sendJson(_ jsonString: UnsafePointer<CChar>) {
        var config = esp_http_client_config_t()
        // [UInt8] -> UnsafePointer<CChar>
        GatewayConfig.serverURLBytes.withUnsafeBufferPointer { ptr in
            config.url = ptr.baseAddress!.withMemoryRebound(to: CChar.self, capacity: ptr.count) { $0 }
        }
        config.method = HTTP_METHOD_POST
        config.timeout_ms = 5000
        
        let client = esp_http_client_init(&config)
        
        esp_http_client_set_header(client, "Content-Type", "application/json")
        esp_http_client_set_post_field(client, jsonString, Int32(strlen(jsonString)))
        
        let err = esp_http_client_perform(client)
        if err == ESP_OK {
            log_msg("HTTP POST Success")
        } else {
            log_int("HTTP POST Failed", Int32(err))
        }
        
        esp_http_client_cleanup(client)
    }
}

class DataManager {
    static var beaconStates = [String: [String: KalmanFilterState]]() // Serial -> AnchorMAC -> State
    
    static func processPacket(_ packet: BeaconDataPacket) {
        // JSON 생성
        // JSON 생성
        let root = cJSON_CreateObject()
        // [UInt8] -> String (cJSON은 char* 필요하므로 unsafe pointer 사용)
        _ = packet.serialNumber.withUnsafeBufferPointer { ptr in
            cJSON_AddStringToObject(root, "serial_number", ptr.baseAddress!)
        }
        cJSON_AddNumberToObject(root, "battery_level", Double(packet.batteryLevel))
        _ = packet.timestamp.withUnsafeBufferPointer { ptr in
            cJSON_AddStringToObject(root, "timestamp", ptr.baseAddress!)
        }
        
        let measurementsArray = cJSON_CreateArray()
        
        for m in packet.measurements {
            let mObj = cJSON_CreateObject()
            // String(format:) 대체
            let macStr = "MAC_ADDR" // 간소화
            
            cJSON_AddStringToObject(mObj, "anchor_mac", macStr)
            cJSON_AddNumberToObject(mObj, "distance", Double(m.distanceMeters))
            cJSON_AddNumberToObject(mObj, "variance", Double(m.variance))
            cJSON_AddNumberToObject(mObj, "rssi", Double(m.rssi))
            
            cJSON_AddItemToArray(measurementsArray, mObj)
        }
        
        cJSON_AddItemToObject(root, "measurements", measurementsArray)
        
        if let jsonStr = cJSON_PrintUnformatted(root) {
            HttpUploader.sendJson(jsonStr)
            free(jsonStr)
        }
        
        cJSON_Delete(root)
    }
}

@_cdecl("swift_beacon_data_recv_cb")
func swift_beacon_data_recv_cb(recvInfo: UnsafePointer<esp_now_recv_info_t>?, data: UnsafePointer<UInt8>?, len: Int32) {
    guard let data = data else { return }
    
    if let packet = BeaconDataPacket.fromBytes(data, length: Int(len)) {
        log_msg("Received Packet")
        DataManager.processPacket(packet)
    }
}

// ===== 메인 애플리케이션 =====

class GatewayApp {
    static var myDeviceName = ""
    static var myFloorNumber: Int32 = 0
    
    static func run() {
        log_msg("Gateway App Started (Swift)")
        
        // NVS 초기화 및 설정 로드
        var err = nvs_flash_init()
        if err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND {
            ESP_ERROR_CHECK(nvs_flash_erase())
            err = nvs_flash_init()
        }
        
        // WiFi 초기화
        WifiAPManager.initAPSTA()
        
        // SNTP 초기화
        // ...
        
        // ESP-NOW 초기화
        ESP_ERROR_CHECK(esp_now_init())
        ESP_ERROR_CHECK(esp_now_register_recv_cb(swift_beacon_data_recv_cb))
        
        // 콘솔 초기화 (프로비저닝)
        // ...
        
        // 메인 루프
        while true {
            vTaskDelay(1000 / 10) // portTICK_PERIOD_MS approx 10ms
            // 층 브로드캐스트 등 주기적 작업 수행
        }
    }
}

@_cdecl("app_main")
func app_main() {
    keep_me_alive()
    GatewayApp.run()
}
