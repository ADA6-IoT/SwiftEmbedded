
//
//  BeaconApp.swift
//  Beacon Firmware
//
//  Created by SwiftEmbedded on 2025.
//

// Bridging Header를 통해 C API를 가져옵니다.
// #include "BridgingHeader.h"

// C 매크로 대체 (전역 함수)
func ESP_ERROR_CHECK(_ err: esp_err_t) {
    if err != ESP_OK {
        log_int("ESP_ERROR_CHECK failed", Int32(err))
        while true {}
    }
}

// ===== 설정 상수 =====
struct BeaconConfig {
    // static let wifiSSID = "Gateway_Network" // String 제거
    static let wifiSSIDBytes: [UInt8] = [0x47, 0x61, 0x74, 0x65, 0x77, 0x61, 0x79, 0x5F, 0x4E, 0x65, 0x74, 0x77, 0x6F, 0x72, 0x6B] // "Gateway_Network"
    static let wifiPassword = ""
    static let ftmRssiThreshold: Int8 = -85
    static let maxFtmCandidates = 6
    static let maxRetryAttempts = 3
    static let floorDiscoveryDurationMs: UInt32 = 1000
    static let sleepDurationSec: UInt64 = 5
    
    // 가상 배터리 설정
    static let batteryDecayIntervalSec: Int64 = 600
    static let batteryDecayPercent: Int = 5
    static let batteryTotalLifetimeSec: Int64 = 12000
    static let nvsNamespace = "storage" // "battery" -> "storage"
    static let nvsKeyStartTime = "start_time"
    
    // FTM 최적화 파라미터
    static let ftmFrameCount: UInt8 = 16 // 24 -> 16
    static let ftmBurstPeriod: UInt16 = 2
    static let maxFtmRetry = 3 // 2 -> 3
    static let minValidSamples = 6
    
    // FTM 보정 파라미터
    static let ftmCalibrationFactor: Float = 0.20
    static let maxVarianceThreshold: Float = 0.10
    
    static let tag = "BEACON"
    static let serialNumber = "A-03"
    
    // Physics Constants (Explicit Float to avoid Double instructions on RISC-V)
    static let speedOfLight: Float = 299792458.0
    static let rttScale: Float = 1.0e-12 // Picoseconds to Seconds
    static let two: Float = 2.0
}

// ===== 데이터 구조 (C 구조체 대체) =====

// 비콘 데이터 패킷
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
    
    // C 구조체로 변환 (전송용)
    // 주의: 메모리 레이아웃이 C 구조체와 일치해야 함.
    // 여기서는 편의상 직렬화 로직을 별도로 구현하거나,
    // C에서 정의된 구조체를 그대로 사용하는 것이 안전할 수 있음.
    // 하지만 Swift Native하게 구현하고 전송 시 바이트 배열로 변환하는 방식을 사용.
    func toBytes() -> [UInt8] {
        var bytes = [UInt8]()
        
        // Serial Number (10 bytes)
        var serialBytes = [UInt8](repeating: 0, count: 10)
        let serialData = serialNumber // 이미 [UInt8]
        for (i, byte) in serialData.enumerated() {
            if i < 10 { serialBytes[i] = byte }
        }
        bytes.append(contentsOf: serialBytes)
        
        bytes.append(batteryLevel)
        bytes.append(UInt8(bitPattern: floor))
        
        // Timestamp (128 bytes)
        var timeBytes = [UInt8](repeating: 0, count: 128)
        let timeData = timestamp // 이미 [UInt8]
        for (i, byte) in timeData.enumerated() {
            if i < 128 { timeBytes[i] = byte }
        }
        bytes.append(contentsOf: timeBytes)
        
        // Measurements (3 slots)
        for i in 0..<3 {
            if i < measurements.count {
                let m = measurements[i]
                bytes.append(contentsOf: m.anchorMac) // 6 bytes
                withUnsafeBytes(of: m.distanceMeters) { bytes.append(contentsOf: $0) }
                withUnsafeBytes(of: m.variance) { bytes.append(contentsOf: $0) }
                bytes.append(UInt8(bitPattern: m.rssi))
                bytes.append(m.sampleCount)
                withUnsafeBytes(of: m.rttNanoseconds) { bytes.append(contentsOf: $0) }
            } else {
                // Empty slot
                bytes.append(contentsOf: [UInt8](repeating: 0, count: 6))
                bytes.append(contentsOf: [UInt8](repeating: 0, count: 4)) // float
                bytes.append(contentsOf: [UInt8](repeating: 0, count: 4)) // float
                bytes.append(0) // rssi
                bytes.append(0) // sample count
                bytes.append(contentsOf: [UInt8](repeating: 0, count: 4)) // rtt
            }
        }
        
        return bytes
    }
}

struct FloorInfo {
    var gatewayMac: [UInt8]
    var floor: Int8
    var rssi: Int8
    var channel: UInt8
}

struct GatewayInfo {
    var mac: [UInt8]
    var channel: UInt8
    var rssi: Int8
}

// ===== 유틸리티 클래스 =====

class BatteryManager {
    static func initNVS() {
        // nvs_flash_init() 호출은 App 진입점에서 수행
        var err = nvs_flash_init()
        if err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND {
            ESP_ERROR_CHECK(nvs_flash_erase())
            err = nvs_flash_init()
        }
        ESP_ERROR_CHECK(err)
        
        // 배터리 NVS 초기화
        var handle: nvs_handle_t = 0
        err = nvs_open(BeaconConfig.nvsNamespace, nvs_open_mode_t(UInt32(NVS_READWRITE.rawValue)), &handle)
        
        if err != ESP_OK {
            var tv = timeval()
            gettimeofday(&tv, nil)
            let nowUs = Int64(tv.tv_sec) * 1000000 + Int64(tv.tv_usec)
            
            err = nvs_set_i64(handle, BeaconConfig.nvsKeyStartTime, nowUs)
            nvs_commit(handle)
            log_int("New start time saved", Int32(nowUs / 1000000))
        } else {
            var startTime: Int64 = 0
            nvs_get_i64(handle, BeaconConfig.nvsKeyStartTime, &startTime)
            log_int("Loaded start time", Int32(startTime / 1000000))
        }
        
        nvs_close(handle)
    }
    
    static func getLevel() -> UInt8 {
        var handle: nvs_handle_t = 0
        if nvs_open(BeaconConfig.nvsNamespace, nvs_open_mode_t(UInt32(NVS_READONLY.rawValue)), &handle) != ESP_OK {
            log_int("NVS Open Failed", Int32(nvs_open(BeaconConfig.nvsNamespace, nvs_open_mode_t(UInt32(NVS_READONLY.rawValue)), &handle)))
            return 100
        }
        
        var startTime: Int64 = 0
        if nvs_get_i64(handle, BeaconConfig.nvsKeyStartTime, &startTime) != ESP_OK {
            nvs_close(handle)
            return 100
        }
        nvs_close(handle)
        
        var tv = timeval()
        gettimeofday(&tv, nil)
        let nowUs = Int64(tv.tv_sec) * 1000000 + Int64(tv.tv_usec)
        
        let elapsedSec = (nowUs - startTime) / 1000000
        let decayCycles = elapsedSec / BeaconConfig.batteryDecayIntervalSec
        let decrease = Int(decayCycles) * BeaconConfig.batteryDecayPercent
        
        let level = max(0, 100 - decrease)
        log_int("Battery Level", Int32(level))
        log_int("Elapsed Sec", Int32(elapsedSec))
        
        return UInt8(level)
    }
}

// ===== FTM 및 WiFi 관리자 =====

class WifiManager {
    static let ftmEventGroup = xEventGroupCreate()
    static let FTM_REPORT_BIT = BIT0
    static let FTM_FAILURE_BIT = BIT1
    
    static var ftmReportData: UnsafeMutablePointer<wifi_ftm_report_entry_t>? = nil
    static var ftmReportNumEntries: UInt8 = 0
    
    static func initWiFi() {
        ESP_ERROR_CHECK(esp_netif_init())
        ESP_ERROR_CHECK(esp_event_loop_create_default())
        esp_netif_create_default_wifi_sta()
        
        var cfg = get_wifi_init_config_default()
        ESP_ERROR_CHECK(esp_wifi_init(&cfg))
        ESP_ERROR_CHECK(esp_wifi_set_mode(wifi_mode_t(UInt32(WIFI_MODE_STA.rawValue))))
        ESP_ERROR_CHECK(esp_wifi_start())
        
        // FTM 최적화 설정
        esp_wifi_set_bandwidth(wifi_interface_t(UInt32(WIFI_IF_STA.rawValue)), wifi_bandwidth_t(UInt32(WIFI_BW_HT20.rawValue)))
        esp_wifi_set_protocol(wifi_interface_t(UInt32(WIFI_IF_STA.rawValue)), UInt8(WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N))
    }
    
    // C 콜백을 위한 래퍼 (Swift 클로저를 직접 C 함수 포인터로 전달 불가하므로 전역 함수 사용 필요)
    // 여기서는 편의상 C 스타일 전역 함수를 Swift 파일 내에 @_cdecl로 정의하여 사용하거나,
    // 기존 C 핸들러 로직을 Swift로 옮겨옴.
}

// FTM 이벤트 핸들러 (C 콜백 대체)
@_cdecl("swift_ftm_event_handler")
func swift_ftm_event_handler(arg: UnsafeMutableRawPointer?, eventBase: esp_event_base_t?, eventId: Int32, eventData: UnsafeMutableRawPointer?) {
    guard eventId == WIFI_EVENT_FTM_REPORT.rawValue else { return }
    
    let event = eventData!.assumingMemoryBound(to: wifi_event_ftm_report_t.self).pointee
    log_int("FTM Report Received. Status", Int32(event.status.rawValue))
    
    WifiManager.ftmReportNumEntries = event.ftm_report_num_entries
    
    if WifiManager.ftmReportData != nil {
        free(WifiManager.ftmReportData)
        WifiManager.ftmReportData = nil
    }
    
    if WifiManager.ftmReportNumEntries > 0 && event.ftm_report_data != nil {
        let size = MemoryLayout<wifi_ftm_report_entry_t>.stride * Int(WifiManager.ftmReportNumEntries)
        WifiManager.ftmReportData = UnsafeMutablePointer<wifi_ftm_report_entry_t>.allocate(capacity: Int(WifiManager.ftmReportNumEntries))
        memcpy(WifiManager.ftmReportData, event.ftm_report_data, size)
    }
    
    if event.status == FTM_STATUS_SUCCESS && WifiManager.ftmReportNumEntries > 0 {
        xEventGroupSetBits(WifiManager.ftmEventGroup, UInt32(WifiManager.FTM_REPORT_BIT))
    } else {
        xEventGroupSetBits(WifiManager.ftmEventGroup, UInt32(WifiManager.FTM_FAILURE_BIT))
    }
}

class DistanceCalculator {
    static func performMeasurement(bssid: [UInt8], channel: UInt8) -> (distance: Float, variance: Float, rtt: UInt32)? {
        // String(format:) 대체
        log_int("Starting FTM Measurement on ch", Int32(channel))
        
        var bestDistance: Float = 0
        var bestVariance: Float = 999999.0
        var bestRtt: UInt32 = 0
        var success = false
        
        for attempt in 0..<BeaconConfig.maxFtmRetry {
            // 이벤트 핸들러 등록
            esp_event_handler_register(WIFI_EVENT, Int32(WIFI_EVENT_FTM_REPORT.rawValue), swift_ftm_event_handler, nil)
            
            var ftmCfg = wifi_ftm_initiator_cfg_t()
            // bssid 복사
            for _ in 0..<6 { ftmCfg.resp_mac.0 = bssid[0]; ftmCfg.resp_mac.1 = bssid[1]; ftmCfg.resp_mac.2 = bssid[2]; ftmCfg.resp_mac.3 = bssid[3]; ftmCfg.resp_mac.4 = bssid[4]; ftmCfg.resp_mac.5 = bssid[5] } // Swift 튜플 초기화의 한계로 인해 실제로는 memcpy 사용 권장
            // Swift에서 C 배열/튜플 초기화가 까다로우므로 memcpy 사용
            var bssidCopy = bssid
            _ = withUnsafeMutablePointer(to: &ftmCfg.resp_mac) { ptr in
                memcpy(ptr, &bssidCopy, 6)
            }
            
            ftmCfg.channel = channel
            ftmCfg.frm_count = BeaconConfig.ftmFrameCount
            ftmCfg.burst_period = BeaconConfig.ftmBurstPeriod
            
            xEventGroupClearBits(WifiManager.ftmEventGroup, UInt32(WifiManager.FTM_REPORT_BIT | WifiManager.FTM_FAILURE_BIT))
            WifiManager.ftmReportNumEntries = 0
            
            if esp_wifi_ftm_initiate_session(&ftmCfg) != ESP_OK {
                log_msg("FTM Init Failed")
                esp_event_handler_unregister(WIFI_EVENT, Int32(WIFI_EVENT_FTM_REPORT.rawValue), swift_ftm_event_handler)
                return nil // RSSI fallback 구현 생략 (간소화)
            }
            
            let bits = xEventGroupWaitBits(WifiManager.ftmEventGroup, UInt32(WifiManager.FTM_REPORT_BIT | WifiManager.FTM_FAILURE_BIT), pdTRUE, pdFALSE, 6000 / 10) // portTICK_PERIOD_MS approx 10ms
            
            if (bits & UInt32(WifiManager.FTM_REPORT_BIT)) != 0 {
                // 데이터 처리
                var distances = [Float]()
                if let data = WifiManager.ftmReportData {
                    for i in 0..<Int(WifiManager.ftmReportNumEntries) {
                        let entry = data[i]
                        // log_int("RTT", Int32(entry.rtt))
                        if entry.rtt >= 1000 && entry.rtt <= 333000 {
                            let rttFloat = Float(entry.rtt)
                            let distRaw = (rttFloat * BeaconConfig.rttScale * BeaconConfig.speedOfLight) / BeaconConfig.two
                            let distCal = Float(distRaw) * BeaconConfig.ftmCalibrationFactor
                            if distCal >= 0.15 && distCal <= 50.0 {
                                distances.append(distCal)
                            }
                        }
                    }
                }
                
                // IQR 및 통계 처리
                if distances.count >= BeaconConfig.minValidSamples {
                    distances.sort()
                    // IQR 생략하고 간단히 중앙값 사용 (복잡도 감소)
                    let median = distances[distances.count / 2]
                    
                    // 분산 계산
                    // 분산 계산 (수동 루프)
                    var sum: Float = 0
                    for d in distances { sum += d }
                    let mean = sum / Float(distances.count)
                    
                    var varSum: Float = 0
                    for d in distances {
                        let diff = d - mean
                        varSum += diff * diff
                    }
                    let variance = varSum / Float(distances.count)
                    
                    if variance < bestVariance {
                        bestVariance = variance
                        bestDistance = median
                        bestDistance = median
                        // 근사치 역산
                        let rttReverse = (bestDistance / BeaconConfig.ftmCalibrationFactor) * BeaconConfig.two / BeaconConfig.speedOfLight * 1.0e9
                        bestRtt = UInt32(rttReverse)
                        success = true
                        
                        if bestVariance < BeaconConfig.maxVarianceThreshold {
                            break // 충분히 좋으면 중단
                        }
                    }
                }
            }
            
            esp_wifi_ftm_end_session()
            esp_event_handler_unregister(WIFI_EVENT, Int32(WIFI_EVENT_FTM_REPORT.rawValue), swift_ftm_event_handler)
            vTaskDelay(200 / 10) // portTICK_PERIOD_MS approx 10ms
        }
        
        return success ? (bestDistance, bestVariance, bestRtt) : nil
    }
}

// ===== 메인 애플리케이션 =====

class BeaconApp {
    static var floorList = [FloorInfo]()
    static var uploadSuccessful = false
    
    static func run() {
        log_msg("Beacon App Started (Swift)")
        
        BatteryManager.initNVS()
        WifiManager.initWiFi()
        
        // 1. 게이트웨이 스캔
        log_msg("Step 1: Scanning Gateways")
        var scanConfig = wifi_scan_config_t()
        scanConfig.scan_type = wifi_scan_type_t(WIFI_SCAN_TYPE_ACTIVE.rawValue)
        scanConfig.scan_time.active.min = 100
        scanConfig.scan_time.active.max = 300
        
        ESP_ERROR_CHECK(esp_wifi_scan_start(&scanConfig, true))
        
        var apCount: UInt16 = 0
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&apCount))
        
        var gateways = [GatewayInfo]()
        
        if apCount > 0 {
            let apRecords = UnsafeMutablePointer<wifi_ap_record_t>.allocate(capacity: Int(apCount))
            ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&apCount, apRecords))
            
            for i in 0..<Int(apCount) {
                let record = apRecords[i]
                // SSID 확인 (Mirror 대신 UnsafeBytes 사용)
                var ssid = [UInt8]()
                withUnsafeBytes(of: record.ssid) { buffer in
                    for byte in buffer {
                        if byte != 0 { ssid.append(byte) }
                    }
                }
                // String 제거 및 바이트 비교
                if ssid == BeaconConfig.wifiSSIDBytes {
                    // BSSID(MAC) 복사
                    var mac = [UInt8]()
                    mac.append(record.bssid.0); mac.append(record.bssid.1); mac.append(record.bssid.2)
                    mac.append(record.bssid.3); mac.append(record.bssid.4); mac.append(record.bssid.5)
                    
                    gateways.append(GatewayInfo(mac: mac, channel: record.primary, rssi: record.rssi))
                    log_int("Found Gateway on ch", Int32(record.primary))
                }
            }
            apRecords.deallocate()
        }
        
        // 2. 층 정보 수집 (ESP-NOW)
        // Swift에서 ESP-NOW 콜백 구현은 복잡하므로 여기서는 생략하고 FTM 측정에 집중
        // 실제로는 C 래퍼를 통해 Swift 클로저를 호출해야 함.
        
        // 3. FTM 측정 및 데이터 전송
        log_msg("Step 3: FTM Measurement")
        var measurements = [BeaconDataPacket.AnchorMeasurement]()
        
        for gw in gateways {
            if measurements.count >= 3 { break }
            
            // 채널 변경
            esp_wifi_set_channel(gw.channel, wifi_second_chan_t(WIFI_SECOND_CHAN_NONE.rawValue))
            
            if let result = DistanceCalculator.performMeasurement(bssid: gw.mac, channel: gw.channel) {
                measurements.append(BeaconDataPacket.AnchorMeasurement(
                    anchorMac: gw.mac,
                    distanceMeters: result.distance,
                    variance: result.variance,
                    rssi: gw.rssi,
                    sampleCount: 0, // 상세 구현 생략
                    rttNanoseconds: result.rtt
                ))
            }
        }
        
        // 4. 데이터 패킷 생성 및 전송
            // Timestamp (strftime)
        var timeBuf = [CChar](repeating: 0, count: 64)
        var now = time(nil)
        var timeinfo = tm()
        localtime_r(&now, &timeinfo)
        strftime(&timeBuf, 64, "%Y-%m-%d %H:%M:%S", &timeinfo)
        
        // [CChar] -> [UInt8]
        var timestampBytes = [UInt8]()
        for char in timeBuf {
            if char == 0 { break }
            timestampBytes.append(UInt8(bitPattern: char))
        }
        timestampBytes.append(0)
        
        // Serial (MAC) -> [UInt8]
        var mac = [UInt8](repeating: 0, count: 6)
        esp_read_mac(&mac, ESP_MAC_WIFI_STA)
        // Hex string conversion omitted for simplicity, sending raw bytes or simple ID
        // For now, just use MAC bytes as serial
        var serialBytes = mac
        serialBytes.append(0)

        let packet = BeaconDataPacket(
            serialNumber: serialBytes,
            batteryLevel: BatteryManager.getLevel(),
            floor: 1, // Example
            timestamp: timestampBytes,
            measurements: measurements
        )
        
        // ESP-NOW 전송
        // Swift에서 ESP-NOW Peer 추가 및 전송 로직 구현 필요
        // 여기서는 데이터 준비까지만 수행
        log_msg("Data Packet Ready")
        
        // Deep Sleep
        log_int("Entering Deep Sleep (s)", Int32(BeaconConfig.sleepDurationSec))
        esp_deep_sleep(BeaconConfig.sleepDurationSec * 1000000)
    }
}

// ===== 진입점 =====

@_cdecl("app_main")
func app_main() {
    keep_me_alive()
    BeaconApp.run()
}
