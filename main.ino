// ESP32 통합 시뮬레이터 v1.0
// 인버터 시뮬레이터 + 모의계량기 + RTU 제어 통합
// ESP32-WROOM-32D, Arduino IDE
// 2025.07.30 - 통합 버전

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <math.h>

// WiFi 설정
const char* ssid = "haezoomAP";
const char* password = "idnstory132";

// 웹서버 설정
WebServer server(80);

// RTU 오픈컬렉터 출력 핀 정의
#define RTU_WP_PIN 25      // RTU로 WP 펄스 출력 (오픈컬렉터)
#define RTU_EOI_PIN 26     // RTU로 EOI 펄스 출력 (오픈컬렉터)
#define WP_PULSE_WIDTH 100   // ms
#define EOI_PULSE_WIDTH 100  // ms

// ⚡ CT/PT 설정 (RTU와 동일하게!)
#define CTR 6              // CT 비율 (전류 변성기)
#define PTR 120            // PT 비율 (전압 변성기)
#define WPP 0.2            // WP 계수 (Watt-Pulse)

// 🔄 RTU 펄스 계산 공식과 맞춤
// RTU: Total_p = pulse_count * CTR * PTR * WPP * 0.001
// 펄스 1개당 전력량 = 6 * 120 * 0.2 * 0.001 = 0.144 kWh = 144 Wh
#define WP_PULSE_WH 144.0  // WP 펄스당 Wh (RTU 계산식과 일치)
#define EOI_INTERVAL_MS (15 * 60 * 1000)  // EOI 신호 간격: 15분

// 🏭 인버터 시뮬레이터 데이터
struct InverterData {
  // 제어 상태
  bool power_on = false;
  bool p_control = false;      // P제어 모드
  bool mppt_control = true;    // MPPT제어 모드 (기본값)
  float limit = 100.0;         // 발전제한율 (%)
  
  // 설정값
  float set_power = 1000.0;    // 설정 전력 (W)
  float nominal_power = 10000.0; // 정격 출력 (W)
  float voltage = 220.0;       // 전압 (V)
  
  // 환경 조건
  float irradiance = 800.0;    // 일사량 (W/m²)
  float temperature = 25.0;    // 온도 (°C)
  
  // 출력값 (1차측 실제값)
  float power_w = 0.0;         // 유효전력 (W)
  float power_q = 0.0;         // 무효전력 (var)
  float current = 0.0;         // 전류 (A)
  float phase_deg = 0.0;       // 위상각 (도)
  
  // 로그
  String last_log = "";
  unsigned long last_log_time = 0;
} inverter;

// 🔋 모의계량기 데이터
struct MeterData {
  // 📡 1차측 데이터 (인버터에서 직접 수신)
  float primary_power_w = 0.0;
  float primary_power_q = 0.0;
  float primary_voltage = 0.0;
  float primary_current = 0.0;
  
  // 🔄 2차측 변환값 (CT/PT 적용 후)
  float secondary_voltage = 0.0;
  float secondary_current = 0.0;
  float secondary_power_w = 0.0;
  
  // 📊 계량기 측정값 (2차측 → 1차측 역산)
  float measured_power_w = 0.0;
  float measured_voltage = 0.0;
  float measured_current = 0.0;
  
  // 발전량 및 펄스
  float energy_wh = 0.0;
  float last_energy_wh = 0.0;
  unsigned long wp_count = 0;
  unsigned long eoi_count = 0;
  
  // 시간 정보
  unsigned long last_wp_time = 0;
  unsigned long last_eoi_time = 0;
  unsigned long last_signal_time = 0;
  String last_signal = "";
} meter;

// 시스템 상태
bool wifi_connected = false;
unsigned long uptime = 0;
unsigned long last_eoi_check = 0;

// 함수 선언
void handleRoot();
void handleStatus();
void handleInverterControl();
void handleMeterControl();
void handleReset();
void generateWPPulse();
void generateEOISignal();
void calculateSecondaryValues();
void calculateMeasuredValues();
void simulateInverter();
void transferDataToMeter();
void addLog(String message);

void setup() {
  // RTU 오픈컬렉터 출력 핀 설정
  pinMode(RTU_WP_PIN, OUTPUT);
  pinMode(RTU_EOI_PIN, OUTPUT);
  digitalWrite(RTU_WP_PIN, HIGH);  // 오픈컬렉터: 평상시 HIGH
  digitalWrite(RTU_EOI_PIN, HIGH);
  
  Serial.begin(115200);
  Serial.println("========================================");
  Serial.println("🏭 ESP32 통합 시뮬레이터 v1.0");
  Serial.println("📊 인버터 시뮬레이터 + 모의계량기 + RTU 제어");
  Serial.println("🔄 내부 직접 통신으로 안정성 극대화");
  Serial.printf("⚡ CT:%d, PT:%d, WP:%.1f\n", CTR, PTR, WPP);
  Serial.println("📡 시리얼 통신 지원: 'help' 명령어로 사용법 확인");
  Serial.println("========================================");
  
  // WiFi 연결
  WiFi.begin(ssid, password);
  Serial.print("WiFi 연결 중...");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("✅ WiFi 연결됨! IP 주소: ");
    Serial.println(WiFi.localIP());
    wifi_connected = true;
  } else {
    Serial.println();
    Serial.println("❌ WiFi 연결 실패!");
    Serial.println("📡 시리얼 통신 모드로 동작합니다.");
    Serial.println("💡 'help' 명령어로 사용법을 확인하세요.");
    wifi_connected = false;
  }
  
  // 웹서버 라우트 설정
  server.on("/", handleRoot);
  server.on("/api/status", handleStatus);
  server.on("/api/inverter_control", HTTP_POST, handleInverterControl);
  server.on("/api/meter_control", HTTP_POST, handleMeterControl);
  server.on("/api/reset", HTTP_POST, handleReset);
  
  server.begin();
  Serial.println("🌐 통합 웹서버 시작됨");
  Serial.println("🎯 준비 완료! 인버터 시뮬레이션 시작...");
  
  // 초기 로그
  addLog("시스템 시작");
}

// 📊 인버터 시뮬레이션
void simulateInverter() {
  if (!inverter.power_on) {
    inverter.power_w = 0.0;
    inverter.current = 0.0;
    inverter.power_q = 0.0;
    return;
  }
  
  // 환경 조건 시뮬레이션 (실제 환경 변화를 모사)
  static unsigned long last_env_update = 0;
  if (millis() - last_env_update > 1000) {  // 1초마다 환경 변화
    inverter.irradiance = random(600, 1000);  // 600~1000 W/m²
    inverter.temperature = random(20, 35);    // 20~35°C
    last_env_update = millis();
  }
  
  // 위상각 계산
  inverter.phase_deg = random(0, 30);  // 0~30도
  float phase_rad = inverter.phase_deg * PI / 180.0;
  
  // 전류 계산 (정격용량 기반)
  float base_current = inverter.nominal_power / inverter.voltage;  // 정격 전류
  float irradiance_factor = inverter.irradiance / 1000.0;
  float temp_factor = 1.0 + (inverter.temperature - 25.0) * 0.002;
  float actual_current = base_current * irradiance_factor * temp_factor;
  
  inverter.current = actual_current;
  
  // 제어 모드별 전력 계산
  if (inverter.p_control) {
    // 🎯 P제어: 설정값대로 강제 출력 (환경 조건 무시)
    float max_power = inverter.nominal_power * (inverter.limit / 100.0);
    inverter.power_w = min(inverter.set_power, max_power);
    
    // P제어 시 전류도 설정 전력에 맞게 재계산
    if (inverter.voltage > 0) {
      float calculated_current = inverter.power_w / (inverter.voltage * cos(phase_rad));
      inverter.current = calculated_current;
    }
    
    static unsigned long last_p_log = 0;
    if (millis() - last_p_log > 5000) {  // 5초마다 로그
      addLog("P제어: " + String(inverter.set_power) + "W → " + String(inverter.power_w) + "W");
      last_p_log = millis();
    }
    
  } else if (inverter.mppt_control) {
    // 🌞 MPPT제어: 환경 조건에 따른 자동 출력
    float base_power = inverter.voltage * inverter.current * cos(phase_rad);
    inverter.power_w = base_power * (1.0 + 0.05 * sin(millis() / 1000.0));  // 약간의 변동
    
    // 발전제한율 적용
    float max_power = inverter.nominal_power * (inverter.limit / 100.0);
    inverter.power_w = min(inverter.power_w, max_power);
    
  } else {
    inverter.power_w = 0.0;
  }
  
  // 무효전력 계산
  inverter.power_q = inverter.voltage * inverter.current * sin(phase_rad);
}

// 🔄 인버터 → 계량기 데이터 전송 (내부 직접 통신)
void transferDataToMeter() {
  // 직접 메모리 복사 (WiFi 통신 없음!)
  meter.primary_power_w = inverter.power_w;
  meter.primary_power_q = inverter.power_q;
  meter.primary_voltage = inverter.voltage;
  meter.primary_current = inverter.current;
  
  meter.last_signal_time = millis();
  meter.last_signal = "내부 데이터 전송";
  
  // 2차측 값 계산
  calculateSecondaryValues();
  calculateMeasuredValues();
}

// 🔄 2차측 값 계산 (CT/PT 적용)
void calculateSecondaryValues() {
  meter.secondary_voltage = meter.primary_voltage / PTR;  // 전압: 1차측 ÷ PT비율
  meter.secondary_current = meter.primary_current / CTR;  // 전류: 1차측 ÷ CT비율
  meter.secondary_power_w = meter.secondary_voltage * meter.secondary_current;
}

// 📊 계량기 측정값 계산 (2차측 → 1차측 역산)
void calculateMeasuredValues() {
  meter.measured_voltage = meter.secondary_voltage * PTR;  // 전압: 2차측 × PT비율
  meter.measured_current = meter.secondary_current * CTR;  // 전류: 2차측 × CT비율
  meter.measured_power_w = meter.measured_voltage * meter.measured_current;
}

// ⚡ WP 펄스 생성 및 출력
void generateWPPulse() {
  meter.wp_count++;
  meter.last_wp_time = millis();
  meter.last_signal = "WP 펄스 생성";
  meter.last_signal_time = millis();
  
  Serial.printf("⚡ WP 펄스 생성 (#%lu)\n", meter.wp_count);
  Serial.printf("  펄스당: %.1f Wh = 0.144kWh (CT:%d × PT:%d × WP:%.1f)\n", 
                WP_PULSE_WH, CTR, PTR, WPP);
  Serial.printf("  누적 발전량: %.3f Wh (1차측 실제)\n", meter.energy_wh);
  Serial.printf("  현재 출력: %.1f W (인버터 실제)\n", inverter.power_w);
  
  // RTU로 WP 펄스 (오픈컬렉터)
  digitalWrite(RTU_WP_PIN, LOW);
  delay(WP_PULSE_WIDTH);
  digitalWrite(RTU_WP_PIN, HIGH);
}

// 📋 EOI 신호 생성 및 출력
void generateEOISignal() {
  meter.eoi_count++;
  meter.last_eoi_time = millis();
  meter.last_signal = "EOI 신호 생성";
  meter.last_signal_time = millis();
  
  Serial.printf("📋 EOI 신호 생성 (#%lu): 15분 경과\n", meter.eoi_count);
  
  // RTU로 EOI 펄스 (오픈컬렉터)
  digitalWrite(RTU_EOI_PIN, LOW);
  delay(EOI_PULSE_WIDTH);
  digitalWrite(RTU_EOI_PIN, HIGH);
}

// 📝 로그 추가
void addLog(String message) {
  inverter.last_log = "[" + String(millis()/1000) + "s] " + message;
  inverter.last_log_time = millis();
  Serial.println("📝 " + inverter.last_log);
}

// 📡 시리얼 통신 처리
void handleSerialInput() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "help" || command == "?") {
      printSerialHelp();
    } else if (command == "status" || command == "s") {
      printSerialStatus();
    } else if (command == "power on" || command == "on") {
      inverter.power_on = true;
      Serial.println("✅ 인버터 전원 ON");
      addLog("인버터 전원 ON");
    } else if (command == "power off" || command == "off") {
      inverter.power_on = false;
      Serial.println("❌ 인버터 전원 OFF");
      addLog("인버터 전원 OFF");
    } else if (command == "mode p" || command == "p") {
      inverter.p_control = true;
      inverter.mppt_control = false;
      Serial.println("🎛️ P제어 모드 활성화");
      addLog("P제어 모드 활성화");
    } else if (command == "mode mppt" || command == "mppt") {
      inverter.p_control = false;
      inverter.mppt_control = true;
      Serial.println("🔄 MPPT제어 모드 활성화");
      addLog("MPPT제어 모드 활성화");
    } else if (command.startsWith("set power ")) {
      float power = command.substring(10).toFloat();
      inverter.set_power = max(0.0f, min(power, inverter.nominal_power));
      Serial.printf("⚡ 설정전력: %.0fW\n", inverter.set_power);
      addLog("설정전력: " + String(inverter.set_power) + "W");
    } else if (command.startsWith("set nominal ")) {
      float nominal = command.substring(12).toFloat();
      inverter.nominal_power = max(1000.0f, min(nominal, 1000000.0f));
      Serial.printf("🏭 정격전력: %.0fW\n", inverter.nominal_power);
      addLog("정격전력: " + String(inverter.nominal_power) + "W");
    } else if (command.startsWith("set limit ")) {
      float limit = command.substring(10).toFloat();
      inverter.limit = max(0.0f, min(limit, 100.0f));
      Serial.printf("📊 발전제한율: %.1f%%\n", inverter.limit);
      addLog("발전제한율: " + String(inverter.limit) + "%");
    } else if (command == "reset") {
      meter.energy_wh = 0.0;
      meter.last_energy_wh = 0.0;
      meter.wp_count = 0;
      meter.eoi_count = 0;
      Serial.println("🔄 발전량 및 펄스 카운터 초기화 완료");
      addLog("발전량 및 펄스 초기화");
    } else if (command == "wp") {
      generateWPPulse();
      Serial.println("⚡ 수동 WP 펄스 생성");
        } else if (command.startsWith("wp ")) {
      int count = command.substring(3).toInt();
      if (count > 0 && count <= 1000) {
        Serial.printf("⚡ 연속 WP 펄스 %d개 생성 시작... (예상 소요시간: %.1f초)\n", count, (count-1) * 0.52 + 0.1);
        for (int i = 0; i < count; i++) {
          generateWPPulse();
          Serial.printf("  WP #%d 완료\n", i+1);
          delay(420);  // 펄스 간 420ms 간격 (RTU 디바운싱 400ms + 여유 20ms)
        }
        Serial.printf("✅ 연속 WP 펄스 %d개 생성 완료!\n", count);
      } else {
        Serial.println("❌ 1~1000 범위의 숫자를 입력하세요. 예: wp 10");
      }
      
    } else if (command.startsWith("wpt ")) {
      // 테스트용 연속 WP (간격 조정 가능)
      // 사용법: wpt <개수> <간격ms>
      int spaceIndex = command.indexOf(' ', 4);
      if (spaceIndex > 0) {
        int count = command.substring(4, spaceIndex).toInt();
        int interval = command.substring(spaceIndex + 1).toInt();
        
        if (count > 0 && count <= 1000 && interval >= 50 && interval <= 2000) {
          Serial.printf("🧪 테스트용 WP 펄스 %d개 생성 시작 (간격: %dms)...\n", count, interval);
          for (int i = 0; i < count; i++) {
            generateWPPulse();
            Serial.printf("  WP #%d 완료\n", i+1);
            if (i < count - 1) {
              delay(interval);
            }
          }
          Serial.printf("✅ 테스트용 WP 펄스 %d개 생성 완료! (간격: %dms)\n", count, interval);
        } else {
          Serial.println("❌ wpt <개수(1-1000)> <간격ms(50-2000)>. 예: wpt 10 200");
        }
      } else {
        Serial.println("❌ wpt <개수> <간격ms>. 예: wpt 10 200");
      }
      
    } else if (command == "wpfast") {
      // 즉시 연속 10개 (최소 간격)
      Serial.println("⚡ 즉시 연속 WP 10개 (최소간격 테스트)...");
      for (int i = 0; i < 10; i++) {
        generateWPPulse();
        Serial.printf("  WP #%d 완료\n", i+1);
        if (i < 9) delay(150);  // 최소 간격
      }
      Serial.println("✅ 즉시 연속 WP 10개 완료!");
      
    } else if (command == "wptest") {
      // 다양한 간격으로 테스트
      Serial.println("🧪 다양한 간격으로 펄스 테스트 시작...");
      int intervals[] = {100, 150, 200, 300, 400, 500};
      for (int j = 0; j < 6; j++) {
        Serial.printf("  📊 간격 %dms로 5개 테스트...\n", intervals[j]);
        for (int i = 0; i < 5; i++) {
          generateWPPulse();
          if (i < 4) delay(intervals[j]);
        }
        delay(1000);  // 각 테스트 사이 1초 대기
      }
      Serial.println("✅ 다양한 간격 테스트 완료!");
      
    } else if (command == "eoi") {
      generateEOISignal();
      Serial.println("📋 수동 EOI 신호 생성");
    } else if (command == "wifi") {
      if (wifi_connected) {
        Serial.printf("🌐 WiFi 연결됨: http://%s\n", WiFi.localIP().toString().c_str());
      } else {
        Serial.println("❌ WiFi 연결 안됨 (시리얼 모드)");
      }
    } else if (command != "") {
      Serial.println("❓ 알 수 없는 명령어. 'help'를 입력하세요.");
    }
  }
}

// 📖 시리얼 도움말
void printSerialHelp() {
  Serial.println("========================================");
  Serial.println("📡 ESP32 시리얼 통신 명령어");
  Serial.println("========================================");
  Serial.println("📊 상태 확인:");
  Serial.println("  status (s)      - 현재 상태 출력");
  Serial.println("  wifi            - WiFi 연결 상태");
  Serial.println("");
  Serial.println("🎛️ 인버터 제어:");
  Serial.println("  power on (on)   - 인버터 전원 ON");
  Serial.println("  power off (off) - 인버터 전원 OFF");
  Serial.println("  mode p (p)      - P제어 모드");
  Serial.println("  mode mppt       - MPPT제어 모드");
  Serial.println("");
  Serial.println("⚙️ 설정 변경:");
  Serial.println("  set power <값>   - 설정전력 (W)");
  Serial.println("  set nominal <값> - 정격전력 (W)");
  Serial.println("  set limit <값>   - 발전제한율 (%)");
  Serial.println("");
  Serial.println("🔧 수동 제어:");
  Serial.println("  wp              - 수동 WP 펄스 1개");
  Serial.println("  wp <개수>       - 연속 WP 펄스 (RTU 호환 간격)");
  Serial.println("  wpt <개수> <간격ms> - 테스트용 연속 WP (예: wpt 10 200)");
  Serial.println("  wpfast          - 빠른 연속 WP 10개 (최소간격)");
  Serial.println("  wptest          - 다양한 간격으로 펄스 테스트");
  Serial.println("  eoi             - 수동 EOI 신호");
  Serial.println("  reset           - 발전량 초기화");
  Serial.println("");
  Serial.println("📖 기타:");
  Serial.println("  help (?)        - 이 도움말");
  Serial.println("========================================");
}

// 📊 시리얼 상태 출력
void printSerialStatus() {
  Serial.println("========================================");
  Serial.println("📊 ESP32 통합 시뮬레이터 상태");
  Serial.println("========================================");
  Serial.printf("⏰ 시스템 가동시간: %lu초\n", uptime/1000);
  Serial.printf("🌐 WiFi: %s\n", wifi_connected ? WiFi.localIP().toString().c_str() : "연결 안됨");
  Serial.println("");
  Serial.println("🏭 인버터 상태:");
  Serial.printf("  전원: %s\n", inverter.power_on ? "ON" : "OFF");
  Serial.printf("  제어모드: %s\n", inverter.p_control ? "P제어" : (inverter.mppt_control ? "MPPT제어" : "정지"));
  Serial.printf("  정격전력: %.0fW\n", inverter.nominal_power);
  Serial.printf("  설정전력: %.0fW\n", inverter.set_power);
  Serial.printf("  발전제한율: %.1f%% (%.0fW)\n", inverter.limit, inverter.nominal_power * inverter.limit / 100.0);
  Serial.printf("  유효전력: %.1fW\n", inverter.power_w);
  Serial.printf("  무효전력: %.1fvar\n", inverter.power_q);
  Serial.printf("  전압: %.1fV\n", inverter.voltage);
  Serial.printf("  전류: %.2fA\n", inverter.current);
  Serial.printf("  일사량: %.0fW/m²\n", inverter.irradiance);
  Serial.printf("  온도: %.1f°C\n", inverter.temperature);
  Serial.println("");
  Serial.println("⚡ 계량기 상태:");
  Serial.printf("  1차측 전력: %.1fW\n", meter.primary_power_w);
  Serial.printf("  2차측 전력: %.1fW\n", meter.secondary_power_w);
  Serial.printf("  측정 전력: %.1fW\n", meter.measured_power_w);
  Serial.printf("  누적 발전량: %.3fkWh\n", meter.energy_wh / 1000.0);
  Serial.printf("  WP 펄스 수: %lu개\n", meter.wp_count);
  Serial.printf("  EOI 신호 수: %lu개\n", meter.eoi_count);
  Serial.println("");
  Serial.println("🔧 CT/PT 설정:");
  Serial.printf("  CTR: %d, PTR: %d, WPP: %.1f\n", CTR, PTR, WPP);
  Serial.printf("  펄스당 전력량: %.0fWh\n", WP_PULSE_WH);
  Serial.println("========================================");
}

void loop() {
  server.handleClient();
  
  // 📡 시리얼 통신 처리
  handleSerialInput();
  
  unsigned long current_time = millis();
  uptime = current_time;
  
  // 🏭 인버터 시뮬레이션 및 발전량 계산 (100ms마다)
  static unsigned long last_sim_time = 0;
  if (current_time - last_sim_time >= 100) {
    simulateInverter();
    transferDataToMeter();  // 내부 직접 전송
    
    // 🔋 발전량 계산 및 WP 펄스 생성 (1차측 실제 발전량으로 계산)
    if (inverter.power_w > 0) {
      // ⚡ 1차측 실제 발전량으로 누적 (인버터 실제 출력)
      // 100ms당 발전량 계산 (Wh)
      float energy_per_100ms = (inverter.power_w / 3600.0) * 0.1;  // 100ms = 0.1초
      meter.energy_wh += energy_per_100ms;
      
      // WP 펄스 생성 조건 확인 (144Wh = 0.144kWh마다 1펄스)
      if (meter.energy_wh - meter.last_energy_wh >= WP_PULSE_WH) {
        generateWPPulse();
        meter.last_energy_wh += WP_PULSE_WH;
      }
    }
    
    last_sim_time = current_time;
  }
  
  // 📋 EOI 신호 생성 (15분마다)
  if (current_time >= last_eoi_check) {
    if (current_time - last_eoi_check >= EOI_INTERVAL_MS) {
      generateEOISignal();
      last_eoi_check = current_time;
    }
  } else {
    last_eoi_check = current_time;  // millis() 오버플로우 처리
  }
  
  // 📊 상태 디버깅 (10초마다)
  static unsigned long last_debug_time = 0;
  if (current_time - last_debug_time >= 10000) {
    Serial.println("======= 통합 시뮬레이터 상태 =======");
    Serial.printf("🏭 인버터: %s, 모드: %s\n", 
                  inverter.power_on ? "ON" : "OFF",
                  inverter.p_control ? "P제어" : (inverter.mppt_control ? "MPPT" : "정지"));
    Serial.printf("   출력: %.1fW, 전압: %.1fV, 전류: %.2fA\n", 
                  inverter.power_w, inverter.voltage, inverter.current);
    Serial.printf("🔋 계량기: 측정 %.1fW, 누적 %.3fWh (1차측 실제)\n", 
                  meter.measured_power_w, meter.energy_wh);
    Serial.printf("⚡ 펄스: WP %lu회, EOI %lu회\n", meter.wp_count, meter.eoi_count);
    Serial.println("=================================");
    last_debug_time = current_time;
  }
  
  delay(50);  // 50ms 간격
}

// 🌐 웹 인터페이스
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <title>ESP32 통합 시뮬레이터 v1.0</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f0f0; }
        .container { max-width: 1200px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        .header { text-align: center; color: #333; margin-bottom: 30px; border-bottom: 3px solid #27ae60; padding-bottom: 20px; }
        .section { margin-bottom: 30px; padding: 20px; background: #f8f9fa; border-radius: 8px; }
        .section h2 { margin: 0 0 20px 0; color: #333; border-bottom: 2px solid #27ae60; padding-bottom: 10px; }
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 20px; margin-bottom: 20px; }
        .card { background: white; padding: 15px; border-radius: 8px; border-left: 4px solid #27ae60; }
        .card.inverter { border-left-color: #3498db; }
        .card.meter { border-left-color: #e74c3c; }
        .card.rtu { border-left-color: #f39c12; }
        .card h3 { margin: 0 0 10px 0; color: #333; font-size: 16px; }
        .value { font-size: 20px; font-weight: bold; color: #27ae60; margin: 5px 0; }
        .value.large { font-size: 24px; }
        .status-on { color: #27ae60; }
        .status-off { color: #e74c3c; }
        .controls { display: flex; flex-wrap: wrap; gap: 10px; margin-top: 15px; }
        .button { background: #27ae60; color: white; border: none; padding: 8px 15px; border-radius: 5px; cursor: pointer; font-size: 14px; }
        .button:hover { background: #229954; }
        .button.danger { background: #e74c3c; }
        .button.danger:hover { background: #c0392b; }
        .button.secondary { background: #6c757d; }
        .button.secondary:hover { background: #545b62; }
        .input-group { display: flex; align-items: center; gap: 10px; margin: 10px 0; }
        .input-group input { padding: 5px 10px; border: 1px solid #ddd; border-radius: 3px; }
        .log-section { background: #2c3e50; color: #ecf0f1; padding: 20px; border-radius: 8px; height: 150px; overflow-y: auto; font-family: monospace; font-size: 12px; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🏭 ESP32 통합 시뮬레이터 v1.0</h1>
            <p>인버터 시뮬레이터 + 모의계량기 + RTU 제어 통합</p>
            <p style="color: #666;">내부 직접 통신 | CT비율:6, PT비율:120, WP계수:0.2</p>
        </div>
        
        <!-- 인버터 시뮬레이터 섹션 -->
        <div class="section">
            <h2>🏭 인버터 시뮬레이터</h2>
            <div class="grid">
                <div class="card inverter">
                    <h3>⚡ 전원 및 제어</h3>
                    <div>전원 상태: <span id="power-status" class="value">OFF</span></div>
                    <div>제어 모드: <span id="control-mode" class="value">-</span></div>
                    <div>발전제한율: <span id="limit" class="value">100</span>% (<span id="limit-power">10000</span>W)</div>
                    <div class="controls">
                        <button class="button" onclick="togglePower()">전원 ON/OFF</button>
                        <button class="button secondary" onclick="setPControl()">P제어</button>
                        <button class="button secondary" onclick="setMPPTControl()">MPPT제어</button>
                    </div>
                </div>
                
                <div class="card inverter">
                    <h3>📊 출력 상태</h3>
                    <div>정격전력: <span id="nominal-power-display" class="value">10000</span> W</div>
                    <div>유효전력: <span id="inv-power-w" class="value large">0</span> W</div>
                    <div>무효전력: <span id="inv-power-q" class="value">0</span> var</div>
                    <div>전압: <span id="inv-voltage" class="value">220</span> V</div>
                    <div>전류: <span id="inv-current" class="value">0.0</span> A</div>
                </div>
                
                <div class="card inverter">
                    <h3>🎛️ 설정</h3>
                    <div class="input-group">
                        <label>정격전력(W):</label>
                        <input type="number" id="nominal-power" value="10000" min="1000" max="1000000">
                        <button class="button" onclick="updateNominalPower()">적용</button>
                    </div>
                    <div class="input-group">
                        <label>설정전력(W):</label>
                        <input type="number" id="set-power" value="1000" min="0" max="100000">
                        <button class="button" onclick="updateSetPower()">적용</button>
                    </div>
                    <div class="input-group">
                        <label>발전제한율(%):</label>
                        <input type="number" id="set-limit" value="100" min="0" max="100">
                        <button class="button" onclick="updateLimit()">적용</button>
                    </div>
                </div>
                
                <div class="card inverter">
                    <h3>🌞 환경 조건</h3>
                    <div>일사량: <span id="irradiance" class="value">800</span> W/m²</div>
                    <div>온도: <span id="temperature" class="value">25</span> °C</div>
                    <div>위상각: <span id="phase" class="value">0</span>°</div>
                    <div style="font-size: 12px; color: #666;">자동 변화</div>
                </div>
            </div>
        </div>
        
        <!-- 모의계량기 섹션 -->
        <div class="section">
            <h2>🔋 모의계량기</h2>
            <div class="grid">
                <div class="card meter">
                    <h3>📡 1차측 데이터 (원본)</h3>
                    <div>전력: <span id="primary-power" class="value">0</span> W</div>
                    <div>전압: <span id="primary-voltage" class="value">0</span> V</div>
                    <div>전류: <span id="primary-current" class="value">0</span> A</div>
                    <div style="font-size: 12px; color: #666;">인버터 직접 수신</div>
                </div>
                
                <div class="card meter">
                    <h3>🔄 2차측 변환값</h3>
                    <div>전력: <span id="secondary-power" class="value">0</span> W</div>
                    <div>전압: <span id="secondary-voltage" class="value">0</span> V</div>
                    <div>전류: <span id="secondary-current" class="value">0</span> A</div>
                    <div style="font-size: 12px; color: #666;">CT÷6, PT÷120</div>
                </div>
                
                <div class="card meter">
                    <h3>📊 계량기 측정값</h3>
                    <div>전력: <span id="measured-power" class="value">0</span> W</div>
                    <div>전압: <span id="measured-voltage" class="value">0</span> V</div>
                    <div>전류: <span id="measured-current" class="value">0</span> A</div>
                    <div style="font-size: 12px; color: #666;">2차측×CT×PT (펄스 생성용)</div>
                </div>
                
                <div class="card meter">
                    <h3>⚡ 발전량 및 펄스</h3>
                    <div>누적 발전량: <span id="energy-wh" class="value large">0</span> Wh</div>
                    <div>WP 펄스: <span id="wp-count" class="value">0</span>회 (144Wh당 1펄스)</div>
                    <div>EOI 신호: <span id="eoi-count" class="value">0</span>회</div>
                    <div style="font-size: 12px; color: #666;">1차측 실제 발전량 누적</div>
                    <div class="controls">
                        <button class="button secondary" onclick="manualWP()">수동 WP</button>
                        <button class="button secondary" onclick="manualEOI()">수동 EOI</button>
                    </div>
                                            <div class="controls" style="margin-top: 8px;">
                            <button class="button secondary" onclick="bulkWP(10)">WP 10회</button>
                            <button class="button secondary" onclick="bulkWP(50)">WP 50회</button>
                            <button class="button secondary" onclick="bulkWP(100)">WP 100회</button>
                        </div>
                        <div class="controls" style="margin-top: 8px;">
                            <button class="button secondary" onclick="testWP('fast')">🚀 빠른 10회</button>
                            <button class="button secondary" onclick="testWP('intervals')">🧪 간격 테스트</button>
                        </div>
                        <div class="controls" style="margin-top: 8px;">
                            <button class="button danger" onclick="resetEnergyCounters()">발전량 초기화</button>
                        </div>
                </div>
            </div>
        </div>
        
        <!-- RTU 연결 섹션 -->
        <div class="section">
            <h2>🎯 RTU 연결</h2>
            <div class="grid">
                <div class="card rtu">
                    <h3>📋 펄스 출력</h3>
                    <div>WP 출력핀: GPIO25 (오픈컬렉터)</div>
                    <div>EOI 출력핀: GPIO26 (오픈컬렉터)</div>
                    <div>펄스당 전력량: 144 Wh = 0.144 kWh</div>
                    <div>마지막 WP: <span id="last-wp">-</span></div>
                </div>
                
                <div class="card rtu">
                    <h3>🌐 시스템 상태</h3>
                    <div>WiFi: <span id="wifi-status" class="value">확인중</span></div>
                    <div>ESP32 IP: <span id="esp32-ip">-</span></div>
                    <div>가동시간: <span id="uptime">0초</span></div>
                    <div class="controls">
                        <button class="button danger" onclick="resetCounters()">카운터 초기화</button>
                    </div>
                </div>
            </div>
        </div>
        
        <!-- 실시간 로그 -->
        <div class="section">
            <h2>📝 실시간 로그</h2>
            <div class="log-section" id="log-content">
                <div>시스템 초기화 중...</div>
            </div>
        </div>
    </div>

    <script>
        // ESP32 IP 주소 표시
        document.getElementById('esp32-ip').textContent = window.location.hostname;
        
        let lastLogTime = 0;
        
        function updateStatus() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    // 인버터 상태 업데이트
                    document.getElementById('power-status').textContent = data.inverter.power_on ? 'ON' : 'OFF';
                    document.getElementById('power-status').className = 'value ' + (data.inverter.power_on ? 'status-on' : 'status-off');
                    
                    let mode = data.inverter.p_control ? 'P제어' : (data.inverter.mppt_control ? 'MPPT제어' : '정지');
                    document.getElementById('control-mode').textContent = mode;
                    
                    document.getElementById('limit').textContent = data.inverter.limit.toFixed(1);
                    document.getElementById('nominal-power-display').textContent = data.inverter.nominal_power.toFixed(0);
                    
                    // 발전제한율에 따른 최대 전력 계산
                    const limitPower = data.inverter.nominal_power * (data.inverter.limit / 100.0);
                    document.getElementById('limit-power').textContent = limitPower.toFixed(0);
                    
                    // 설정 입력 필드 업데이트 (포커스 중이 아닐 때만)
                    const nominalInput = document.getElementById('nominal-power');
                    const setPowerInput = document.getElementById('set-power');
                    const setLimitInput = document.getElementById('set-limit');
                    
                    if (document.activeElement !== nominalInput) {
                        nominalInput.value = data.inverter.nominal_power.toFixed(0);
                    }
                    if (document.activeElement !== setPowerInput) {
                        setPowerInput.value = data.inverter.set_power.toFixed(0);
                    }
                    if (document.activeElement !== setLimitInput) {
                        setLimitInput.value = data.inverter.limit.toFixed(1);
                    }
                    document.getElementById('inv-power-w').textContent = data.inverter.power_w.toFixed(1);
                    document.getElementById('inv-power-q').textContent = data.inverter.power_q.toFixed(1);
                    document.getElementById('inv-voltage').textContent = data.inverter.voltage.toFixed(1);
                    document.getElementById('inv-current').textContent = data.inverter.current.toFixed(2);
                    document.getElementById('irradiance').textContent = data.inverter.irradiance.toFixed(0);
                    document.getElementById('temperature').textContent = data.inverter.temperature.toFixed(1);
                    document.getElementById('phase').textContent = data.inverter.phase_deg.toFixed(1);
                    
                    // 계량기 상태 업데이트
                    document.getElementById('primary-power').textContent = data.meter.primary_power_w.toFixed(1);
                    document.getElementById('primary-voltage').textContent = data.meter.primary_voltage.toFixed(1);
                    document.getElementById('primary-current').textContent = data.meter.primary_current.toFixed(2);
                    document.getElementById('secondary-power').textContent = data.meter.secondary_power_w.toFixed(1);
                    document.getElementById('secondary-voltage').textContent = data.meter.secondary_voltage.toFixed(1);
                    document.getElementById('secondary-current').textContent = data.meter.secondary_current.toFixed(2);
                    document.getElementById('measured-power').textContent = data.meter.measured_power_w.toFixed(1);
                    document.getElementById('measured-voltage').textContent = data.meter.measured_voltage.toFixed(1);
                    document.getElementById('measured-current').textContent = data.meter.measured_current.toFixed(2);
                    document.getElementById('energy-wh').textContent = data.meter.energy_wh.toFixed(3);
                    document.getElementById('wp-count').textContent = data.meter.wp_count;
                    document.getElementById('eoi-count').textContent = data.meter.eoi_count;
                    
                    // RTU 상태 업데이트
                    document.getElementById('wifi-status').textContent = data.wifi_connected ? '연결됨' : '연결끊김';
                    document.getElementById('wifi-status').className = 'value ' + (data.wifi_connected ? 'status-on' : 'status-off');
                    document.getElementById('last-wp').textContent = data.meter.last_wp_time > 0 ? 
                        Math.floor((Date.now() - data.meter.last_wp_time) / 1000) + '초 전' : '-';
                    
                    // 가동시간 업데이트
                    const uptimeSeconds = Math.floor(data.uptime / 1000);
                    const hours = Math.floor(uptimeSeconds / 3600);
                    const minutes = Math.floor((uptimeSeconds % 3600) / 60);
                    const seconds = uptimeSeconds % 60;
                    document.getElementById('uptime').textContent = 
                        hours + '시간 ' + minutes + '분 ' + seconds + '초';
                    
                    // 로그 업데이트
                    if (data.inverter.last_log_time > lastLogTime) {
                        addLog(data.inverter.last_log);
                        lastLogTime = data.inverter.last_log_time;
                    }
                })
                .catch(error => {
                    console.error('상태 업데이트 오류:', error);
                    addLog('오류: 상태 업데이트 실패');
                });
        }
        
        function addLog(message) {
            const logContent = document.getElementById('log-content');
            const timestamp = new Date().toLocaleTimeString();
            const logEntry = document.createElement('div');
            logEntry.innerHTML = '[' + timestamp + '] ' + message;
            logContent.appendChild(logEntry);
            logContent.scrollTop = logContent.scrollHeight;
            
            if (logContent.children.length > 50) {
                logContent.removeChild(logContent.firstChild);
            }
        }
        
        // 인버터 제어 함수들
        function togglePower() {
            fetch('/api/inverter_control', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: 'toggle_power'})
            }).then(() => updateStatus());
        }
        
        function setPControl() {
            fetch('/api/inverter_control', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: 'set_p_control'})
            }).then(() => updateStatus());
        }
        
        function setMPPTControl() {
            fetch('/api/inverter_control', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: 'set_mppt_control'})
            }).then(() => updateStatus());
        }
        
        function updateNominalPower() {
            const nominalPower = document.getElementById('nominal-power').value;
            fetch('/api/inverter_control', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: 'set_nominal_power', value: parseFloat(nominalPower)})
            }).then(() => updateStatus());
        }
        
        function updateSetPower() {
            const power = document.getElementById('set-power').value;
            fetch('/api/inverter_control', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: 'set_power', value: parseFloat(power)})
            }).then(() => updateStatus());
        }
        
        function updateLimit() {
            const limit = document.getElementById('set-limit').value;
            fetch('/api/inverter_control', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: 'set_limit', value: parseFloat(limit)})
            }).then(() => updateStatus());
        }
        
        // 계량기 제어 함수들
        function manualWP() {
            fetch('/api/meter_control', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: 'manual_wp'})
            }).then(() => updateStatus());
        }
        
        function manualEOI() {
            fetch('/api/meter_control', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: 'manual_eoi'})
            }).then(() => updateStatus());
        }
        
        function bulkWP(count) {
            if (confirm(`WP 펄스를 ${count}회 연속으로 생성하시겠습니까?\n(약 ${((count-1) * 0.52 + 0.1).toFixed(1)}초 소요)`)) {
                fetch('/api/meter_control', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({action: 'bulk_wp', count: count})
                })
                .then(response => response.json())
                .then(data => {
                    if (data.result === 'ok') {
                        addLog(`연속 WP 펄스 ${count}개 생성 시작`);
                        // 완료까지 시간이 걸리므로 지연 후 업데이트
                        setTimeout(() => updateStatus(), (count-1) * 520 + 100 + 500);
                    } else {
                        addLog('오류: 연속 WP 펄스 생성 실패');
                    }
                })
                .catch(error => {
                    addLog('오류: 연속 WP 펄스 실패');
                    console.error('Error:', error);
                });
            }
        }
        
        function testWP(type) {
            if (type === 'fast') {
                if (confirm('빠른 연속 WP 10개를 생성하시겠습니까?\\n(최소간격, 약 2.6초 소요)')) {
                    fetch('/api/meter_control', {
                        method: 'POST',
                        headers: {'Content-Type': 'application/json'},
                        body: JSON.stringify({action: 'test_wp_fast'})
                    })
                    .then(response => response.json())
                    .then(data => {
                        if (data.result === 'ok') {
                            addLog('빠른 연속 WP 10개 테스트 시작');
                            setTimeout(() => updateStatus(), 3000);
                        } else {
                            addLog('오류: 빠른 WP 테스트 실패');
                        }
                    })
                    .catch(error => {
                        addLog('오류: 빠른 WP 테스트 실패');
                    });
                }
            } else if (type === 'intervals') {
                if (confirm('다양한 간격으로 펄스 테스트를 하시겠습니까?\\n(약 35초 소요)')) {
                    fetch('/api/meter_control', {
                        method: 'POST',
                        headers: {'Content-Type': 'application/json'},
                        body: JSON.stringify({action: 'test_wp_intervals'})
                    })
                    .then(response => response.json())
                    .then(data => {
                        if (data.result === 'ok') {
                            addLog('다양한 간격 펄스 테스트 시작');
                            setTimeout(() => updateStatus(), 36000);
                        } else {
                            addLog('오류: 간격 테스트 실패');
                        }
                    })
                    .catch(error => {
                        addLog('오류: 간격 테스트 실패');
                    });
                }
            }
        }
        
        function resetCounters() {
            if (confirm('모든 카운터를 초기화하시겠습니까?')) {
                fetch('/api/reset', {method: 'POST'})
                    .then(() => {
                        addLog('카운터 초기화 완료');
                        updateStatus();
                    });
            }
        }
        
        function resetEnergyCounters() {
            if (confirm('발전량 및 펄스 카운터를 초기화하시겠습니까?')) {
                fetch('/api/reset', {method: 'POST'})
                    .then(response => response.json())
                    .then(data => {
                        if (data.result === 'ok') {
                            addLog('발전량 및 펄스 초기화 완료');
                            updateStatus();
                        }
                    })
                    .catch(error => {
                        addLog('오류: 초기화 실패');
                        console.error('Error:', error);
                    });
            }
        }
        
        // 1초마다 상태 업데이트
        setInterval(updateStatus, 1000);
        updateStatus();
    </script>
</body>
</html>
  )rawliteral";
  
  server.send(200, "text/html", html);
}

// 📊 상태 API
void handleStatus() {
  // 시간 계산
  unsigned long current_time = millis();
  unsigned long wp_time_ago = (meter.last_wp_time > 0) ? current_time - meter.last_wp_time : 0;
  unsigned long eoi_time_ago = (meter.last_eoi_time > 0) ? current_time - meter.last_eoi_time : 0;
  
  DynamicJsonDocument doc(2048);
  
  // 인버터 데이터
  JsonObject inv = doc.createNestedObject("inverter");
  inv["power_on"] = inverter.power_on;
  inv["p_control"] = inverter.p_control;
  inv["mppt_control"] = inverter.mppt_control;
  inv["limit"] = inverter.limit;
  inv["set_power"] = inverter.set_power;
  inv["nominal_power"] = inverter.nominal_power;
  inv["voltage"] = inverter.voltage;
  inv["irradiance"] = inverter.irradiance;
  inv["temperature"] = inverter.temperature;
  inv["power_w"] = inverter.power_w;
  inv["power_q"] = inverter.power_q;
  inv["current"] = inverter.current;
  inv["phase_deg"] = inverter.phase_deg;
  inv["last_log"] = inverter.last_log;
  inv["last_log_time"] = inverter.last_log_time;
  
  // 계량기 데이터
  JsonObject met = doc.createNestedObject("meter");
  met["primary_power_w"] = meter.primary_power_w;
  met["primary_power_q"] = meter.primary_power_q;
  met["primary_voltage"] = meter.primary_voltage;
  met["primary_current"] = meter.primary_current;
  met["secondary_voltage"] = meter.secondary_voltage;
  met["secondary_current"] = meter.secondary_current;
  met["secondary_power_w"] = meter.secondary_power_w;
  met["measured_voltage"] = meter.measured_voltage;
  met["measured_current"] = meter.measured_current;
  met["measured_power_w"] = meter.measured_power_w;
  met["energy_wh"] = meter.energy_wh;
  met["wp_count"] = meter.wp_count;
  met["eoi_count"] = meter.eoi_count;
  met["last_wp_time"] = wp_time_ago;
  met["last_eoi_time"] = eoi_time_ago;
  met["last_signal"] = meter.last_signal;
  
  // 시스템 상태
  doc["wifi_connected"] = wifi_connected;
  doc["uptime"] = uptime;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

// 🏭 인버터 제어 API
void handleInverterControl() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"result\":\"error\",\"msg\":\"No data\"}");
    return;
  }
  
  String body = server.arg("plain");
  DynamicJsonDocument doc(1024);
  
  DeserializationError error = deserializeJson(doc, body);
  if (error) {
    server.send(400, "application/json", "{\"result\":\"error\",\"msg\":\"Invalid JSON\"}");
    return;
  }
  
  String action = doc["action"];
  
  if (action == "toggle_power") {
    inverter.power_on = !inverter.power_on;
    addLog(inverter.power_on ? "인버터 전원 ON" : "인버터 전원 OFF");
    
  } else if (action == "set_p_control") {
    inverter.p_control = true;
    inverter.mppt_control = false;
    addLog("P제어 모드 활성화");
    
  } else if (action == "set_mppt_control") {
    inverter.p_control = false;
    inverter.mppt_control = true;
    addLog("MPPT제어 모드 활성화");
    
  } else if (action == "set_nominal_power") {
    float nominal = doc["value"];
    inverter.nominal_power = max(1000.0f, min(nominal, 1000000.0f));  // 1kW ~ 1MW
    addLog("정격전력: " + String(inverter.nominal_power) + "W");
    
  } else if (action == "set_power") {
    float power = doc["value"];
    inverter.set_power = max(0.0f, min(power, inverter.nominal_power));
    addLog("설정전력: " + String(inverter.set_power) + "W");
    
  } else if (action == "set_limit") {
    float limit = doc["value"];
    inverter.limit = max(0.0f, min(limit, 100.0f));
    addLog("발전제한율: " + String(inverter.limit) + "%");
    
  } else {
    server.send(400, "application/json", "{\"result\":\"error\",\"msg\":\"Unknown action\"}");
    return;
  }
  
  server.send(200, "application/json", "{\"result\":\"ok\"}");
}

// 🔋 계량기 제어 API
void handleMeterControl() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"result\":\"error\",\"msg\":\"No data\"}");
    return;
  }
  
  String body = server.arg("plain");
  DynamicJsonDocument doc(512);
  
  DeserializationError error = deserializeJson(doc, body);
  if (error) {
    server.send(400, "application/json", "{\"result\":\"error\",\"msg\":\"Invalid JSON\"}");
    return;
  }
  
  String action = doc["action"];
  
  if (action == "manual_wp") {
    generateWPPulse();
    addLog("수동 WP 펄스 생성");
    
  } else if (action == "manual_eoi") {
    generateEOISignal();
    addLog("수동 EOI 신호 생성");
    
  } else if (action == "bulk_wp") {
    int count = doc["count"];
    if (count > 0 && count <= 1000) {
      addLog("연속 WP 펄스 " + String(count) + "개 생성 시작");
      
      // 연속 펄스 생성 (논블로킹)
              for (int i = 0; i < count; i++) {
          generateWPPulse();
          if (i < count - 1) {  // 마지막이 아니면 대기
            delay(420);  // 펄스 간 420ms 간격 (RTU 디바운싱 400ms + 여유 20ms)
          }
        }
      
      addLog("연속 WP 펄스 " + String(count) + "개 생성 완료");
    } else {
      server.send(400, "application/json", "{\"result\":\"error\",\"msg\":\"Invalid count (1-1000)\"}");
      return;
    }
    
  } else if (action == "test_wp_fast") {
    addLog("빠른 연속 WP 10개 테스트 시작 (최소간격)");
    for (int i = 0; i < 10; i++) {
      generateWPPulse();
      if (i < 9) delay(150);  // 최소 간격
    }
    addLog("빠른 연속 WP 10개 테스트 완료");
    
  } else if (action == "test_wp_intervals") {
    addLog("다양한 간격으로 펄스 테스트 시작");
    int intervals[] = {100, 150, 200, 300, 400, 500};
    for (int j = 0; j < 6; j++) {
      addLog("간격 " + String(intervals[j]) + "ms로 5개 테스트");
      for (int i = 0; i < 5; i++) {
        generateWPPulse();
        if (i < 4) delay(intervals[j]);
      }
      delay(1000);  // 각 테스트 사이 1초 대기
    }
    addLog("다양한 간격 테스트 완료");
    
  } else {
    server.send(400, "application/json", "{\"result\":\"error\",\"msg\":\"Unknown action\"}");
    return;
  }
  
  server.send(200, "application/json", "{\"result\":\"ok\"}");
}

// 🔄 초기화 API
void handleReset() {
  meter.wp_count = 0;
  meter.eoi_count = 0;
  meter.energy_wh = 0;
  meter.last_energy_wh = 0;
  
  addLog("카운터 초기화 완료");
  
  server.send(200, "application/json", "{\"result\":\"ok\",\"msg\":\"카운터 초기화 완료\"}");
}