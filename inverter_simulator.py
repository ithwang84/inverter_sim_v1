#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32 통합 시뮬레이터 v1.0 - Python 버전
인버터 시뮬레이터 + 모의계량기 + RTU 제어 통합
PC 환경에서 실행 가능한 버전
2025.10.15 - Python 변환 버전
"""

import time
import json
import math
import random
import threading
from datetime import datetime, timedelta
from flask import Flask, render_template_string, request, jsonify
import logging

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Flask 앱 초기화
app = Flask(__name__)

# RTU 오픈컬렉터 출력 핀 정의 (시뮬레이션용)
RTU_WP_PIN = 25      # RTU로 WP 펄스 출력 (오픈컬렉터)
RTU_EOI_PIN = 26     # RTU로 EOI 펄스 출력 (오픈컬렉터)
WP_PULSE_WIDTH = 100   # ms
EOI_PULSE_WIDTH = 100  # ms

# ⚡ CT/PT 설정 (RTU와 동일하게!)
CTR = 6              # CT 비율 (전류 변성기)
PTR = 120            # PT 비율 (전압 변성기)
WPP = 0.2            # WP 계수 (Watt-Pulse)

# 🔄 RTU 펄스 계산 공식과 맞춤
# RTU: Total_p = pulse_count * CTR * PTR * WPP * 0.001
# 펄스 1개당 전력량 = 6 * 120 * 0.2 * 0.001 = 0.144 kWh = 144 Wh
WP_PULSE_WH = 144.0  # WP 펄스당 Wh (RTU 계산식과 일치)
EOI_INTERVAL_MS = 15 * 60 * 1000  # EOI 신호 간격: 15분

class InverterData:
    """인버터 시뮬레이터 데이터 클래스"""
    def __init__(self):
        # 제어 상태
        self.power_on = False
        self.p_control = False      # P제어 모드
        self.mppt_control = True    # MPPT제어 모드 (기본값)
        self.limit = 100.0         # 발전제한율 (%)
        
        # 설정값
        self.set_power = 1000.0    # 설정 전력 (W)
        self.nominal_power = 10000.0 # 정격 출력 (W)
        self.voltage = 220.0       # 전압 (V)
        
        # 환경 조건
        self.irradiance = 800.0    # 일사량 (W/m²)
        self.temperature = 25.0    # 온도 (°C)
        
        # 출력값 (1차측 실제값)
        self.power_w = 0.0         # 유효전력 (W)
        self.power_q = 0.0         # 무효전력 (var)
        self.current = 0.0         # 전류 (A)
        self.phase_deg = 0.0       # 위상각 (도)
        
        # 로그
        self.last_log = ""
        self.last_log_time = 0

class MeterData:
    """모의계량기 데이터 클래스"""
    def __init__(self):
        # 📡 1차측 데이터 (인버터에서 직접 수신)
        self.primary_power_w = 0.0
        self.primary_power_q = 0.0
        self.primary_voltage = 0.0
        self.primary_current = 0.0
        
        # 🔄 2차측 변환값 (CT/PT 적용 후)
        self.secondary_voltage = 0.0
        self.secondary_current = 0.0
        self.secondary_power_w = 0.0
        
        # 📊 계량기 측정값 (2차측 → 1차측 역산)
        self.measured_power_w = 0.0
        self.measured_voltage = 0.0
        self.measured_current = 0.0
        
        # 발전량 및 펄스
        self.energy_wh = 0.0
        self.last_energy_wh = 0.0
        self.wp_count = 0
        self.eoi_count = 0
        
        # 시간 정보
        self.last_wp_time = 0
        self.last_eoi_time = 0
        self.last_signal_time = 0
        self.last_signal = ""

class InverterSimulator:
    """ESP32 통합 시뮬레이터 메인 클래스"""
    
    def __init__(self):
        self.inverter = InverterData()
        self.meter = MeterData()
        self.start_time = time.time()
        self.last_eoi_check = 0
        self.last_env_update = 0
        self.last_sim_time = 0
        self.last_debug_time = 0
        
        # 시뮬레이션 스레드 시작
        self.simulation_thread = threading.Thread(target=self.simulation_loop, daemon=True)
        self.simulation_thread.start()
        
        logger.info("🏭 ESP32 통합 시뮬레이터 v1.0 (Python 버전)")
        logger.info("📊 인버터 시뮬레이터 + 모의계량기 + RTU 제어")
        logger.info("🔄 내부 직접 통신으로 안정성 극대화")
        logger.info(f"⚡ CT:{CTR}, PT:{PTR}, WP:{WPP}")
        logger.info("🌐 웹서버 시작: http://localhost:5000")
        logger.info("🎯 준비 완료! 인버터 시뮬레이션 시작...")
        
        self.add_log("시스템 시작")
    
    def get_uptime(self):
        """시스템 가동시간 반환 (ms)"""
        return int((time.time() - self.start_time) * 1000)
    
    def add_log(self, message):
        """로그 추가"""
        uptime_sec = int((time.time() - self.start_time))
        self.inverter.last_log = f"[{uptime_sec}s] {message}"
        self.inverter.last_log_time = int(time.time() * 1000)
        logger.info(f"📝 {self.inverter.last_log}")
    
    def simulate_inverter(self):
        """인버터 시뮬레이션"""
        if not self.inverter.power_on:
            self.inverter.power_w = 0.0
            self.inverter.current = 0.0
            self.inverter.power_q = 0.0
            return
        
        current_time = time.time() * 1000
        
        # 환경 조건 시뮬레이션 (실제 환경 변화를 모사)
        if current_time - self.last_env_update > 1000:  # 1초마다 환경 변화
            self.inverter.irradiance = random.uniform(600, 1000)  # 600~1000 W/m²
            self.inverter.temperature = random.uniform(20, 35)    # 20~35°C
            self.last_env_update = current_time
        
        # 위상각 계산
        self.inverter.phase_deg = random.uniform(0, 30)  # 0~30도
        phase_rad = math.radians(self.inverter.phase_deg)
        
        # 전류 계산 (정격용량 기반)
        base_current = self.inverter.nominal_power / self.inverter.voltage  # 정격 전류
        irradiance_factor = self.inverter.irradiance / 1000.0
        temp_factor = 1.0 + (self.inverter.temperature - 25.0) * 0.002
        actual_current = base_current * irradiance_factor * temp_factor
        
        self.inverter.current = actual_current
        
        # 제어 모드별 전력 계산
        if self.inverter.p_control:
            # 🎯 P제어: 설정값대로 강제 출력 (환경 조건 무시)
            max_power = self.inverter.nominal_power * (self.inverter.limit / 100.0)
            self.inverter.power_w = min(self.inverter.set_power, max_power)
            
            # P제어 시 전류도 설정 전력에 맞게 재계산
            if self.inverter.voltage > 0:
                calculated_current = self.inverter.power_w / (self.inverter.voltage * math.cos(phase_rad))
                self.inverter.current = calculated_current
            
        elif self.inverter.mppt_control:
            # 🌞 MPPT제어: 환경 조건에 따른 자동 출력
            base_power = self.inverter.voltage * self.inverter.current * math.cos(phase_rad)
            self.inverter.power_w = base_power * (1.0 + 0.05 * math.sin(current_time / 1000.0))  # 약간의 변동
            
            # 발전제한율 적용
            max_power = self.inverter.nominal_power * (self.inverter.limit / 100.0)
            self.inverter.power_w = min(self.inverter.power_w, max_power)
            
        else:
            self.inverter.power_w = 0.0
        
        # 무효전력 계산
        self.inverter.power_q = self.inverter.voltage * self.inverter.current * math.sin(phase_rad)
    
    def transfer_data_to_meter(self):
        """인버터 → 계량기 데이터 전송 (내부 직접 통신)"""
        # 직접 메모리 복사 (WiFi 통신 없음!)
        self.meter.primary_power_w = self.inverter.power_w
        self.meter.primary_power_q = self.inverter.power_q
        self.meter.primary_voltage = self.inverter.voltage
        self.meter.primary_current = self.inverter.current
        
        self.meter.last_signal_time = int(time.time() * 1000)
        self.meter.last_signal = "내부 데이터 전송"
        
        # 2차측 값 계산
        self.calculate_secondary_values()
        self.calculate_measured_values()
    
    def calculate_secondary_values(self):
        """2차측 값 계산 (CT/PT 적용)"""
        self.meter.secondary_voltage = self.meter.primary_voltage / PTR  # 전압: 1차측 ÷ PT비율
        self.meter.secondary_current = self.meter.primary_current / CTR  # 전류: 1차측 ÷ CT비율
        self.meter.secondary_power_w = self.meter.secondary_voltage * self.meter.secondary_current
    
    def calculate_measured_values(self):
        """계량기 측정값 계산 (2차측 → 1차측 역산)"""
        self.meter.measured_voltage = self.meter.secondary_voltage * PTR  # 전압: 2차측 × PT비율
        self.meter.measured_current = self.meter.secondary_current * CTR  # 전류: 2차측 × CT비율
        self.meter.measured_power_w = self.meter.measured_voltage * self.meter.measured_current
    
    def generate_wp_pulse(self):
        """WP 펄스 생성 및 출력"""
        self.meter.wp_count += 1
        self.meter.last_wp_time = int(time.time() * 1000)
        self.meter.last_signal = "WP 펄스 생성"
        self.meter.last_signal_time = int(time.time() * 1000)
        
        logger.info(f"⚡ WP 펄스 생성 (#{self.meter.wp_count})")
        logger.info(f"  펄스당: {WP_PULSE_WH:.1f} Wh = 0.144kWh (CT:{CTR} × PT:{PTR} × WP:{WPP})")
        logger.info(f"  누적 발전량: {self.meter.energy_wh:.3f} Wh (1차측 실제)")
        logger.info(f"  현재 출력: {self.inverter.power_w:.1f} W (인버터 실제)")
        
        # RTU로 WP 펄스 (오픈컬렉터) - 시뮬레이션
        logger.info(f"🔌 RTU WP 펄스 출력: GPIO{RTU_WP_PIN} LOW → HIGH ({WP_PULSE_WIDTH}ms)")
    
    def generate_eoi_signal(self):
        """EOI 신호 생성 및 출력"""
        self.meter.eoi_count += 1
        self.meter.last_eoi_time = int(time.time() * 1000)
        self.meter.last_signal = "EOI 신호 생성"
        self.meter.last_signal_time = int(time.time() * 1000)
        
        logger.info(f"📋 EOI 신호 생성 (#{self.meter.eoi_count}): 15분 경과")
        
        # RTU로 EOI 펄스 (오픈컬렉터) - 시뮬레이션
        logger.info(f"🔌 RTU EOI 펄스 출력: GPIO{RTU_EOI_PIN} LOW → HIGH ({EOI_PULSE_WIDTH}ms)")
    
    def simulation_loop(self):
        """메인 시뮬레이션 루프"""
        while True:
            current_time = time.time() * 1000
            
            # 🏭 인버터 시뮬레이션 및 발전량 계산 (100ms마다)
            if current_time - self.last_sim_time >= 100:
                self.simulate_inverter()
                self.transfer_data_to_meter()  # 내부 직접 전송
                
                # 🔋 발전량 계산 및 WP 펄스 생성 (1차측 실제 발전량으로 계산)
                if self.inverter.power_w > 0:
                    # ⚡ 1차측 실제 발전량으로 누적 (인버터 실제 출력)
                    # 100ms당 발전량 계산 (Wh)
                    energy_per_100ms = (self.inverter.power_w / 3600.0) * 0.1  # 100ms = 0.1초
                    self.meter.energy_wh += energy_per_100ms
                    
                    # WP 펄스 생성 조건 확인 (144Wh = 0.144kWh마다 1펄스)
                    if self.meter.energy_wh - self.meter.last_energy_wh >= WP_PULSE_WH:
                        self.generate_wp_pulse()
                        self.meter.last_energy_wh += WP_PULSE_WH
                
                self.last_sim_time = current_time
            
            # 📋 EOI 신호 생성 (15분마다)
            if current_time >= self.last_eoi_check:
                if current_time - self.last_eoi_check >= EOI_INTERVAL_MS:
                    self.generate_eoi_signal()
                    self.last_eoi_check = current_time
            else:
                self.last_eoi_check = current_time  # 오버플로우 처리
            
            # 📊 상태 디버깅 (10초마다)
            if current_time - self.last_debug_time >= 10000:
                logger.info("======= 통합 시뮬레이터 상태 =======")
                mode = "P제어" if self.inverter.p_control else ("MPPT" if self.inverter.mppt_control else "정지")
                logger.info(f"🏭 인버터: {'ON' if self.inverter.power_on else 'OFF'}, 모드: {mode}")
                logger.info(f"   출력: {self.inverter.power_w:.1f}W, 전압: {self.inverter.voltage:.1f}V, 전류: {self.inverter.current:.2f}A")
                logger.info(f"🔋 계량기: 측정 {self.meter.measured_power_w:.1f}W, 누적 {self.meter.energy_wh:.3f}Wh (1차측 실제)")
                logger.info(f"⚡ 펄스: WP {self.meter.wp_count}회, EOI {self.meter.eoi_count}회")
                logger.info(f"🔧 디버그: power_on={self.inverter.power_on}, p_control={self.inverter.p_control}, mppt_control={self.inverter.mppt_control}")
                logger.info("=================================")
                self.last_debug_time = current_time
            
            time.sleep(0.05)  # 50ms 간격

# 전역 시뮬레이터 인스턴스
simulator = InverterSimulator()

# Flask 라우트들
@app.route('/')
def handle_root():
    """웹 인터페이스 메인 페이지"""
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/status')
def handle_status():
    """상태 API"""
    current_time = int(time.time() * 1000)
    wp_time_ago = (simulator.meter.last_wp_time - current_time) if simulator.meter.last_wp_time > 0 else 0
    eoi_time_ago = (simulator.meter.last_eoi_time - current_time) if simulator.meter.last_eoi_time > 0 else 0
    
    status_data = {
        "inverter": {
            "power_on": simulator.inverter.power_on,
            "p_control": simulator.inverter.p_control,
            "mppt_control": simulator.inverter.mppt_control,
            "limit": simulator.inverter.limit,
            "set_power": simulator.inverter.set_power,
            "nominal_power": simulator.inverter.nominal_power,
            "voltage": simulator.inverter.voltage,
            "irradiance": simulator.inverter.irradiance,
            "temperature": simulator.inverter.temperature,
            "power_w": simulator.inverter.power_w,
            "power_q": simulator.inverter.power_q,
            "current": simulator.inverter.current,
            "phase_deg": simulator.inverter.phase_deg,
            "last_log": simulator.inverter.last_log,
            "last_log_time": simulator.inverter.last_log_time
        },
        "meter": {
            "primary_power_w": simulator.meter.primary_power_w,
            "primary_power_q": simulator.meter.primary_power_q,
            "primary_voltage": simulator.meter.primary_voltage,
            "primary_current": simulator.meter.primary_current,
            "secondary_voltage": simulator.meter.secondary_voltage,
            "secondary_current": simulator.meter.secondary_current,
            "secondary_power_w": simulator.meter.secondary_power_w,
            "measured_voltage": simulator.meter.measured_voltage,
            "measured_current": simulator.meter.measured_current,
            "measured_power_w": simulator.meter.measured_power_w,
            "energy_wh": simulator.meter.energy_wh,
            "wp_count": simulator.meter.wp_count,
            "eoi_count": simulator.meter.eoi_count,
            "last_wp_time": wp_time_ago,
            "last_eoi_time": eoi_time_ago,
            "last_signal": simulator.meter.last_signal
        },
        "wifi_connected": True,  # PC 환경에서는 항상 연결됨
        "uptime": simulator.get_uptime()
    }
    
    return jsonify(status_data)

@app.route('/api/inverter_control', methods=['POST'])
def handle_inverter_control():
    """인버터 제어 API"""
    try:
        data = request.get_json()
        action = data.get('action')
        
        if action == 'toggle_power':
            old_state = simulator.inverter.power_on
            simulator.inverter.power_on = not simulator.inverter.power_on
            new_state = simulator.inverter.power_on
            simulator.add_log(f"인버터 전원 {'ON' if new_state else 'OFF'} (이전: {'ON' if old_state else 'OFF'})")
            logger.info(f"🔌 인버터 전원 상태 변경: {old_state} → {new_state}")
            
        elif action == 'set_p_control':
            simulator.inverter.p_control = True
            simulator.inverter.mppt_control = False
            simulator.add_log("P제어 모드 활성화")
            
        elif action == 'set_mppt_control':
            simulator.inverter.p_control = False
            simulator.inverter.mppt_control = True
            simulator.add_log("MPPT제어 모드 활성화")
            
        elif action == 'set_nominal_power':
            nominal = data.get('value', 10000)
            simulator.inverter.nominal_power = max(1000.0, min(nominal, 1000000.0))  # 1kW ~ 1MW
            simulator.add_log(f"정격전력: {simulator.inverter.nominal_power:.0f}W")
            
        elif action == 'set_power':
            power = data.get('value', 1000)
            simulator.inverter.set_power = max(0.0, min(power, simulator.inverter.nominal_power))
            simulator.add_log(f"설정전력: {simulator.inverter.set_power:.0f}W")
            
        elif action == 'set_limit':
            limit = data.get('value', 100)
            simulator.inverter.limit = max(0.0, min(limit, 100.0))
            simulator.add_log(f"발전제한율: {simulator.inverter.limit:.1f}%")
            
        else:
            return jsonify({"result": "error", "msg": "Unknown action"}), 400
        
        return jsonify({"result": "ok"})
        
    except Exception as e:
        return jsonify({"result": "error", "msg": str(e)}), 400

@app.route('/api/meter_control', methods=['POST'])
def handle_meter_control():
    """계량기 제어 API"""
    try:
        data = request.get_json()
        action = data.get('action')
        
        if action == 'manual_wp':
            simulator.generate_wp_pulse()
            simulator.add_log("수동 WP 펄스 생성")
            
        elif action == 'manual_eoi':
            simulator.generate_eoi_signal()
            simulator.add_log("수동 EOI 신호 생성")
            
        elif action == 'bulk_wp':
            count = data.get('count', 1)
            if 1 <= count <= 1000:
                simulator.add_log(f"연속 WP 펄스 {count}개 생성 시작")
                
                # 연속 펄스 생성
                for i in range(count):
                    simulator.generate_wp_pulse()
                    if i < count - 1:  # 마지막이 아니면 대기
                        time.sleep(0.42)  # 펄스 간 420ms 간격
                
                simulator.add_log(f"연속 WP 펄스 {count}개 생성 완료")
            else:
                return jsonify({"result": "error", "msg": "Invalid count (1-1000)"}), 400
            
        elif action == 'test_wp_fast':
            simulator.add_log("빠른 연속 WP 10개 테스트 시작 (최소간격)")
            for i in range(10):
                simulator.generate_wp_pulse()
                if i < 9:
                    time.sleep(0.15)  # 최소 간격
            simulator.add_log("빠른 연속 WP 10개 테스트 완료")
            
        elif action == 'test_wp_intervals':
            simulator.add_log("다양한 간격으로 펄스 테스트 시작")
            intervals = [100, 150, 200, 300, 400, 500]
            for j, interval in enumerate(intervals):
                simulator.add_log(f"간격 {interval}ms로 5개 테스트")
                for i in range(5):
                    simulator.generate_wp_pulse()
                    if i < 4:
                        time.sleep(interval / 1000.0)
                time.sleep(1.0)  # 각 테스트 사이 1초 대기
            simulator.add_log("다양한 간격 테스트 완료")
            
        else:
            return jsonify({"result": "error", "msg": "Unknown action"}), 400
        
        return jsonify({"result": "ok"})
        
    except Exception as e:
        return jsonify({"result": "error", "msg": str(e)}), 400

@app.route('/api/reset', methods=['POST'])
def handle_reset():
    """초기화 API"""
    simulator.meter.wp_count = 0
    simulator.meter.eoi_count = 0
    simulator.meter.energy_wh = 0
    simulator.meter.last_energy_wh = 0
    
    simulator.add_log("카운터 초기화 완료")
    
    return jsonify({"result": "ok", "msg": "카운터 초기화 완료"})

# HTML 템플릿 (Arduino 코드의 HTML과 동일)
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <title>ESP32 통합 시뮬레이터 v1.0 (Python)</title>
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
            <h1>🏭 ESP32 통합 시뮬레이터 v1.0 (Python)</h1>
            <p>인버터 시뮬레이터 + 모의계량기 + RTU 제어 통합</p>
            <p style="color: #666;">PC 환경 실행 | CT비율:6, PT비율:120, WP계수:0.2</p>
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
                    <div>연결: <span id="wifi-status" class="value status-on">PC 환경</span></div>
                    <div>서버: <span id="esp32-ip">localhost:5000</span></div>
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
            if (confirm(`WP 펄스를 ${count}회 연속으로 생성하시겠습니까?\\n(약 ${((count-1) * 0.52 + 0.1).toFixed(1)}초 소요)`)) {
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
'''

if __name__ == '__main__':
    print("ESP32 통합 시뮬레이터 Python 버전 시작")
    print("웹 인터페이스: http://localhost:5000")
    print("인버터 시뮬레이터 + 모의계량기 + RTU 제어")
    print("CT비율:6, PT비율:120, WP계수:0.2")
    print("=" * 50)
    
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
