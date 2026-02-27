"""
RP2040 Modbus RTU Slave → SQLite 데이터 수집기

COM22의 RP2040 Connect에서 Modbus RTU로 센서 데이터를 읽어
SQLite DB(shkimdb.db)의 sensor 테이블에 저장합니다.

Register Map:
  0: accel_x  (g, ÷100)
  1: accel_y  (g, ÷100)
  2: accel_z  (g, ÷100)
  3: gyro_x   (dps, ÷10)
  4: gyro_y   (dps, ÷10)
  5: gyro_z   (dps, ÷10)
  6: temperature (°C, ÷100)
"""

import sqlite3
import time
import signal
import sys
from datetime import datetime

from pymodbus.client import ModbusSerialClient

# ── 설정 ─────────────────────────────────────────────────
SERIAL_PORT = "COM22"
BAUD_RATE = 9600
SLAVE_ID = 1
POLL_INTERVAL = 1.0  # 초

DB_NAME = "shkimdb.db"
TABLE_NAME = "sensor"

running = True


def signal_handler(sig, frame):
    global running
    print("\n종료 중...")
    running = False


def init_db():
    """SQLite DB 및 sensor 테이블 생성"""
    conn = sqlite3.connect(DB_NAME)
    conn.execute(f"""
        CREATE TABLE IF NOT EXISTS {TABLE_NAME} (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp   TEXT    NOT NULL,
            accel_x     REAL    NOT NULL,
            accel_y     REAL    NOT NULL,
            accel_z     REAL    NOT NULL,
            gyro_x      REAL    NOT NULL,
            gyro_y      REAL    NOT NULL,
            gyro_z      REAL    NOT NULL,
            temperature REAL    NOT NULL
        )
    """)
    conn.commit()
    return conn


def read_and_store(client, conn):
    """Modbus 레지스터를 읽어 DB에 저장"""
    result = client.read_holding_registers(address=0, count=7, device_id=SLAVE_ID)

    if result.isError():
        print(f"[{datetime.now():%H:%M:%S}] Modbus 읽기 실패: {result}")
        return False

    regs = result.registers

    # 스케일 변환 (signed 처리)
    def to_signed(val):
        return val - 65536 if val >= 32768 else val

    accel_x = to_signed(regs[0]) / 100.0
    accel_y = to_signed(regs[1]) / 100.0
    accel_z = to_signed(regs[2]) / 100.0
    gyro_x  = to_signed(regs[3]) / 10.0
    gyro_y  = to_signed(regs[4]) / 10.0
    gyro_z  = to_signed(regs[5]) / 10.0
    temp    = to_signed(regs[6]) / 100.0

    now = datetime.now().isoformat()

    conn.execute(
        f"INSERT INTO {TABLE_NAME} (timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temperature) "
        f"VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
        (now, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temp),
    )
    conn.commit()

    print(
        f"[{datetime.now():%H:%M:%S}] "
        f"Accel({accel_x:+.2f}, {accel_y:+.2f}, {accel_z:+.2f})g  "
        f"Gyro({gyro_x:+.1f}, {gyro_y:+.1f}, {gyro_z:+.1f})dps  "
        f"Temp={temp:.1f}°C"
    )
    return True


def main():
    signal.signal(signal.SIGINT, signal_handler)

    # DB 초기화
    conn = init_db()
    print(f"DB: {DB_NAME} / 테이블: {TABLE_NAME}")

    # Modbus 클라이언트 연결
    client = ModbusSerialClient(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        parity="N",
        stopbits=1,
        bytesize=8,
        timeout=2,
    )

    if not client.connect():
        print(f"COM 포트 연결 실패: {SERIAL_PORT}")
        conn.close()
        sys.exit(1)

    print(f"연결됨: {SERIAL_PORT} @ {BAUD_RATE}bps, Slave ID={SLAVE_ID}")
    print(f"폴링 간격: {POLL_INTERVAL}초 (Ctrl+C로 종료)")
    print("-" * 70)

    # 포트 오픈 후 RP2040 재시작 대기
    time.sleep(3)

    try:
        while running:
            read_and_store(client, conn)
            time.sleep(POLL_INTERVAL)
    finally:
        client.close()
        conn.close()
        print("연결 종료. DB 저장 완료.")


if __name__ == "__main__":
    main()
