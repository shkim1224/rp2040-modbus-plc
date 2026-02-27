/*
 * RP2040 Connect - Modbus RTU Slave
 *
 * 온보드 IMU(LSM6DSOX) 센서 데이터를 Modbus 레지스터로 제공합니다.
 * 통신: USB Serial (CDC), Modbus RTU 프로토콜
 *
 * Slave ID : 1
 * Baud Rate: 9600, 8N1
 *
 * 지원 Function Code:
 *   FC 03 - Read Holding Registers
 *   FC 04 - Read Input Registers
 *
 * Register Map (int16_t):
 * +---------+---------------------+-----------+--------+
 * | Address | Description         | Unit      | Scale  |
 * +---------+---------------------+-----------+--------+
 * |    0    | Accelerometer X     | g x 100   | / 100  |
 * |    1    | Accelerometer Y     | g x 100   | / 100  |
 * |    2    | Accelerometer Z     | g x 100   | / 100  |
 * |    3    | Gyroscope X         | dps x 10  | / 10   |
 * |    4    | Gyroscope Y         | dps x 10  | / 10   |
 * |    5    | Gyroscope Z         | dps x 10  | / 10   |
 * |    6    | Temperature         | C x 100   | / 100  |
 * |  7~19   | (Reserved)          | 0         |        |
 * +---------+---------------------+-----------+--------+
 *
 * Required Library:
 *   - Arduino_LSM6DSOX  (Board Manager에 포함)
 */

#include <Arduino_LSM6DSOX.h>

// ── Modbus 설정 ──────────────────────────────────────────
#define MODBUS_SLAVE_ID   1
#define MODBUS_BAUD       9600
#define NUM_REGISTERS     20     // 레지스터 여유분 포함 (Modbus Poll 기본 Quantity 대응)

// USB CDC 프레임 간 타임아웃 (ms)
#define FRAME_TIMEOUT_MS  50

// ── 센서 & 레지스터 ────────────────────────────────────
int16_t inputRegisters[NUM_REGISTERS];

// ── Modbus 수신 버퍼 ──────────────────────────────────
static uint8_t  rxBuf[256];
static uint16_t rxLen = 0;
static uint32_t lastRxTime = 0;

// ── LED ──────────────────────────────────────────────────
#define LED_PIN LED_BUILTIN

// =========================================================
//  CRC-16 / Modbus
// =========================================================
static uint16_t modbusCRC16(const uint8_t *buf, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)buf[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// =========================================================
//  센서 데이터 갱신
// =========================================================
static void updateSensors() {
  float ax, ay, az;
  float gx, gy, gz;
  int tempInt;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    inputRegisters[0] = (int16_t)(ax * 100.0f);
    inputRegisters[1] = (int16_t)(ay * 100.0f);
    inputRegisters[2] = (int16_t)(az * 100.0f);
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    inputRegisters[3] = (int16_t)(gx * 10.0f);
    inputRegisters[4] = (int16_t)(gy * 10.0f);
    inputRegisters[5] = (int16_t)(gz * 10.0f);
  }

  if (IMU.temperatureAvailable()) {
    IMU.readTemperature(tempInt);
    inputRegisters[6] = (int16_t)(tempInt * 100);
  }
}

// =========================================================
//  Modbus 예외 응답 전송
// =========================================================
static void sendException(uint8_t fc, uint8_t exCode) {
  uint8_t resp[5];
  resp[0] = MODBUS_SLAVE_ID;
  resp[1] = fc | 0x80;
  resp[2] = exCode;
  uint16_t crc = modbusCRC16(resp, 3);
  resp[3] = crc & 0xFF;
  resp[4] = (crc >> 8) & 0xFF;
  Serial.write(resp, 5);
}

// =========================================================
//  FC03 / FC04 - Read Registers 처리
// =========================================================
static void handleReadRegisters() {
  uint16_t startAddr = ((uint16_t)rxBuf[2] << 8) | rxBuf[3];
  uint16_t quantity  = ((uint16_t)rxBuf[4] << 8) | rxBuf[5];

  if (startAddr + quantity > NUM_REGISTERS) {
    sendException(rxBuf[1], 0x02);
    return;
  }
  if (quantity < 1 || quantity > 125) {
    sendException(rxBuf[1], 0x03);
    return;
  }

  uint8_t txBuf[256];
  uint8_t txLen = 0;

  txBuf[txLen++] = MODBUS_SLAVE_ID;
  txBuf[txLen++] = rxBuf[1];
  txBuf[txLen++] = (uint8_t)(quantity * 2);

  for (uint16_t i = 0; i < quantity; i++) {
    int16_t val = inputRegisters[startAddr + i];
    txBuf[txLen++] = (val >> 8) & 0xFF;
    txBuf[txLen++] = val & 0xFF;
  }

  uint16_t crc = modbusCRC16(txBuf, txLen);
  txBuf[txLen++] = crc & 0xFF;
  txBuf[txLen++] = (crc >> 8) & 0xFF;

  Serial.write(txBuf, txLen);
  Serial.flush();
}

// =========================================================
//  Modbus 프레임 처리
// =========================================================
static void processModbusFrame() {
  if (rxLen < 8) return;
  if (rxBuf[0] != MODBUS_SLAVE_ID) return;

  uint16_t recvCRC = (uint16_t)rxBuf[rxLen - 2] | ((uint16_t)rxBuf[rxLen - 1] << 8);
  uint16_t calcCRC = modbusCRC16(rxBuf, rxLen - 2);
  if (recvCRC != calcCRC) return;

  uint8_t fc = rxBuf[1];
  switch (fc) {
    case 0x03:
    case 0x04:
      handleReadRegisters();
      break;
    default:
      sendException(fc, 0x01);
      break;
  }

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

// =========================================================
//  SETUP
// =========================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(MODBUS_BAUD);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 5000));

  if (!IMU.begin()) {
    while (1) {
      digitalWrite(LED_PIN, HIGH); delay(100);
      digitalWrite(LED_PIN, LOW);  delay(100);
    }
  }

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(200);
    digitalWrite(LED_PIN, LOW);  delay(200);
  }

  memset(inputRegisters, 0, sizeof(inputRegisters));
}

// =========================================================
//  LOOP
// =========================================================
void loop() {
  updateSensors();

  while (Serial.available()) {
    if (rxLen < sizeof(rxBuf)) {
      rxBuf[rxLen++] = Serial.read();
    } else {
      Serial.read();
      rxLen = 0;
    }
    lastRxTime = millis();
  }

  if (rxLen > 0 && (millis() - lastRxTime >= FRAME_TIMEOUT_MS)) {
    processModbusFrame();
    rxLen = 0;
  }
}
