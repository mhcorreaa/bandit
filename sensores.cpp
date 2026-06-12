// ============================================================
//  sensores.cpp — MPU6050 (caída) + MAX30102 (BPM) + DFPlayer
//  SDA=D7, SCL=D8 | MAX: 0x57 | MPU: 0x68
// ============================================================
#include "sensores.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <DFRobotDFPlayerMini.h>

// ============================================================
//  MPU6050
// ============================================================
#define MPU_ADDR     0x68
#define MPU_PWR      0x6B
#define MPU_ACCEL    0x3B
#define ACCEL_LSB    16384.0f

static bool _caidaDetectada = false;

static void _initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR);
  Wire.write(0x00);
  if (Wire.endTransmission(true) == 0)
    Serial.println("[MPU6050] OK");
  else
    Serial.println("[MPU6050] No responde — verifica SDA=D7 SCL=D8");
}

static void _leerMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  float ax = ((int16_t)(Wire.read() << 8 | Wire.read())) / ACCEL_LSB;
  float ay = ((int16_t)(Wire.read() << 8 | Wire.read())) / ACCEL_LSB;
  float az = ((int16_t)(Wire.read() << 8 | Wire.read())) / ACCEL_LSB;
  float g  = sqrt(ax*ax + ay*ay + az*az);
  _caidaDetectada = (g > ACCEL_CAIDA_THRESHOLD) || (g < ACCEL_CAIDA_LIBRE);
}

// ============================================================
//  MAX30102 — basado en el ejemplo oficial de SparkFun
// ============================================================
static MAX30105 _sensor;
static bool     _maxListo     = false;
static bool     _dedo         = false;
static int      _bpmPromedio  = 0;
static long     _ultimoLatido = 0;
static float    _bpmInstant   = 0;

#define RATE_SIZE 4
static byte _tasas[RATE_SIZE] = {0};
static byte _idx = 0;

static void _initMAX() {
  if (!_sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("[MAX30102] No responde — verifica SDA=D7 SCL=D8");
    return;
  }
  _sensor.setup();
  _sensor.setPulseAmplitudeRed(0x0A);
  _sensor.setPulseAmplitudeGreen(0);
  _maxListo = true;
  Serial.println("[MAX30102] OK — apoya el dedo con presión constante");
}

static void _leerMAX() {
  if (!_maxListo) return;

  long ir = _sensor.getIR();
  _dedo = (ir >= 50000);

  if (!_dedo) {
    for (byte i = 0; i < RATE_SIZE; i++) _tasas[i] = 0;
    _bpmPromedio  = 0;
    _bpmInstant   = 0;
    _ultimoLatido = 0;
    _idx          = 0;
    return;
  }

  if (checkForBeat(ir)) {
    long delta    = millis() - _ultimoLatido;
    _ultimoLatido = millis();
    _bpmInstant   = 60.0f / (delta / 1000.0f);

    if (_bpmInstant > 20 && _bpmInstant < 255) {
      _tasas[_idx++] = (byte)_bpmInstant;
      _idx %= RATE_SIZE;

      _bpmPromedio = 0;
      for (byte i = 0; i < RATE_SIZE; i++) _bpmPromedio += _tasas[i];
      _bpmPromedio /= RATE_SIZE;

      Serial.print("[MAX30102] IR=");  Serial.print(ir);
      Serial.print(" BPM=");          Serial.print((int)_bpmInstant);
      Serial.print(" Avg BPM=");      Serial.println(_bpmPromedio);
    }
  }
}

// ============================================================
//  DFPlayer
// ============================================================
static HardwareSerial      _serialDF(1);
static DFRobotDFPlayerMini _df;
static bool                _dfListo = false;

void audioInit() {
  _serialDF.begin(9600, SERIAL_8N1, PIN_DFPLAYER_RX, PIN_DFPLAYER_TX);
  delay(1000);
  if (_df.begin(_serialDF)) {
    _df.volume(10);
    _dfListo = true;
    Serial.println("[DFPlayer] OK — volumen 10/30");
  } else {
    Serial.println("[DFPlayer] No responde — verifica MicroSD y parlante");
  }
}

void audioReproducirBucle() { if (_dfListo) _df.loop(1); }
void audioDetener()         { if (_dfListo) _df.stop();  }

void audioTick() {
  if (!_dfListo || !_df.available()) return;
  if (_df.readType() == DFPlayerError) {
    Serial.print("[DFPlayer ERROR] "); Serial.println(_df.read());
  }
}

// ============================================================
//  API PÚBLICA
// ============================================================
void sensoresInit() {
  Wire.begin(D7, D8);
  _initMPU();
  _initMAX();
}

void sensoresTick() {
  _leerMPU();
  _leerMAX();
}

bool caidaDetectada() { return _caidaDetectada;                      }
int  bpmActual()      { return _bpmPromedio;                         }
bool bpmElevado()     { return _bpmPromedio >= BPM_ELEVADO && _dedo; }
bool dedoDetectado()  { return _dedo;                                }
