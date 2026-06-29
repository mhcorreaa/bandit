// ============================================================
//  sensores.cpp — MPU6050 (caída) + MAX30102 (BPM+SpO2) + DFPlayer
//  SDA=D4, SCL=D5 | MAX: 0x57 | MPU: 0x68
// ============================================================
#include "sensores.h"
#include "config.h"
#include <Arduino.h>
#include <limits.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <DFRobotDFPlayerMini.h>

// ============================================================
//  MPU6050
// ============================================================
#define MPU_ADDR  0x68
#define MPU_PWR   0x6B
#define MPU_ACCEL 0x3B
#define ACCEL_LSB 16384.0f

static bool     _caidaDetectada    = false;
static float    _gActual           = 1.0f;
static uint32_t _tUltimoMovimiento = 0;
static bool     _inmovil           = false;

// Detección de caída en dos fases:
// 1) Se entra en caída libre (g < ACCEL_CAIDA_LIBRE) y se registra el momento
// 2) Si luego llega un impacto (g > ACCEL_CAIDA_THRESHOLD) habiendo pasado
//    al menos CAIDA_LIBRE_MIN_MS en caída libre → caída real confirmada
static bool     _enCaidaLibre      = false;
static uint32_t _tInicioCaidaLibre = 0;

#define INMOVIL_TOLERANCIA  0.08f
#define INMOVIL_TIEMPO_MS   3000

static void _initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR);
  Wire.write(0x00);
  if (Wire.endTransmission(true) == 0)
    Serial.println("[MPU6050] OK");
  else
    Serial.println("[MPU6050] No responde — verifica SDA=D4 SCL=D5");
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
  _gActual = g;

  // ── Detección de caída en dos fases ──────────────────────
  // Fase 1: entrada en caída libre
  if (g < ACCEL_CAIDA_LIBRE) {
    if (!_enCaidaLibre) {
      _enCaidaLibre      = true;
      _tInicioCaidaLibre = millis();
    }
  } else {
    // Fase 2: impacto después de haber estado en caída libre suficiente tiempo
    if (_enCaidaLibre && g > ACCEL_CAIDA_THRESHOLD) {
      uint32_t duracionLibre = millis() - _tInicioCaidaLibre;
      if (duracionLibre >= CAIDA_LIBRE_MIN_MS) {
        _caidaDetectada = true;  // latch — se consume en caidaDetectada()
        Serial.print("[MPU] Caida confirmada: libre=");
        Serial.print(duracionLibre);
        Serial.print("ms impacto=");
        Serial.print(g, 2);
        Serial.println("g");
      } else {
        Serial.print("[MPU] Impacto descartado (caida libre muy breve: ");
        Serial.print(duracionLibre);
        Serial.println("ms) — probable movimiento brusco");
      }
    }
    // Salir de caída libre en cuanto g vuelve a rango normal
    if (g > ACCEL_CAIDA_LIBRE) _enCaidaLibre = false;
  }

  // Inmovilidad
  uint32_t ahora = millis();
  if (fabs(g - _gActual) > INMOVIL_TOLERANCIA) _tUltimoMovimiento = ahora;
  _inmovil = (ahora - _tUltimoMovimiento) >= INMOVIL_TIEMPO_MS;
}

// ============================================================
//  MAX30102
// ============================================================
static MAX30105 _sensor;
static bool     _maxListo    = false;
static bool     _dedo        = false;
static int      _bpmPromedio = 0;
static long     _ultimoLatido = 0;
static float    _bpmInstant  = 0;

#define RATE_SIZE 4
static byte _tasas[RATE_SIZE] = {0};
static byte _idx = 0;

// BPM elevado sostenido — para evitar lecturas random
// Se lleva un contador de cuánto tiempo lleva el BPM sobre el umbral
static uint32_t _tInicioBpmElevado = 0;
static bool     _bpmElevadoSostenido = false;

// Variación brusca de BPM
#define BPM_HISTORIAL_MS  5000
static int      _bpmHace5s       = 0;
static uint32_t _tUltimoSnapshot = 0;
static bool     _bpmCaidaBrusca  = false;

// SpO2
#define SPO2_BUFFER 100
static long     _bufIR[SPO2_BUFFER];
static long     _bufRojo[SPO2_BUFFER];
static int      _bufN        = 0;
static byte     _bufIdx      = 0;
static int      _spo2        = 0;
static uint32_t _tInicioBajo = 0;

static void _initMAX() {
  if (!_sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("[MAX30102] No responde — verifica SDA=D4 SCL=D5");
    return;
  }
  _sensor.setup();
  _sensor.setPulseAmplitudeRed(0x1F);
  _sensor.setPulseAmplitudeGreen(0);
  _maxListo = true;
  Serial.println("[MAX30102] OK — apoya el dedo con presión constante");
}

static void _actualizarSpo2(long ir, long rojo) {
  _bufIR[_bufIdx]   = ir;
  _bufRojo[_bufIdx] = rojo;
  _bufIdx = (_bufIdx + 1) % SPO2_BUFFER;
  if (_bufN < SPO2_BUFFER) _bufN++;
  if (_bufN < SPO2_BUFFER) return;

  long sumaIR = 0, sumaRojo = 0;
  long maxIR = 0, minIR = LONG_MAX, maxRojo = 0, minRojo = LONG_MAX;
  for (int i = 0; i < SPO2_BUFFER; i++) {
    sumaIR   += _bufIR[i];
    sumaRojo += _bufRojo[i];
    if (_bufIR[i]   > maxIR)   maxIR   = _bufIR[i];
    if (_bufIR[i]   < minIR)   minIR   = _bufIR[i];
    if (_bufRojo[i] > maxRojo) maxRojo = _bufRojo[i];
    if (_bufRojo[i] < minRojo) minRojo = _bufRojo[i];
  }

  float dcIR   = sumaIR   / (float)SPO2_BUFFER;
  float dcRojo = sumaRojo / (float)SPO2_BUFFER;
  float acIR   = (float)(maxIR   - minIR);
  float acRojo = (float)(maxRojo - minRojo);

  if (dcIR <= 0 || dcRojo <= 0 || acIR <= 0) return;

  float R    = (acRojo / dcRojo) / (acIR / dcIR);
  int   calc = (int)(SPO2_COEF_A - SPO2_COEF_B * R);
  if (calc > 100) calc = 100;

  if (calc >= 70 && calc <= 100) {
    _spo2 = calc;
    if (_spo2 < SPO2_MINIMO) {
      if (_tInicioBajo == 0) _tInicioBajo = millis();
    } else {
      _tInicioBajo = 0;
    }
  }
}

static void _leerMAX() {
  if (!_maxListo) return;

  _sensor.check();

  while (_sensor.available()) {
    long ir   = _sensor.getFIFOIR();
    long rojo = _sensor.getFIFORed();
    _sensor.nextSample();

    bool dedoAntes = _dedo;
    _dedo = (ir >= 50000);

    if (!_dedo) {
      if (dedoAntes) {
        for (byte i = 0; i < RATE_SIZE; i++) _tasas[i] = 0;
        _bpmPromedio          = 0;
        _bpmInstant           = 0;
        _ultimoLatido         = 0;
        _idx                  = 0;
        _bufN                 = 0;
        _bufIdx               = 0;
        _spo2                 = 0;
        _tInicioBajo          = 0;
        _bpmHace5s            = 0;
        _bpmCaidaBrusca       = false;
        _tUltimoSnapshot      = 0;
        _tInicioBpmElevado    = 0;
        _bpmElevadoSostenido  = false;
      }
      continue;
    }

    _actualizarSpo2(ir, rojo);

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
      }
    }
  }

  // BPM elevado sostenido — solo cuenta si el promedio (no una muestra)
  // lleva más de BPM_ELEVADO_TIEMPO_MS sobre el umbral
  if (_bpmPromedio >= BPM_ELEVADO && _dedo) {
    if (_tInicioBpmElevado == 0) _tInicioBpmElevado = millis();
    _bpmElevadoSostenido = (millis() - _tInicioBpmElevado) >= BPM_ELEVADO_TIEMPO_MS;
  } else {
    _tInicioBpmElevado   = 0;
    _bpmElevadoSostenido = false;
  }

  // Variación brusca de BPM (cada 5s)
  uint32_t ahora = millis();
  if (_tUltimoSnapshot == 0) _tUltimoSnapshot = ahora;
  if (ahora - _tUltimoSnapshot >= BPM_HISTORIAL_MS) {
    if (_bpmHace5s > 0 && _bpmPromedio > 0) {
      int delta = _bpmHace5s - _bpmPromedio;
      _bpmCaidaBrusca = (delta >= 25);
      if (_bpmCaidaBrusca) {
        Serial.print("[ALERTA BPM] Caida brusca: ");
        Serial.print(_bpmHace5s);
        Serial.print(" -> ");
        Serial.println(_bpmPromedio);
      }
    }
    _bpmHace5s       = _bpmPromedio;
    _tUltimoSnapshot = ahora;
  }
}

// ============================================================
//  DFPlayer
// ============================================================
static HardwareSerial      _serialDF(1);
static DFRobotDFPlayerMini _df;
static bool                _dfListo = false;

void audioInit() {
  _serialDF.begin(9600, SERIAL_8N1, D7, D6);
  for (byte intento = 1; intento <= 3; intento++) {
    delay(2000);
    if (_df.begin(_serialDF, true, false)) {
      _df.volume(25);
      _dfListo = true;
      Serial.println("[DFPlayer] OK — volumen 25/30");
      return;
    }
    Serial.print("[DFPlayer] Intento "); Serial.print(intento); Serial.println("/3 fallido...");
  }
  Serial.println("[DFPlayer] No responde — verifica RX/TX, VCC=5V, MicroSD");
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
  Wire.begin(D4, D5);
  _initMPU();
  _initMAX();
}

void sensoresTick() {
  _leerMPU();
  _leerMAX();
}

// Latch: retorna true una sola vez por evento y se autoreset
bool caidaDetectada() {
  if (_caidaDetectada) {
    _caidaDetectada = false;
    return true;
  }
  return false;
}

float gActual()            { return _gActual;                                        }
int   bpmActual()          { return _bpmPromedio;                                    }
bool  bpmElevado()         { return _bpmElevadoSostenido;                            } // sostenido >5s
bool  dedoDetectado()      { return _dedo;                                           }
int   spo2Actual()         { return _spo2;                                           }
bool  spo2Bajo()           { return _tInicioBajo > 0 && (millis() - _tInicioBajo) >= SPO2_TIEMPO_BAJO_MS; }
bool  cuerpoInmovil()      { return _inmovil;                                        }
bool  bpmCaidaBrusca()     { return _bpmCaidaBrusca && _dedo;                        }
bool  sospechaDesmayo()    { return spo2Bajo() && bpmCaidaBrusca();                  }
