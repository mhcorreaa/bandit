// ============================================================
//  sensores.cpp — MPU6050 (caída) + MAX30102 (BPM) + DFPlayer
//  SDA=D7, SCL=D8 | MAX: 0x57 | MPU: 0x68
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
#define MPU_ADDR     0x68
#define MPU_PWR      0x6B
#define MPU_ACCEL    0x3B
#define ACCEL_LSB    16384.0f

static bool _caidaDetectada = false;

// Detección de inmovilidad: aceleración estable cerca de 1g por varios segundos
// (típico de una persona desmayada/inconsciente en el piso)
static float    _gActual          = 1.0f;
static uint32_t _tUltimoMovimiento = 0;
#define INMOVIL_TOLERANCIA   0.08f  // variación de G considerada "sin movimiento"
#define INMOVIL_TIEMPO_MS    3000   // ms de quietud para considerar inmovilidad
static bool _inmovil = false;

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

  // Inmovilidad: si la variación de G respecto a la lectura anterior
  // es mínima, el cuerpo no se está moviendo (posible inconsciencia)
  uint32_t ahora = millis();
  if (fabs(g - _gActual) > INMOVIL_TOLERANCIA) {
    _tUltimoMovimiento = ahora;  // hubo movimiento, reinicia el contador
  }
  _gActual = g;
  _inmovil = (ahora - _tUltimoMovimiento) >= INMOVIL_TIEMPO_MS;
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

// ── Variación brusca de BPM (posible pre-síncope) ──────────
// Guarda el promedio de hace ~5s y lo compara con el actual.
// Una caída brusca de BPM (bradicardia súbita) es señal clásica
// de un síncope vasovagal inminente.
#define BPM_HISTORIAL_MS  5000
static int      _bpmHace5s        = 0;
static uint32_t _tUltimoSnapshot  = 0;
static bool     _bpmCaidaBrusca   = false;

// ── SpO2 — completamente independiente del cálculo de BPM ──
// Buffer circular de muestras IR/Rojo para estimar AC/DC de cada canal
#define SPO2_BUFFER 100
static long _bufIR[SPO2_BUFFER];
static long _bufRojo[SPO2_BUFFER];
static int  _bufN     = 0;   // cantidad de muestras válidas acumuladas
static byte _bufIdx   = 0;
static int  _spo2     = 0;   // 0 = aún no disponible

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

// Calcula SpO2 a partir del buffer circular (ratio R = ACred/DCred / ACir/DCir)
static void _actualizarSpo2(long ir, long rojo) {
  _bufIR[_bufIdx]   = ir;
  _bufRojo[_bufIdx] = rojo;
  _bufIdx = (_bufIdx + 1) % SPO2_BUFFER;
  if (_bufN < SPO2_BUFFER) _bufN++;

  // Espera a tener suficientes muestras para un cálculo estable
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

  float R = (acRojo / dcRojo) / (acIR / dcIR);

  // Fórmula calibrable: SpO2 = SPO2_COEF_A - SPO2_COEF_B * R
  int calc = (int)(SPO2_COEF_A - SPO2_COEF_B * R);

  // Log de R crudo para calibración manual (ver config.h)
  Serial.print("[SpO2 calib] R="); Serial.print(R, 3);
  Serial.print(" -> calc="); Serial.println(calc);

  // Si la presión del dedo aumenta, R puede bajar de lo normal y
  // el cálculo da >100% — fisiológicamente el máximo real es 100%,
  // así que se recorta (clamp) en vez de descartar la lectura.
  if (calc > 100) calc = 100;

  if (calc >= 70 && calc <= 100) {
    _spo2 = calc;
  }
}

static void _leerMAX() {
  if (!_maxListo) return;

  long ir   = _sensor.getIR();
  long rojo = _sensor.getRed();
  _dedo = (ir >= 50000);

  if (!_dedo) {
    for (byte i = 0; i < RATE_SIZE; i++) _tasas[i] = 0;
    _bpmPromedio  = 0;
    _bpmInstant   = 0;
    _ultimoLatido = 0;
    _idx          = 0;
    _bufN         = 0;
    _bufIdx       = 0;
    _spo2         = 0;
    _bpmHace5s       = 0;
    _bpmCaidaBrusca  = false;
    _tUltimoSnapshot = 0;
    return;
  }

  // SpO2: se acumula en paralelo, no afecta el cálculo de BPM
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

      Serial.print("[MAX30102] IR=");  Serial.print(ir);
      Serial.print(" BPM=");          Serial.print((int)_bpmInstant);
      Serial.print(" Avg BPM=");      Serial.print(_bpmPromedio);
      Serial.print(" SpO2=");
      if (_spo2 > 0) Serial.print(_spo2); else Serial.print("--");
      Serial.println("%");
    }
  }

  // ── Tracking de variación brusca de BPM (independiente de checkForBeat) ──
  uint32_t ahora = millis();
  if (_tUltimoSnapshot == 0) _tUltimoSnapshot = ahora;
  if (ahora - _tUltimoSnapshot >= BPM_HISTORIAL_MS) {
    // Compara el BPM de hace 5s contra el actual
    if (_bpmHace5s > 0 && _bpmPromedio > 0) {
      int delta = _bpmHace5s - _bpmPromedio;
      // Caída de más de 25 bpm en 5s = posible bradicardia súbita (pre-síncope)
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
  _serialDF.begin(9600, SERIAL_8N1, PIN_DFPLAYER_RX, PIN_DFPLAYER_TX);
  delay(1000);
  if (_df.begin(_serialDF)) {
    _df.volume(10);
    _dfListo = true;
    Serial.println("[DFPlayer] OK — volumen 25/30");
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
int  spo2Actual()     { return _spo2;                                }
bool spo2Bajo()       { return _spo2 > 0 && _spo2 < 90;              }
bool cuerpoInmovil()  { return _inmovil;                             }
bool bpmCaidaBrusca() { return _bpmCaidaBrusca && _dedo;             }

// Señal combinada de pre-síncope: SpO2 bajo + caída brusca de BPM,
// sin necesidad de que ya haya ocurrido la caída física
bool sospechaDesmayo() {
  return spo2Bajo() && bpmCaidaBrusca();
}
