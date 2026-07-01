// ============================================================ a
//  sensores.cpp — MPU6050 (caída + postura) + MAX30102 (BPM)
//                 + DFPlayer
//  v3: sin SpO2 | + filtro postural | + choque cardíaco
//                (colapso amplitud IR eliminado — ver nota más abajo)
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

// Ejes crudos — necesarios para filtro postural
static float _axFilt = 0.0f;   // promedio exponencial de ax
static float _ayFilt = 0.0f;   // promedio exponencial de ay
static float _azFilt = 0.0f;   // promedio exponencial de az
#define POSTURA_ALPHA 0.05f     // constante del filtro paso-bajo (τ ≈ 1s a 20ms/ciclo)

// Postura erguida:
// La pulsera va en el bíceps con el brazo colgando → eje longitudinal
// del brazo ≈ eje Y del sensor. Cuando el brazo está vertical (de pie/sentado)
// |ay| > POSTURA_UMBRAL_VERTICAL y la proyección horizontal es pequeña.
// Cuando el paciente está acostado, la gravedad se distribuye en ax/az.
static bool _posturaErguida = false;

// Detección de caída en dos fases
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
  float g  = sqrtf(ax*ax + ay*ay + az*az);
  _gActual = g;

  // ── Filtro paso-bajo para estimación postural ─────────
  // Aísla el componente DC (gravedad) de cada eje.
  // Alpha pequeño → respuesta lenta → inmune a movimientos bruscos.
  _axFilt += POSTURA_ALPHA * (ax - _axFilt);
  _ayFilt += POSTURA_ALPHA * (ay - _ayFilt);
  _azFilt += POSTURA_ALPHA * (az - _azFilt);

  // ── Clasificación postural ────────────────────────────
  // Con la pulsera en el bíceps y el brazo colgando naturalmente,
  // el eje que apunta hacia el suelo es Y. Se considera "erguido"
  // cuando la componente vertical filtrada es dominante y supera
  // el umbral. El umbral 0.65g equivale a ~40° de desviación máxima
  // respecto a la vertical — cubre sentado con el brazo apoyado.
  float ayAbs = fabsf(_ayFilt);
  float horizPlane = sqrtf(_axFilt*_axFilt + _azFilt*_azFilt);

  // Erguido: componente vertical dominante Y > 0.65g
  //          Y el plano horizontal es <0.75g (no acostado de lado)
  _posturaErguida = (ayAbs > POSTURA_UMBRAL_VERTICAL) &&
                    (horizPlane < POSTURA_UMBRAL_HORIZONTAL);

  // ── Detección de caída en dos fases ──────────────────
  if (g < ACCEL_CAIDA_LIBRE) {
    if (!_enCaidaLibre) {
      _enCaidaLibre      = true;
      _tInicioCaidaLibre = millis();
    }
  } else {
    if (_enCaidaLibre && g > ACCEL_CAIDA_THRESHOLD) {
      uint32_t duracionLibre = millis() - _tInicioCaidaLibre;
      if (duracionLibre >= CAIDA_LIBRE_MIN_MS) {
        _caidaDetectada = true;
        Serial.print("[MPU] Caida confirmada: libre=");
        Serial.print(duracionLibre);
        Serial.print("ms impacto=");
        Serial.print(g, 2);
        Serial.println("g");
      } else {
        Serial.print("[MPU] Impacto descartado (caida libre breve: ");
        Serial.print(duracionLibre);
        Serial.println("ms)");
      }
    }
    if (g > ACCEL_CAIDA_LIBRE) _enCaidaLibre = false;
  }

  // ── Inmovilidad ───────────────────────────────────────
  uint32_t ahora = millis();
  static float _gAnterior = 1.0f;
  if (fabsf(g - _gAnterior) > INMOVIL_TOLERANCIA) _tUltimoMovimiento = ahora;
  _gAnterior = g;
  _inmovil   = (ahora - _tUltimoMovimiento) >= INMOVIL_TIEMPO_MS;
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

// BPM elevado sostenido
static uint32_t _tInicioBpmElevado   = 0;
static bool     _bpmElevadoSostenido = false;

// NOTA: el detector de "colapso de amplitud IR" fue eliminado.
// En pruebas reales con el sensor sin contacto directo en el bíceps
// (~1mm de separación de la piel), la amplitud IR mostraba saltos
// de escalón mecánicos (cambios de ángulo/presión del sensor que se
// mantenían estables varios segundos y luego saltaban a otro nivel).
// Estos saltos producían ratios de colapso de 0.03–0.04 — fisiológicamente
// imposibles como caída real de perfusión en 2s — incluso con período
// de estabilización y baseline mínimo. La amplitud absoluta IR no es
// un proxy fiable de perfusión con este montaje de hardware.
// El choque cardíaco (basado en timing de latidos, no en magnitud de
// señal) es robusto a este problema y queda como única ruta óptica.

// ── Choque cardíaco ───────────────────────────────────────
//
// Máquina de estados simple de tres fases:
//   REPOSO → TAQUI (cuando BPM > BPM_CHOQUE_TAQUI durante al menos
//            BPM_CHOQUE_TAQUI_CONFIRMA_MS)
//          → BRADI (cuando, dentro de BPM_CHOQUE_VENTANA_MS desde el
//            inicio de la taquicardia, el BPM cae bajo BPM_CHOQUE_BRADI)
//            → flag choqueCardiaco = true (latch, se limpia al soltar dedo
//              o al salir de ESTADO_ALERTA desde el .ino)
//
// Se confirman ambas fases con un tiempo mínimo sostenido para evitar
// disparos por latidos aislados.

enum FaseChoque { CHOQUE_REPOSO, CHOQUE_TAQUI, CHOQUE_BRADI };
static FaseChoque _faseChoque       = CHOQUE_REPOSO;
static uint32_t   _tInicioTaqui     = 0;   // cuando entró en taquicardia
static uint32_t   _tTaquiConfirmada = 0;   // cuando se confirmó la taquicardia
static bool       _choqueCardiaco   = false;

static void _actualizarChoqueCardiaco() {
  if (!_dedo || _bpmPromedio == 0) {
    _faseChoque   = CHOQUE_REPOSO;
    _tInicioTaqui = 0;
    return;
  }

  switch (_faseChoque) {
    case CHOQUE_REPOSO:
      if (_bpmPromedio > BPM_CHOQUE_TAQUI) {
        _faseChoque   = CHOQUE_TAQUI;
        _tInicioTaqui = millis();
        Serial.print("[CHOQUE] Taquicardia iniciada: BPM=");
        Serial.println(_bpmPromedio);
      }
      break;

    case CHOQUE_TAQUI:
      if (_bpmPromedio <= BPM_CHOQUE_TAQUI) {
        // Se perdió la taquicardia antes de confirmar → vuelve a reposo
        _faseChoque = CHOQUE_REPOSO;
        break;
      }
      // Confirmar que la taquicardia se sostiene al menos BPM_CHOQUE_TAQUI_CONFIRMA_MS
      if (_tTaquiConfirmada == 0 &&
          (millis() - _tInicioTaqui) >= BPM_CHOQUE_TAQUI_CONFIRMA_MS) {
        _tTaquiConfirmada = millis();
        Serial.println("[CHOQUE] Taquicardia confirmada — esperando bradicardia");
      }
      // Ventana agotada sin bradicardia
      if (_tTaquiConfirmada > 0 &&
          (millis() - _tTaquiConfirmada) > BPM_CHOQUE_VENTANA_MS) {
        Serial.println("[CHOQUE] Ventana expirada sin bradicardia — reset");
        _faseChoque       = CHOQUE_REPOSO;
        _tTaquiConfirmada = 0;
      }
      // Caída a bradicardia dentro de la ventana
      if (_tTaquiConfirmada > 0 &&
          _bpmPromedio < BPM_CHOQUE_BRADI &&
          (millis() - _tTaquiConfirmada) <= BPM_CHOQUE_VENTANA_MS) {
        _choqueCardiaco   = true;
        _faseChoque       = CHOQUE_BRADI;
        _tTaquiConfirmada = 0;
        Serial.print("[CHOQUE] PATRON DETECTADO: taqui→bradi | BPM=");
        Serial.println(_bpmPromedio);
      }
      break;

    case CHOQUE_BRADI:
      // La bandera es latch — se mantiene hasta que el .ino la consuma
      // o el dedo se suelte (limpieza en _leerMAX al perder contacto)
      break;
  }
}

// ── DFPlayer ─────────────────────────────────────────────
static HardwareSerial      _serialDF(1);
static DFRobotDFPlayerMini _df;
static bool                _dfListo = false;

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

static void _leerMAX() {
  if (!_maxListo) return;

  _sensor.check();

  while (_sensor.available()) {
    long ir = _sensor.getFIFOIR();
    // Nota: ya no leemos el canal rojo (getFIFORed) — solo se usaba
    // para SpO2 (eliminado) y colapso de amplitud IR (eliminado).
    // Solo IR es necesario para detección de latidos (checkForBeat).
    _sensor.nextSample();

    bool dedoAntes = _dedo;
    _dedo = (ir >= 50000);

    if (!_dedo) {
      if (dedoAntes) {
        // Reset completo al perder contacto
        for (byte i = 0; i < RATE_SIZE; i++) _tasas[i] = 0;
        _bpmPromedio         = 0;
        _bpmInstant          = 0;
        _ultimoLatido        = 0;
        _idx                 = 0;
        _tInicioBpmElevado   = 0;
        _bpmElevadoSostenido = false;

        // Choque cardíaco — reset
        _faseChoque       = CHOQUE_REPOSO;
        _tInicioTaqui     = 0;
        _tTaquiConfirmada = 0;
        _choqueCardiaco   = false;
      }
      continue;
    }

    if (checkForBeat(ir)) {
      long delta    = millis() - _ultimoLatido;
      _ultimoLatido = millis();
      _bpmInstant   = 60000.0f / (float)delta;   // delta en ms

      if (_bpmInstant > 20 && _bpmInstant < 255) {
        _tasas[_idx++] = (byte)_bpmInstant;
        _idx %= RATE_SIZE;
        _bpmPromedio = 0;
        for (byte i = 0; i < RATE_SIZE; i++) _bpmPromedio += _tasas[i];
        _bpmPromedio /= RATE_SIZE;
      }
    }
  }

  // BPM elevado sostenido
  if (_bpmPromedio >= BPM_ELEVADO && _dedo) {
    if (_tInicioBpmElevado == 0) _tInicioBpmElevado = millis();
    _bpmElevadoSostenido = (millis() - _tInicioBpmElevado) >= BPM_ELEVADO_TIEMPO_MS;
  } else {
    _tInicioBpmElevado   = 0;
    _bpmElevadoSostenido = false;
  }

  // Choque cardíaco
  _actualizarChoqueCardiaco();
}

// ============================================================
//  DFPlayer
// ============================================================
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

bool caidaDetectada() {
  if (_caidaDetectada) { _caidaDetectada = false; return true; }
  return false;
}

float gActual()       { return _gActual;           }
bool  cuerpoInmovil() { return _inmovil;            }
bool  posturaErguida(){ return _posturaErguida;     }

int  bpmActual()      { return _bpmPromedio;        }
bool bpmElevado()     { return _bpmElevadoSostenido;}
bool dedoDetectado()  { return _dedo;               }

bool choqueCardiaco()    { return _choqueCardiaco && _dedo;   }
