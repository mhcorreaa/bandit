// ============================================================ a
//  sketch_may14a.ino — MAIN
//  Pulsera Disautonomia | XIAO ESP32-C3
//  v3: máquina de estados OR | sin SpO2 | + filtro postural
//      + choque cardíaco | caída como contexto, no ruta independiente
// ============================================================

#include "config.h"
#include "sensores.h"
#include "botones.h"
#include <NimBLEDevice.h>

// ============================================================
//  PROTOTIPOS
// ============================================================
void entrarNormal();
void entrarAdvertencia(const char* motivo);
void entrarAlerta(const char* tipo);
void bleEnviarJson(const char* json);
void bleInit();
void motorInit();
void motorEncender();
void motorApagar();
void motorIniciarPulsos();
void motorTick();

// ============================================================
//  MOTOR
// ============================================================
static bool     _pulsoActual      = false;
static uint32_t _tUltimoPulso     = 0;
static bool     _modoIntermitente = false;

void motorInit() {
  pinMode(PIN_MOTOR, OUTPUT);
  digitalWrite(PIN_MOTOR, LOW);
}

void motorEncender() {
  _modoIntermitente = false;
  digitalWrite(PIN_MOTOR, HIGH);
}

void motorApagar() {
  _modoIntermitente = false;
  _pulsoActual      = false;
  digitalWrite(PIN_MOTOR, LOW);
}

void motorIniciarPulsos() {
  _modoIntermitente = true;
  _tUltimoPulso     = millis();
  _pulsoActual      = true;
  digitalWrite(PIN_MOTOR, HIGH);
}

void motorTick() {
  if (!_modoIntermitente) return;
  if (millis() - _tUltimoPulso >= INTERVALO_PULSO_MS) {
    _tUltimoPulso = millis();
    _pulsoActual  = !_pulsoActual;
    digitalWrite(PIN_MOTOR, _pulsoActual ? HIGH : LOW);
  }
}

// ============================================================
//  BLE — Nordic UART Service
// ============================================================
static bool                  _bleConectado = false;
static NimBLECharacteristic* _pTxChar      = nullptr;

class CBServidor : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
    _bleConectado = true;
    Serial.println("[BLE] App conectada");
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    _bleConectado = false;
    Serial.println("[BLE] Desconectada — reiniciando publicidad");
    NimBLEDevice::startAdvertising();
  }
};

class CBComando : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo&) override {
    String cmd = String(pChar->getValue().c_str());
    cmd.trim();
    Serial.print("[BLE RX] "); Serial.println(cmd);
    if      (cmd == "ACTIVAR")  entrarAdvertencia("MANUAL_BLE");
    else if (cmd == "CANCELAR") entrarNormal();
  }
};

void bleEnviarJson(const char* json) {
  if (!_bleConectado || !_pTxChar) return;
  _pTxChar->setValue(json);
  _pTxChar->notify();
  Serial.print("[BLE TX] "); Serial.println(json);
}

void bleInit() {
  NimBLEDevice::init(BLE_NOMBRE);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEServer* pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new CBServidor());

  NimBLEService* pSvc = pServer->createService(BLE_SERVICE_UUID);
  _pTxChar = pSvc->createCharacteristic(BLE_UUID_TX, NIMBLE_PROPERTY::NOTIFY);

  NimBLECharacteristic* pRx = pSvc->createCharacteristic(
    BLE_UUID_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pRx->setCallbacks(new CBComando());

  pSvc->start();
  NimBLEDevice::getAdvertising()->addServiceUUID(BLE_SERVICE_UUID);
  NimBLEDevice::startAdvertising();
  Serial.print("[BLE] Visible como '");
  Serial.print(BLE_NOMBRE);
  Serial.println("'");
}

// ============================================================
//  MÁQUINA DE ESTADOS
// ============================================================
static EstadoSistema _estado         = ESTADO_NORMAL;
static uint32_t      _tInicioAdvert  = 0;
static uint32_t      _tUltimoSegundo = 0;
static uint32_t      _tUltimoBpm     = 0;
static uint32_t      _tUltimoMPU     = 0;
static uint32_t      _tUltimoBle     = 0;

// Triple-click para cancelar falso positivo durante ADVERTENCIA
static byte           _cancelCount   = 0;
static uint32_t       _tUltimoCancel = 0;

// Contexto de caída:
// Cuando el MPU detecta una caída real, se registra el timestamp.
// Durante CAIDA_CONTEXTO_MS las rutas ópticas se evalúan sin
// requerir postura erguida. Si el tiempo expira sin anomalía
// cardíaca, la caída se descarta silenciosamente.
static uint32_t      _tUltimaCaida  = 0;   // 0 = sin caída reciente

inline bool _calidaEnContexto() {
  return _tUltimaCaida > 0 &&
         (millis() - _tUltimaCaida) < CAIDA_CONTEXTO_MS;
}

void entrarNormal() {
  _estado = ESTADO_NORMAL;
  motorApagar();
  audioDetener();
  Serial.println("[ESTADO] NORMAL");
}

void entrarAdvertencia(const char* motivo) {
  if (_estado == ESTADO_ADVERTENCIA) return;
  _estado         = ESTADO_ADVERTENCIA;
  _tInicioAdvert  = millis();
  _tUltimoSegundo = millis();
  _cancelCount    = 0;
  _tUltimoCancel  = 0;
  motorIniciarPulsos();
  Serial.print("[ESTADO] ADVERTENCIA motivo=");
  Serial.println(motivo);
  Serial.println("          → presiona CANCELAR x3 en 10s para descartar");
}

void entrarAlerta(const char* tipo) {
  if (_estado == ESTADO_ALERTA) return;
  _estado = ESTADO_ALERTA;
  motorEncender();
  audioReproducirBucle();
  bleEnviarJson("{\"alert\":1}");
  Serial.print("[ESTADO] ALERTA tipo="); Serial.println(tipo);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n================================");
  Serial.println("   Pulsera Disautonomia BOOT v2");
  Serial.println("================================");

  motorInit();
  botonesInit();
  audioInit();
  sensoresInit();
  bleInit();

  Serial.println("[BOOT] Listo\n");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {

  sensoresTick();
  motorTick();
  audioTick();

  // ── Envío BLE de BPM cada 1s ─────────────────────────
  if (millis() - _tUltimoBle >= 1000) {
    _tUltimoBle = millis();
    if (dedoDetectado() && bpmActual() > 0) {
      char json[24];
      snprintf(json, sizeof(json), "{\"bpm\":%d}", bpmActual());
      bleEnviarJson(json);
    }
  }

  // ── Reporte serial MPU cada 500ms ────────────────────
  if (millis() - _tUltimoMPU >= 500) {
    _tUltimoMPU = millis();
    float g = gActual();
    Serial.print("[MPU] G=");
    Serial.print(g, 2);
    Serial.print("g");
    Serial.print(cuerpoInmovil() ? "  INMOVIL" : "  movimiento");
    Serial.print(posturaErguida() ? "  ERGUIDO" : "  ACOSTADO");
    Serial.println();
  }

  // ── Reporte serial vitales cada 2s ───────────────────
  if (millis() - _tUltimoBpm >= 2000) {
    _tUltimoBpm = millis();

    float g = gActual();
    const char* estadoG;
    if      (g < ACCEL_CAIDA_LIBRE)     estadoG = "CAIDA LIBRE";
    else if (g > ACCEL_CAIDA_THRESHOLD) estadoG = "IMPACTO";
    else if (g < 0.8f || g > 1.2f)     estadoG = "movimiento";
    else                                estadoG = "reposo";

    Serial.print("[MPU] G="); Serial.print(g, 2);
    Serial.print("g  estado="); Serial.print(estadoG);
    Serial.print("  postura="); Serial.print(posturaErguida() ? "ERGUIDO" : "ACOSTADO");
    if (cuerpoInmovil()) Serial.print("  INMOVIL");
    Serial.println();

    if (dedoDetectado()) {
      Serial.print("[VIT] BPM="); Serial.print(bpmActual());
      if (bpmElevado())        Serial.print(" (ALTO-SOSTENIDO)");
      if (choqueCardiaco())    Serial.print(" [CHOQUE-CARDIACO]");
      Serial.println();
    } else {
      Serial.println("[VIT] Sin contacto en sensor de pulso");
    }
    Serial.println("---");
  }

  // ── Máquina de estados ────────────────────────────────
  switch (_estado) {

    case ESTADO_NORMAL: {

      // Alerta manual — botón rojo
      // Pasa por ADVERTENCIA para dar 10s de gracia al paciente
      // en caso de apriete accidental (bolsillo, deporte, etc.).
      // Si no cancela en 10s → ALERTA automática igual que las rutas sensor.
      if (botonAlertaPresionado()) {
        Serial.println("[BTN] Alerta manual — 10s para cancelar");
        entrarAdvertencia("MANUAL");
        break;
      }

      // ── Registro de caída como contexto ──────────────────
      // Una caída sola NO dispara advertencia (falso positivo deportivo).
      // Solo abre una ventana de CAIDA_CONTEXTO_MS durante la cual
      // las rutas ópticas se evalúan sin requerir postura erguida,
      // porque el paciente puede haber quedado tendido en el suelo.
      if (caidaDetectada()) {
        _tUltimaCaida = millis();
        Serial.println("[MPU] Caida registrada como contexto — esperando anomalia cardiaca");
      }

      // Expiración silenciosa del contexto de caída
      if (_tUltimaCaida > 0 && !_calidaEnContexto()) {
        Serial.println("[MPU] Contexto de caida expirado sin anomalia — descartado");
        _tUltimaCaida = 0;
      }

      // ── Evaluación de ruta óptica (choque cardíaco) ───────
      // Se activa si:
      //   a) paciente erguido (situación normal de síncope), O
      //   b) hay una caída reciente en contexto (relajamos postura
      //      porque puede estar en el suelo tras el colapso)
      //
      // En ambos casos se requiere dedo detectado.

      if (dedoDetectado() && (posturaErguida() || _calidaEnContexto())) {

        // Patrón de choque cardíaco:
        // Taquicardia compensatoria → bradicardia brusca en ≤15s.
        if (choqueCardiaco()) {
          const char* motivo = _calidaEnContexto() ? "CAIDA+CHOQUE_CARDIACO" : "CHOQUE_CARDIACO";
          Serial.print("[SENSOR] "); Serial.print(motivo);
          Serial.print(" | BPM="); Serial.println(bpmActual());
          _tUltimaCaida = 0;   // consumir contexto
          entrarAdvertencia(motivo);
          break;
        }
      }
      break;
    }

    case ESTADO_ADVERTENCIA: {

      // Triple-click para cancelar falso positivo
      if (botonCancelarPresionado()) {
        uint32_t ahora = millis();
        if (_cancelCount == 0 || (ahora - _tUltimoCancel) <= CANCEL_VENTANA_MS) {
          _cancelCount++;
          _tUltimoCancel = ahora;
          Serial.print("[ADVERTENCIA] Cancelar (");
          Serial.print(_cancelCount);
          Serial.print("/");
          Serial.print(CANCEL_PULSACIONES_REQUERIDAS);
          Serial.println(")");
          if (_cancelCount >= CANCEL_PULSACIONES_REQUERIDAS) {
            Serial.println("[ADVERTENCIA] Falso positivo confirmado");
            entrarNormal();
            break;
          }
        } else {
          _cancelCount   = 1;
          _tUltimoCancel = ahora;
          Serial.println("[ADVERTENCIA] Cancelar (1/3) — reiniciado por timeout");
        }
      }

      if (_cancelCount > 0 && (millis() - _tUltimoCancel) > CANCEL_VENTANA_MS) {
        _cancelCount = 0;
      }

      uint32_t t = millis() - _tInicioAdvert;
      if (t >= TIEMPO_ESPERA_ALERTA) {
        entrarAlerta("AUTO");
        break;
      }
      if (millis() - _tUltimoSegundo >= 1000) {
        _tUltimoSegundo = millis();
        Serial.print("[ADVERTENCIA] Alerta en ");
        Serial.print((TIEMPO_ESPERA_ALERTA - t) / 1000);
        Serial.println("s");
      }
      break;
    }

    case ESTADO_ALERTA:
      if (botonCancelarPresionado()) entrarNormal();
      break;
  }

  delay(20);
}
