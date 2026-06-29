// ============================================================
//  sketch_may14a.ino — MAIN
//  Pulsera Disautonomia | XIAO ESP32-C3
// ============================================================

#include "config.h"
#include "sensores.h"
#include "botones.h"
#include <NimBLEDevice.h>

// ============================================================
//  PROTOTIPOS
// ============================================================
void entrarNormal();
void entrarAdvertencia();
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
    if      (cmd == "ACTIVAR")  entrarAlerta("MANUAL");
    else if (cmd == "CANCELAR") entrarNormal();
  }
};

// Envía un JSON por BLE notify
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
static uint32_t      _tUltimoBpm     = 0;  // reporte serial cada 2s
static uint32_t      _tUltimoMPU     = 0;
static uint32_t      _tUltimoBle     = 0;  // envío BLE cada 1s

// Triple-click para cancelar falso positivo durante ADVERTENCIA
static byte           _cancelCount    = 0;
static uint32_t       _tUltimoCancel  = 0;

void entrarNormal() {
  _estado = ESTADO_NORMAL;
  motorApagar();
  audioDetener();
  Serial.println("[ESTADO] NORMAL");
}

void entrarAdvertencia() {
  if (_estado == ESTADO_ADVERTENCIA) return;
  _estado         = ESTADO_ADVERTENCIA;
  _tInicioAdvert  = millis();
  _tUltimoSegundo = millis();
  _cancelCount    = 0;
  _tUltimoCancel  = 0;
  motorIniciarPulsos();
  Serial.println("[ESTADO] ADVERTENCIA — presiona CANCELAR en 10s");
}

// tipo = "MANUAL" o "AUTO" — solo para log serial, la app recibe siempre {"alert":1}
void entrarAlerta(const char* tipo) {
  if (_estado == ESTADO_ALERTA) return;
  _estado = ESTADO_ALERTA;
  motorEncender();
  audioReproducirBucle();

  // JSON de alerta → app: 1 = alerta activa
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
  Serial.println("   Pulsera Disautonomia BOOT");
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
  // Solo si hay dedo detectado y hay conexión BLE activa
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
    if (cuerpoInmovil()) Serial.print(" (inmovil)");
    else Serial.print(" (normal)");
    Serial.println();
  }

  // ── Reporte serial vitales cada 2s ───────────────────
  if (millis() - _tUltimoBpm >= 2000) {
    _tUltimoBpm = millis();

    float g = gActual();
    const char* estadoG;
    if      (g < ACCEL_CAIDA_LIBRE)       estadoG = "CAIDA LIBRE";
    else if (g > ACCEL_CAIDA_THRESHOLD)   estadoG = "IMPACTO";
    else if (g < 0.8f || g > 1.2f)        estadoG = "movimiento";
    else                                   estadoG = "reposo";

    Serial.print("[MPU] G=");
    Serial.print(g, 2);
    Serial.print("g  estado=");
    Serial.print(estadoG);
    if (cuerpoInmovil()) Serial.print("  INMOVIL");
    Serial.println();

    if (dedoDetectado()) {
      Serial.print("[VIT] BPM=");
      Serial.print(bpmActual());
      if (bpmElevado()) Serial.print("(ALTO-SOSTENIDO)");
      Serial.print("  SpO2=");
      if (spo2Actual() > 0) {
        Serial.print(spo2Actual());
        Serial.print("%");
        if (spo2Bajo()) Serial.print("(BAJO)");
      } else {
        Serial.print("--");
      }
      Serial.println();
    } else {
      Serial.println("[VIT] Sin contacto en sensor de pulso");
    }
    Serial.println("---");
  }

  // ── Máquina de estados ────────────────────────────────
  switch (_estado) {

    case ESTADO_NORMAL:

      // Alerta manual — botón rojo
      if (botonAlertaPresionado()) {
        Serial.println("[BTN] Alerta manual");
        entrarAlerta("MANUAL");
        break;
      }

      // Ruta A: caída confirmada + BPM elevado sostenido + SpO2 bajo + dedo detectado
      if (dedoDetectado() && caidaDetectada() && bpmElevado() && spo2Bajo()) {
        Serial.print("[SENSOR] Caida + BPM alto sostenido + SpO2 bajo | BPM=");
        Serial.print(bpmActual());
        Serial.print(" SpO2="); Serial.print(spo2Actual()); Serial.println("%");
        entrarAdvertencia();
        break;
      }

      // Ruta B: estático + BPM elevado sostenido + SpO2 bajo + dedo detectado
      if (dedoDetectado() && cuerpoInmovil() && bpmElevado() && spo2Bajo()) {
        Serial.print("[SENSOR] Estatico + BPM alto sostenido + SpO2 bajo | BPM=");
        Serial.print(bpmActual());
        Serial.print(" SpO2="); Serial.print(spo2Actual()); Serial.println("%");
        entrarAdvertencia();
        break;
      }
      break;

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
        entrarAlerta("AUTO");   // pasaron 10s sin cancelar → alerta automática
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
