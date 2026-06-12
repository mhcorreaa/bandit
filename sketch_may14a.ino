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
void entrarAlerta();
void bleEnviar(const char* msg);
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
    Serial.println("[BLE] Tablet conectada");
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
    if      (cmd == "ACTIVAR")  entrarAlerta();
    else if (cmd == "CANCELAR") entrarNormal();
  }
};

void bleEnviar(const char* msg) {
  if (!_bleConectado || !_pTxChar) return;
  _pTxChar->setValue(msg);
  _pTxChar->notify();
  Serial.print("[BLE TX] "); Serial.println(msg);
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

void entrarNormal() {
  _estado = ESTADO_NORMAL;
  motorApagar();
  audioDetener();
  bleEnviar("ALERTA_CANCELADA");
  Serial.println("[ESTADO] NORMAL");
}

void entrarAdvertencia() {
  if (_estado == ESTADO_ADVERTENCIA) return;
  _estado         = ESTADO_ADVERTENCIA;
  _tInicioAdvert  = millis();
  _tUltimoSegundo = millis();
  motorIniciarPulsos();
  bleEnviar("ADVERTENCIA_CAIDA");
  Serial.println("[ESTADO] ADVERTENCIA — presiona CANCELAR en 10s");
}

void entrarAlerta() {
  if (_estado == ESTADO_ALERTA) return;
  _estado = ESTADO_ALERTA;
  motorEncender();
  audioReproducirBucle();
  bleEnviar("ALERTA_ACTIVA");
  Serial.println("[ESTADO] ALERTA — motor ON, audio ON");
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

  // Reporte BPM cada 2s
  if (millis() - _tUltimoBpm >= 2000) {
    _tUltimoBpm = millis();
    if (dedoDetectado()) {
      Serial.print("[BPM] ");
      Serial.print(bpmActual());
      Serial.println(bpmElevado() ? " bpm ELEVADO" : " bpm");
    } else {
      Serial.println("[BPM] Sin dedo en MAX30102");
    }
  }

  switch (_estado) {

    case ESTADO_NORMAL:
      if (botonAlertaPresionado()) { entrarAlerta(); break; }
      if (caidaDetectada() && bpmElevado()) {
        Serial.print("[SENSOR] Caida + BPM: ");
        Serial.println(bpmActual());
        entrarAdvertencia();
      }
      break;

    case ESTADO_ADVERTENCIA: {
      if (botonCancelarPresionado()) { entrarNormal(); break; }
      uint32_t t = millis() - _tInicioAdvert;
      if (t >= TIEMPO_ESPERA_ALERTA) { entrarAlerta(); break; }
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
