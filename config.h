// ============================================================
//  config.h — Pines y constantes globales
//  Pulsera Disautonomia | XIAO ESP32-C3
// ============================================================
#pragma once

// ------------------------------------------------------------
//  PINES
// ------------------------------------------------------------
#define PIN_BOTON_ALERTA    D2
#define PIN_BOTON_CANCELAR  D6
#define PIN_MOTOR           D3
#define PIN_DFPLAYER_TX     D5   // XIAO TX → 1kΩ → DFPlayer RX
#define PIN_DFPLAYER_RX     D4   // DFPlayer TX → 1kΩ → XIAO RX
// MAX30102 y MPU6050 comparten I2C: SDA=D7, SCL=D8

// ------------------------------------------------------------
//  PARÁMETROS DE DETECCIÓN
// ------------------------------------------------------------
#define BPM_ELEVADO             100    // BPM considerado elevado
#define BPM_MINIMO_VALIDO        40
#define BPM_MAXIMO_VALIDO       200
#define ACCEL_CAIDA_THRESHOLD     2.5f
#define ACCEL_CAIDA_LIBRE         0.3f
#define TIEMPO_ESPERA_ALERTA    10000  // ms antes de alerta automática
#define INTERVALO_PULSO_MS        500  // ms entre pulsos de vibración

// ------------------------------------------------------------
//  BLE
// ------------------------------------------------------------
#define BLE_NOMBRE       "Pulsera_Disautonomia"
#define BLE_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_UUID_RX      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_UUID_TX      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ------------------------------------------------------------
//  ESTADOS
// ------------------------------------------------------------
enum EstadoSistema {
  ESTADO_NORMAL,
  ESTADO_ADVERTENCIA,
  ESTADO_ALERTA
};
