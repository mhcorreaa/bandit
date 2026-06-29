// ============================================================
//  config.h — Pines y constantes globales
//  Pulsera Disautonomia | XIAO ESP32-C3
// ============================================================
#pragma once

// ------------------------------------------------------------
//  PINES
// ------------------------------------------------------------
#define PIN_BOTON_ALERTA    D2   // Botón rojo
#define PIN_BOTON_CANCELAR  D1   // Botón negro
#define PIN_MOTOR           D0
#define PIN_DFPLAYER_RX     D7   // XIAO RX ← DFPlayer TX
#define PIN_DFPLAYER_TX     D6   // XIAO TX → DFPlayer RX (con 1kΩ)
// MPU6050 y MAX30102 comparten I2C: SDA=D4, SCL=D5

// ------------------------------------------------------------
//  PARÁMETROS DE DETECCIÓN
// ------------------------------------------------------------
#define BPM_ELEVADO             100
#define BPM_MINIMO_VALIDO        40
#define BPM_MAXIMO_VALIDO       200

// Caída — umbrales más exigentes para evitar falsos positivos
#define ACCEL_CAIDA_THRESHOLD     3.5f   // antes 2.0 — requiere impacto real, no movimiento brusco
#define ACCEL_CAIDA_LIBRE         0.3f   // antes 0.5 — solo caída libre real (<0.3g)

// Para que una caída cuente, la fase de caída libre (g < ACCEL_CAIDA_LIBRE)
// debe durar al menos este tiempo antes del impacto
#define CAIDA_LIBRE_MIN_MS        150    // 150ms en caída libre = caída real, no traspié

#define TIEMPO_ESPERA_ALERTA    10000
#define INTERVALO_PULSO_MS        500

// SpO2
#define SPO2_COEF_A              115.0f
#define SPO2_COEF_B                8.0f
#define SPO2_MINIMO               89    // Alerta si SpO2 < 89%
#define SPO2_TIEMPO_BAJO_MS     7000    // 7s consecutivos con SpO2 bajo

// BPM — para que sea "alteración constante" y no una lectura random,
// el BPM debe mantenerse elevado durante este tiempo
#define BPM_ELEVADO_TIEMPO_MS   5000   // 5s con BPM alto sostenido para considerarlo real

// Confirmación de falso positivo (triple-click en ADVERTENCIA)
#define CANCEL_PULSACIONES_REQUERIDAS  3
#define CANCEL_VENTANA_MS           2000

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
