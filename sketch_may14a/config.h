// ============================================================ a
//  config.h — Pines y constantes globales
//  Pulsera Disautonomia | XIAO ESP32-C3
//  v3: + postura | + choque cardíaco | sin SpO2 | sin colapso IR
// ============================================================
#pragma once

// ------------------------------------------------------------
//  PINES
// ------------------------------------------------------------
#define PIN_BOTON_ALERTA    D2
#define PIN_BOTON_CANCELAR  D1
#define PIN_MOTOR           D0
#define PIN_DFPLAYER_RX     D7   // XIAO RX ← DFPlayer TX
#define PIN_DFPLAYER_TX     D6   // XIAO TX → DFPlayer RX (con 1kΩ)
// MPU6050 y MAX30102 comparten I2C: SDA=D4, SCL=D5

// ------------------------------------------------------------
//  PARÁMETROS DE DETECCIÓN — ACELERÓMETRO / CAÍDA
// ------------------------------------------------------------
#define BPM_ELEVADO             100
#define BPM_MINIMO_VALIDO        40
#define BPM_MAXIMO_VALIDO       200

#define ACCEL_CAIDA_THRESHOLD     3.5f
#define ACCEL_CAIDA_LIBRE         0.3f
#define CAIDA_LIBRE_MIN_MS        150

// Ventana de contexto post-caída:
// Durante este tiempo después de una caída confirmada, la ruta
// de choque cardíaco se evalúa SIN requerir postura erguida
// (el paciente puede haber quedado en el suelo).
// Si expira sin anomalía cardíaca → la caída se descarta como benigna.
#define CAIDA_CONTEXTO_MS        20000   // 20s

// ------------------------------------------------------------
//  FILTRO POSTURAL
//
//  La pulsera va en el bíceps. Con el brazo colgando:
//    eje Y del sensor ≈ eje longitudinal del brazo (vertical al estar de pie)
//
//  Erguido: |ay_filtrado| > POSTURA_UMBRAL_VERTICAL
//           y proyección horizontal < POSTURA_UMBRAL_HORIZONTAL
//  Acostado: la gravedad se reparte en ax y az
//
//  Ajusta si el sensor está montado en orientación diferente.
// ------------------------------------------------------------
#define POSTURA_UMBRAL_VERTICAL   0.65f  // g   — tolerancia ±40° de la vertical
#define POSTURA_UMBRAL_HORIZONTAL 0.75f  // g   — descarta posición lateral/decúbito

// NOTA: el detector de colapso de amplitud IR (AMPL_COLAPSO_*) fue
// eliminado. En pruebas con el sensor en el bíceps (sin contacto directo,
// ~1mm de la piel), la amplitud absoluta mostraba saltos de escalón
// mecánicos no relacionados a perfusión real. El choque cardíaco
// (basado en BPM, no en magnitud de señal) es la única ruta óptica.

// ------------------------------------------------------------
//  CHOQUE CARDÍACO
//
//  Patrón: taquicardia compensatoria → bradicardia brusca en ≤15s
//  Representa el colapso del mecanismo compensatorio (pre-síncope vasovagal
//  o shock cardíaco).
// ------------------------------------------------------------
#define BPM_CHOQUE_TAQUI             110   // umbral de taquicardia compensatoria
#define BPM_CHOQUE_BRADI              60   // umbral de caída a bradicardia
#define BPM_CHOQUE_TAQUI_CONFIRMA_MS 3000  // la taqui debe sostenerse 3s antes de contar
#define BPM_CHOQUE_VENTANA_MS       15000  // ventana máxima taqui→bradi (15s)

// ------------------------------------------------------------
//  TEMPORIZACIÓN
// ------------------------------------------------------------
#define TIEMPO_ESPERA_ALERTA    10000
#define INTERVALO_PULSO_MS        500

// BPM elevado sostenido
#define BPM_ELEVADO_TIEMPO_MS   5000

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
