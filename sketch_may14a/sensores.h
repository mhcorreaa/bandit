// ============================================================ a
//  sensores.h — Interfaz sensores y audio
//  v3: sin SpO2 | + filtro postural | + choque cardíaco
//      (colapso amplitud IR eliminado — no fiable con sensor
//       sin contacto directo en bíceps, ver notas en sensores.cpp)
// ============================================================
#pragma once

// Sensores
void  sensoresInit();
void  sensoresTick();

// Acelerómetro
bool  caidaDetectada();    // latcheada: se consume una sola vez al leer
float gActual();           // magnitud del vector aceleración (g)
bool  cuerpoInmovil();     // true si no hay movimiento hace >INMOVIL_TIEMPO_MS

// Postura
// true si el eje vertical del brazo (estimado por la proyección del
// vector gravedad) indica posición erguida (de pie o sentado).
// false si el paciente está acostado → suprime alertas por vitales.
bool  posturaErguida();

// Pulso
int   bpmActual();
bool  bpmElevado();        // taquicardia sostenida >BPM_ELEVADO_TIEMPO_MS
bool  dedoDetectado();

// ── Choque cardíaco ───────────────────────────────────────────
// Se registró taquicardia compensatoria (>BPM_CHOQUE_TAQUI) seguida
// de bradicardia (<BPM_CHOQUE_BRADI) en una ventana de BPM_CHOQUE_VENTANA_MS.
bool  choqueCardiaco();

// Audio
void audioInit();
void audioReproducirBucle();
void audioDetener();
void audioTick();
