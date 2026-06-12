// ============================================================
//  sensores.h — Interfaz sensores y audio
// ============================================================
#pragma once

// Sensores
void sensoresInit();
void sensoresTick();
bool caidaDetectada();
int  bpmActual();
bool bpmElevado();
bool dedoDetectado();
int  spo2Actual();   // 0 = aún no disponible
bool spo2Bajo();     // true si SpO2 < 95%
bool cuerpoInmovil();    // true si no hay movimiento hace >3s
bool bpmCaidaBrusca();   // true si BPM cayó >25 en 5s
bool sospechaDesmayo();  // SpO2 bajo + caída brusca de BPM (pre-síncope)

// Audio
void audioInit();
void audioReproducirBucle();
void audioDetener();
void audioTick();
