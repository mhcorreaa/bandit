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

// Audio
void audioInit();
void audioReproducirBucle();
void audioDetener();
void audioTick();
