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

// Audio
void audioInit();
void audioReproducirBucle();
void audioDetener();
void audioTick();
