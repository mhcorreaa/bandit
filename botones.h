// ============================================================
//  botones.h — Struct y lógica de botones
//  Separado del .ino para que el IDE vea el struct antes
//  de generar prototipos automáticos
// ============================================================
#pragma once
#include <Arduino.h>
#include "config.h"

struct BotonState {
  int      pin;
  bool     anterior;
  uint32_t tCambio;
  bool     consumido;
};

static BotonState _btnAlerta = { PIN_BOTON_ALERTA,   HIGH, 0, false };
static BotonState _btnCancel = { PIN_BOTON_CANCELAR, HIGH, 0, false };

inline void botonesInit() {
  pinMode(PIN_BOTON_ALERTA,   INPUT_PULLUP);
  pinMode(PIN_BOTON_CANCELAR, INPUT_PULLUP);
}

inline bool _presionado(BotonState& b) {
  bool actual = digitalRead(b.pin);
  if (actual != b.anterior) {
    b.tCambio   = millis();
    b.anterior  = actual;
    b.consumido = false;
  }
  if (!b.consumido && (millis() - b.tCambio) > 50 && actual == LOW) {
    b.consumido = true;
    return true;
  }
  return false;
}

inline bool botonAlertaPresionado()   { return _presionado(_btnAlerta); }
inline bool botonCancelarPresionado() { return _presionado(_btnCancel); }