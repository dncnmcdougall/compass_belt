#include "HC595.h"

HC595::HC595(uint8_t DS_PIN, uint8_t SH_CP_PIN, uint8_t ST_CP_PIN)
    : _DS_PIN(DS_PIN)
    , _SH_CP_PIN(SH_CP_PIN)
    , _ST_CP_PIN(ST_CP_PIN)
    , _data(0)
{ }

void HC595::begin() {
    pinMode(_DS_PIN, OUTPUT);
    pinMode(_SH_CP_PIN, OUTPUT);
    pinMode(_ST_CP_PIN, OUTPUT);
}

void HC595::setData(uint8_t data) {
    _data = data;
}

void HC595::shift() const {
    digitalWrite(_ST_CP_PIN, LOW);
    digitalWrite(_SH_CP_PIN, LOW);
    shiftOut(_DS_PIN, _SH_CP_PIN, LSBFIRST, _data);
    digitalWrite(_ST_CP_PIN, HIGH);
    digitalWrite(_ST_CP_PIN, LOW);
}

uint8_t HC595::getData() const {
    return _data;
}
