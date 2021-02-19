
#ifndef _74H595_H_
#define _74H595_H_

#include "Arduino.h"

class HC595 {
public:
    HC595(uint8_t DS_PIN, uint8_t SH_CP_PIN, uint8_t ST_CP_PIN);

    void begin();
    void setData(uint8_t data);
    void shift() const;
    uint8_t getData() const;

private:
    uint8_t _DS_PIN;
    uint8_t _SH_CP_PIN;
    uint8_t _ST_CP_PIN;
    uint8_t _data;
};

#endif
