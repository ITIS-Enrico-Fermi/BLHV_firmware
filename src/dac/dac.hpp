#pragma once

#include <Wire.h>


namespace dac {
    class DAC8571 {
        static constexpr uint8_t ADDR = 0x4c;
        static constexpr uint8_t SET_CMD = 0x00;
        static constexpr uint8_t SET_UPDATE_CMD = 0x10;
        static constexpr uint8_t UPDATE_CMD = 0x20;
        
        private:
            uint8_t sdaPin, sclPin;
            TwoWire i2c;

        public:
            inline DAC8571(uint8_t sdaPin, uint8_t sclPin, uint8_t busNum) :
                sdaPin(sdaPin), sclPin(sclPin), i2c(TwoWire(busNum)) {
                i2c.setPins(sdaPin, sclPin);
                i2c.begin();
            }
            
            inline bool dacWrite(uint16_t val) {
                i2c.beginTransmission(ADDR);
                i2c.write(SET_UPDATE_CMD);
                i2c.write(val & 0xff00 >> 8);
                i2c.write(val & 0x00ff >> 0);
                i2c.endTransmission();
                return true;
            };
    };
}