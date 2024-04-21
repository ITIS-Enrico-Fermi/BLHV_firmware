#pragma once

#include <Wire.h>
#include <error.h>
#include "esp_log.h"

namespace dac {
    template<typename T>
    struct DAC {
        virtual void write(T) = 0;
    };

    class DAC8571 : public DAC<uint16_t> {
        static constexpr uint8_t ADDR = 0x4c;
        static constexpr uint8_t SET_CMD = 0x00;
        static constexpr uint8_t SET_UPDATE_CMD = 0x10;
        static constexpr uint8_t UPDATE_CMD = 0x20;
        
        private:
            uint8_t sdaPin, sclPin;
            TwoWire *i2c;

        public:
            inline DAC8571(uint8_t sdaPin, uint8_t sclPin, uint8_t busNum) :
                sdaPin(sdaPin), sclPin(sclPin) {
                i2c = new TwoWire(busNum);
                i2c->setPins(sdaPin, sclPin);
                i2c->setClock(100e3);
                i2c->begin();
            }

            inline DAC8571(TwoWire &i2c) : i2c(&i2c) {}

            
            inline void write(uint16_t val) override {
                // ESP_LOGI("DAC", "DAC writing: %d", val);
                printf("DAC writing: %d\n", val);
                
                i2c->beginTransmission(ADDR);
                i2c->write(SET_UPDATE_CMD);
                i2c->write((val & 0xff00) >> 8);
                i2c->write((val & 0x00ff) >> 0);
                
                if (i2c->endTransmission()) {
                    puts("DAC writing error");
                    errorHandler();
                }
            };
    };
}