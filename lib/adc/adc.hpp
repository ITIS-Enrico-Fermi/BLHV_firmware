#pragma once

#include <Wire.h>
#include <cassert>
#include "esp_log.h"


namespace adc {
    template<typename T>
    struct ADC {
        virtual T read() = 0;
    };

    class MCP3428 : public ADC<uint16_t> {
        static constexpr uint8_t ADDR = 0x68;

        enum class Channel {CH_1, CH_2, CH_3, CH_4};
        enum class Mode {ONE_SHOT, CONTINUOUS};
        enum class Resolution {BITS_12, BITS_14, BITS_16};
        enum class Gain {GAIN_1, GAIN_2, GAIN_4, GAIN_8};
        
        private:
            uint8_t sdaPin, sclPin;
            TwoWire *i2c;
        
        public:
            inline MCP3428(uint8_t sdaPin, uint8_t sclPin, uint8_t busNum) :
                sdaPin(sdaPin), sclPin(sclPin) {
                    i2c = new TwoWire(busNum);
                    i2c->setPins(sdaPin, sclPin);
                    i2c->begin();
            }

            inline MCP3428(TwoWire &i2c) : i2c(&i2c) {}

            inline bool config(Channel c, Mode m, Resolution r, Gain g) {
                const uint8_t CONF =
                    (1 << 7)  // RDY
                    | ((uint8_t) c << 5)
                    | ((uint8_t) m << 4)
                    | ((uint8_t) r << 2)
                    | (uint8_t) g;
                i2c->beginTransmission(ADDR);
                i2c->write(CONF);
                return !i2c->endTransmission();
            }

            inline uint16_t read() override {
                config(Channel::CH_1, Mode::ONE_SHOT, Resolution::BITS_16, Gain::GAIN_1);
                constexpr uint8_t BYTES = 3;  // MSB, LSB, config
                uint8_t bytes[BYTES];
                i2c->requestFrom(ADDR, BYTES);
                i2c->readBytes(bytes, BYTES);

                uint16_t val = (bytes[0] << 8) | bytes[1];

                // ESP_LOGI("ADC", "ADC reading: %d", val);
                printf("ADC reading: %d\n", val);
                return val;
            }
    };
}