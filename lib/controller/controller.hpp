#pragma once

#include <Wire.h>
#include <adc.hpp>
#include <dac.hpp>
#include <stdint.h>

namespace controls {
    template<typename T>
    struct Controller {
        virtual void setProcessVar(T procVar) = 0;
        virtual void setTarget(T target) = 0;
        virtual T getCompensation() = 0;
    };

    struct PID : public Controller<uint16_t> {
        private:
            adc::ADC<uint16_t> *adc;
            dac::DAC<uint16_t> *dac;
        
        public:
            PID(adc::ADC<uint16_t> *adc, dac::DAC<uint16_t> *dac) : adc(adc), dac(dac) {};

            void setProcessVar(uint16_t pv) {}
            void setTarget(uint16_t pv) {}

            uint16_t getCompensation() {
                // ATM just a mock
                static uint16_t i = 0;
                printf("%d\n", i);
                dac->write(i);
                i+=5;

                printf("ADC: %d\n", adc->read());
                return 0;
            }
    };
}