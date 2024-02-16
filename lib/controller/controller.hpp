#pragma once

#include <Wire.h>
#include <adc.hpp>
#include <dac.hpp>
#include <stdint.h>

namespace controls {
    template<typename T>
    class Controller {
        protected:
            adc::ADC<uint16_t> *adc;
            dac::DAC<uint16_t> *dac;
        
        public:
            Controller(adc::ADC<uint16_t> *adc, dac::DAC<uint16_t> *dac) : adc(adc), dac(dac) {};
            void loopAsync();
    };

    struct PID : public Controller<uint16_t> {
        public:
            using Controller<uint16_t>::Controller;

            void loopAsync() {
                constexpr auto Inom = 20;  // Amp
                constexpr double Vscale = 10.f/12.f;

                // // ATM just a mock
                // static uint16_t i = 0;
                // printf("%d\n", i);
                // i += 1e3;

                dac->write(1);  // Testing DAC granularity

                printf("ADC: %d\n", adc->read());
            }
    };
}