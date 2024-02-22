#pragma once

#include <Wire.h>
#include <adc.hpp>
#include <dac.hpp>
#include <utils.h>
#include <stdint.h>
#include "esp_log.h"

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
        private:
            float target = 0.318; // < target current measured in ADC units
            struct {
                float kp;
                float ki;
                float kd;
                float sat_min; /** Saturation lower bound */
                float sat_max; /** Saturation upper bound */
            } pid_tuning = {
                .kp = 0.1,
                .ki = 0.004,
                .kd = 0,
                .sat_min = -0.235,
                .sat_max = 0.235
            };
        
        public:
            using Controller<uint16_t>::Controller;

            void holdAsync(uint16_t _setpoint) {
                // constexpr auto Inom = 20;  // Amp
                // constexpr double Vscale = 10.f/12.f;
                // dac->write(target);  // Testing DAC granularity

                float processvar = adc->read();
                // float setpoint = clamp(
                //     normalize(_setpoint, 0, 1 << 16),
                //     0, 1
                // );
                float setpoint = target;

                processvar = clamp(
                    normalize(processvar, 0, 1 << 15),
                    0, 1
                );

                float error = setpoint - processvar; // < e(t) = error at current time
                static float integralerror = 0;
                static float lasterror = 0;
                static float output = 0;

                // Proportional component
                float prop = pid_tuning.kp * error;

                // integral component
                if (output > pid_tuning.sat_min && output < pid_tuning.sat_max) {
                    integralerror += error;
                }
                float integral = pid_tuning.ki * integralerror;

                // Derivative component
                float deriv = pid_tuning.kd * (error - lasterror);
                lasterror = error;

                output = prop + integral + deriv;
                output = clamp(output, pid_tuning.sat_min, pid_tuning.sat_max);

                float _out = clamp(output, 0, 1);
                uint16_t dac_out = _out * (1 << 16);

                // ESP_LOGI(
                //     "PID",
                //     "Setpoint: %.3f | Processvar: %.3f | Compensation: %.3f - %d",
                //     setpoint, processvar, output, dac_out
                // );

                printf(
                    "PID Setpoint: %.3f | Processvar: %.3f | Compensation: %.3f %.3f - %d\n",
                    setpoint, processvar, output, _out, dac_out
                );

                dac->write(dac_out);
            }

            void rampAsync(uint16_t val) {
                dac->write(val);
            }
    };
}