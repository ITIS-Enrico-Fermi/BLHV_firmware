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
            adc::ADC<T> *adc;
            dac::DAC<T> *dac;
            T target;
            T initialOutput;
        
        public:
            Controller(adc::ADC<T> *adc, dac::DAC<T> *dac) : adc(adc), dac(dac) {};
            inline void setOutput(T output) { this->initialOutput = output; }
            inline void setTarget(T target) { this->target = target; }
            virtual void forward(T val) { dac->write(val); }
            virtual T loopAsync() = 0;
    };

    struct PidTuning {
        float kp;
        float ki;
        float kd;
        float satMin; /** Saturation lower and upepr bound for anti-windup */
        float satMax;
        float compMin;  /** < Compensation lower and upper bound */
        float compMax;
    };

    class PID : public Controller<uint16_t> {
        private:
            PidTuning tuning;

        public:
            using Controller<uint16_t>::Controller;
            inline void setTuning(const PidTuning &tuning) {this->tuning = tuning; }
            inline void sampleTarget() {
                this->setTarget(this->adc->read());
            }

            uint16_t loopAsync() override {
                float processvar = processvar = clamp(
                    normalize(this->adc->read(), 0, 1 << 16),
                    0, 1
                );

                float setpoint = clamp(
                    normalize(this->target, 0, 1 << 16),
                    0, 1
                );
                
                float error = setpoint - processvar;
                static float integralError = 0;
                static float lastError = 0;
                static float output = normalize(this->initialOutput, 0, 1 << 16);
                
                float prop = this->tuning.kp * error;
                if (output > this->tuning.satMin && output < this->tuning.satMax) integralError += error;
                float integral = this->tuning.ki * integralError;
                float deriv = this->tuning.kd * (error - lastError);
                
                lastError = error;

                float compensation = clamp(
                    prop + integral + deriv,
                    this->tuning.compMin, this->tuning.compMax
                );
                
                output = clamp(
                    output + compensation,
                    0, 1
                );

                uint16_t dac_out = output * (1 << 16);

                // ESP_LOGI(
                //     "PID",
                //     "Setpoint: %.3f | Processvar: %.3f | Compensation: %.3f - %d",
                //     setpoint, processvar, output, dac_out
                // );

                printf(
                    "PID Setpoint: %.3f | Processvar: %.3f | Compensation: %.3f | Output: %.3f\n",
                    setpoint, processvar, compensation, output
                );

                this->dac->write(dac_out);

                return dac_out;
            }
    };
}