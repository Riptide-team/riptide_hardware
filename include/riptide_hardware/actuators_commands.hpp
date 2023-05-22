#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <math.h>
#include <mutex>
#include <utility>
#include <vector>
#include <variant>

namespace riptide_hardware{
    template <class T>
    class ActuatorsCommands {
        public:
            ActuatorsCommands(const T time, const std::vector<uint16_t> commands) {
                time_ = time;
                thruster_us_ = std::max(std::min(commands[0], (uint16_t)2000), (uint16_t)1000);
                d_fin_us_ = std::max(std::min(commands[1], (uint16_t)2000), (uint16_t)1000);
                p_fin_us_ = std::max(std::min(commands[2], (uint16_t)2000), (uint16_t)1000);
                s_fin_us_ = std::max(std::min(commands[3], (uint16_t)2000), (uint16_t)1000);
            };

            // Thruster command getters 
            double Thruster() const {
                return ((double)thruster_us_ - 1500.) / 500.;
            };
            
            // D Fin command getter
            double DFinAngle() const {
                return M_PI * ((double)d_fin_us_ - 1500.) / 2000.;
            };

            // P Fin command getter
            double PFinAngle() const {
                return M_PI * ((double)p_fin_us_ - 1500.) / 2000.;
            };

            // S Fin command getter
            double SFinAngle() const {
                return M_PI * ((double)s_fin_us_ - 1500.) / 2000.;
            };

        private:
            // Time at last frame
            T time_;

            // Thruster PWM us
            uint16_t thruster_us_;

            // D Fin PWM us
            uint16_t d_fin_us_;

            // P Fin PWM us
            uint16_t p_fin_us_;

            // S Fin PWM us
            uint16_t s_fin_us_;
    };
} // namespace riptide_hardware