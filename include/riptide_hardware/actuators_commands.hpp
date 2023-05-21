#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <math.h>
#include <vector>

namespace riptide_hardware{
    class ActuatorsCommands {
        public:
            ActuatorsCommands();

            // Commands setter
            void SetCommands(const double time, const uint16_t thruster_us, const uint16_t d_fin_us, const uint16_t p_fin_us, const int16_t s_fin_us) {
                data_consumable_ = true;
                time_ = time;
                thruster_us_ = std::max(std::min(thruster_us, (uint16_t)2000), (uint16_t)1000);
                d_fin_us_ = std::max(std::min(d_fin_us, (uint16_t)2000), (uint16_t)1000);
                p_fin_us_ = std::max(std::min(p_fin_us, (uint16_t)2000), (uint16_t)1000);
                s_fin_us_ = std::max(std::min(s_fin_us, (uint16_t)2000), (uint16_t)1000);
            };

            void SetCommands(const std::chrono::system_clock::time_point time, const std::vector<uint16_t> commands) {
                data_consumable_ = true;
                time_ = time;
                unavailable_counter = 0;
                thruster_us_ = std::max(std::min(commands[0], (uint16_t)2000), (uint16_t)1000);
                d_fin_us_ = std::max(std::min(commands[1], (uint16_t)2000), (uint16_t)1000);
                p_fin_us_ = std::max(std::min(commands[2], (uint16_t)2000), (uint16_t)1000);
                s_fin_us_ = std::max(std::min(commands[3], (uint16_t)2000), (uint16_t)1000);
            };

            std::vector<double> GetCommands() {
                std::vector<double> commands;
                if (data_consumable_) {
                    commands.push_back((thruster_us_ - 1500.) / 500.);
                    commands.push_back(M_PI *  (d_fin_us_ - 1500.) / 2000.);
                    commands.push_back(M_PI *  (p_fin_us_ - 1500.) / 2000.);
                    commands.push_back(M_PI *  (s_fin_us_ - 1500.) / 2000.);
                    data_consumable_ = false;
                }
                else {
                    unavailable_counter ++;
                    return commands;
                }
                return commands;
            };

            // D Fin command getter
            double DFinAngle() const {
                return M_PI *  (d_fin_us_ - 1500.) / 2000.;
            };

            // P Fin command getter
            double PFinAngle() const {
                return M_PI *  (p_fin_us_ - 1500.) / 2000.;
            };

            // S Fin command getter
            double SFinAngle() const {
                return M_PI *  (s_fin_us_ - 1500.) / 2000.;
            };

        private:
            // Check if data is consumable or already consumed
            bool data_consumable_ = false;

            // Time at last frame
            std::chrono::system_clock::time_point time_;

            // Unavailable counter
            uint16_t unavailable_counter;

            // Thruster PWM us
            double thruster_us_;

            // D Fin PWM us
            double d_fin_us_;

            // P Fin PWM us
            double p_fin_us_;

            // S Fin PWM us
            double s_fin_us_;
    }
} // namespace riptide_hardware