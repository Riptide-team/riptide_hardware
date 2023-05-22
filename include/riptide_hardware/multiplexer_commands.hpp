#pragma once

#include <cstdint>
#include <utility>
#include <vector>

namespace riptide_hardware{
    template <class T>
    class MultiplexerCommands {
        public:
            MultiplexerCommands(const T time, const double able_control, const double remaining_time) {
                time_ = time;
                able_control_ = able_control;
                remaining_time_ = remaining_time;
            };

            std::vector<double> GetMultiplexerInfos() const {
                return std::vector<double>({able_control_, remaining_time_});
            }

        private:
            // Time at last frame
            T time_;

            // Thruster PWM us
            double able_control_;

            // D Fin PWM us
            double remaining_time_;
    };
} // namespace riptide_hardware