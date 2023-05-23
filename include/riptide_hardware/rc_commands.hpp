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
    class RCCommands {
        public:
            RCCommands(const T time, const std::vector<uint16_t> commands) {
                time_ = time;
                commands_ = commands;
            };

            std::vector<double> GetCommands() const {
                std::vector<double> commands;
                std::transform(std::begin(commands_), std::end(commands_), std::back_inserter(commands), [](uint16_t v){ return (static_cast<double>(v) - 1500.) / 500.; });
                return commands;
            };

        private:
            // Time at last frame
            T time_;

            // RC Commands
            std::vector<uint16_t> commands_;
    };
} // namespace riptide_hardware