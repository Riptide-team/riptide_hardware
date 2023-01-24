#ifndef BATTERY_CARD_HPP
#define BATTERY_CARD_HPP

#include <memory>
#include <mutex>
#include <string>

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include <rtac_asio/Stream.h>
#include <rtac_asio/SerialStream.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;


namespace riptide_hardware {

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class BatteryCardHardware : public hardware_interface::SensorInterface {

        public:
            BatteryCardHardware() {};

            CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            // Pointer to the serial stream
            rtac::asio::Stream::Ptr serial_;

            // Port of the serial communication
            std::string port_;

            // Baud rate of the serial communication
            uint32_t baud_rate_;

            // Tension of the battery
            double tension_;

            // Current of the battery
            double current_;

            // Buffer
            std::string buffer_;

            // Data mutex
            std::mutex data_mutex_;

            // Last tension
            double last_tension_;

            // Last current
            double last_current_;

            // Data available
            bool data_available_ = false;

            void serial_callback(rtac::asio::Stream::Ptr stream, std::string* data, const rtac::asio::SerialStream::ErrorCode& err, std::size_t count);
    };
} // riptide_hardware

#endif // BATTERY_CARD_HPP