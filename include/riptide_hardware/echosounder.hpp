#ifndef ECHOSOUNDER_HARDWARE
#define ECHOSOUNDER_HARDWARE

#include <string>

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include <memory>

#include <rtac_asio/Stream.h>
#include <rtac_asio/SerialStream.h>


namespace riptide_hardware {

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class EchosounderHardware : public hardware_interface::SensorInterface {

        public:
            EchosounderHardware() {};

            CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            // Exported sensor states
            std::vector<double> hw_sensor_states_;

            // Serial port
            std::string port_;

            // Serial baud rate
            unsigned int baud_rate_;

            // Driver
            rtac::asio::Stream::Ptr serial_ = nullptr;
    };
} // riptide_hardware

#endif // ECHOSOUNDER_HARDWARE