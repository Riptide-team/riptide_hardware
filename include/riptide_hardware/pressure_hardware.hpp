#ifndef PRESSURE_HARDWARE
#define PRESSURE_HARDWARE

#include <string>

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include <ms5837_driver/ms5837_driver.hpp>

#include <memory>


namespace riptide_hardware {

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class PressureHardware : public hardware_interface::SensorInterface {

        public:
            PressureHardware() {};

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

            // Driver
            std::unique_ptr<MS5837Driver> driver_ = nullptr;
    };
} // riptide_hardware

#endif // SPARTON_AHRS_M1_HARDWARE