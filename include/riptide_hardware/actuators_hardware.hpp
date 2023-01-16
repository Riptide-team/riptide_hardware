#ifndef ACTUATORS_HARDWARE_HPP
#define ACTUATORS_HARDWARE_HPP

#include <memory>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "pololu_maestro_driver/pololu_maestro_driver.hpp"


namespace riptide_hardware {

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class ActuatorsHardware : public hardware_interface::SystemInterface {

        public:
            ActuatorsHardware() {};

            CallbackReturn on_init(const hardware_interface::HardwareInfo & info_) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces()override;

            CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

            hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

        private:
            std::vector<double> hw_states_positions_;
            std::vector<double> hw_commands_positions_;

            // Serial port
            std::string port_;

            // Serial baud rate
            unsigned int baud_rate_;

            // Driver
            std::unique_ptr<PololuMaestroDriver> driver_ = nullptr;
    };
} // riptide_hardware

#endif // ACTUATORS_HARDWARE_HPP