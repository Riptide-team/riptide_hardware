#include "riptide_hardware/actuators_hardware.hpp"

#include <string>
#include <algorithm>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include "pololu_maestro_driver/pololu_maestro_driver.hpp"


namespace riptide_hardware {

    CallbackReturn ActuatorsHardware::on_init(const hardware_interface::HardwareInfo & info_) {
        if (hardware_interface::SystemInterface::on_init(info_) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

       // Checking port and baud rate specified in ros2_control urdf tag
        if (info_.hardware_parameters.find("port") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("ActuatorsHardware"),
                "You need to specify the serial port in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info_.hardware_parameters.find("baud_rate") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("ActuatorsHardware"),
                "You need to specify the serial baud rate in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Getting port and baud_rate parameters
        port_ = info_.hardware_parameters.at("port");
        baud_rate_ = stoi(info_.hardware_parameters.at("baud_rate"));
        
        RCLCPP_INFO(rclcpp::get_logger("ActuatorsHardware"), "port: %s - baud rate: %d", port_.c_str(), baud_rate_);

        hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        RCLCPP_INFO(
            rclcpp::get_logger("ActuatorsHardware"),
            "Serial communication: %s:%i", port_.c_str(), baud_rate_
        );

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ActuatorsHardware::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // export sensor state interface
        for (uint i = 0; i < info_.joints.size(); ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, info_.joints[i].state_interfaces[0].name, &hw_states_positions_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ActuatorsHardware::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // export command state interface
        for (uint i = 0; i < info_.joints.size(); ++i) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, info_.joints[i].state_interfaces[0].name, &hw_commands_positions_[i]));
        }

        return command_interfaces;
    }

    CallbackReturn ActuatorsHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_DEBUG(rclcpp::get_logger("ActuatorsHardware"), "Activating ...please wait...");
        for (unsigned int i=0; i<hw_states_positions_.size(); ++i) {
            hw_states_positions_[i] = 0;
        }
        for (unsigned int i=0; i<hw_commands_positions_.size(); ++i) {
            hw_commands_positions_[i] = 0;
        }
        RCLCPP_DEBUG(rclcpp::get_logger("ActuatorsHardware"), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn ActuatorsHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_DEBUG(rclcpp::get_logger("ActuatorsHardware"), "Deactivating ...please wait...");
        RCLCPP_DEBUG(rclcpp::get_logger("ActuatorsHardware"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ActuatorsHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        RCLCPP_INFO(rclcpp::get_logger("ActuatorsHardware"), "Read");

        uint16_t position;
        driver_->GetPosition(0, position);
        RCLCPP_INFO(rclcpp::get_logger("ActuatorsHardware"), "Get position 0");
        hw_states_positions_[0] = (position - 1500) / 500;
        for (std::size_t channel=1; channel<4; ++channel) {
            driver_->GetPosition(channel, position);
            RCLCPP_INFO(rclcpp::get_logger("ActuatorsHardware"), "Get position %d", channel);
            hw_states_positions_[channel] = M_PI * (position - 1500) / 1000;
        };

        RCLCPP_INFO(
            rclcpp::get_logger("ActuatorsHardware"),
            "Reading positions: %f, %f, %f, %f",
            hw_states_positions_[0], hw_states_positions_[1],
            hw_states_positions_[2], hw_states_positions_[3]
        );

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ActuatorsHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        RCLCPP_INFO(rclcpp::get_logger("ActuatorsHardware"), "Write");

        uint16_t positions[4];

        positions[0] = uint16_t(500 * std::clamp(hw_commands_positions_[0], -1., 1.) + 1500);
        for (std::size_t i=1; i<4; ++i) {
            positions[i] = uint16_t(1000 * std::clamp(double(hw_commands_positions_[i]), -M_PI_2, M_PI_2) / M_PI + 1500);
        }
        
        driver_->SetMultiplePositions(4, 0, positions);

        return hardware_interface::return_type::OK;
    }
} // riptide_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_hardware::ActuatorsHardware, hardware_interface::SystemInterface)