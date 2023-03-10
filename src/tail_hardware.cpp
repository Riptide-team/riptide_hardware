#include "riptide_hardware/tail_hardware.hpp"

#include <algorithm>
#include <mutex>
#include <string>
#include <sstream>
#include <iomanip>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include <rtac_asio/Stream.h>
#include <rtac_asio/SerialStream.h>


namespace riptide_hardware {

    std::size_t TailHardware::serial_write_actuators_commands() {
        RCLCPP_DEBUG(rclcpp::get_logger("TailHardware"),
            "Writing %f %f %f %f", hw_commands_positions_[0],
            hw_commands_positions_[1], hw_commands_positions_[2],
            hw_commands_positions_[3]
        );

        // Transforming double in [-1, 1] and [-pi/3, pi/3] to uint16_t in [1000, 2000] us
        std::uint16_t value;
        std::uint8_t commands[10];
        for (std::size_t i=0; i<4; ++i) {
            if (i==0) {
                value = static_cast<std::uint16_t>(500 * std::clamp(hw_commands_positions_[0], -1., 1.) + 1500);
            }
            else {
                value = static_cast<std::uint16_t>(1500 * std::clamp(double(hw_commands_positions_[i]), -M_PI/3., M_PI/3.) / M_PI + 1500);
            }

            // Correcting endian before sending to Arduino
            std::reverse(
                reinterpret_cast<char*>(&value),
                reinterpret_cast<char*>(&value) + sizeof(value)
            );

            std::memcpy(commands + 2*i, &value, sizeof(std::uint16_t));
        }

        commands[9] = '\n';
        std::size_t n = serial_->write(9, commands, write_timeout_);
        return n;
    }

    void TailHardware::read_callback(const rtac::asio::SerialStream::ErrorCode& /*err*/, std::size_t count) {
        // Storage of the received time
        time_last_received_ = time_read_;

        // Getting data with a mutex lock
        {
            std::lock_guard<std::mutex> lock_(m_read_);
            data_consumable_ = true;
            if((count == 21) and (read_buffer_[20] == '\n')) {
                // Correcting endian for incoming values from Arduino
                for (std::size_t i=0; i<20; ++i) {
                    std::reverse(
                        reinterpret_cast<char*>(&read_buffer_[i]),
                        reinterpret_cast<char*>(&read_buffer_[i]) + sizeof(read_buffer_[i])
                    );
                }

                // Getting values
                for(std::size_t i=0; i < 10; ++i) {
                    std::uint16_t value = (static_cast<std::uint8_t>(read_buffer_[2*i]) << 8) + static_cast<std::uint8_t>(read_buffer_[2*i+1]);
                    read_data_[i] = value;
                }
            }
            else {
                if (count <= 0) {
                    RCLCPP_WARN(
                        rclcpp::get_logger("TailHardware"),
                        "Got no data (timeout probably reached)!"
                    );
                }
                else {
                    RCLCPP_WARN(
                        rclcpp::get_logger("TailHardware"),
                        "Bad number of data, the frame was probably not complete!"
                    );
                }
            }
        }

        std::stringstream ss;
        for (unsigned int i=0; i<21; ++i) {
            ss << std::setw(2) << std::setfill('0') << std::hex << static_cast<std::int8_t>(read_buffer_[i]) << " ";
        }

        RCLCPP_DEBUG_STREAM(
            rclcpp::get_logger("TailHardware"),
            ss.str()
        );

        // Relaunching an async read
        if(!serial_->async_read_until(read_buffer_.size(), (uint8_t*)read_buffer_.c_str(), '\n', std::bind(&TailHardware::read_callback, this, std::placeholders::_1, std::placeholders::_2))) {
            RCLCPP_FATAL(
                rclcpp::get_logger("TailHardware"),
                "Async read until could not start !"
            );
        }
    }

    CallbackReturn TailHardware::on_init(const hardware_interface::HardwareInfo & info_) {
        if (hardware_interface::SystemInterface::on_init(info_) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

       // Checking port and baud rate specified in ros2_control urdf tag
        if (info_.hardware_parameters.find("port") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("TailHardware"),
                "You need to specify the serial port in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info_.hardware_parameters.find("baud_rate") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("TailHardware"),
                "You need to specify the serial baud rate in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Getting port and baud_rate parameters
        port_ = info_.hardware_parameters.at("port");
        baud_rate_ = stoi(info_.hardware_parameters.at("baud_rate"));
        
        RCLCPP_INFO(rclcpp::get_logger("TailHardware"), "port: %s - baud rate: %d", port_.c_str(), baud_rate_);

        hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_positions_.resize(info_.joints.size()+info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());

        RCLCPP_INFO(
            rclcpp::get_logger("TailHardware"),
            "Serial communication: %s:%i", port_.c_str(), baud_rate_
        );

        write_timeout_ = 1000; // 1000 ms of timeot to write commands

        duration_expiration_ = rclcpp::Duration(1, 0); // 1 s, 0 ns

        data_consumable_ = false;

        read_data_.resize(10);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> TailHardware::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // export actuators state interfaces
        for (uint i = 0; i < info_.joints.size(); ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, info_.joints[i].state_interfaces[0].name, &hw_states_positions_[i]));
        }

        // export RC Receiver state interfaces
        for (uint i = 4; i < 10; ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name, info_.sensors[0].state_interfaces[i-4].name, &hw_states_positions_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> TailHardware::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // export command state interface
        for (uint i = 0; i < info_.joints.size(); ++i) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, info_.joints[i].state_interfaces[0].name, &hw_commands_positions_[i]));
        }

        return command_interfaces;
    }

    CallbackReturn TailHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_DEBUG(rclcpp::get_logger("TailHardware"), "Activating ...please wait...");
        

        // Trying to instanciate the driver
        try {
            serial_ = rtac::asio::Stream::CreateSerial(port_, baud_rate_);
            RCLCPP_INFO(
                rclcpp::get_logger("TailHardware"),
                "Driver sucessfully created!"
            );
        }
        catch(boost::system::system_error& e) {
            RCLCPP_FATAL(
                rclcpp::get_logger("TailHardware"),
                "Serial error: '%s'", e.what()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Starting the serial
        serial_->start();

        // Putting state to 0
        for (unsigned int i=0; i<hw_states_positions_.size(); ++i) {
            hw_states_positions_[i] = 0;
        }

        // Sending neutral command
        for (unsigned int i=0; i<hw_commands_positions_.size(); ++i) {
            hw_commands_positions_[i] = 0;
        }
        serial_write_actuators_commands();

        // Launching async read
        read_buffer_ = std::string(1024, '\0');
        if  (!serial_->async_read_until(
                read_buffer_.size(), (uint8_t*)read_buffer_.c_str(), '\n',
                std::bind(&TailHardware::read_callback, this, std::placeholders::_1, std::placeholders::_2))
            ) {
            RCLCPP_FATAL(
                rclcpp::get_logger("TailHardware"),
                "Async read until could not start !"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_DEBUG(rclcpp::get_logger("TailHardware"), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn TailHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("TailHardware"), "Deactivating ...please wait...");

        // Sending neutral command
        for (unsigned int i=0; i<hw_commands_positions_.size(); ++i) {
            hw_commands_positions_[i] = 0;
        }
        serial_write_actuators_commands();

        RCLCPP_INFO(rclcpp::get_logger("TailHardware"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn TailHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("TailHardware"), "Cleanup ...please wait...");

        // Sending neutral command
        for (unsigned int i=0; i<hw_commands_positions_.size(); ++i) {
            hw_commands_positions_[i] = 0;
        }
        serial_write_actuators_commands();

        RCLCPP_INFO(rclcpp::get_logger("TailHardware"), "Successfully cleanup!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type TailHardware::read(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {
        time_read_ = time;

        // Check if the received is expired
        // rclcpp::Duration d = time - time_last_received_;
        // if (d > duration_expiration_) {
        //     RCLCPP_FATAL(
        //         rclcpp::get_logger("TailHardware"),
        //         "Read states expired!"
        //     );
        //     return hardware_interface::return_type::ERROR;
        // }

        // Copy async recevied states to hw_state_interfaces
        std::lock_guard<std::mutex> lock_(m_read_);
        if (data_consumable_) {
            // Get thruster state
            hw_states_positions_[0] = std::clamp((static_cast<double>(read_data_[0]) - 1500.) / 500., -1., 1.);

            // Get fin states
            for (std::size_t i=1; i<4; ++i) {
                hw_states_positions_[i] = std::clamp(M_PI/3 * (static_cast<double>(read_data_[i]) - 1500.) / 500., -M_PI/3, M_PI/3);
            }

            // Get RC Receiver state
            for (std::size_t i=4; i<10; ++i) {
                hw_states_positions_[i] = std::clamp((static_cast<double>(read_data_[i]) - 1500.) / 500., -1., 1.);
            }

            // Reset data consumable
            data_consumable_ = false;

            RCLCPP_INFO(
                rclcpp::get_logger("TailHardware"),
                "Reading positions: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
                hw_states_positions_[0], hw_states_positions_[1],
                hw_states_positions_[2], hw_states_positions_[3],
                hw_states_positions_[4], hw_states_positions_[5],
                hw_states_positions_[6], hw_states_positions_[7],
                hw_states_positions_[8], hw_states_positions_[9]
            );
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type TailHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        // Write hardware commands
        std::size_t n = serial_write_actuators_commands();

        if (n != 9) {
            RCLCPP_FATAL(
                rclcpp::get_logger("TailHardware"),
                "Unable to correctly write actuators commands!"
            );
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }
} // riptide_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_hardware::TailHardware, hardware_interface::SystemInterface)
