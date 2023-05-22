#ifndef TAIL_HARDWARE_HPP
#define TAIL_HARDWARE_HPP

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include <rtac_asio/Stream.h>
#include <rtac_asio/SerialStream.h>

#include <nmeaparse/nmea.h>

#include "riptide_hardware/actuators_commands.hpp"


namespace riptide_hardware {

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class TailHardware : public hardware_interface::SystemInterface {

        public:
            TailHardware() : duration_expiration_(rclcpp::Duration(1, 0)) {};

            CallbackReturn on_init(const hardware_interface::HardwareInfo & info_) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces()override;

            CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

            hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

        private:
            std::vector<double> hw_states_positions_;
            std::vector<double> hw_commands_positions_;

            // Serial port
            std::string port_;

            // Serial baud rate
            unsigned int baud_rate_;

            // NMEA parser
            nmea::NMEAParser parser;

            // Expiration duration
            rclcpp::Duration duration_expiration_;

            // Driver
            rtac::asio::Stream::Ptr serial_ = nullptr;

            // Write timeout
            uint16_t write_timeout_;

            // Write actuator commands on serial
            std::size_t serial_write_actuators_commands();

            // Read buffer
            std::string read_buffer_;

            // Time last read call
            rclcpp::Time time_read_;

            // Time received last state
            rclcpp::Time time_last_received_;

            // mutex for read data
            std::mutex m_read_;

            // Is data consumable
            bool data_consumable_;

            // Data
            std::vector<uint16_t> read_data_;

            // Read callback
            void read_callback(const rtac::asio::SerialStream::ErrorCode& /*err*/, std::size_t count);

            // Actuators mutex
            std::mutex actuators_mutex_;
            
            // Actuators Commands
            std::unique_ptr<ActuatorsCommands> actuators_commands_ = nullptr;

            // RTACT Handler
            void RTACT_handler(const nmea::NMEASentence& n);
    };
} // riptide_hardware

#endif // TAIL_HARDWARE_HPP