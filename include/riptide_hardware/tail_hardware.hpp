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


namespace riptide_hardware {

    struct JointParameters {
        double min;
        double max;
        uint16_t pwm_neutral;
    };

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class TailHardware : public hardware_interface::SystemInterface {

        public:
            TailHardware() : expiration_duration_(std::chrono::milliseconds(500)) {};

            CallbackReturn on_init(const hardware_interface::HardwareInfo & info_) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces()override;

            CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) override;

            hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

        private:

            /// Command and State Interfaces 
            // Actuators commands
            std::vector<double> hw_actuators_commands_;

            // Actuators states
            std::vector<double> hw_actuators_states_;

            // RC states
            std::vector<double> hw_rc_states_;

            // Multiplexer states
            std::vector<double> hw_multiplexer_states_;


            /// Hardware Interface Parameters
            // Serial port
            std::string port_;

            // Serial baud rate
            unsigned int baud_rate_;

            // Joint command parameters
            std::vector<JointParameters> joint_parameters_;

            // Check if the rc want to be used
            bool use_rc_;

            // Check if the multiplexer want to be used
            bool use_multiplexer_;
            

            // Driver
            rtac::asio::Stream::Ptr serial_ = nullptr;

            // Time last read call
            rclcpp::Duration expiration_duration_;

            // Write timeout
            uint16_t write_timeout_;
            

            /// Serial read
            // Read buffer
            std::string read_buffer_;

            // Read callback
            void read_callback(const rtac::asio::SerialStream::ErrorCode& /*err*/, std::size_t count);

            // NMEA parser
            nmea::NMEAParser parser;

            // Writen command
            nmea::NMEACommand RHACT_command_;

            // RTACT Handler
            void RTACT_handler(const nmea::NMEASentence& n);

            // Actuators mutex
            std::mutex actuators_mutex_;

            // Actuators read time
            rclcpp::Time actuators_time_;

            // Read actuators states
            std::vector<std::uint16_t> read_actuators_states_;

            // RTRCR handler
            void RTRCR_handler(const nmea::NMEASentence& n);

            // RCR mutex
            std::mutex rc_mutex_;

            // RC read time
            rclcpp::Time rc_time_;

            // Read RC states
            std::vector<std::uint16_t> read_rc_states_;

            // RTMPX handler
            void RTMPX_handler(const nmea::NMEASentence& n);

            // Multiplexer mutex
            std::mutex multiplexer_mutex_;

            // Multiplexer read time
            rclcpp::Time multiplexer_time_;

            // Read multiplexer states
            std::vector<double> read_multiplexer_states_;

    };

} // riptide_hardware

#endif // TAIL_HARDWARE_HPP