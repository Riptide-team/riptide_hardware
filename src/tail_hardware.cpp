#include "riptide_hardware/tail_hardware.hpp"

#include <algorithm>
#include <mutex>
#include <string>
#include <sstream>
#include <iomanip>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>

#include <rtac_asio/Stream.h>
#include <rtac_asio/SerialStream.h>

#include <nmeaparse/nmea.h>


namespace riptide_hardware {

    void TailHardware::read_callback(const rtac::asio::SerialStream::ErrorCode& err, std::size_t count) {
        // Serial error reading
        if (err) {
           RCLCPP_WARN(rclcpp::get_logger("TailHardware"), "Error while serial reading: %s", (err.message()).c_str());
        }

        // Adding received data to the nmea parser
        for (std::size_t i = 0; i < count; ++i){
            try {
                parser.readByte(read_buffer_[i]);
            }
            catch (nmea::NMEAParseError& e){
                RCLCPP_WARN(
                    rclcpp::get_logger("TailHardware"),
                    "Error while parsing NMEA data (%s)!", (e.what()).c_str()
                );
            }
        }

        read_buffer_ = std::string(1024, '\0');
        // Relaunching an async read
        if(!serial_->async_read_until(read_buffer_.size(),
            reinterpret_cast<std::uint8_t*>(const_cast<char*>(read_buffer_.c_str())),
            '\n', std::bind(&TailHardware::read_callback, this, std::placeholders::_1, std::placeholders::_2)
            )) {
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

        // Getting port from hardware parameters
        if (info_.hardware_parameters.find("port") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("TailHardware"),
                "You need to specify the serial port in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        port_ = info_.hardware_parameters.at("port");

        // Getting baudrate from hardware parameters
        if (info_.hardware_parameters.find("baudrate") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("TailHardware"),
                "You need to specify the serial baud rate in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        baud_rate_ = std::stoi(info_.hardware_parameters.at("baudrate"));

        RCLCPP_DEBUG(rclcpp::get_logger("TailHardware"), "port: %s - baud rate: %d", port_.c_str(), baud_rate_);

        // Getting baudrate from hardware parameters
        if (info_.hardware_parameters.find("expiration_duration_ms") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("TailHardware"),
                "You need to specify the expiration_duration_ms in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        expiration_duration_ = rclcpp::Duration(
            std::chrono::milliseconds(
                std::stoi(info_.hardware_parameters.at("expiration_duration_ms"))
            )
        );

        // Resizing joint commands and states interface
        std::stringstream ss;
        for (const auto &joint: info_.joints) {
            ss << joint.name << ", ";
        }
        if (info_.joints.size() != 4) {
            RCLCPP_FATAL(
                rclcpp::get_logger("TailHardware"),
                "TailHardware should have exactly 4 joints [`thruster`, `d_fin`, `p_fin`, `s_fin`]\nProvided joints are: %s", (ss.str()).c_str()
            );
            return hardware_interface::CallbackReturn::FAILURE;
        }
        else {
            hw_actuators_commands_.resize(4, std::numeric_limits<double>::quiet_NaN());
            hw_actuators_states_.resize(4, std::numeric_limits<double>::quiet_NaN());
            RCLCPP_DEBUG(
                rclcpp::get_logger("TailHardware"),
                "Provided joints `%s`", (ss.str()).c_str()
            );
            RCLCPP_DEBUG(
                rclcpp::get_logger("TailHardware"),
                "Joints command interfaces size: 4"
            );
            RCLCPP_DEBUG(
                rclcpp::get_logger("TailHardware"),
                "Joints state interfaces size: 4"
            );
        }

        // TODO add try except around stoi or stod
        // Getting joint command parameters
        auto get_joint_params = [&](std::string name) -> JointParameters {
            auto it = std::find_if(
                std::begin(info_.joints), std::end(info_.joints),
                [&](const auto &joint) { return joint.name.find(name) != std::string::npos; }
            );
            if (it->parameters.find("pwm_neutral") == it->parameters.end()) {
                RCLCPP_DEBUG(
                    rclcpp::get_logger("TailHardware"),
                    "Neutral PWM will be set to the default 1500 for %s joint", (it->name).c_str()
                );
                return {
                    std::stod(it->command_interfaces[0].min),
                    std::stod(it->command_interfaces[0].max),
                    static_cast<std::uint16_t>(1500)
                };
            }
            else {
                RCLCPP_DEBUG(rclcpp::get_logger("TailHardware"), "JointParameter %s: %s, %s, %s", (it->name).c_str(), (it->command_interfaces[0].min).c_str(), (it->command_interfaces[0].max).c_str(), (it->parameters.at("pwm_neutral")).c_str());
                return {
                    std::stod(it->command_interfaces[0].min),
                    std::stod(it->command_interfaces[0].max),
                    static_cast<std::uint16_t>(std::stoul(it->parameters.at("pwm_neutral")))
                };
            }
        };

        std::vector<std::string> actuator_names = {"thruster", "d_fin", "p_fin", "s_fin"};
        std::transform(
            actuator_names.cbegin(), actuator_names.cend(),
            std::back_inserter(joint_parameters_), [&](std::string name) { return get_joint_params(name); }
        );

        // Resizing sensors states vectors
        for (const auto &sensor: info_.sensors) {
            if (sensor.name.find("rc") != std::string::npos) {
                ss.str("");
                for (const auto &state_interface: sensor.state_interfaces) {
                    ss << state_interface.name << ", ";
                }
                use_rc_ = true;
                hw_rc_states_.resize(sensor.state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
                RCLCPP_DEBUG(
                    rclcpp::get_logger("TailHardware"),
                    "RC sensor found in ros2_control tag, state interfaces will be published!"
                );
                RCLCPP_DEBUG(
                    rclcpp::get_logger("TailHardware"),
                    "RC sensor state interfaces `%s`", (ss.str()).c_str()
                );
            }
            else if (sensor.name.find("multiplexer") != std::string::npos) {
                ss.str("");
                for (const auto &state_interface: sensor.state_interfaces) {
                    ss << state_interface.name << ", ";
                }
                use_multiplexer_ = true;
                if (sensor.state_interfaces.size() != 2) {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("TailHardware"),
                        "Multiplexer sensor should have exactly 2 state interfaces [`automatic`, `remaining_time`]\nProvided state interfaces are: %s", (ss.str()).c_str()
                    );
                }
                else {
                    hw_multiplexer_states_.resize(sensor.state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
                    RCLCPP_DEBUG(
                        rclcpp::get_logger("TailHardware"),
                        "Multiplexer sensor found in ros2_control tag, state interfaces will be published!"
                    );
                    RCLCPP_DEBUG(
                        rclcpp::get_logger("TailHardware"),
                        "Multiplexer sensor state interfaces `%s`", (ss.str()).c_str()
                    );
                }
            }
        }

        // TODO Think about it
        write_timeout_ = 1000; // 1000 ms of timeout to write commands

        // Written command initialization
        RHACT_command_.name = "RHACT";

        // Adding RTACT handler to the nmea parser
        parser.setSentenceHandler("RTACT", std::bind(&TailHardware::RTACT_handler, this, std::placeholders::_1));

        // Adding RTRCR handler to the nmea parser
        if (use_rc_) {
            parser.setSentenceHandler("RTRCR", std::bind(&TailHardware::RTRCR_handler, this, std::placeholders::_1));
        }

        // Adding RTMPX handler to the nmea parser
        if (use_multiplexer_) {
            parser.setSentenceHandler("RTMPX", std::bind(&TailHardware::RTMPX_handler, this, std::placeholders::_1));
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> TailHardware::export_state_interfaces() {
        // TODO migrate all check in on_init
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (const auto &joint: info_.joints) {
            // Thruster [state: `velocity`]
            if (joint.name.find("thruster") != std::string::npos) {
                if ((joint.command_interfaces.size() == 1) and (joint.command_interfaces[0].name == "velocity")) {
                    state_interfaces.emplace_back(
                        hardware_interface::StateInterface(
                            joint.name, joint.state_interfaces[0].name, &hw_actuators_states_[0]
                        )
                    );
                }
                else {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("TailHardware"),
                        "`thruster` accept only one state interface of type `velocity`\nProvided command interface `%s`", joint.command_interfaces[0].name.c_str()
                    );
                }
            }

            // D Fin [state: `position`]
            if (joint.name.find("d_fin") != std::string::npos) {
                if ((joint.command_interfaces.size() == 1) and (joint.command_interfaces[0].name == "position")) {
                    state_interfaces.emplace_back(
                        hardware_interface::StateInterface(
                            joint.name, joint.state_interfaces[0].name, &hw_actuators_states_[1]
                        )
                    );
                }
                else {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("TailHardware"),
                        "`d_fin` accept only one state interface of type `position`\nProvided command interface `%s`", joint.command_interfaces[0].name.c_str()
                    );
                }
            }

            // P Fin [state: `position`]
            if (joint.name.find("p_fin") != std::string::npos) {
                if ((joint.state_interfaces.size() == 1) and (joint.state_interfaces[0].name == "position")) {
                    state_interfaces.emplace_back(
                        hardware_interface::StateInterface(
                            joint.name, joint.state_interfaces[0].name, &hw_actuators_states_[2]
                        )
                    );
                }
                else {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("TailHardware"),
                        "`p_fin` accept only one command interface of type `position`\nProvided command interface `%s`", joint.command_interfaces[0].name.c_str()
                    );
                }
            }

            // S Fin [state: `position`]
            if (joint.name.find("s_fin") != std::string::npos) {
                if ((joint.state_interfaces.size() == 1) and (joint.state_interfaces[0].name == "position")) {
                    state_interfaces.emplace_back(
                        hardware_interface::StateInterface(
                            joint.name, joint.state_interfaces[0].name, &hw_actuators_states_[3]
                        )
                    );
                }
                else {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("TailHardware"),
                        "`s_fin` accept only one command interface of type `position`\nProvided command interface `%s`", joint.command_interfaces[0].name.c_str()
                    );
                }
            }
        }

        for (const auto& sensor: info_.sensors) {
            
            // RC sensor provided [states: `ch1`, `ch2`, ..., `chn`]
            if (sensor.name.find("rc") != std::string::npos) {
                for (std::size_t i = 0; i < sensor.state_interfaces.size(); ++i) {

                    state_interfaces.emplace_back(
                        hardware_interface::StateInterface(
                            sensor.name, sensor.state_interfaces[i].name, &hw_rc_states_[i]
                        )
                    );
                }
            }

            // Multiplexer sensor provided [states: `automatic`, `remaining_time`]
            else if (sensor.name.find("multiplexer") != std::string::npos) {
                for (const auto &state_interface: sensor.state_interfaces) {
                    if (state_interface.name == "automatic") {
                        state_interfaces.emplace_back(
                            hardware_interface::StateInterface(
                                sensor.name, state_interface.name, &hw_multiplexer_states_[0]
                            )
                        );
                    }
                    else if (state_interface.name == "remaining_time") {
                        state_interfaces.emplace_back(
                            hardware_interface::StateInterface(
                                sensor.name, state_interface.name, &hw_multiplexer_states_[1]
                            )
                        );
                    }
                    else {
                        std::stringstream ss;
                        for (const auto &state_interface: sensor.state_interfaces) {
                            ss << state_interface.name << ", ";
                        }
                        RCLCPP_FATAL(
                            rclcpp::get_logger("TailHardware"),
                            "`multiplexer` sensor accept only two state interfaces of type `automatic` and `remaining_time`\nProvided state interfaces `%s", (ss.str()).c_str()
                        );
                    }
                }
            }
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> TailHardware::export_command_interfaces() {
        // TODO migrate all check in on_init
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (const auto &joint: info_.joints) {
            // Thruster [command: `velocity`]
            if (joint.name.find("thruster") != std::string::npos) {
                if ((joint.command_interfaces.size() == 1) and (joint.command_interfaces[0].name == "velocity")) {
                    command_interfaces.emplace_back(
                        hardware_interface::CommandInterface(
                            joint.name, joint.command_interfaces[0].name, &hw_actuators_commands_[0]
                        )
                    );
                }
                else {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("TailHardware"),
                        "`thruster` joint accept only one command interface of type `velocity`"
                    );
                }
            }

            // D Fin [command: `position`]
            if (joint.name.find("d_fin") != std::string::npos) {
                if ((joint.command_interfaces.size() == 1) and (joint.command_interfaces[0].name == "position")) {
                    command_interfaces.emplace_back(
                        hardware_interface::CommandInterface(
                            joint.name, joint.command_interfaces[0].name, &hw_actuators_commands_[1]
                        )
                    );
                }
                else {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("TailHardware"),
                        "`d_fin` joint accept only one command interface of type `position`"
                    );
                }
            }

            // P Fin [command: `position`]
            if (joint.name.find("p_fin") != std::string::npos) {
                if ((joint.command_interfaces.size() == 1) and (joint.command_interfaces[0].name == "position")) {
                    command_interfaces.emplace_back(
                        hardware_interface::CommandInterface(
                            joint.name, joint.command_interfaces[0].name, &hw_actuators_commands_[2]
                        )
                    );
                }
                else {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("TailHardware"),
                        "`p_fin` joint accept only one command interface of type `position`"
                    );
                }
            }

            // S Fin [command: `position`]
            if (joint.name.find("s_fin") != std::string::npos) {
                if ((joint.command_interfaces.size() == 1) and (joint.command_interfaces[0].name == "position")) {
                    command_interfaces.emplace_back(
                        hardware_interface::CommandInterface(
                            joint.name, joint.command_interfaces[0].name, &hw_actuators_commands_[3]
                        )
                    );
                }
                else {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("TailHardware"),
                        "`s_fin` joint accept only one command interface of type `position`"
                    );
                }
            }
        }

        return command_interfaces;
    }

    CallbackReturn TailHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_DEBUG(rclcpp::get_logger("TailHardware"), "Activating");
        
        // Trying to instanciate the driver
        try {
            serial_ = rtac::asio::Stream::CreateSerial(port_, baud_rate_);
            RCLCPP_DEBUG(
                rclcpp::get_logger("TailHardware"),
                "Driver sucessfully created on port `%s` with the baudrate `%d`!", port_.c_str(), baud_rate_
            );
        }
        catch(boost::system::system_error& e) {
            RCLCPP_FATAL(
                rclcpp::get_logger("TailHardware"),
                "Serial creation error: '%s'", e.what()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Starting the serial
        serial_->start();
        serial_->flush();
        serial_->enable_io_dump("./rx_tail.log", "./tx_tail.log");

        // Initialize time read variables
        actuators_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
        rc_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
        multiplexer_time_ = rclcpp::Clock(RCL_ROS_TIME).now();

        // Filling read_actuators_states
        read_actuators_states_.resize(4);
        for (std::size_t i = 0; i < read_actuators_states_.size(); ++i) {
            read_actuators_states_[i] = joint_parameters_[i].pwm_neutral;
        }

        // Filling read_rc_states
        read_rc_states_.resize(6);
        std::fill(read_rc_states_.begin(), read_rc_states_.end(), static_cast<std::uint16_t>(1500));

        // Filling read_multiplexer_states
        read_multiplexer_states_.resize(2);
        std::fill(read_multiplexer_states_.begin(), read_multiplexer_states_.end(), 0.);

        // Putting actuators command and state interface to 0
        std::fill(hw_actuators_commands_.begin(), hw_actuators_commands_.end(), 0.);
        std::fill(hw_actuators_states_.begin(), hw_actuators_states_.end(), 0.);

        // Putting RC state interface to NaN
        std::fill(hw_rc_states_.begin(), hw_rc_states_.end(), 0.);

        // Putting Multiplexer state interface to NaN
        std::fill(hw_multiplexer_states_.begin(), hw_multiplexer_states_.end(), 0.);

        // Send neutral PWM
        std::vector<uint16_t> values;
        for (const auto &joint_parameter: joint_parameters_) {
            values.push_back(joint_parameter.pwm_neutral);
        }

        std::stringstream ss;
        std::copy(values.begin(), values.end(), std::ostream_iterator<std::uint16_t>(ss, ","));
        
        // Writing actuators commands
        RHACT_command_.message = (ss.str()).substr(0, ss.str().size() - 1);
        std::string frame = RHACT_command_.toString();
        std::size_t n = serial_->write(frame.size(), reinterpret_cast<const uint8_t*>(frame.c_str()), write_timeout_);

        if (n != frame.size()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("TailHardware"),
                "Unable to write commands on the serial port `%s` with baudrate`%d`",
                port_.c_str(), baud_rate_
            );
            RCLCPP_DEBUG(
                rclcpp::get_logger("TailHardware"),
                "RHACT not written: `%s`", (RHACT_command_.toString()).c_str()
            );
            return hardware_interface::CallbackReturn::FAILURE;
        }
        else {
            RCLCPP_DEBUG(
                rclcpp::get_logger("TailHardware"),
                "RHACT written: `%s`", (RHACT_command_.toString()).c_str()
            );
        }

        // Launching async read
        read_buffer_ = std::string(1024, '\0');
        if  (!serial_->async_read_until(
                read_buffer_.size(), reinterpret_cast<std::uint8_t*>(const_cast<char*>(read_buffer_.c_str())), '\n',
                std::bind(&TailHardware::read_callback, this, std::placeholders::_1, std::placeholders::_2))
            ) {
            RCLCPP_FATAL(
                rclcpp::get_logger("TailHardware"),
                "Unable to start async read on serial port `%s` with the baudrate `%d`!", port_.c_str(), baud_rate_
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_DEBUG(rclcpp::get_logger("TailHardware"), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn TailHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_DEBUG(rclcpp::get_logger("TailHardware"), "Deactivating");

        // Closing serial connection
        serial_ = nullptr;

        // Putting actuators command and state interface to NaN
        std::fill(hw_actuators_commands_.begin(), hw_actuators_commands_.end(), std::numeric_limits<double>::quiet_NaN());
        std::fill(hw_actuators_states_.begin(), hw_actuators_states_.end(), std::numeric_limits<double>::quiet_NaN());

        // Putting RC state interface to NaN
        std::fill(hw_rc_states_.begin(), hw_rc_states_.end(), std::numeric_limits<double>::quiet_NaN());

        // Putting Multiplexer state interface to NaN
        std::fill(hw_multiplexer_states_.begin(), hw_multiplexer_states_.end(), std::numeric_limits<double>::quiet_NaN());

        // Send neutral PWM
        std::vector<uint16_t> values;
        for (const auto &joint_parameter: joint_parameters_) {
            values.push_back(joint_parameter.pwm_neutral);
        }

        std::stringstream ss;
        std::copy(values.begin(), values.end(), std::ostream_iterator<std::uint16_t>(ss, ","));
        
        // Sending neutral PWM
        RHACT_command_.message = (ss.str()).substr(0, ss.str().size() - 1);
        std::string frame = RHACT_command_.toString();
        std::size_t n = serial_->write(frame.size(), reinterpret_cast<const uint8_t*>(frame.c_str()), write_timeout_);

        if (n != frame.size()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("TailHardware"),
                "Unable to write commands on the serial port `%s` with baudrate`%d`",
                port_.c_str(), baud_rate_
            );
            RCLCPP_DEBUG(
                rclcpp::get_logger("TailHardware"),
                "RHACT not written: `%s`", (RHACT_command_.toString()).c_str()
            );
            return hardware_interface::CallbackReturn::FAILURE;
        }
        else {
            RCLCPP_DEBUG(
                rclcpp::get_logger("TailHardware"),
                "RHACT written: `%s`", (RHACT_command_.toString()).c_str()
            );
            RCLCPP_DEBUG(
                rclcpp::get_logger("TailHardware"),
                "Successfully deactivated!"
            );
            return hardware_interface::CallbackReturn::SUCCESS;
        }

    }

    CallbackReturn TailHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_DEBUG(rclcpp::get_logger("TailHardware"), "Cleanup");

        // Send neutral PWM
        std::vector<uint16_t> values;
        for (const auto &joint_parameter: joint_parameters_) {
            values.push_back(joint_parameter.pwm_neutral);
        }

        std::stringstream ss;
        std::copy(values.begin(), values.end(), std::ostream_iterator<std::uint16_t>(ss, ","));
        
        // Sending neutral PWM
        RHACT_command_.message = (ss.str()).substr(0, ss.str().size() - 1);
        std::string frame = RHACT_command_.toString();
        std::size_t n = serial_->write(frame.size(), reinterpret_cast<const uint8_t*>(frame.c_str()), write_timeout_);

        if (n != frame.size()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("TailHardware"),
                "Unable to write commands on the serial port `%s` with baudrate`%d`",
                port_.c_str(), baud_rate_
            );
            RCLCPP_DEBUG(
                rclcpp::get_logger("TailHardware"),
                "RHACT not written: `%s`", (RHACT_command_.toString()).c_str()
            );
            return hardware_interface::CallbackReturn::FAILURE;
        }
        else {
            RCLCPP_DEBUG(
                rclcpp::get_logger("TailHardware"),
                "RHACT written: `%s`", (RHACT_command_.toString()).c_str()
            );
            RCLCPP_DEBUG(
                rclcpp::get_logger("TailHardware"),
                "Successfully cleanup!"
            );
            return hardware_interface::CallbackReturn::SUCCESS;
        }
    }

    hardware_interface::return_type TailHardware::read(const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {
        
        // Getting actuators commands
        {
            if ((time - actuators_time_) < expiration_duration_) {
                std::scoped_lock<std::mutex> lock(actuators_mutex_);

                // Thruster clamp
                std::cout << read_actuators_states_[0] << std::endl;
                std::cout << static_cast<double>(read_actuators_states_[0] - joint_parameters_[0].pwm_neutral) / 500. << std::endl;
                double value = std::clamp<double>(
                    static_cast<double>(read_actuators_states_[0] - joint_parameters_[0].pwm_neutral) / 500.,
                    joint_parameters_[0].min,
                    joint_parameters_[0].max
                );
                std::cout << value << std::endl;
                hw_actuators_states_[0] = value;

                // Fin clamp
                for (std::size_t i = 1; i < 4; ++i) {
                    // Clamping values between joint_parameters_ min and max
                    double value = std::clamp<double>(
                        M_PI / 2000. * static_cast<double>(read_actuators_states_[i] - joint_parameters_[i].pwm_neutral),
                        joint_parameters_[i].min,
                        joint_parameters_[i].max
                    );
                    std::cout << value << std::endl;
                    hw_actuators_states_[i] = value;
                }
                RCLCPP_DEBUG(rclcpp::get_logger("TailHardware"), "Reading RTACT %f %f %f %f (%f seconds ago)", hw_actuators_states_[0], hw_actuators_states_[1], hw_actuators_states_[2], hw_actuators_states_[3], (time - actuators_time_).seconds());
            }
            else {
                RCLCPP_FATAL(rclcpp::get_logger("TailHardware"), "Actuators states are expired! Last received RTACT frame was %f seconds ago (considered expired after %f s).", (time - actuators_time_).seconds(), expiration_duration_.seconds());
                return hardware_interface::return_type::ERROR;
            }
        }

        // Getting RC commands
        {
            if ((time - rc_time_) < expiration_duration_) {
                std::scoped_lock<std::mutex> lock(rc_mutex_);
                for (std::size_t i = 0; i < 6; ++i) {
                    hw_rc_states_[i] = (static_cast<double>(read_rc_states_[i]) - 1500.) / 500.;
                }
                RCLCPP_DEBUG(rclcpp::get_logger("TailHardware"), "Read RTRCR %f %f %f %f %f %f (%f seconds ago)", hw_rc_states_[0], hw_rc_states_[1], hw_rc_states_[2], hw_rc_states_[3], hw_rc_states_[4], hw_rc_states_[5], (time - rc_time_).seconds());
            }
            else {
                RCLCPP_FATAL(rclcpp::get_logger("TailHardware"), "RC states are expired! Last received RTRCR frame was %f seconds ago (considered expired after %f s).", (time - rc_time_).seconds(), expiration_duration_.seconds());
                return hardware_interface::return_type::ERROR;
            }
        }


        // Getting Multiplexer commands commands
        {
            if ((time - multiplexer_time_) < expiration_duration_) {
                std::scoped_lock<std::mutex> lock(multiplexer_mutex_);
                hw_multiplexer_states_ = read_multiplexer_states_;
                RCLCPP_DEBUG(rclcpp::get_logger("TailHardware"), "Read RTMPX %f %f (%f seconds ago)", hw_multiplexer_states_[0], hw_multiplexer_states_[1], (time - multiplexer_time_).seconds());
            }
            else {
                RCLCPP_FATAL(rclcpp::get_logger("TailHardware"), "Multiplexer states are expired! Last received RTMPX frame was %f seconds ago (considered expired after %f s).", (time - multiplexer_time_).seconds(), expiration_duration_.seconds());
                return hardware_interface::return_type::ERROR;
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type TailHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

        // Generate commands from values
        std::vector<uint16_t> values;

        if (std::all_of(hw_actuators_commands_.begin(), hw_actuators_commands_.end(), [](double i){ return !std::isnan(i); })) {
            // Thruster clamp
            values.push_back(
                static_cast<std::uint16_t>(
                    std::clamp(
                        500. * std::clamp(
                            hw_actuators_commands_[0],
                            joint_parameters_[0].min,
                            joint_parameters_[0].max
                        ) + joint_parameters_[0].pwm_neutral,
                        1000.,
                        2000.
                    )
                )
            );

            // Fin clamp
            for (std::size_t i = 1; i < 4; ++i) {
                // Clamping values between 1000 and 2000
                values.push_back(
                    static_cast<std::uint16_t>(
                        joint_parameters_[i].pwm_neutral + 
                        2000. / M_PI *
                        std::clamp(
                            hw_actuators_commands_[i],
                            joint_parameters_[i].min,
                            joint_parameters_[i].max
                        )
                    )
                );
            }

           
        }
        else {
            for (std::size_t i = 0; i < 4; ++i) {
                values.push_back(joint_parameters_[i].pwm_neutral);
            }
            RCLCPP_WARN(rclcpp::get_logger("TailHardware"),
                "NaN in writed commands %f %f %f %f",
                hw_actuators_commands_[0], hw_actuators_commands_[1], hw_actuators_commands_[2], hw_actuators_commands_[3]
            );
        }

        // Generate NMEA frame
        std::stringstream ss;
        std::copy(std::begin(values), std::end(values), std::ostream_iterator<uint16_t>(ss,","));
        RHACT_command_.message = (ss.str()).substr(0, ss.str().size() - 1);

        // Writing frame
        std::string frame = RHACT_command_.toString();
        std::size_t n = serial_->write(frame.size(), reinterpret_cast<const uint8_t*>(frame.c_str()), write_timeout_);

        if (n != frame.size()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("TailHardware"),
                "Unable to write commands on the serial port `%s` with baudrate`%d`",
                port_.c_str(), baud_rate_
            );
            RCLCPP_DEBUG(
                rclcpp::get_logger("TailHardware"),
                "RHACT not written: `%s`", (RHACT_command_.toString()).c_str()
            );
            return hardware_interface::return_type::ERROR;
        }
        else {
            RCLCPP_DEBUG(
                rclcpp::get_logger("TailHardware"),
                "RHACT written: `%s`", (RHACT_command_.toString()).c_str()
            );
            return hardware_interface::return_type::OK;
        }
    }

    void TailHardware::RTACT_handler(const nmea::NMEASentence& n) {
        // Only process frame if the frame contains 4 arguments
        if (n.parameters.size() == 4) {
            // Getting each actuators commands
            std::vector<std::uint16_t> commands;
            for (size_t i = 0; i < n.parameters.size(); ++i){
                try {
                    commands.push_back(std::stoul(n.parameters[i], nullptr, 10));
                }
                catch (const std::invalid_argument& ia) {
                    RCLCPP_WARN(
                        rclcpp::get_logger("TailHardware"),
                        "RTRCR invalid argument parsing error (%s)", ia.what()
                    );
                }
                catch (const std::out_of_range& oor) {
                    RCLCPP_WARN(
                        rclcpp::get_logger("TailHardware"),
                        "RTRCR out or range parsing error (%s)", oor.what()
                    );
                }
                catch (const std::exception& e) {
                    RCLCPP_WARN(
                        rclcpp::get_logger("TailHardware"),
                        "RTRCR parsing error (%s)", e.what()
                    );
                }
            }

            // Check if all received pwm are between 1000 and 2000
            if (std::all_of(commands.begin(), commands.end(), [](std::uint16_t i){ return ((i>=1000) and (i<=2000)); })) {
                {
                    std::scoped_lock<std::mutex> lock(actuators_mutex_);
                    read_actuators_states_ = commands;
                    actuators_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
                    RCLCPP_DEBUG(rclcpp::get_logger("TailHardware"), "RTACT %d %d %d %d, %f", commands[0], commands[1], commands[2], commands[3], actuators_time_.seconds());
                }
            }
            else {
                RCLCPP_WARN(rclcpp::get_logger("TailHardware"), "RTACT commands are not in range [1000, 2000]us!");
                RCLCPP_WARN(rclcpp::get_logger("TailHardware"), "%s", (n.text).c_str());
                RCLCPP_WARN(rclcpp::get_logger("TailHardware"), "RTACT %d %d %d %d, %f", commands[0], commands[1], commands[2], commands[3], rclcpp::Clock(RCL_ROS_TIME).now().seconds());
            }
        }
        else {
            std::stringstream ss;
            std::copy(std::begin(n.parameters), std::end(n.parameters), std::ostream_iterator<std::string>(ss,","));
            RCLCPP_WARN(rclcpp::get_logger("TailHardware"), "RTACT NMEA frame doesn't contains 4 values!");
            RCLCPP_WARN(rclcpp::get_logger("TailHardware"), "RTACT contains: %s", (ss.str()).c_str());
        }
    }

    void TailHardware::RTRCR_handler(const nmea::NMEASentence& n) {
        // Only process frame if the frame contains 6 arguments
        if (n.parameters.size() == 6) {
            // Getting each actuators commands
            std::vector<std::uint16_t> commands;
            for (size_t i = 0; i < n.parameters.size(); ++i){
                try {
                    commands.push_back(std::stoul(n.parameters[i], nullptr, 10));
                }
                catch (const std::invalid_argument& ia) {
                    RCLCPP_DEBUG(
                        rclcpp::get_logger("TailHardware"),
                        "RTACT invalid argument parsing error (%s)", ia.what()
                    );
                }
                catch (const std::out_of_range& oor) {
                    RCLCPP_DEBUG(
                        rclcpp::get_logger("TailHardware"),
                        "RTACT out or range parsing error (%s)", oor.what()
                    );
                }
                catch (const std::exception& e) {
                    RCLCPP_DEBUG(
                        rclcpp::get_logger("TailHardware"),
                        "RTACT parsing error (%s)", e.what()
                    );
                }
            }

            // Check if all received pwm are between 1000 and 2000
            if (std::all_of(commands.begin(), commands.end(), [](std::uint16_t i){ return ((i>=1000) and (i<=2000)); })) {
                {
                    std::scoped_lock<std::mutex> lock(rc_mutex_);
                    rc_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
                    read_rc_states_ = commands;
                    RCLCPP_DEBUG(rclcpp::get_logger("TailHardware"), "RTRCR %d %d %d %d %d %d, %f", commands[0], commands[1], commands[2], commands[3], commands[4], commands[5], rc_time_.seconds());
                }
            }
            else {
                RCLCPP_WARN(rclcpp::get_logger("TailHardware"), "RTRCR commands are not in range [1000, 2000]us!");
                RCLCPP_WARN(rclcpp::get_logger("TailHardware"), "RTRCR %d %d %d %d %d %d, %f", commands[0], commands[1], commands[2], commands[3], commands[4], commands[5], rclcpp::Clock(RCL_ROS_TIME).now().seconds());
            }
        }
        else {
            std::stringstream ss;
            std::copy(std::begin(n.parameters), std::end(n.parameters), std::ostream_iterator<std::string>(ss,","));
            RCLCPP_WARN(rclcpp::get_logger("TailHardware"), "RTRCR NMEA frame doesn't contains 6 values!");
            RCLCPP_WARN(rclcpp::get_logger("TailHardware"), "RTRCR contains: %s", (ss.str()).c_str());
        }
    }

    void TailHardware::RTMPX_handler(const nmea::NMEASentence& n) {
        // Only process the frame if there is 2 arguments
        if (n.parameters.size() == 2) {
            // Getting multiplexer infos
            std::vector<double> commands;

            try {
                commands.push_back(std::stod(n.parameters[0]));
                commands.push_back(std::stod(n.parameters[1]));
            }
            catch (const std::invalid_argument& ia) {
                RCLCPP_DEBUG(
                    rclcpp::get_logger("TailHardware"),
                    "RTMPX invalid argument parsing error (%s)", ia.what()
                );
            }
            catch (const std::out_of_range& oor) {
                RCLCPP_DEBUG(
                    rclcpp::get_logger("TailHardware"),
                    "RTMPX out or range parsing error (%s)", oor.what()
                );
            }
            catch (const std::exception& e) {
                RCLCPP_DEBUG(
                    rclcpp::get_logger("TailHardware"),
                    "RTMPX parsing error (%s)", e.what()
                );
            }

            // Check if that recieved multiplexer is between 0 and 1 and that the remaining time is in [0, 100]s
            if ((commands[1] >= 0.) and (commands[1] <= 100.) and (commands[0] >= 0.) and (commands[0] <= 1.)) {
                {
                    std::scoped_lock<std::mutex> lock(multiplexer_mutex_);
                    read_multiplexer_states_ = commands;
                    multiplexer_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
                    RCLCPP_DEBUG(rclcpp::get_logger("TailHardware"), "RTMPX %f %f, %f", commands[0], commands[1], multiplexer_time_.seconds());
                }
            }
            else {
                RCLCPP_DEBUG(rclcpp::get_logger("TailHardware"), "RTMPX commands are not in range [1000, 2000]us!");
                RCLCPP_DEBUG(rclcpp::get_logger("TailHardware"), "RTMPX %f %f, %f", commands[0], commands[1], rclcpp::Clock(RCL_ROS_TIME).now().seconds());
            }
        }
        else {
            std::stringstream ss;
            std::copy(std::begin(n.parameters), std::end(n.parameters), std::ostream_iterator<std::string>(ss,","));
            RCLCPP_WARN(rclcpp::get_logger("TailHardware"), "RTMPX NMEA frame doesn't contains 2 values!");
            RCLCPP_WARN(rclcpp::get_logger("TailHardware"), "RTMPX contains: %s", (ss.str()).c_str());
        }
    }

} // riptide_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_hardware::TailHardware, hardware_interface::SystemInterface)
