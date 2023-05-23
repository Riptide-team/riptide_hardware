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

#include <nmeaparse/nmea.h>

#include "riptide_hardware/actuators_commands.hpp"
#include "riptide_hardware/rc_commands.hpp"
#include "riptide_hardware/multiplexer_commands.hpp"


namespace riptide_hardware {

    std::size_t TailHardware::serial_write_actuators_commands() {
        nmea::NMEACommand command;
        command.name = "RHACT";
        command.message = "";

        // Transforming double in [-1, 1] and [-pi/3, pi/3] to uint16_t in [1000, 2000] us
        std::vector<uint16_t> values;
        for (std::size_t i=0; i<4; ++i) {
            if (i==0) {
                values.push_back(static_cast<std::uint16_t>(500 * std::clamp(hw_commands_positions_[0], -1., 1.) + 1500));
            }
            else {
                values.push_back(static_cast<std::uint16_t>(4000. / M_PI * std::clamp(hw_commands_positions_[i], -M_PI/4., M_PI/4.) + 1500));
            }
        }

        std::stringstream ss;

        std::copy(std::begin(values), std::end(values), std::ostream_iterator<uint16_t>(ss,","));
        std::string message = (ss.str()).substr(0, ss.str().size() - 1);
        command.message = message;

        // Writing frame
        std::string frame = command.toString();
        std::size_t n = serial_->write(frame.size(), reinterpret_cast<const uint8_t*>(frame.c_str()), write_timeout_);
        RCLCPP_INFO(rclcpp::get_logger("TailHardware"), "Serial write %s", (command.toString()).c_str());
        return n;
    }

    void TailHardware::read_callback(const rtac::asio::SerialStream::ErrorCode& /*err*/, std::size_t count) {

        RCLCPP_INFO(rclcpp::get_logger("TailHardware"), "Read %s", read_buffer_.c_str());

        // Adding received data to the nmea parser
        for (int i = 0; i < count; ++i){
            try {
                parser.readByte(read_buffer_[i]);
            }
            catch (nmea::NMEAParseError& e){
                RCLCPP_DEBUG(
                        rclcpp::get_logger("TailHardware"),
                        "Error while parsing NMEA data (%s)!", (e.what()).c_str()
                );
            }
        }


        // Relaunching an async read
        if(!serial_->async_read_until(read_buffer_.size(), (uint8_t*)read_buffer_.c_str(), '\n', std::bind(&TailHardware::read_callback, this, std::placeholders::_1, std::placeholders::_2))) {
            RCLCPP_FATAL(
                rclcpp::get_logger("TailHardware"),
                "Async read until could not start !"
            );
        }
    }

    void TailHardware::RTACT_handler(const nmea::NMEASentence& n) {
        // Getting each actuators commands
        std::vector<uint16_t> commands;
        for (size_t i = 0; i < n.parameters.size(); ++i){
            try {
                commands.push_back(std::stoul(n.parameters[i], nullptr, 10));
            }
            catch (const std::invalid_argument& ia) {
                RCLCPP_DEBUG(
                    rclcpp::get_logger("TailHardware"),
                    "RTRCR invalid argument parsing error (%s)", ia.what()
                );
            }
            catch (const std::out_of_range& oor) {
                RCLCPP_DEBUG(
                    rclcpp::get_logger("TailHardware"),
                    "RTRCR out or range parsing error (%s)", oor.what()
                );
            }
            catch (const std::exception& e) {
                RCLCPP_DEBUG(
                    rclcpp::get_logger("TailHardware"),
                    "RTRCR parsing error (%s)", e.what()
                );
            }
		}

        // Store in actuators_commands with time
        RCLCPP_INFO(rclcpp::get_logger("TailHardware"), "RTACT %d %d %d %d", commands[0], commands[1], commands[2], commands[3]);
        {
            std::scoped_lock<std::mutex> lock(rc_mutex_);
            rc_commands_ = std::make_unique<RCCommands<rclcpp::Time>>(time_read_, commands);
        }
    }

    void TailHardware::RTRCR_handler(const nmea::NMEASentence& n) {
        // Getting each actuators commands
        std::vector<uint16_t> commands;
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

        // Store in actuators_commands with time
        RCLCPP_INFO(rclcpp::get_logger("TailHardware"), "RTRCR %d %d %d %d %d %d", commands[0], commands[1], commands[2], commands[3], commands[4], commands[5]);
        {
            std::scoped_lock<std::mutex> lock(actuators_mutex_);
            actuators_commands_ = std::make_unique<ActuatorsCommands<rclcpp::Time>>(time_read_, commands);
        }
    }

    void TailHardware::RTMPX_handler(const nmea::NMEASentence& n) {
        // Getting multiplexer infos
        double able_control;
        double remaining_time;

        try {
            able_control = std::stof(n.parameters[0]);
            remaining_time = std::stof(n.parameters[1]);
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

        // Store in multiplexer infos with time
        RCLCPP_INFO(rclcpp::get_logger("TailHardware"), "RTMPX %f %f", able_control, remaining_time);
        {
            std::scoped_lock<std::mutex> lock(multiplexer_mutex_);
            multiplexer_commands_ = std::make_unique<MultiplexerCommands<rclcpp::Time>>(time_read_, able_control, remaining_time);
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
        
        std::size_t state_number = info_.joints.size();
        for (const auto &s : info_.sensors) {
            state_number += s.state_interfaces.size();
        }
        hw_states_positions_.resize(state_number, std::numeric_limits<double>::quiet_NaN());

        RCLCPP_INFO(
            rclcpp::get_logger("TailHardware"),
            "Serial communication: %s:%i", port_.c_str(), baud_rate_
        );

        write_timeout_ = 1000; // 1000 ms of timeout to write commands

        duration_expiration_ = rclcpp::Duration(1, 0); // 1 s, 0 ns

        data_consumable_ = false;

        // Adding RTACT handler to the nmea parser
        parser.setSentenceHandler("RTACT", std::bind(&TailHardware::RTACT_handler, this, std::placeholders::_1));

        // Adding RTRCR handler to the nmea parser
        parser.setSentenceHandler("RTRCR", std::bind(&TailHardware::RTRCR_handler, this, std::placeholders::_1));

        // Adding RTMPX handler to the nmea parser
        parser.setSentenceHandler("RTMPX", std::bind(&TailHardware::RTMPX_handler, this, std::placeholders::_1));

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
        for (uint i = 4; i<10; ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name, info_.sensors[0].state_interfaces[i-4].name, &hw_states_positions_[i]));
        }

        // Multiplexer infos
        for (uint i=0; i<2; ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[1].name, info_.sensors[1].state_interfaces[i].name, &hw_states_positions_[i+10]));
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

        // Getting actuators commands
        {
            if (actuators_commands_ != nullptr) {
                std::scoped_lock<std::mutex> lock(actuators_mutex_);
                hw_states_positions_[0] = actuators_commands_->Thruster();
                hw_states_positions_[1] = actuators_commands_->DFinAngle();
                hw_states_positions_[2] = actuators_commands_->PFinAngle();
                hw_states_positions_[3] = actuators_commands_->SFinAngle();
                RCLCPP_INFO(rclcpp::get_logger("TailHardware"), "Ros2Control RTACT %f %f %f %f", hw_states_positions_[0], hw_states_positions_[1], hw_states_positions_[2], hw_states_positions_[3]);
                actuators_commands_ = nullptr;
            }
        }

        // Getting RC commands
        {
            if (rc_commands_ != nullptr) {
                std::scoped_lock<std::mutex> lock(rc_mutex_);
                std::vector<double> commands = rc_commands_->GetCommands();
                for (std::size_t i=4; i<hw_states_positions_.size(); ++i) {
                    hw_states_positions_[i] = commands[i-4];
                }
                RCLCPP_INFO(rclcpp::get_logger("TailHardware"), "Ros2Control RTRCR %f %f %f %f %f %f", hw_states_positions_[4], hw_states_positions_[5], hw_states_positions_[6], hw_states_positions_[7], hw_states_positions_[8], hw_states_positions_[9]);
                rc_commands_ = nullptr;
            }
        }


        // Getting Multiplexer commands commands
        {
            if (multiplexer_commands_ != nullptr) {
                std::scoped_lock<std::mutex> lock(multiplexer_mutex_);
                std::vector<double> infos = multiplexer_commands_->GetMultiplexerInfos();
                hw_states_positions_[10] = infos[0];
                hw_states_positions_[11] = infos[1];
                RCLCPP_INFO(rclcpp::get_logger("TailHardware"), "Ros2Control RTMPX %f %f", hw_states_positions_[10], hw_states_positions_[11]);
                multiplexer_commands_ = nullptr;
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type TailHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        // Write hardware commands
        /*std::size_t n = */ serial_write_actuators_commands();
        return hardware_interface::return_type::OK;
    }
} // riptide_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_hardware::TailHardware, hardware_interface::SystemInterface)
