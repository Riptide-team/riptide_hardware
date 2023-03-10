#include "riptide_hardware/echosounder_hardware.hpp"

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include <mutex>
#include <string>

#include <rtac_asio/Stream.h>
#include <rtac_asio/SerialStream.h>

#include <SeaScanEcho/Command.hpp>
#include <SeaScanEcho/Reply.hpp>
#include <SeaScanEcho/SeaScanEchoCommands.hpp>


namespace riptide_hardware {

    CallbackReturn EchosounderHardware::on_init(const hardware_interface::HardwareInfo & info) {
        if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Checking port and baud rate specified in ros2_control urdf tag
        if (info_.hardware_parameters.find("port") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("EchosounderHardware"),
                "You need to specify the serial port in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info_.hardware_parameters.find("baud_rate") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("EchosounderHardware"),
                "You need to specify the serial baud rate in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Checking port and baud rate specified in ros2_control urdf tag
        if (info_.hardware_parameters.find("port") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("EchosounderHardware"),
                "You need to specify the serial port in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info_.hardware_parameters.find("baud_rate") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("EchosounderHardware"),
                "You need to specify the serial baud rate in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Getting port and baud_rate parameters
        port_ = info_.hardware_parameters.at("port");
        baud_rate_ = stoi(info_.hardware_parameters.at("baud_rate"));
        
        RCLCPP_INFO(rclcpp::get_logger("EchosounderHardware"), "port: %s - baud rate: %d", port_.c_str(), baud_rate_);
        
        RCLCPP_INFO(rclcpp::get_logger("EchosounderHardware"), "Successfully initialized !");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> EchosounderHardware::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.sensors[0].name,
                info_.sensors[0].state_interfaces[0].name,
                &distance_
            )
        );
        return state_interfaces;
    }

    CallbackReturn EchosounderHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("EchosounderHardware"), "Activating ...please wait...");

        // Instanciate the driver
        try {
            serial_ = rtac::asio::Stream::CreateSerial(port_, baud_rate_);
            serial_->start();
            serial_->flush();
        }
        catch(boost::system::system_error& e) {
            RCLCPP_FATAL(
                rclcpp::get_logger("EchosounderHardware"),
                "Serial error: '%s'", e.what()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Send RESET message
        SeaScanEcho::Command msg = SeaScanEcho::Commands::Reset;
        std::string command = msg();
        int count = serial_->write(command.size(), (const uint8_t*)command.c_str());

        RCLCPP_DEBUG(rclcpp::get_logger("EchosounderHardware"), "RESET message witten! %d/%ld char written", count, command.size());

        std::string data(1024, '\0');
        count = serial_->read_until(data.size(), (uint8_t*)data.c_str(), '\n');

        SeaScanEcho::Reply s_reset(data.substr(0, count));

        std::vector<std::string> fields = s_reset.Fields();
        if (!s_reset.Valid() && fields[0] != "MSALT" && fields[1] != "INFO") {
            RCLCPP_FATAL(
                rclcpp::get_logger("EchosounderHardware"),
                "Bad response to RESET: '%s' : got %s, %s", (data.substr(0, count)).c_str(), (std::string(fields[0])).c_str(), (std::string(fields[1])).c_str()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Send MARCO message
        msg = SeaScanEcho::Commands::Marco;
        command = msg();
        count = serial_->write(command.size(), (const uint8_t*)command.c_str());

        RCLCPP_DEBUG(rclcpp::get_logger("EchosounderHardware"), "MARCO message witten! %d/%ld char written", count, command.size());

        // Read POLO response
        data = std::string(1024, '\0');
        count = serial_->read_until(data.size(), (uint8_t*)data.c_str(), '\n');

        RCLCPP_DEBUG(rclcpp::get_logger("EchosounderHardware"), "POLO message read! %s", (data.substr(0, count)).c_str());

        SeaScanEcho::Reply s_polo(data.substr(0, count));

        fields = s_polo.Fields();
        if (!s_polo.Valid() && fields[0] != "MSALT" && fields[1] != "POLO") {
            RCLCPP_FATAL(
                rclcpp::get_logger("EchosounderHardware"),
                "Bad response to MARCO: '%s'", (data.substr(0, count)).c_str()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("EchosounderHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn EchosounderHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("EchosounderHardware"), "Deactivating ...please wait...");

        // Destruct the driver pointer
        serial_ = nullptr;

        RCLCPP_INFO(rclcpp::get_logger("EchosounderHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type EchosounderHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        // Send Trigger message
        SeaScanEcho::Command msg = SeaScanEcho::Commands::Trigger;
        std::string command = msg();
        serial_->write(command.size(), (const uint8_t*)command.c_str());

        // Read response
        std::string data(1024, '\0');
        int count = serial_->read_until(data.size(), (uint8_t*)data.c_str(), '\n');

        SeaScanEcho::Reply s(data.substr(0, count));
        std::vector<std::string> fields = s.Fields();
        if (!s.Valid() && fields[0] != "MSALT" && fields[1] != "ACK") {
            RCLCPP_FATAL(
                rclcpp::get_logger("EchosounderHardware"),
                "Bad response to TRIGGER: '%s'", (data.substr(0, count)).c_str()
            );
            return hardware_interface::return_type::ERROR;
        }

        // Read response
        data = std::string(1024, '\0');
        count = serial_->read_until(data.size(), (uint8_t*)data.c_str(), '\n');

        SeaScanEcho::Reply s_data(data.substr(0, count));
        std::vector<std::string> fields_data = s_data.Fields();
        if (!s_data.Valid() && fields_data[0] != "MSALT" && fields_data[1] != "DATA") {
            RCLCPP_FATAL(
                rclcpp::get_logger("EchosounderHardware"),
                "Bad DATA: '%s'", (data.substr(0, count)).c_str()
            );
            return hardware_interface::return_type::ERROR;
        }

        // Getting the unfiltered distance (filtered distance in fields_data[2])
        distance_ = std::stod(fields_data[3]);

        RCLCPP_DEBUG(rclcpp::get_logger("EchosounderHardware"), "GOT %s", (data.substr(0, count)).c_str());
        RCLCPP_DEBUG(rclcpp::get_logger("EchosounderHardware"), "Distance: %s, %f", (fields_data[3]).c_str(), distance_);

        // Read IMAGE
        data = std::string(1048, '\0');
        count = serial_->read_until(data.size(), (uint8_t*)data.c_str(), '\n');

        return hardware_interface::return_type::OK;
    }
} // riptide_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_hardware::EchosounderHardware, hardware_interface::SensorInterface)