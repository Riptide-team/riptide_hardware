#include "riptide_hardware/echosounder_hardware.hpp"

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include <mutex>
#include <string>
#include <string_view>

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

        RCLCPP_INFO(rclcpp::get_logger("EchosounderHardware"), "RESET message witten! %d/%ld char written", count, command.size());

        // Send MARCO message
        msg = SeaScanEcho::Commands::Marco;
        command = msg();
        int count = serial_->write(command.size(), (const uint8_t*)command.c_str());

        RCLCPP_INFO(rclcpp::get_logger("EchosounderHardware"), "MARCO message witten! %d/%ld char written", count, command.size());

        // Read POLO response
        std::string data(1024, '\0');
        count = serial_->read_until(data.size(), (uint8_t*)data.c_str(), '\n');
        RCLCPP_INFO(rclcpp::get_logger("EchosounderHardware"), "POLO message read!");

        SeaScanEcho::Reply s(data.substr(0, count));

        std::vector<std::string_view> fields = s.Fields();
        if (!s.Valid() && fields[0] != "MSALT" && fields[1] != "POLO") {
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
        std::vector<std::string_view> fields = s.Fields();
        if (!s.Valid() && fields[0] != "MSALT" && fields[1] != "DATA") {
            RCLCPP_FATAL(
                rclcpp::get_logger("EchosounderHardware"),
                "Bad response to TRIGGER: '%s'", (data.substr(0, count)).c_str()
            );
            return hardware_interface::return_type::ERROR;
        }

        // Getting the distance
        distance_ = std::stod(std::string(fields[2]));

        return hardware_interface::return_type::OK;
    }
} // riptide_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_hardware::EchosounderHardware, hardware_interface::SensorInterface)