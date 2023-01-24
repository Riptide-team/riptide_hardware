#include "riptide_hardware/battery_card_hardware.hpp"

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include <mutex>
#include <string>

#include <nlohmann/json.hpp>
using json = nlohmann::json;


namespace riptide_hardware {
    void BatteryCardHardware::serial_callback(rtac::asio::Stream::Ptr /*stream*/, std::string* data, const rtac::asio::SerialStream::ErrorCode& /*err*/, std::size_t count) {
        std::scoped_lock<std::mutex> lock_(json_mutex_);
        received_data_ = json::parse((*data).substr(0, count));

        RCLCPP_INFO(rclcpp::get_logger("BatteryCardHardware"), "Received: %s", (*data).substr(0, count).c_str());

        buffer_ = std::string(1024, '\0');
        if(!serial_->async_read_until(buffer_.size(), (uint8_t*)buffer_.c_str(), '}',
            std::bind(&BatteryCardHardware::serial_callback, this, serial_, &buffer_, std::placeholders::_1, std::placeholders::_2))
        ) {
            RCLCPP_FATAL(
                rclcpp::get_logger("BatteryCardHardware"),
                "Unable to launch async read until"
            );
        }
    }

    CallbackReturn BatteryCardHardware::on_init(const hardware_interface::HardwareInfo & info) {
        if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Checking that there is exactly two states interfaces: tension and current
        if (info_.sensors[0].state_interfaces.size() != 2) {
            RCLCPP_FATAL(
                rclcpp::get_logger("BatteryCardHardware"),
                "The hardware interface requires two state interfaces : tension, current!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (info_.sensors[0].state_interfaces[0].name != "tension") {
            RCLCPP_FATAL(
                rclcpp::get_logger("BatteryCardHardware"),
                "The first state interface type has to be tension!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (info_.sensors[0].state_interfaces[1].name != "current") {
            RCLCPP_FATAL(
                rclcpp::get_logger("BatteryCardHardware"),
                "The second state interface type has to be current!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Checking port and baud rate specified in ros2_control urdf tag
        if (info_.hardware_parameters.find("port") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("BatteryCardHardware"),
                "You need to specify the serial port in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info_.hardware_parameters.find("baud_rate") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("BatteryCardHardware"),
                "You need to specify the serial baud rate in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Getting port and baud_rate parameters
        port_ = info_.hardware_parameters.at("port");
        baud_rate_ = stoi(info_.hardware_parameters.at("baud_rate"));
        
        RCLCPP_INFO(rclcpp::get_logger("BatteryCardHardware"), "port: %s - baud rate: %d", port_.c_str(), baud_rate_);

        // Initialize state interfaces to std::quiet_NaN
        tension_ = std::numeric_limits<double>::quiet_NaN();
        current_ = std::numeric_limits<double>::quiet_NaN();

        received_data_ = json::parse("\"volt\": 16.,\"current\": 0.}");
        
        RCLCPP_INFO(rclcpp::get_logger("BatteryCardHardware"), "Successfully initialized !");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> BatteryCardHardware::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.sensors[0].name,
                info_.sensors[0].state_interfaces[0].name,
                &tension_
            )
        );
        
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.sensors[0].name,
                info_.sensors[0].state_interfaces[1].name,
                &current_
            )
        );

        return state_interfaces;
    }

    CallbackReturn BatteryCardHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("BatteryCardHardware"), "Activating ...please wait...");

        // Instanciate the driver
        try {
            serial_ = rtac::asio::Stream::CreateSerial(port_, baud_rate_);
            serial_->start();
            serial_->flush();
        }
        catch(boost::system::system_error& e) {
            RCLCPP_FATAL(
                rclcpp::get_logger("BatteryCardHardware"),
                "Serial error: '%s'", e.what()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Launching an asynchronous read on the serial until '}' is on the string
        buffer_ = std::string(1024, '\0');
        if(!serial_->async_read_until(buffer_.size(), (uint8_t*)buffer_.c_str(), '}',
            std::bind(&BatteryCardHardware::serial_callback, this, serial_, &buffer_, std::placeholders::_1, std::placeholders::_2))
        ) {
            RCLCPP_FATAL(
                rclcpp::get_logger("BatteryCardHardware"),
                "Unable to launch async read until"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(
            rclcpp::get_logger("BatteryCardHardware"),
            "Started the async read_until"
        );

        RCLCPP_INFO(rclcpp::get_logger("BatteryCardHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn BatteryCardHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("BatteryCardHardware"), "Deactivating ...please wait...");

        // Destruct the driver pointer
        serial_ = nullptr;

        RCLCPP_INFO(rclcpp::get_logger("BatteryCardHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type BatteryCardHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        tension_ = received_data_["volt"];
        current_ = received_data_["current"];

        return hardware_interface::return_type::OK;
    }
} // riptide_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_hardware::BatteryCardHardware, hardware_interface::SensorInterface)