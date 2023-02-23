#include "riptide_hardware/pressure_hardware.hpp"

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include <ms5837_driver/ms5837_driver.hpp>

#include <algorithm>
#include <memory>
#include <thread>
#include <mutex>


namespace riptide_hardware {
    void PressureHardware::async_read_data() {
        while (true == thread_running_.load()) {
            bool read = driver_->read_data();
            if (read) {
                std::scoped_lock<std::mutex> lock_(data_mutex_);
                driver_states_ = std::vector<double> {
                    driver_->pressure(),
                    driver_->temperature(),
                    (driver_->pressure() - calibration_pressure_) / 100.,
                    driver_->altitude()
                };
            }
        }
    }

    CallbackReturn PressureHardware::on_init(const hardware_interface::HardwareInfo & info) {
        if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Checking port and baud rate specified in ros2_control urdf tag
        if (info_.hardware_parameters.find("port") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("PressureHardware"),
                "You need to specify the serial port in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Getting calibration pressure
        if (info_.hardware_parameters.find("calibration_pressure") == info_.hardware_parameters.end()) {
            calibration_pressure_ = std::stod(info_.hardware_parameters["calibration_pressure"]);
        }
        else {
            calibration_pressure_ = 1013.0;
        }

        // Getting port and baud_rate parameters
        port_ = info_.hardware_parameters["port"];
        
        RCLCPP_INFO(rclcpp::get_logger("PressureHardware"), "port: %s", port_.c_str());

        // Resizing hw_sensor_states with the number of state interfaces
        hw_sensor_states_.resize(info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

        RCLCPP_INFO(rclcpp::get_logger("PressureHardware"), "Successfully initialized !");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> PressureHardware::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // export sensor state interface
        for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
        }

        return state_interfaces;
    }

    CallbackReturn PressureHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("PressureHardware"), "Activating ...please wait...");

        // Instanciate the driver
        driver_ = std::make_unique<MS5837Driver>(port_);
        bool init = driver_->init();
        if (!init) {
            RCLCPP_FATAL(
                rclcpp::get_logger("PressureHardware"),
                "Unable to init the driver!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Set joint state
        for (uint8_t i = 0; i < info_.sensors[0].state_interfaces.size(); ++i) {
            if (std::isnan(hw_sensor_states_[i]))
                hw_sensor_states_[i] = 0;
        }

        thread_running_.store(true);
        thread_ = std::thread(&PressureHardware::async_read_data, this);
        thread_.detach();

        RCLCPP_INFO(rclcpp::get_logger("PressureHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn PressureHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("PressureHardware"), "Deactivating ...please wait...");

        thread_running_.store(false);

        // Destruct the driver pointer
        driver_ = nullptr;

        RCLCPP_INFO(rclcpp::get_logger("PressureHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type PressureHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        std::scoped_lock<std::mutex> lock_(data_mutex_);
        hw_sensor_states_ = driver_states_;

        return hardware_interface::return_type::OK;
    }
} // riptide_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_hardware::PressureHardware, hardware_interface::SensorInterface)