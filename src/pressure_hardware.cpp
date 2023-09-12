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

                // Comparing if rate is not too high
                if (std::abs(driver_->pressure()*100 - driver_states_[0]) > max_pressure_rate_) {
                    RCLCPP_WARN(
                        rclcpp::get_logger("PressureHardware"),
                        "Pressure rate is too high! (current: %f, previous: %f, max: %f)",
                        driver_->pressure(), driver_states_[0], max_pressure_rate_
                    );
                }
                else {
                    driver_states_[0] = driver_->pressure()*100; // Pa
                }

                // Setting temperature
                driver_states_[1] = driver_->temperature(); // °C

                // Comparing if depth rate is not too high
                if (std::abs(driver_->depth() - driver_states_[2]) > max_pressure_rate_ / 10000.) {
                    RCLCPP_WARN(
                        rclcpp::get_logger("PressureHardware"),
                        "Depth rate is too high! (current: %f, previous: %f, max: %f)",
                        driver_->depth(), driver_states_[2], max_pressure_rate_ / 10000.
                    );
                }
                else {
                    driver_states_[2] = driver_->depth(); // m
                }

                // Setting altitude
                driver_states_[3] = driver_->altitude(); // m
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
        if (info_.hardware_parameters.find("calibration_pressure") != info_.hardware_parameters.end()) {
            calibration_pressure_ = std::stod(info_.hardware_parameters["calibration_pressure"]);
        }
        else {
            calibration_pressure_ = 101300.0;   // Pa
        }

        // Getting fluid density
        if (info_.hardware_parameters.find("fluid_density") != info_.hardware_parameters.end()) {
            fluid_density_ = std::stod(info_.hardware_parameters["fluid_density"]);
        }
        else {
            fluid_density_ = 1023.6;    // kg.m^{-3}
        }

        // Getting calibration pressure
        if (info_.hardware_parameters.find("max_pressure_rate") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("PressureHardware"),
                "You need to specify the max pressure rate in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        max_pressure_rate_ = std::stod(info_.hardware_parameters["max_pressure_rate"]);

        RCLCPP_DEBUG(rclcpp::get_logger("PressureHardware"), "Calibration pressure: %f", calibration_pressure_);

        // Getting port and baud_rate parameters
        port_ = info_.hardware_parameters["port"];
        
        RCLCPP_DEBUG(rclcpp::get_logger("PressureHardware"), "port: %s", port_.c_str());

        // Resizing hw_sensor_states with the number of state interfaces
        hw_sensor_states_.resize(info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

        RCLCPP_DEBUG(rclcpp::get_logger("PressureHardware"), "Successfully initialized !");
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

        // Setting fluid density
        driver_->setFluidDensity(fluid_density_);
        
        // Setting calibration pressure
        driver_->setCalibrationPressure(calibration_pressure_/100.);

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