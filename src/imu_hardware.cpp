#include "riptide_hardware/imu_hardware.hpp"

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include "sparton_ahrs_m1_driver/sparton_ahrs_m1_driver.hpp"

#include <algorithm>
#include <memory>


namespace riptide_hardware {

    CallbackReturn IMUHardware::on_init(const hardware_interface::HardwareInfo & info) {
        if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Checking port and baud rate specified in ros2_control urdf tag
        if (info_.hardware_parameters.find("port") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("IMUHardware"),
                "You need to specify the serial port in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info_.hardware_parameters.find("baud_rate") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("IMUHardware"),
                "You need to specify the serial baud rate in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Getting port and baud_rate parameters
        port_ = info_.hardware_parameters["port"];
        baud_rate_ = stoi(info_.hardware_parameters["baud_rate"]);
        
        RCLCPP_INFO(rclcpp::get_logger("IMUHardware"), "port: %s - baud rate: %d", port_.c_str(), baud_rate_);

        // Resizing hw_sensor_states with the number of state interfaces
        hw_sensor_states_.resize(info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

        RCLCPP_INFO(rclcpp::get_logger("IMUHardware"), "Successfully initialized !");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> IMUHardware::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // export sensor state interface
        for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
        }

        return state_interfaces;
    }

    CallbackReturn IMUHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("IMUHardware"), "Activating ...please wait...");

        // Instanciate the driver
        // try {
            driver_ = std::make_unique<SpartonAHRSM1Driver>(port_, baud_rate_);
            bool ret = driver_->reset();
            if (!ret) {
                RCLCPP_FATAL(
                    rclcpp::get_logger("IMUHardware"),
                    "Driver reset was not sucessful!"
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
        // }
        // catch(boost::system::system_error& e) {
        //     RCLCPP_FATAL(
        //         rclcpp::get_logger("IMUHardware"),
        //         "Serial error: '%s'", e.what()
        //     );
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        // Set joint state
        for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); ++i) {
            if (std::isnan(hw_sensor_states_[i]))
                hw_sensor_states_[i] = 0;
        }

        RCLCPP_INFO(rclcpp::get_logger("IMUHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn IMUHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("IMUHardware"), "Deactivating ...please wait...");

        // Destruct the driver pointer
        driver_ = nullptr;

        RCLCPP_INFO(rclcpp::get_logger("IMUHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type IMUHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        // Accelerometer
        std::vector<float> a;
        try {
            a = driver_->read_accelerometer();
        }
        catch(boost::system::system_error& e) {
            RCLCPP_FATAL(
                rclcpp::get_logger("IMUHardware"),
                "Serial error: '%s'", e.what()
            );
            return hardware_interface::return_type::ERROR;
        }

        std::copy(a.begin(), a.end(), hw_sensor_states_.begin());

        // Gyrometer
        std::vector<float> g;
        try {
            g = driver_->read_gyroscope();
        }
        catch(boost::system::system_error& e) {
            RCLCPP_FATAL(
                rclcpp::get_logger("IMUHardware"),
                "Serial error: '%s'", e.what()
            );
            return hardware_interface::return_type::ERROR;
        }

        std::copy(g.begin(), g.end(), hw_sensor_states_.begin()+3);

        // Magnetometer
        std::vector<float> m;
        try {
            m = driver_->read_magnetometer();
        }
        catch(boost::system::system_error& e) {
            RCLCPP_FATAL(
                rclcpp::get_logger("IMUHardware"),
                "Serial error: '%s'", e.what()
            );
            return hardware_interface::return_type::ERROR;
        }

        std::copy(m.begin(), m.end(), hw_sensor_states_.begin()+6);

        // Debug
        std::stringstream ss;
        std::copy(hw_sensor_states_.begin(), hw_sensor_states_.end(), std::ostream_iterator<float>(ss, " "));
        RCLCPP_DEBUG(rclcpp::get_logger("IMUHardware"), "Reading : %s", (ss.str()).c_str());

        return hardware_interface::return_type::OK;
    }
} // riptide_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_hardware::IMUHardware, hardware_interface::SensorInterface)