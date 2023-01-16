
#include <gmock/gmock.h>

#include <string>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"


class TestGenericSystem : public ::testing::Test {
    public:
        void test_generic_system_with_fake_sensor_commands(std::string & urdf);
        void test_generic_system_with_mimic_joint(std::string & urdf);

    protected:
        std::string urdf_;

        void SetUp() override {
            urdf_ = R"(<ros2_control name="SPartonAHRS" type="system">
                            <hardware>
                                <plugin>sparton_ahrs_m1_hwi/SpartonAHRSM1Interface</plugin>
                            </hardware>
                            <joint name="joint1">
                                <command_interface name="position"/>
                                <state_interface name="position"/>
                                <param name="initial_position">1.57</param>
                            </joint>
                        </ros2_control>)";
        };
};

class TestableResourceManager : public hardware_interface::ResourceManager {
    public:
        friend TestGenericSystem;

        FRIEND_TEST(TestGenericSystem, load_sparton_ahrs);

        TestableResourceManager() : hardware_interface::ResourceManager() {};

        TestableResourceManager(const std::string & urdf, bool validate_interfaces = true, bool activate_all = false)
            : hardware_interface::ResourceManager(urdf, validate_interfaces, activate_all) {};
};
        
TEST_F(TestGenericSystem, load_sparton_ahrs) {
  auto urdf = ros2_control_test_assets::urdf_head + urdf_ +
              ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(TestableResourceManager rm(urdf));
}

