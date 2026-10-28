#include <gtest/gtest.h>
#include <juxie_ros2_control/jx_hardware.h>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <filesystem>
#include <fstream>
#include <unistd.h>

using juxie_ros2_control::JxHardware;
using hardware_interface::CallbackReturn;

class GravityTableRequired : public ::testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    hardware_interface::HardwareComponentParams params(const std::string& path) {
        hardware_interface::HardwareComponentParams result;
        result.executor = executor;
        result.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
        auto& info = result.hardware_info;
        info.name = "gravity_table_test";
        info.type = "system";
        info.hardware_parameters = {{"motor_ids", "1,2,3,4"},
                                    {"control_mode", "current"}, {"gravity_ff_table", path}};
        for (int i = 1; i <= 4; ++i) {
            hardware_interface::ComponentInfo joint;
            joint.name = "body_joint" + std::to_string(i);
            for (const std::string name : {"position", "velocity"}) {
                hardware_interface::InterfaceInfo interface;
                interface.name = name;
                joint.command_interfaces.push_back(interface);
                joint.state_interfaces.push_back(interface);
            }
            info.joints.push_back(joint);
        }
        return result;
    }
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
};

TEST_F(GravityTableRequired, EmptyPathRejectsCurrentLoopInitialization) {
    JxHardware hardware;
    EXPECT_EQ(hardware.init(params("")), CallbackReturn::ERROR);
}

TEST_F(GravityTableRequired, MissingFileRejectsInitializationAndActivation) {
    JxHardware hardware;
    const auto missing = (std::filesystem::temp_directory_path() /
        ("missing_juxie_gravity_" + std::to_string(getpid()))).string();
    ASSERT_FALSE(std::filesystem::exists(missing));
    EXPECT_EQ(hardware.init(params(missing)), CallbackReturn::ERROR);
    EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
}

TEST_F(GravityTableRequired, IncompleteTableRejectsInitialization) {
    char file_template[] = "/tmp/juxie_table_test_XXXXXX";
    const int fd = mkstemp(file_template);
    ASSERT_GE(fd, 0);
    close(fd);
    {
        std::ofstream file(file_template);
        file << "2 2 2\n0 0 0\n1 1 1\n0 0 0 100 200 300 400\n";
    }
    JxHardware hardware;
    EXPECT_EQ(hardware.init(params(file_template)), CallbackReturn::ERROR);
    std::filesystem::remove(file_template);
}
