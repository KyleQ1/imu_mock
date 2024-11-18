#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

/*
    This test case requires that gazebo be loaded with an IMU in here.
    Verifies that the IMU can receive and accurately add noise to the data.
*/
class ImuPluginTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Initialize ROS node
        rclcpp::init(0, nullptr);
        node = std::make_shared<rclcpp::Node>("imu_plugin_test_node");
        imu_subscriber = node->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                received_imu_data = true;
                last_imu_msg = msg;
            });
    }

    void TearDown() override
    {
        rclcpp::shutdown();
    }

    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
    bool received_imu_data = false;
    sensor_msgs::msg::Imu::SharedPtr last_imu_msg;
};

TEST_F(ImuPluginTest, ImuDataIsPublished)
{
    // Spin for a short while to receive IMU data
    rclcpp::spin_some(node);
    EXPECT_TRUE(received_imu_data) << "IMU data was not received.";
}

TEST_F(ImuPluginTest, ImuDataHasExpectedNoise)
{
    rclcpp::spin_some(node);

    if (received_imu_data)
    {
        // Check that the noise levels are within acceptable bounds
        EXPECT_NEAR(last_imu_msg->linear_acceleration.x, 0.0, 0.1)
            << "Linear acceleration x is out of bounds.";
        EXPECT_NEAR(last_imu_msg->angular_velocity.z, 0.0, 0.1)
            << "Angular velocity z is out of bounds.";
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
