#include "imu_mock/imu_mock.hpp"

namespace gazebo
{
    IMUMock::IMUMock() : SensorPlugin(), m_updateRate(10.0), m_gaussianNoise(0.01) {}

    void IMUMock::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {
        if (!rclcpp::ok()) {
            RCLCPP_FATAL(rclcpp::get_logger("imu_plugin"), "A ROS2 node for Gazebo has not been initialized.");
            return;
        }

        // Save the pointer to the sensor
        m_imuSensor = std::dynamic_pointer_cast<sensors::ImuSensor>(_sensor);
        if (!m_imuSensor) {
            RCLCPP_ERROR(rclcpp::get_logger("imu_plugin"), "IMUMock requires an ImuSensor.");
            return;
        }

        // Read parameters from SDF
        if (_sdf->HasElement("update_rate"))
            m_updateRate = _sdf->Get<double>("update_rate");

        if (_sdf->HasElement("gaussianNoise"))
            m_gaussianNoise = _sdf->Get<double>("gaussianNoise");

        m_rosNode = rclcpp::Node::make_shared("imu_plugin");

        m_imuPublisher = m_rosNode->create_publisher<sensor_msgs::msg::Imu>("imu/data", m_updateRate);

        // Connect the sensor update function
        m_updateConnection = m_imuSensor->ConnectUpdated(
            std::bind(&IMUMock::OnUpdate, this));

        m_imuSensor->SetActive(true);
    }

    void IMUMock::OnUpdate()
    {
        // Create an IMU message
        auto imuMsg = sensor_msgs::msg::Imu();

        // Fill the IMU message with data from the sensor
        imuMsg.header.stamp = m_rosNode->now();
        imuMsg.header.frame_id = "imu_link";

        // Simulate data with Gaussian noise
        imuMsg.linear_acceleration.x = m_imuSensor->LinearAcceleration().X() + GaussianNoise();
        imuMsg.linear_acceleration.y = m_imuSensor->LinearAcceleration().Y() + GaussianNoise();
        imuMsg.linear_acceleration.z = m_imuSensor->LinearAcceleration().Z() + GaussianNoise();

        imuMsg.angular_velocity.x = m_imuSensor->AngularVelocity().X() + GaussianNoise();
        imuMsg.angular_velocity.y = m_imuSensor->AngularVelocity().Y() + GaussianNoise();
        imuMsg.angular_velocity.z = m_imuSensor->AngularVelocity().Z() + GaussianNoise();

        m_imuPublisher->publish(imuMsg);
    }

    double IMUMock::GaussianNoise() {
        static std::default_random_engine generator;
        std::normal_distribution<double> distribution(0.0, m_gaussianNoise);
        return distribution(generator);
    }

  GZ_REGISTER_SENSOR_PLUGIN(IMUMock)
}
