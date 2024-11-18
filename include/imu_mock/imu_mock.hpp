#ifndef IMU_MOCK_HPP
#define IMU_MOCK_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace gazebo
{
  class IMUMock : public SensorPlugin
  {
  public:
    IMUMock();
    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

  private:
    void OnUpdate();
    double GaussianNoise();

    sensors::ImuSensorPtr m_imuSensor;
    rclcpp::Node::SharedPtr m_rosNode;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuPublisher;
    event::ConnectionPtr m_updateConnection;
    double m_updateRate;
    double m_gaussianNoise;
  };
}

#endif // IMU_MOCK_HPP