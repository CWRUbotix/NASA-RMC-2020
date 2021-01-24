#ifndef COLLISIONS_PUBLISHER_PLUGIN_H_
#define COLLISIONS_PUBLISHER_PLUGIN_H_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class CollisionsPublisherPlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: CollisionsPublisherPlugin();

    /// \brief Destructor.
    public: virtual ~CollisionsPublisherPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the contact sensor's update signal.
    private: virtual void OnUpdate();

    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;

    private: std::string getModelFromCollision(std::string collision);
  };
}
#endif // COLLISIONS_PUBLISHER_PLUGIN_H_