#include <custom_gazebo_plugins/collisions_publisher_plugin.h>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(CollisionsPublisherPlugin)

/////////////////////////////////////////////////
CollisionsPublisherPlugin::CollisionsPublisherPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
CollisionsPublisherPlugin::~CollisionsPublisherPlugin()
{
}

/////////////////////////////////////////////////
void CollisionsPublisherPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&CollisionsPublisherPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void CollisionsPublisherPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  std::cout << contacts.contact_size() << std::endl;
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    // Normally collision1 is the robot and 2 is the other object, but not always
    std::string model_name = getModelFromCollision(contacts.contact(i).collision2());
    if (model_name == "robot")
    {
      model_name = getModelFromCollision(contacts.contact(i).collision1());
    }

  }
}

std::string CollisionsPublisherPlugin::getModelFromCollision(std::string collision)
{
    int colon_pos = collision.find(":");
    return collision.substr(0, colon_pos);
}