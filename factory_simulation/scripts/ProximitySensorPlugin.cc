#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo {
  class ProximitySensorPlugin : public SensorPlugin {
    public:
      void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/) override {
        if (!_sensor) {
          gzerr << "Invalid sensor pointer.\n";
          return;
        }

        this->sensor = std::dynamic_pointer_cast<sensors::ProximitySensor>(_sensor);
        if (!this->sensor) {
          gzerr << "ProximitySensorPlugin requires a proximity sensor.\n";
          return;
        }

        // Connect to the sensor update event.
        this->updateConnection = this->sensor->ConnectUpdated(
          std::bind(&ProximitySensorPlugin::OnUpdate, this));
      }

      // Custom update function for the sensor data.
      void OnUpdate() {
        if (this->sensor) {
          // Simulate the proximity sensor data here.
          double distance = 1.0; // Replace with your distance calculation logic.
          this->sensor->SetRange(distance);
        }
      }

    private:
      sensors::ProximitySensorPtr sensor;
      event::ConnectionPtr updateConnection;
  };
  GZ_REGISTER_SENSOR_PLUGIN(ProximitySensorPlugin)
}

