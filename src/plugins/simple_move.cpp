#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class SimpleMove : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;

      // Parse waypoints and times
      if (_sdf->HasElement("target_positions"))
      {
        std::string positions = _sdf->Get<std::string>("target_positions");
        std::istringstream posStream(positions);
        double x, y, z;
        while (posStream >> x >> y >> z)
          this->positions.push_back(ignition::math::Vector3d(x, y, z));
      }

      if (_sdf->HasElement("target_times"))
      {
        std::string times = _sdf->Get<std::string>("target_times");
        std::istringstream timeStream(times);
        double t;
        while (timeStream >> t)
          this->times.push_back(t);
      }

      this->loop = _sdf->Get<bool>("loop", true).first;

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&SimpleMove::OnUpdate, this));
    }

    void OnUpdate()
    {
      if (this->positions.empty() || this->times.empty())
        return;

      double simTime = this->model->GetWorld()->SimTime().Double();

      // Determine target position based on simulation time
      for (size_t i = 0; i < this->times.size() - 1; ++i)
      {
        if (simTime >= this->times[i] && simTime < this->times[i + 1])
        {
          double alpha = (simTime - this->times[i]) / (this->times[i + 1] - this->times[i]);
          // Calculate interpolated position manually
          ignition::math::Vector3d pos = this->positions[i] * (1 - alpha) + this->positions[i + 1] * alpha;

          this->model->SetWorldPose(ignition::math::Pose3d(pos, ignition::math::Quaterniond::Identity));
          break;
        }
      }

      // Loop motion if enabled
      if (this->loop && simTime > this->times.back())
        this->model->SetWorldPose(ignition::math::Pose3d(this->positions.front(), ignition::math::Quaterniond::Identity));
    }

  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    std::vector<ignition::math::Vector3d> positions;
    std::vector<double> times;
    bool loop;
  };

  GZ_REGISTER_MODEL_PLUGIN(SimpleMove)
}
