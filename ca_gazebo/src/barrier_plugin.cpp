#include <functional>
//ros libraries
#include <ros/ros.h>
#include <ros/console.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#define GOING_UP 0
#define GOING_DOWN 1
#define IS_UP 2
#define IS_DOWN 3
#define BARRIER_VEL 0.8
namespace gazebo
{
class ModelPush : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    this->model = _parent;
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ModelPush::OnUpdate, this));
    this->world_ = this->model->GetJoint("barrier_joint")->GetWorld();
    this->last_update_ = this->world_->SimTime();
    this->update_period_ = 1 / (_sdf->Get<double>("update_rate"));
  }

  // Called by the world update start event
public:
  void OnUpdate()
  {
    static int i = 1;
    // Apply a small linear velocity to the model.
    float position = this->model->GetJoint("barrier_joint")->Position();
    static int state = GOING_UP;

    switch (state)
    {
    case GOING_DOWN:
      this->model->GetJoint("barrier_joint")->SetVelocity(0, - BARRIER_VEL);
      if (position <= HORIZONTAL_POSITION)
      {
        state = IS_DOWN;
        this->model->GetJoint("barrier_joint")->SetVelocity(0, 0);
        this->last_update_ = this->world_->SimTime();
      }
      break;

    case GOING_UP:
      this->model->GetJoint("barrier_joint")->SetVelocity(0, BARRIER_VEL);
      if (position >= VERTICAL_POSITION)
      {
        state = IS_UP;
        this->model->GetJoint("barrier_joint")->SetVelocity(0, 0);
        this->last_update_ = this->world_->SimTime();
      }
      break;
    case IS_UP:
      if ((this->world_->SimTime() - this->last_update_) > this->update_period_)
      {
        state = GOING_DOWN;
      }
      break;
    case IS_DOWN:
      if ((this->world_->SimTime() - this->last_update_) > this->update_period_)
      {
        state = GOING_UP;
      }
      break; // code to be executed if n doesn't match any cases
    }
  }

  // Pointer to the model
private:
  physics::ModelPtr model;
  event::ConnectionPtr updateConnection; // Pointer to the update event connection
  common::Time last_update_;
  physics::WorldPtr world_;
  const double HORIZONTAL_POSITION = 1.577;
  const double VERTICAL_POSITION = 3.145;
  double update_period_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)
} // namespace gazebo
