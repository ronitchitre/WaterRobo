#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>


namespace gazebo
{
  class HydroDrag : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {

      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&HydroDrag::OnUpdate, this));
      
      if(_sdf->HasElement("X_u")){
        this->X_u = _sdf->Get<double>("X_u");
      }
      else{
        this->X_u = 0.0;
      };

      if(_sdf->HasElement("Y_v")){
        this->Y_v = _sdf->Get<double>("Y_v");
      }
      else{
        this->Y_v = 0.0;
      };

      if(_sdf->HasElement("Z_w")){
        this->Z_w = _sdf->Get<double>("Z_v");
      }
      else{
        this->Z_w = 0.0;
      };

      if(_sdf->HasElement("K_p")){
        this->K_p = _sdf->Get<double>("K_p");
      }
      else{
        this->K_p = 0.0;
      };

      if(_sdf->HasElement("M_q")){
        this->M_q = _sdf->Get<double>("M_q");
      }
      else{
        this->M_q = 0.0;
      };

      if(_sdf->HasElement("N_r")){
        this->N_r = _sdf->Get<double>("N_r");
      }
      else{
        this->N_r = 0.0;
      };

      std::string link_name = _sdf->Get<std::string>("link");
      this->link = this->model->GetLink(link_name);


    };

    // Called by the world update start event
    public: void OnUpdate()
    {
      ignition::math::Vector3d vel = this->link->RelativeLinearVel();
      ignition::math::Vector3d acc = this->link->RelativeLinearAccel();
      ignition::math::Vector3d ang_vel = this->link->RelativeAngularVel();
      ignition::math::Vector3d ang_acc = this->link->RelativeAngularAccel();

      double X_a = this->X_u * acc.X() + this->Z_w * vel.Z() * ang_vel.Y() - this->Y_v * vel.Y() * ang_vel.Z();
      double Y_a = this->Y_v * acc.Y() + this->X_u * vel.X() * ang_vel.Z() - this->Z_w * vel.Z() * ang_vel.X();
      double Z_a = this->Z_w * acc.Z() + this->Y_v * vel.Y() * ang_vel.X() - this->X_u * vel.X() * ang_vel.Y();

      ignition::math::Vector3d force(-1 * X_a, -1 * Y_a, -1 * Z_a);

      double K_a = this->K_p * ang_acc.X() - (this->Y_v - this->Z_w) * vel.Y() * vel.Z() - (this->M_q - this->N_r) * ang_vel.Y() * ang_vel.Z();
      double M_a = this->M_q * ang_acc.Y() - (this->Z_w - this->X_u) * vel.Z() * vel.X() + (this->K_p - this->N_r) * ang_vel.Z() * ang_vel.X();
      double N_a = this->N_r * ang_acc.Z() - (this->X_u - this->Y_v) * vel.X() * vel.Y() - (this->K_p - this->M_q) * ang_vel.X() * ang_vel.Y();

      ignition::math::Vector3d torque(-1 * K_a, -1 * M_a, -1 * N_a);

      this->link->AddRelativeForce(force);
      this->link->AddRelativeTorque(torque);      
    }

    private: 

    // Pointer to the model
    physics::ModelPtr model;

    physics::LinkPtr link;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    double X_u, Y_v, Z_w, K_p, M_q, N_r;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(HydroDrag)
}