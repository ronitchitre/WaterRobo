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
      
      if(_sdf->HasElement("X_uu")){
        this->X_uu = _sdf->Get<double>("X_uu");
      }
      else{
        this->X_uu = 0.0;
      };

      if(_sdf->HasElement("Y_vv")){
        this->Y_vv = _sdf->Get<double>("Y_vv");
      }
      else{
        this->Y_vv = 0.0;
      };

      if(_sdf->HasElement("Z_ww")){
        this->Z_ww = _sdf->Get<double>("Z_vv");
      }
      else{
        this->Z_ww = 0.0;
      };

      if(_sdf->HasElement("K_pp")){
        this->K_pp = _sdf->Get<double>("K_pp");
      }
      else{
        this->K_pp = 0.0;
      };

      if(_sdf->HasElement("M_qq")){
        this->M_qq = _sdf->Get<double>("M_qq");
      }
      else{
        this->M_qq = 0.0;
      };

      if(_sdf->HasElement("M_ww")){
        this->M_ww = _sdf->Get<double>("M_ww");
      }
      else{
        this->M_ww = 0.0;
      };

      if(_sdf->HasElement("N_rr")){
        this->N_rr = _sdf->Get<double>("N_rr");
      }
      else{
        this->N_rr = 0.0;
      };

      if(_sdf->HasElement("N_vv")){
        this->N_vv = _sdf->Get<double>("N_vv");
      }
      else{
        this->N_vv = 0.0;
      };
      std::string link_name = _sdf->Get<std::string>("link");
      this->link = model->GetLink(link_name);


    };

    // Called by the world update start event
    public: void OnUpdate()
    {
      ignition::math::Vector3d vel = this->link->RelativeLinearVel();

      double X_drag = this->X_uu * vel.X() * abs(vel.X());
      double Y_drag = this->Y_vv * vel.Y() * abs(vel.Y());
      double Z_drag = this->Z_ww * vel.Z() * abs(vel.Z());
      ignition::math::Vector3d force(X_drag, Y_drag, Z_drag);

      this->link->AddRelativeForce(force);

      ignition::math::Vector3d ang_vel = this->link->RelativeAngularVel();

      double K_drag = this->K_pp * ang_vel.X() * abs(ang_vel.X());
      double M_drag = this->M_qq * ang_vel.Y() * abs(ang_vel.Y());
      double N_drag = this->N_rr * ang_vel.Z() * abs(ang_vel.Z());
      ignition::math::Vector3d torque(K_drag, M_drag, N_drag);

      this->link->AddRelativeTorque(torque);


      
    }

    private: 

    // Pointer to the model
    physics::ModelPtr model;

    physics::LinkPtr link;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    double X_uu, Y_vv, Z_ww, K_pp, M_qq, M_ww, N_rr, N_vv;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(HydroDrag)
}