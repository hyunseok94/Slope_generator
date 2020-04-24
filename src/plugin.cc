#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <functional>
#include <ignition/math/Vector3.hh>
#include <Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>

#define PI      3.141592
#define D2R     3.141592/180
#define R2D     180/PI

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

namespace gazebo
{

    class SLOPE_GENERATOR : public ModelPlugin
    {
        physics::LinkPtr BASE_LINK;
        physics::LinkPtr MIDDLE_LINK;
        physics::LinkPtr INIT_LINK;
        physics::LinkPtr END_LINK;

        physics::JointPtr MIDDLE_JOINT;
        physics::JointPtr INIT_JOINT;
        physics::JointPtr END_JOINT;

        physics::ModelPtr model;

        VectorXd init_angle = VectorXd::Zero(3);
        VectorXd target_angle = VectorXd::Zero(3);
        VectorXd goal_angle = VectorXd::Zero(3);
        VectorXd target_angle_dot = VectorXd::Zero(3);

        VectorXd actual_angle = VectorXd::Zero(3);
        VectorXd actual_angle_dot = VectorXd::Zero(3);

        VectorXd torque = VectorXd::Zero(3);
        VectorXd kp = VectorXd::Zero(3);
        VectorXd kd = VectorXd::Zero(3);

        //setting for getting <dt>(=derivative time)
        common::Time last_update_time;
        event::ConnectionPtr update_connection;
        double dt=0.001;
        double step_cnt=3000;
        double inclination=0.0;

        bool target_flag=false;

        unsigned int cnt=0;

        ros::NodeHandle n;
        ros::Subscriber S_target;

        //For model load
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
        void UpdateAlgorithm();
        void Callback(const std_msgs::Float64Ptr &msg);
    };
    GZ_REGISTER_MODEL_PLUGIN(SLOPE_GENERATOR);
}

void gazebo::SLOPE_GENERATOR::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    // model = link + joint +sensor
    this->model = _model;

    //LINK DEFINITION
    this->BASE_LINK = this->model->GetLink("BASE_LINK");
    this->MIDDLE_LINK = this->model->GetLink("MIDDLE_LINK");
    this->INIT_LINK = this->model->GetLink("INIT_LINK");
    this->END_LINK = this->model->GetLink("END_LINK");

    //JOINT DEFINITION
    this->MIDDLE_JOINT = this->model->GetJoint("MIDDLE_JOINT");
    this->INIT_JOINT = this->model->GetJoint("INIT_JOINT");
    this->END_JOINT = this->model->GetJoint("END_JOINT");

    this->last_update_time = this->model->GetWorld()->GetSimTime();
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SLOPE_GENERATOR::UpdateAlgorithm, this));

    //ROS Communication setting

    S_target = n.subscribe("INCLINATION", 1, &gazebo::SLOPE_GENERATOR::Callback, this);
}

void gazebo::SLOPE_GENERATOR::UpdateAlgorithm()
{
    //* Calculate time
    common::Time current_time = this->model->GetWorld()->GetSimTime();

   actual_angle[0]=this->MIDDLE_JOINT->GetAngle(1).Radian();
   actual_angle[1]=this->INIT_JOINT->GetAngle(1).Radian();
   actual_angle[2]=this->END_JOINT->GetAngle(1).Radian();

   actual_angle_dot[0]=this->MIDDLE_JOINT->GetVelocity(1);
   actual_angle_dot[1]=this->INIT_JOINT->GetVelocity(1);
   actual_angle_dot[2]=this->END_JOINT->GetVelocity(1);

   kp<<2500000,1000000,1000000;
   kd<<200000,80000,80000;

    if(cnt==0){
      init_angle=goal_angle;
      goal_angle[0]=inclination;  //middle
      goal_angle[1]=-inclination; //init
      goal_angle[2]=-inclination;  //end
      target_flag=false;
      cnt++;
    }
    else if(cnt<step_cnt){
      target_angle=init_angle+(goal_angle-init_angle)/2.0*(1-cos(PI/step_cnt*cnt));
      cnt++;
    }
   else{
      target_angle=goal_angle;
      if(target_flag==true){
        cnt=0;
      }
   }

    //Calculate Torque
    for(int i=0; i<3; i++){
      torque[i]=kp[i]*(target_angle[i]-actual_angle[i])+kd[i]*(target_angle_dot[i]-actual_angle_dot[i]);
    }
    
    //* Apply torque to joint
    this->MIDDLE_JOINT->SetForce(1, torque[0]);
    this->INIT_JOINT->SetForce(1, torque[1]);
    this->END_JOINT->SetForce(1, torque[2]);

    //*setting for getting dt
    this->last_update_time = current_time;
}

void gazebo::SLOPE_GENERATOR::Callback(const std_msgs::Float64Ptr &msg)
{
    inclination=msg->data*D2R;
    target_flag=true;
}
