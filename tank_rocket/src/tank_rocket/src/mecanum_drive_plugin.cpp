#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
namespace gazebo
{
class MecanumDrivePlugin : public ModelPlugin
{
public:
  MecanumDrivePlugin() {}
  ~MecanumDrivePlugin() {}
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->model_ = model;
    this->world_ = model->GetWorld();
    if (sdf->HasElement("cmdVelTopic"))
      cmd_vel_topic_ = sdf->Get<std::string>("cmdVelTopic");
    else
      cmd_vel_topic_ = "/cmd_vel";

    if (sdf->HasElement("wheelRadius"))
      wheel_radius_ = sdf->Get<double>("wheelRadius");
    else
      wheel_radius_ = 0.065; 

    if (sdf->HasElement("wheelSeparationWidth"))
      wheel_separation_width_ = sdf->Get<double>("wheelSeparationWidth");
    else
      wheel_separation_width_ = 0.19; 

    if (sdf->HasElement("wheelSeparationLength"))
      wheel_separation_length_ = sdf->Get<double>("wheelSeparationLength");
    else
    wheel_separation_length_ = 0.125;
    front_right_joint_name_ = GetSDFElement(sdf, "frontRightJoint", "joint_right_front");
    back_right_joint_name_  = GetSDFElement(sdf, "backRightJoint",  "joint_right_back");
    front_left_joint_name_  = GetSDFElement(sdf, "frontLeftJoint",  "joint_left_front");
    back_left_joint_name_   = GetSDFElement(sdf, "backLeftJoint",   "joint_left_back");
    front_right_joint_ = model->GetJoint(front_right_joint_name_);
    back_right_joint_  = model->GetJoint(back_right_joint_name_);
    front_left_joint_  = model->GetJoint(front_left_joint_name_);
    back_left_joint_   = model->GetJoint(back_left_joint_name_);
    if (!front_right_joint_ || !back_right_joint_ || !front_left_joint_ || !back_left_joint_) {
      gzerr << "[MecanumDrivePlugin] Could not find all the joints. Plugin won't work.\n";
      return;
    }
    ros::NodeHandle nh("~");
    cmd_vel_sub_ = nh.subscribe(cmd_vel_topic_, 1, &MecanumDrivePlugin::CmdVelCallback, this);
    update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&MecanumDrivePlugin::OnUpdate, this));
    ROS_INFO_STREAM("[MecanumDrivePlugin] Loaded with topic " << cmd_vel_topic_);
  }
  void OnUpdate()
  {
    double R = wheel_radius_;
    double L = wheel_separation_length_;
    double W = wheel_separation_width_;
    double omegaRF = (vx_ - vy_ - (L+W)*wz_) / R;
    double omegaRB = (vx_ + vy_ - (L+W)*wz_) / R;
    double omegaLF = (vx_ + vy_ + (L+W)*wz_) / R;
    double omegaLB = (vx_ - vy_ + (L+W)*wz_) / R;
    front_right_joint_->SetVelocity(0, omegaRF);
    back_right_joint_->SetVelocity(0, omegaRB);
    front_left_joint_->SetVelocity(0, omegaLF);
    back_left_joint_->SetVelocity(0, omegaLB);
  }
  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
    wz_ = msg->angular.z;
  }
private:
  std::string GetSDFElement(sdf::ElementPtr sdf, const std::string &tag, const std::string &def)
  {
    return sdf->HasElement(tag) ? sdf->Get<std::string>(tag) : def;
  }
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  std::string cmd_vel_topic_;
  std::string front_right_joint_name_, back_right_joint_name_;
  std::string front_left_joint_name_,  back_left_joint_name_;
  physics::JointPtr front_right_joint_;
  physics::JointPtr back_right_joint_;
  physics::JointPtr front_left_joint_;
  physics::JointPtr back_left_joint_;
  double wheel_radius_;
  double wheel_separation_width_;
  double wheel_separation_length_;
  double vx_ = 0.0, vy_ = 0.0, wz_ = 0.0;
  ros::Subscriber cmd_vel_sub_;
  event::ConnectionPtr update_connection_;
};
GZ_REGISTER_MODEL_PLUGIN(MecanumDrivePlugin)
} 
