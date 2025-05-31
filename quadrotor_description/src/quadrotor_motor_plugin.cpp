#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class QuadrotorMotorPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override
    {
      // Store the pointer to the model
      this->model = _parent;
      
      // Initialize ROS if it hasn't been initialized already
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }
      
      // Create ROS node handle
      this->rosNode.reset(new ros::NodeHandle(""));
      
      // Subscribe to motor speed commands
      this->motorSub = this->rosNode->subscribe(
        "/quadrotor/motor_speeds", 10, &QuadrotorMotorPlugin::OnMotorSpeeds, this);
      
      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&QuadrotorMotorPlugin::OnUpdate, this));
        
      // Get the base link
      this->baseLink = this->model->GetLink("base_link");
      if (!this->baseLink)
      {
        ROS_ERROR("Could not find base_link");
        return;
      }
      
      // Motor parameters (you can adjust these)
      this->thrustCoeff = 5.84e-06;  // Thrust coefficient
      this->momentCoeff = 0.06;      // Moment coefficient  
      this->armLength = 0.25;        // Distance from center to motor
      this->maxMotorSpeed = 1500.0;  // Maximum motor speed (rad/s)
      
      // Initialize motor speeds to zero
      for (int i = 0; i < 4; i++)
        this->motorSpeeds[i] = 0.0;
      
      ROS_INFO("Quadrotor motor plugin loaded successfully");
    }
    
    private: void OnMotorSpeeds(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
      if (msg->data.size() >= 4)
      {
        for (int i = 0; i < 4; i++)
        {
          // Clamp motor speeds to reasonable values
          this->motorSpeeds[i] = std::max(0.0, std::min(msg->data[i], this->maxMotorSpeed));
        }
      }
      else
      {
        ROS_WARN_THROTTLE(1.0, "Motor speed message should contain 4 values, got %lu", msg->data.size());
      }
    }
    
    private: void OnUpdate()
    {
      // Convert motor speeds to thrust forces
      double thrust[4];
      for (int i = 0; i < 4; i++)
      {
        // Thrust = coefficient * omega^2
        thrust[i] = this->thrustCoeff * this->motorSpeeds[i] * this->motorSpeeds[i];
      }
      
      // Calculate total thrust (vertical force)
      double totalThrust = thrust[0] + thrust[1] + thrust[2] + thrust[3];
      
      // Apply vertical thrust force
      ignition::math::Vector3d forceVector(0, 0, totalThrust);
      this->baseLink->AddForce(forceVector);
      
      // Calculate torques based on quadrotor configuration
      // Standard X configuration:
      // Motor 0: Front Right (+x, -y) - CW
      // Motor 1: Front Left  (+x, +y) - CCW  
      // Motor 2: Back Right  (-x, -y) - CW
      // Motor 3: Back Left   (-x, +y) - CCW
      
      double sqrt2 = sqrt(2.0);
      
      // Roll torque (rotation around x-axis)
      double rollTorque = (this->armLength / sqrt2) * (thrust[0] - thrust[1] + thrust[2] - thrust[3]);
      
      // Pitch torque (rotation around y-axis) 
      double pitchTorque = (this->armLength / sqrt2) * (thrust[0] + thrust[1] - thrust[2] - thrust[3]);
      
      // Yaw torque (rotation around z-axis) - from motor reaction torques
      // CW motors (0,2) produce negative yaw, CCW motors (1,3) produce positive yaw
      double yawTorque = this->momentCoeff * (-thrust[0] + thrust[1] - thrust[2] + thrust[3]);
      
      // Apply torques
      ignition::math::Vector3d torqueVector(rollTorque, pitchTorque, yawTorque);
      this->baseLink->AddTorque(torqueVector);
    }
    
    // Pointer to the model
    private: physics::ModelPtr model;
    
    // Pointer to the base link
    private: physics::LinkPtr baseLink;
    
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // ROS node handle
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    
    // ROS subscriber
    private: ros::Subscriber motorSub;
    
    // Motor speeds array [front_right, front_left, back_right, back_left]
    private: double motorSpeeds[4];
    
    // Motor parameters
    private: double thrustCoeff;
    private: double momentCoeff; 
    private: double armLength;
    private: double maxMotorSpeed;
  };
  
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(QuadrotorMotorPlugin)
}