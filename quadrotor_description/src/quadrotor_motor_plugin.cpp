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
      ROS_INFO("Motor configuration: [FR, FL, BL, BR] = [data[0], data[1], data[2], data[3]]");
      ROS_INFO("Rotation directions: FR(CW), FL(CCW), BL(CW), BR(CCW)");
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
      
      // Motor configuration: [FR, FL, BL, BR] = [data[0], data[1], data[2], data[3]]
      // Motor positions in X configuration:
      // FR (data[0]): Front Right  (+x, -y) - CW rotation
      // FL (data[1]): Front Left   (+x, +y) - CCW rotation  
      // BL (data[2]): Back Left    (-x, +y) - CW rotation
      // BR (data[3]): Back Right   (-x, -y) - CCW rotation
      
      double thrustFR = thrust[0];  // Front Right
      double thrustFL = thrust[1];  // Front Left
      double thrustBL = thrust[2];  // Back Left
      double thrustBR = thrust[3];  // Back Right
      
      // Calculate total thrust (vertical force)
      double totalThrust = thrustFR + thrustFL + thrustBL + thrustBR;
      
      // Apply vertical thrust force
      ignition::math::Vector3d forceVector(0, 0, totalThrust);
      this->baseLink->AddForce(forceVector);
      
      double sqrt2 = sqrt(2.0);

      double pitchTorque = (this->armLength / sqrt2) * ((thrustFR + thrustFL) - (thrustBL + thrustBR));
      
      double rollTorque = (this->armLength / sqrt2) * ((thrustFL + thrustBL) - (thrustFR + thrustBR));
      
      double yawTorque = this->momentCoeff * ((thrustFR - thrustFL + thrustBL - thrustBR));
      
      // Apply torques (order: pitch, roll, yaw for x, y, z axes)
      ignition::math::Vector3d torqueVector(pitchTorque, rollTorque, yawTorque);
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
    
    // Motor speeds array [front_right, front_left, back_left, back_right]
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