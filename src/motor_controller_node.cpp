#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "ethercat.h"
#include <sstream>

#include "motor_drive.hpp"

using namespace raisin;
using namespace robot;

float motor_speed;
int32_t motor_target_speed = 0;
float64 motor_actual_speed;
int16_t motor_id = 0;
int16_t target_command;
MotorDrive<Elmo> drive_;

struct Gain
{
float64 p_gain = 1;
float64 i_gain = 1;
float64 d_gain = 1;
};

struct Gain gain;

void update_motor_speed(const sensor_msgs::Joy::ConstPtr& msg)
{
  motor_speed = msg->axes[1];
  motor_target_speed = int32_t(motor_speed*3500);
  // //motor_target_speed
  // motor_actual_speed = drive_.getMotorVelocity(motor_id);
  // //change the encoder/cnt to rpm
  // motor_actual_speed = (motor_actual_speed/262144)*60;
  
  // //implement the PD controller

  // target_command = (1000-motor_actual_speed)*p_gain;

  // if(target_command > 100)
  // {
  //   target_command = 100;
  // }
  // //end of the implemention of the PD controller
  // //drive_.setTargetVelocity(motor_id,motor_target_speed);
  // std::cout << target_command << std::endl;
  // drive_.setDesiredMotorTorque(motor_id,target_command);
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("joy", 1000, update_motor_speed);
  ros::Rate loop_rate(2000);

  std::string ifname = "eno1";
  
  drive_.conn(ifname);
  drive_.start();

  int count = 0;

  while (ros::ok())
  {
    //ROS_INFO("I heard: [%d]", motor_speed_int);
    drive_.cycle();
    ros::spinOnce();
    loop_rate.sleep();
    ++count;

    motor_actual_speed = drive_.getMotorVelocity(motor_id);
    motor_actual_speed = (motor_actual_speed/262144)*60;
    

    //implement the P controller
    target_command = (motor_target_speed-motor_actual_speed)*gain.p_gain;
    /*
    Threahold setting
    The target_command could be between 0 - 1000
    200 means 20%, 100% used the maxium A setting by the EAS. Now It's 30A
    */

    if(target_command > 500)
    {
      target_command = 500;
    }
    if(target_command < -500)
    {
      target_command = -500;
    }

    //end of the implemention of the Pcontroller

    //std::cout << target_command << std::endl;

    drive_.setDesiredMotorTorque(motor_id,target_command);

  }
  return 0;
}
