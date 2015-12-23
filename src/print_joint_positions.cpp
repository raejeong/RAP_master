/*
 * RAP teleop Joy
 * Teleoperation to the Robotic Arm with a joystick
 * Author :: Rae Jeong
 * Email  :: raychanjeongjeong@gmail.com
 */
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <ros/console.h>

float joint_1_position = 0;
float joint_2_position = 0;
float joint_3_position = 0;

void joint_1_position_Cb(const std_msgs::Float32& joint_1_position_msg);
void joint_2_position_Cb(const std_msgs::Float32& joint_2_position_msg);
void joint_3_position_Cb(const std_msgs::Float32& joint_3_position_msg);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "RAP_teleop_joy");
  ros::NodeHandle nh;

  ros::Subscriber joint_1_subscriber = nh.subscribe("joint_1_position", 1000, joint_1_position_Cb);
  ros::Subscriber joint_2_subscriber = nh.subscribe("joint_2_position", 1000, joint_2_position_Cb);
  ros::Subscriber joint_3_subscriber = nh.subscribe("joint_3_position", 1000, joint_3_position_Cb);

  ros::Rate loop_rate(10);
  
  while(ros::ok())
  {
    ROS_INFO("Joint 1: %f  Joint 2: %f  Joint 3: %f", joint_1_position, joint_2_position, joint_3_position);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void joint_1_position_Cb(const std_msgs::Float32& joint_1_position_msg)
{
  joint_1_position = joint_1_position_msg.data;
}

void joint_2_position_Cb(const std_msgs::Float32& joint_2_position_msg)
{
  joint_2_position = joint_2_position_msg.data;
}


void joint_3_position_Cb(const std_msgs::Float32& joint_3_position_msg)
{
  joint_3_position = joint_3_position_msg.data;
}

