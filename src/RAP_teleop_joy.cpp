/*
 * RAP teleop Joy
 * Teleoperation to the Robotic Arm with a joystick
 * Author :: Rae Jeong
 * Email  :: raychanjeongjeong@gmail.com
 */
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>

void joyCb(const sensor_msgs::Joy::ConstPtr& joy_msg);

float joint_1_setpoint = 0;
float joint_1_range = 180;

float joint_2_setpoint = 0;
float joint_2_range = 90;

float joint_3_setpoint = 0;
float joint_3_range = 180;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "RAP_teleop_joy");
  ros::NodeHandle nh;

  ros::Subscriber joy_subscriber = nh.subscribe("joy", 1000, joyCb);
 
  ros::Publisher joint_1_publisher = nh.advertise<std_msgs::Float32>(
    "joint_1_set_setpoint", 1000);
  ros::Publisher joint_2_publisher = nh.advertise<std_msgs::Float32>(
    "joint_2_set_setpoint", 1000);
  ros::Publisher joint_3_publisher = nh.advertise<std_msgs::Float32>(
    "joint_3_set_setpoint", 1000);

  ros::Rate loop_rate(10);
  
  while(ros::ok())
  {
    std_msgs::Float32 joint_1_setpoint_msg;
    joint_1_setpoint_msg.data = joint_1_setpoint;
    joint_1_publisher.publish(joint_1_setpoint_msg);

    std_msgs::Float32 joint_2_setpoint_msg;
    joint_2_setpoint_msg.data = joint_2_setpoint;
    joint_2_publisher.publish(joint_2_setpoint_msg);

    std_msgs::Float32 joint_3_setpoint_msg;
    joint_3_setpoint_msg.data = joint_3_setpoint;
    joint_3_publisher.publish(joint_3_setpoint_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

void joyCb(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  joint_1_setpoint = joint_1_range*joy_msg->axes[3];
  joint_2_setpoint = joint_2_range*joy_msg->axes[4];
  joint_3_setpoint = joint_3_range*joy_msg->axes[0];
}
