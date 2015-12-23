/*
 * position_loop
 * Static position loop
 * Author :: Rae Jeong
 * Email  :: raychanjeongjeong@gmail.com
 */
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <ros/console.h>

#define ARRAY_SIZE 3

int arm_index[ARRAY_SIZE] = {1,2,3};

std_msgs::Int32 position_msg;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_loop");
  ros::NodeHandle nh;

  ros::Publisher position_publisher = nh.advertise<std_msgs::Int32>(
    "arm_position_index", 1000);

  ros::Rate loop_rate(10);
  
  while(ros::ok())
  {
  	for(int i = 0; i < ARRAY_SIZE; i++)
  	{
  		position_msg.data = arm_index[i];
	    position_publisher.publish(position_msg);
	    ros::Duration(5).sleep(); // sleep for 30 second
  	}
    
    ros::spinOnce();
    loop_rate.sleep();
  }
}