/*
 * set_multiple_position
 * Static positions for the robotic arm
 * Author :: Rae Jeong
 * Email  :: raychanjeongjeong@gmail.com
 */
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <ros/console.h>


/*
 * DEFINES
 */
#define NUMBER_OF_POSITIONS 10

/*
 *  Different positions
 */

#define POSITION_1_JOINT_1 475.0
#define POSITION_1_JOINT_2 536.0
#define POSITION_1_JOINT_3 500.0

#define POSITION_2_JOINT_1 453.0
#define POSITION_2_JOINT_2 695.0
#define POSITION_2_JOINT_3 428.0

#define POSITION_3_JOINT_1 367.0
#define POSITION_3_JOINT_2 496.0
#define POSITION_3_JOINT_3 292.0

#define POSITION_4_JOINT_1 480.0
#define POSITION_4_JOINT_2 700.0
#define POSITION_4_JOINT_3 573.0

#define POSITION_5_JOINT_1 480.0
#define POSITION_5_JOINT_2 500.0
#define POSITION_5_JOINT_3 684.0

#define POSITION_6_JOINT_1 480.0
#define POSITION_6_JOINT_2 550.0
#define POSITION_6_JOINT_3 634.0

#define POSITION_7_JOINT_1 480.0
#define POSITION_7_JOINT_2 432.0
#define POSITION_7_JOINT_3 703.0

#define POSITION_8_JOINT_1 480.0
#define POSITION_8_JOINT_2 624.0
#define POSITION_8_JOINT_3 543.0

#define POSITION_9_JOINT_1 480.0
#define POSITION_9_JOINT_2 457.0
#define POSITION_9_JOINT_3 257.0

#define POSITION_10_JOINT_1 480.0
#define POSITION_10_JOINT_2 500.0
#define POSITION_10_JOINT_3 380.0

#define JOINT_1_MAX 800
#define JOINT_1_MIN 200

#define JOINT_2_MAX 800
#define JOINT_2_MIN 200

#define JOINT_3_MAX 800
#define JOINT_3_MIN 200

#define NULL_POSITION (-1)

float joint_1_positions[NUMBER_OF_POSITIONS] = {POSITION_1_JOINT_1, 
                                                POSITION_2_JOINT_1,
                                                POSITION_3_JOINT_1,
                                                POSITION_4_JOINT_1,
                                                POSITION_5_JOINT_1,
                                                POSITION_6_JOINT_1,
                                                POSITION_7_JOINT_1,
                                                POSITION_8_JOINT_1,
                                                POSITION_9_JOINT_1,
                                                POSITION_10_JOINT_1 };


float joint_2_positions[NUMBER_OF_POSITIONS] = {POSITION_1_JOINT_2, 
                                                POSITION_2_JOINT_2,
                                                POSITION_3_JOINT_2,
                                                POSITION_4_JOINT_2,
                                                POSITION_5_JOINT_2,
                                                POSITION_6_JOINT_2,
                                                POSITION_7_JOINT_2,
                                                POSITION_8_JOINT_2,
                                                POSITION_9_JOINT_2,
                                                POSITION_10_JOINT_2 };

float joint_3_positions[NUMBER_OF_POSITIONS] = {POSITION_1_JOINT_3, 
                                                POSITION_2_JOINT_3,
                                                POSITION_3_JOINT_3,
                                                POSITION_4_JOINT_3,
                                                POSITION_5_JOINT_3,
                                                POSITION_6_JOINT_3,
                                                POSITION_7_JOINT_3,
                                                POSITION_8_JOINT_3,
                                                POSITION_9_JOINT_3,
                                                POSITION_10_JOINT_3 };

void arm_position_index_Cb(const std_msgs::Int32& arm_position_index_msg);
bool is_valid_position(int joint_number, float joint_position_to_be_checked);

std_msgs::Float32 joint_1_setpoint_msg;
std_msgs::Float32 joint_2_setpoint_msg;
std_msgs::Float32 joint_3_setpoint_msg;

float joint_1_setpoint = NULL_POSITION;
float joint_2_setpoint = NULL_POSITION;
float joint_3_setpoint = NULL_POSITION;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_multiple_position");
  ros::NodeHandle nh;

  ros::Subscriber joy_subscriber = nh.subscribe("arm_position_index", 1000, arm_position_index_Cb);

  ros::Publisher joint_1_publisher = nh.advertise<std_msgs::Float32>(
    "joint_1_set_setpoint", 1000);
  ros::Publisher joint_2_publisher = nh.advertise<std_msgs::Float32>(
    "joint_2_set_setpoint", 1000);
  ros::Publisher joint_3_publisher = nh.advertise<std_msgs::Float32>(
    "joint_3_set_setpoint", 1000);


  ros::Rate loop_rate(10);
  
  while(ros::ok())
  {
    if(is_valid_position(1, joint_1_setpoint))
    {
      joint_1_setpoint_msg.data = joint_1_setpoint;
      joint_1_publisher.publish(joint_1_setpoint_msg);
    }

    if(is_valid_position(2, joint_2_setpoint))
    {
      joint_2_setpoint_msg.data = joint_2_setpoint;
      joint_2_publisher.publish(joint_2_setpoint_msg);
    }

    if(is_valid_position(3, joint_3_setpoint))
    {
      joint_3_setpoint_msg.data = joint_3_setpoint;
      joint_3_publisher.publish(joint_3_setpoint_msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

void arm_position_index_Cb(const std_msgs::Int32& arm_position_index_msg)
{
  joint_1_setpoint = joint_1_positions[arm_position_index_msg.data-1];
  joint_2_setpoint = joint_2_positions[arm_position_index_msg.data-1];
  joint_3_setpoint = joint_3_positions[arm_position_index_msg.data-1];
}

bool is_valid_position(int joint_number, float joint_position_to_be_checked)
{
  switch(joint_number)
  {
    case 1:
      return (joint_position_to_be_checked > JOINT_1_MIN && joint_position_to_be_checked < JOINT_1_MAX && joint_position_to_be_checked != NULL_POSITION);
      break;

    case 2:
      return (joint_position_to_be_checked > JOINT_2_MIN && joint_position_to_be_checked < JOINT_2_MAX && joint_position_to_be_checked != NULL_POSITION);
      break;

    case 3:
      return (joint_position_to_be_checked > JOINT_3_MIN && joint_position_to_be_checked < JOINT_3_MAX && joint_position_to_be_checked != NULL_POSITION);
      break;
  }

  return 0;
}