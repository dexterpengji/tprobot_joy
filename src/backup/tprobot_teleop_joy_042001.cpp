#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

// create the TeleopTPRobot class and define the joyCallback function that will take a joy msg
class TeleopTPRobot
{
public:
  TeleopTPRobot();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_x, linear_y, angular_l, angular_r;   // used to define which axes of the joystick will control our tprobot
  double l_scale_x, l_scale_y, a_scale_;
  
  ros::Publisher vel_pub_;
  ros::Publisher arduino_servo;
  
  ros::Subscriber joy_sub_;

};


TeleopTPRobot::TeleopTPRobot(): linear_x(1), linear_y(0), angular_l(2), angular_r(5)
{
  //  initialize some parameters
  nh_.param("axis_move_linear_x", linear_x, linear_x);  
  nh_.param("axis_move_linear_y", linear_y, linear_y);  
  nh_.param("axis_move_angular_l", angular_l, angular_l);
  nh_.param("axis_move_angular_r", angular_r, angular_r);
  
  nh_.param("scale_move_linear_x", l_scale_x, l_scale_x);
  nh_.param("scale_move_linear_y", l_scale_y, l_scale_y);
  nh_.param("scale_move_angular", a_scale_, a_scale_);

  // subscribe to the joystick topic for the input to drive the tprobot
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTPRobot::joyCallback, this);
  
  // create a publisher that will advertise on the command_velocity topic
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
  // create a publisher that will advertise on the command_velocity topic
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}


void TeleopTPRobot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;

  // take the data from the joystick and manipulate it by scaling it and using independent axes to control the linear and angular velocities of the tprobot
  twist.linear.x = l_scale_x*joy->axes[linear_x];
  twist.linear.y = l_scale_y*joy->axes[linear_y];
  twist.angular.z = a_scale_*(joy->axes[angular_r]-joy->axes[angular_l])/2;

  vel_pub_.publish(twist); 
}


int main(int argc, char** argv)
{
  // initialize our ROS node, create a teleop_tprobot, and spin our node until Ctrl-C is pressed
  ros::init(argc, argv, "teleop_tprobot");
  TeleopTPRobot teleop_tprobot;

  ros::spin();
}
