#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt16.h>

// create the TeleopTPRobot class and define the joyCallback function that will take a joy msg
class TeleopTPRobot
{
public:
  TeleopTPRobot();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  // variables
	// vel_pub_
  int axis_move_linear_x, axis_move_linear_y, axis_move_angular_l, axis_move_angular_r;  //joystick axises number
  double scale_move_linear_x, scale_move_linear_y, scale_move_angular;  //scale values of joystick axises
    // arduino_servo_cam
  int axis_view_angular_x, axis_view_angular_y;  //joystick axises number
  int scale_view_angular_x, scale_view_angular_y;  //scale values of joystick axises
    // arduino_actuator
  int axis_actuarot_linear;  //joystick axises number
  
  // ros handle/subscriber/publisher	
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Publisher vel_pub_;
  ros::Publisher arduino_actuator;
  ros::Publisher arduino_servo_cam;

};


TeleopTPRobot::TeleopTPRobot(): axis_move_linear_x(1), axis_move_linear_y(0), axis_move_angular_l(2), axis_move_angular_r(5), axis_view_angular_x(3), axis_view_angular_y(4), axis_actuarot_linear(7)
{
  // initialize parameters from launch file
    // joystick chanel number
      // robot motion
  nh_.param("axis_move_linear_x", axis_move_linear_x, axis_move_linear_x);  
  nh_.param("axis_move_linear_y", axis_move_linear_y, axis_move_linear_y);  
  nh_.param("axis_move_angular_l", axis_move_angular_l, axis_move_angular_l);
  nh_.param("axis_move_angular_r", axis_move_angular_r, axis_move_angular_r);
      // camera motion
  nh_.param("axis_view_angular_x", axis_view_angular_x, axis_view_angular_x);
  nh_.param("axis_view_angular_y", axis_view_angular_y, axis_view_angular_y);
      // actuator motion
  nh_.param("axis_actuarot_linear", axis_actuarot_linear, axis_actuarot_linear);

    // scale-to enlarge the value from joystick
      // robot motion
  nh_.param("scale_move_linear_x", scale_move_linear_x, scale_move_linear_x);
  nh_.param("scale_move_linear_y", scale_move_linear_y, scale_move_linear_y);
  nh_.param("scale_move_angular", scale_move_angular, scale_move_angular);
      // camera motion
  nh_.param("scale_view_angular_x", scale_view_angular_x, scale_view_angular_x);
  nh_.param("scale_view_angular_x", scale_view_angular_x, scale_view_angular_x);

  // subscribe to the joystick topic for the input to drive the tprobot
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTPRobot::joyCallback, this);
  
  // create a publisher that will advertise on the command_velocity topic
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  arduino_actuator = nh_.advertise<std_msgs::UInt16>("arduino_actuator", 1);
  arduino_servo_cam = nh_.advertise<std_msgs::UInt16>("arduino_servo", 1);

}


void TeleopTPRobot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  //move tprobot-1 | msg_type:geometry_msgs::Twist
  geometry_msgs::Twist twist;
  twist.linear.x = scale_move_linear_x*joy->axes[axis_move_linear_x];
  twist.linear.y = scale_move_linear_y*joy->axes[axis_move_linear_y];
  twist.angular.z = scale_move_angular*(joy->axes[axis_move_angular_r]-joy->axes[axis_move_angular_l])/2;
  vel_pub_.publish(twist); 
  
  //move tprobot-2 | msg_type:std_msgs::UInt16 UInt16 UInt16 UInt16
  
  
  //rotate servo_cam on tprobot | msg_type:std_msgs::UInt16
  std_msgs::UInt16 arduino_servo_cam_degree;
  arduino_servo_cam_degree.data = scale_view_angular_x*(joy->axes[axis_view_angular_x]+2);
  arduino_servo_cam.publish(arduino_servo_cam_degree);
  
  //extend/shrink the actuator of tprobot | msg_type:std_msgs::UInt16
  //0:stop; 1:up; 2:down
  std_msgs::UInt16 arduino_actuator_status;
  if(joy->axes[axis_actuarot_linear] == 0 || joy->axes[axis_actuarot_linear] == 1){
	  arduino_actuator_status.data = joy->axes[axis_actuarot_linear];
	  }
  else{
	  arduino_actuator_status.data = 2;
	  }
  arduino_actuator.publish(arduino_actuator_status);

}


int main(int argc, char** argv)
{
  // initialize our ROS node, create a teleop_tprobot, and spin our node until Ctrl-C is pressed
  ros::init(argc, argv, "teleop_tprobot");
  TeleopTPRobot teleop_tprobot;

  ros::spin();
}
