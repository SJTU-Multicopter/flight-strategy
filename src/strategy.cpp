#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "Eigen/Dense"
#include "ardrone_autonomy/Navdata.h"
#include "ardrone_control/ROI.h"
#include <flight_strategy/ctrl.h>
#include <flight_strategy/ctrlBack.h>
#include "image_process/robot_info.h"
#include "image_process/drone_info.h"
#define LOOP_RATE 20

#define STATE_IDLE 0
#define STATE_TAKEOFF 1
#define STATE_ALT_CHANGE 2
#define STATE_LOCATING 3
#define STATE_STANDBY 4
#define STATE_FLYING_TO_CATCH 5
#define STATE_REVOLVING 6
#define STATE_FLYING_AWAY 7
#define STATE_ON_GROUND 8


using namespace std;
using namespace Eigen;
float absolute_f(float a){return (a>0?a:-a);}
class position
{
public:
	Matrix<float, 3, 3> R_field_body;
	Matrix<float, 3, 3> R_field_world;
	Vector3f pos_b;
	Vector3f pos_f;
	Vector3f vel_b;
	Vector3f vel_f;
	Vector2f pos_img;
	float yaw;
	void get_R_field_body(float yaw);
	bool self_located(void);
};
void position::get_R_field_body(float yaw)
{
	R_field_body(0,0) = cos(yaw);
	R_field_body(0,1) = sin(-yaw);
	R_field_body(0,2) = 0;
	R_field_body(1,0) = sin(yaw);
	R_field_body(1,1) = cos(yaw);
	R_field_body(1,2) = 0;
	R_field_body(2,0) = 0;
	R_field_body(2,1) = 0;
	R_field_body(2,2) = 1;
}
bool position::self_located(void)
{
	if (absolute_f(pos_img(0) - 320)< 300){
		if(absolute_f(pos_img(1) - 180)<160)
			return true;
	}
	return false;
}
struct control
{
	bool flag_arrived;
	int mode;
	bool pos_halt[3];
	float pos_sp[3];
	bool enabled;
};

struct flight
{
	int state;
	int last_state;
	int drone_state;
};
struct robot
{
    float pos_f[2];
	float yaw;
};
position pos;
struct control ctrl={false, 0, {false,false,false},{0,0,0},false};
struct flight flight={0,0,0};
struct robot robot={{0,0},0};
ros::Publisher rawpos_b_pub;
ros::Publisher rawpos_f_pub;
void navCallback(const ardrone_autonomy::Navdata &msg)
{
	geometry_msgs::PoseStamped rawpos_b;
	geometry_msgs::PoseStamped rawpos_f;
	Vector3f raw_v ;

	static float last_time = 0;
	raw_v(0) = msg.vx;
	raw_v(1) = msg.vy;
	raw_v(2) = msg.vz;

	float dt = (msg.tm - last_time)/1000000.0;
	last_time = msg.tm;
	pos.pos_b += raw_v * dt / 1000.0;

	pos.yaw = (msg.rotZ) / 57.3;
	pos.get_R_field_body(pos.yaw);
//	first_yaw_received = true;

	pos.pos_f = pos.R_field_body * pos.pos_b;

	rawpos_b.pose.position.x = pos.pos_b(0);
	rawpos_b.pose.position.y = pos.pos_b(1);
	rawpos_b.pose.position.z = pos.pos_b(2);
	rawpos_f.pose.position.x = pos.pos_f(0);
	rawpos_f.pose.position.y = pos.pos_f(1);
	rawpos_f.pose.position.z = pos.pos_f(2);

	rawpos_b_pub.publish(rawpos_b);
	rawpos_f_pub.publish(rawpos_f);

	// 0: Unknown
	// 1: Inited
	// 2: Landed
	// 3,7: Flying
	// 4: Hovering
	// 5: Test (?)
	// 6: Taking off
	// 8: Landing
	// 9: Looping (?)
	flight.drone_state = msg.state;
}
void odometryCallback(const nav_msgs::Odometry &msg)
{
	pos.pos_f(2) = msg.pose.pose.position.z;
}
void drone_info_Callback(const image_process::drone_info msg)
{
    pos.pos_img(0)=msg.pose.x;
    pos.pos_img(1)=msg.pose.y;
    pos.yaw=msg.pose.theta;
}
void robot_info_Callback(const image_process::robot_info msg)
{
    robot.pos_f[0]=msg.pose.x;
    robot.pos_f[1]=msg.pose.y;
    robot.yaw=msg.pose.theta;
}
void ctrlBack_Callback(const flight_strategy::ctrlBack msg)
{
	if(msg.arrived[0]&&msg.arrived[1]&&msg.arrived[2])
		ctrl.flag_arrived = true;
	else
		ctrl.flag_arrived = false;
}
int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "pingpong");
	ros::NodeHandle n;
//	ros::Subscriber image_pos_sub = n.subscribe("/ROI", 1, imagepositionCallback);
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
	ros::Publisher land_pub = n.advertise<std_msgs::Empty>("/ardrone/land", 1);
	ros::Publisher stop_pub = n.advertise<std_msgs::Empty>("/ardrone/reset", 1);
	rawpos_b_pub = n.advertise<geometry_msgs::PoseStamped>("/ardrone/rawpos_b", 1);
	rawpos_f_pub = n.advertise<geometry_msgs::PoseStamped>("/ardrone/rawpos_f", 1);
	ros::Publisher ctrl_pub = n.advertise<flight_strategy::ctrl>("ctrl", 1);
	ros::Subscriber ctrlBack_sub = n.subscribe("ctrlBack", 1, ctrlBack_Callback);
	ros::Subscriber robot_info_sub = n.subscribe("/ardrone/robot_info", 1, robot_info_Callback);
	ros::Subscriber drone_info_sub = n.subscribe("/ardrone/position_reset_info", 1, drone_info_Callback);
	ros::Subscriber nav_sub = n.subscribe("/ardrone/navdata", 1, navCallback);

	ros::Rate loop_rate(LOOP_RATE);
	std_msgs::Empty order;
	geometry_msgs::Twist cmd;
	while(ros::ok())
	{
		switch(flight.state){
			case STATE_IDLE:{
				if(flight.last_state != flight.state){
					ROS_INFO("State to IDLE\n");
				}
				flight.last_state = flight.state;
				flight.state = STATE_TAKEOFF;
				break;
			}
			case STATE_TAKEOFF:{
				if(flight.last_state != flight.state){
					ROS_INFO("State TAKEOFF\n");
					takeoff_pub.publish(order);
					// flight_strategy::ctrl ctrl_msg;
					// ctrl_msg.pos_sp[0] = pos.pos_f(0);
					// ctrl_msg.pos_halt[0] = 1;
					// ctrl_msg.pos_halt[1] = 1;
					// ctrl_msg.pos_halt[2] = 1;
					// ctrl_msg.enable = 1;
					// ctrl_pub.publish(ctrl_msg);
				}
				flight.last_state = flight.state;
				if(flight.drone_state != 6){
					//fly to center
					flight.state = STATE_LOCATING;
				}
				break;
			}
			case STATE_LOCATING:{
				static unsigned int loop_times = 0;
				flight_strategy::ctrl ctrl_msg;
				if(flight.last_state != flight.state){
					ROS_INFO("State LOCATING\n");
					
					ctrl_msg.pos_sp[2] = pos.pos_f(2)+0.5;
					ctrl_msg.pos_halt[0] = 1;
					ctrl_msg.pos_halt[1] = 1;
					ctrl_msg.pos_halt[2] = 0;
					ctrl_msg.enable = 1;
					ctrl_msg.mode = 0;
					ctrl_pub.publish(ctrl_msg);
				}
				flight.last_state = flight.state;
				loop_times++;
				if(pos.self_located()){
					ctrl.flag_arrived = false;
					loop_times = 0;
					flight.state = STATE_STANDBY;
				}
				else if(loop_times > 100){
					ctrl_msg.pos_sp[2] = pos.pos_f(2)+0.5;
					ctrl_msg.pos_halt[0] = 1;
					ctrl_msg.pos_halt[1] = 1;
					ctrl_msg.pos_halt[2] = 0;
					ctrl_msg.enable = 1;
					ctrl_msg.mode = 0;
					ctrl_pub.publish(ctrl_msg);
					loop_times = 0;
				}
				break;
			}
			case STATE_STANDBY:{
				flight_strategy::ctrl ctrl_msg;
				if(flight.last_state != flight.state){
					ROS_INFO("State STANDBY\n");
					ctrl_msg.mode = 1;
					ctrl_msg.enable = 1;
					ctrl_msg.pos_halt[0] = 0;
					ctrl_msg.pos_halt[1] = 0;
					ctrl_msg.pos_halt[2] = 0;
					ctrl_pub.publish(ctrl_msg);
				}
				flight.last_state = flight.state;
				if(1){//locating_timeout()){
					
				//	flight.state = STATE_LOCATING;
				}
				else if(0){//robot_coming()){
					flight.state = STATE_FLYING_TO_CATCH;
				}
				break;
			}
			case STATE_FLYING_TO_CATCH:{
				if(flight.last_state != flight.state){
					ROS_INFO("State FLY_TO_CATCH\n");
				}
				flight.last_state = flight.state;
				if(1){//robot_captured()){
					//alt set to a suitable point
					flight.state = STATE_REVOLVING;
				}
				break;
			}
			case STATE_REVOLVING:{
				if(flight.last_state != flight.state){
					ROS_INFO("State REVOLVING\n");
				}
				flight.last_state = flight.state;
				if(1){//robot_at_desired_angle()){
					flight.state = STATE_FLYING_AWAY;
				}
				break;
			}
			case STATE_FLYING_AWAY:{
				if(flight.last_state != flight.state){
					ROS_INFO("State FLYING_AWAY\n");
				}
				flight.last_state = flight.state;
				if(1){//close_to_center()){
					//fly across center to a further pos
				}
				else{
					//fly to center
				}
				if(ctrl.flag_arrived){

				}
				break;
			}
			case STATE_ON_GROUND:{
				if(flight.last_state != flight.state){
					ROS_INFO("State ON_GROUND\n");
				}
				flight.last_state = flight.state;
				break;
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
