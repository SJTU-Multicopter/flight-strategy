#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include "Eigen/Dense"
#include "ardrone_autonomy/Navdata.h"
#include "ardrone_autonomy/navdata_altitude.h"
#include <flight_strategy/ctrl.h>
#include <flight_strategy/ctrlBack.h>
#include "image_process/robot_info.h"
#include "image_process/drone_info.h"
#include "mavros/Ardrone.h"
#define LOOP_RATE 20
#define IMG_TOL 25

#define STATE_IDLE 0
#define STATE_TAKEOFF 1
#define STATE_ALT_CHANGE 2
#define STATE_LOCATING 3
#define STATE_STANDBY 4
#define STATE_FLYING_TO_CATCH 5
#define STATE_REVOLVING 6
#define STATE_FLYING_AWAY 7
#define STATE_ON_GROUND 8

#define NORMAL_CTL 0
#define IMAGE_CTL 1

using namespace std;
using namespace Eigen;
class position
{
public:
	Matrix<float, 3, 3> R_field_body;
	//Matrix<float, 3, 3> R_field_world;
	Vector3f pos_b;
	Vector3f pos_f;
	Vector3f vel_b;
	Vector3f vel_f;
	Vector2f pos_img;	//position of the blue circle
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
	if (fabs(pos_img(0) - 320) < IMG_TOL){
		if(fabs(pos_img(1) - 180) < IMG_TOL)
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
    float pos_b[2];
	float yaw_body;
	float yaw_field;
	bool whole;
};

position pos;
struct control ctrl={false, 0, {false,false,false},{0,0,0},false};
struct flight flight={5,0,0};
struct robot robot={{0,0},{0,0},0,0,false};
float altitude_sp;
ros::Publisher rawpos_b_pub;
ros::Publisher rawpos_f_pub;
ros::Time position_reset_stamp;
ros::Time robot_stamp;
Vector2f pos_update;
Vector2f catch_position;
Vector2f catch_position_last;

bool arrived_inaccurate = false;
bool arrived_height = false;
bool arrived_find = false;
bool moving = false;

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
	pos.get_R_field_body(pos.yaw/57.3);
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
	
}

//altitude of sonar
void altitudeCallback(const ardrone_autonomy::navdata_altitude &msg)
{
	pos.pos_f(2) = msg.altitude_vision/1000.0;
}

//yaw corresponding to the field
void yawCallback(const std_msgs::Float32 &msg)
{
	pos.yaw = msg.data;
}

//circle position
void position_reset_info_Callback(const image_process::drone_info msg)
{
    pos.pos_img(0) = msg.pose.x;
    pos.pos_img(1) = msg.pose.y;
    position_reset_stamp = ros::Time::now();
}

void robot_info_Callback(const image_process::robot_info msg)
{
    robot.pos_b[0] = msg.pose.x;
    robot.pos_b[1] = msg.pose.y;
    robot.yaw_body = msg.pose.theta;
    robot.yaw_field = robot.yaw_body + pos.yaw;
    robot.whole = msg.whole;
    robot_stamp = ros::Time::now();
}

void ctrlBack_Callback(const flight_strategy::ctrlBack msg)
{
	if(msg.arrived[0]&&msg.arrived[1]&&msg.arrived[2])
		ctrl.flag_arrived = true;
	else
		ctrl.flag_arrived = false;
}

// void catch_pos_Callback(const xxx::xxx msg)
// {
	
// }

void robot_pos_Callback(const mavros::Ardrone msg)
{
	robot.pos_f[0] = msg.robot_x;
	robot.pos_f[1] = msg.robot_y;
	catch_position_last(0) = catch_position(0);
	catch_position_last(1) = catch_position(1);
	catch_position(0) = msg.catch_x;
	catch_position(1) = msg.catch_y;
	if(fabs(catch_position_last(0) - catch_position(0)) < 0.0001 && fabs(catch_position_last(1) - catch_position(1)) < 0.0001)
	{
		moving = false;
	}else{
		moving = true;
	}
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "pingpong");
	ros::NodeHandle n;
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
	ros::Publisher land_pub = n.advertise<std_msgs::Empty>("/ardrone/land", 1);
	ros::Publisher stop_pub = n.advertise<std_msgs::Empty>("/ardrone/reset", 1);
	rawpos_b_pub = n.advertise<geometry_msgs::PoseStamped>("/ardrone/rawpos_b", 1);
	rawpos_f_pub = n.advertise<geometry_msgs::PoseStamped>("/ardrone/rawpos_f", 1);
	ros::Publisher ctrl_pub = n.advertise<flight_strategy::ctrl>("ctrl", 1);
	ros::Subscriber ctrlBack_sub = n.subscribe("ctrlBack", 1, ctrlBack_Callback);
	ros::Subscriber robot_info_sub = n.subscribe("/ardrone/robot_info", 1, robot_info_Callback);
	ros::Subscriber position_reset_info_sub = n.subscribe("/ardrone/position_reset_info", 1, position_reset_info_Callback);
	ros::Subscriber nav_sub = n.subscribe("/ardrone/navdata", 1, navCallback);
	ros::Subscriber altitude_sub = n.subscribe("/ardrone/navdata_altitude", 1, altitudeCallback);
	ros::Subscriber yaw_sub = n.subscribe("/ardrone/yaw", 1, yawCallback);
	//ros::Subscriber catch_pos_sub = n.subscribe("", 1, catch_pos_Callback);
	ros::Subscriber robot_pos_sub = n.subscribe("/ardrone/robot_position", 1, robot_pos_Callback);


	ros::Rate loop_rate(LOOP_RATE);
	std_msgs::Empty order;
	//geometry_msgs::Twist cmd;
	pos.pos_f(0) = -1.95;
	pos.pos_f(1) = 0;
	pos.pos_b(0) = -1.95;
	pos.pos_b(1) = 0;

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
				}
				flight.last_state = flight.state;				

				if(!arrived_height)
				{
					ctrl_msg.enable = 1;
					ctrl_msg.mode = NORMAL_CTL;
					ctrl_msg.pos_sp[2] = 2;
					ctrl_msg.pos_halt[0] = 1;
					ctrl_msg.pos_halt[1] = 1;
					ctrl_msg.pos_halt[2] = 0;
					ctrl_pub.publish(ctrl_msg);
				}else if(!arrived_inaccurate){
					ctrl_msg.enable = 1;
					ctrl_msg.mode = NORMAL_CTL;
					ctrl_msg.pos_sp[0] =  -1.95 - pos_update(0);
					ctrl_msg.pos_sp[1] =  0 - pos_update(1);
					ctrl_msg.pos_halt[0] = 0;
					ctrl_msg.pos_halt[1] = 0;
					ctrl_msg.pos_halt[2] = 1;
					ctrl_pub.publish(ctrl_msg);
				}

				if(!arrived_height)
				{
					if(ctrl.flag_arrived){
						arrived_height = true;
						ROS_INFO("HEIGHT:2m");
					} 	
				}
				
				if(!arrived_inaccurate && arrived_height)
				{
					if(ctrl.flag_arrived){
						arrived_inaccurate = true;
						ROS_INFO("POSITION RESET POSITION");
					} 	
				}
				
				if(arrived_inaccurate)
				{
					ros::Duration dur;
					dur = ros::Time::now() - position_reset_stamp;

					if(dur.toSec() > 0.2){
						loop_times++;
						if(loop_times > 100){
							if(pos.pos_f(2) < 2.6){
								ctrl_msg.pos_sp[2] = pos.pos_f(2) + 0.5;
								ctrl_msg.pos_halt[0] = 1;
								ctrl_msg.pos_halt[1] = 1;
								ctrl_msg.pos_halt[2] = 0;
								ctrl_msg.enable = 1;
								ctrl_msg.mode = 0;
								ctrl_pub.publish(ctrl_msg);
								ROS_INFO("HIGHER");
							}
							
							loop_times = 0;
						}

					}else{
						ctrl_msg.enable = 1;
						ctrl_msg.mode = IMAGE_CTL;
						ctrl_msg.pos_sp[0] =  pos.pos_img(0); //blue point position???
						ctrl_msg.pos_sp[1] =  pos.pos_img(1);
						ctrl_msg.pos_halt[0] = 0;
						ctrl_msg.pos_halt[1] = 0;
						ctrl_msg.pos_halt[2] = 1;
						ctrl_pub.publish(ctrl_msg);

						if(pos.self_located()){
							ctrl.flag_arrived = false;
							arrived_inaccurate = false;
							arrived_height = false;
							loop_times = 0;
							flight.state = STATE_STANDBY;
							pos.pos_f(0) = -1.95;
							pos.pos_f(1) = 0;
							pos.pos_b = pos.R_field_body.transpose() * pos.pos_f;
							pos_update(0) = -1.95;
							pos_update(1) = 0;
							break;
						}
					}
				}

				break;
			}
			case STATE_STANDBY:{
				flight_strategy::ctrl ctrl_msg;
				if(flight.last_state != flight.state){
					ROS_INFO("State STANDBY\n");
				}
				flight.last_state = flight.state;

				if(!arrived_height)
				{
					ctrl_msg.enable = 1;
					ctrl_msg.mode = NORMAL_CTL;
					ctrl_msg.pos_sp[2] = 1.5;
					ctrl_msg.pos_halt[0] = 1;
					ctrl_msg.pos_halt[1] = 1;
					ctrl_msg.pos_halt[2] = 0;
					ctrl_pub.publish(ctrl_msg);
				}else{
					if(catch_position(0) < -900){//locating_timeout()){
						ctrl_msg.mode = NORMAL_CTL;
						ctrl_msg.enable = 1;
						ctrl_msg.pos_halt[0] = 1;
						ctrl_msg.pos_halt[1] = 1;
						ctrl_msg.pos_halt[2] = 1;
						ctrl_pub.publish(ctrl_msg);
					}
					else{//robot_coming()){
						flight.state = STATE_FLYING_TO_CATCH;
						arrived_height = false;
						break;
					}
				}
				if(!arrived_height){
					if(ctrl.flag_arrived){
						arrived_height = true;
						ROS_INFO("HEIGHT:1.5m");
					} 	
				}

				break;
			}
			case STATE_FLYING_TO_CATCH:{
				if(flight.last_state != flight.state){
					ROS_INFO("State FLY_TO_CATCH\n");
				}
				flight.last_state = flight.state;
				flight_strategy::ctrl ctrl_msg;
				ros::Duration dur;
				dur = ros::Time::now() - robot_stamp;
				if(dur.toSec() < 0.2){
					flight.state = STATE_REVOLVING;
					arrived_inaccurate = false;
					break;
				}
				if(moving) 
				{
					arrived_inaccurate = false;
				}

				if(!arrived_inaccurate)
				{
					ros::Duration dur_1;
					dur_1 = ros::Time::now() - robot_stamp;
					if(dur_1.toSec() < 0.2){
						flight.state = STATE_REVOLVING;
						arrived_inaccurate = false;
						break;
					}else{
						if(catch_position(0) > -900){
							ctrl_msg.mode = NORMAL_CTL;
							ctrl_msg.enable = 1;
							ctrl_msg.pos_sp[0] =  catch_position(0);
							ctrl_msg.pos_sp[1] =  catch_position(1);
							ctrl_msg.pos_halt[0] = 0;
							ctrl_msg.pos_halt[1] = 0;
							ctrl_msg.pos_halt[2] = 1;
							ctrl_pub.publish(ctrl_msg);
						}else{
							ctrl_msg.mode = NORMAL_CTL;
							ctrl_msg.enable = 1;
							ctrl_msg.pos_halt[0] = 1;
							ctrl_msg.pos_halt[1] = 1;
							ctrl_msg.pos_halt[2] = 1;
							ctrl_pub.publish(ctrl_msg);
						}	
					}	
					
				}else{
					ros::Duration dur_2;
					dur_2 = ros::Time::now() - robot_stamp;
					if(dur_2.toSec() > 0.2){
						flight_strategy::ctrl ctrl_msg;
						ctrl_msg.enable = 1;
						ctrl_msg.mode = NORMAL_CTL;
						ctrl_msg.pos_halt[0] = 1;
						ctrl_msg.pos_halt[1] = 1;
						ctrl_msg.pos_halt[2] = 1;
						ctrl_pub.publish(ctrl_msg);
					}else{
						flight.state = STATE_REVOLVING;
						arrived_inaccurate = false;
					}
				}
				if(!arrived_inaccurate){
					if(ctrl.flag_arrived){
						arrived_inaccurate = true;
					} 
				}

				break;
			}
			case STATE_REVOLVING:{
				if(flight.last_state != flight.state){
					ROS_INFO("State REVOLVING\n");
				}
				flight.last_state = flight.state;

				ros::Duration dur;
				dur = ros::Time::now() - robot_stamp;
				if(dur.toSec() > 0.2){
					flight_strategy::ctrl ctrl_msg;
					ctrl_msg.enable = 1;
					ctrl_msg.mode = NORMAL_CTL;
					ctrl_msg.pos_halt[0] = 1;
					ctrl_msg.pos_halt[1] = 1;
					ctrl_msg.pos_halt[2] = 1;
					ctrl_pub.publish(ctrl_msg);
				}else{
					flight_strategy::ctrl ctrl_msg;
					ctrl_msg.enable = 1;
					ctrl_msg.mode = IMAGE_CTL;
					ctrl_msg.pos_sp[0] = robot.pos_b[0];
					ctrl_msg.pos_sp[1] = robot.pos_b[1];
					ctrl_msg.pos_halt[0] = 0;
					ctrl_msg.pos_halt[1] = 0;
					ctrl_msg.pos_halt[2] = 1;
					ctrl_pub.publish(ctrl_msg);
				}
				
				if(robot.whole){
					//robot_at_desired_angle
					if(fabs(robot.yaw_field) > 0 && fabs(robot.yaw_field) < 30){
						flight.state = STATE_FLYING_AWAY;
						ROS_INFO("AAA");
						// Update the position of the drone
						pos.pos_f(0) = robot.pos_f[0];
						pos.pos_f(1) = robot.pos_f[1];
						pos.pos_b = pos.R_field_body.transpose() * pos.pos_f;
						pos_update(0) = robot.pos_f[0];
						pos_update(1) = robot.pos_f[1];
						break;
					}
				}
				if(robot.pos_f[0] > 0.1){
					flight.state = STATE_FLYING_AWAY;
					ROS_INFO("TEST%f", robot.pos_f[0]);
					break;
				}
				break;
			}
			case STATE_FLYING_AWAY:{
				if(flight.last_state != flight.state){
					ROS_INFO("State FLYING_AWAY\n");

					flight.last_state = flight.state;
					flight_strategy::ctrl ctrl_msg;
					ctrl_msg.enable = 1;
					ctrl_msg.mode = NORMAL_CTL;
					ctrl_msg.pos_sp[0] = pos.pos_f(0)-0.5;
					ctrl_msg.pos_sp[1] = pos.pos_f(1);
					ctrl_msg.pos_halt[0] = 0;
					ctrl_msg.pos_halt[1] = 0;
					ctrl_msg.pos_halt[2] = 1;
					ctrl_pub.publish(ctrl_msg);
					// Update the current position
					pos_update(0) = pos_update(0) - 0.5;
					pos_update(1) = pos_update(1);
				}

				if(ctrl.flag_arrived){
					flight.state = STATE_LOCATING;
					break;
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
