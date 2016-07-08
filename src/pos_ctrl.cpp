#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include "Eigen/Dense"
#include "ardrone_autonomy/Navdata.h"
#include "ardrone_control/ROI.h"
#include <flight_strategy/ctrl.h>
#include <flight_strategy/ctrlBack.h>
#include "image_process/robot_info.h"
#include "image_process/drone_info.h"

#define LOOP_RATE 20
#define IMG_TOL 50

using namespace std;
using namespace Eigen;
int constrain(int a, int b, int c){return ((a)<(b)?(b):(a)>(c)?c:a);}
int dead_zone(int a, int b){return ((a)>(b)?(a):(a)<(-b)?(a):0);}
float constrain_f(float a, float b, float c){return ((a)<(b)?(b):(a)>(c)?c:a);}
float dead_zone_f(float a, float b){return ((a)>(b)?(a):(a)<(-b)?(a):0);}
int minimum(int a, int b){return (a>b?b:a);}
int maximum(int a, int b){return (a>b?a:b);}
struct raw_state
{
	Vector3f pos_b;
	Vector3f pos_f;
	Vector3f vel_b;
	Vector3f vel_f;
	float yaw;
};

struct image_state
{
	Vector2f pos_b;
	float yaw;
};

struct control
{
	bool flag_arrived;
	int mode;
	bool pos_halt[3];
	Vector3f pos_sp;
	bool enabled;
};

struct output
{
	Vector3f vel_sp;
};

raw_state raw_state;
image_state img_state;
control ctrl;// = {false, 0,{true,true,true},{0,0,0},false};
output output;// = {{0,0,0}};
int time_count = 0;
int stop_count = 0;
bool image_ctl_flag = false;
int pulse_length = 0;
Vector2f vel_ctl_direction;
float vel_set_value;
bool x_arrived = false;
bool y_arrived = false;
bool x_arrived_l = false;
bool y_arrived_l = false;
Vector3f vel;

void ctrl_sp_callback(const flight_strategy::ctrl msg)
{
	for(int i=0;i<3;i++){
		ctrl.pos_sp(i) = msg.pos_sp[i];
		if(msg.pos_halt[i])
			ctrl.pos_halt[i] = true;
		else
			ctrl.pos_halt[i] = false;
	}
	ctrl.mode = msg.mode;
	if(ctrl.mode == 1){
		img_state.pos_b(0) = msg.pos_sp[0];
		img_state.pos_b(1) = msg.pos_sp[1];
	}
	if(ctrl.mode == 2){
		img_state.pos_b(0) = msg.pos_sp[0];
		img_state.pos_b(1) = msg.pos_sp[1];
	}
	if(ctrl.mode == 3){
		img_state.pos_b(0) = msg.pos_sp[0];
		img_state.pos_b(1) = msg.pos_sp[1];
	}
}
void yawCallback(const std_msgs::Float32 &msg)
{
	img_state.yaw = 0/57.3;
}
void odometryCallback(const nav_msgs::Odometry &msg)
{
 
}

void rawpos_f_Callback(const geometry_msgs::PoseStamped &msg)
{
	raw_state.pos_f(0)=msg.pose.position.x;
	raw_state.pos_f(1)=msg.pose.position.y;
	raw_state.pos_f(2)=msg.pose.position.z;
}

void rawpos_b_Callback(const geometry_msgs::PoseStamped &msg)
{
	raw_state.pos_b(0)=msg.pose.position.x;
	raw_state.pos_b(1)=msg.pose.position.y;
	raw_state.pos_b(2)=msg.pose.position.z;
}

void navCallback(const ardrone_autonomy::Navdata &msg)
{
	vel(0) = msg.vx/1000;
	vel(1) = msg.vy/1000;
	vel(2) = msg.vz/1000;
}

void drone_info_Callback(const image_process::drone_info msg)
{

}

bool inaccurate_control_1D(float pos_sp, float pos, float &vel_sp)
{
	float speed = 0.1;
	bool is_arrived;
	float err = pos_sp - pos;
	float dist = fabs(err);
	if(dist > 0.3){
		float direction = err / dist;
		vel_sp = direction * speed;
		is_arrived = false;
	}
	else{
		is_arrived = true;
	}
	return is_arrived;
}
bool inaccurate_control_2D(const Vector3f& pos_sp, const Vector3f& pos, Vector2f& vel_sp)
{
	float speed = 0.15;
	bool is_arrived;
	Vector3f err3 = pos_sp - pos;
	err3(2) = 0;
	Vector2f err;
	err(0) = err3(0);
	err(1) = err3(1);
	float dist = err.norm();
	if(dist > 0.2){
		Vector2f direction = err / dist;
		vel_sp = direction * speed;

		is_arrived = false;
	//	ROS_INFO("\npos(%f, %f)\nsetpt(%f, %f)\nvelsp(%f, %f)\n",_pos(0),_pos(1),_pos_sp(0),_pos_sp(1),vel_sp(0),vel_sp(1));
	}
	else{
		is_arrived = true;
	}
	return is_arrived;
}
bool inaccurate_control_3D(const Vector3f& pos_sp, const Vector3f& pos, Vector3f& vel_sp)
{
	float speed = 0.1;
	bool is_arrived;
	Vector3f err = pos_sp - pos;
	err(2) = 0;
	float dist = err.norm();
	if(dist > 0.3){
		Vector3f direction = err / dist;
		vel_sp = direction * speed;

		is_arrived = false;
	//	ROS_INFO("\npos(%f, %f)\nsetpt(%f, %f)\nvelsp(%f, %f)\n",_pos(0),_pos(1),_pos_sp(0),_pos_sp(1),vel_sp(0),vel_sp(1));
	}
	else{
		is_arrived = true;
	}
	return is_arrived;
}

bool accurate_P_PID(const Vector2f& image_pos, Vector2f& vel_sp_out, bool& x_arrived, bool& y_arrived)
{
	static Vector2f err_last;
	static Vector2f err_int;
	static bool new_start = true;
	float P_pos = 0.0022;
	float P_vel = 0.12, D_vel = 0.008, I_vel = 0.003;
	Vector2f vel_sp_2d;
	Vector2f vel_2d;
	Vector2f image_center(320.0,180.0);
	Vector2f image_pos_2d;
	image_pos_2d(0) = image_pos(0);
	image_pos_2d(1) = image_pos(1);
	Vector2f err_pos =  image_center - image_pos_2d;
	Vector2f vel_sp;
	vel_sp = err_pos * P_pos;
	vel_sp(0)=constrain_f(vel_sp(0), -0.4, 0.4);
	vel_sp(1)=constrain_f(vel_sp(1), -0.4, 0.4);
	vel_2d(0) = vel(0);
	vel_2d(1) = vel(1);
	Vector2f err_vel = vel_sp - vel_2d;

	if(new_start){
		err_last = err_vel;
		err_int(0) = 0;
		err_int(1) = 0;
		new_start = false;
	}
	Vector2f err_d = (err_vel - err_last) * LOOP_RATE;
	vel_sp_2d = err_vel * P_vel + err_d * D_vel + err_int * I_vel;
	
	vel_sp_out(0) = vel_sp_2d(1);
	vel_sp_out(1) = vel_sp_2d(0);
	vel_sp_out(0)=constrain_f(vel_sp_out(0), -0.1, 0.1);
	vel_sp_out(1)=constrain_f(vel_sp_out(1), -0.1, 0.1);
	
	if(fabs(err_pos(0)) < IMG_TOL){
		x_arrived = true;
	}else{
		x_arrived = false;
	}

	if(fabs(err_pos(1)) < IMG_TOL){
		y_arrived = true;
	}else{
		y_arrived = false;
	}

	bool is_arrived;
	float dist = err_pos.norm();
	if(dist > IMG_TOL){
		is_arrived = false;
	}
	else{
		is_arrived = true;
	}
	err_last = err_vel;
	err_int += err_vel / LOOP_RATE;
	return is_arrived;
}
bool accurate_PID(const Vector2f& image_pos, Vector2f& vel_sp_out, bool& x_arrived, bool& y_arrived)
{
	static Vector2f err_last;
	static Vector2f err_int;
	static bool new_start = true;
	float P_pos = 0.00012, D_pos = 0.00007, I_pos = 0.0;
	Vector2f vel_sp_2d;
	Vector2f image_center(320.0,180.0);
	Vector2f image_pos_2d;
	image_pos_2d(0) = image_pos(0);
	image_pos_2d(1) = image_pos(1);
	Vector2f err = image_pos_2d - image_center;
	if(new_start){
	err_last = err;
	err_int(0) = 0;
	err_int(1) = 0;
	new_start = false;
	}
	Vector2f err_d = (err - err_last) * LOOP_RATE;
	vel_sp_2d = err * P_pos + err_d * D_pos + err_int * I_pos;
	vel_sp_out(0) = -vel_sp_2d(1);
	vel_sp_out(1) = -vel_sp_2d(0);
	vel_sp_out(0)=constrain_f(vel_sp_out(0), -0.08, 0.08);
	vel_sp_out(1)=constrain_f(vel_sp_out(1), -0.08, 0.08);

	if(fabs(err(0)) < IMG_TOL){
		x_arrived = true;
	}else{
		x_arrived = false;
	}

	if(fabs(err(1)) < IMG_TOL){
		y_arrived = true;
	}else{
		y_arrived = false;
	}

	bool is_arrived;
	float dist = err.norm();
	if(dist > IMG_TOL){
	is_arrived = false;
	}
	else{
	is_arrived = true;
	}
	err_last = err;
	err_int += err / LOOP_RATE;
	return is_arrived;
}
bool image_control(const Vector2f& image_pos, Vector2f& vel_sp, float& distance)
{
	bool is_arrived;
	Vector2f image_center(320.0,180.0);
	Vector2f image_pos_2d;
	image_pos_2d(0) = image_pos(0);
	image_pos_2d(1) = image_pos(1);
	Vector2f err = image_pos_2d - image_center;
	float dist = err.norm();
	distance = dist;
	vel_sp(0) = -err(1) / dist;
	vel_sp(1) = -err(0) / dist;
	if(dist > IMG_TOL){
		is_arrived = false;
	}
	else{
		is_arrived = true;
	}
	return is_arrived;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_controler");
	ros::NodeHandle n;
//	ros::Subscriber image_pos_sub = n.subscribe("/ROI", 1, imagepositionCallback);
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::Publisher ctrlBack_pub = n.advertise<flight_strategy::ctrlBack>("ctrlBack",1);
	ros::Subscriber nav_sub = n.subscribe("/ardrone/navdata", 1, navCallback);
	ros::Subscriber odometry_sub = n.subscribe("/ardrone/odometry", 1, odometryCallback);
	ros::Subscriber rawpos_f_sub = n.subscribe("/ardrone/rawpos_f", 1, rawpos_f_Callback);
	ros::Subscriber rawpos_b_sub = n.subscribe("/ardrone/rawpos_b", 1, rawpos_b_Callback);
	ros::Subscriber ctrl_sub = n.subscribe("ctrl", 1, ctrl_sp_callback);
	ros::Subscriber drone_info_sub = n.subscribe("/ardrone/position_reset_info", 1, drone_info_Callback);
	ros::Subscriber yaw_sub = n.subscribe("/ardrone/yaw", 1, yawCallback);
	ros::Rate loop_rate(LOOP_RATE);
	bool arrived, alt_arrived;
	flight_strategy::ctrlBack ctrlBack_msg;
	for(int i = 0; i < 3; i++)
		ctrlBack_msg.arrived[i] = 0;
	while(ros::ok()){
		if(ctrl.mode == 0){
			if(ctrl.pos_halt[0] || ctrl.pos_halt[1])
			{
				output.vel_sp(0) = 0;
				output.vel_sp(1) = 0;
				ctrlBack_msg.arrived[0] = 1;
				ctrlBack_msg.arrived[1] = 1;
				ROS_INFO("STOP");
			}else{
				Vector2f vel_sp_f;
				arrived = inaccurate_control_2D(ctrl.pos_sp, raw_state.pos_f, vel_sp_f);
				ROS_INFO("\npos:%f,%f\nsp:%f,%f\nvel:%f,%f",raw_state.pos_f(0),raw_state.pos_f(1),ctrl.pos_sp(0),ctrl.pos_sp(1),vel_sp_f(0),vel_sp_f(1));
				if(arrived){
					ROS_INFO("arrived(node ctrl)");
					ctrlBack_msg.arrived[0] = 1;
					ctrlBack_msg.arrived[1] = 1;
					output.vel_sp(0) = 0;
					output.vel_sp(1) = 0;
				}
				else{
					ctrlBack_msg.arrived[0] = 0;
					ctrlBack_msg.arrived[1] = 0;
				//	output.vel_sp(0) = vel_sp(0);
				//	output.vel_sp(1) = vel_sp(1);
					Matrix<float, 2, 2> Rf;
					Rf(0,0) = cos(img_state.yaw);
					Rf(0,1) = sin(-img_state.yaw);
				//	Rf(0,2) = 0;
					Rf(1,0) = sin(img_state.yaw);
					Rf(1,1) = cos(img_state.yaw);
				//	Rf(1,2) = 0;
				//	Rf(2,0) = 0;
				//	Rf(2,1) = 0;
				//	Rf(2,2) = 1;
					Vector2f vel_sp_b_2D = Rf.transpose()*vel_sp_f;
					output.vel_sp(0) = vel_sp_b_2D(0);
					output.vel_sp(1) = vel_sp_b_2D(1);
					ROS_INFO("\nvelsp_body:%f,%f",vel_sp_b_2D(0),vel_sp_b_2D(1));
				}
			}	
			//z
			if(ctrl.pos_halt[2])
			{
				output.vel_sp(2) = 0;
				ctrlBack_msg.arrived[2] = 1;
			}else
			{
				float vel_sp_1D;
				alt_arrived = inaccurate_control_1D(ctrl.pos_sp(2), raw_state.pos_f(2), vel_sp_1D);
				if(alt_arrived){
				//	ROS_INFO("arrived");
					output.vel_sp(2) = 0;
					ctrlBack_msg.arrived[2] = 1;
				}
				else{
					ctrlBack_msg.arrived[2] = 0;
					output.vel_sp(2) = vel_sp_1D;
				}
			}		
		}
		else if(ctrl.mode == 1){
			Vector2f vel_sp_direction;
			Vector2f vel_sp;
			
			float distance;
			arrived = image_control(img_state.pos_b, vel_sp_direction, distance);
			
			if(distance < 80){
				vel_set_value = 0.1;
			}else if (distance < 120){
				vel_set_value = 0.2;
			}else if (distance < 200){
				vel_set_value = 0.4;
			}
			else{
				vel_set_value = 0.5;
			}
			if(arrived){
				ctrlBack_msg.arrived[0] = 1;
				ctrlBack_msg.arrived[1] = 1;
				output.vel_sp(0) = 0;
				output.vel_sp(1) = 0;
				output.vel_sp(2) = 0;
				ROS_INFO("Arrived");
				ctrlBack_msg.is_controlling = false;	
			}
			else{
				vel_sp(0) = 0;
				vel_sp(1) = 0;
				if(!image_ctl_flag){
					//pulse_length = distance / 40 + 1;
					pulse_length = 3;
					//if(pulse_length > 5)pulse_length = 5;
					stop_count++;
					if(stop_count > 2){
						image_ctl_flag = true;
						vel_ctl_direction = vel_sp_direction;
						stop_count = 0;
					}	
					ctrlBack_msg.is_controlling = false;	
					ROS_INFO("Stop Stop");
				}else{
					time_count++;
					//ROS_INFO("time_count:%d",time_count);
					//ROS_INFO("pulse_length:%d",pulse_length);
					if(time_count > pulse_length){
						time_count = 0;
						vel_sp(0) = 0;
						vel_sp(1) = 0;
						image_ctl_flag = false;
					}else{
						vel_sp = vel_set_value * vel_ctl_direction;
						
					}	
					ctrlBack_msg.is_controlling = true;				
				}

				ctrlBack_msg.arrived[0] = 0;
				ctrlBack_msg.arrived[1] = 0;
				output.vel_sp(0) = vel_sp(0);
				output.vel_sp(1) = vel_sp(1);
				output.vel_sp(2) = 0;
				
			}
		}
		else if(ctrl.mode == 2){
			Vector2f vel_sp;
			arrived = accurate_P_PID(img_state.pos_b, vel_sp, x_arrived, y_arrived);

			if(arrived){
				ctrlBack_msg.arrived[0] = 1;
				ctrlBack_msg.arrived[1] = 1;
				output.vel_sp(0) = 0;
				output.vel_sp(1) = 0;
				output.vel_sp(2) = 0;
			}
			else{
				ctrlBack_msg.arrived[0] = 0;
				ctrlBack_msg.arrived[1] = 0;
				output.vel_sp(0) = vel_sp(0);
				output.vel_sp(1) = vel_sp(1);
				output.vel_sp(2) = 0;
			}
		}
		else if(ctrl.mode == 3){
			Vector2f vel_sp;
			x_arrived_l = x_arrived;
			y_arrived_l = y_arrived;
			arrived = accurate_P_PID(img_state.pos_b, vel_sp, x_arrived, y_arrived);

			if(arrived){
				ctrlBack_msg.arrived[0] = 1;
				ctrlBack_msg.arrived[1] = 1;
				output.vel_sp(0) = 0;
				output.vel_sp(1) = 0;
				output.vel_sp(2) = 0;
			}
			else{
				output.vel_sp(0) = vel_sp(0);
				output.vel_sp(1) = vel_sp(1);

				if(x_arrived && !x_arrived_l){
					output.vel_sp(0) = 0;
					output.vel_sp(1) = 0;
					ROS_INFO("X && !X_L");
				}
				if(y_arrived && !y_arrived_l){
					output.vel_sp(0) = 0;
					output.vel_sp(1) = 0;
					ROS_INFO("Y && !Y_L");
				}
				if(x_arrived && y_arrived)
				{
					output.vel_sp(0) = 0;
					output.vel_sp(1) = 0;
					ROS_INFO("X && Y");
				}
				if(x_arrived && x_arrived_l)
				{
					output.vel_sp(0) = 0;
					ROS_INFO("X");
				}
				if(y_arrived && y_arrived_l)
				{
					output.vel_sp(0) = 0;
					ROS_INFO("Y");
				}
				ctrlBack_msg.arrived[0] = 0;
				ctrlBack_msg.arrived[1] = 0;
				
				output.vel_sp(2) = 0;
			}
		}
		ctrlBack_pub.publish(ctrlBack_msg);
		geometry_msgs::Twist cmd;
		cmd.linear.x = output.vel_sp(0);
		cmd.linear.y = output.vel_sp(1);
		cmd.linear.z = output.vel_sp(2);
		ROS_INFO("OUT:%f   %f", output.vel_sp(0), output.vel_sp(1));
		cmd_pub.publish(cmd);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}