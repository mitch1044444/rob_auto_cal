//#include "ur10_kinematics/ur_kin.h"

#include <math.h>
#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <sstream>
#include <rob_auto_cal/reqChessTransform.h>
#include <rob_auto_cal/reqTransCalc.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <vector>

float Angle_R2D = 180/M_PI;

using namespace std;

double current_pos[6]={0, 0, 0, 0, 0, 0};

float trajectory[6]={0, 0, 0, 0, 0, 0};

const double d1 =  0.1273;
const double a2 = -0.612;
const double a3 = -0.5723;
const double d4 =  0.163941;
const double d5 =  0.1157;
const double d6 =  0.0922;



void forward(double* q, double* T) {
	double s1 = sin(q[0]);
    double c1 = cos(q[0]);
    double s2 = sin(q[1]);
    double c2 = cos(q[1]);
    double s3 = sin(q[2]);
    double c3 = cos(q[2]);
    double s5 = sin(q[4]);
    double c5 = cos(q[4]);
    double s6 = sin(q[5]);
    double c6 = cos(q[5]);
    double  q234 = q[1] + q[2] + q[3];
    double  s234 = sin(q234);
    double  c234 = cos(q234);

    T[2] = -(((c1*c234-s1*s234)*s5)/2.0 - c5*s1 + ((c1*c234+s1*s234)*s5)/2.0);
    T[0] = (c6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0) -
          (s6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0);
    T[1] = (-(c6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0 -
          s6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0));
    T[3] = -((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 -
          d4*s1 + (d6*(c1*c234-s1*s234)*s5)/2.0 + (d6*(c1*c234+s1*s234)*s5)/2.0 -
          a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3);
    T[6] = -(c1*c5 + ((s1*c234+c1*s234)*s5)/2.0 + ((s1*c234-c1*s234)*s5)/2.0);
    T[4] = (c6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0) +
          s6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0));
    T[5] = (c6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0) -
          s6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0));
    T[7] = -((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1 +
          (d6*(s1*c234+c1*s234)*s5)/2.0 + (d6*(s1*c234-c1*s234)*s5)/2.0 + d6*c1*c5 -
          a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3);
    T[10] = ((c234*c5-s234*s5)/2.0 - (c234*c5+s234*s5)/2.0);
    T[8] = -((s234*c6-c234*s6)/2.0 - (s234*c6+c234*s6)/2.0 - s234*c5*c6);
    T[9] = -(s234*c5*s6 - (c234*c6+s234*s6)/2.0 - (c234*c6-s234*s6)/2.0);
    T[11] = (d1 + (d6*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3) + a2*s2 -
         (d6*(c234*c5+s234*s5))/2.0 - d5*c234);
    T[12] = 0.0;
    T[13] = 0.0;
    T[14] = 0.0;
    T[15] = 1.0;
}
    



void printJoint_D(float* joint)
{
  for (int i;i<6;i++)
      joint[i] = joint[i] * Angle_R2D ;
  
  ROS_INFO("current joint values in degree:%.3f %.3f %.3f %.3f %.3f %.3f", joint[0], joint[1], joint[2],joint[3],joint[4],joint[5]); 
}

void printJoint_R(double* joint)
{  
  ROS_INFO("current joint values:%.3f %.3f %.3f %.3f %.3f %.3f", joint[0], joint[1], joint[2],joint[3],joint[4],joint[5]); 
}

void JointMonitorCb(const sensor_msgs::JointState data)
{
   
//     disordered sensor data !!

	for (int i = 0; i < 6; ++i)
	{
		current_pos[i] = (double)data.position[i];
	}
	
   
//    printJoint_D(current_pos);
//    printJoint_R(current_pos);
   //cout << current_pos[0] << " " << current_pos[1] << " " << current_pos[2] << " " << current_pos[3] << " " << current_pos[4] << " " << current_pos[5] << endl;
}



void TrajectoryMonitorCb(const geometry_msgs::TwistStampedPtr& data)
{
	trajectory[0] = data->twist.linear.x;
	trajectory[1] = data->twist.linear.y;
	trajectory[2] = data->twist.linear.z;
	trajectory[3] = data->twist.angular.x;
	trajectory[4] = data->twist.angular.y;
	trajectory[5] = data->twist.angular.z;
}

void ur_movel(std::stringstream ss, float x, float rot, float vel, float accel)
{
//     ss<<std::fixed <<std::setprecision(4)<<"movel(p["<<x[0]<<", "<<x[1]<<", "<<x[2]<<", "<<rot[0]<<", "<<rot[1]<<", "<<rot[2]<<"], "<<accel<<", "<<vel<<")\n";
}

void ur_movej(double j0, double j1, double j2,
         double j3, double j4, double j5,
         double accel, double vel)
{
    char line[200];
    sprintf(line, "movej([%.4f, %.4f, %.4f, %.4f, %.4f, %.4f], %.4f, %.4f)\n",
                          j0,    j1,    j2,    j3, j4, j5, accel, vel);
    
    printf("\n%s", line);
//     sendline(line);
}

void wait_move(float timeout)
{
	ros::Duration(0.1).sleep();
	ros::Time start_time = ros::Time::now();
	while(true)
	{
		ros::spinOnce();

		if (ros::Time::now()-start_time >= ros::Duration((double)timeout))
		{
			cout << "Move took too long, continuing..." << endl;
			break;
		}
		
		if(trajectory[0] == 0.0 && trajectory[1] == 0.0 && trajectory[2] == 0.0 &&
			trajectory[3] == 0.0 && trajectory[4] == 0.0 && trajectory[5] == 0.0)
		{
			cout << "Move completed, continuing..." << endl;
			break;
		}
	}
}


geometry_msgs::TransformStamped createTransformStamped(double* T)
{
	tf2::Matrix3x3 R;
  	tf2::Quaternion q;
	geometry_msgs::TransformStamped tempBVect;
	tempBVect.header.stamp = ros::Time::now();
	tempBVect.header.frame_id = "base";
	tempBVect.child_frame_id = "hand";
	tempBVect.transform.translation.x = T[3];
	tempBVect.transform.translation.y = T[7];
	tempBVect.transform.translation.z = T[11];
	R = tf2::Matrix3x3(T[0], T[1], T[2], 
					T[4], T[5], T[6], 
					T[8], T[9], T[10]);
	R.getRotation(q);
	tempBVect.transform.rotation.x = q.x();
	tempBVect.transform.rotation.y = q.y();
	tempBVect.transform.rotation.z = q.z();
	tempBVect.transform.rotation.w = q.w();
	return tempBVect;
}


void toRobotConf(double* robot_conf, tf2::Transform& T)
{
    tf2::Vector3 axis_scaled;
    tf2::Vector3 direction;
    direction = T.getOrigin();
    axis_scaled = ((T.getRotation().getAngle())*(T.getRotation().getAxis()));
    robot_conf[0] = direction.x();
    robot_conf[1] = direction.y();
    robot_conf[2] = direction.z();
    robot_conf[3] = axis_scaled.x();
    robot_conf[4] = axis_scaled.y();
    robot_conf[5] = axis_scaled.z();
}

void  move_ur10(double* robot_conf, ros::Publisher& pub_Pose)
{

  float accel = 0.1;
  float vel = 0.2;
  int wait_time = 0.1;


  std_msgs::String msg;

  double goal[6] = {-0.202, -0.7152, -0.3, -M_PI, 0, 0};

  std::stringstream ss;

  ss<<std::fixed <<std::setprecision(10)<<"movel(p["<< robot_conf[0] <<", "
                             << robot_conf[1] <<", "
                             << robot_conf[2] <<", "
                             << robot_conf[3] <<", "
                             << robot_conf[4] <<", "
                             << robot_conf[5] <<"], "<<accel<<", "<<vel<<")\n";

  msg.data = ss.str(); 
  pub_Pose.publish(msg);
  ros::spinOnce();
  wait_move(10);
  ros::spinOnce();
  ros::Duration(wait_time).sleep();
  ros::spinOnce();

}



int main(int argc, char* argv[])
{
  

  ros::init(argc, argv, "chess_pickup_sequence");
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("joint_states", 6, JointMonitorCb);
  ros::Subscriber sub2 = n.subscribe("tool_velocity", 6, TrajectoryMonitorCb);
  ros::Time::init();
  ros::Rate loop_rate(125);
  ros::Publisher pub_Pose = n.advertise<std_msgs::String>("ur_driver/URScript", 1);
  ros::ServiceClient srv_client_chess = n.serviceClient<rob_auto_cal::reqChessTransform>("reqChessTransform");
  ros::Duration(10).sleep();

  // Read file for X and Z

  if(argc < 2)
  {
    cout << "Please specify location of Z file as input" << endl;
    return 0;  
  }

  ifstream bFile(argv[1]);
  if (!bFile)
  {
    cout << "Cannot open Z file: " << argv[1] << endl;
    return 0;
  }
  

  double X[16] = {0};
  double B[16] = {0};

  for (int i = 0; i < 16; ++i)
  {
    bFile >> X[i];
  }
  tf2::Transform X_t(tf2::Matrix3x3(X[0],X[1],X[2],X[4],X[5],X[6],X[8],X[9],X[10]),tf2::Vector3(X[3],X[7],X[11]));


  ros::spinOnce();
  forward(current_pos,B);
  
  tf2::Transform B_t(tf2::Matrix3x3(B[0],B[1],B[2],B[4],B[5],B[6],B[8],B[9],B[10]),tf2::Vector3(B[3],B[7],B[11]));

  tf2::Transform T_tp_t(tf2::Matrix3x3(1,0,0,0,1,0,0,0,1),tf2::Vector3(0,0,0.220));

  tf2::Transform T_x(tf2::Matrix3x3(1,0,0,0,1,0,0,0,1),tf2::Vector3(0.0972,0,0));

  tf2::Transform T_y(tf2::Matrix3x3(1,0,0,0,1,0,0,0,1),tf2::Vector3(0,0.0608,0));

  tf2::Transform T_cube(tf2::Matrix3x3(1,0,0,0,1,0,0,0,1),tf2::Vector3(0,0,-0.030));


  double robot_conf[6] = {0};

  vector<tf2::Transform> transform_sequence;


  


  transform_sequence.push_back(X_t.inverse()*T_tp_t.inverse()*T_tp_t.inverse()); // over center
  transform_sequence.push_back(transform_sequence.at(0)*T_x*T_y); // over first cube
  transform_sequence.push_back(transform_sequence.at(1)*T_tp_t); // grap frist cube
  transform_sequence.push_back(transform_sequence.at(1)); // over first cube
  transform_sequence.push_back(transform_sequence.at(0)*T_tp_t); // center surface
  transform_sequence.push_back(transform_sequence.at(0));  // over center
  transform_sequence.push_back(transform_sequence.at(5)*T_x*T_y.inverse()); // over secound cube
  transform_sequence.push_back(transform_sequence.at(6)*T_tp_t); // grap secound cube
  transform_sequence.push_back(transform_sequence.at(6)); // over secound cube
  transform_sequence.push_back(transform_sequence.at(4)*T_cube); // center over first cube
  transform_sequence.push_back(transform_sequence.at(0));       // over center
  transform_sequence.push_back(transform_sequence.at(0)*T_x.inverse()*T_y.inverse()); // over thrid cube
  transform_sequence.push_back(transform_sequence.at(11)*T_tp_t); // grap third cube
  transform_sequence.push_back(transform_sequence.at(11));      // over third cube
  transform_sequence.push_back(transform_sequence.at(9)*T_cube); // center over secound cube
  transform_sequence.push_back(transform_sequence.at(0));       // over center
  transform_sequence.push_back(transform_sequence.at(0)*T_x.inverse()*T_y); // Over forth cube
  transform_sequence.push_back(transform_sequence.at(16)*T_tp_t);   // grap forth cube
  transform_sequence.push_back(transform_sequence.at(16));  // over forth cube
  transform_sequence.push_back(transform_sequence.at(14)*T_cube); // center over third cube


  cout << "bleh" << endl;

  tf2::Transform next_position;



  bool gripper_open[20] =  {1,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};

  bool gripper_close[20] = {0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0};

  cout << "bluh" << endl;

  for (int i = 0; i < transform_sequence.size(); ++i)
  {

    next_position = transform_sequence.at(i);

    toRobotConf(robot_conf,next_position);

    

    ros::spinOnce();
  
    move_ur10(robot_conf, pub_Pose);
  
    if (gripper_open[i])
    {
      std::stringstream ss;
      ss << "set_digital_out(8,False)\n";
      std_msgs::String msg;
      msg.data = ss.str(); 
      pub_Pose.publish(msg);
      ros::Duration(1).sleep();
    }

    if (gripper_close[i])
    {
      std::stringstream ss;
      ss << "set_digital_out(8,True)\n";
      std_msgs::String msg;
      msg.data = ss.str(); 
      pub_Pose.publish(msg);
      ros::Duration(1).sleep();
    }


  }




  ros::shutdown();
  return 0;
}
