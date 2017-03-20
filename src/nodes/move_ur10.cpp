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

int main(int argc, char* argv[])
{
  

  ros::init(argc, argv, "move_ur10");
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("joint_states", 6, JointMonitorCb);
  ros::Subscriber sub2 = n.subscribe("tool_velocity", 6, TrajectoryMonitorCb);

  ros::Rate loop_rate(125);
  ros::Publisher pub_Pose = n.advertise<std_msgs::String>("ur_driver/URScript", 1);

  ros::Duration(4).sleep();


  sleep(2);

  ros::spinOnce();
  
  printf("Assuming series of arm positions!\n");
  
  float accel = 0.1;
  float vel = 0.1;
  int wait_time = 5;

  std_msgs::String msg;

  double goal[6] = {-0.202, -0.7152, -0.3, -M_PI, 0, 0};

  std::stringstream ss;

  ss<<std::fixed <<std::setprecision(4)<<"movel(p["<< goal[0] <<", "
  													<< goal[1] <<", "
  													<< goal[2] <<", "
  													<< goal[3] <<", "
  													<< goal[4] <<", "
  													<< goal[5] <<"], "<<accel<<", "<<vel<<")\n";

  msg.data = ss.str(); 
  pub_Pose.publish(msg);

  wait_move(10);

  ss.clear();

  goal[0] = -0.1997; 
  goal[1] = -0.7196; 
  goal[2] = -0.4412+0.1; 
  goal[3] = -M_PI; 
  goal[4] =  0; 
  goal[5] =  0;

  ss<<std::fixed <<std::setprecision(4)<<"movel(p["<< goal[0] <<", "
                            << goal[1] <<", "
                            << goal[2] <<", "
                            << goal[3] <<", "
                            << goal[4] <<", "
                            << goal[5] <<"], "<<accel<<", "<<vel<<")\n";
  msg.data = ss.str(); 
  pub_Pose.publish(msg);

  wait_move(10);
  ros::spinOnce();

  ss.clear();

  goal[0] = -0.1997; 
  goal[1] = -0.7196; 
  goal[2] = -0.4412; 
  goal[3] = -M_PI; 
  goal[4] =  0; 
  goal[5] =  0;
  ss<<std::fixed <<std::setprecision(4)<<"movel(p["<< goal[0] <<", "
                            << goal[1] <<", "
                            << goal[2] <<", "
                            << goal[3] <<", "
                            << goal[4] <<", "
                            << goal[5] <<"], "<<accel<<", "<<vel<<")\n";
  msg.data = ss.str(); 
  pub_Pose.publish(msg);
  
  //ros::Duration(5).sleep();
  wait_move(10);
  ros::spinOnce();

  ros::Duration(wait_time).sleep();
  ros::spinOnce();



  ros::spin();
  return 0;
}
