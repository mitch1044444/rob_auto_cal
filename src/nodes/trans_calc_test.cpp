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

//const bool Cartesian = 1; // if 1 goes for Cartesian coordinates in file if 0 uses joint values

  
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
    

void toRobotConfiguration(vector<double>& HTM, vector<double>& robot_conf)
{
  tf2::Quaternion q;
  tf2::Vector3 axis_scaled;
  double angle;
  robot_conf.resize(6);
  tf2::Matrix3x3(HTM.at(0), HTM.at(1), HTM.at(2),
            HTM.at(4), HTM.at(5), HTM.at(6),
            HTM.at(8), HTM.at(9), HTM.at(10)).getRotation(q);
  axis_scaled = ((q.getAngle())*(q.getAxis()));
  robot_conf.at(0)=HTM.at(3);
  robot_conf.at(1)=HTM.at(7);
  robot_conf.at(2)=HTM.at(11);
  robot_conf.at(3)=axis_scaled.x();
  robot_conf.at(4)=axis_scaled.y();
  robot_conf.at(5)=axis_scaled.z();
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



void wait_move(float timeout)
{
	ros::Duration(0.1).sleep();
	ros::Time start_time = ros::Time::now();
	while(true)
	{
		ros::spinOnce();

		if (ros::Time::now()-start_time >= ros::Duration((double)timeout))
		{
			//cout << "Move took too long, continuing..." << endl;
			break;
		}
		
		if(trajectory[0] == 0.0 && trajectory[1] == 0.0 && trajectory[2] == 0.0 &&
			trajectory[3] == 0.0 && trajectory[4] == 0.0 && trajectory[5] == 0.0)
		{
			//cout << "Move completed, continuing..." << endl;
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
  

  ros::init(argc, argv, "trans_calc_test");
  ros::Time::init();
  ros::NodeHandle n;
  ros::ServiceClient srv_client_trans = n.serviceClient<rob_auto_cal::reqTransCalc>("reqTransCalc");
  ros::Rate loop_rate(125);
  vector<geometry_msgs::TransformStamped> AVect;
  vector<geometry_msgs::TransformStamped> BVect;

  


  int n_pos = 40;                  // Number of configurations of the robot arm

  // Load synthetic A and B from files

 

  ifstream A_file("/misc/shome/ex09/Thesis/A_file_synthetic"); 
  ifstream B_file("/misc/shome/ex09/Thesis/B_file_synthetic");



  for (int i = 0; i < n_pos; ++i)
  {
    double A[16] = {0};
    for (int j = 0; j < 16; ++j)
    {
      A_file >> A[j];
    }
    AVect.push_back(createTransformStamped(A)); 
  }
  
   for (int i = 0; i < n_pos; ++i)
  {
    double B[16] = {0};
    for (int j = 0; j < 16; ++j)
    {
      B_file >> B[j];
    }
    BVect.push_back(createTransformStamped(B));
  }


  
  rob_auto_cal::reqTransCalc srv_trans;

  srv_trans.request.Avec = AVect;
  srv_trans.request.Bvec = BVect;

  cout << "Transformation calcualtion started, this may take several minuts." << endl;
  cout << "When the calcualtion has completed or failed a message will be printed." << endl;

  if(srv_client_trans.call(srv_trans))
  {
  	cout << "\033[38;5;2mSuccess!\033[0m" << endl << endl;
  		

    tf2::Transform X;
    tf2::Transform Z;

    tf2::convert(srv_trans.response.X.transform, X);
    tf2::convert(srv_trans.response.Z.transform, Z);

    tf2::Matrix3x3 Rx = X.getBasis();
    tf2::Vector3 tx = X.getOrigin();
    tf2::Matrix3x3 Rz = Z.getBasis();
    tf2::Vector3 tz = Z.getOrigin();
 
    double T_x[16] = {0};
    double T_z[16] = {0};

    for (int i = 0; i < 3; ++i)
    {
      T_x[i] = Rx[0][i];
      T_x[i+4] = Rx[1][i];
      T_x[i+8] = Rx[2][i];
    }
    T_x[3] = tx.x();
    T_x[7] = tx.y();
    T_x[11] = tx.z();
    T_x[15] = 1.0;

    for (int i = 0; i < 3; ++i)
    {
      T_z[i] = Rz[0][i];
      T_z[i+4] = Rz[1][i];
      T_z[i+8] = Rz[2][i];
    }
    T_z[3] = tz.x();
    T_z[7] = tz.y();
    T_z[11] = tz.z();
    T_z[15] = 1.0;

    

   	cout << "X is equal to:" << endl;

    for (int i = 0; i < 16; ++i)
    {
      if(!(i == 0 || i == 4 || i == 8 || i == 12))
        cout << " ";

      cout << internal << fixed << setw(13) << setfill(' ') << setprecision(10) << T_x[i];
      
      if(i == 3 || i == 7 || i == 11 || i == 15)
        cout << endl;
    }

    cout << endl;
          

    cout << "Z is equal to:" << endl;

    for (int i = 0; i < 16; ++i)
    {
      if(!(i == 0 || i == 4 || i == 8 || i == 12))
        cout << " ";

      cout << internal << fixed << setw(13) << setfill(' ') << setprecision(10) << T_z[i];
      
      if(i == 3 || i == 7 || i == 11 || i == 15)
        cout << endl;
    }
    cout << endl;
            

  }
  else
  	cout << "\033[1;31mFailure!\033[0m Algorithm failed to return any transformations." << endl << "This may be caused by not having enough robot configureations." 
    << endl << "Try adding more arm positions or changeing the initial position. " << endl;

  ros::spin();
  return 0;
}
