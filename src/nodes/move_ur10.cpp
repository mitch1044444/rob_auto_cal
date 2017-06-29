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
  ros::ServiceClient srv_client_chess = n.serviceClient<rob_auto_cal::reqChessTransform>("reqChessTransform");
  ros::Duration(4).sleep();


  tf2::Quaternion q;
  tf2::Vector3 axis_scaled;
  double robot_conf[6] = {0};

  double T_conf[16] = 
  {
-0.552983432583, 0.833190074694, -0.001903355156, 0.025534308427, 
0.833178211123, 0.552959412428, -0.007068005162, -0.690487390695, 
-0.004836513579, -0.005494323719, -0.999973209925, -0.113499676085, 
0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000 
  };
  
  // double T_tool_end[16] = 
  // {
  // 1.000000000000, 0.000000000000, 0.000000000000, -0.000000000000, 
  // 0.000000000000, 1.000000000000, 0.000000000000, -0.000000000000, 
  // 0.000000000000, 0.000000000000, 1.000000000000, 0.201500000000, 
  // 0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000
  // }

  // tf2::Transform Tranform_conf; 
  // tf2::Transform Tranform_tool;
  // Transform_conf.rotation(T_conf[0])


  tf2::Matrix3x3(T_conf[0],T_conf[1],T_conf[2],
                T_conf[4],T_conf[5],T_conf[6],
                T_conf[8],T_conf[9],T_conf[10]).getRotation(q);
    axis_scaled = ((q.getAngle())*(q.getAxis()));
    robot_conf[0] = T_conf[3];
    robot_conf[1] = T_conf[7];
    robot_conf[2] = T_conf[11];
    robot_conf[3] = axis_scaled.x();
    robot_conf[4] = axis_scaled.y();
    robot_conf[5] = axis_scaled.z();

    for (int i = 0; i < 6; ++i)
    {
      cout << robot_conf[i] << " ";
    }
    cout << endl;


  sleep(2);

  ros::spinOnce();
  
  printf("Assuming series of arm positions!\n");
  
  float accel = 0.1;
  float vel = 0.2;
  int wait_time = 1;

  std_msgs::String msg;

  double goal[6] = {3.12165, -0.152307, 1.32192, -1.14524, 1.57082, -2.26646e-05}; //{-0.202, -0.7152, -0.3, -M_PI, 0, 0};


  std::stringstream ss;

  ofstream myfileA;
  ofstream myfileB;

  myfileA.open("/misc/shome/ex09/Thesis/A_file", std::ios::app); 
  myfileB.open("/misc/shome/ex09/Thesis/B_file", std::ios::app); 




  //goal[0] = robot_conf[0];
  //goal[1] = robot_conf[1];
  //goal[2] = robot_conf[2];
  //goal[3] = robot_conf[3];
  //goal[4] = robot_conf[4];
  //goal[5] = robot_conf[5];

  goal[3] += 0.05;

  //<<std::fixed <<std::setprecision(10)<<"movel(p["<< goal[0] <<", "
  //                          << goal[1] <<", "
  //                          << goal[2] <<", "
  //                          << goal[3] <<", "
  //                          << goal[4] <<", "
  //                          << goal[5] <<"], "<<accel<<", "<<vel<<")\n";

   ss<<std::fixed <<std::setprecision(10)<<"movej(["<< goal[0] <<", "
                              << goal[1] <<", "
                              << goal[2] <<", "
                              << goal[3] <<", "
                              << goal[4] <<", "
                              << goal[5] <<"], "<<accel<<", "<<vel<<")\n";

  msg.data = ss.str(); 
  pub_Pose.publish(msg);

  wait_move(10);
  ros::spinOnce();
  ros::Duration(wait_time).sleep();
  ros::spinOnce();


  tf2::Transform first_pos;

  rob_auto_cal::reqChessTransform srv_chess;
  
  if(srv_client_chess.call(srv_chess))
  { 
    geometry_msgs::TransformStamped tempTFStamped = srv_chess.response.transformStamped;
    tf2::convert(tempTFStamped.transform, first_pos);


    cout << "Transformation to chessboard recieved!" << endl;
  }
  else {cout << "FAILURE! Transformation to chessboard failed!" << endl;}
  ros::spinOnce();

  double A_T[16] = {0};

  for (int i = 0; i < 3; ++i)
  {
    A_T[i] = first_pos.getBasis()[0][i];
    A_T[i+4] = first_pos.getBasis()[1][i];
    A_T[i+8] = first_pos.getBasis()[2][i];
    A_T[3] = first_pos.getOrigin()[0];
    A_T[7] = first_pos.getOrigin()[1];
    A_T[11] = first_pos.getOrigin()[2];
  }
    A_T[12] = 0;
    A_T[13] = 0;
    A_T[14] = 0;
    A_T[15] = 1;

  for (int i = 0; i < 16; ++i)
  {
    if(!(i == 0 || i == 4 || i == 8 || i == 12))
      myfileA << " ";

    myfileA << internal << fixed << setw(13) << setfill(' ') << setprecision(10) << A_T[i];
    
    if(i == 3 || i == 7 || i == 11 || i == 15)
      myfileA << endl;
  }
 
  myfileA << "\n";

  double B_T[16] = {0};

  forward(current_pos,B_T);

    for (int i = 0; i < 16; ++i)
    {
      if(!(i == 0 || i == 4 || i == 8 || i == 12))
        myfileB << " ";

      myfileB << internal << fixed << setw(13) << setfill(' ') << setprecision(10) << B_T[i];
      
      if(i == 3 || i == 7 || i == 11 || i == 15)
        myfileB << endl;
    }
  myfileB << "\n";


  ss.clear();


  //goal[2] += -0.2;
  //goal[4] += 0.1;
  //goal[5] += 0;
  goal[3] += -0.1;
  //ss<<std::fixed <<std::setprecision(10)<<"movel(p["<< goal[0] <<", "
  //                          << goal[1] <<", "
  //                          << goal[2] <<", "
  //                          << goal[3] <<", "
  //                        << goal[4] <<", "
  //                          << goal[5] <<"], "<<accel<<", "<<vel<<")\n";
  
   ss<<std::fixed <<std::setprecision(10)<<"movej(["<< goal[0] <<", "
                            << goal[1] <<", "
                            << goal[2] <<", "
                            << goal[3] <<", "
                            << goal[4] <<", "
                            << goal[5] <<"], "<<accel<<", "<<vel<<")\n";



  msg.data = ss.str(); 
  pub_Pose.publish(msg);
  
  wait_move(10);
  ros::spinOnce();
  ros::Duration(wait_time).sleep();
  ros::spinOnce();



  tf2::Transform second_pos;

  if(srv_client_chess.call(srv_chess))
  { 
    geometry_msgs::TransformStamped tempTFStamped = srv_chess.response.transformStamped;
    tf2::convert(tempTFStamped.transform, second_pos);
    cout << "Transformation to chessboard recieved!" << endl;
  }
  else {cout << "FAILURE! Transformation to chessboard failed!" << endl;}
  ros::spinOnce();


    for (int i = 0; i < 3; ++i)
  {
    A_T[i] = second_pos.getBasis()[0][i];
    A_T[i+4] = second_pos.getBasis()[1][i];
    A_T[i+8] = second_pos.getBasis()[2][i];
    A_T[3] = second_pos.getOrigin()[0];
    A_T[7] = second_pos.getOrigin()[1];
    A_T[11] = second_pos.getOrigin()[2];
  }
    A_T[12] = 0;
    A_T[13] = 0;
    A_T[14] = 0;
    A_T[15] = 1;

  for (int i = 0; i < 16; ++i)
  {
    if(!(i == 0 || i == 4 || i == 8 || i == 12))
      myfileA << " ";

    myfileA << internal << fixed << setw(13) << setfill(' ') << setprecision(10) << A_T[i];
    
    if(i == 3 || i == 7 || i == 11 || i == 15)
      myfileA << endl;
  }

  myfileA << "\n";
  myfileA.close();



  forward(current_pos,B_T);
  for (int i = 0; i < 16; ++i)
  {
    if(!(i == 0 || i == 4 || i == 8 || i == 12))
      myfileB << " ";

    myfileB << internal << fixed << setw(13) << setfill(' ') << setprecision(10) << B_T[i];
      
    if(i == 3 || i == 7 || i == 11 || i == 15)
    myfileB << endl;
  }
 
  myfileB << "\n";
  myfileB.close();


  tf2::Transform pos_dif;


  pos_dif = (first_pos.inverse()*second_pos);


  for (int i = 0; i < 3; ++i)
  {
    A_T[i] = pos_dif.getBasis()[0][i];
    A_T[i+4] = pos_dif.getBasis()[1][i];
    A_T[i+8] = pos_dif.getBasis()[2][i];
    A_T[3] = pos_dif.getOrigin()[0];
    A_T[7] = pos_dif.getOrigin()[1];
    A_T[11] = pos_dif.getOrigin()[2];
  }
    A_T[12] = 0;
    A_T[13] = 0;
    A_T[14] = 0;
    A_T[15] = 1;

  for (int i = 0; i < 16; ++i)
  {
    if(!(i == 0 || i == 4 || i == 8 || i == 12))
      cout << " ";

    cout << internal << fixed << setw(13) << setfill(' ') << setprecision(10) << A_T[i];
    
    if(i == 3 || i == 7 || i == 11 || i == 15)
      cout << endl;
  }
 
  cout << "Total length traveled: " << sqrt(A_T[3]*A_T[3] + A_T[7]*A_T[7] + A_T[11]*A_T[11]) << endl;
   





  ros::shutdown();
  return 0;
}
