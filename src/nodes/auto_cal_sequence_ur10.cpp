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
  

  ros::init(argc, argv, "auto_cal_sequence_ur10");
  ros::Time::init();
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("joint_states", 1, JointMonitorCb);
  ros::Subscriber sub2 = n.subscribe("tool_velocity", 1, TrajectoryMonitorCb);
  ros::ServiceClient srv_client_chess = n.serviceClient<rob_auto_cal::reqChessTransform>("reqChessTransform");
  ros::ServiceClient srv_client_trans = n.serviceClient<rob_auto_cal::reqTransCalc>("reqTransCalc");
  ros::Rate loop_rate(125);
  ros::Publisher pub_Pose = n.advertise<std_msgs::String>("ur_driver/URScript", 1);
  vector<geometry_msgs::TransformStamped> AVect;
  vector<geometry_msgs::TransformStamped> BVect;
  ros::Duration(4).sleep();

  int n_pos;                  // Number of configurations of the robot arm
  double T_center[16] = {0};  // Initial position of robot

  



  // Loading configuration parameters from ROS Parameter server

  string camera_placement = "";
  ros::param::get("camera_placement", camera_placement);
  cout << "camera_placement: " << camera_placement.c_str() << (!strcmp(camera_placement.c_str(), "hand") || (!strcmp(camera_placement.c_str(), "world"))) << endl;
  if (!(!strcmp(camera_placement.c_str(), "hand") || !strcmp(camera_placement.c_str(), "world")))
  {
    cout << "\033[38;5;1mERROR! ABORTING!\033[0m camera_placement argument must be defined as ether \"world\" or \"hand\" depending on camera placement."<< endl;
    return 0;
  }


  string config_acquisition_setting = "";
  ros::param::get("config_acquisition_setting", config_acquisition_setting);

  if (!(!strcmp(config_acquisition_setting.c_str(), "automatic square") || !strcmp(config_acquisition_setting.c_str(), "automatic cylinder") || !strcmp(config_acquisition_setting.c_str(), "file")))
  {
    cout << "\033[38;5;1mERROR! ABORTING!\033[0m config_acquisition_setting argument argument must be defined as ether \"automatic cylinder\", \"automatic square\" or \"file\" depending on choice of robot configuration acquisition method."<< endl;
    return 0;
  }





  ifstream bFile; 

  if (!strcmp(config_acquisition_setting.c_str(), "file"))
  {
    // Load file with configurations
    string config_file_path = "";
    ros::param::get("config_file_path", config_file_path);

    if (!(strcmp(config_file_path.c_str(), "")))
    {
      cout << "\033[38;5;1mERROR! ABORTING!\033[0m config_file_path argument not defined. Define the complete path to file containing configurations of the robot." << endl;
      return 0;
    }
    
    bFile.open(config_file_path.c_str());
    if (!bFile)
    {
      cout << "\033[38;5;1mERROR! ABORTING!\033[0m Cannot open config file: " << config_file_path.c_str();
      return 0;
    }
    bFile >> n_pos;

    if(n_pos < 4)
    {
      cout << "\033[38;5;1mERROR! ABORTING!\033[0m number of robot configurations are too few, please select at lest 4" << config_file_path.c_str();
      return 0;
    }


  }
  else
  {
    int config_count = 1337;
    ros::param::get("config_count", config_count);  
    if (config_count == 1337)
    {
      cout << "\033[38;5;202mWARNING!\033[0m config_count not defined, default of 20 is used" << endl;
      n_pos = 20;
    }
    else
    {
      n_pos = config_count;
    }
  }


  vector< vector<double> > T_configs(n_pos);
  vector< vector<double> > joint_configurations(n_pos);

  if(!strcmp(config_acquisition_setting.c_str(), "automatic cylinder"))
  {
    cout << "Cylinder mode selected" << endl;
    double r = 0.1;             // Radius of movement cylinder
    double r_p = 0;             // Radius of movement cylender inrease
    double h = 0;               // cylinder height based on the initial position of the robot
    double h_p = -0.3;          // Change in cylinder height based on initial position of the robot
    double h_f = 2;             // Focal point of the end effector, is useful if camera has narrow field of view
    double rounds = 2*M_PI;          // Number of rounds, is only useful if defined with h_p or r_p
    double rot = 1*M_PI;        // Total rotation of end effector across all robot positions

    double c_radius = 1337;
    ros::param::get("c_radius", c_radius);  
    if (c_radius == 1337)
    {
      cout << "\033[38;5;202mWARNING!\033[0m c_radius not defined, default of " << r << " is used." << endl;
    }
    else
    {
      r = c_radius;
    }

    double c_radius_change = 1337;
    ros::param::get("c_radius_change", c_radius_change);  
    cout << c_radius_change << endl;
    if (c_radius_change == 1337)
    {
      cout << "\033[38;5;202mWARNING!\033[0m c_radius_change not defined, default of " << r_p << " is used." << endl;
    }
    else
    {
      r_p = c_radius_change;
    }

    double c_height = 1337;
    ros::param::get("c_height", c_height);  
    if (c_height == 1337)
    {
      cout << "\033[38;5;202mWARNING!\033[0m c_height not defined, default of " << h << " is used." << endl;
    }
    else
    {
      h = c_height;
    }

    double c_height_change = 1337;
    ros::param::get("c_height_change", c_height_change);  
    if (c_height_change == 1337)
    {
      cout << "\033[38;5;202mWARNING!\033[0m c_height_change not defined, default of " << h_p << " is used." << endl;
    }
    else
    {
      h_p = c_height_change;
    }


    double focal_point = 1337;
    ros::param::get("focal_point", focal_point);  
    if (focal_point == 1337)
    {
      cout << "\033[38;5;202mWARNING!\033[0m focal_point not defined, default of " << h_f << " is used." << endl;
    }
    else
    {
      h_f = focal_point;
    }

    double c_rounds = 1337;
    ros::param::get("c_rounds", c_rounds);  
    if (c_rounds == 1337)
    {
      cout << "\033[38;5;202mWARNING!\033[0m c_rounds not defined, default of " << rounds << " is used." << endl;
    }
    else
    {
      rounds = c_rounds;
    }

    double end_rotations = 1337;
    ros::param::get("end_rotations", end_rotations);  
    if (end_rotations == 1337)
    {
      cout << "\033[38;5;202mWARNING!\033[0m end_rotations not defined, default of " << rot << " is used." << endl;
    }
    else
    {
      rot = end_rotations;
    }

    ros::spinOnce();
    forward(current_pos,T_center);
    tf2::Transform T_center_tf2(tf2::Matrix3x3(T_center[0],T_center[1],T_center[2],
      T_center[4],T_center[5],T_center[6],
      T_center[8],T_center[9],T_center[10]),tf2::Vector3(T_center[3],T_center[7],T_center[11]));

    for (int j = 0; j < 12; ++j)
    {
      cout << T_center[j] << " ";
      if(j == 3 || j == 7 || j == 11)
        cout << endl;
    }

    for (int i = 0; i < n_pos; ++i)
    {
      T_configs.at(i).resize(16);
      tf2::Transform T_temp_tf2(tf2::Matrix3x3(1,0,0,0,1,0,0,0,1),
        tf2::Vector3((r+(r_p/(double)n_pos)*(double)i)*cos(((rounds)/(double)n_pos)*(double)i),
          (r+(r_p/(double)n_pos)*(double)i)*sin(((rounds)/(double)n_pos)*(double)i),
          (h+((double)i/(double)n_pos)*h_p)));

      tf2::Transform T_config_tf2(T_center_tf2*T_temp_tf2);

      for (int j = 0; j < 3; ++j)
      {
        tf2::Matrix3x3 basisRot(T_config_tf2.getBasis());
        T_configs.at(i).at(j) = basisRot.getColumn(j).x();
        T_configs.at(i).at(j+4) = basisRot.getColumn(j).y();
        T_configs.at(i).at(j+8) = basisRot.getColumn(j).z();


      }
      T_configs.at(i).at(3) = T_config_tf2.getOrigin().x();
      T_configs.at(i).at(7) = T_config_tf2.getOrigin().y();
      T_configs.at(i).at(11) = T_config_tf2.getOrigin().z();
     

      //shift distace to forcal point by h_f
      tf2::Transform T_focalshift_tf2(tf2::Matrix3x3(1,0,0,0,1,0,0,0,1),
        tf2::Vector3(0,0,h_f));



      tf2::Vector3 tar_pos((T_center_tf2*T_focalshift_tf2).getOrigin());
      tf2::Vector3 tar_dir(tar_pos.x()-T_configs.at(i).at(3),
        tar_pos.y()-T_configs.at(i).at(7),
        tar_pos.z()-T_configs.at(i).at(11));
      tar_dir.normalize();
     
       tf2::Vector3 Z_vect(T_configs.at(i).at(2), 
           T_configs.at(i).at(6), 
           T_configs.at(i).at(10));
       Z_vect.normalize();
       double ang = acos(tar_dir.dot(Z_vect)); 
       tf2::Vector3 rot_axis(tar_dir.cross(Z_vect));
       rot_axis.normalize();

       tf2::Matrix3x3 rotation;
       rotation.setRPY(0,0,(rot/n_pos)*i);
       tf2::Matrix3x3 rot_temp = (tf2::Matrix3x3(tf2::Quaternion(rot_axis,-ang))*tf2::Matrix3x3(T_configs.at(i).at(0),T_configs.at(i).at(1),T_configs.at(i).at(2),
           T_configs.at(i).at(4),T_configs.at(i).at(5),T_configs.at(i).at(6),
           T_configs.at(i).at(8),T_configs.at(i).at(9),T_configs.at(i).at(10))*rotation);

       for (int j = 0; j < 3; ++j)
       {
           T_configs.at(i).at(j) = rot_temp.getColumn(j).x();
           T_configs.at(i).at(j+4) = rot_temp.getColumn(j).y();
           T_configs.at(i).at(j+8) = rot_temp.getColumn(j).z();
       }

    }
  }
  else if (!strcmp(config_acquisition_setting.c_str(), "automatic square"))
  {
    cout << "Square mode selected" << "n_pos: " << n_pos << endl;
    double sq_x = 0.1;             // box x size
    double sq_y = 0;               // box y size
    double sq_z = 0;               // box z size
    double sq_z_s = -0.3;          // box z shift
    double h_f = 2;             // Focal point of the end effector, is useful if camera has narrow field of view
    double rot = 1*M_PI;        // Total rotation of end effector across all robot positions
    int r_s = 1;

    double sq_x_size = 1337;
    ros::param::get("sq_x_size", sq_x_size);  
    if (sq_x_size == 1337)
    {
      cout << "\033[38;5;202mWARNING!\033[0m sq_x_size not defined, default of " << sq_x << " is used." << endl;
    }
    else
    {
      sq_x = sq_x_size;
    }

    double sq_y_size = 1337;
    ros::param::get("sq_y_size", sq_y_size);  
    cout << sq_y_size << endl;
    if (sq_y_size == 1337)
    {
      cout << "\033[38;5;202mWARNING!\033[0m sq_y_size not defined, default of " << sq_y << " is used." << endl;
    }
    else
    {
      sq_y = sq_x_size;
    }

    double sq_z_size = 1337;
    ros::param::get("sq_z_size", sq_z_size);  
    if (sq_z_size == 1337)
    {
      cout << "\033[38;5;202mWARNING!\033[0m sq_z_size not defined, default of " << sq_z << " is used." << endl;
    }
    else
    {
      sq_z = sq_z_size;
    }

    double sq_z_shift = 1337;
    ros::param::get("sq_z_shift", sq_z_shift);  
    if (sq_z_shift == 1337)
    {
      cout << "\033[38;5;202mWARNING!\033[0m sq_z_shift not defined, default of " << sq_z_s << " is used." << endl;
    }
    else
    {
      sq_z_s = sq_z_shift;
    }


    double focal_point = 1337;
    ros::param::get("focal_point", focal_point);  
    if (focal_point == 1337)
    {
      cout << "\033[38;5;202mWARNING!\033[0m focal_point not defined, default of " << h_f << " is used." << endl;
    }
    else
    {
      h_f = focal_point;
    }


    double random_seed = 1337133;
    ros::param::get("random_seed", random_seed);  
    if (random_seed == 1337133)
    {
      cout << "\033[38;5;202mWARNING!\033[0m random_seed not defined, time is used as default which is now: " << std::setprecision(12) << round(ros::Time::now().toSec()) << endl;
      r_s = round(ros::Time::now().toSec());
    }
    else
    {
      r_s = random_seed;
    }


    double end_rotations = 1337;
    ros::param::get("end_rotations", end_rotations);  
    if (end_rotations == 1337)
    {
      cout << "\033[38;5;202mWARNING!\033[0m end_rotations not defined, default of " << rot << " is used." << endl;
    }
    else
    {
      rot = end_rotations;
    }

    ros::spinOnce();
    forward(current_pos,T_center);
    tf2::Transform T_center_tf2(tf2::Matrix3x3(T_center[0],T_center[1],T_center[2],
      T_center[4],T_center[5],T_center[6],
      T_center[8],T_center[9],T_center[10]),tf2::Vector3(T_center[3],T_center[7],T_center[11]));

    for (int j = 0; j < 12; ++j)
    {
      cout << T_center[j] << " ";
      if(j == 3 || j == 7 || j == 11)
        cout << endl;
    }
    
    srand (r_s);

    for (int i = 0; i < n_pos; ++i)
    {
      T_configs.at(i).resize(16);
      tf2::Transform T_temp_tf2(tf2::Matrix3x3(1,0,0,0,1,0,0,0,1),
        tf2::Vector3(((double) rand() / (RAND_MAX))*sq_x-(sq_x/2),
          ((double) rand() / (RAND_MAX))*sq_y-(sq_y/2),
          ((double) rand() / (RAND_MAX))*sq_z-sq_z+sq_z_s));


      tf2::Transform T_config_tf2(T_center_tf2*T_temp_tf2);

      for (int j = 0; j < 3; ++j)
      {
        tf2::Matrix3x3 basisRot(T_config_tf2.getBasis());
        T_configs.at(i).at(j) = basisRot.getColumn(j).x();
        T_configs.at(i).at(j+4) = basisRot.getColumn(j).y();
        T_configs.at(i).at(j+8) = basisRot.getColumn(j).z();


      }
      T_configs.at(i).at(3) = T_config_tf2.getOrigin().x();
      T_configs.at(i).at(7) = T_config_tf2.getOrigin().y();
      T_configs.at(i).at(11) = T_config_tf2.getOrigin().z();
     

      //shift distace to forcal point by h_f
      tf2::Transform T_focalshift_tf2(tf2::Matrix3x3(1,0,0,0,1,0,0,0,1),
        tf2::Vector3(0,0,h_f));



      tf2::Vector3 tar_pos((T_center_tf2*T_focalshift_tf2).getOrigin());
      tf2::Vector3 tar_dir(tar_pos.x()-T_configs.at(i).at(3),
        tar_pos.y()-T_configs.at(i).at(7),
        tar_pos.z()-T_configs.at(i).at(11));
      tar_dir.normalize();
     
       tf2::Vector3 Z_vect(T_configs.at(i).at(2), 
           T_configs.at(i).at(6), 
           T_configs.at(i).at(10));
       Z_vect.normalize();
       double ang = acos(tar_dir.dot(Z_vect)); 
       tf2::Vector3 rot_axis(tar_dir.cross(Z_vect));
       rot_axis.normalize();

       tf2::Matrix3x3 rotation;
       rotation.setRPY(0,0,(rot/n_pos)*i);
       tf2::Matrix3x3 rot_temp = (tf2::Matrix3x3(tf2::Quaternion(rot_axis,-ang))*tf2::Matrix3x3(T_configs.at(i).at(0),T_configs.at(i).at(1),T_configs.at(i).at(2),
           T_configs.at(i).at(4),T_configs.at(i).at(5),T_configs.at(i).at(6),
           T_configs.at(i).at(8),T_configs.at(i).at(9),T_configs.at(i).at(10))*rotation);

       for (int j = 0; j < 3; ++j)
       {
           T_configs.at(i).at(j) = rot_temp.getColumn(j).x();
           T_configs.at(i).at(j+4) = rot_temp.getColumn(j).y();
           T_configs.at(i).at(j+8) = rot_temp.getColumn(j).z();
       }
     }
  }
  else
  {
    ros::spinOnce();
    forward(current_pos,T_center); // log initial position

    int total_number_of_points = 0;
    for (int i = 0; i < n_pos; ++i)
    {
      joint_configurations.at(i).resize(6);
      for (int j = 0; j < 6; ++j)
      {
       bFile >>  joint_configurations.at(i).at(j);
       total_number_of_points++;
      }
    }
    if(total_number_of_points % 6 != 0)
    {
      cout << "ERROR! The file given does not have the right formatting! Aborting!" << endl;
      cout << "Joint space input is selected, make sure the input file has this formatting." << endl;
      return 0;
    }

    // Forward kenimatics!
    for (int i = 0; i < n_pos; ++i)
    {
      T_configs.at(i).resize(16);
      forward(&joint_configurations.at(i)[0],&T_configs.at(i)[0]);
    }

     // Axis-Angle reprecentation
  }
  
  vector<vector<double> >robot_configurations(n_pos);
  
  for (int i = 0; i < n_pos; ++i)
  {
    toRobotConfiguration(T_configs.at(i),robot_configurations.at(i));
  }




  double T[16] = {0};

  sleep(2);

  ros::spinOnce();
  
  printf("Assuming series of arm positions!\n");
  
  float accel = 0.2;
  float vel = 0.2;
  int wait_time = 3;

  double res_T[16] = {0};
  ofstream myfileA;
  myfileA.open("/misc/shome/ex09/Thesis/AVects", std::ios::app); 
  ofstream myfileB;
  myfileB.open("/misc/shome/ex09/Thesis/BVects", std::ios::app); 



  std_msgs::String msg;




  for (int i = 0; i < n_pos; ++i)
  {
    if (!(ros::ok()))
    {
      ros::shutdown();
      return 0;
    }


    std::stringstream ss;

    if(!strcmp(config_acquisition_setting.c_str(), "file"))
    { 
      ss<<std::fixed <<std::setprecision(8)<<"movej(["<<joint_configurations.at(i).at(0)<<", "
        <<joint_configurations.at(i).at(1)<<", "
        <<joint_configurations.at(i).at(2)<<", "
        <<joint_configurations.at(i).at(3)<<", "
        <<joint_configurations.at(i).at(4)<<", "
        <<joint_configurations.at(i).at(5)<<"], "<<accel<<", "<<vel<<")\n";

    }
    else
    {
      ss<<std::fixed <<std::setprecision(8)<<"movel(p["<<robot_configurations.at(i).at(0)<<", "
        <<robot_configurations.at(i).at(1)<<", "
        <<robot_configurations.at(i).at(2)<<", "
        <<robot_configurations.at(i).at(3)<<", "
        <<robot_configurations.at(i).at(4)<<", "
        <<robot_configurations.at(i).at(5)<<"], "<<accel<<", "<<vel<<")\n";
    }
    
    msg.data = ss.str();   
    pub_Pose.publish(msg);
    ros::Duration(0.5).sleep();

    wait_move(15);
    cout << "Position " << i+1 << "/" << n_pos << " reached." << endl;
    ros::Duration(wait_time).sleep();


    rob_auto_cal::reqChessTransform srv_chess;

    if(srv_client_chess.call(srv_chess))
    {	


    	geometry_msgs::TransformStamped tempTFStamped = srv_chess.response.transformStamped;

      if ((double)(ros::Time::now()-tempTFStamped.header.stamp).toSec() < 0.5) // If the sample from the chessboard transformation is older than 1 second the sample is useless and disgarded!
      { 

      	tf2::Transform tempTF; 
      	tf2::convert(tempTFStamped.transform, tempTF);
      	
        // Write the A transformation to file for debugging
        for (int j = 0; j < 3; ++j)
        {
          res_T[j] = tempTF.getBasis()[0][j];
          res_T[j+4] = tempTF.getBasis()[1][j];
          res_T[j+8] = tempTF.getBasis()[2][j];
          res_T[3] = tempTF.getOrigin()[0];
          res_T[7] = tempTF.getOrigin()[1];
          res_T[11] = tempTF.getOrigin()[2];
        }
        res_T[12] = 0;
        res_T[13] = 0;
        res_T[14] = 0;
        res_T[15] = 1;
        
        for (int j = 0; j < 16; ++j)
        {
          if(!(j == 0 || j == 4 || j == 8 || j == 12))
            myfileA << " ";

          myfileA << internal << fixed << setw(13) << setfill(' ') << setprecision(10) << res_T[j];
          
          if(j == 3 || j == 7 || j == 11 || j == 15)
            myfileA << endl;
        }

        myfileA << "\n";
          
        

        if(!(strcmp(camera_placement.c_str(), "hand")))  // If the camera is on the hand the tranformation is inversed
        {
          tf2::convert(tempTF.inverse(), tempTFStamped.transform);
        }
        


      	AVect.push_back(tempTFStamped);

      	cout << "Transformation to chessboard recieved!" << endl;
      
        ros::spinOnce(); // Updates current_pos
        //current_pos[1] += -0.006;
        forward(current_pos,T);
        //double dist_to_origo = sqrt(T[3]*T[3]+T[7]*T[7]+T[11]*T[11]);
        //cout << dist_to_origo << endl;;
        //T[11] += 0.005*dist_to_origo;
        BVect.push_back(createTransformStamped(T));

        // Write the B transformation to file for debugging
        
        for (int j = 0; j < 16; ++j)
        {
          if(!(j == 0 || j == 4 || j == 8 || j == 12))
            myfileB << " ";

          myfileB << internal << fixed << setw(13) << setfill(' ') << setprecision(10) << T[j];
          
          if(j == 3 || j == 7 || j == 11 || j == 15)
            myfileB << endl;
        }

        myfileB << "\n";
          
        


      }
      else
      {
        cout << "\033[38;5;202mWARNING!\033[0m Sample disgarded, chessboard transformation could not be obtained in this configuration!" << endl;
      }
    }
    else 
    {
      cout << "\033[38;5;1mFAILURE!\033[0m Transformation to chessboard failed!" << endl
      << "The chessboard ROS node is not running, check camera and driver" << endl;
    }

  }

  // Move back to initial position

  std::stringstream ss;

  vector<double> final_robot_configuration(6);
  vector<double> vect_T_center(T_center, T_center + sizeof T_center / sizeof T_center[0]);
  toRobotConfiguration(vect_T_center,final_robot_configuration);


  ss<<std::fixed <<std::setprecision(8)<<"movel(p["<<final_robot_configuration.at(0)<<", "
                              <<final_robot_configuration.at(1)<<", "
                              <<final_robot_configuration.at(2)<<", "
                              <<final_robot_configuration.at(3)<<", "
                              <<final_robot_configuration.at(4)<<", "
                              <<final_robot_configuration.at(5)<<"], "<<accel<<", "<<vel<<")\n";

  msg.data = ss.str();   
  pub_Pose.publish(msg);



  myfileA.close();
  myfileB.close();
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
