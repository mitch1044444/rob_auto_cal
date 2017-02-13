#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <stdio.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <fstream>

using namespace std;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "save_trans_node");
	ros::NodeHandle nh_;
	ros::param::set("parth_param_trans",(std::string)argv[1]);
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped transformStamped;
	while (transformStamped.header.frame_id.empty())
	{
		try
		{
			transformStamped = tfBuffer.lookupTransform("chessboard", "camera",ros::Time(0));
		}

			catch (tf2::TransformException &ex) 
		{
		
		//ROS_WARN("%s",ex.what());
		continue;
		}
	}
	

	ofstream myfile;
    string path;
	ros::param::get("parth_param_trans",path);

	if(path.empty())
	{
		myfile.open("~/transformationData.txt", std::ios::app); 
		cout << "File saved with HTM in Home" << endl;
	}
	else
	{
		myfile.open(path.c_str(), std::ios::app);
		cout << "File saved with HTM in " << path.c_str() << endl;
	}

	tf2::Matrix3x3 R = tf2::Matrix3x3(tf2::Quaternion(transformStamped.transform.rotation.x,
													transformStamped.transform.rotation.y,
													transformStamped.transform.rotation.z,
													transformStamped.transform.rotation.w));
	for (int i = 0; i < 3; ++i)
	{
		myfile << R.getRow(i).x() << " " << R.getRow(i).y() << " " << R.getRow(i).z() << " ";
	}
		myfile << "\n";

	myfile << transformStamped.transform.translation.x << " " 
		<< transformStamped.transform.translation.y << " " 
		<< transformStamped.transform.translation.z;

	myfile << "\n";
	myfile.close();
	
	ros::shutdown();
}