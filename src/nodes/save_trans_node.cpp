#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <stdio.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <rob_auto_cal/reqChessTransform.h>

using namespace std;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "save_trans_node");
	ros::NodeHandle nh_;
	ros::param::set("parth_param_trans",(std::string)argv[1]);

	ros::ServiceClient srv_client_chess = nh_.serviceClient<rob_auto_cal::reqChessTransform>("reqChessTransform");
	geometry_msgs::TransformStamped transformStamped;
	// while (transformStamped.header.frame_id.empty())
	// {
	// 	try
	// 	{
	// 		transformStamped = tfBuffer.lookupTransform("chessboard", "camera",ros::Time(0));
	// 	}

	// 		catch (tf2::TransformException &ex) 
	// 	{
		
	// 	//ROS_WARN("%s",ex.what());
	// 	continue;
	// 	}
	// }
	rob_auto_cal::reqChessTransform srv_chess;

	tf2::Transform pos;
	if(srv_client_chess.call(srv_chess))
  	{ 
    	geometry_msgs::TransformStamped tempTFStamped = srv_chess.response.transformStamped;
    	tf2::convert(tempTFStamped.transform, pos);
    	cout << "Transformation to chessboard recieved!" << endl;
  	}
  	else {cout << "FAILURE! Transformation to chessboard failed!" << endl;}


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

	

	double res_T[16] = {0};

	for (int i = 0; i < 3; ++i)
  {
    res_T[i] = pos.getBasis()[0][i];
    res_T[i+4] = pos.getBasis()[1][i];
    res_T[i+8] = pos.getBasis()[2][i];
    res_T[3] = pos.getOrigin()[0];
    res_T[7] = pos.getOrigin()[1];
    res_T[11] = pos.getOrigin()[2];
  }
    res_T[12] = 0;
    res_T[13] = 0;
    res_T[14] = 0;
    res_T[15] = 1;

  for (int i = 0; i < 16; ++i)
  {
    if(!(i == 0 || i == 4 || i == 8 || i == 12))
      myfile << " ";

    myfile << internal << fixed << setw(13) << setfill(' ') << setprecision(10) << res_T[i];
    
    if(i == 3 || i == 7 || i == 11 || i == 15)
      myfile << endl;
  }
 

	myfile << "\n";
	myfile.close();
	
	ros::shutdown();
}