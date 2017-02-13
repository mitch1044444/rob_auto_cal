#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <stdio.h>

void StateCallBack(const sensor_msgs::JointState::ConstPtr &msg)
{	
	
	
	std::ofstream myfile;
    std::string path;
	ros::param::get("parth_param_joints",path);

	if(path.empty())
	{
		myfile.open("~/Desktop/jointData.txt", std::ios::app); 
		std::cout << "File saved with joint data on Desktop" << std::endl;
	}
	else
	{
		myfile.open(path.c_str(), std::ios::app);
		std::cout << "File saved with joint data in " << path.c_str() << std::endl;
	}


	for (int i = 0; i < msg->position.size(); ++i)
	{
		myfile << msg->position[i] << " ";
	}

	myfile << "\n";
	myfile.close();
	
	ros::shutdown();

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_joints_node");
  ros::NodeHandle nh;
  if (!argv[1])
  {
  	std::cout << "Error! Include path so save file as first argument!" << std::endl;
  	ros::shutdown();
  }
  	
  std::cout << argv[1] << std::endl;
  ros::param::set("parth_param_joints",(std::string)argv[1]);

	
  ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 1000, StateCallBack); //while robot is executing the joint values are in the StateCallBack
  ros::spin();
}