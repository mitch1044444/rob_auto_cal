#include <ros/ros.h>
#include <rob_auto_cal/vectorVector3.h>
#include <stdio.h>
#include <fstream>

using namespace std;

void subCB(const rob_auto_cal::vectorVector3::ConstPtr& msg)
{
	ofstream myfile;
	myfile.open("/shome/ex09/Thesis/myfile.txt", std::ios::app); // This path should be changed to where the file should be placed
	
	myfile << ros::Time::now() << "\n";

	for (int i=0; i<msg->vectors.size(); ++i)
	{
		cout << msg->vectors.size() << endl;
		const geometry_msgs::Vector3 &data = msg->vectors[i];
		myfile << data.x << " " << data.y << " " << data.z << "\n";
	}

	myfile << "---" << "\n";
	myfile.close();
	ros::shutdown();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "save_data_node");
	ros::NodeHandle nh_;
	ros::Subscriber sub = nh_.subscribe("/feature_vectors", 1000, subCB);
	ros::spin();
}