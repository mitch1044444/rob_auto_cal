#include <ros/ros.h>
#include <rob_auto_cal/vectorVector3.h>
#include <stdio.h>
#include <fstream>

using namespace std;

void subCBvec(const rob_auto_cal::vectorVector3::ConstPtr& msg)
{
	ofstream myfile;
    string path;
	ros::param::get("parth_param",path);

	if(path.empty())
	{
		myfile.open("~/vectorData.txt", std::ios::app); // This path should be changed to where the file should be placed
		cout << "File saved with feature vectors in Home" << endl;
	}
	else
	{
		myfile.open(path.c_str(), std::ios::app);
		cout << "File saved with feature vectors in " << path.c_str() << endl;
	}

	myfile << ros::Time::now() << "\n";

	for (int i=0; i<msg->vectors.size(); ++i)
	{
		//cout << msg->vectors.size() << endl;
		const geometry_msgs::Vector3 &data = msg->vectors[i];
		myfile << data.x << " " << data.y << " " << data.z << "\n";
	}

	myfile << "---" << "\n";
	myfile.close();
	
	ros::shutdown();
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "save_vects_node");
	ros::NodeHandle nh_;
	ros::param::set("parth_param",(std::string)argv[1]);
	ros::Subscriber subvec = nh_.subscribe("/feature_vectors", 1000, subCBvec);
	
	ros::spin();
}