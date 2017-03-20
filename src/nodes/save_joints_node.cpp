#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <stdio.h>
#include <ncurses.h>

using namespace std;

double joint_state[6];

void StateCallBack(const sensor_msgs::JointState::ConstPtr &msg)
{	
	
	for (int i = 0; i < 6; ++i)
	{
		joint_state[i] = msg->position[i];
	}
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
  
	// Handle Ncurses
	initscr();		/* Initiate Ncurses  */
  	printw("Press the Enter key to log joint values press s to save and Esc to exit.\n");		/* Print message to screen */
	printw("Joint values logged: \n");		
	printw("Number of positions logged: ");
	curs_set(0);		/* Hide cursor */
	noecho();		/* Hide user input */ 
	start_color();
	init_pair(1, COLOR_RED, COLOR_BLACK);
	refresh();		/* Refresh screen */
			

	std::ofstream myfile;
	std::string path;
	ros::param::get("parth_param_joints",path);
	std::stringstream ss;
	int joint_pos_count = 0;
	myfile.open(path.c_str(), std::ios::app);
	bool save_flag = false;
	bool exit_flag = false;
  	while(ros::ok())
  	{
  		ros::spinOnce();
  		timeout(100);
  		int c = getch();
  		//printw("c: %d \n", c);
		if(c == 10)
		{
			
			//std::cout << "File saved with joint data in " << path.c_str() << std::endl;



			for (int i = 0; i < 6; ++i)
			{
				ss << joint_state[i] << " ";
			}
			ss << "\n";
			move(1, 21);
			clrtoeol();
			mvprintw(1,21,"%f %f %f %f %f %f ", joint_state[0], joint_state[1], joint_state[2], joint_state[3], joint_state[4], joint_state[5]);
			joint_pos_count++;
			//move(2, 0);
			//clrtoeol();
			mvprintw(2,28, "%d", joint_pos_count);


			refresh();
			c = 0;
			save_flag = false;

		}
	  	
	  	if (c == 115)
	  	{

	  		if (!save_flag)
	  		{
	  			myfile << joint_pos_count << "\n" << ss.str();

	  			mvprintw(3,0,"%d values gathered saved in file: ", joint_pos_count);
	  			mvprintw(3,30,path.c_str());
	  			refresh();
	  			save_flag = true;
	  		}

	  	}
	  	if (c == 27)
	  	{
	  		if(!save_flag)
	  		{
	  			attron(COLOR_PAIR(1));
	  			mvprintw(4,0,"Are you sure you want to exit         saving? y/n");
	  			attron(A_UNDERLINE);
	  			mvprintw(4,30,"without");
	  			attroff(A_UNDERLINE);
	  			attroff(COLOR_PAIR(1));
		  		refresh();
		  		while(1)
		  		{
		  			timeout(100);
	  				int c = getch();
	  				if (c == 121)
	  				{
	  					exit_flag = true;
	  					break;
	  				}
	  				if (c == 110)
	  				{
	  					move(4,0);
	  					clrtoeol();
	  					refresh();
	  					break;
	  				}
		  		}
	  		}
	  		else
	  		{
	  			exit_flag = true;
	  		}
	  
	  	}
	  	if (exit_flag)
	  	{	
	  		mvprintw(4,0,"Program is closing down.                                  ");
	  		refresh();
	  		ros::Duration(4).sleep();
	  		break;
	  	}
		
  	}
  	ros::spinOnce();
  	myfile.close();
	endwin();			/* End curses mode		  */
	ros::shutdown();
  	return 0;
}