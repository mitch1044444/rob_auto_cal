#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <stdio.h>
#include <rob_auto_cal/vectorVector3.h>
#include <rob_auto_cal/reqChessTransform.h>



/*
 * Takes images in image_raw and finds the chessboard coners of a 9x6 opencv chessboard of 25mm square size
 * It outputs the translation and vectors to image points in tf2 
 */

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace std;

int h = 9; 					// Chessboard inner corner dimensions width and height
int w = 6;					
//double square_size = 0.0145; //0.0145;	// Chessboard printed square size in meters

void draw_coordinate_system(Mat img, vector<Point2f>& imgpts, double dist)
{
	int line_thickness = (int)lround((1/(dist*dist)+1));
	if(line_thickness > 255)
		line_thickness = 255;
	if(line_thickness < 1)
		line_thickness = 1;

	line(img, Point((int)imgpts[0].x,(int)imgpts[0].y), Point((int)imgpts[1].x,(int)imgpts[1].y), Scalar(0,0,255), line_thickness); // 
	line(img, Point((int)imgpts[0].x,(int)imgpts[0].y), Point((int)imgpts[2].x,(int)imgpts[2].y), Scalar(0,255,0), line_thickness);
	line(img, Point((int)imgpts[0].x,(int)imgpts[0].y), Point((int)imgpts[3].x,(int)imgpts[3].y), Scalar(255,0,0), line_thickness);
}



class findChessboardTrans
{
	geometry_msgs::TransformStamped transformStamped_;
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::CameraSubscriber camera_sub_;
	ros::ServiceServer service_;
	tf2_ros::TransformBroadcaster tf_br_;

	//ros::Publisher vp_ ;

	  
public:
  	findChessboardTrans()
    	: it_(nh_)
  	{

		// Subscribe to input video feed 
  		string image_topic_url = "camera/image_raw";
		ros::param::get("image_topic_url", image_topic_url);
		camera_sub_ = it_.subscribeCamera(image_topic_url.c_str(), 1, &findChessboardTrans::imageCb, this);
		cv::namedWindow(OPENCV_WINDOW);

		//vp_ = nh_.advertise<rob_auto_cal::vectorVector3>("/feature_vectors", 1000);
		service_ = nh_.advertiseService("reqChessTransform", &findChessboardTrans::serviceCb, this);
		

	}

	~findChessboardTrans()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	bool serviceCb(rob_auto_cal::reqChessTransform::Request &req, rob_auto_cal::reqChessTransform::Response &res)
	{
		res.transformStamped = transformStamped_;
	}



	void imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
	{

		// Copy image to OpenCV format from sensor_msgs
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	    
	    // Convert image to grayscale
	    Mat gray_image;
	    cvtColor( cv_ptr->image, gray_image, COLOR_BGR2GRAY );
	    

	    double camMatArray[9];
		double distMatArray[5] = {0};

		for (int i = 0; i < 9; ++i)
		{
			camMatArray[i] = cam_info->K.elems[i];
		 	//cout << camMatArray[i] << " ";
		}

		for (int i = 0; i < cam_info->D.size(); ++i)
		{
			distMatArray[i] = cam_info->D[i];
			//cout << distMatArray[i] << " ";
	   	}
	   	//cout << endl;

		Mat cameraMatrix = Mat(3, 3, CV_64F, &camMatArray);
		Mat distCoeffs = Mat(5, 1, CV_64F, &distMatArray);


		// Find chessboard inner corners
	    Size patternsize(h,w);
	    vector<Point2f> corners;
	    bool patternfound = findChessboardCorners(gray_image, patternsize, corners,
	    	CV_CALIB_CB_ADAPTIVE_THRESH |  CV_CALIB_CB_FILTER_QUADS);
	    bool rotation_error_flag = 0;
	    bool translation_error_flag = 0;


	    if(patternfound)
	    {
	    	// Draw tracking points of chessboard
	    	
		    cornerSubPix(gray_image, corners, Size(5, 5), Size(-1, -1),TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.05));
		    //drawChessboardCorners(cv_ptr->image, patternsize, Mat(corners), patternfound);

		    



			
		    string square_size_string = "0.02393";
		   	ros::param::get("chess_square_size", square_size_string);
		   	double square_size = atof(square_size_string.c_str());
		   	double factor;
		   	double aspect;

		   	if(patternsize.height > patternsize.width)
		   	{
				factor = 2/(square_size*(patternsize.height-1));
				aspect = (double)(patternsize.height-1)/(double)(patternsize.width-1);
		   	}
			else
			{
				factor = 2/(square_size*(patternsize.width-1));
				aspect = (double)(patternsize.width-1)/(double)(patternsize.height-1);
			}

			//factor = 1;
			//aspect = 0;

			// Create 3d point model for chessboard
			vector<Point3f> obj3d;
		    for (int i = 0; i < patternsize.height; i++)
		    {
		        for (int j = 0; j < patternsize.width; j++)
		        {
		            obj3d.push_back(Point3f(square_size*j*factor-1,square_size*i*factor-1/aspect,0.0));
		        }
		    }
		    
		    //cout << "Obj3d: " << obj3d << endl;

			Mat rvec = Mat(3,1,CV_64F, 0.0);
			Mat rvec2 = Mat(3,1,CV_64F, 0.0);
			Mat tvec = Mat(3,1,CV_64F, 0.0);
			Mat tvec2 = Mat(3,1,CV_64F, 0.0);

			vector<Point2d> cornersd;
			for (size_t i=0 ; i<corners.size(); i++)
			{
				cornersd.push_back( cv::Point2d((double)corners[i].x, (double)corners[i].y));
			}

			solvePnP(obj3d, corners, cameraMatrix, distCoeffs, rvec, tvec, false);
			tvec.copyTo(tvec2);
			rvec.copyTo(rvec2);

			solvePnP(obj3d, corners, cameraMatrix, distCoeffs, rvec2, tvec2, true, CV_EPNP);

			//cout << "tvec: " << tvec << " rvec: " << rvec << endl;
			//cout << "tvec2: " << tvec2 << " rvec2: " << rvec2 << endl;


			//cout << "tvec-tvec2 distance: " << sqrt((tvec.at<double>(0)-tvec2.at<double>(0))*(tvec.at<double>(0)-tvec2.at<double>(0))+
			//	(tvec.at<double>(1)-tvec2.at<double>(1))*(tvec.at<double>(1)-tvec2.at<double>(1))+
			//	(tvec.at<double>(2)-tvec2.at<double>(2))*(tvec.at<double>(2)-tvec2.at<double>(2))) << endl;

			if (!(sqrt((tvec.at<double>(0)-tvec2.at<double>(0))*(tvec.at<double>(0)-tvec2.at<double>(0))+
				(tvec.at<double>(1)-tvec2.at<double>(1))*(tvec.at<double>(1)-tvec2.at<double>(1))+
				(tvec.at<double>(2)-tvec2.at<double>(2))*(tvec.at<double>(2)-tvec2.at<double>(2))) > 0.02))
			{
				tvec2.copyTo(tvec);
			}
			else
			{
				cout << "Error in CV_EPNP translation, using CV_INTERATIVE" << endl;
				translation_error_flag = true;
			}

			//	cout << "rvec-rvec2 distance: " << sqrt((rvec.at<double>(0)-rvec2.at<double>(0))*(rvec.at<double>(0)-rvec2.at<double>(0))+
			//	(rvec.at<double>(1)-rvec2.at<double>(1))*(rvec.at<double>(1)-rvec2.at<double>(1))+
			//	(rvec.at<double>(2)-rvec2.at<double>(2))*(rvec.at<double>(2)-rvec2.at<double>(2))) << endl;

			if (!(sqrt((rvec.at<double>(0)-rvec2.at<double>(0))*(rvec.at<double>(0)-rvec2.at<double>(0))+
				(rvec.at<double>(1)-rvec2.at<double>(1))*(rvec.at<double>(1)-rvec2.at<double>(1))+
				(rvec.at<double>(2)-rvec2.at<double>(2))*(rvec.at<double>(2)-rvec2.at<double>(2))) > 0.02))			
			{
				rvec2.copyTo(rvec);
			}
			else
			{
				cout << "Error in CV_EPNP rotation, using CV_INTERATIVE" << endl;
				rotation_error_flag = true;
			}
			//solvePnPRansac(obj3d,corners,cameraMatrix,distCoeffs,rvec,tvec,true,
			//	100, 8, 100, 
			//	noArray(), CV_EPNP); //



			vector<Point3f> axis3d;
	    	vector<Point2f> imgpnts;
	    	axis3d.push_back(Point3f(0,0,0));// Origin
		    axis3d.push_back(Point3f(square_size*factor*2,0,0)); // x
		   	axis3d.push_back(Point3f(0,square_size*factor*2,0)); // y
		   	axis3d.push_back(Point3f(0,0, -square_size*factor*2));// z

			projectPoints(axis3d, rvec, tvec, cameraMatrix, distCoeffs, imgpnts);

			double distance = sqrt(tvec.at<double>(0)*tvec.at<double>(0) + tvec.at<double>(1)*tvec.at<double>(1) + tvec.at<double>(2)*tvec.at<double>(2))/factor;
			draw_coordinate_system(cv_ptr->image, imgpnts, distance);
			cout << "Distance: " << distance << endl;

			Mat R = Mat::zeros(3,3,CV_64F);

			Rodrigues(rvec,R);

			//cout << R << endl << tvec << endl;	    

			// broadcase translation to tf2 
			//geometry_msgs::TransformStamped transformStamped;



			transformStamped_.header.stamp = ros::Time::now();
			transformStamped_.header.frame_id = "chessboard";
			transformStamped_.child_frame_id = "camera";


			transformStamped_.transform.translation.x = tvec.at<double>(0)/factor;
			transformStamped_.transform.translation.y = tvec.at<double>(1)/factor;
			transformStamped_.transform.translation.z = tvec.at<double>(2)/factor;
			tf2::Quaternion q;
			double theta = (double)(sqrt(rvec.at<double>(0)*rvec.at<double>(0) + 
										 rvec.at<double>(1)*rvec.at<double>(1) + 
										 rvec.at<double>(2)*rvec.at<double>(2)));
			tf2::Vector3 axis = tf2::Vector3(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
			q.setRotation(axis,theta);

			transformStamped_.transform.rotation.x = q.x();
			transformStamped_.transform.rotation.y = q.y();
			transformStamped_.transform.rotation.z = q.z();
			transformStamped_.transform.rotation.w = q.w();
			

			tf_br_.sendTransform(transformStamped_);



	    }
 
		Mat undistorted;
		cv::undistort(cv_ptr->image, undistorted, cameraMatrix, distCoeffs);
	    // Update GUI Window
	    
	    float aspect = ((float)(cv_ptr->image.rows))/((float)(cv_ptr->image.cols));
	    Size newsize(1000,1000*aspect);
	    resize(undistorted,undistorted, newsize);

	    if(rotation_error_flag)
	    	putText(undistorted, "CV_EPNP rot error", Point(5,40), 1, 1, Scalar(0,0,255), 2);
	    if(translation_error_flag)
	    	putText(undistorted, "CV_EPNP trans error", Point(5,60), 1, 1, Scalar(0,0,255), 2);

	    cv::imshow(OPENCV_WINDOW, undistorted);
	    cv::waitKey(1);
	}
};

int main(int argc, char** argv)
{
	cout << "Initiated" << endl;
	ros::init(argc, argv, "chess_trans_finder");
	if(argc<2)
	{
		cout << "\033[1;31mWarning! Input argument needed for chessboard square size! default of 0.02393 set!\033[0m" << endl;
		ros::param::set("chess_square_size","0.02393");
	}	
	else
	{
		ros::param::set("chess_square_size",(std::string)argv[1]);
	}
	
	if(argc!=3)
	{
		cout << "\033[1;31mWarning! Input argument needed for image topic default of /camera/image_raw set\033[0m" << endl;
		ros::param::set("image_topic_url","camera/image_raw");
	}	
	else
	{
		ros::param::set("image_topic_url",(std::string)argv[2]);
	}


	findChessboardTrans fct;
	


	ros::spin();
	return 0;
}

