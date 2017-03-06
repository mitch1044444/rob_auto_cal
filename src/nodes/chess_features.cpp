#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
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
double square_size = 0.025; //0.0145;	// Chessboard printed square size in meters

void draw_coordinate_system(Mat img, vector<Point2f>& corners, vector<Point2f>& imgpts)
{

	line(img, Point((int)corners[0].x,(int)corners[0].y), Point((int)imgpts[0].x,(int)imgpts[0].y), Scalar(0,0,255), 6);
	line(img, Point((int)corners[0].x,(int)corners[0].y), Point((int)imgpts[1].x,(int)imgpts[1].y), Scalar(0,255,0), 6);
	line(img, Point((int)corners[0].x,(int)corners[0].y), Point((int)imgpts[2].x,(int)imgpts[2].y), Scalar(255,0,0), 6);
}



class findChessboardTrans
{
	geometry_msgs::TransformStamped transformStamped_;
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::CameraSubscriber camera_sub_;
	ros::ServiceServer service_;
	//tf2_ros::TransformBroadcaster tf_br_;
	//ros::Publisher vp_ ;

	  
public:
  	findChessboardTrans()
    	: it_(nh_)
  	{

		// Subscribe to input video feed 

		camera_sub_ = it_.subscribeCamera("/camera/image_raw", 1, &findChessboardTrans::imageCb, this);
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
	    

		// Find chessboard inner corners
	    Size patternsize(h,w);
	    vector<Point2f> corners;
	    bool patternfound = findChessboardCorners(gray_image, patternsize, corners,
	          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

	    if(patternfound)
	    {
	    	// Draw tracking points of chessboard

		    cornerSubPix(gray_image, corners, Size(5, 5), Size(-1, -1),TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.05));
		    //drawChessboardCorners(cv_ptr->image, patternsize, Mat(corners), patternfound);

		    
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

		    // double camMatArray[9] = {688.302912, 0.000000, 319.798533,
			//       				0.000000, 690.195379, 246.386891,
		    //                         0.000000, 0.000000, 1.000000}; // Obtained for camera calibration (gubby)
		    //double distMatArray[5] = {-0.180610, 0.130998, 0.000918, 0.002666, 0.000000};
			
		    //double camMatArray[9] = {1612.839416, 0.000000, 769.049297,
			//		       			0.000000, 1598.467770, 538.047934,
		    //                        0.000000, 0.000000, 1.000000}; // Obtained for camera calibration (microsoft 1080p usb webcam)
		    //double distMatArray[5] = {0.093800, -0.039139, 0.011473, -0.031648, 0.000000};
			
		    



			// Create 3d point model for chessboard
			vector<Point3d> obj3d;
		    for (int i = 0; i < patternsize.height; i++)
		    {
		        for (int j = 0; j < patternsize.width; j++)
		        {
		            obj3d.push_back(Point3d(square_size*i,square_size*j,0.0));
		        }
		    }

			Mat cameraMatrix = Mat(3, 3, CV_64F, &camMatArray);
			Mat distCoeffs = Mat(5, 1, CV_64F, &distMatArray);
			Mat rvec, tvec;


			vector<Point2d> cornersd;
			for (size_t i=0 ; i<corners.size(); i++)
			{
				cornersd.push_back( cv::Point2d((double)corners[i].x, (double)corners[i].y));
			}

			solvePnP(obj3d,cornersd,cameraMatrix,distCoeffs,rvec,tvec);

			vector<Point3f> axis3d;
	    	vector<Point2f> imgpnts;
		    axis3d.push_back(Point3f(0.06,0,0)); // x
		   	axis3d.push_back(Point3f(0,0.06,0)); // y
		   	axis3d.push_back(Point3f(0,0, 0.06));// z

			projectPoints(axis3d, rvec, tvec, cameraMatrix, distCoeffs, imgpnts);

			draw_coordinate_system(cv_ptr->image, corners, imgpnts);


			Mat R = Mat::zeros(3,3,CV_64F);

			Rodrigues(rvec,R);

			cout << "Rotation Matrix:" << endl << R << endl << "Translation Vector:" << endl << tvec << endl;	    

			// broadcase translation to tf2 
			//geometry_msgs::TransformStamped transformStamped;
			transformStamped_.header.stamp = ros::Time::now();
			transformStamped_.header.frame_id = "chessboard";
			transformStamped_.child_frame_id = "camera";
			transformStamped_.transform.translation.x = tvec.at<double>(0);
			transformStamped_.transform.translation.y = tvec.at<double>(1);
			transformStamped_.transform.translation.z = tvec.at<double>(2);
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

			//tf_br_.sendTransform(transformStamped);


			// Publish image features in the from of vectors.
			// rob_auto_cal::vectorVector3 vectorArrayMsg;

			// Mat invcameraMatrix = cameraMatrix.inv();

			// geometry_msgs::Vector3 vect;

			// for (int i = 0; i < corners.size(); ++i)
			// {
			// 	double imPoint2d[3] = {cornersd[i].x, cornersd[i].y, 1.0};
			// 	Mat hom_pt = Mat(3, 1, CV_64F, &imPoint2d);

			// 	hom_pt = invcameraMatrix*hom_pt; //put in world coordinates by removing distortion
				
			// 	Point3d direction(hom_pt.at<double>(0),hom_pt.at<double>(1),hom_pt.at<double>(2));
				
			// 	//To get a unit vector, direction just needs to be normalized
			// 	direction *= 1/norm(direction);

			// 	vect.x = direction.x;
			// 	vect.y = direction.y;
			// 	vect.z = direction.z;
				
			// 	vectorArrayMsg.vectors.push_back(vect);
			// }
			
			// vp_.publish(vectorArrayMsg);

	    }

	    // Update GUI Window
	    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	    cv::waitKey(3);
	}
};

int main(int argc, char** argv)
{
	cout << "Initiated" << endl;
	ros::init(argc, argv, "chess_features");
	findChessboardTrans fct;


	ros::spin();
	return 0;
}

