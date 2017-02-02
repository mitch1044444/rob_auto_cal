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



/*
 * Takes images in image_raw and finds the chessboard coners of a 9x6 opencv chessboard of 25mm square size
 * It outputs the translation and vectors to image points in tf2 
 */

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace std;



class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	static tf2_ros::TransformBroadcaster tf_br_;
	  
public:
  	ImageConverter()
    	: it_(nh_)
  	{
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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
    
     
    Mat gray_image;
    cvtColor( cv_ptr->image, gray_image, COLOR_BGR2GRAY );
    
    Size patternsize(9,6);

    vector<Point2f> corners;

    vector<Point3d> obj3d;

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 9; j++)
        {
            Point3d temp(0.025*j,0.025*i,0.0);
            obj3d.push_back(temp);
        }
    }

    bool patternfound = findChessboardCorners(gray_image, patternsize, corners,
          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS | CALIB_CB_FAST_CHECK);


    if(patternfound)
    {
      cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1),TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

      drawChessboardCorners(cv_ptr->image, patternsize, Mat(corners), patternfound);


      double camMatArray[9] = {688.302912, 0.000000, 319.798533,
			       			   0.000000, 690.195379, 246.386891,
                               0.000000, 0.000000, 1.000000}; // Obtained for camera calibration

      double distMatArray[5] = {-0.180610, 0.130998, 0.000918, 0.002666, 0.000000};


      Mat cameraMatrix = Mat(3, 3, CV_64F, &camMatArray);
      Mat distCoeffs = Mat(5, 1, CV_64F, &distMatArray);
      Mat rvec, tvec;

      //cout << cameraMatrix << endl;

      vector<Point2d> cornersd;
      
      for (size_t i=0 ; i<corners.size(); i++)
      {
	cornersd.push_back( cv::Point2d( (double)corners[i].x, (double)corners[i].y  ) );
      }
      
      
      solvePnP(obj3d,cornersd,cameraMatrix,distCoeffs,rvec,tvec);

      Mat R = Mat::zeros(3,3,CV_64F);

      Rodrigues(rvec,R);

      cout << "Rotation Matrix:" << endl << R << endl << "Translation Vector:" << endl << tvec << endl;	    
	
      // Output HTM to tf
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = "camera";
      transformStamped.child_frame_id = "world";
      transformStamped.transform.translation.x = tvec.at<double>(0,0,0);
      transformStamped.transform.translation.y = tvec[1];
      transformStamped.transform.translation.z = tvec[2];
      tf2::Quaternion q;
      
      tf2::tf2Scaler theta = (double)(sqrt(rvec[0]*rvec[0] + rvec[1]*rvec[1] + rvec[2]*rvec[2]));
      tf2::Vector3 axis = tf2::Vector3(tvec[0], tvec[1], tvec[2]);
      q.setRotation(axis,theta);
      
      transformStamped.transform.rotation.x = q.x();
      transformStamped.transform.rotation.y = q.y();
      transformStamped.transform.rotation.z = q.z();
      transformStamped.transform.rotation.w = q.w();
      
      
      tf_br_.sendTransform(transformStamped);

      
    }


    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}