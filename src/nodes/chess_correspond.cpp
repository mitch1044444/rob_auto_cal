#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace std;


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
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

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    // cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    
    
    
    
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
            //obj3d[i+j].x = 0.025*j;
            //obj3d[i+j].y = 0.025*i;
            //obj3d[i+j].z = 0;
        }
    }

    //cout << obj3d << endl;

    bool patternfound = findChessboardCorners(gray_image, patternsize, corners,
          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    //cout << patternfound << endl;

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