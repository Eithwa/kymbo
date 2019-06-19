#include <cstdio>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <highgui.h>
#include <math.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
//#include <sensor_msgs/CompressedImage.h>

using namespace cv;
using namespace std;

class Vision
{
public:
	Vision();
	~Vision();
	cv::Mat get_image();
	void show_image();	

private:
	ros::NodeHandle nh;
	ros::Subscriber image_sub;
	//void imageCb(const sensor_msgs::CompressedImageConstPtr& msg);
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
	cv::Mat main_image;

};

/*
class Object: public Vision
{
public:
	Object();
	cv::Mat converter();
	void show_mask();

private:
	cv::Mat iframe;
	cv::Mat mask;
};
*/

