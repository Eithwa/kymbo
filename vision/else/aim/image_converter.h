#include <cstdio>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <highgui.h>
#include <math.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "vision/bin.h"
#include "vision/aim.h"
#include <std_msgs/Int32.h>
#define VISION_TOPIC "/usb_cam/image_raw"
//#include <sensor_msgs/CompressedImage.h>

using namespace cv;
using namespace std;

class NodeHandle
{
public:
	NodeHandle();

	bool Get_Param;
	vector<int> Sent_HSV(int index);
	vector<vector<int> > Object_HSV;
	vector<int> Setting;
	//void Set_Mask(const std_msgs::Int32 msg);
private:
	ros::NodeHandle nh;
	ros::Subscriber param_sub;
	ros::Subscriber setting_sub;
	void Set_Param(const std_msgs::Int32 msg);
	//void Set_Setting(const std_msgs::Int32 msg);
		
	
};
class Object: NodeHandle
{
public:
	Object();
	//Object(Mat frame, int index);
	void Reset();
	string name;
	double angle;
	Mat mask;
	int maskindex;
	int offset;
	cv::Point2f _center;
	Point distance_point;
	float distance;
	float area;

	Mat ColorModel(Mat frame);
	Mat Monitor(Mat Frame);
	void Set_Mask(const std_msgs::Int32 msg);
	void Get_HSV(vector<int> HSV);
private:	
	vector<int> HSVrange;
	vector<vector<Point> > contours;
	int largest_contour_index;
	int extLeft,extRight,extTop,extBot;

	
	void Findoffset(int center);
	void FindAngle(Mat frame);
	void FindDistance(Mat frame);

	void FindContours(Mat frame, Mat mask);
	Point FindCenter();
};



class Vision
{
public:
	Vision();
	Vision(string sub);
	~Vision();
	cv::Mat get_image();
	ros::Publisher aim_pub;
	void Pub(Mat frame,Object red,Object blue,Object yellow, Object white, Object black);
	void Pubvision();
	void Pubmask();
	bool update;
	Mat Monitor;
	vector<Mat> Mask;
	int maskindex;
private:
	ros::NodeHandle nh;
	ros::Subscriber image_sub;
	ros::Subscriber mode_sub;

	ros::Publisher mask_pub;
	ros::Publisher monitor_pub;
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
	void MaskIndex(const std_msgs::Int32 msg);
	cv::Mat main_image;
};


